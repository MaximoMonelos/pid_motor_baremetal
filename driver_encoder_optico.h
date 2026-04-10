/**
 * @file driver_encoder_optico.h
 * @brief Driver para encoder óptico de ranura con filtrado FIR.
 * @author [Autor]
 * @date [Fecha]
 * 
 * @details
 * Este driver maneja un encoder óptico de tipo ranura (slotted optical sensor)
 * conectado a un GPIO del Raspberry Pi Pico. Proporciona medición de velocidad
 * en RPM con opciones de filtrado mediante un filtro FIR (Finite Impulse Response).
 * 
 * ## Concepto de operación
 * 
 * El encoder óptico de ranura consiste en un emisor de luz infrarroja y un
 * receptor placedos uno frente al otro. Cuando un objeto (como un disco con
 * ranuras) interrumpe el haz de luz, el receptor genera un pulso eléctrico.
 * Cada pulso representa una "ventana" del disco encoder.
 * 
 * ## Conversión a RPM
 * 
 * La velocidad angular se calcula a partir de la frecuencia de pulsos:
 * 
 *     Frecuencia (Hz) = pulsos / segundo
 *     RPM = (frecuencia / ticks_por_revolución) * 60
 * 
 * Por ejemplo, con un disco de 10 ranuras: si contamos 300 pulsos en 1 segundo,
 * eso equivale a 300/10 = 30 revoluciones/segundo = 1800 RPM.
 * 
 * ## Filtrado FIR
 * 
 * La señal de RPM puede presentar oscilaciones, especialmente a bajas velocidades.
 * Para suavizar la señal, se aplica un filtro FIR que promedia la medición actual
 * con valores anteriores ponderados por coeficientes.
 * 
 * @note Se requiere incluir las librerías CMSIS-DSP del SDK de Raspberry Pi Pico.
 * 
 * @see encoder_init(), encoder_get_freq(), encoder_get_rpm_raw(), encoder_get_rpm_filtered()
 */

#ifndef DRIVER_ENCODER_OPTICO_H
#define DRIVER_ENCODER_OPTICO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "pico/stdlib.h"
#include "dsp/filtering_functions.h"
#include "arm_math.h"

/**
 * @struct encoder_config_t
 * @brief Parámetros de configuración del encoder.
 * @details
 * Configure estos valores antes de llamar a encoder_init(). Esta estructura
 * contiene los parámetros necesarios para inicializar el hardware del encoder.
 * 
 * | Campo      | Tipo          | Descripción                                                    |
 * |------------|---------------|----------------------------------------------------------------|
 * | pin        | uint8_t       | Número de GPIO del Raspberry Pi Pico. Valores válidos: 23-25, 27-29. Estos pines no tienen funciones especiales en el Pico que puedan causar conflictos (como SPI o I2C). |
 * | ticks      | float         | Número de pulsos o ranuras por revolución del disco encoder. Este valor depende del disco físico utilizado. Un disco común tiene 10 ranuras. |
 * | coef       | const float*  | Puntero al array de coeficientes del filtro FIR. Estos coeficientes determinan cómo se suaviza la señal.Pueden ser generados con herramientas como MATLAB o Python scipy. |
 * | num_taps   | uint16_t      | Cantidad de coeficientes del filtro FIR, también conocida como la "longitud" o "orden" del filtro. Más taps = mayor suavizado pero mayor latencia y uso de memoria. |
 * 
 * @warning Usar pines fuera del rango 23-25 o 27-29 puede causar comportamiento
 *          indefinido o conflictos con otras funciones del hardware.
 * 
 * @note El filtro FIR requiere (num_taps + BLOCK_SIZE) elementos en fir_state.
 *       Para num_taps=32 y BLOCK_SIZE=1, se necesitan 33 floats.
 * 
 * @example
 * @code
 * // Configuración típica con disco de 10 ranuras
 * static float fir_coef[32] = { 0 }; // coeficientes del filtro
 * static float fir_state[33];
 * 
 * encoder_config_t cfg = {
 *     .pin = 10,
 *     .ticks = 10.0f,
 *     .coef = fir_coef,
 *     .num_taps = 32
 * };
 * @endcode
 */
typedef struct {
    /** @brief Número de pin GPIO (23-25, 27-29) */
    uint8_t pin;
    /** @brief Pulsos por revolución del disco encoder */
    float ticks;
    /** @brief Puntero a coeficientes del filtro FIR */
    const float* coef;
    /** @brief Número de coeficientes del filtro FIR */
    uint16_t num_taps;
} encoder_config_t;


/**
 * @struct encoder_internal_t
 * @brief Estado interno del encoder (uso privado del driver).
 * @details
 * Esta estructura contiene las variables necesarias para el funcionamiento
 * interno del encoder. NO modificar manualmente salvo casos muy específicos.
 * 
 * | Campo            | Tipo                    | Descripción                                                                 |
 * |------------------|-------------------------|-----------------------------------------------------------------------------|
 * | counter_pulses   | volatile uint32_t       | Contador atómico de pulsos detectados. Se incrementa en la ISR (Interrupt Service Routine) cada vez que se detecta un flanco ascendente. El calificador volatile es necesario porque esta variable es modificada por una interrupción. |
 * | last_time_us     | uint32_t                | Timestamp del último cálculo de frecuencia, medido en microsegundos (μs). Se usa como referencia temporal para calcular el período entre muestreos. |
 * | last_pulses      | uint32_t                | Conteo de pulsos registrado en el último cálculo. Se usa para calcular el delta de pulsos entre dos muestreos. |
 * | fir              | arm_fir_instance_f32    | Instancia del filtro FIR de ARM CMSIS-DSP. Contiene punteros internos a los coeficientes, estado y configuración del filtro. |
 * | fir_state        | float*                  | Puntero al buffer de estado del filtro FIR. Este buffer actúa como "memoria" del filtro, almacenando las muestras anteriores necesarias para el cálculo. |
 * 
 * @warning No modificar estos campos manualmente durante la operación normal.
 *          El acceso concurrente desde la ISR y el código principal puede causar
 *          condiciones de carrera (race conditions).
 */
typedef struct {
    /** @brief Contador atómico de pulsos (incrementado por ISR) */
    volatile uint32_t counter_pulses;
    /** @brief Tiempo del último cálculo en microsegundos */
    uint32_t last_time_us;
    /** @brief Pulsos en el último cálculo */
    uint32_t last_pulses;
    /** @brief Instancia del filtro FIR de ARM CMSIS-DSP */
    arm_fir_instance_f32 fir;
    /** @brief Buffer de estado del filtro FIR (memoria de trabajo) */
    float *fir_state; 
} encoder_internal_t;


/**
 * @struct encoder_t
 * @brief Contexto principal del encoder.
 * @details
 * Esta es la estructura principal que mantiene toda la información del encoder.
 * Debe ser declarada en el código del usuario y pasada a las funciones del driver.
 * 
 * Esta estructura se divide en tres secciones:
 * 1. **config**: Parámetros de configuración (establecer antes de encoder_init)
 * 2. **internal**: Estado interno del driver (solo lectura)
 * 3. **mediciones**: Resultados calculados (actualizados por las funciones get)
 * 
 * | Campo          | Tipo                  | Descripción                                                              |
 * |----------------|-----------------------|--------------------------------------------------------------------------|
 * | config         | encoder_config_t      | Parámetros de configuración. Establecer ANTES de llamar encoder_init().   |
 * | internal       | encoder_internal_t    | Estado interno del driver. NO modificar manualmente.                      |
 * | freq           | float                 | Frecuencia de giro actual calculada en Hertz (Hz = pulsos/segundo).      |
 * | rpm_raw        | float                 | Velocidad en RPM sin filtrar. Puede presentar ruido y oscilaciones.       |
 * | rpm_filtered   | float                 | Velocidad en RPM después de aplicar el filtro FIR. Señal suavizada.       |
 * 
 * @example
 * @code
 * encoder_t my_encoder;
 * 
 * void main() {
 *     // Configurar antes de inicializar
 *     encoder_config_t cfg = { .pin = 10, .ticks = 10.0f, ... };
 *     
 *     encoder_init(&my_encoder, &cfg, fir_state, encoder_isr);
 *     
 *     while(1) {
 *         encoder_get_freq(&my_encoder);
 *         encoder_get_rpm_filtered(&my_encoder);
 *         printf("RPM: %.2f\n", my_encoder.rpm_filtered);
 *         sleep_ms(10);
 *     }
 * }
 * @endcode
 */
typedef struct {
    /** @brief Configuración del encoder */
    encoder_config_t config;
    /** @brief Estado interno (uso privado del driver) */
    encoder_internal_t internal;
    /** @brief Frecuencia calculada en Hz */
    float freq;
    /** @brief RPM sin filtrar (señal cruda) */
    float rpm_raw;
    /** @brief RPM filtrada con FIR (señal suavizada) */
    float rpm_filtered;
} encoder_t;

/**
 * @brief Obtiene las RPM del encoder con filtrado FIR aplicado.
 * @param[out] enc Puntero al encoder. Actualiza enc->rpm_filtered.
 * @pre encoder_get_rpm_raw() debe haber sido llamada previamente.
 * @see encoder_get_rpm_raw(), encoder_init()
 */
void encoder_get_rpm_filtered(encoder_t *enc);

/**
 * @brief Obtiene las RPM del encoder sin filtrar.
 * @param[out] enc Puntero al encoder. Actualiza enc->rpm_raw.
 * @pre encoder_get_freq() debe haber sido llamada previamente.
 * @see encoder_get_freq(), encoder_init()
 */
void encoder_get_rpm_raw(encoder_t *enc);

/**
 * @brief Calcula la frecuencia de giro actual en Hz.
 * @param[out] enc Puntero al encoder. Actualiza enc->freq.
 * @post enc->freq contiene la frecuencia en Hz (pulsos por segundo).
 * @see encoder_init()
 */
void encoder_get_freq(encoder_t *enc);

/**
 * @brief Inicializa el encoder óptico y configura el GPIO.
 * @param[out] enc Puntero a la estructura del encoder (debe existir)
 * @param[in] conf_enc Puntero a la estructura de configuración
 * @param[in] fir_state Buffer de estado del filtro FIR (tamaño: num_taps + 1 floats)
 * @param[in] isr Puntero a la función ISR que maneja la interrupción del GPIO
 * @pre El array fir_state debe estar reservado con anticipación.
 * @warning Solo pines GPIO 23-25 y 27-29 son válidos.
 * @see encoder_config_t
 */
void encoder_init(encoder_t *enc, encoder_config_t *conf_enc, float *fir_state, void *isr);

#ifdef __cplusplus
}
#endif

#endif // DRIVER_ENCODER_OPTICO_H
