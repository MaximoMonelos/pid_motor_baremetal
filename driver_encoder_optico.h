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
