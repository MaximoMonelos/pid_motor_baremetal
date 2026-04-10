/**
 * @file pid.c
 * @brief Implementación del controlador PID discreto.
 * @author [Autor]
 * @date [Fecha]
 * 
 * @details
 * Este archivo contiene la implementación de las funciones del controlador PID.
 * El PID calcula una señal de control basada en el error entre un setpoint
 * deseado y la variable del proceso medida.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "pid.h"
// #include "motor.h"
#include "dsp/filtering_functions.h"
#include "arm_math.h"
#include "driver_encoder_optico.h"

/**
 * @brief Constante de zona muerta (deadband).
 * 
 * Define cuántos ciclos consecutivos debe mantenerse el error dentro
 * de la zona muerta antes de congelar la salida del PID.
 * 
 * Valor actual: 10 ciclos
 * Zona muerta: ±2.0 RPM
 * 
 * ## Cómo funciona
 * 
 * Cuando |error| < 2.0 RPM, se incrementa el contador. Si el error
 * permanece dentro de la zona muerta por más de 10 ciclos consecutivos,
 * la salida del PID se "congela" al último valor calculado.
 * 
 * Esto previene el "hunting" (oscilación sostenida) cuando el sistema
 * está muy cerca del setpoint pero no exactamente en él.
 * 
 * ## Cálculo del tiempo real
 * 
 * El tiempo real en la zona muerta depende de la frecuencia de llamada:
 * - A 100 Hz (cada 10 ms): 10 ciclos = 100 ms
 * - A 1 kHz (cada 1 ms): 10 ciclos = 10 ms
 * 
 * @note Ajustar según la frecuencia a la que se llame a pid_set_rpm().
 *       Más ciclos = más tiempo de estabilización antes de congelar.
 *       Menos ciclos = respuesta más rápida pero más riesgo de oscilación.
 */
#define DEADBAND        10

/**
 * @brief Calcula la salida del PID basada en el error actual.
 * 
 * Implementa la ecuación PID discreta:
 * \f[
 * u(k) = K_p \cdot e(k) + K_i \cdot \sum e(i) \cdot \Delta t + K_d \cdot \frac{e(k) - e(k-1)}{\Delta t}
 * \f]
 * 
 * ## Pasos del cálculo
 * 
 * 1. **Término Proporcional (P)**: \f$P = K_p \cdot e(k)\f$
 *    - Respuesta inmediata al error actual
 * 
 * 2. **Término Integral (I)**: \f$I = K_i \cdot \sum e(i) \cdot \Delta t\f$
 *    - Se acumula en pid->integral
 *    - Elimina el error de estado estable (offset)
 * 
 * 3. **Término Derivativo (D)**: \f$D = K_d \cdot \frac{e(k) - e(k-1)}{\Delta t}\f$
 *    - Usa pid->sampling_time como \f$\Delta t\f$
 *    - Amortigua la respuesta (reduce sobreimpulso)
 * 
 * 4. **Saturación (Clamping)**: La salida se limita a [out_min, out_max]
 *    - Previene que la señal de control exceda los límites físicos
 *    - Solo afecta la salida retornada, NO el acumulador integral
 * 
 * @param[in] pid   Puntero al contexto del PID (contiene ganancias y estado)
 * @param[in] error Error actual: setpoint - variable_del_proceso (rpm_value)
 * 
 * @return Salida del PID limitada entre out_min y out_max
 * 
 * @post 
 * - pid->integral se actualiza (suma el error actual)
 * - pid->prev_error se actualiza (almacena error para el próximo ciclo)
 * 
 * @note Esta función debe llamarse periódicamente a la frecuencia
 *       definida en pid->sampling_time para resultados correctos.
 * 
 * @warning El término integral puede causar "integral windup" si:
 *          - El sistema permanece en saturación (no puede alcanzar setpoint)
 *          - El error es siempre del mismo signo por mucho tiempo
 *          - Para mitigar, usar pid_reset() antes de cambios grandes de setpoint
 * 
 * @see pid_set_rpm(), pid_reset()
 */
float pid_calculate(pid_ctrl_t *pid, float error){

    float p = pid->kp * error;

    pid->integral += error;
    
    float i = pid->integral * pid->ki;


    float d = pid->kd * ((error - pid->prev_error) / pid->sampling_time);
    // printf("P: %.2f, D: %.2f, I: %.2f, sampling: %.2f\n", p, d, i, pid->sampling_time);
    pid->prev_error = error;

    float output = p + i + d;
    // printf("OUTPUT: %.2f\n", output);
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;
    return output;
}

/**
 * @brief Wrapper del PID con zona muerta (deadband) para control de RPM.
 * 
 * Esta función combina el cálculo de error y la llamada a pid_calculate(),
 * añadiendo una zona muerta (deadband) para evitar oscilaciones.
 * 
 * ## Algoritmo
 * 
 * ```
 * 1. Calcular error = set_point - rpm_value
 * 
 * 2. Verificar zona muerta:
 *    └─ Si |error| < 2.0:
 *          ├─ Incrementar deadband_counter
 *          └─ Si counter > 10: retornar last_output (congelado)
 *    └─ Si |error| >= 2.0:
 *          ├─ Resetear deadband_counter a 0
 *          └─ Continuar al paso 3
 * 
 * 3. Calcular PID: last_output = pid_calculate(pid, error)
 * 
 * 4. Retornar last_output
 * ```
 * 
 * ## Por qué una zona muerta?
 * 
 * Cuando el sistema está muy cerca del setpoint (dentro de ±2 RPM),
 * las pequeñas variaciones en la medición pueden causar que la salida
 * del PID oscile constantemente, creando un ciclo de "sobrecorrección"
 * conocido como "hunting".
 * 
 * La zona muerta "congela" la salida cuando el error es pequeño y
 * persistente, permitiendo que el sistema se estabilice.
 * 
 * @param[in] rpm_value  Valor actual de RPM medido (variable del proceso)
 * @param[in] set_point  Valor deseado de RPM (setpoint)
 * @param[in] pid        Puntero al contexto del PID
 * 
 * @return 
 * - Si |error| < 2.0 por ≤10 ciclos: Salida calculada normalmente
 * - Si |error| < 2.0 por >10 ciclos: last_output (salida congelada)
 * 
 * @post 
 * - pid->current_error = set_point - rpm_value
 * - pid->deadband_counter se incrementa o resetea
 * - pid->last_output se actualiza (excepto cuando está congelado)
 * 
 * @note El umbral de 2.0 RPM y 10 ciclos son valores típicos.
 *       Ajustar según la precisión requerida y la frecuencia de llamada.
 * 
 * @example
 * @code
 * // En el loop principal (ej. cada 10ms)
 * encoder_get_rpm_filtered(&encoder);
 * float pwm = pid_set_rpm(encoder.rpm_filtered, target_rpm, &pid);
 * motor_set_pwm(pwm);
 * @endcode
 * 
 * @see pid_calculate(), pid_reset()
 */
float pid_set_rpm(float rpm_value, float set_point, pid_ctrl_t *pid){
    pid->current_error = set_point - rpm_value;

    if(pid->current_error < 2.0f && pid->current_error > -2.0f){
        pid->deadband_counter++;
        if (pid->deadband_counter > DEADBAND){
            return pid->last_output;
        }
    } else{
        pid->deadband_counter = 0;
    }
    
    pid->last_output = pid_calculate(pid, pid->current_error);
    return pid->last_output;
    
}

/**
 * @brief Reinicia los términos históricos del PID a cero.
 * 
 * Esta función limpia las "variables de estado" del PID, restaurándolas
 * a sus valores iniciales. Es útil en las siguientes situaciones:
 * 
 * ## Casos de uso
 * 
 * | Situación | Por qué reiniciar |
 * |-----------|-------------------|
 * | Cambio drástico de setpoint | Evita que el acumulador integral cause sobreimpulso |
 * | Recuperación de error | Reinicia el control desde cero |
 * | Cambio de modo de operación | Prepara el PID para nueva referencia |
 * | Arranque del sistema | Asegura estado inicial consistente |
 * 
 * ## Qué se reinicia
 * 
 * - **integral** = 0.0f: Se reinicia el acumulador de la acción integral.
 *   Esto previene que errores pasados afecten la respuesta inmediata.
 * 
 * - **prev_error** = 0.0f: Se limpia el error del ciclo anterior.
 *   La primera acción derivativa será 0 (no hay "cambio" de error).
 * 
 * - **deadband_counter** = 0: Se resetea el contador de zona muerta.
 *   El PID no estará en estado "congelado".
 * 
 * ## Qué NO se reinicia
 * 
 * - Las ganancias (kp, ki, kd)
 * - Los límites de salida (out_min, out_max)
 * - El tiempo de muestreo (sampling_time)
 * 
 * @param[in] pid Puntero al contexto del PID (acepta NULL para seguridad)
 * 
 * @post Si pid != NULL:
 *       - pid->integral = 0.0f
 *       - pid->prev_error = 0.0f
 *       - pid->deadband_counter = 0
 * 
 * @note La verificación de NULL permite llamar pid_reset(pid) sin担心
 *       si el puntero podría ser NULL (aunque en la práctica no debería).
 * 
 * @example
 * @code
 * // Antes de cambiar el setpoint de 500 a 2000 RPM:
 * pid_reset(&motor_pid);  // Limpia el historial
 * setpoint = 2000.0f;     // Nuevo objetivo
 * @endcode
 * 
 * @see pid_calculate(), pid_set_rpm()
 */
void pid_reset(pid_ctrl_t *pid){
    if(pid != NULL){
        pid->integral = 0.0f;
        pid->prev_error = 0.0f;
        pid->deadband_counter = 0.0f;
    }
}