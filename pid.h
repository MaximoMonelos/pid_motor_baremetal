/**
 * @file pid.h
 * @brief Implementación de controlador PID discreto.
 * @author [Autor]
 * @date [Fecha]
 * 
 * @details
 * Este módulo implementa un controlador PID (Proporcional-Integral-Derivativo)
 * en tiempo discreto, optimizado para control de motores DC mediante señales PWM.
 * 
 * ## Ecuación del PID discreto
 * 
 * El PID discreto calcula la señal de control \f$u(k)\f$ en cada ciclo:
 * \f[
 * u(k) = K_p \cdot e(k) + K_i \cdot \sum_{i=0}^{k} e(i) \cdot \Delta t + K_d \cdot \frac{e(k) - e(k-1)}{\Delta t}
 * \f]
 * 
 * donde:
 * - \f$u(k)\f$ = señal de control (salida del PID)
 * - \f$e(k)\f$ = error actual = setpoint - variable_del_proceso
 * - \f$K_p, K_i, K_d\f$ = ganancias proporcional, integral y derivativa
 * - \f$\Delta t\f$ = tiempo de muestreo (sampling_time)
 * 
 * ## Términos del PID
 * 
 * | Término | Efecto | Problema si es muy alto | Problema si es muy bajo |
 * |---------|--------|-------------------------|--------------------------|
 * | **P** (Proporcional) | Respuesta inmediata al error | Oscilaciones, inestabilidad | Respuesta lenta, error estático |
 * | **I** (Integral) | Elimina el error de estado estable | Windup, sobreimpulso | No elimina error estático |
 * | **D** (Derivativo) | Amortigua oscilaciones, predice | Sensible al ruido | Poco efecto |
 * 
 * ## Características implementadas
 * 
 * - **Anti-windup**: La saturación en out_min/out_max previene que la acción
 *   integral crezca indefinidamente cuando el sistema no puede alcanzar el setpoint.
 * 
 * - **Deadband (Zona muerta)**: Evita oscilaciones mantenidas cuando el error
 *   es muy pequeño, "congelando" la salida del PID.
 * 
 * - **Clamping (Limitación)**: La salida siempre se limita a los valores
 *   mínimo y máximo especificados.
 * 
 * @note Para más información sobre tuning de PID, buscar "Ziegler-Nichols method"
 *       o "Cohen-Coon method".
 */

#include <stdint.h>
#include "pico/stdlib.h"

/**
 * @struct pid_ctrl_t
 * @brief Estructura de contexto y configuración del PID.
 * @details
 * Esta estructura contiene todos los parámetros y el estado del controlador PID.
 * Se divide en tres secciones:
 * 
 * 1. **Ganancias** (Parameters): Los valores Kp, Ki, Kd que definen el
 *    comportamiento del controlador.
 * 
 * 2. **Límites** (Constraints): Los valores out_min y out_max que definen
 *    el rango de salida válido.
 * 
 * 3. **Estado interno** (Context): Variables que mantienen el "memoria"
 *    del PID entre llamadas.
 * 
 * ## Campos de Configuración (establecer antes de usar)
 * 
 * | Campo         | Tipo   | Descripción                                                                                              |
 * |---------------|--------|----------------------------------------------------------------------------------------------------------|
 * | kp            | float  | **Ganancia proporcional**. Controla la respuesta inmediata ante el error. Mayor Kp = respuesta más rápida pero más oscilaciones. Un Kp típico para control de motor está entre 0.1 y 10. |
 * | ki            | float  | **Ganancia integral**. Elimina el error de estado estable (offset). Se acumula en 'integral'.小心: Un Ki muy alto causa "windup" y oscilaciones. Valores típicos: 0.01 a 1. |
 * | kd            | float  | **Ganancia derivativa**. Predice el comportamiento futuro basándose en la tasa de cambio del error. Amortigua las oscilaciones. Sensible al ruido. Valores típicos: 0.001 a 1. |
 * | out_min       | float  | **Límite inferior** de la salida del PID (saturación mínima). Típicamente 0 para PWM unidireccional. |
 * | out_max       | float  | **Límite superior** de la salida del PID (saturación máxima). Típicamente 100 para duty cycle en porcentaje, o 65535 para PWM de 16 bits. |
 * | sampling_time | float  | **Período de muestreo** en segundos. Se usa en el cálculo de la acción derivativa: \f$K_d \cdot (e(k) - e(k-1)) / \Delta t\f$. Debe coincidir con la frecuencia real de llamada a pid_calculate(). Ejemplo: dt=0.001 para llamadas cada 1 ms (1 kHz). |
 * 
 * ## Campos de Estado (manejados internamente)
 * 
 * | Campo             | Tipo    | Descripción                                                                                              |
 * |-------------------|---------|----------------------------------------------------------------------------------------------------------|
 * | current_error     | float   | **Error actual**: setpoint - variable_del_proceso. Calculado por pid_set_rpm(). Se guarda para acceso posterior si es necesario. |
 * | last_output       | float   | **Última salida** calculada. Usada para retornar el último valor cuando se está dentro de la deadband. |
 * | deadband_counter  | uint8_t | **Contador de deadband**. Cuenta cuántos ciclos consecutivos el error ha permanecido dentro de la zona muerta (±2.0). Cuando supera DEADBAND, congela la salida. |
 * | integral          | float   | **Acumulador integral** (∑error·dt). Representa la suma histórica del error. Si es muy grande, causa "integral windup". Se resetea con pid_reset(). |
 * | prev_error        | float   | **Error anterior**. El error del ciclo de control anterior. Se usa para calcular la derivada del error: (error - prev_error) / dt. |
 * 
 * @warning El campo 'integral' puede crecer indefinidamente si:
 *          - El sistema no puede alcanzar el setpoint (error siempre positivo)
 *          - No se usa saturación (out_min/out_max)
 *          - Esto se conoce como "integral windup" y causa sobreimpulso.
 * 
 * @example
 * @code
 * pid_ctrl_t motor_pid = {
 *     .kp = 2.5f,           // Ganancia proporcional
 *     .ki = 0.1f,           // Ganancia integral
 *     .kd = 0.05f,          // Ganancia derivativa
 *     .out_min = 0.0f,      // PWM mínimo (0%)
 *     .out_max = 100.0f,    // PWM máximo (100%)
 *     .sampling_time = 0.01f // 10ms entre llamadas
 * };
 * 
 * // En el loop principal:
 * float pwm = pid_set_rpm(encoder.rpm_filtered, 1000.0f, &motor_pid);
 * set_motor_pwm(pwm);
 * @endcode
 */
typedef struct {
    // Ganancias (Parameters)
    /** @brief Ganancia proporcional */
    float kp;
    /** @brief Ganancia integral */
    float ki;
    /** @brief Ganancia derivativa */
    float kd;
    // Límites (Constraints)
    /** @brief Límite inferior de salida (saturación mínima) */
    float out_min;
    /** @brief Límite superior de salida (saturación máxima) */
    float out_max;

    /** @brief Error actual (setpoint - medición) */
    float current_error;
    /** @brief Última salida calculada del PID */
    float last_output;
    /** @brief Contador de ciclos en zona muerta (deadband) */
    uint8_t deadband_counter;

    // Estado interno (Context)
    /** @brief Acumulador integral (∑error·dt) */
    float integral;
    /** @brief Error del ciclo anterior */
    float prev_error;
    /** @brief Tiempo de muestreo en segundos (dt) */
    float sampling_time; // dt en segundos
} pid_ctrl_t;

/**
 * @brief Calcula la salida del PID basada en el error actual.
 * 
 * Implementa la ecuación PID discreta:
 * \f[
 * u(k) = K_p \cdot e(k) + K_i \cdot \sum e(i) \cdot \Delta t + K_d \cdot \frac{e(k) - e(k-1)}{\Delta t}
 * \f]
 * 
 * Después del cálculo, la salida se limita (clamping) a los valores
 * out_min y out_max para evitar sobreestimulación del actuador.
 * 
 * @param[in] pid   Puntero al contexto del PID (contiene ganancias y estado)
 * @param[in] error Error actual: setpoint - variable_del_proceso
 * 
 * @return Salida del PID limitada entre out_min y out_max
 * 
 * @note Esta función debe llamarse periódicamente con el mismo período
 *       definido en pid->sampling_time para un comportamiento correcto.
 *       Si sampling_time no coincide con la frecuencia real de llamada,
 *       la acción derivativa y la integral tendrán errores.
 * 
 * @warning La acción integral (ki) puede causar "windup" si el sistema
 *          permanece en saturación (la salida no puede alcanzar el setpoint).
 *          Para sistemas propensos a esto, considerar anti-windup adicional.
 * 
 * @see pid_set_rpm(), pid_reset()
 */
float pid_calculate(pid_ctrl_t *pid, float error);

/**
 * @brief Reinicia los términos históricos del PID a cero.
 * 
 * Esta función limpia el estado interno del PID, útil cuando:
 * - Se quiere detener el control de forma limpia antes de un cambio de setpoint
 * - Se detecta una condición de error y se quiere reiniciar el control
 * - Se cambia el modo de operación del sistema
 * 
 * Es equivalente a "reiniciar" el controlador desde el inicio.
 * 
 * @param[in] pid Puntero al contexto del PID (acepta NULL para seguridad)
 * 
 * @post 
 * - pid->integral = 0
 * - pid->prev_error = 0
 * - pid->deadband_counter = 0
 * 
 * @note Los valores de Kp, Ki, Kd, out_min, out_max y sampling_time
 *       NO se modifican. Solo se reinician las "variables de estado".
 * 
 * @example
 * @code
 * // Antes de cambiar el setpoint drásticamente:
 * pid_reset(&motor_pid);
 * set_new_setpoint(500.0f);
 * @endcode
 */
void pid_reset(pid_ctrl_t *pid);

/**
 * @brief Wrapper del PID con zona muerta (deadband) para control de RPM.
 * 
 * Esta función combina varias operaciones:
 * 1. Calcula el error actual: setpoint - rpm_value
 * 2. Verifica si el error está dentro de la zona muerta (|error| < 2.0)
 * 3. Si está en la zona muerta por muchos ciclos, congela la salida
 * 4. Si no, calcula la salida del PID usando pid_calculate()
 * 
 * ## Cómo funciona la Deadband (Zona Muerta)
 * 
 * Cuando el sistema está muy cerca del setpoint (|error| < 2.0 RPM),
 * pequeñas variaciones pueden causar que la salida del PID oscile,
 * creando un "hunting" (persecución) inestable. La deadband evita esto:
 * 
 * ```
 * Error |error|<2.0?
 *    No → Reset counter, calcular PID normalmente
 *    Sí → Incrementar counter
 *          counter > 10? → Congelar salida (retornar last_output)
 * ```
 * 
 * @param[in] rpm_value  Valor actual de RPM (variable del proceso/medición)
 * @param[in] set_point Valor deseado de RPM (setpoint)
 * @param[in] pid       Puntero al contexto del PID
 * 
 * @return 
 * - Salida del PID calculada normalmente, O
 * - last_output si está en deadband por más de 10 ciclos
 * 
 * @note Los 10 ciclos de deadband asumen que esta función se llama a ~100Hz.
 *       En ese caso, 10 ciclos = ~100ms de espera antes de congelar.
 *       Ajustar según la frecuencia real de llamada.
 * 
 * @pre El encoder debe haber calculado rpm_value (ej. encoder_get_rpm_filtered())
 * 
 * @post pid->current_error, pid->last_output, pid->deadband_counter se actualizan.
 * 
 * @see pid_calculate(), pid_reset()
 */
float pid_set_rpm(float rpm_value, float set_point, pid_ctrl_t *pid);