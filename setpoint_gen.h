/**
 * @file setpoint_gen.h
 * @brief Generador de setpoint configurable para el controlador PID.
 * @author [Autor]
 * @date [Fecha]
 *
 * @details
 * Este módulo genera una señal de referencia (setpoint) para el controlador PID,
 * soportando dos modos de operación:
 *
 * ## Modos de operación
 *
 * | Modo | Comportamiento | Caso de uso |
 * |------|---------------|-------------|
 * | **SETPOINT_MODE_STEP** | Cambio instantáneo al valor objetivo | Pruebas de respuesta al escalón |
 * | **SETPOINT_MODE_RAMP** | Transición gradual con pendiente configurable | Arranque suave, perfilado de velocidad |
 *
 * ## Ecuación del modo rampa
 *
 * En cada llamada a setpoint_gen_update(), el valor actual se actualiza según:
 * \f[
 * v(k) = v(k-1) + \text{signo} \cdot r \cdot \Delta t
 * \f]
 *
 * donde:
 * - \f$v(k)\f$ = valor actual del setpoint en el ciclo k
 * - \f$r\f$ = velocidad de rampa (ramp_rate) en RPM/segundo
 * - \f$\Delta t\f$ = tiempo transcurrido desde la última actualización (en segundos)
 * - \f$\text{signo}\f$ = +1 si target > current, -1 si target < current
 *
 * ## Ejemplo de uso
 *
 * @code
 * // Configuración
 * setpoint_gen_config_t sp_config = {
 *     .mode = SETPOINT_MODE_RAMP,
 *     .target_value = 500.0f,
 *     .ramp_rate = 100.0f,    // 100 RPM por segundo
 * };
 *
 * // Inicialización
 * setpoint_gen_t sp_gen;
 * setpoint_gen_init(&sp_gen, &sp_config);
 *
 * // En el loop:
 * setpoint_gen_update(&sp_gen);
 * float setpoint = sp_gen.current_value;
 * pid_set_rpm(enc.rpm_filtered, setpoint, &pid);
 * @endcode
 *
 * @note Este módulo no tiene dependencias con el hardware. Solo utiliza
 *       funciones de tiempo del Pico SDK (get_absolute_time, absolute_time_diff_us).
 *
 * @see pid_set_rpm(), pid_reset()
 */

#ifndef SETPOINT_GEN_H
#define SETPOINT_GEN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"

/**
 * @enum setpoint_mode_t
 * @brief Modos de operación del generador de setpoint.
 *
 * @details
 * Define cómo se comporta el setpoint al recibir un nuevo valor objetivo.
 *
 * | Valor | Comportamiento |
 * |-------|----------------|
 * | SETPOINT_MODE_STEP | El setpoint salta instantáneamente a target_value |
 * | SETPOINT_MODE_RAMP | El setpoint transiciona gradualmente hacia target_value |
 */
typedef enum {
    /** @brief Modo escalón: cambio instantáneo al objetivo */
    SETPOINT_MODE_STEP,
    /** @brief Modo rampa: transición gradual al objetivo */
    SETPOINT_MODE_RAMP,
} setpoint_mode_t;

/**
 * @struct setpoint_gen_config_t
 * @brief Parámetros de configuración del generador de setpoint.
 *
 * @details
 * Esta estructura contiene los parámetros que definen el comportamiento
 * del generador. Debe configurarse antes de llamar a setpoint_gen_init().
 *
 * | Campo        | Tipo             | Descripción |
 * |--------------|------------------|-------------|
 * | mode         | setpoint_mode_t  | Modo de operación (step o ramp) |
 * | target_value | float            | Valor objetivo del setpoint en RPM |
 * | ramp_rate    | float            | Velocidad de la rampa en RPM/segundo. Solo se usa en SETPOINT_MODE_RAMP. Debe ser > 0. |
 *
 * @note ramp_rate se ignora en modo SETPOINT_MODE_STEP.
 *
 * @example
 * @code
 * // Configuración para rampa de 200 RPM/s hacia 1000 RPM
 * setpoint_gen_config_t cfg = {
 *     .mode = SETPOINT_MODE_RAMP,
 *     .target_value = 1000.0f,
 *     .ramp_rate = 200.0f,
 * };
 * @endcode
 */
typedef struct {
    /** @brief Modo de operación del generador */
    setpoint_mode_t mode;
    /** @brief Valor objetivo del setpoint (RPM) */
    float target_value;
    /** @brief Velocidad de transición en modo rampa (RPM/s). Debe ser > 0. */
    float ramp_rate;
} setpoint_gen_config_t;

/**
 * @struct setpoint_gen_t
 * @brief Contexto y estado del generador de setpoint.
 *
 * @details
 * Estructura principal del módulo. Contiene la configuración activa,
 * el valor actual del setpoint, y el estado interno para el cálculo temporal.
 *
 * Se divide en:
 * 1. **Configuración** (config): Parámetros de comportamiento.
 * 2. **Salida** (current_value): Valor actual del setpoint a usar por el PID.
 * 3. **Estado interno** (last_update, arrived): Variables de control temporal.
 *
 * | Campo        | Tipo              | Descripción |
 * |--------------|-------------------|-------------|
 * | config       | setpoint_gen_config_t | Configuración activa del generador |
 * | current_value | float            | Valor actual del setpoint (RPM). Este es el valor que se pasa al PID. |
 * | arrived      | bool              | Flag que indica si current_value ha alcanzado target_value |
 * | last_update  | absolute_time_t   | Timestamp de la última actualización (uso interno) |
 *
 * @warning No modificar `last_update` manualmente. Es gestionado internamente
 *          por setpoint_gen_update().
 */
typedef struct {
    /** @brief Configuración activa del generador */
    setpoint_gen_config_t config;
    /** @brief Valor actual del setpoint (RPM), salida del generador */
    float current_value;
    /** @brief Indica si el setpoint alcanzó el valor objetivo */
    bool arrived;
    /** @brief Timestamp de la última actualización (uso interno) */
    absolute_time_t last_update;
} setpoint_gen_t;

/**
 * @brief Inicializa el generador de setpoint con la configuración dada.
 *
 * Copia la configuración proporcionada a la estructura del generador,
 * establece current_value a 0.0f (o al target_value si es modo step),
 * y registra el timestamp inicial.
 *
 * ## Comportamiento según modo
 *
 * | Modo | current_value inicial |
 * |------|----------------------|
 * | SETPOINT_MODE_STEP | = config.target_value (cambio instantáneo) |
 * | SETPOINT_MODE_RAMP | = 0.0f (inicia la rampa desde cero) |
 *
 * @param[out] sp_gen  Puntero a la estructura del generador (debe existir)
 * @param[in]  config  Puntero a la configuración deseada
 *
 * @pre sp_gen y config no deben ser NULL.
 * @post sp_gen->current_value está inicializado.
 * @post sp_gen->last_update contiene el timestamp actual.
 * @post sp_gen->arrived se establece apropiadamente.
 *
 * @see setpoint_gen_update(), setpoint_gen_set_target()
 */
void setpoint_gen_init(setpoint_gen_t *sp_gen, const setpoint_gen_config_t *config);

/**
 * @brief Actualiza el valor del setpoint según el modo configurado.
 *
 * Esta función debe llamarse en cada iteración del loop de control.
 * Calcula el nuevo valor de current_value basándose en el tiempo
 * transcurrido y el modo de operación.
 *
 * ## Comportamiento por modo
 *
 * - **SETPOINT_MODE_STEP:** No hace nada (current_value ya es target_value).
 * - **SETPOINT_MODE_RAMP:** Incrementa o decrementa current_value hacia
 *   target_value a la velocidad ramp_rate. Cuando llega al target, lo
 *   fija exactamente y establece arrived = true.
 *
 * ## Cálculo de la rampa
 *
 * ```
 * dt = (tiempo_actual - last_update) en segundos
 * delta = ramp_rate * dt
 *
 * Si target > current:
 *     current += delta
 *     Si current >= target: current = target, arrived = true
 *
 * Si target < current:
 *     current -= delta
 *     Si current <= target: current = target, arrived = true
 * ```
 *
 * @param[in,out] sp_gen  Puntero a la estructura del generador
 *
 * @pre sp_gen debe haber sido inicializado con setpoint_gen_init().
 *
 * @post sp_gen->current_value se actualiza.
 * @post sp_gen->last_update se actualiza al timestamp actual.
 * @post sp_gen->arrived se establece a true si se alcanzó el target.
 *
 * @note Esta función usa get_absolute_time() y absolute_time_diff_us()
 *       del Pico SDK para medir el dt real, lo que la hace independiente
 *       de la frecuencia de llamada.
 *
 * @see setpoint_gen_init(), setpoint_gen_set_target()
 */
void setpoint_gen_update(setpoint_gen_t *sp_gen);

/**
 * @brief Establece un nuevo valor objetivo para el setpoint.
 *
 * Permite cambiar el target_value en tiempo de ejecución sin
 * reinicializar todo el generador. El current_value actual se
 * mantiene como punto de partida de la nueva transición.
 *
 * ## Comportamiento según modo
 *
 * | Modo | Efecto |
 * |------|--------|
 * | SETPOINT_MODE_STEP | current_value se actualiza instantáneamente a new_target |
 * | SETPOINT_MODE_RAMP | Se inicia una nueva transición desde current_value hacia new_target |
 *
 * @param[in,out] sp_gen      Puntero a la estructura del generador
 * @param[in]     new_target  Nuevo valor objetivo (RPM)
 *
 * @post sp_gen->config.target_value = new_target
 * @post sp_gen->arrived = false (se reinicia el flag de llegada)
 * @post En modo step: sp_gen->current_value = new_target
 *
 * @note Esta función NO reinicia el PID. Si se desea un reset del PID
 *       al cambiar de target, debe hacerse externamente con pid_reset().
 *
 * @example
 * @code
 * // Cambiar el objetivo de 500 a 1000 RPM
 * pid_reset(&pid);  // Opcional: limpiar historial del PID
 * setpoint_gen_set_target(&sp_gen, 1000.0f);
 * @endcode
 *
 * @see setpoint_gen_init(), setpoint_gen_update(), pid_reset()
 */
void setpoint_gen_set_target(setpoint_gen_t *sp_gen, float new_target);

/**
 * @brief Cambia el modo de operación del generador.
 *
 * Permite alternar entre modo step y rampa en tiempo de ejecución.
 * El current_value se mantiene intacto al cambiar de modo.
 *
 * @param[in,out] sp_gen    Puntero a la estructura del generador
 * @param[in]     new_mode  Nuevo modo de operación
 *
 * @post sp_gen->config.mode = new_mode
 * @post sp_gen->arrived = false
 * @post En modo step: sp_gen->current_value = sp_gen->config.target_value
 *
 * @see setpoint_mode_t, setpoint_gen_set_target()
 */
void setpoint_gen_set_mode(setpoint_gen_t *sp_gen, setpoint_mode_t new_mode);

#ifdef __cplusplus
}
#endif

#endif // SETPOINT_GEN_H
