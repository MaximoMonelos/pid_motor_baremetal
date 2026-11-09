#ifndef SETPOINT_GEN_H
#define SETPOINT_GEN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
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


typedef struct {
    /** @brief Modo de operación del generador */
    setpoint_mode_t mode;
    /** @brief Valor objetivo del setpoint (RPM) */
    float target_value;
    /** @brief Velocidad de transición en modo rampa (RPM/s). Debe ser > 0. */
    float ramp_rate;
} setpoint_gen_config_t;


typedef struct {
    /** @brief Configuración activa del generador */
    setpoint_gen_config_t config;
    /** @brief Valor actual del setpoint (RPM), salida del generador */
    float current_value;
    /** @brief Indica si el setpoint alcanzó el valor objetivo */
    bool arrived;
} setpoint_gen_t;

void setpoint_gen_init(setpoint_gen_t *sp_gen, const setpoint_gen_config_t *config);

void setpoint_gen_update(setpoint_gen_t *sp_gen, float dt_s);

/**
 * @brief Establece un nuevo valor objetivo para el setpoint.
 *
 * Permite cambiar el target_value en tiempo de ejecución sin
 * reinicializar todo el generador. El current_value actual se
 * mantiene como punto de partida de la nueva transición.
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
