/**
 * @file setpoint_gen.c
 * @brief Implementación del generador de setpoint configurable.
 * @author [Autor]
 * @date [Fecha]
 *
 * @details
 * Este archivo contiene la implementación de las funciones del generador
 * de setpoint. El módulo soporta modo escalón (step) y modo rampa (ramp),
 * utilizando el timer absoluto del Pico SDK para calcular el dt real
 * entre actualizaciones.
 */

#include "setpoint_gen.h"

/**
 * @brief Inicializa el generador de setpoint con la configuración dada.
 *
 * Copia la configuración, establece el valor inicial según el modo,
 * y registra el timestamp inicial con get_absolute_time().
 *
 * @param[out] sp_gen  Puntero a la estructura del generador
 * @param[in]  config  Puntero a la configuración deseada
 */
void setpoint_gen_init(setpoint_gen_t *sp_gen, const setpoint_gen_config_t *config){
    if(sp_gen == NULL || config == NULL){
        return;
    }

    sp_gen->config = *config;
    sp_gen->last_update = get_absolute_time();

    if(config->mode == SETPOINT_MODE_STEP){
        sp_gen->current_value = config->target_value;
        sp_gen->arrived = true;
    } else {
        sp_gen->current_value = 0.0f;
        sp_gen->arrived = false;
    }
}

/**
 * @brief Actualiza el valor del setpoint según el modo configurado.
 *
 * En modo STEP, si arrived es false (no debería ocurrir normalmente),
 * fija current_value al target. En modo RAMP, calcula el incremento
 * basado en el tiempo real transcurrido y avanza hacia el target.
 *
 * @param[in,out] sp_gen  Puntero a la estructura del generador
 */
void setpoint_gen_update(setpoint_gen_t *sp_gen){
    if(sp_gen == NULL){
        return;
    }

    if(sp_gen->arrived){
        return;
    }

    if(sp_gen->config.mode == SETPOINT_MODE_STEP){
        sp_gen->current_value = sp_gen->config.target_value;
        sp_gen->arrived = true;
        return;
    }

    // Modo RAMP: calcular dt real
    absolute_time_t now = get_absolute_time();
    int64_t dt_us = absolute_time_diff_us(sp_gen->last_update, now);
    float dt_s = (float)dt_us / 1000000.0f;
    sp_gen->last_update = now;

    float delta = sp_gen->config.ramp_rate * dt_s;

    if(sp_gen->config.target_value > sp_gen->current_value){
        sp_gen->current_value += delta;
        if(sp_gen->current_value >= sp_gen->config.target_value){
            sp_gen->current_value = sp_gen->config.target_value;
            sp_gen->arrived = true;
        }
    } else if(sp_gen->config.target_value < sp_gen->current_value){
        sp_gen->current_value -= delta;
        if(sp_gen->current_value <= sp_gen->config.target_value){
            sp_gen->current_value = sp_gen->config.target_value;
            sp_gen->arrived = true;
        }
    } else {
        // current_value == target_value
        sp_gen->arrived = true;
    }
}

/**
 * @brief Establece un nuevo valor objetivo para el setpoint.
 *
 * Actualiza el target y reinicia el flag de llegada. En modo step,
 * el current_value se actualiza instantáneamente.
 *
 * @param[in,out] sp_gen      Puntero a la estructura del generador
 * @param[in]     new_target  Nuevo valor objetivo (RPM)
 */
void setpoint_gen_set_target(setpoint_gen_t *sp_gen, float new_target){
    if(sp_gen == NULL){
        return;
    }

    sp_gen->config.target_value = new_target;
    sp_gen->arrived = false;

    if(sp_gen->config.mode == SETPOINT_MODE_STEP){
        sp_gen->current_value = new_target;
        sp_gen->arrived = true;
    }
}

/**
 * @brief Cambia el modo de operación del generador.
 *
 * Actualiza el modo y reinicia el flag de llegada. En modo step,
 * el current_value se fija instantáneamente al target actual.
 *
 * @param[in,out] sp_gen    Puntero a la estructura del generador
 * @param[in]     new_mode  Nuevo modo de operación
 */
void setpoint_gen_set_mode(setpoint_gen_t *sp_gen, setpoint_mode_t new_mode){
    if(sp_gen == NULL){
        return;
    }

    sp_gen->config.mode = new_mode;
    sp_gen->arrived = false;

    if(new_mode == SETPOINT_MODE_STEP){
        sp_gen->current_value = sp_gen->config.target_value;
        sp_gen->arrived = true;
    }
}
