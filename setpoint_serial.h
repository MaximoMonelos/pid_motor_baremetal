#ifndef SETPOINT_SERIAL_H
#define SETPOINT_SERIAL_H

#include "setpoint_gen.h"

/* Lee un caracter por llamada y aplica las RPM al recibir Enter.
 * Acepta cero y valores positivos finitos, sin un maximo de RPM configurado.
 * Acepta hasta 31 caracteres; descarta lineas invalidas o demasiado largas.
 * Conserva la entrada parcial internamente: un solo lector y generador.
 * Llamar periodicamente desde el loop, nunca desde una interrupcion.
 * No requiere inicializacion, no reserva memoria ni imprime mensajes. */
void setpoint_serial_update(setpoint_gen_t *sp);

#endif
