#ifndef ENCODER_ROT_H
#define ENCODER_ROT_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "pico/stdlib.h"


typedef struct{
    uint8_t pin_dt;
    uint8_t pin_clk;
    uint8_t pin_sw;
} enc_rot_conf_t;

typedef struct {
    enc_rot_conf_t conf;
    int32_t counter;   // Posicion acumulada desde la configuracion.
    int8_t step;       // -1, 0 o +1 en esta llamada.
    bool pressed;     // Un pulso al detectar una pulsacion estable.
    bool down;        // Estado estable del boton, para detectar pulsacion larga en la UI.

    // Estado interno del lector.
    uint8_t previous_state;
    int8_t transitions;
    bool button_raw;
    uint32_t button_changed_us;
    bool configured;
} enc_rot_t;


/* Lectura por polling, sin registrar ni modificar interrupciones GPIO.
 * Un paso corresponde a cuatro transiciones de cuadratura.
 * Configurar una vez; luego llamar update frecuentemente (por ejemplo cada 1 ms).
 * No esperar 100 ms entre lecturas ni bloquear el loop con envios largos.
 * step y pressed se renuevan en cada llamada: consumirlos antes de la siguiente.
 * El boton tiene antirrebote de 20 ms y se considera pulsado con nivel bajo. */
bool encoder_rot_config(enc_rot_t *enc, const enc_rot_conf_t *enc_conf);
void encoder_rot_update(enc_rot_t *enc);

#ifdef __cplusplus
}
#endif

#endif // ENCODER_ROT
