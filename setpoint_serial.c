#include "setpoint_serial.h"
#include <math.h>
#include <stdlib.h>

static char buffer[32];
static size_t length;
static bool discard_line;

void setpoint_serial_update(setpoint_gen_t *sp)
{
    if (sp == NULL) {
        return;
    }

    int c = getchar_timeout_us(0);
    if (c < 0) {
        return;
    }

    if (c == '\n' || c == '\r') {
        if (!discard_line && length > 0) {
            buffer[length] = '\0';
            char *end;
            float value = strtof(buffer, &end);
            if (end != buffer && isfinite(value) && value >= 0) {
                while (*end == ' ' || *end == '\t') {
                    end++;
                }
                if (end == buffer + length) {
                    setpoint_gen_set_target(sp, value);
                }
            }
        }
        length = 0;
        discard_line = false;
    } else if (!discard_line) {
        if (length < sizeof(buffer) - 1) {
            buffer[length++] = (char)c;
        } else {
            // Descartar hasta Enter, sin aplicar un numero cortado.
            discard_line = true;
        }
    }
}
