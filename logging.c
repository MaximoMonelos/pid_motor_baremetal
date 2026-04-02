// logging.c
#include "logging.h"
#include <stdarg.h>
#include <stdio.h> 

void log_init(void) {
    // Como ya no hay RTOS ni Mutex, esta función puede quedar vacía.
    // Podés eliminarla completamente si tampoco la necesitas en tu main.
}

void log_safe_print(log_level_t level, const char *tag, const char *format, ...) {
    va_list args;
    va_start(args, format);

    const char *color_code;
    const char *level_char;

    // Asignación de colores y letras según el nivel de log
    switch (level) {
        case LOG_LEVEL_ERROR: color_code = ANSI_COLOR_RED;    level_char = "E"; break;
        case LOG_LEVEL_WARN:  color_code = ANSI_COLOR_YELLOW; level_char = "W"; break;
        case LOG_LEVEL_INFO:  color_code = ANSI_COLOR_GREEN;  level_char = "I"; break;
        case LOG_LEVEL_DEBUG: color_code = ANSI_COLOR_BLUE;   level_char = "D"; break;
        default:              color_code = ANSI_COLOR_RESET;  level_char = "?"; break;
    }

    // Imprimir cabecera (Color, Nivel, Tag)
    printf("%s[%s] [%s] ", color_code, level_char, tag);
    
    // Imprimir el mensaje en sí
    vprintf(format, args);
    
    // Restaurar color y hacer salto de línea
    printf("%s\n", ANSI_COLOR_RESET);

    va_end(args);
}

const char* error_to_name(error_t code) {
    switch(code) {
        case ERR_OK: return "OK";
        case ERR_FAIL: return "FAIL";
        case ERR_INVALID_ARG: return "INVALID_ARG";
        case ERR_NO_MEM: return "NO_MEM";
        case ERR_TIMEOUT: return "TIMEOUT";
        case ERR_NOT_FOUND: return "NOT_FOUND";
        case ERR_NOT_SUPPORTED: return "NOT_SUPPORTED";
        case ERR_BUSY: return "BUSY";
        case ERR_IO: return "IO_ERROR";
        case ERR_INVALID_STATE: return "INVALID_STATE";
        default: return "UNKNOWN";
    }
}

__attribute__((weak)) void app_error_handler(error_t code, const char *msg, const char *func, int line) {
    LOGE("SYS", "Error %s (%d) en %s:%d -> %s", error_to_name(code), code, func, line, msg);
}