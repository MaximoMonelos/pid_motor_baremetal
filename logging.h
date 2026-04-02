// logging.h
#ifndef LOGGING_H
#define LOGGING_H

#include <stdio.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ERR_OK          = 0,    // Todo bien
    ERR_FAIL        = -1,   // Error genérico
    ERR_INVALID_ARG = -2,   // Argumento nulo o fuera de rango
    ERR_NO_MEM      = -3,   // Malloc falló
    ERR_TIMEOUT     = -4,   // Expiró el tiempo de espera
    ERR_NOT_FOUND   = -5,   // Recurso no encontrado
    ERR_NOT_SUPPORTED = -6, // Operación no soportada
    ERR_BUSY        = -7,   // Recurso ocupado (Mutex/Semáforo)
    ERR_IO          = -8,   // Error de hardware (I2C/SPI/UART)
    ERR_INVALID_STATE = -9  // La máquina de estados no permite esto
} error_t;

// Definición de Colores ANSI
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_RESET   "\x1b[0m"

// Niveles de Log
typedef enum {
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARN,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_DEBUG
} log_level_t;

// Función de inicialización (llamar antes de arrancar el scheduler)
void log_init(void);

// La función segura (no la llames directamente, usa los macros)
void log_safe_print(log_level_t level, const char *tag, const char *format, ...);

void app_error_handler(error_t code, const char *msg, const char *func, int line);

// Función auxiliar para convertir el enum a string (útil para logs)
const char* error_to_name(error_t code);

// --- MACROS PARA EL USUARIO ---
// Uso: LOGI("MOTOR", "Corriente actual: %.2f A", corriente);

#define LOGI(tag, fmt, ...) log_safe_print(LOG_LEVEL_INFO,  tag, fmt, ##__VA_ARGS__)
#define LOGW(tag, fmt, ...) log_safe_print(LOG_LEVEL_WARN,  tag, fmt, ##__VA_ARGS__)
#define LOGE(tag, fmt, ...) log_safe_print(LOG_LEVEL_ERROR, tag, fmt, ##__VA_ARGS__)
#define LOGD(tag, fmt, ...) log_safe_print(LOG_LEVEL_DEBUG, tag, fmt, ##__VA_ARGS__)

/**
 * CHECK_OR_RETURN:
 * Si la condición es falsa:
 * 1. Llama al error_handler global.
 * 2. Imprime el LOGE (dentro del handler o aquí).
 * 3. Retorna el código de error.
 */
#define CHECK_OR_RETURN(cond, err_code, msg) do { \
    if (!(cond)) { \
        /* Llamamos al handler centralizado antes de salir */ \
        app_error_handler((err_code), (msg), __func__, __LINE__); \
        return (err_code); \
    } \
} while(0)

/**
 * CHECK_OR_EXIT:
 * Igual que arriba pero salta a una etiqueta (goto) para limpiar recursos.
 */
#define CHECK_OR_EXIT(cond, label, err_code, msg) do { \
    if (!(cond)) { \
        app_error_handler((err_code), (msg), __func__, __LINE__); \
        goto label; \
    } \
} while(0)

#ifdef __cplusplus
}
#endif

#endif // LOGGING_H