/**
 * @file motor.c
 * @author MaximoMonelos (monelosmaximo@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-03-21
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "motor.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#define MOTOR_CONFIG_TAG "MOTOR_CONFIG"
#define DEBUG_MODE 1
/**
 * @brief Traduce códigos de error numéricos a descripciones de texto legibles.
 * 
 * Esta función actúa como un diccionario (lookup table) que mapea cada constante 
 * definida en @ref motor_error_t a una cadena de caracteres específica. Sirve para 
 * transformar valores numéricos abstractos en mensajes comprensibles, facilitando el 
 * diagnóstico y la depuración del sistema a través del monitor serial.
 * * @param res Código de error a traducir.
 * @return const char* Puntero a la cadena de texto descriptiva correspondiente al error.
 */
const char* motor_strerror(motor_error_t res){
    switch (res) {
        case MOTOR_SUCCESS:
            return "OK :D";
        case MOTOR_ERROR:
            return "Error general del motor";
        case MOTOR_ERROR_PIN_INVALIDO:
            return "Pin GPIO fuera de rango (debe ser 0-29)";
        case MOTOR_ERROR_PIN_RESERVADO:
            return "Pin GPIO reservado";
        case MOTOR_ERROR_PUNTERO_NULO:
            return "Puntero a la estructura del motor es NULL";
        case MOTOR_ERROR_PWM_INVALIDO:
            return "Error al inicializar el hardware PWM";
        case MOTOR_ERROR_DIRECCION_INVALIDA:
            return "Direccion de giro no reconocida";
        case MOTOR_ERROR_DIV_INVALIDO:
            return "Valor de frecuencia no alcanzable";
        case MOTOR_ERROR_SLICE_INVALIDO:
            return "Slice de PWM fuera de rango (debe ser 0-8)";
        case MOTOR_WARNING_DUTY_CYCLE_SUPERADO:
            return "Valor Duty Cycle superado (fue truncado)";
        case MOTOR_WARNING_FREQ_MAX_SUPERADA:
            return "Frecuencia maxima superada (fue truncada a 36k Hz)";
        case MOTOR_WARNING_DIV_MAX_SUPERADO:
            return "Divisor de reloj maximo superado (se limito el calculo)";
        default:
            return "Error de motor desconocido";
    }
}

static char motor_msg[128]; 

/**
 * @brief Filtro inteligente y gestor de logs para la validación de estados del motor.
 * 
 * * Esta función centraliza el procesamiento de retornos, liberando al programador de la 
 * tarea de validar datos y gestionar logs manualmente en cada llamada. Su objetivo es 
 * abstraer la lógica de reporte mediante una SEPARACIÓN DE CATEGORÍAS:
 * * 1. **Filtrado de Éxito**: Si la operación es exitosa (@ref MOTOR_SUCCESS), la función 
 * valida el estado y solo genera un log informativo si **DEBUG_MODE** está activo. 
 * Esto permite mantener una consola limpia en producción sin perder trazabilidad en desarrollo.
 * * 2. **Categorización de Severidad**: 
 * - **Warnings**: Identifica resultados no fatales (como el truncamiento de valores). 
 * Imprime automáticamente un log de advertencia y permite continuar la ejecución.
 * - **Errores Críticos**: Captura fallos de hardware o punteros, imprimiendo logs de 
 * error de alta prioridad para alertar sobre estados inválidos.
 * * @param context Etiqueta descriptiva del punto de llamada (ej: "PWM_CFG", "DIR_SET"). 
 * Ayuda a localizar el origen del reporte en el monitor serial.
 * @param res     Resultado de tipo @ref motor_error_t a filtrar y procesar.
 * * @return motor_error_t Retorna el mismo código 'res', permitiendo su uso directo en 
 * estructuras de control o retornos de función.
 * * @note El uso de esta función garantiza que el usuario no necesite implementar 
 * lógica de impresión de logs personalizada en las funciones de control de bajo nivel.
 */
motor_error_t motor_raise_error(const char* context, motor_error_t res) {
    if (res == MOTOR_SUCCESS) {
        #if DEBUG_MODE
        LOGI(context, "%s", motor_strerror(res));
        #endif
        return MOTOR_SUCCESS;
    }

    const char* msg = motor_strerror(res);
    
    snprintf(motor_msg, sizeof(motor_msg), "Fallo en motor: %s", msg);

    if (res > 1) {
        LOGW(context, "%s", motor_msg);
    } else {
        LOGE(context, "%s", motor_msg);
        LOGE("[STOP]", "Abortando main_task para evitar daños en el driver del motor");
    }
    return res;
}

/**
 * @brief Inicializa un pin GPIO como salida digital aplicando filtros de seguridad.
 * 
 * Esta función realiza una validación previa del hardware antes de la inicialización. 
 * Verifica que el pin esté dentro del rango físico del RP2040 (0-29) y que no pertenezca 
 * a los pines reservados para el sistema (23-25). Al finalizar la configuración, 
 * establece el pin en estado bajo (0) para asegurar un estado inicial conocido.
 * * @param pin Número de GPIO a configurar como salida de lógica.
 * @return motor_error_t Resultado de la operación procesado por @ref motor_raise_error.
 */
motor_error_t config_gpio_output(uint8_t pin){

//-----------Validacion pines--------------

    if(pin > 29) return motor_raise_error(MOTOR_CONFIG_TAG, MOTOR_ERROR_PIN_INVALIDO);
    if(pin >= 23 && pin <= 25) return motor_raise_error(MOTOR_CONFIG_TAG, MOTOR_ERROR_PIN_RESERVADO);

//-------------------------------------------
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
    return motor_raise_error("OUTPUT_PIN", MOTOR_SUCCESS);
}


/**
 * @brief Inicializa un pin GPIO para su uso con el periférico de hardware PWM.
 * 
 * Realiza una comprobación de seguridad para asegurar que el pin sea válido 
 * y no esté reservado por el sistema. Si la validación es exitosa, inicializa el pin 
 * y cambia su multiplexación interna a la función @ref GPIO_FUNC_PWM, vinculándolo 
 * directamente al generador de señales PWM del microcontrolador.
 * * @param pin Número de GPIO destinado a la señal de velocidad (PWM).
 * @return motor_error_t Resultado de la operación procesado por @ref motor_raise_error.
 */
motor_error_t config_gpio_pwm(uint8_t pin){

//-----------Validacion pines--------------

    if(pin > 29) return motor_raise_error(MOTOR_CONFIG_TAG, MOTOR_ERROR_PWM_INVALIDO);
    if(pin >= 23 && pin <= 25) return motor_raise_error(MOTOR_CONFIG_TAG,MOTOR_ERROR_PIN_RESERVADO);

//-------------------------------------------

    gpio_init(pin);
    gpio_set_function(pin, GPIO_FUNC_PWM);

    return motor_raise_error("PWM_PIN", MOTOR_SUCCESS);
}

/**
 * @brief Función principal para la inicialización integral del hardware del motor.
 *
 * Ejecuta la secuencia de configuración necesaria para poner el motor en estado operativo. 
 * Realiza el seteo de los pines GPIO (tanto para PWM como para lógica de dirección), 
 * asigna el slice de hardware PWM correspondiente y establece los parámetros base de 
 * funcionamiento como el Wrap, el nivel de potencia inicial y el sentido de giro. 
 * Incluye una validación de seguridad del puntero de entrada antes de proceder.
 *
 * @param m Puntero a la estructura @ref motor_t que contiene los parámetros de configuración.
 * @return motor_error_t Resultado de la inicialización procesado por @ref motor_raise_error.
 */
motor_error_t motor_config(motor_t *m, const motor_config_t *motor_conf){
    
    if(m == NULL){
        return motor_raise_error(MOTOR_CONFIG_TAG, MOTOR_ERROR_PUNTERO_NULO);
    } else motor_raise_error("POINTER", MOTOR_SUCCESS);

    //-------------Guardo la config--------------
    m->motor_config = *motor_conf;
    
    MOTOR_CHECK(config_gpio_pwm(m->motor_config.pin_pwm));
    MOTOR_CHECK(config_gpio_output(m->motor_config.pin_a));
    MOTOR_CHECK(config_gpio_output(m->motor_config.pin_b));
    
    MOTOR_CHECK(calculate_pwm_parameters(&(m->motor_config.frequency_hz), &m->pwm_internal_config));

    MOTOR_CHECK(pwm_set_config(m));

    MOTOR_CHECK(motor_set_lvl(m, 0));
    MOTOR_CHECK(motor_pwm_set_enabled(m->pwm_internal_config.slice, 1));
    
    MOTOR_CHECK(motor_set_dir(m, m->dir));

    return motor_raise_error("MOTOR_CONFIG", MOTOR_SUCCESS);
}

motor_error_t motor_pwm_set_enabled(uint8_t slice, bool estado){
    if(slice > 8) return motor_raise_error(MOTOR_CONFIG_TAG, MOTOR_ERROR_SLICE_INVALIDO);
    pwm_set_enabled(slice, estado);
    return motor_raise_error("PWM_ENABLED", MOTOR_SUCCESS);
}

motor_error_t pwm_set_config(motor_t *m){
    if(m == NULL){
        return motor_raise_error(MOTOR_CONFIG_TAG, MOTOR_ERROR_PUNTERO_NULO);
    } else motor_raise_error("POINTER", MOTOR_SUCCESS);

    m->pwm_internal_config.slice = (uint8_t)pwm_gpio_to_slice_num(m->motor_config.pin_pwm);
    pwm_set_clkdiv(m->pwm_internal_config.slice, m->pwm_internal_config.clk_div);
    pwm_set_wrap(m->pwm_internal_config.slice, m->pwm_internal_config.wrap_value);

    return motor_raise_error("PWM_SET_CONFIG", MOTOR_SUCCESS);
}

/**
 * @brief Establece el nivel de potencia (Duty Cycle) del motor.
 *
 * Realiza una validación de seguridad sobre el puntero de entrada y comprueba que 
 * el valor de PWM solicitado no exceda el límite definido en el WRAP del hardware. 
 * En caso de superación, aplica una saturación al valor máximo permitido, actualiza 
 * el registro de hardware y reporta un estado de advertencia. Si el valor es válido, 
 * actualiza el periférico y el estado interno de la estructura.
 *
 * @param m         Puntero a la estructura @ref motor_t.
 * @param pwm_value Valor de intensidad solicitado para el ciclo de trabajo.
 * @return motor_error_t Resultado de la operación filtrado por @ref motor_raise_error.
 */
motor_error_t motor_set_lvl(motor_t *m, float duty_cycle){

//-----------Validacion puntero--------------
    if(m == NULL){
        return motor_raise_error(MOTOR_CONFIG_TAG, MOTOR_ERROR_PUNTERO_NULO);
    }
//-------------------------------------------

    motor_error_t res = validate_duty_cycle(&duty_cycle);
    m->duty_cycle = duty_cycle;

    m->pwm_internal_config.pwm_value = (uint16_t)(( (float)m->pwm_internal_config.wrap_value * duty_cycle) * 0.01f);
    pwm_set_gpio_level(m->motor_config.pin_pwm, m->pwm_internal_config.pwm_value);
    return motor_raise_error("DUTY_SET", res);
}


/**
 * @brief Configura el sentido de giro del motor mediante lógica de GPIO.
 *
 * Valida la integridad del puntero y utiliza un bloque switch-case para gestionar
 * los estados de giro. Incluye un caso default de seguridad que detiene el motor
 * ante valores de dirección inválidos.
 *
 * @param m   Puntero a la estructura motor_t.
 * @param dir Dirección de giro solicitada (CLOCKWISE, COUNTERCLOCKWISE, STOP).
 * @return motor_error_t Resultado de la operación.
 */
motor_error_t motor_set_dir(motor_t *m, dir_t dir) { 

    //-----------Validación puntero--------------
    if (m == NULL) {
        return motor_raise_error(MOTOR_CONFIG_TAG, MOTOR_ERROR_PUNTERO_NULO);
    }

    // Implementación de switch-case con seguridad según el issue
    switch (dir) {
        case CLOCKWISE:
            gpio_put(m->motor_config.pin_a, 0);
            gpio_put(m->motor_config.pin_b, 1);
            break;

        case COUNTERCLOCKWISE:
            gpio_put(m->motor_config.pin_a, 1);
            gpio_put(m->motor_config.pin_b, 0);
            break;

        case STOP:
            gpio_put(m->motor_config.pin_a, 0);
            gpio_put(m->motor_config.pin_b, 0);
            break;

        default:
            gpio_put(m->motor_config.pin_a, 0);
            gpio_put(m->motor_config.pin_b, 0);
            return motor_raise_error(MOTOR_CONFIG_TAG, MOTOR_ERROR_DIRECCION_INVALIDA);
    }

    m->dir = dir;
    return motor_raise_error("DIR_CONFIG", MOTOR_SUCCESS);
}

motor_error_t calculate_pwm_parameters(uint32_t *freq, pwm_internal_config_t *internal){

    internal->wrap_value = WRAP_ESTABLECIDO;
    
    if(*freq >= FREQ_MAX){
        *freq = FREQ_MAX;
        motor_raise_error(MOTOR_CONFIG_TAG, MOTOR_WARNING_FREQ_MAX_SUPERADA);
    }

    internal->clk_div = (float)SYS_CLK_FREQ / ((float)*freq * (float)(WRAP_ESTABLECIDO + 1.0f));

    return motor_raise_error("PWM_PARAMETERS", internal->clk_div <= 255.0f ? MOTOR_SUCCESS : MOTOR_WARNING_DIV_MAX_SUPERADO);
    }

motor_error_t validate_duty_cycle(float *duty_cycle){
    if(*duty_cycle > 100.0f){
        *duty_cycle = 100.0f;
        return MOTOR_WARNING_DUTY_CYCLE_SUPERADO;
    }
    return MOTOR_SUCCESS;
}