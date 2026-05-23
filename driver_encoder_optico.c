#include "driver_encoder_optico.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "pico/stdlib.h"
#include "hardware/sync.h"

// Incluye el header generado por pioasm (solo metodo PIO)
#ifdef ENCODER_METHOD_PIO
#include "freq_encoder.pio.h"
#endif

#define SEG_TO_MIN      60.0f
#define US_TO_S         1000000.0f
#define BLOCK_SIZE      1

// Puntero global para que el alarm callback pueda acceder al encoder.
// Si tenes mas de un encoder, expandir a un array.
static encoder_t *_enc_timer_instance = NULL;

// ============================================================
// METODO 1 — TIMER HARDWARE
// ============================================================
// El alarm del RP2040 dispara cada ENCODER_WINDOW_MS ms de forma
// exacta (hardware, no depende del loop). En el callback:
//   - Captura el contador de pulsos atomicamente
//   - Resetea el alarm para el proximo periodo
//
// La ISR de GPIO sigue contando pulsos en counter_pulses.
// encoder_get_freq() lee la captura cuando timer_fired == true.
// ============================================================

#ifdef ENCODER_METHOD_TIMER

static int64_t _timer_alarm_callback(alarm_id_t id, void *user_data){
    encoder_t *enc = (encoder_t *)user_data;

    // Captura atomica: deshabilitar IRQ un instante
    uint32_t s = save_and_disable_interrupts();
    enc->internal.snap_pulses = enc->internal.counter_pulses;
    restore_interrupts(s);

    enc->internal.timer_fired = true;

    // Retornar el delay en us para re-armar automaticamente
    return (int64_t)(ENCODER_WINDOW_MS * 1000);
}

static void _method_timer_init(encoder_t *enc, void *isr){
    _enc_timer_instance = enc;
    enc->internal.snap_pulses      = 0;
    enc->internal.last_snap_pulses = 0;
    enc->internal.timer_fired      = false;
    enc->internal.counter_pulses   = 0;

    // Configurar GPIO con IRQ rise+fall
    gpio_init(enc->config.pin);
    gpio_set_dir(enc->config.pin, GPIO_IN);
    gpio_pull_up(enc->config.pin);
    gpio_set_irq_enabled_with_callback(enc->config.pin,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, isr);

    // Armar alarm repetitivo
    add_alarm_in_ms(ENCODER_WINDOW_MS, _timer_alarm_callback, enc, true);
}

static bool _method_timer_get_freq(encoder_t *enc){
    if(!enc->internal.timer_fired) return false;
    enc->internal.timer_fired = false;

    uint32_t current = enc->internal.snap_pulses;
    uint32_t delta   = current - enc->internal.last_snap_pulses;
    enc->internal.last_snap_pulses = current;

    // delta flancos en ENCODER_WINDOW_MS ms
    // Cada ranura genera 2 flancos (rise+fall), pero el osciloscopio
    // mide la frecuencia de la onda = flancos / 2.
    // Para ser consistente con el osciloscopio: freq = delta / 2 / T
    float T = ENCODER_WINDOW_MS / 1000.0f;
    enc->freq = ((float)delta / 2.0f) / T;
    return true;
}

#endif // ENCODER_METHOD_TIMER


// ============================================================
// METODO 2 — PWM INPUT CAPTURE
// ============================================================
// El hardware PWM del RP2040 puede contar pulsos externos en vez
// de generar una señal. Se configura el slice en modo B (input)
// apuntando al pin del encoder. El contador hardware se incrementa
// en cada flanco de SUBIDA del pin (solo rise, no fall).
//
// Ventaja: cero overhead de CPU durante el conteo.
// Nota: como solo cuenta rise (no fall), delta flancos = delta ranuras.
//       freq del osciloscopio = delta / T directamente.
// ============================================================

#ifdef ENCODER_METHOD_PWM_IC

static void _method_pwm_ic_init(encoder_t *enc){
    // El pin B de cada slice PWM es el que puede recibir input.
    // Hay que asegurarse que enc->config.pin sea un pin B de PWM.
    // Pines B del RP2040: 1,3,5,7,9,11,13,15,17,19,21,23,25,27,29
    // Si tu pin no es B, usar el pin B del mismo slice.

    gpio_set_function(enc->config.pin, GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(enc->config.pin);
    enc->internal.pwm_slice = slice;

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_RISING); // cuenta flancos rise en pin B
    pwm_config_set_wrap(&cfg, 0xFFFF);                   // maximo wrap
    pwm_init(slice, &cfg, false);                        // no arrancar aun

    pwm_set_counter(slice, 0);
    enc->internal.pwm_last_count   = 0;
    enc->internal.pwm_last_time_us = time_us_32();

    pwm_set_enabled(slice, true);
}

static bool _method_pwm_ic_get_freq(encoder_t *enc){
    uint32_t now = time_us_32();
    uint32_t elapsed_us = now - enc->internal.pwm_last_time_us;

    if(elapsed_us < ENCODER_WINDOW_MS * 1000u) return false;

    uint32_t current_count = pwm_get_counter(enc->internal.pwm_slice);

    // Manejar overflow del contador de 16 bits
    uint32_t delta;
    if(current_count >= enc->internal.pwm_last_count){
        delta = current_count - enc->internal.pwm_last_count;
    } else {
        delta = (0xFFFF - enc->internal.pwm_last_count) + current_count + 1;
    }

    enc->internal.pwm_last_count   = current_count;
    enc->internal.pwm_last_time_us = now;

    // Solo cuenta rise -> delta = numero de ranuras pasadas
    // freq = ranuras/segundo = igual al osciloscopio
    float T = (float)elapsed_us / US_TO_S;
    enc->freq = (T > 0) ? ((float)delta / T) : 0.0f;
    return true;
}

#endif // ENCODER_METHOD_PWM_IC


// ============================================================
// METODO 3 — PIO FRECUENCIMETRO
// ============================================================
// El PIO mide el tiempo entre flancos de subida consecutivos
// en unidades de ciclos de clock (125MHz -> 8ns de resolucion).
//
// El programa PIO pushea al RX FIFO el numero de ciclos de cada
// semiciclo alto de la señal. encoder_get_freq() drena el FIFO,
// promedia los ultimos N periodos y calcula la frecuencia.
//
// Precision teorica a 315Hz: error < 0.001 Hz
// ============================================================

#ifdef ENCODER_METHOD_PIO

#define PIO_PERIOD_SAMPLES  8   // promediar los ultimos N periodos

static uint32_t _pio_period_buf[PIO_PERIOD_SAMPLES];
static uint32_t _pio_period_head = 0;
static uint32_t _pio_period_count = 0;

static void _method_pio_init(encoder_t *enc){
    enc->internal.pio    = ENCODER_PIO;
    enc->internal.pio_sm = ENCODER_PIO_SM;

    // Cargar el programa PIO
    uint offset = pio_add_program(enc->internal.pio, &freq_encoder_program);
    enc->internal.pio_offset = offset;

    freq_encoder_program_init(enc->internal.pio,
                               enc->internal.pio_sm,
                               offset,
                               enc->config.pin);

    for(int i = 0; i < PIO_PERIOD_SAMPLES; i++) _pio_period_buf[i] = 0;
}

static bool _method_pio_get_freq(encoder_t *enc){
    // Drena todos los valores disponibles en el FIFO
    bool got_new = false;
    PIO pio = enc->internal.pio;
    uint sm  = enc->internal.pio_sm;

    while(!pio_sm_is_rx_fifo_empty(pio, sm)){
        uint32_t cycles = pio_sm_get(pio, sm);  // ciclos del semiciclo alto

        if(cycles == 0) continue;  // timeout/overflow del PIO -> ignorar

        // El PIO mide semiciclos (solo parte alta de la onda cuadrada).
        // Periodo completo = 2 * semiciclo (asumiendo duty ~50% del encoder).
        // Para mayor precision se puede medir ambos semiciclos y sumar.
        _pio_period_buf[_pio_period_head] = cycles * 2;  // periodo completo en ciclos
        _pio_period_head = (_pio_period_head + 1) % PIO_PERIOD_SAMPLES;
        if(_pio_period_count < PIO_PERIOD_SAMPLES) _pio_period_count++;
        got_new = true;
    }

    if(!got_new || _pio_period_count == 0) return false;

    // Promediar los ultimos N periodos
    uint64_t sum = 0;
    uint32_t n   = _pio_period_count;
    for(uint32_t i = 0; i < n; i++) sum += _pio_period_buf[i];

    float avg_cycles  = (float)sum / (float)n;
    float pio_clk_hz  = (float)clock_get_hz(clk_sys);  // tipicamente 125MHz
    enc->freq = pio_clk_hz / avg_cycles;

    return true;
}

#endif // ENCODER_METHOD_PIO


// ============================================================
// API PUBLICA
// ============================================================

void encoder_init(encoder_t *enc, encoder_config_t *conf_enc,
                  float *fir_state, void *isr)
{
    enc->config = *conf_enc;
    enc->freq           = 0.0f;
    enc->rpm_raw        = 0.0f;
    enc->rpm_filtered   = 0.0f;

    enc->internal.fir_state = fir_state;
    arm_fir_init_f32(&enc->internal.fir,
                     enc->config.num_taps,
                     enc->config.coef,
                     enc->internal.fir_state,
                     BLOCK_SIZE);

#ifdef ENCODER_METHOD_TIMER
    _method_timer_init(enc, isr);

#elif defined(ENCODER_METHOD_PWM_IC)
    // PWM IC no necesita ISR de GPIO
    (void)isr;
    gpio_set_dir(enc->config.pin, GPIO_IN);
    gpio_pull_up(enc->config.pin);
    _method_pwm_ic_init(enc);

#elif defined(ENCODER_METHOD_PIO)
    // PIO tampoco necesita ISR
    (void)isr;
    _method_pio_init(enc);
#endif
}

bool encoder_get_freq(encoder_t *enc){
#ifdef ENCODER_METHOD_TIMER
    return _method_timer_get_freq(enc);

#elif defined(ENCODER_METHOD_PWM_IC)
    return _method_pwm_ic_get_freq(enc);

#elif defined(ENCODER_METHOD_PIO)
    return _method_pio_get_freq(enc);

#else
    #error "Definir ENCODER_METHOD_TIMER, ENCODER_METHOD_PWM_IC o ENCODER_METHOD_PIO"
    return false;
#endif
}

void encoder_isr(encoder_t *enc){
    // Solo relevante para ENCODER_METHOD_TIMER
    // En los otros metodos esta funcion no hace nada
#ifdef ENCODER_METHOD_TIMER
    enc->internal.counter_pulses++;
#else
    (void)enc;
#endif
}

void encoder_get_rpm_raw(encoder_t *enc){
    enc->rpm_raw = (enc->freq > 0.0f)
        ? (enc->freq / enc->config.ticks * SEG_TO_MIN)
        : 0.0f;
}

void encoder_get_rpm_filtered(encoder_t *enc){
    encoder_get_rpm_raw(enc);
    arm_fir_f32(&enc->internal.fir,
                &enc->rpm_raw,
                &enc->rpm_filtered,
                BLOCK_SIZE);
}
