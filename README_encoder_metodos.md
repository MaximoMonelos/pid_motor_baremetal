# Métodos de medición de frecuencia del encoder

## Cómo cambiar de método

En `driver_encoder_optico.h`, línea 15, cambiá el `#define`:

```c
// Opción 1 (default):
#define ENCODER_METHOD_TIMER

// Opción 2:
#define ENCODER_METHOD_PWM_IC

// Opción 3:
#define ENCODER_METHOD_PIO
```

Solo un método activo a la vez. Recompilar después de cambiar.

---

## Método 1 — Timer hardware (`ENCODER_METHOD_TIMER`)

**Cómo funciona:**  
Un alarm del RP2040 dispara exactamente cada `ENCODER_WINDOW_MS` ms (hardware, no depende del loop). En ese momento captura atómicamente el contador de pulsos que viene de la ISR de GPIO (rise+fall). La frecuencia es `delta_pulsos / 2 / T`.

**Ventaja:** Mínimo cambio respecto a tu código original. Ventana exacta.  
**Limitación:** Error de cuantización = `1/(2*T)`. Con 100ms → ±5 Hz = ±15 RPM.  
**ISR necesaria:** Sí. Pasar `master_callback` al init.  
**Pin del encoder:** Cualquier GPIO válido.

**Ajuste de ventana:**
```c
#define ENCODER_WINDOW_MS  100u   // 100ms: ±5Hz = ±15 RPM (buen balance)
#define ENCODER_WINDOW_MS   50u   //  50ms: ±10Hz = ±30 RPM (más rápido)
#define ENCODER_WINDOW_MS  200u   // 200ms: ±2.5Hz = ±7.5 RPM (más suave)
```

---

## Método 2 — PWM Input Capture (`ENCODER_METHOD_PWM_IC`)

**Cómo funciona:**  
Configura un slice PWM del RP2040 en modo contador de pulsos externos. El hardware cuenta los flancos de SUBIDA del encoder sin ningún overhead de CPU ni ISR. `encoder_get_freq()` lee el contador cada `ENCODER_WINDOW_MS` ms.

**Ventaja:** Cero carga de CPU durante el conteo. Muy limpio.  
**Limitación:** Mismo error de cuantización que el método 1. Solo cuenta rise (no fall), así que `freq` es la mitad de flancos — pero coincide con lo que muestra el osciloscopio (frecuencia de la onda).  
**ISR necesaria:** No. Pasar `NULL` al init.  
**Pin del encoder:** ⚠️ **DEBE ser un pin B de PWM** (pines impares: 1,3,5,7,9,**11**,13,15...). Pin 11 ✓ es válido.

**Verificar el pin:**
```c
// En tu caso PIN_ENCODER = 11 -> slice 5, canal B -> OK para PWM IC
// Si necesitás cambiar el pin, buscá uno que sea "canal B" en el pinout del RP2040
```

---

## Método 3 — PIO Frecuencímetro (`ENCODER_METHOD_PIO`)

**Cómo funciona:**  
Un programa PIO corre a 125 MHz y mide el tiempo entre flancos de subida consecutivos contando ciclos de clock (resolución 8ns). Pushea cada medición al RX FIFO. `encoder_get_freq()` drena el FIFO y promedia los últimos `PIO_PERIOD_SAMPLES` períodos.

**Ventaja:** Precisión teórica < 0.001 Hz. No tiene error de cuantización de ventana.  
**Limitación:** Requiere que el archivo `.pio` esté en el proyecto y que CMakeLists.txt incluya `pico_generate_pio_header`. Ver sección de setup.  
**ISR necesaria:** No. Pasar `NULL` al init.  
**Pin del encoder:** Cualquier GPIO válido.

**Ajuste de suavizado:**
```c
// En driver_encoder_optico.c, método PIO:
#define PIO_PERIOD_SAMPLES  8    // promedio de 8 períodos (~25ms a 315Hz) para PID
#define PIO_PERIOD_SAMPLES  20   // promedio de 20 períodos (~63ms) para display
```

**Setup adicional en CMakeLists.txt:**
```cmake
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/freq_encoder.pio)
target_link_libraries(${PROJECT_NAME} hardware_pio hardware_pwm hardware_timer)
```

---

## Comparativa de precisión a 315 Hz (945 RPM)

| Método | Error Hz | Error RPM | Latencia | CPU overhead |
|--------|----------|-----------|----------|--------------|
| Timer 50ms  | ±10 Hz | ±30 RPM | 50ms | Bajo (ISR) |
| Timer 100ms | ±5 Hz  | ±15 RPM | 100ms | Bajo (ISR) |
| Timer 200ms | ±2.5 Hz| ±7.5 RPM | 200ms | Bajo (ISR) |
| PWM IC 100ms| ±5 Hz  | ±15 RPM | 100ms | **Cero** |
| PIO (8 muestras) | ±0.001 Hz | ±0.003 RPM | ~25ms | Cero |

---

## Flujo de llamadas en el loop

```c
// Todos los métodos usan la misma API:
bool new_sample = encoder_get_freq(&enc);    // retorna true si hay dato nuevo
if(new_sample){
    encoder_get_rpm_filtered(&enc);
    pid_set_rpm(enc.rpm_filtered, SETPOINT, &pid);
    motor_set_lvl(&motor_a, pid.last_output);
}
sleep_ms(LOOP_DELAY_MS);  // puede ser menor que ENCODER_WINDOW_MS
```

## Output serie para graficar

El `main.c` imprime CSV:
```
freq_Hz, rpm_raw, rpm_filtrado
315.20, 945.60, 944.80
314.95, 944.85, 945.10
...
```
Podés abrirlo directo en Excel/Python/SerialPlot para comparar los tres métodos.
