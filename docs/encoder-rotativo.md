# Encoder rotativo del menú

El encoder manual se lee por polling: no registra interrupciones ni modifica el
callback que usa el encoder óptico del motor.

Pines del esquema: CLK en GP9, DT en GP8 y SW en GP7. El botón conecta a GND al
pulsar y las entradas se configuran con pull-up.

```c
static enc_rot_t encoder_menu;
static const enc_rot_conf_t config = {
    .pin_clk = 9, .pin_dt = 8, .pin_sw = 7
};

// Una vez al inicializar la interfaz; comprobar el retorno.
bool ok = encoder_rot_config(&encoder_menu, &config);

// Desde la rutina frecuente de entrada de la UI:
encoder_rot_update(&encoder_menu);
// encoder_menu.step: -1, 0 o +1 desde esta lectura.
// encoder_menu.counter: posición acumulada.
// encoder_menu.pressed: pulsación nueva, solo durante esta lectura.
// encoder_menu.down: botón mantenido presionado, con antirrebote.
```

La lectura se propone cada 1 ms. No hay esperas dentro del driver. Hay que consumir
`step` y `pressed` antes de la siguiente llamada; la tarea de dibujo puede ejecutarse
con menor frecuencia. `down` permite que la UI distinga una pulsación larga más
adelante, si se elige ese manejo.

El conteo usa cuatro transiciones de cuadratura por paso; las transiciones de ida
y vuelta por rebote se cancelan. El botón exige 20 ms estables. Falta comprobar con
el encoder físico el sentido de giro y la correspondencia entre paso y muesca.

Aunque el giro sea manual, un polling lento puede perder transiciones. El envío
completo de la OLED puede demorar la siguiente lectura: conviene usar envíos
parciales y mantener cortas las demás tareas.

La nueva firma de configuración ya no recibe una ISR. El módulo está listo para
integrarlo en `app_ui`; el programa actual aún no instancia el encoder del menú.
