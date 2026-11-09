# Capa opcional de pantallas 128×64

La biblioteca `ssd1306_screens` distribuye texto, mediciones, barras y menús sin
pedirte coordenadas. Se enlaza por separado; `ssd1306` y sus componentes siguen
disponibles para dibujos personalizados. No usa FreeRTOS ni crea tareas.

## Cómo probarla

En VS Code: **Terminal → Run Task → Run Screens Demo**. Compila, carga y arranca
`build/screens_demo.elf`. También podés copiar `build/screens_demo.uf2` a la Pico 2
en modo BOOTSEL. La carga reemplaza el programa que tenga la placa.

La OLED debe ser SSD1306 128×64, dirección 0x3C, I²C0, SDA GP4 y SCL GP1.
La demo no configura salidas de motor. Recorre automáticamente monitor, edición,
submenús, indicador PWM y mensaje; los valores son simulados.

Desde el terminal USB:

- `s`: siguiente opción / siguiente página / incrementar valor.
- `w`: anterior opción / anterior página / disminuir valor.
- Enter: abrir / confirmar.
- `b`: volver / cancelar.
- `m`: reiniciar el recorrido automático, descartando una edición pendiente.

Cualquier tecla de navegación detiene el recorrido automático. El recorrido
demuestra primero cancelar la referencia y después confirmarla: solo el segundo
cambio se aplica a la variable de la aplicación.

## Ejemplo mínimo

Después de inicializar `oled` como en la demo, definís las pantallas una vez:

```c
#include "ssd1306_screens.h"

enum { INICIO, MONITOR, REFERENCIA };
static const screen_menu_item_t opciones[] = {
    {"Monitor", MONITOR}, {"Referencia", REFERENCIA}
};
static const screen_measurement_t datos[] = {
    {"Velocidad", "RPM", 0},
    {"Referencia", "RPM", 0},
    {"Salida", "%", 1}
};
static const screen_page_t pantallas[] = {
    [INICIO] = {
        .kind = SCREEN_MENU, .title = "Motor",
        .content.menu = {opciones, 2}
    },
    [MONITOR] = {
        .kind = SCREEN_MEASUREMENTS, .title = "Monitor",
        .content.measurements = {datos, 3}
    },
    [REFERENCIA] = {
        .kind = SCREEN_NUMBER_EDITOR, .title = "Referencia",
        // Unidad, decimales, minimo, maximo, paso:
        .content.editor = {"RPM", 0, 0, 3000, 100}
    }
};
static ssd1306_screens_t ui;

// Inicializar una vez. Comprobar cada retorno contra SSD1306_OK.
ssd1306_screens_init(&ui, &oled, pantallas, 3, INICIO);
ssd1306_screens_update(&ui, REFERENCIA, &referencia, 1);
```

Cuando actualizás las mediciones, pasás los valores en el orden declarado:

```c
float valores[] = {rpm, referencia, pwm};
ssd1306_screens_update(&ui, MONITOR, valores, 3);
ssd1306_screens_show(&ui);
```

El arreglo de valores puede ser temporal: se copia. Las definiciones, listas y
etiquetas deben permanecer válidas e inmutables; por eso el ejemplo usa `static
const`. La instancia `ui` y la pantalla también deben permanecer válidas.

Un evento del encoder o de los botones se entrega así:

```c
screen_result_t resultado;
ssd1306_screens_input(&ui, SCREEN_NEXT, &resultado);
// SCREEN_PREVIOUS, SCREEN_ENTER o SCREEN_BACK funcionan igual.

if (resultado.action == SCREEN_VALUE_CONFIRMED &&
    resultado.page == REFERENCIA) {
    referencia = resultado.value;
    // Aplicar a la lógica del motor acá, si corresponde.
}
```

La capa no guarda punteros a tus variables ni escribe sobre ellas. La edición
modifica un borrador; Enter lo confirma y entrega el valor, Back lo descarta.
Ambos vuelven al menú anterior. Mientras se edita una página, actualizarla desde
afuera devuelve `SSD1306_ERR_STATE`; las demás mediciones pueden seguir llegando.

## Qué distribuye automáticamente

- `SCREEN_MENU`: cuatro opciones visibles, resaltado y desplazamiento; una opción
  puede abrir otro menú. Volver restaura la selección anterior.
- `SCREEN_MEASUREMENTS`: cuatro filas por página y hasta ocho mediciones; siguiente
  y anterior permiten recorrerlas. Etiqueta a la izquierda, valor/unidad a la derecha.
- `SCREEN_INDICATOR`: valor principal grande y barra horizontal con mínimo/máximo.
- `SCREEN_NUMBER_EDITOR`: valor temporal, paso, mínimo, máximo, confirmar y cancelar.
- `SCREEN_MESSAGE`: hasta cuatro renglones, salto explícito con `\n` o corte cada
  21 caracteres. El texto que excede el área visible no se muestra.

## Actualización y errores

`update()` copia valores y marca una pantalla si cambió algún dato. `draw()` prepara
la imagen cuando es necesario, sin enviar por I²C. `show()` hace lo mismo y envía
solo si la imagen requiere actualización. Cambios ocultos o diferencias que no
alteran los píxeles, por ejemplo decimales no mostrados, no provocan un envío.

Si falla el envío, queda pendiente para el próximo `show()`. Esas funciones no
esperan hasta la siguiente tarea ni deciden cuándo reintentar: la aplicación las
llama desde su rutina de pantalla. La transmisión es bloqueante, con timeout.

La capa debe ser dueña del framebuffer. Si dibujás con otro módulo, llamá luego
a `ssd1306_screens_invalidate()` para restaurar y reenviar la pantalla gestionada.
No uses la misma instancia simultáneamente desde interrupciones u otro núcleo.

## Límites actuales

- Solo OLED 128×64 y texto ASCII, sin tildes/ñ UTF-8.
- Hasta 12 pantallas, 8 mediciones por pantalla y 8 niveles de historial.
- Sin memoria dinámica adicional en esta capa; la instancia ocupa aproximadamente
  1,6 KB en Pico 2. El driver base conserva su propio framebuffer de 1024 bytes.
- Hasta 6 decimales. Textos largos se acortan; números grandes pueden usar notación
  científica y omitir una unidad que no entre.
- Los pasos de edición usan `float`; elegí paso/decimales acordes a la magnitud.
- No incluye persistencia en flash, adquisición de sensores ni lectura del encoder.

Para utilizarla desde otro proyecto, agregá este repositorio con `add_subdirectory`
y enlazá `ssd1306_screens`. No hace falta habilitar ninguna demo:

```cmake
add_subdirectory(ruta/a/ssd1306_custom oled)
target_link_libraries(mi_app PRIVATE ssd1306::screens)
```

La demo de componentes anterior sigue siendo `motor_demo`; esta capa y su demo
son opcionales. Ver [API](../include/ssd1306_screens.h) y
[ejemplo completo](../examples/screens_demo/main.c).
