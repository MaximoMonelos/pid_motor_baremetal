# ssd1306_custom

Biblioteca en C para OLED SSD1306 por I²C en Raspberry Pi Pico/Pico 2. Incluye
gráficos, texto, menús y barras, más una **capa opcional de pantallas 128×64**
que distribuye la información sin pedir coordenadas. No requiere FreeRTOS.

La aplicación entrega mediciones y eventos de botones/encoder. La biblioteca
dibuja la interfaz; no adquiere sensores ni controla motores.

## Integración en otro proyecto

Ubicá este repositorio en `lib/ssd1306_custom`. En el CMake de tu aplicación,
**después de `pico_sdk_init()`**:

```cmake
add_subdirectory(lib/ssd1306_custom)
target_link_libraries(mi_app PRIVATE ssd1306::all)
```

La biblioteca usa el SDK y la placa que ya configuraste. No inicializa otro SDK,
no cambia el estándar de tu aplicación y no habilita ejemplos por defecto.

| Target | Cabecera | Uso |
|---|---|---|
| `ssd1306::all` | `ssd1306_screens.h` | Biblioteca completa; entrada recomendada. |
| `ssd1306::screens` | `ssd1306_screens.h` | Pantallas prearmadas; incluye la base. |
| `ssd1306::core` | `ssd1306_widgets.h` / `ssd1306.h` | Componentes y dibujos con posiciones propias. |

También existen los targets `ssd1306_custom`, `ssd1306_screens` y `ssd1306` sin
prefijo. No necesitás copiar cabeceras ni enlazar manualmente dependencias del SDK.
Requisitos: Pico SDK inicializado, compilador C11 y CMake 3.13 o posterior.
Verificado con Pico SDK 2.2.0, ARM GCC 14.2 y Pico 2.

## Inicializar el display

Los ejemplos usan OLED 128×64, dirección 0x3C, I²C0, SDA **GP4** (pin físico 6),
SCL **GP1** (pin físico 2), 3V3 y GND. Los pines se configuran en la aplicación:

```c
#include "ssd1306_screens.h"

static ssd1306_t oled = {0};

bool display_init(void)
{
    i2c_init(i2c0, 400000);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(1);

    return ssd1306_init(&oled, 128, 64, 0x3C,
                        i2c0, SSD1306) == SSD1306_OK;
}
```

Inicializá la estructura en cero y mantenela válida durante el uso. El driver
reserva el framebuffer; `ssd1306_deinit(&oled)` lo libera. Desinicializá antes de
reinicializar y no copies por valor una instancia ya inicializada.

## Pantallas prearmadas

Definís una vez etiquetas, unidades y pantallas. No necesitás coordenadas:

```c
enum { INICIO, MONITOR, REFERENCIA };
static const screen_menu_item_t opciones[] = {
    {"Monitor", MONITOR}, {"Referencia", REFERENCIA}
};
static const screen_measurement_t datos[] = {
    {"Velocidad", "RPM", 0}, {"Referencia", "RPM", 0}, {"Salida", "%", 1}
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
        // Unidad, decimales, minimo, maximo y paso:
        .content.editor = {"RPM", 0, 0, 3000, 100}
    }
};
static ssd1306_screens_t ui;

// Una vez, despues de display_init():
ssd1306_screens_init(&ui, &oled, pantallas, 3, INICIO);
ssd1306_screens_update(&ui, REFERENCIA, &referencia, 1);

// En la rutina de pantalla:
float valores[] = {rpm, referencia, pwm};
ssd1306_screens_update(&ui, MONITOR, valores, 3);
ssd1306_screens_show(&ui);
```

Los fragmentos omiten algunos controles de retorno para facilitar la lectura:
comprobá cada resultado contra `SSD1306_OK`. `referencia`, `rpm` y `pwm` representan
tus propias variables. Los valores se copian, por lo que el arreglo puede ser
temporal. Las definiciones y etiquetas deben permanecer válidas e inmutables;
por eso el ejemplo usa `static const`.

### Navegación y edición

| Evento | Menú / mediciones | Editor |
|---|---|---|
| `SCREEN_NEXT` | Siguiente opción/página | Incrementar |
| `SCREEN_PREVIOUS` | Opción/página anterior | Disminuir |
| `SCREEN_ENTER` | Abrir opción | Confirmar y volver |
| `SCREEN_BACK` | Volver | Cancelar y volver |

```c
screen_result_t resultado;
ssd1306_screens_input(&ui, SCREEN_ENTER, &resultado);

if (resultado.action == SCREEN_VALUE_CONFIRMED &&
    resultado.page == REFERENCIA) {
    referencia = resultado.value;
    // Aplicar el cambio a la logica de la aplicacion aca.
}
```

El editor modifica un borrador con límites y paso. La biblioteca nunca escribe
sobre tus variables: entrega el valor al confirmar. Cancelar conserva el anterior.
Actualizar desde afuera una página en edición devuelve `SSD1306_ERR_STATE`;
las demás pueden recibir mediciones normalmente.

También hay menús anidados, `SCREEN_INDICATOR` (número y barra) y `SCREEN_MESSAGE`.
Ver [guía completa](docs/capa-pantallas.md), [API](include/ssd1306_screens.h)
y [ejemplo ejecutable](examples/screens_demo/main.c).

## Componentes para diseños propios

Usá `ssd1306_widgets.h` si querés elegir posiciones. El campo borra el texto
anterior y la barra borra el relleno sobrante cuando baja el valor:

```c
ssd1306_text_field_t campo = {
    .x = 0, .y = 16, .width = 128, .height = 16,
    .scale = 2, .align = SSD1306_ALIGN_RIGHT
};
ssd1306_field_printf(&oled, &campo, "%.0f RPM", rpm);

ssd1306_bar_t barra;
ssd1306_bar_config_t config = {
    .x = 0, .y = 48, .width = 128, .height = 12,
    .min = 0, .max = 3000
};
ssd1306_bar_init(&barra, &oled, &config); // Una vez.
ssd1306_bar_update(&barra, rpm);         // Luego, por cada valor.
ssd1306_show(&oled);
```

Las barras admiten negativos y decimales, y limitan el relleno al rango.
`.vertical = true` activa una barra vertical. La base también ofrece píxeles,
líneas, figuras, imágenes y menús por renglones.

## Actualización, memoria y límites

- Dibujar cambia RAM. `ssd1306_screens_show()` dibuja y transmite solo cuando
  corresponde. Cambios ocultos o sin diferencias de píxeles no causan un envío;
  un fallo I²C deja el envío pendiente para reintentar.
- Las transferencias son bloqueantes, con timeout. La aplicación decide cuándo
  actualizar y reintentar. No se crean tareas ni se debe enviar desde una ISR.
  El driver base ofrece `ssd1306_show_pages()` para repartir el envío.
- Éxito es `SSD1306_OK` (**cero**). Hay errores de argumentos, memoria, I²C,
  timeout y estado. Los campos pueden devolver `SSD1306_ERR_TRUNCATED`.
- Texto ASCII, sin UTF-8 para tildes/ñ. La capa superior acorta etiquetas largas
  y puede abreviar números o suprimir una unidad que no entre.
- Capa superior: **128×64**, hasta 12 pantallas, 8 mediciones por pantalla y
  8 niveles de historial. Ocupa aproximadamente 1,6 KB adicionales en Pico 2
  sin memoria dinámica. El driver reserva 1024 bytes para el framebuffer.
- La base admite configuraciones de ancho 64/128 y alto 32/64; no todas las
  combinaciones corresponden a módulos físicos verificados. Conserva SH1106,
  pero la demo y las comprobaciones actuales se enfocan en SSD1306.
- Cada instancia debe tener un solo dueño. Al mezclar dibujo externo y pantallas
  gestionadas, usá `ssd1306_screens_invalidate()` para restaurar la imagen.
- No incluye persistencia en flash, sensores ni lectura del encoder. Las pruebas
  por software no sustituyen las pruebas con una OLED real.

## Compilar los ejemplos

Al abrir este repositorio directamente como proyecto Pico, los ejemplos se
habilitan por defecto. Se conserva `.vscode` para trabajar con la extensión Pico.

| Ejemplo | Contenido | Tarea de VS Code |
|---|---|---|
| `motor_demo` | Componentes y pantallas simuladas del motor | `Compile Project` / `Run Project` |
| `screens_demo` | Pantallas prearmadas, submenús y edición | `Compile Screens Demo` / `Run Screens Demo` |

Desde una terminal con CMake, Ninja y el toolchain Pico configurados:

```sh
cmake -S . -B build -G Ninja -DPICO_BOARD=pico2
cmake --build build --target motor_demo screens_demo
```

Los `.elf` y `.uf2` quedan en `build/`. Cargar un ejemplo reemplaza el programa de
la placa. No se configura hardware de motor. En terminal USB: `w/s` navegan,
Enter abre/confirma, `b` vuelve/cancela y `m` reinicia el recorrido.

Para compilar solo la biblioteca como proyecto independiente:

```sh
cmake -S . -B build-lib -G Ninja -DPICO_BOARD=pico2 -DSSD1306_BUILD_MOTOR_DEMO=OFF -DSSD1306_BUILD_SCREENS_DEMO=OFF
cmake --build build-lib
```

También se conserva la compilación independiente de `examples/motor_demo`.

## Pruebas sin hardware

Con un compilador C de escritorio (por ejemplo GCC), CMake y Ninja:

```sh
cmake -S tests/screens -B build-tests -G Ninja -DCMAKE_C_COMPILER=gcc
cmake --build build-tests
ctest --test-dir build-tests --output-on-failure
```

No necesitan Pico SDK: simulan I²C y comprueban paginado, navegación anidada,
edición, límites, detección de imágenes idénticas y reintentos después de timeout.

## Estructura

```text
include/               Cabeceras públicas
src/                   Implementación y fuente de texto interna
examples/motor_demo/   Ejemplo de componentes
examples/screens_demo/ Ejemplo de pantallas prearmadas
tests/screens/         Pruebas en PC
docs/                  Guías y demostración visual sin conexión
.vscode/               Configuración opcional del editor y tareas Pico
CMakeLists.txt         Biblioteca y ejemplos opcionales
pico_sdk_import.cmake  Arranque del SDK para compilación independiente
LICENSE                Licencia MIT
```

[Guía visual](docs/guia-oled.html) · [Vista de las pantallas](docs/pantallas-superiores.png)

## Licencia

MIT. Se conservan los avisos de autoría del driver original; ver
[LICENSE](LICENSE) y los encabezados de los archivos derivados.
