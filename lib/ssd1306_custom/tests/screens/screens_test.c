#include "ssd1306_screens.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static int transactions, failure;
int i2c_write_timeout_us(i2c_inst_t *bus, uint8_t addr, const uint8_t *data,
    size_t count, bool no_stop, unsigned timeout) {
    (void)bus; (void)addr; (void)data; (void)no_stop; (void)timeout;
    ++transactions;
    return failure ? failure : (int)count;
}

enum { HOME, DATA, EDIT, SUB, INDICATOR, MESSAGE };
static const screen_menu_item_t menu[] = {
    {"Datos", DATA}, {"Editar", EDIT}, {"Submenu", SUB},
    {"Barra", INDICATOR}, {"Aviso", MESSAGE}
};
static const screen_menu_item_t nested[] = {{"Editar", EDIT}, {"Bucle", SUB}};
static const screen_measurement_t fields[] = {
    {"RPM", "RPM", 0}, {"Referencia", "RPM", 0}, {"PWM", "%", 1},
    {"Kp", NULL, 4}, {"Ki", NULL, 4}
};
static const screen_page_t pages[] = {
    {.kind=SCREEN_MENU, .title="Inicio", .content.menu={menu, 5}},
    {.kind=SCREEN_MEASUREMENTS, .title="Mediciones", .content.measurements={fields, 5}},
    {.kind=SCREEN_NUMBER_EDITOR, .title="Editar", .content.editor={"RPM", 0, -100, 100, 10}},
    {.kind=SCREEN_MENU, .title="Submenu", .content.menu={nested, 2}},
    {.kind=SCREEN_INDICATOR, .title="PWM", .content.indicator={"%", 1, 0, 100}},
    {.kind=SCREEN_MESSAGE, .title="Aviso", .content.message={"Primera linea\nSegunda linea"}}
};

static ssd1306_screens_t ui;
static uint8_t storage[1026];
static i2c_inst_t bus;
static ssd1306_t display = {.width=128, .height=64, .pages=8, .bufsize=1024,
    .buffer=storage+1, .i2c_i=&bus, .address=0x3C, .timeout_us=10000};

static screen_result_t event(screen_input_t input) {
    screen_result_t result;
    assert(ssd1306_screens_input(&ui,input,&result)==SSD1306_OK);
    return result;
}

int main(void) {
    storage[0]=0xA5; storage[1025]=0x5A;
    assert(ssd1306_screens_init(&ui,&display,pages,6,EDIT)==SSD1306_ERR_ARGUMENT);
    assert(ssd1306_screens_init(&ui,&display,pages,6,HOME)==SSD1306_OK);
    float values[]={1000,1500,50,0.02f,0.0049f};
    assert(ssd1306_screens_update(&ui,DATA,values,5)==SSD1306_OK);
    assert(ssd1306_screens_update(&ui,DATA,values,4)==SSD1306_ERR_ARGUMENT);
    float invalid=NAN;
    assert(ssd1306_screens_update(&ui,INDICATOR,&invalid,1)==SSD1306_ERR_ARGUMENT);
    assert(ssd1306_screens_show(&ui)==SSD1306_OK && transactions>0);
    transactions=0;
    assert(ssd1306_screens_show(&ui)==SSD1306_OK && !transactions);
    // Changes to a hidden page must not send the current unchanged image.
    values[0]=1001;
    assert(ssd1306_screens_update(&ui,DATA,values,5)==SSD1306_OK);
    assert(ssd1306_screens_show(&ui)==SSD1306_OK && !transactions);
    event(SCREEN_ENTER);
    assert(ssd1306_screens_show(&ui)==SSD1306_OK);
    uint8_t first_page[1024]; memcpy(first_page,display.buffer,1024);
    event(SCREEN_NEXT);
    bool changed=false;
    assert(ssd1306_screens_draw(&ui,&changed)==SSD1306_OK && changed);
    assert(memcmp(first_page,display.buffer,1024)!=0); // Fifth item is on page 2.
    event(SCREEN_PREVIOUS);
    assert(ssd1306_screens_show(&ui)==SSD1306_OK);
    transactions=0; values[0]=1001.1f;
    assert(ssd1306_screens_update(&ui,DATA,values,5)==SSD1306_OK);
    assert(ssd1306_screens_show(&ui)==SSD1306_OK && !transactions); // Same rounded pixels.
    event(SCREEN_BACK); event(SCREEN_NEXT);
    float application_value=20;
    assert(ssd1306_screens_update(&ui,EDIT,&application_value,1)==SSD1306_OK);
    event(SCREEN_ENTER); event(SCREEN_NEXT);
    assert(application_value==20); // No application pointer is stored or changed.
    assert(ssd1306_screens_update(&ui,EDIT,&application_value,1)==SSD1306_ERR_STATE);
    screen_result_t result=event(SCREEN_BACK);
    assert(result.action==SCREEN_EDIT_CANCELLED && result.value==20 && result.page==EDIT);
    event(SCREEN_ENTER); event(SCREEN_NEXT);
    result=event(SCREEN_ENTER);
    assert(result.action==SCREEN_VALUE_CONFIRMED && result.value==30 && result.page==EDIT);
    application_value=result.value;
    assert(application_value==30);
    event(SCREEN_ENTER);
    for(int i=0;i<30;++i) event(SCREEN_NEXT);
    result=event(SCREEN_ENTER); assert(result.value==100); // Clamp at max.
    event(SCREEN_ENTER);
    for(int i=0;i<30;++i) event(SCREEN_PREVIOUS);
    result=event(SCREEN_ENTER); assert(result.value==-100); // Negative range and min.
    event(SCREEN_NEXT); event(SCREEN_ENTER); // Enter nested menu.
    event(SCREEN_ENTER); event(SCREEN_NEXT);
    result=event(SCREEN_BACK); assert(result.action==SCREEN_EDIT_CANCELLED);
    event(SCREEN_BACK); // Back from submenu to home.
    event(SCREEN_NEXT); event(SCREEN_ENTER); // Indicator.
    float duty=125;
    assert(ssd1306_screens_update(&ui,INDICATOR,&duty,1)==SSD1306_OK);
    failure=PICO_ERROR_TIMEOUT;
    assert(ssd1306_screens_show(&ui)==SSD1306_ERR_TIMEOUT);
    failure=0; transactions=0;
    assert(ssd1306_screens_show(&ui)==SSD1306_OK && transactions>0); // Retry unchanged.
    transactions=0;
    assert(ssd1306_screens_show(&ui)==SSD1306_OK && !transactions);
    ssd1306_screens_invalidate(&ui);
    assert(ssd1306_screens_show(&ui)==SSD1306_OK && transactions>0);
    event(SCREEN_BACK); event(SCREEN_NEXT); event(SCREEN_ENTER); // Message.
    assert(ssd1306_screens_show(&ui)==SSD1306_OK);
    assert(storage[0]==0xA5 && storage[1025]==0x5A);
    event(SCREEN_BACK); event(SCREEN_PREVIOUS); event(SCREEN_PREVIOUS);
    event(SCREEN_ENTER); event(SCREEN_NEXT); // Nested menu recursive destination.
    for(unsigned i=1;i<SSD1306_SCREENS_HISTORY;++i) event(SCREEN_ENTER);
    assert(ssd1306_screens_input(&ui,SCREEN_ENTER,NULL)==SSD1306_ERR_STATE);
    // Malformed definitions are rejected before replacing a valid UI.
    screen_page_t bad[6]; memcpy(bad,pages,sizeof(bad)); bad[EDIT].content.editor.step=0;
    assert(ssd1306_screens_init(&ui,&display,bad,6,HOME)==SSD1306_ERR_ARGUMENT);
    memcpy(bad,pages,sizeof(bad)); bad[EDIT].content.editor.min=NAN;
    assert(ssd1306_screens_init(&ui,&display,bad,6,HOME)==SSD1306_ERR_ARGUMENT);
    puts("PASS: paging, nested navigation, confirm/cancel, limits, unchanged frames, timeout retry and bounds");
    return 0;
}
