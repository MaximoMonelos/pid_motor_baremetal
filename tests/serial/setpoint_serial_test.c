#include "setpoint_serial.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static const unsigned char *input;
static size_t available, consumed;

int getchar_timeout_us(uint32_t timeout_us)
{
    assert(timeout_us == 0);
    if (consumed == available) return PICO_ERROR_TIMEOUT;
    return input[consumed++];
}

static void feed_bytes(setpoint_gen_t *sp, const void *bytes, size_t count)
{
    input = bytes;
    available = count;
    consumed = 0;
    while (consumed < available) {
        size_t before = consumed;
        setpoint_serial_update(sp);
        assert(consumed == before + 1);
    }
}

static void feed(setpoint_gen_t *sp, const char *text)
{
    feed_bytes(sp, text, strlen(text));
}

int main(void)
{
    setpoint_gen_t sp;
    setpoint_gen_config_t config = {
        .mode = SETPOINT_MODE_RAMP, .target_value = 500, .ramp_rate = 50
    };
    setpoint_gen_init(&sp, &config);
    feed(&sp, "15");
    assert(sp.config.target_value == 500); // No Enter yet.
    setpoint_serial_update(&sp); // Empty poll preserves partial input.
    feed(&sp, "00\r\n");
    assert(sp.config.target_value == 1500 && sp.current_value == 0);
    feed(&sp, "  650.5 \t\n");
    assert(sp.config.target_value == 650.5f);
    const char *invalid[] = {"hola\n", "1500abc\n", "nan\n", "inf\n",
        "1e99\n", "-20\n", "   \n", "\r\n"};
    for (size_t i = 0; i < sizeof(invalid)/sizeof(invalid[0]); i++) {
        feed(&sp, invalid[i]);
        assert(sp.config.target_value == 650.5f);
    }
    feed(&sp, "1234567890123456789012345678901234567890\n");
    assert(sp.config.target_value == 650.5f); // Discard entire oversized line.
    const unsigned char binary[] = {'1','5','0','0',0,'1','\n'};
    feed_bytes(&sp, binary, sizeof(binary));
    assert(sp.config.target_value == 650.5f);
    feed(&sp, "750\n0\n"); assert(sp.config.target_value == 0);
    feed(&sp, "1200\n"); assert(sp.config.target_value == 1200); // Above 500 allowed.
    sp.arrived = true;
    feed(&sp, "\r\n"); assert(sp.arrived); // CRLF does not reapply the last target.
    setpoint_serial_update(NULL);
    puts("PASS: partial lines, decimals, invalid input, oversized lines, bounded polling and zero");
    return 0;
}
