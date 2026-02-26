#include "pico/stdio.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "board_config.h"
#include "board_init.h"
#include "led.h"
#include "motor.h"
#include "uart.h"

void init_board(void) {
    stdio_init_all();
    busy_wait_ms(10);

    set_sys_clock_khz(PICO_CLK_KHZ, true);
    busy_wait_ms(10);

    init_led();
    busy_wait_ms(10);

    init_motor();
    busy_wait_ms(10);

    init_uart();
    busy_wait_ms(10);
}