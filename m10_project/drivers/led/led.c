#include "led.h"
#include "board_config.h"
#include "hardware/gpio.h"

void init_led(void) {
    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);
}

void start_signal(void) {
    for (int i=0; i<4; i++) {
        gpio_put(LED, 1);
        sleep_ms(500);
        gpio_put(LED, 0);
        sleep_ms(500);
    }
}