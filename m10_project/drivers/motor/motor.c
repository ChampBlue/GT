#include "board_config.h"
#include "motor.h"
#include "hardware/gpio.h"

void init_motor(void) {
    gpio_init(R_ENABLE);
    gpio_set_dir(R_ENABLE, GPIO_OUT);
    gpio_put(R_ENABLE, MOTOR_DISABLE);

    gpio_init(R_DIR);
    gpio_set_dir(R_DIR, GPIO_OUT);

    gpio_init(L_ENABLE);
    gpio_set_dir(L_ENABLE, GPIO_OUT);
    gpio_put(L_ENABLE, MOTOR_DISABLE);

    gpio_init(L_DIR);
    gpio_set_dir(L_DIR, GPIO_OUT);    
}