#include "board_init.h"
#include "pico/stdlib.h"

int main(void) {

    init_board();

    while (1) {
        tight_loop_contents();
    }

    return 0;
}