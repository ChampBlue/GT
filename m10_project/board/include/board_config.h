// P06
// 구버전 pcb -> 리셋 21, 27

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

//PINS
#define LED             25

#define LIMIT_DOWN      6  
#define LIMIT_UP        5  

#define MNQ1            10
#define MNQ2            11
#define MNQ3            12
#define MNQ_DIR         19
#define MNQ_PWM         20

#define ADC_PIN         28
#define ADC_CHANNEL     2

#define R_ENABLE        18
#define R_DIR           17
#define R_PWM           16

#define L_ENABLE        13
#define L_DIR           14
#define L_PWM           15

#define R_ENCODER_A     8
#define R_ENCODER_B     9

#define L_ENCODER_A     2
#define L_ENCODER_B     3

#define L_ALARM_RESET   21 
#define R_ALARM_RESET   27

#define UART_TX         0
#define UART_RX         1

#define PICO_CLK_KHZ    125000
#define PICO_CLK        125000000

//RULES
#define MOTOR_ENABLE    0
#define MOTOR_DISABLE   1

#define MNQ_UP          0
#define MNQ_DOWN        1

#endif