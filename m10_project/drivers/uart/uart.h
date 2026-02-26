#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/uart.h"

// IRQ = interrupt ReQuest : 인터럽트 요청 자체를 말함. 예) UART0_IRQ는 UART0가 인터럽트를 걸 때 들어오는 IRQ라인/번호라고 보면 됨
// -> 사건(요청/라인 번호)

// ISR = Interrupt Service Routine : IRQ가 발생했을 때 CPU가 실행하는 함수(핸들러) 예) uart_rx_isr() 같은 게 ISR
// -> 그 사건이 나면 실행되는 함수 

//NVIC = Nested Vectored Interrupt Controller : Cortex-M0+ 코어 안에 있는 인터럽트 컨트롤러/
// 어떤 IRQ가 어떤 ISR로 연결되는지, 우선순위는 뭔지, enable/disable 등을 관리함.

// FIFO = First in, First Out (하드웨어 버퍼)
// DMA = Direct Memory Access(CPU 개입없이 주변장치 <-> 메모리 전송)

// +++ uart_inst_t +++ //
//RP2040에는 UART가 2개 있음.
//Pico SDK는 uart0, uart1을 전역 포인터로 제공함. 
//내부적으로 uart_inst_t는 이 uart가 어떤 hw 베이스 주소(uart_hw_t*)를 쓰는지, 번호가 뭔지 같은 정보를 들고 있음.
// 즉, uart_inst_t *uart인자로 uart0, uart1을 넘기면 SDK함수들이 해당 uart 레지스터 블록에 접근해서 설정하는 구조.
void uart_drv_init(uart_inst_t *uart, uint tx_pin, uint rx_pin, uint baud);

uint32_t uart_drv_available(void);
uint32_t uart_drv_read(uint8_t *dst, uint32_t max_len);
bool uart_drv_overflowed_and_clear(void);

#endif