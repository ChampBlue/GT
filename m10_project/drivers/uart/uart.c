#include "uart.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"

#define UART_RX_BUF_SIZE    512u
#define UART_RX_MASK        (UART_RX_BUF_SIZE - 1u)

static uart_inst_t *s_uart;
static volatile uint16_t s_head = 0;
static volatile uint16_t s_tail = 0;
static uint8_t s_buf[UART_RX_BUF_SIZE];
static volatile bool s_overflow = false;

static inline uint16_t next_idx(uint16_t v) {return (uint16_t)((v+1u) & UART_RX_MASK);}

static void uart_rx_isr(void) {
    while (uart_is_readable(s_uart)) {
        uint8_t c = (uint8_t)uart_getc(s_uart);
        uint16_t head = s_head;
        uint16_t n = next_idx(head);
    
        if (n==s_tail) {
            s_overflow = true;
            break;
        }

        s_buf[head] = c;
        s_head = n;
    }
}
void uart_drv_init(uart_inst_t *uart, uint tx_pin, uint rx_pin, uint baud){
    // 드라이버 내부 전역(static) 변수에 현재 사용할 UART 인스턴스 포인터를 저장함. 
    // 이유: ISR(uart_rx_isr)에서 어느 UART에서 읽어야하는지 알아야하는데, ISR 시그니처는 보통 인자를 못받음. 그래서 전역으로 잡아두는 방식임.
    s_uart = uart;

    //이 핀을 일반 GPIO가 아니라 UART의 TX/RX 신호가 나오는 핀이 됨.
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);

    // uart 기본 초기화
    uart_init(uart, baud);

    //hw flow contorl 끄기(RTS/CTS 미사용) 첫번째 FALSE : CTS / 두번째 FALSE : RTS
    uart_set_hw_flow(uart, false, false);

    //데이터 포멧 설정 : 8N1 = 8data, No parity, 1 stop -> pc시리얼이나 다른 mcu도 똑같이 맞춰줘야함.
    uart_set_format(uart, 8, 1, UART_PARITY_NONE);

    // FIFO enable (RX/TX FIFO켜기)
    // RP2040 UART에는 하드웨어 FIFO(버퍼)가 있음. 
    // FIFO OFF: RX에 바이트가 들어오면 거의 즉시 CPU가 읽어줘야함. (인터럽트 지연 생기면 오버런으로 유실 가능성 높아짐)
    // FIFO ON: 하드웨어가 여러 바이트를 잠깐 쌓아둠. cpu가 조금 늦어도 한꺼번에 읽어서 따라잡기가 쉬워짐.
    // FIFO를 켰으면, ISR에서 while(uart_is_readable())로 읽을 수 있을 만큼 다 읽기가 궁합이 좋음.
    uart_set_fifo_enabled(uart, true);

    //IRQ 번호 선택 : RP2040은 UART마다 IRQ라인이 따로 있음. UART0 -> UART0_IRQ, UART1 -> UART1_IRQ 
    // 넘어온 인스턴스가 uart0인지, uart1인지 보고 올바른 IRQ 번호 선택
    int irq = (uart = uart0) ? UART0_IRQ : UART1_IRQ;

    // 인터럽트 핸들러(ISR) 등록 : NVIC(인스터스 컨트롤러)에 이 IRQ가 발생하면 uart_rx_isr()함수를 실행하라라고 등록하는 동작
    // exclusive_handler라는 말은 이 IRQ에 대한 핸들러는 하나만(독점) 쓴다는 의미. 다른 코드가 같은 irq에 handler를 추가로 달 수 없음(충돌 방지)
    irq_set_exclusive_handler(irq, uart_rx_isr);

    //핸들러 등록후 끝이 아니라 nvic에서 해당 IRQ를 실제로 켜줘야 인터럽트가 들어옴.
    irq_set_enabled(irq, true);

    uart_set_irq_enables(uart, true, false);
}
