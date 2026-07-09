#include "board.h"
#include "stdio.h"

volatile uint32_t timer_ms = 0;
volatile int32_t speedA = 0;
volatile int32_t speedB = 0;
volatile uint8_t uart_data = 0;

int main()
{
	board_init();

	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

	NVIC_ClearPendingIRQ(TIMA0_INT_IRQn);
	NVIC_EnableIRQ(TIMA0_INT_IRQn);

	printf("MSPM0G3507 UART Test\r\n");
	printf("System Clock: 4MHz\r\n");
	printf("UART Baudrate: 9600\r\n");
	printf("LED will blink every 500ms\r\n");
	printf("Send any character to echo back\r\n");

	while(1)
	{
	}
}

void TIMA0_IRQHandler(void)
{
	if (DL_TimerA_getEnabledInterruptStatus(TIMA0, DL_TIMERA_INTERRUPT_ZERO_EVENT))
	{
		DL_TimerA_clearInterruptStatus(TIMA0, DL_TIMERA_INTERRUPT_ZERO_EVENT);
		timer_ms++;
		if (timer_ms >= 500)
		{
			timer_ms = 0;
			DL_GPIO_togglePins(LED_PORT, LED_PIN_0_PIN);
		}
	}
}

