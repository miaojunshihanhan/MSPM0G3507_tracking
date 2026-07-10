#include "board.h"
#include "stdio.h"
#include "key.h"
#include "tracking.h"
#include "status_display.h"

volatile uint32_t timer_ms = 0;

int main()
{
	board_init();
	StatusDisplay_Init();
	Tracking_Init();

	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

	NVIC_ClearPendingIRQ(TIMA0_INT_IRQn);
	NVIC_EnableIRQ(TIMA0_INT_IRQn);

	printf("MSPM0G3507 UART Test\r\n");
	printf("System Clock: 32MHz\r\n");
	printf("UART Baudrate: 115200\r\n");
	printf("LED will blink every 500ms\r\n");
	printf("VOFA: targetL,targetR,actualL,actualR,pwmL,pwmR,lineErr,turn,sensors,lineMask\r\n");

	while(1)
	{
		Tracking_Task();
	}
}

void TIMA0_IRQHandler(void)
{
	if (DL_TimerA_getEnabledInterruptStatus(TIMA0, DL_TIMERA_INTERRUPT_ZERO_EVENT))
	{
		DL_TimerA_clearInterruptStatus(TIMA0, DL_TIMERA_INTERRUPT_ZERO_EVENT);
		Key_Task_1ms();
		Tracking_Timer1ms_ISR();
		timer_ms++;
		if (timer_ms >= 500)
		{
			timer_ms = 0;
			DL_GPIO_togglePins(LED_PORT, LED_PIN_0_PIN);
		}
	}
}

