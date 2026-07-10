#include "board.h"
#include "stdio.h"
#include "oled.h"
#include "key.h"
#include "tracking.h"

volatile uint32_t timer_ms = 0;
volatile uint32_t task_1ms_pending = 0;
volatile uint8_t uart_data = 0;

static void OLED_DisplayEncoderHeader(void);
static void OLED_DisplayEncoderValues(int32_t encoder_a, int32_t encoder_b);
static void OLED_ShowInt32(u8 x, u8 y, int32_t num, u8 len, u8 size1, u8 mode);

int main()
{
	board_init();
	OLED_Init();
	OLED_DisplayEncoderHeader();
	OLED_DisplayEncoderValues(Get_Encoder_countA, Get_Encoder_countB);
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
		if(task_1ms_pending > 0u)
		{
			__disable_irq();
			task_1ms_pending--;
			__enable_irq();
		}
		Tracking_Task();
	}
}

static void OLED_DisplayEncoderHeader(void)
{
	OLED_Clear();
	OLED_ShowString(0, 0, (u8 *)"ENCODER COUNT", 8, 1);
	OLED_ShowString(0, 16, (u8 *)"A:", 16, 1);
	OLED_ShowString(0, 40, (u8 *)"B:", 16, 1);
	OLED_Refresh();
}

static void OLED_DisplayEncoderValues(int32_t encoder_a, int32_t encoder_b)
{
	OLED_ShowInt32(24, 16, encoder_a, 10, 16, 1);
	OLED_ShowInt32(24, 40, encoder_b, 10, 16, 1);
	OLED_Refresh();
}

static void OLED_ShowInt32(u8 x, u8 y, int32_t num, u8 len, u8 size1, u8 mode)
{
	uint32_t magnitude;

	if(num < 0)
	{
		OLED_ShowChar(x, y, '-', size1, mode);
		magnitude = (uint32_t)(-(num + 1)) + 1u;
	}
	else
	{
		OLED_ShowChar(x, y, '+', size1, mode);
		magnitude = (uint32_t)num;
	}

	OLED_ShowNum((u8)(x + (size1 / 2u)), y, magnitude, len, size1, mode);
}

void TIMA0_IRQHandler(void)
{
	if (DL_TimerA_getEnabledInterruptStatus(TIMA0, DL_TIMERA_INTERRUPT_ZERO_EVENT))
	{
		DL_TimerA_clearInterruptStatus(TIMA0, DL_TIMERA_INTERRUPT_ZERO_EVENT);
		task_1ms_pending++;
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

