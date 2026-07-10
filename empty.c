#include "board.h"
#include "stdio.h"
#include "oled.h"
#include "key.h"

volatile uint32_t timer_ms = 0;
volatile uint32_t task_1ms_pending = 0;
volatile uint8_t encoder_display_pending = 0;
volatile int32_t speedA = 0;
volatile int32_t speedB = 0;
volatile uint8_t uart_data = 0;

#define ENCODER_DISPLAY_PERIOD_MS      (100u)

static void OLED_DisplayEncoderHeader(void);
static void OLED_DisplayEncoderValues(int32_t encoder_a, int32_t encoder_b);
static void OLED_EncoderDisplayTask(void);
static void OLED_ShowInt32(u8 x, u8 y, int32_t num, u8 len, u8 size1, u8 mode);

int main()
{
	board_init();
	OLED_Init();
	OLED_DisplayEncoderHeader();
	OLED_DisplayEncoderValues(Get_Encoder_countA, Get_Encoder_countB);

	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

	NVIC_ClearPendingIRQ(TIMA0_INT_IRQn);
	NVIC_EnableIRQ(TIMA0_INT_IRQn);

	printf("MSPM0G3507 UART Test\r\n");
	printf("System Clock: 4MHz\r\n");
	printf("UART Baudrate: 9600\r\n");
	printf("LED will blink every 500ms\r\n");
	printf("OLED displays encoder A/B every 100ms based on 1ms timer\r\n");
	printf("Send any character to echo back\r\n");

	while(1)
	{
		if(task_1ms_pending > 0u)
		{
			__disable_irq();
			task_1ms_pending--;
			__enable_irq();

			if(click())
			{
				printf("B21 pressed\r\n");
			}

			if(encoder_display_pending != 0u)
			{
				__disable_irq();
				encoder_display_pending = 0u;
				__enable_irq();
				OLED_EncoderDisplayTask();
			}
		}
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

static void OLED_EncoderDisplayTask(void)
{
	int32_t encoder_a;
	int32_t encoder_b;

	__disable_irq();
	encoder_a = Get_Encoder_countA;
	encoder_b = Get_Encoder_countB;
	__enable_irq();
	OLED_DisplayEncoderValues(encoder_a, encoder_b);
}

void TIMA0_IRQHandler(void)
{
	static uint16_t encoder_display_tick = 0u;

	if (DL_TimerA_getEnabledInterruptStatus(TIMA0, DL_TIMERA_INTERRUPT_ZERO_EVENT))
	{
		DL_TimerA_clearInterruptStatus(TIMA0, DL_TIMERA_INTERRUPT_ZERO_EVENT);
		task_1ms_pending++;
		Key_Task_1ms();
		encoder_display_tick++;
		if(encoder_display_tick >= ENCODER_DISPLAY_PERIOD_MS)
		{
			encoder_display_tick = 0u;
			encoder_display_pending = 1u;
		}
		timer_ms++;
		if (timer_ms >= 500)
		{
			timer_ms = 0;
			DL_GPIO_togglePins(LED_PORT, LED_PIN_0_PIN);
		}
	}
}

