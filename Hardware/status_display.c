#include "status_display.h"
#include "encoder.h"
#include "oled.h"
#include <stdint.h>

static void DisplayEncoderHeader(void);
static void DisplayEncoderValues(int32_t encoder_a, int32_t encoder_b);
static void DisplayInt32(u8 x, u8 y, int32_t num, u8 len, u8 size1, u8 mode);

void StatusDisplay_Init(void)
{
	OLED_Init();
	DisplayEncoderHeader();
	DisplayEncoderValues(Get_Encoder_countA, Get_Encoder_countB);
}

static void DisplayEncoderHeader(void)
{
	OLED_Clear();
	OLED_ShowString(0, 0, (u8 *)"ENCODER COUNT", 8, 1);
	OLED_ShowString(0, 16, (u8 *)"A:", 16, 1);
	OLED_ShowString(0, 40, (u8 *)"B:", 16, 1);
	OLED_Refresh();
}

static void DisplayEncoderValues(int32_t encoder_a, int32_t encoder_b)
{
	DisplayInt32(24, 16, encoder_a, 10, 16, 1);
	DisplayInt32(24, 40, encoder_b, 10, 16, 1);
	OLED_Refresh();
}

static void DisplayInt32(u8 x, u8 y, int32_t num, u8 len, u8 size1, u8 mode)
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
