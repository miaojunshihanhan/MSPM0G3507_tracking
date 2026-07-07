/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "oled.h"
#include "board.h"
#include "motor.h"
#include "car.h"
#include "encoder.h"
#include "tracking.h"

volatile int32_t speedA = 0;
volatile int32_t speedB = 0;
volatile int8_t sensors;

int main()
{
	board_init();
	OLED_Init();
	OLED_Clear();

	NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
	NVIC_EnableIRQ(ENCODERA_INT_IRQN);
	NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
	NVIC_EnableIRQ(ENCODERB_INT_IRQN);

	NVIC_ClearPendingIRQ(TIMA0_INT_IRQn);
	NVIC_EnableIRQ(TIMA0_INT_IRQn);

	while(1)
	{
		OLED_ShowString(0, 0, (uint8_t *)"A:", 8, 1);
		if(speedA >= 0)
		{
			OLED_ShowChar(12, 0, '+', 8, 1);
			OLED_ShowNum(18, 0, speedA, 4, 8, 1);
		}
		else
		{
			OLED_ShowChar(12, 0, '-', 8, 1);
			OLED_ShowNum(18, 0, -speedA, 4, 8, 1);
		}
		OLED_ShowString(0, 10, (uint8_t *)"B:", 8, 1);
		if(speedB >= 0)
		{
			OLED_ShowChar(12, 10, '+', 8, 1);
			OLED_ShowNum(18, 10, speedB, 4, 8, 1);
		}
		else
		{
			OLED_ShowChar(12, 10, '-', 8, 1);
			OLED_ShowNum(18, 10, -speedB, 4, 8, 1);
		}
		sensors=Tracking_ReadSensors();
		OLED_ShowBits(30,30, sensors, 1);
		OLED_Refresh();
		delay_ms(10);
		}
}

void TIMA0_IRQHandler(void)
{
	speedA = Get_Encoder_countA;
	Get_Encoder_countA = 0;
	speedB = Get_Encoder_countB;
	Get_Encoder_countB = 0;
}
