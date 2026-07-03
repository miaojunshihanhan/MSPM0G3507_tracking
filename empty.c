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
#include "board.h"
#include "car.h"
#include "key.h"
#include "oled.h"
#include "tracking.h"

/*========================== 小车可调参数 ==========================*/
#define CAR_INITIAL_ENABLED          0U      // 上电状态：0=停止，1=自动循迹
#define CAR_BASE_SPEED               6       // 直行目标速度，单位：编码器计数/5ms
#define CAR_TRACKING_OUTPUT_MAX      20      // 循迹PID输出限幅，需大于0
#define CAR_CORNER_SPEED_SCALE       10      // 大转弯时外侧轮速度倍数，越大速度差越大
#define CAR_BASE_PWM                 500     // 基础前馈PWM，速度PID为0时主要由它驱动
#define CAR_PWM_MAX                  3200    // 电机最大PWM，范围建议为0~3200

#define TRACKING_WEIGHT_S1           (-6)    // 最左侧探头权重
#define TRACKING_WEIGHT_S2           (-3)    // 左中探头权重
#define TRACKING_WEIGHT_S3           3       // 右中探头权重
#define TRACKING_WEIGHT_S4           6       // 最右侧探头权重
#define TRACKING_KP                  10.0f    // 循迹比例：越大，偏离黑线时转向越猛
#define TRACKING_KI                  0.0f     // 循迹积分：消除长期偏差，过大会左右摆动
#define TRACKING_KD                  0.0f     // 循迹微分：抑制转向变化过快

#define SPEED_KP                     0.0f     // 速度比例：轮速偏差越大，修正越强
#define SPEED_KI                     0.0f     // 速度积分：消除长期轮速误差，过大会振荡
#define SPEED_KD                     0.0f     // 速度微分：抑制轮速突变，一般先保持为0
/*==================================================================*/

static const Car_Config CarConfig =
{
	CAR_INITIAL_ENABLED,
	CAR_BASE_SPEED,
	CAR_TRACKING_OUTPUT_MAX,
	CAR_CORNER_SPEED_SCALE,
	CAR_BASE_PWM,
	CAR_PWM_MAX,
	{TRACKING_WEIGHT_S1, TRACKING_WEIGHT_S2,
		TRACKING_WEIGHT_S3, TRACKING_WEIGHT_S4},
	TRACKING_KP,
	TRACKING_KI,
	TRACKING_KD,
	SPEED_KP,
	SPEED_KI,
	SPEED_KD
};

int main(void)
{
	int32_t targetSpeed;
	uint8_t trackingSensors;
	Car_Status carStatus;

	board_init();
	Car_Init(&CarConfig);
	OLED_Init();
	OLED_Clear();
    DL_Timer_startCounter(PWM_0_INST);
	NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
	NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
	NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
	// 使能定时器和两路编码器GPIO中断
	NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
	NVIC_EnableIRQ(ENCODERA_INT_IRQN);
	NVIC_EnableIRQ(ENCODERB_INT_IRQN);
    while (1) 
    {
		if(click())
		{
			Car_Toggle();//小车开关
		}

		Car_GetStatus(&carStatus);//获取小车状态
		targetSpeed = carStatus.enabled ? CAR_BASE_SPEED : 0;
		trackingSensors = Tracking_ReadSensors();

		OLED_ShowString(0, 0, (uint8_t *)"RUN:", 8, 1);
		OLED_ShowNum(24, 0, carStatus.enabled, 1, 8, 1);
		OLED_ShowString(36, 0, (uint8_t *)"SPD:", 8, 1);
		OLED_ShowNum(60, 0, targetSpeed, 3, 8, 1);
		OLED_ShowString(84, 0, (uint8_t *)"S:", 8, 1);
		OLED_ShowNum(96, 0, (trackingSensors >> 0) & 0x01, 1, 8, 1);
		OLED_ShowNum(102, 0, (trackingSensors >> 1) & 0x01, 1, 8, 1);
		OLED_ShowNum(108, 0, (trackingSensors >> 2) & 0x01, 1, 8, 1);
		OLED_ShowNum(114, 0, (trackingSensors >> 3) & 0x01, 1, 8, 1);
		OLED_ShowString(0, 16, (uint8_t *)"A ACT:", 8, 1);
		OLED_ShowNum(36, 16, carStatus.actualSpeedA, 3, 8, 1);
		OLED_ShowString(0, 32, (uint8_t *)"B ACT:", 8, 1);
		OLED_ShowNum(36, 32, carStatus.actualSpeedB, 3, 8, 1);
		OLED_ShowString(0, 48, (uint8_t *)"PWM A:", 8, 1);
		OLED_ShowNum(36, 48, carStatus.pwmA, 4, 8, 1);
		OLED_ShowString(66, 48, (uint8_t *)"B:", 8, 1);
		OLED_ShowNum(78, 48, carStatus.pwmB, 4, 8, 1);
        OLED_Refresh();
        delay_ms(10);
    }
}



void TIMER_0_INST_IRQHandler(void)
{
	switch(DL_TimerA_getPendingInterrupt(TIMER_0_INST))
	{
		case DL_TIMER_IIDX_ZERO:
			Key_Scan_5ms();
			Car_ControlUpdate5ms();
			break;
		default:
			break;
	}
}

