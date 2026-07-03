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
#include "stdio.h"
#include "key.h"
#include "motor.h"
#include "oled.h"
#include "tracking.h"

/*========================== 可调参数 ==========================*/
#define CAR_INITIAL_ENABLED          0U      // 上电初始状态：0=停止，1=自动运行
#define CAR_BASE_SPEED               6       // 小车基础速度：编码器计数/5ms
#define TRACKING_WEIGHT_S1           (-5)    // 最左侧灰度传感器权重
#define TRACKING_WEIGHT_S2           (-2)    // 左中侧灰度传感器权重
#define TRACKING_WEIGHT_S3           2       // 右中侧灰度传感器权重
#define TRACKING_WEIGHT_S4           5       // 最右侧灰度传感器权重
#define TRACKING_KP                  8.0f    // 循迹PID比例系数
#define TRACKING_KI                  0.0f    // 循迹PID积分系数
#define TRACKING_KD                  0.0f    // 循迹PID微分系数
#define SPEED_KP                     0.0f   // 速度PID比例系数
#define SPEED_KI                     0.0f    // 速度PID积分系数
#define SPEED_KD                     0.0f    // 速度PID微分系数
/*==================================================================*/

volatile int32_t Get_Encoder_countA, encoderA_cnt;
volatile int32_t Get_Encoder_countB, encoderB_cnt;
volatile int32_t TargetVelocityA, TargetVelocityB;
volatile int32_t PWMA, PWMB;
volatile uint8_t CarEnabled = CAR_INITIAL_ENABLED;
volatile unsigned int delay_times = 0;

static int32_t ClampTargetSpeed(int32_t speed)
{
	if(speed > CAR_BASE_SPEED * 4) return CAR_BASE_SPEED * 4;
	if(speed < 0) return 0;
	return speed;
}

// 每5 ms执行一次：循迹外环分配目标速度，速度内环计算两轮PWM
static void Cascade_Control_Update(void)
{
	int32_t trackingCorrection;
	int32_t speedCorrection;

	Key_Scan_5ms();

	// 编码器只用于OLED显示，不参与电机控制
	encoderA_cnt = Get_Encoder_countA;
	encoderB_cnt = Get_Encoder_countB;
	Get_Encoder_countA = 0;
	Get_Encoder_countB = 0;

	if(!CarEnabled)
	{
		Tracking_PID_Reset();
		Motor_PID_Reset();
		TargetVelocityA = 0;
		TargetVelocityB = 0;
		PWMA = 0;
		PWMB = 0;
		Set_PWM(0, 0);
		return;
	}

	trackingCorrection = Tracking_PID_Calculate(20);
	if(trackingCorrection >= 20)
	{
		TargetVelocityA = CAR_BASE_SPEED * 4;
		TargetVelocityB = 0;
	}
	else if(trackingCorrection <= -20)
	{
		TargetVelocityA = 0;
		TargetVelocityB = CAR_BASE_SPEED * 4;
	}
	else
	{
		speedCorrection = trackingCorrection * CAR_BASE_SPEED / 20;
		TargetVelocityA = ClampTargetSpeed(CAR_BASE_SPEED + speedCorrection);
		TargetVelocityB = ClampTargetSpeed(CAR_BASE_SPEED - speedCorrection);
	}
	PWMA = Velocity_A(TargetVelocityA, ABS(encoderA_cnt));
	PWMB = Velocity_B(TargetVelocityB, ABS(encoderB_cnt));
	Set_PWM(PWMA, PWMB);
}

int main(void)
{
	int32_t currentPwmA, currentPwmB;
	int32_t targetSpeed;
	int32_t speedA, speedB;
	uint8_t trackingSensors;

	Tracking_Kp = TRACKING_KP;
	Tracking_Ki = TRACKING_KI;
	Tracking_Kd = TRACKING_KD;
	Tracking_Weights[0] = TRACKING_WEIGHT_S1;
	Tracking_Weights[1] = TRACKING_WEIGHT_S2;
	Tracking_Weights[2] = TRACKING_WEIGHT_S3;
	Tracking_Weights[3] = TRACKING_WEIGHT_S4;
	Motor_PID_SetParameters(SPEED_KP, SPEED_KI, SPEED_KD);
	Motor_PID_SetBase(500, CAR_BASE_SPEED);
	Motor_SetPWMMax(3000);

	board_init();
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
			CarEnabled = !CarEnabled;
			Tracking_PID_Reset();
			Motor_PID_Reset();
		}

		currentPwmA = PWMA;
		currentPwmB = PWMB;
		targetSpeed = CarEnabled ? CAR_BASE_SPEED : 0;
		speedA = ABS(encoderA_cnt);
		speedB = ABS(encoderB_cnt);
		trackingSensors = Tracking_ReadSensors();

		OLED_ShowString(0, 0, (uint8_t *)"RUN:", 8, 1);
		OLED_ShowNum(24, 0, CarEnabled, 1, 8, 1);
		OLED_ShowString(36, 0, (uint8_t *)"SPD:", 8, 1);
		OLED_ShowNum(60, 0, targetSpeed, 3, 8, 1);
		OLED_ShowString(84, 0, (uint8_t *)"S:", 8, 1);
		OLED_ShowNum(96, 0, (trackingSensors >> 0) & 0x01, 1, 8, 1);
		OLED_ShowNum(102, 0, (trackingSensors >> 1) & 0x01, 1, 8, 1);
		OLED_ShowNum(108, 0, (trackingSensors >> 2) & 0x01, 1, 8, 1);
		OLED_ShowNum(114, 0, (trackingSensors >> 3) & 0x01, 1, 8, 1);
		OLED_ShowString(0, 16, (uint8_t *)"A ACT:", 8, 1);
		OLED_ShowNum(36, 16, speedA, 3, 8, 1);
		OLED_ShowString(0, 32, (uint8_t *)"B ACT:", 8, 1);
		OLED_ShowNum(36, 32, speedB, 3, 8, 1);
		OLED_ShowString(0, 48, (uint8_t *)"PWM A:", 8, 1);
		OLED_ShowNum(36, 48, currentPwmA, 4, 8, 1);
		OLED_ShowString(66, 48, (uint8_t *)"B:", 8, 1);
		OLED_ShowNum(78, 48, currentPwmB, 4, 8, 1);
        OLED_Refresh();
        delay_ms(10);
    }
}



void TIMER_0_INST_IRQHandler(void)
{
	switch(DL_TimerA_getPendingInterrupt(TIMER_0_INST))
	{
		case DL_TIMER_IIDX_ZERO:
			Cascade_Control_Update();
			break;
		default:
			break;
	}
}

