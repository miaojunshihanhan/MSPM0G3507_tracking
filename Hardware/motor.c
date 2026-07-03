#include "motor.h"
#define MOTOR_PWM_MAX 3200
#define VELOCITY_INTEGRAL_LIMIT 300

static float VelocityKp;
static float VelocityKi;
static float VelocityKd;
static float VelocityFeedForwardGain;
static int ControlVelocityA, Sum_biasA, Last_biasA;
static int ControlVelocityB, Sum_biasB, Last_biasB;
static int MotorPwmLimit = MOTOR_PWM_MAX;
/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2024-07-019

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2024-07-019

All rights reserved
***********************************************/
/***************************************************************************
函数功能：电机的PID闭环控制
入口参数：左右电机的编码器值
返回值  ：电机的PWM
***************************************************************************/
int Velocity_A(int TargetVelocity, int CurrentVelocity)
{
	int Bias;

	if(TargetVelocity <= 0)
	{
		ControlVelocityA = 0;
		Sum_biasA = 0;
		Last_biasA = 0;
		return 0;
	}

	Bias = TargetVelocity - CurrentVelocity;
	Sum_biasA += Bias;
	if(Sum_biasA > VELOCITY_INTEGRAL_LIMIT) Sum_biasA = VELOCITY_INTEGRAL_LIMIT;
	else if(Sum_biasA < -VELOCITY_INTEGRAL_LIMIT) Sum_biasA = -VELOCITY_INTEGRAL_LIMIT;
	ControlVelocityA = VelocityFeedForwardGain * TargetVelocity
		+ VelocityKp * Bias
		+ VelocityKi * Sum_biasA
		+ VelocityKd * (Bias - Last_biasA);
	Last_biasA = Bias;

	if(ControlVelocityA > MotorPwmLimit) ControlVelocityA = MotorPwmLimit;
	else if(ControlVelocityA < 0) ControlVelocityA = 0;
	return ControlVelocityA;
}

/***************************************************************************
函数功能：电机的PID闭环控制
入口参数：左右电机的编码器值
返回值  ：电机的PWM
***************************************************************************/
int Velocity_B(int TargetVelocity, int CurrentVelocity)
{
	int Bias;

	if(TargetVelocity <= 0)
	{
		ControlVelocityB = 0;
		Sum_biasB = 0;
		Last_biasB = 0;
		return 0;
	}

	Bias = TargetVelocity - CurrentVelocity;
	Sum_biasB += Bias;
	if(Sum_biasB > VELOCITY_INTEGRAL_LIMIT) Sum_biasB = VELOCITY_INTEGRAL_LIMIT;
	else if(Sum_biasB < -VELOCITY_INTEGRAL_LIMIT) Sum_biasB = -VELOCITY_INTEGRAL_LIMIT;
	ControlVelocityB = VelocityFeedForwardGain * TargetVelocity
		+ VelocityKp * Bias
		+ VelocityKi * Sum_biasB
		+ VelocityKd * (Bias - Last_biasB);
	Last_biasB = Bias;

	if(ControlVelocityB > MotorPwmLimit) ControlVelocityB = MotorPwmLimit;
	else if(ControlVelocityB < 0) ControlVelocityB = 0;
	return ControlVelocityB;
}

void Motor_PID_SetParameters(float kp, float ki, float kd)
{
	VelocityKp = kp;
	VelocityKi = ki;
	VelocityKd = kd;
}

void Motor_PID_SetBase(int basePwm, int baseVelocity)
{
	if(baseVelocity > 0)
	{
		VelocityFeedForwardGain = basePwm / (float)baseVelocity;
	}
	else
	{
		VelocityFeedForwardGain = 0.0f;
	}
}

void Motor_PID_Reset(void)
{
	ControlVelocityA = 0;
	Sum_biasA = 0;
	Last_biasA = 0;
	ControlVelocityB = 0;
	Sum_biasB = 0;
	Last_biasB = 0;
}

void Motor_SetPWMMax(int pwmMax)
{
	if(pwmMax < 0) pwmMax = 0;
	else if(pwmMax > MOTOR_PWM_MAX) pwmMax = MOTOR_PWM_MAX;
	MotorPwmLimit = pwmMax;

	if(ControlVelocityA > MotorPwmLimit) ControlVelocityA = MotorPwmLimit;
	else if(ControlVelocityA < -MotorPwmLimit) ControlVelocityA = -MotorPwmLimit;
	if(ControlVelocityB > MotorPwmLimit) ControlVelocityB = MotorPwmLimit;
	else if(ControlVelocityB < -MotorPwmLimit) ControlVelocityB = -MotorPwmLimit;
}

void Set_PWM(int pwma,int pwmb)
{
	// D153C接线：PA12/PA13控制A路方向，PB0输出A路PWM；
	// PA14/PA15控制B路方向，PB1输出B路PWM
	if(pwma>0) 
	{
		DL_GPIO_setPins(AIN2_PORT,AIN2_PIN_13_PIN);
		DL_GPIO_clearPins(AIN1_PORT,AIN1_PIN_12_PIN);
		DL_Timer_setCaptureCompareValue(PWM_0_INST,ABS(pwma),GPIO_PWM_0_C0_IDX);
	}
	else
	{
		DL_GPIO_setPins(AIN1_PORT,AIN1_PIN_12_PIN);
		DL_GPIO_clearPins(AIN2_PORT,AIN2_PIN_13_PIN);
		DL_Timer_setCaptureCompareValue(PWM_0_INST,ABS(pwma),GPIO_PWM_0_C0_IDX);
	}
	if(pwmb>0)
	{
		DL_GPIO_setPins(BIN2_PORT,BIN2_Pin_Bin2_PIN);
		DL_GPIO_clearPins(BIN1_PORT,BIN1_Pin_Bin1_PIN);
		DL_Timer_setCaptureCompareValue(PWM_0_INST,ABS(pwmb),GPIO_PWM_0_C1_IDX);
	}
    else
	{
		DL_GPIO_setPins(BIN1_PORT,BIN1_Pin_Bin1_PIN);
		DL_GPIO_clearPins(BIN2_PORT,BIN2_Pin_Bin2_PIN);
		DL_Timer_setCaptureCompareValue(PWM_0_INST,ABS(pwmb),GPIO_PWM_0_C1_IDX);
	}		

}



