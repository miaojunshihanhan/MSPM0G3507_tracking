#include "car.h"
#include "board.h"
#include "motor.h"
#include "tracking.h"

volatile int32_t Get_Encoder_countA;
volatile int32_t Get_Encoder_countB;

static volatile uint8_t CarEnabled;
static volatile int32_t BaseSpeed;
static int32_t TrackingOutputMax;
static int32_t CornerSpeedScale;
static int32_t PwmMax;
static volatile int32_t TargetSpeedA;
static volatile int32_t TargetSpeedB;
static volatile int32_t ActualSpeedA;
static volatile int32_t ActualSpeedB;
static volatile int32_t PwmA;
static volatile int32_t PwmB;

static int32_t Car_LimitPWM(int32_t pwm)
{
	if(pwm > PwmMax) return PwmMax;
	if(pwm < 0) return 0;
	return pwm;
}

static int32_t Car_ClampTargetSpeed(int32_t speed)
{
	int32_t maxSpeed = BaseSpeed * CornerSpeedScale;

	if(speed > maxSpeed) return maxSpeed;
	if(speed < 0) return 0;
	return speed;
}

void Car_Init(const Car_Config *config)
{
	uint8_t i;

	CarEnabled = config->initialEnabled;
	BaseSpeed = config->baseSpeed;
	TrackingOutputMax = config->trackingOutputMax;
	CornerSpeedScale = config->cornerSpeedScale;
	PwmMax = config->pwmMax;

	Tracking_Kp = config->trackingKp;
	Tracking_Ki = config->trackingKi;
	Tracking_Kd = config->trackingKd;
	for(i = 0; i < 4; i++)
	{
		Tracking_Weights[i] = config->trackingWeights[i];
	}

	Motor_PID_SetParameters(config->speedKp, config->speedKi, config->speedKd);
	Motor_PID_SetBase(config->basePwm, BaseSpeed);
	Motor_SetPWMMax(PwmMax);
	Motor_PID_Reset();
	Tracking_PID_Reset();
	Car_Stop();
}

void Car_Toggle(void)
{
	Car_SetEnabled(!CarEnabled);
}

void Car_SetEnabled(uint8_t enabled)
{
	CarEnabled = enabled ? 1U : 0U;
	Motor_PID_Reset();
	Tracking_PID_Reset();
	if(!CarEnabled)
	{
		Car_Stop();
	}
}

uint8_t Car_IsEnabled(void)
{
	return CarEnabled;
}

void Car_Forward(int32_t pwmA, int32_t pwmB)
{
	PwmA = Car_LimitPWM(pwmA);
	PwmB = Car_LimitPWM(pwmB);
	Set_PWM(PwmA, PwmB);
}

void Car_Backward(int32_t pwmA, int32_t pwmB)
{
	PwmA = -Car_LimitPWM(pwmA);
	PwmB = -Car_LimitPWM(pwmB);
	Set_PWM(PwmA, PwmB);
}

void Car_Stop(void)
{
	PwmA = 0;
	PwmB = 0;
	Set_PWM(0, 0);
}

void Car_TurnLeft(int32_t pwm)
{
	Car_Forward(0, pwm);
}

void Car_TurnRight(int32_t pwm)
{
	Car_Forward(pwm, 0);
}

void Car_SpinLeft(int32_t pwm)
{
	PwmA = -Car_LimitPWM(pwm);
	PwmB = Car_LimitPWM(pwm);
	Set_PWM(PwmA, PwmB);
}

void Car_SpinRight(int32_t pwm)
{
	PwmA = Car_LimitPWM(pwm);
	PwmB = -Car_LimitPWM(pwm);
	Set_PWM(PwmA, PwmB);
}

void Car_ControlUpdate5ms(void)
{
	int32_t trackingCorrection;
	int32_t speedCorrection;

	ActualSpeedA = ABS(Get_Encoder_countA);
	ActualSpeedB = ABS(Get_Encoder_countB);
	Get_Encoder_countA = 0;
	Get_Encoder_countB = 0;

	if(!CarEnabled)
	{
		Tracking_PID_Reset();
		Motor_PID_Reset();
		TargetSpeedA = 0;
		TargetSpeedB = 0;
		Car_Stop();
		return;
	}

	trackingCorrection = Tracking_PID_Calculate(TrackingOutputMax);
	if(trackingCorrection >= TrackingOutputMax)
	{
		TargetSpeedA = BaseSpeed * CornerSpeedScale;
		TargetSpeedB = 0;
	}
	else if(trackingCorrection <= -TrackingOutputMax)
	{
		TargetSpeedA = 0;
		TargetSpeedB = BaseSpeed * CornerSpeedScale;
	}
	else
	{
		speedCorrection = trackingCorrection * BaseSpeed
			/ TrackingOutputMax;
		TargetSpeedA = Car_ClampTargetSpeed(BaseSpeed + speedCorrection);
		TargetSpeedB = Car_ClampTargetSpeed(BaseSpeed - speedCorrection);
	}

	PwmA = Velocity_A(TargetSpeedA, ActualSpeedA);
	PwmB = Velocity_B(TargetSpeedB, ActualSpeedB);
	Car_Forward(PwmA, PwmB);
}

void Car_GetStatus(Car_Status *status)
{
	status->enabled = CarEnabled;
	status->targetSpeedA = TargetSpeedA;
	status->targetSpeedB = TargetSpeedB;
	status->actualSpeedA = ActualSpeedA;
	status->actualSpeedB = ActualSpeedB;
	status->pwmA = PwmA;
	status->pwmB = PwmB;
}
