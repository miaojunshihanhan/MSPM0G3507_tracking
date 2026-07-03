#ifndef _CAR_H
#define _CAR_H

#include "ti_msp_dl_config.h"

typedef struct
{
	uint8_t initialEnabled;
	int32_t baseSpeed;
	int32_t trackingOutputMax;
	int32_t cornerSpeedScale;
	int32_t basePwm;
	int32_t pwmMax;
	int32_t trackingWeights[4];
	float trackingKp;
	float trackingKi;
	float trackingKd;
	float speedKp;
	float speedKi;
	float speedKd;
} Car_Config;

typedef struct
{
	uint8_t enabled;
	int32_t targetSpeedA;
	int32_t targetSpeedB;
	int32_t actualSpeedA;
	int32_t actualSpeedB;
	int32_t pwmA;
	int32_t pwmB;
} Car_Status;

void Car_Init(const Car_Config *config);
void Car_Toggle(void);
void Car_SetEnabled(uint8_t enabled);
uint8_t Car_IsEnabled(void);
void Car_Forward(int32_t pwmA, int32_t pwmB);
void Car_Backward(int32_t pwmA, int32_t pwmB);
void Car_Stop(void);
void Car_TurnLeft(int32_t pwm);
void Car_TurnRight(int32_t pwm);
void Car_SpinLeft(int32_t pwm);
void Car_SpinRight(int32_t pwm);
void Car_ControlUpdate5ms(void);
void Car_GetStatus(Car_Status *status);

#endif
