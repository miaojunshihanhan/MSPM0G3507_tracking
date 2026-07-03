#ifndef _MOTOR_H
#define _MOTOR_H
#include "ti_msp_dl_config.h"
#include "board.h"

int Velocity_A(int TargetVelocity, int CurrentVelocity);
int Velocity_B(int TargetVelocity, int CurrentVelocity);
void Motor_PID_SetParameters(float kp, float ki, float kd);
void Motor_PID_SetBase(int basePwm, int baseVelocity);
void Motor_PID_Reset(void);
void Motor_SetPWMMax(int pwmMax);
void Set_PWM(int pwma,int pwmb);
#endif
