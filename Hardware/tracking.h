#ifndef _TRACKING_H
#define _TRACKING_H

#include "ti_msp_dl_config.h"

extern float Tracking_Kp;
extern float Tracking_Ki;
extern float Tracking_Kd;
extern int32_t Tracking_Weights[4];

uint8_t Tracking_ReadSensors(void);
int32_t Tracking_PID_Calculate(int32_t maxCorrection);
void Tracking_PID_Reset(void);

#endif
