#ifndef _TRACKING_H
#define _TRACKING_H

#include <stdint.h>

uint8_t Tracking_ReadSensors(void);
void Tracking_Init(void);
void Tracking_Timer1ms_ISR(void);
void Tracking_Task(void);
void Tracking_UartRxByte(uint8_t data);

#endif
