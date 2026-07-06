#ifndef _MOTOR_H
#define _MOTOR_H

#include "ti_msp_dl_config.h"
#include "board.h"

#define AIN1_OUT(X)  ( (X) ? (DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_12)) : (DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_12)) )
#define AIN2_OUT(X)  ( (X) ? (DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_13)) : (DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_13)) )
#define BIN1_OUT(X)  ( (X) ? (DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_14)) : (DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_14)) )
#define BIN2_OUT(X)  ( (X) ? (DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_15)) : (DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_15)) )

void TB6612_Motor_Stop(void);
void AO_Control(uint8_t dir, uint32_t speed);
void BO_Control(uint8_t dir, uint32_t speed);

#endif