#ifndef _CAR_H
#define _CAR_H

#include "ti_msp_dl_config.h"

void car_forward(int speed);
void car_backward(int speed);
void car_turnLeft(int Left_speed,int Right_speed);
void car_turnRight(int Left_speed,int Right_speed);
void Self_left(int Left_speed,int Right_speed);
void Self_right(int Left_speed,int Right_speed);
void Car_stop();

#endif
