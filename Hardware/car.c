#include "car.h"
#include "board.h"
#include "motor.h"
#include "tracking.h"
//A为左，B为右
void car_forward(int speed)
{
	AO_Control(1,speed);//1正转
	BO_Control(1,speed);
}
void car_backward(int speed)
{
	AO_Control(0,speed);//0反转
	BO_Control(0,speed);
}
void car_turnLeft(int Left_speed,int Right_speed)
{
	AO_Control(1,Left_speed);
	BO_Control(1,Right_speed);
}
void car_turnRight(int Left_speed,int Right_speed)
{
	AO_Control(1,Left_speed);
	BO_Control(1,Right_speed);
}
void Self_left(int Left_speed,int Right_speed)
{
	AO_Control(0,Left_speed);
	BO_Control(1,Right_speed);
}
void Self_right(int Left_speed,int Right_speed)
{
	AO_Control(1,Left_speed);
	BO_Control(0,Right_speed);
}
void Car_stop()
{
	TB6612_Motor_Stop();
}
