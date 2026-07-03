#include "tracking.h"

#define TRACKING_DIRECTION       (-1)
#define TRACKING_INTEGRAL_LIMIT  100

float Tracking_Kp;
float Tracking_Ki;
float Tracking_Kd;
int32_t Tracking_Weights[4];

static int32_t TrackingIntegral;
static int32_t TrackingLastError;
static int32_t TrackingLastValidError;

typedef enum
{
	TRACKING_STATE_NORMAL = 0,
	TRACKING_STATE_LINE_LOST
} TrackingState;

static TrackingState TrackingCurrentState;

uint8_t Tracking_ReadSensors(void)
{
	uint32_t pins;
	uint8_t sensors = 0;

	pins = DL_GPIO_readPins(Gray_PORT,
		Gray_S1_PIN | Gray_S2_PIN | Gray_S3_PIN | Gray_S4_PIN);

	if(pins & Gray_S1_PIN) sensors |= 0x01;
	if(pins & Gray_S2_PIN) sensors |= 0x02;
	if(pins & Gray_S3_PIN) sensors |= 0x04;
	if(pins & Gray_S4_PIN) sensors |= 0x08;

	return sensors;
}

static int32_t Tracking_CalculateError(uint8_t sensors)
{
	int32_t weightedSum = 0;
	int32_t activeCount = 0;

	// 灰度输出：黑线为0、白底为1，只有检测到0的探头参与循迹
	if((sensors & 0x01) == 0) { weightedSum += Tracking_Weights[0]; activeCount++; }
	if((sensors & 0x02) == 0) { weightedSum += Tracking_Weights[1]; activeCount++; }
	if((sensors & 0x04) == 0) { weightedSum += Tracking_Weights[2]; activeCount++; }
	if((sensors & 0x08) == 0) { weightedSum += Tracking_Weights[3]; activeCount++; }

	if(activeCount == 0)
	{
		return 0;
	}

	return weightedSum / activeCount;
}

int32_t Tracking_PID_Calculate(int32_t maxCorrection)
{
	int32_t error;
	int32_t correction;
	float output;
	uint8_t sensors;

	sensors = Tracking_ReadSensors();
	// 原始灰度定义为黑=0、白=1，因此0x0F表示四路全白、黑线丢失
	if(sensors == 0x0F)
	{
		TrackingCurrentState = TRACKING_STATE_LINE_LOST;
		TrackingIntegral = 0;
		TrackingLastError = 0;

		if(TrackingLastValidError < 0)
		{
			return -maxCorrection * TRACKING_DIRECTION;
		}
		if(TrackingLastValidError > 0)
		{
			return maxCorrection * TRACKING_DIRECTION;
		}
		return 0;
	}

	if(TrackingCurrentState == TRACKING_STATE_LINE_LOST)
	{
		TrackingCurrentState = TRACKING_STATE_NORMAL;
		TrackingIntegral = 0;
		TrackingLastError = 0;
	}

	error = Tracking_CalculateError(sensors);
	TrackingLastValidError = error;
	TrackingIntegral += error;
	if(TrackingIntegral > TRACKING_INTEGRAL_LIMIT)
	{
		TrackingIntegral = TRACKING_INTEGRAL_LIMIT;
	}
	else if(TrackingIntegral < -TRACKING_INTEGRAL_LIMIT)
	{
		TrackingIntegral = -TRACKING_INTEGRAL_LIMIT;
	}

	output = Tracking_Kp * error
		+ Tracking_Ki * TrackingIntegral
		+ Tracking_Kd * (error - TrackingLastError);
	TrackingLastError = error;
	correction = (int32_t)(output * TRACKING_DIRECTION);

	if(correction > maxCorrection) correction = maxCorrection;
	else if(correction < -maxCorrection) correction = -maxCorrection;

	return correction;
}

void Tracking_PID_Reset(void)
{
	TrackingIntegral = 0;
	TrackingLastError = 0;
	TrackingLastValidError = 0;
	TrackingCurrentState = TRACKING_STATE_NORMAL;
}
