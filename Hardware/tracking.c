#include "tracking.h"

void Tracking()
{
	
}

uint8_t Tracking_ReadSensors(void)
{
	uint32_t pins;
	uint8_t sensors;

	pins = DL_GPIO_readPins(Gray_PORT,
		Gray_S1_PIN | Gray_S2_PIN | Gray_S3_PIN | Gray_S4_PIN |
		Gray_S5_PIN | Gray_S6_PIN | Gray_S7_PIN | Gray_S8_PIN);

	sensors = 0;
	if(pins & Gray_S1_PIN) sensors |= 0x01;
	if(pins & Gray_S2_PIN) sensors |= 0x02;
	if(pins & Gray_S3_PIN) sensors |= 0x04;
	if(pins & Gray_S4_PIN) sensors |= 0x08;
	if(pins & Gray_S5_PIN) sensors |= 0x10;
	if(pins & Gray_S6_PIN) sensors |= 0x20;
	if(pins & Gray_S7_PIN) sensors |= 0x40;
	if(pins & Gray_S8_PIN) sensors |= 0x80;

	return sensors;
}