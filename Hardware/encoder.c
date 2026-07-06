#include "encoder.h"

volatile int32_t Get_Encoder_countA = 0;
volatile int32_t Get_Encoder_countB = 0;

void GROUP1_IRQHandler(void)
{
	uint32_t encoderA_interrupt;
	uint32_t encoderB_interrupt;

	encoderA_interrupt = DL_GPIO_getEnabledInterruptStatus(
		ENCODERA_PORT, ENCODERA_E1A_PIN | ENCODERA_E1B_PIN);
	if((encoderA_interrupt & ENCODERA_E1A_PIN) == ENCODERA_E1A_PIN)
	{
		if(!DL_GPIO_readPins(ENCODERA_PORT, ENCODERA_E1B_PIN))
		{
			Get_Encoder_countA++;
		}
		else
		{
			Get_Encoder_countA--;
		}
	}
	else if((encoderA_interrupt & ENCODERA_E1B_PIN) == ENCODERA_E1B_PIN)
	{
		if(!DL_GPIO_readPins(ENCODERA_PORT, ENCODERA_E1A_PIN))
		{
			Get_Encoder_countA--;
		}
		else
		{
			Get_Encoder_countA++;
		}
	}
	DL_GPIO_clearInterruptStatus(
		ENCODERA_PORT, ENCODERA_E1A_PIN | ENCODERA_E1B_PIN);

	encoderB_interrupt = DL_GPIO_getEnabledInterruptStatus(
		ENCODERB_PORT, ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);
	if((encoderB_interrupt & ENCODERB_E2A_PIN) == ENCODERB_E2A_PIN)
	{
		if(!DL_GPIO_readPins(ENCODERB_PORT, ENCODERB_E2B_PIN))
		{
			Get_Encoder_countB--;
		}
		else
		{
			Get_Encoder_countB++;
		}
	}
	else if((encoderB_interrupt & ENCODERB_E2B_PIN) == ENCODERB_E2B_PIN)
	{
		if(!DL_GPIO_readPins(ENCODERB_PORT, ENCODERB_E2A_PIN))
		{
			Get_Encoder_countB++;
		}
		else
		{
			Get_Encoder_countB--;
		}
	}
	DL_GPIO_clearInterruptStatus(
		ENCODERB_PORT, ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);
}
