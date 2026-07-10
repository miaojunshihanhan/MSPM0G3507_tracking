#ifndef _KEY_H
#define _KEY_H
#include "ti_msp_dl_config.h"
#include "board.h"

uint8_t click(void);
uint8_t Key_IsPressed(void);
void Key_B21_IRQHandler(void);
void Key_Task_1ms(void);
void Key_Scan_5ms(void);

#endif
