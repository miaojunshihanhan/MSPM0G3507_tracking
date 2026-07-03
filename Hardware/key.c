#include "key.h"
/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2024-07-019

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2024-07-019

All rights reserved
***********************************************/
static volatile uint8_t key_press_event = 0;

// 每5 ms调用一次，连续稳定20 ms后确认按键状态
void Key_Scan_5ms(void)
{
	static uint8_t last_sample = 1;
	static uint8_t stable_state = 1;
	static uint8_t stable_count = 0;
	uint8_t current_sample;

	current_sample = DL_GPIO_readPins(KEY_PORT, KEY_PIN_21_PIN) ? 1 : 0;
	if(current_sample == last_sample)
	{
		if(stable_count < 4)
		{
			stable_count++;
		}
		if((stable_count >= 4) && (stable_state != current_sample))
		{
			stable_state = current_sample;
			if(stable_state == 0)
			{
				key_press_event = 1;
			}
		}
	}
	else
	{
		last_sample = current_sample;
		stable_count = 1;
	}
}

// 返回一次按下事件，不等待按键松开
uint8_t click(void)
{
	if(key_press_event)
	{
		key_press_event = 0;
		return 1;
	}
	return 0;
}



