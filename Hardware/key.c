#include "key.h"

#define KEY_RELEASED_LEVEL     (1u)
#define KEY_PRESSED_LEVEL      (0u)
#define KEY_DEBOUNCE_MS        (10u)

static volatile uint8_t key_press_event = 0u;
static volatile uint8_t key_irq_pending = 0u;
static volatile uint8_t key_pressed = 0u;

static uint8_t Key_ReadLevel(void)
{
	return DL_GPIO_readPins(KEY_PORT, KEY_PIN_21_PIN) ? KEY_RELEASED_LEVEL : KEY_PRESSED_LEVEL;
}

static void Key_UpdateStableState(uint8_t current_sample)
{
	static uint8_t stable_state = KEY_RELEASED_LEVEL;

	if(stable_state != current_sample)
	{
		stable_state = current_sample;
		key_pressed = (current_sample == KEY_PRESSED_LEVEL) ? 1u : 0u;
		if(stable_state == KEY_PRESSED_LEVEL)
		{
			key_press_event = 1u;
		}
	}
}

void Key_B21_IRQHandler(void)
{
	key_irq_pending = 1u;
}

void Key_Task_1ms(void)
{
	static uint8_t last_sample = KEY_RELEASED_LEVEL;
	static uint8_t stable_state = KEY_RELEASED_LEVEL;
	static uint8_t stable_count = 0u;
	uint8_t current_sample;

	current_sample = Key_ReadLevel();
	if(key_irq_pending != 0u)
	{
		key_irq_pending = 0u;
		stable_count = 0u;
	}

	if(current_sample == last_sample)
	{
		if(stable_count < KEY_DEBOUNCE_MS)
		{
			stable_count++;
		}
	}
	else
	{
		last_sample = current_sample;
		stable_count = 1u;
	}

	if((stable_count >= KEY_DEBOUNCE_MS) && (stable_state != current_sample))
	{
		stable_state = current_sample;
		Key_UpdateStableState(stable_state);
	}
}

uint8_t Key_IsPressed(void)
{
	return key_pressed;
}

void Key_Scan_5ms(void)
{
	static uint8_t last_sample = KEY_RELEASED_LEVEL;
	static uint8_t stable_count = 0u;
	uint8_t current_sample;

	current_sample = Key_ReadLevel();
	if(current_sample == last_sample)
	{
		if(stable_count < 4u)
		{
			stable_count++;
		}
		if(stable_count >= 4u)
		{
			Key_UpdateStableState(current_sample);
		}
	}
	else
	{
		last_sample = current_sample;
		stable_count = 1u;
	}
}

uint8_t click(void)
{
	uint8_t event;

	__disable_irq();
	event = key_press_event;
	key_press_event = 0u;
	__enable_irq();

	if(event != 0u)
	{
		return 1u;
	}
	return 0u;
}
