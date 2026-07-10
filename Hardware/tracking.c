#include "board.h"
#include "tracking.h"
#include "motor.h"
#include "w25q128.h"
#include <stdio.h>
/*=============================================
 * 传感器读取函数：Tracking_ReadSensors
 *=============================================*/
/*
 * 函数名：Tracking_ReadSensors
 * 功  能：读取8路灰度传感器，合成8位值
 * 返回值：sensors（8位无符号整数）
 *         bit0（D0）= 传感器1（最左），值1表示检测到白线（GPIO高电平）
 *         bit1（D1）= 传感器2，值1表示检测到白线
 *         ...
 *         bit7（D7）= 传感器8（最右），值1表示检测到白线
 *         值0表示检测到黑线（GPIO低电平）
 * 注意：此函数为硬件接口函数，请勿修改读取逻辑
 */
uint8_t Tracking_ReadSensors(void)
{
	uint32_t pinsA, pinsB;
	uint8_t sensors;

	pinsA = DL_GPIO_readPins(GrayA_PORT,
		GrayA_S1_PIN | GrayA_S2_PIN | GrayA_S3_PIN | GrayA_S4_PIN | GrayA_S7_PIN);

	pinsB = DL_GPIO_readPins(GraytB_PORT,
		GraytB_S5_PIN | GraytB_S6_PIN | GraytB_S8_PIN);

	sensors = 0;
	if(pinsA & GrayA_S1_PIN)   sensors |= 0x01;
	if(pinsA & GrayA_S2_PIN)   sensors |= 0x02;
	if(pinsA & GrayA_S3_PIN)   sensors |= 0x04;
	if(pinsA & GrayA_S4_PIN)   sensors |= 0x08;
	if(pinsB & GraytB_S5_PIN)  sensors |= 0x10;
	if(pinsB & GraytB_S6_PIN)  sensors |= 0x20;
	if(pinsA & GrayA_S7_PIN)   sensors |= 0x40;
	if(pinsB & GraytB_S8_PIN)  sensors |= 0x80;

	return sensors;
}

#define SPEED_LOOP_PERIOD_MS          (10u)
#define VOFA_DEFAULT_PERIOD_MS        (100u)
#define UART_LINE_MAX                 (64u)
#define CMD_QUIET_MS                  (300u)

#define SPEED_COUNT_MAX               (999)
#define PWM_MAX                       (3199)
#define PWM_MIN_DEFAULT               (500)
#define PWM_MIN_LOW_SPEED             (260)
#define PWM_MIN_LOW_SPEED_COUNT       (10)
#define PWM_SLEW_DEFAULT              (900)
#define KP_DEFAULT_X1000              (20000)
#define KI_DEFAULT_X1000              (2000)
#define KD_DEFAULT_X1000              (0)
#define FF_DEFAULT_X1000              (50000)
#define KP_MAX_X1000                  (100000)
#define KI_MAX_X1000                  (50000)
#define KD_MAX_X1000                  (50000)
#define FF_MAX_X1000                  (50000)
#define INTEGRAL_MARGIN_X1000         (400000)
#define PID_FLASH_ADDR                (0x010000u)
#define PID_FLASH_MAGIC               (0x53504C32u)

#define TRACK_MODE_OFF                (0u)
#define TRACK_MODE_SPIN_ALIGN         (1u)
#define TRACK_MODE_FOLLOW             (2u)
#define TRACK_KP_DEFAULT_X1000        (1)
#define TRACK_KI_DEFAULT_X1000        (0)
#define TRACK_KD_DEFAULT_X1000        (1)
#define TRACK_GAIN_MAX_X1000          (1000)
#define TRACK_BASE_DEFAULT            (4)
#define TRACK_TURN_MAX_DEFAULT        (3)
#define TRACK_TURN_MIN_DEFAULT        (0)
#define TRACK_DEADBAND_DEFAULT        (750)
#define TRACK_FOLLOW_DEADBAND         (250)
#define TRACK_CENTER_MASK             (0x18u)
#define TRACK_POSITIVE_SIDE_MASK      (0x03u)
#define TRACK_NEGATIVE_SIDE_MASK      (0xC0u)
#define TRACK_RIGHT_ANGLE_TURN        (8)
#define TRACK_RIGHT_ANGLE_HOLD_TICKS  (120u)
#define TRACK_LOST_TURN               (6)
#define TRACK_SLOW_ERROR              (2000)
#define TRACK_SLOW_DELTA              (2)
#define TRACK_ERROR_LOST              (8000)
#define TRACK_LOST_GRACE_TICKS        (500u)

#define MOTOR_FORWARD_DIR             (1u)
#define MOTOR_REVERSE_DIR             (0u)
#define LEFT_ENCODER_DIR              (1)
#define RIGHT_ENCODER_DIR             (1)

typedef struct
{
	int32_t target;
	int32_t actual;
	int32_t last_count;
	int32_t error;
	int32_t last_error;
	int32_t integral_x1000;
	int32_t pwm;
	int32_t kp_x1000;
	int32_t ki_x1000;
	int32_t kd_x1000;
	int32_t ff_x1000;
} SpeedWheel;

typedef struct
{
	uint32_t magic;
	int32_t left_kp_x1000;
	int32_t right_kp_x1000;
	int32_t left_ki_x1000;
	int32_t right_ki_x1000;
	int32_t left_kd_x1000;
	int32_t right_kd_x1000;
	int32_t left_ff_x1000;
	int32_t right_ff_x1000;
	int32_t min_pwm;
	int32_t pwm_slew;
	uint32_t checksum;
} SpeedPidConfig;

typedef struct
{
	int32_t error;
	int32_t last_error;
	int32_t integral_x1000;
	int32_t output;
	int32_t kp_x1000;
	int32_t ki_x1000;
	int32_t kd_x1000;
	int32_t base_speed;
	int32_t turn_max;
	int32_t turn_min;
	int32_t deadband;
	uint8_t mode;
	uint8_t sensors;
	uint8_t line_mask;
	uint8_t line_lost;
	uint8_t right_angle;
	uint16_t right_angle_ticks;
	int32_t right_angle_turn;
	uint16_t lost_ticks;
} TrackLoop;

static SpeedWheel left_wheel = {0, 0, 0, 0, 0, 0, 0, KP_DEFAULT_X1000, KI_DEFAULT_X1000, KD_DEFAULT_X1000, FF_DEFAULT_X1000};
static SpeedWheel right_wheel = {0, 0, 0, 0, 0, 0, 0, KP_DEFAULT_X1000, KI_DEFAULT_X1000, KD_DEFAULT_X1000, FF_DEFAULT_X1000};
static TrackLoop track_loop = {0, 0, 0, 0, TRACK_KP_DEFAULT_X1000, TRACK_KI_DEFAULT_X1000, TRACK_KD_DEFAULT_X1000, TRACK_BASE_DEFAULT, TRACK_TURN_MAX_DEFAULT, TRACK_TURN_MIN_DEFAULT, TRACK_DEADBAND_DEFAULT, TRACK_MODE_OFF, 0xFFu, 0u, 1u, 0u, 0u, 0, 0u};

static volatile uint8_t speed_loop_pending = 0u;
static volatile uint8_t uart_line_ready = 0u;
static volatile uint8_t uart_line_index = 0u;
static volatile uint8_t uart_idle_ms = 0u;
static volatile char uart_line_buf[UART_LINE_MAX];

static uint8_t vofa_enabled = 0u;
static uint16_t vofa_period_ms = VOFA_DEFAULT_PERIOD_MS;
static uint16_t cmd_quiet_ms = 0u;
static int32_t min_pwm = PWM_MIN_DEFAULT;
static int32_t pwm_slew = PWM_SLEW_DEFAULT;

static int32_t LimitInt32(int32_t value, int32_t min_value, int32_t max_value)
{
	if(value > max_value)
	{
		return max_value;
	}
	if(value < min_value)
	{
		return min_value;
	}
	return value;
}

static int32_t LimitStepInt32(int32_t current, int32_t target, int32_t max_step)
{
	int32_t delta = target - current;

	if(max_step <= 0)
	{
		return target;
	}
	if(delta > max_step)
	{
		return current + max_step;
	}
	if(delta < -max_step)
	{
		return current - max_step;
	}
	return target;
}

static uint32_t AbsInt32ToU32(int32_t value)
{
	if(value < 0)
	{
		return (uint32_t)(-value);
	}
	return (uint32_t)value;
}

static int32_t SignInt32(int32_t value)
{
	if(value > 0)
	{
		return 1;
	}
	if(value < 0)
	{
		return -1;
	}
	return 0;
}

static int32_t GetEffectiveMinPwm(int32_t target)
{
	uint32_t target_abs = AbsInt32ToU32(target);

	if((target_abs > 0u) && (target_abs <= PWM_MIN_LOW_SPEED_COUNT))
	{
		return LimitInt32(PWM_MIN_LOW_SPEED, 0, PWM_MAX);
	}
	return min_pwm;
}

static int32_t EncoderDeltaToSpeed(int32_t current_count, int32_t *last_count, int32_t encoder_dir)
{
	int32_t delta = current_count - *last_count;

	*last_count = current_count;
	return delta * encoder_dir;
}

static char ToUpperChar(char ch)
{
	if((ch >= 'a') && (ch <= 'z'))
	{
		return (char)(ch - ('a' - 'A'));
	}
	return ch;
}

static uint8_t IsCommandSeparator(char ch)
{
	return ((ch == '\0') || ((uint8_t)ch <= ' ') ||
	        (ch == '=') || (ch == ':') || (ch == ','));
}

static const char *FindCommandStart(const char *text)
{
	while((*text != '\0') &&
	      !(((*text >= 'A') && (*text <= 'Z')) ||
	        ((*text >= 'a') && (*text <= 'z')) ||
	        (*text == '?')))
	{
		text++;
	}
	return text;
}

static const char *SkipSeparators(const char *text)
{
	while((*text != '\0') &&
	      (((uint8_t)*text <= ' ') || (*text == '=') ||
	       (*text == ':') || (*text == ',')))
	{
		text++;
	}
	return text;
}

static uint8_t ParseInt32(const char **text, int32_t *value)
{
	int32_t sign = 1;
	int32_t result = 0;
	uint8_t has_digit = 0u;
	const char *p = SkipSeparators(*text);

	if(*p == '-')
	{
		sign = -1;
		p++;
	}
	else if(*p == '+')
	{
		p++;
	}

	while((*p >= '0') && (*p <= '9'))
	{
		has_digit = 1u;
		result = (result * 10) + (int32_t)(*p - '0');
		p++;
	}

	if(has_digit == 0u)
	{
		return 0u;
	}

	*value = result * sign;
	*text = p;
	return 1u;
}

static uint8_t ParseKpX1000(const char **text, int32_t *value)
{
	int32_t integer = 0;
	int32_t fraction = 0;
	int32_t scale = 100;
	uint8_t has_digit = 0u;
	const char *p = SkipSeparators(*text);

	while((*p >= '0') && (*p <= '9'))
	{
		has_digit = 1u;
		integer = (integer * 10) + (int32_t)(*p - '0');
		p++;
	}

	if(*p == '.')
	{
		p++;
		while(((*p >= '0') && (*p <= '9')) && (scale > 0))
		{
			has_digit = 1u;
			fraction += (int32_t)(*p - '0') * scale;
			scale /= 10;
			p++;
		}
		while((*p >= '0') && (*p <= '9'))
		{
			p++;
		}
	}

	if(has_digit == 0u)
	{
		return 0u;
	}

	*value = (integer * 1000) + fraction;
	*text = p;
	return 1u;
}

static uint32_t ConfigChecksum(const SpeedPidConfig *config)
{
	return config->magic ^
	       (uint32_t)config->left_kp_x1000 ^
	       ((uint32_t)config->right_kp_x1000 << 1) ^
	       ((uint32_t)config->left_ki_x1000 << 2) ^
	       ((uint32_t)config->right_ki_x1000 << 3) ^
	       ((uint32_t)config->left_kd_x1000 << 4) ^
	       ((uint32_t)config->right_kd_x1000 << 5) ^
	       ((uint32_t)config->left_ff_x1000 << 6) ^
	       ((uint32_t)config->right_ff_x1000 << 7) ^
	       ((uint32_t)config->min_pwm << 8) ^
	       ((uint32_t)config->pwm_slew << 9) ^
	       0xA55A5AA5u;
}

static void SavePidConfig(void)
{
	SpeedPidConfig config;

	config.magic = PID_FLASH_MAGIC;
	config.left_kp_x1000 = left_wheel.kp_x1000;
	config.right_kp_x1000 = right_wheel.kp_x1000;
	config.left_ki_x1000 = left_wheel.ki_x1000;
	config.right_ki_x1000 = right_wheel.ki_x1000;
	config.left_kd_x1000 = left_wheel.kd_x1000;
	config.right_kd_x1000 = right_wheel.kd_x1000;
	config.left_ff_x1000 = left_wheel.ff_x1000;
	config.right_ff_x1000 = right_wheel.ff_x1000;
	config.min_pwm = min_pwm;
	config.pwm_slew = pwm_slew;
	config.checksum = ConfigChecksum(&config);
	W25Q128_write((uint8_t *)&config, PID_FLASH_ADDR, (uint16_t)sizeof(config));
}

static void LoadPidConfig(void)
{
	SpeedPidConfig config;

	W25Q128_read((uint8_t *)&config, PID_FLASH_ADDR, (uint16_t)sizeof(config));
	if((config.magic == PID_FLASH_MAGIC) &&
	   (config.checksum == ConfigChecksum(&config)))
	{
		left_wheel.kp_x1000 = LimitInt32(config.left_kp_x1000, 0, KP_MAX_X1000);
		right_wheel.kp_x1000 = LimitInt32(config.right_kp_x1000, 0, KP_MAX_X1000);
		left_wheel.ki_x1000 = LimitInt32(config.left_ki_x1000, 0, KI_MAX_X1000);
		right_wheel.ki_x1000 = LimitInt32(config.right_ki_x1000, 0, KI_MAX_X1000);
		left_wheel.kd_x1000 = LimitInt32(config.left_kd_x1000, 0, KD_MAX_X1000);
		right_wheel.kd_x1000 = LimitInt32(config.right_kd_x1000, 0, KD_MAX_X1000);
		left_wheel.ff_x1000 = LimitInt32(config.left_ff_x1000, 0, FF_MAX_X1000);
		right_wheel.ff_x1000 = LimitInt32(config.right_ff_x1000, 0, FF_MAX_X1000);
		min_pwm = LimitInt32(config.min_pwm, 0, PWM_MAX);
		pwm_slew = LimitInt32(config.pwm_slew, 0, PWM_MAX);
	}
}

static void MotorOutput(int32_t left_pwm, int32_t right_pwm)
{
	uint8_t left_dir = MOTOR_FORWARD_DIR;
	uint8_t right_dir = MOTOR_FORWARD_DIR;

	left_pwm = LimitInt32(left_pwm, -PWM_MAX, PWM_MAX);
	right_pwm = LimitInt32(right_pwm, -PWM_MAX, PWM_MAX);

	if(left_pwm < 0)
	{
		left_dir = MOTOR_REVERSE_DIR;
	}
	if(right_pwm < 0)
	{
		right_dir = MOTOR_REVERSE_DIR;
	}

	AO_Control(left_dir, AbsInt32ToU32(left_pwm));
	BO_Control(right_dir, AbsInt32ToU32(right_pwm));
}

static void ResetWheelState(SpeedWheel *wheel)
{
	wheel->error = 0;
	wheel->last_error = 0;
	wheel->integral_x1000 = 0;
	wheel->pwm = 0;
}

static int32_t CalculateWheelPwm(SpeedWheel *wheel)
{
	int32_t target_sign;
	int32_t base_pwm;
	int32_t pid_pwm;
	int32_t command_pwm;
	int32_t error_delta;
	int32_t integral_limit;
	int32_t integral_update;
	int32_t effective_min_pwm;

	wheel->error = wheel->target - wheel->actual;

	target_sign = SignInt32(wheel->target);
	if(target_sign == 0)
	{
		wheel->pwm = LimitStepInt32(wheel->pwm, 0, pwm_slew);
		if(wheel->pwm == 0)
		{
			wheel->last_error = 0;
			wheel->integral_x1000 = 0;
		}
		return wheel->pwm;
	}

	error_delta = wheel->error - wheel->last_error;
	wheel->last_error = wheel->error;

	effective_min_pwm = GetEffectiveMinPwm(wheel->target);
	base_pwm = (wheel->ff_x1000 * (int32_t)AbsInt32ToU32(wheel->target)) / 1000;
	if(effective_min_pwm > 0)
	{
		base_pwm += effective_min_pwm;
	}
	base_pwm *= target_sign;

	integral_limit = ((PWM_MAX - (int32_t)AbsInt32ToU32(base_pwm)) * 1000) +
	                 INTEGRAL_MARGIN_X1000;
	if(integral_limit < 0)
	{
		integral_limit = 0;
	}

	integral_update = wheel->ki_x1000 * wheel->error;
	wheel->integral_x1000 += integral_update;
	wheel->integral_x1000 = LimitInt32(wheel->integral_x1000,
	                                  -integral_limit,
	                                  integral_limit);

	pid_pwm = ((wheel->kp_x1000 * wheel->error) +
	           wheel->integral_x1000 +
	           (wheel->kd_x1000 * error_delta)) / 1000;
	command_pwm = base_pwm + pid_pwm;

	if(target_sign > 0)
	{
		command_pwm = LimitInt32(command_pwm, 0, PWM_MAX);
		if(command_pwm == PWM_MAX)
		{
			wheel->integral_x1000 -= wheel->ki_x1000 * wheel->error;
		}
	}
	else
	{
		command_pwm = LimitInt32(command_pwm, -PWM_MAX, 0);
		if(command_pwm == -PWM_MAX)
		{
			wheel->integral_x1000 -= wheel->ki_x1000 * wheel->error;
		}
	}

	wheel->pwm = LimitStepInt32(wheel->pwm, command_pwm, pwm_slew);
	return wheel->pwm;
}

static void SetWheelKp(SpeedWheel *wheel, int32_t value)
{
	wheel->kp_x1000 = LimitInt32(value, 0, KP_MAX_X1000);
	wheel->integral_x1000 = 0;
}

static void SetWheelKi(SpeedWheel *wheel, int32_t value)
{
	wheel->ki_x1000 = LimitInt32(value, 0, KI_MAX_X1000);
	wheel->integral_x1000 = 0;
}

static void SetWheelKd(SpeedWheel *wheel, int32_t value)
{
	wheel->kd_x1000 = LimitInt32(value, 0, KD_MAX_X1000);
}

static void SetWheelFf(SpeedWheel *wheel, int32_t value)
{
	wheel->ff_x1000 = LimitInt32(value, 0, FF_MAX_X1000);
}

static void PrintFixed3(const char *name, int32_t value)
{
	printf("%s=%ld.%03ld",
	       name,
	       (long)(value / 1000),
	       (long)AbsInt32ToU32(value % 1000));
}

static void PrintParamSaved(const char *name, int32_t value)
{
	SavePidConfig();
	printf("OK ");
	PrintFixed3(name, value);
	printf(" SAVED\r\n");
}

static void SetBothSpeedTarget(int32_t left_target, int32_t right_target)
{
	left_wheel.target = LimitInt32(left_target, -SPEED_COUNT_MAX, SPEED_COUNT_MAX);
	right_wheel.target = LimitInt32(right_target, -SPEED_COUNT_MAX, SPEED_COUNT_MAX);
	left_wheel.integral_x1000 = 0;
	right_wheel.integral_x1000 = 0;
	left_wheel.last_error = 0;
	right_wheel.last_error = 0;
}

static void UpdateBothSpeedTarget(int32_t left_target, int32_t right_target)
{
	left_wheel.target = LimitInt32(left_target, -SPEED_COUNT_MAX, SPEED_COUNT_MAX);
	right_wheel.target = LimitInt32(right_target, -SPEED_COUNT_MAX, SPEED_COUNT_MAX);
}

static void SetSingleSpeedTarget(SpeedWheel *active_wheel, SpeedWheel *idle_wheel, int32_t target)
{
	active_wheel->target = LimitInt32(target, -SPEED_COUNT_MAX, SPEED_COUNT_MAX);
	idle_wheel->target = 0;
	active_wheel->integral_x1000 = 0;
	idle_wheel->integral_x1000 = 0;
	active_wheel->last_error = 0;
	idle_wheel->last_error = 0;
}

static void ResetTrackLoop(void)
{
	track_loop.error = 0;
	track_loop.last_error = 0;
	track_loop.integral_x1000 = 0;
	track_loop.output = 0;
	track_loop.line_lost = 0u;
	track_loop.right_angle = 0u;
	track_loop.right_angle_ticks = 0u;
	track_loop.right_angle_turn = 0;
	track_loop.lost_ticks = 0u;
}

static uint8_t IsCenterLine(uint8_t line_mask)
{
	return ((line_mask == 0x08u) ||
	        (line_mask == 0x10u) ||
	        (line_mask == TRACK_CENTER_MASK)) ? 1u : 0u;
}

static void UpdateForwardPivotTarget(int32_t turn)
{
	if(turn > 0)
	{
		UpdateBothSpeedTarget(LimitInt32(turn, 0, SPEED_COUNT_MAX), 0);
	}
	else if(turn < 0)
	{
		UpdateBothSpeedTarget(0, LimitInt32(-turn, 0, SPEED_COUNT_MAX));
	}
	else
	{
		UpdateBothSpeedTarget(0, 0);
	}
}

static int32_t CalculateLineError(uint8_t sensors, uint8_t *line_mask, uint8_t *line_lost)
{
	static const int16_t sensor_position[8] = {
		3500, 2500, 1500, 500, -500, -1500, -2500, -3500
	};
	uint8_t mask = (uint8_t)(~sensors) & 0xFFu;
	uint8_t i;
	int32_t sum = 0;
	int32_t count = 0;

	*line_mask = mask;
	if(mask == 0u)
	{
		*line_lost = 1u;
		if(track_loop.error > 0)
		{
			return TRACK_ERROR_LOST;
		}
		if(track_loop.error < 0)
		{
			return -TRACK_ERROR_LOST;
		}
		return 0;
	}

	*line_lost = 0u;
	if(IsCenterLine(mask) != 0u)
	{
		return 0;
	}
	for(i = 0u; i < 8u; i++)
	{
		if((mask & (uint8_t)(1u << i)) != 0u)
		{
			sum += sensor_position[i];
			count++;
		}
	}

	return sum / count;
}

static int32_t GetRightAngleTurn(uint8_t line_mask)
{
	uint8_t has_center = ((line_mask & TRACK_CENTER_MASK) != 0u) ? 1u : 0u;
	uint8_t has_positive_side = ((line_mask & TRACK_POSITIVE_SIDE_MASK) != 0u) ? 1u : 0u;
	uint8_t has_negative_side = ((line_mask & TRACK_NEGATIVE_SIDE_MASK) != 0u) ? 1u : 0u;

	if(has_center != 0u)
	{
		return 0;
	}
	if((has_positive_side != 0u) && (has_negative_side == 0u))
	{
		return TRACK_RIGHT_ANGLE_TURN;
	}
	if((has_negative_side != 0u) && (has_positive_side == 0u))
	{
		return -TRACK_RIGHT_ANGLE_TURN;
	}
	return 0;
}

static int32_t CalculateTrackTurn(void)
{
	int32_t error_delta;
	int32_t output;
	int32_t turn_abs;
	int32_t effective_deadband;
	int32_t right_angle_turn;

	track_loop.sensors = Tracking_ReadSensors();
	track_loop.error = CalculateLineError(track_loop.sensors,
	                                      &track_loop.line_mask,
	                                      &track_loop.line_lost);
	if(track_loop.line_lost == 0u)
	{
		track_loop.lost_ticks = 0u;
	}
	track_loop.right_angle = 0u;
	right_angle_turn = GetRightAngleTurn(track_loop.line_mask);
	if(right_angle_turn != 0)
	{
		track_loop.error = right_angle_turn * 1000;
		track_loop.last_error = track_loop.error;
		track_loop.integral_x1000 = 0;
		track_loop.output = right_angle_turn;
		track_loop.right_angle = 1u;
		track_loop.right_angle_ticks = TRACK_RIGHT_ANGLE_HOLD_TICKS;
		track_loop.right_angle_turn = right_angle_turn;
		return right_angle_turn;
	}
	if(track_loop.right_angle_ticks > 0u)
	{
		if((track_loop.line_lost == 0u) &&
		   (IsCenterLine(track_loop.line_mask) != 0u))
		{
			track_loop.right_angle_ticks = 0u;
			track_loop.right_angle_turn = 0;
		}
		else
		{
			track_loop.right_angle_ticks--;
			track_loop.error = track_loop.right_angle_turn * 1000;
			track_loop.last_error = track_loop.error;
			track_loop.integral_x1000 = 0;
			track_loop.output = track_loop.right_angle_turn;
			track_loop.right_angle = 1u;
			return track_loop.right_angle_turn;
		}
	}

	effective_deadband = (track_loop.mode == TRACK_MODE_FOLLOW) ?
	                     TRACK_FOLLOW_DEADBAND : track_loop.deadband;
	if(AbsInt32ToU32(track_loop.error) <= (uint32_t)effective_deadband)
	{
		track_loop.error = 0;
	}

	error_delta = track_loop.error - track_loop.last_error;
	track_loop.last_error = track_loop.error;
	track_loop.integral_x1000 += track_loop.ki_x1000 * track_loop.error;
	track_loop.integral_x1000 = LimitInt32(track_loop.integral_x1000,
	                                      -((int32_t)track_loop.turn_max * 1000),
	                                      ((int32_t)track_loop.turn_max * 1000));

	output = ((track_loop.kp_x1000 * track_loop.error) +
	          track_loop.integral_x1000 +
	          (track_loop.kd_x1000 * error_delta)) / 1000;
	output = LimitInt32(output, -track_loop.turn_max, track_loop.turn_max);

	if(output != 0)
	{
		turn_abs = (int32_t)AbsInt32ToU32(output);
		if((track_loop.turn_min > 0) && (turn_abs < track_loop.turn_min))
		{
			output = track_loop.turn_min * SignInt32(output);
		}
	}

	track_loop.output = output;
	return output;
}

static void ApplyLostFollowTarget(int32_t turn)
{
	if(track_loop.lost_ticks >= TRACK_LOST_GRACE_TICKS)
	{
		UpdateBothSpeedTarget(0, 0);
		return;
	}

	track_loop.lost_ticks++;
	if(turn != 0)
	{
		UpdateForwardPivotTarget(TRACK_LOST_TURN * SignInt32(turn));
		return;
	}

	UpdateBothSpeedTarget(track_loop.base_speed, track_loop.base_speed);
}

static int32_t GetTrackBaseSpeed(void)
{
	int32_t base;

	if(track_loop.mode == TRACK_MODE_SPIN_ALIGN)
	{
		return 0;
	}

	base = track_loop.base_speed;
	if((track_loop.mode == TRACK_MODE_FOLLOW) &&
	   (AbsInt32ToU32(track_loop.error) >= TRACK_SLOW_ERROR))
	{
		base = LimitInt32(base - TRACK_SLOW_DELTA, 0, SPEED_COUNT_MAX);
	}

	return base;
}

static void ApplyTrackTarget(int32_t turn, int32_t base)
{
	if(track_loop.mode == TRACK_MODE_FOLLOW)
	{
		if(track_loop.right_angle != 0u)
		{
			UpdateForwardPivotTarget(turn);
		}
		else
		{
			UpdateBothSpeedTarget(LimitInt32(base + turn, 0, SPEED_COUNT_MAX),
			                      LimitInt32(base - turn, 0, SPEED_COUNT_MAX));
		}
		return;
	}

	UpdateForwardPivotTarget(turn);
}

static void TrackLoop10ms(void)
{
	int32_t turn;
	int32_t base = 0;

	if(track_loop.mode == TRACK_MODE_OFF)
	{
		return;
	}

	turn = CalculateTrackTurn();
	if(track_loop.line_lost != 0u)
	{
		if(track_loop.mode == TRACK_MODE_FOLLOW)
		{
			ApplyLostFollowTarget(turn);
			return;
		}
	}
	else
	{
		base = GetTrackBaseSpeed();
	}

	ApplyTrackTarget(turn, base);
}

static void SpeedLoop10ms(void)
{
	static uint16_t vofa_tick_ms = 0u;
	int32_t encoder_a;
	int32_t encoder_b;

	__disable_irq();
	encoder_a = Get_Encoder_countA;
	encoder_b = Get_Encoder_countB;
	__enable_irq();

	left_wheel.actual = EncoderDeltaToSpeed(encoder_b, &left_wheel.last_count, LEFT_ENCODER_DIR);
	right_wheel.actual = EncoderDeltaToSpeed(encoder_a, &right_wheel.last_count, RIGHT_ENCODER_DIR);

	TrackLoop10ms();
	CalculateWheelPwm(&left_wheel);
	CalculateWheelPwm(&right_wheel);
	MotorOutput(left_wheel.pwm, right_wheel.pwm);

	if(cmd_quiet_ms >= SPEED_LOOP_PERIOD_MS)
	{
		cmd_quiet_ms -= SPEED_LOOP_PERIOD_MS;
	}
	else
	{
		cmd_quiet_ms = 0u;
	}

	vofa_tick_ms += SPEED_LOOP_PERIOD_MS;
	if((vofa_enabled != 0u) &&
	   (cmd_quiet_ms == 0u) &&
	   (vofa_tick_ms >= vofa_period_ms))
	{
		vofa_tick_ms = 0u;
		printf("%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%u,%u\r\n",
		       (long)left_wheel.target,
		       (long)right_wheel.target,
		       (long)left_wheel.actual,
		       (long)right_wheel.actual,
		       (long)left_wheel.pwm,
		       (long)right_wheel.pwm,
		       (long)track_loop.error,
		       (long)track_loop.output,
		       (unsigned int)track_loop.sensors,
		       (unsigned int)track_loop.line_mask);
	}
}

static void StopSpeedLoop(void)
{
	track_loop.mode = TRACK_MODE_OFF;
	ResetTrackLoop();
	left_wheel.target = 0;
	right_wheel.target = 0;
	ResetWheelState(&left_wheel);
	ResetWheelState(&right_wheel);
	MotorOutput(0, 0);
}

static void PrintStatus(void)
{
	int32_t encoder_a;
	int32_t encoder_b;
	uint8_t left_saturated = ((AbsInt32ToU32(left_wheel.pwm) >= PWM_MAX) &&
	                          (left_wheel.target != left_wheel.actual)) ? 1u : 0u;
	uint8_t right_saturated = ((AbsInt32ToU32(right_wheel.pwm) >= PWM_MAX) &&
	                           (right_wheel.target != right_wheel.actual)) ? 1u : 0u;

	__disable_irq();
	encoder_a = Get_Encoder_countA;
	encoder_b = Get_Encoder_countB;
	__enable_irq();

	printf("EA=%ld,EB=%ld,LT_CNT10=%ld,RT_CNT10=%ld,LA_CNT10=%ld,RA_CNT10=%ld,LPWM=%ld,RPWM=%ld,SATL=%u,SATR=%u,",
	       (long)encoder_a,
	       (long)encoder_b,
	       (long)left_wheel.target,
	       (long)right_wheel.target,
	       (long)left_wheel.actual,
	       (long)right_wheel.actual,
	       (long)left_wheel.pwm,
	       (long)right_wheel.pwm,
	       (unsigned int)left_saturated,
	       (unsigned int)right_saturated);
	PrintFixed3("KPL", left_wheel.kp_x1000);
	printf(",");
	PrintFixed3("KPR", right_wheel.kp_x1000);
	printf(",");
	PrintFixed3("KIL", left_wheel.ki_x1000);
	printf(",");
	PrintFixed3("KIR", right_wheel.ki_x1000);
	printf(",");
	PrintFixed3("KDL", left_wheel.kd_x1000);
	printf(",");
	PrintFixed3("KDR", right_wheel.kd_x1000);
	printf(",");
	PrintFixed3("FFL", left_wheel.ff_x1000);
	printf(",");
	PrintFixed3("FFR", right_wheel.ff_x1000);
	printf(",MINPWM=%ld,SLEW=%ld,TRK=%u,TERR=%ld,TURN=%ld,SENS=0x%02X,LINE=0x%02X,LOST=%u,LTICK=%u,RA=%u,RAT=%u,TBASE=%ld,TMAX=%ld,TMIN=%ld,TDB=%ld,",
	       (long)min_pwm,
	       (long)pwm_slew,
	       (unsigned int)track_loop.mode,
	       (long)track_loop.error,
	       (long)track_loop.output,
	       (unsigned int)track_loop.sensors,
	       (unsigned int)track_loop.line_mask,
	       (unsigned int)track_loop.line_lost,
	       (unsigned int)track_loop.lost_ticks,
	       (unsigned int)track_loop.right_angle,
	       (unsigned int)track_loop.right_angle_ticks,
	       (long)track_loop.base_speed,
	       (long)track_loop.turn_max,
	       (long)track_loop.turn_min,
	       (long)track_loop.deadband);
	PrintFixed3("TKP", track_loop.kp_x1000);
	printf(",");
	PrintFixed3("TKI", track_loop.ki_x1000);
	printf(",");
	PrintFixed3("TKD", track_loop.kd_x1000);
	printf("\r\n");
}

static void PrintLineStatus(void)
{
	uint8_t sensors = Tracking_ReadSensors();
	uint8_t line_mask;
	uint8_t line_lost;
	int32_t error = CalculateLineError(sensors, &line_mask, &line_lost);

	printf("SENS=0x%02X,LINE=0x%02X,ERR=%ld,TURN=%ld,TRK=%u,LOST=%u\r\n",
	       (unsigned int)sensors,
	       (unsigned int)line_mask,
	       (long)error,
	       (long)track_loop.output,
	       (unsigned int)track_loop.mode,
	       (unsigned int)line_lost);
}

static void ProcessCommand(const char *line)
{
	const char *p = FindCommandStart(line);
	char c0 = ToUpperChar(p[0]);
	char c1 = ToUpperChar(p[1]);
	char c2 = ToUpperChar(p[2]);
	int32_t first;
	int32_t second;

	if(c0 == '\0')
	{
		return;
	}

	if(((c0 == 'O') && (c1 == 'K') && IsCommandSeparator(p[2])) ||
	   ((c0 == 'E') && (c1 == 'R') && (c2 == 'R')) ||
	   ((c0 == 'L') && (c1 == 'T') && (c2 == '_')))
	{
		return;
	}

	if((c0 == 'K') && (c1 == 'P') && (c2 == 'L'))
	{
		p += 3;
		if(ParseKpX1000(&p, &first) != 0u)
		{
			cmd_quiet_ms = CMD_QUIET_MS;
			SetWheelKp(&left_wheel, first);
			PrintParamSaved("KPL", left_wheel.kp_x1000);
		}
		return;
	}

	if((c0 == 'K') && (c1 == 'P') && (c2 == 'R'))
	{
		p += 3;
		if(ParseKpX1000(&p, &first) != 0u)
		{
			cmd_quiet_ms = CMD_QUIET_MS;
			SetWheelKp(&right_wheel, first);
			PrintParamSaved("KPR", right_wheel.kp_x1000);
		}
		return;
	}

	if((c0 == 'K') && (c1 == 'P') && IsCommandSeparator(p[2]))
	{
		p += 2;
		if(ParseKpX1000(&p, &first) != 0u)
		{
			cmd_quiet_ms = CMD_QUIET_MS;
			SetWheelKp(&left_wheel, first);
			SetWheelKp(&right_wheel, first);
			PrintParamSaved("KP", left_wheel.kp_x1000);
		}
		return;
	}

	if((c0 == 'K') && (c1 == 'I') && (c2 == 'L'))
	{
		p += 3;
		if(ParseKpX1000(&p, &first) != 0u)
		{
			cmd_quiet_ms = CMD_QUIET_MS;
			SetWheelKi(&left_wheel, first);
			PrintParamSaved("KIL", left_wheel.ki_x1000);
		}
		return;
	}

	if((c0 == 'K') && (c1 == 'I') && (c2 == 'R'))
	{
		p += 3;
		if(ParseKpX1000(&p, &first) != 0u)
		{
			cmd_quiet_ms = CMD_QUIET_MS;
			SetWheelKi(&right_wheel, first);
			PrintParamSaved("KIR", right_wheel.ki_x1000);
		}
		return;
	}

	if((c0 == 'K') && (c1 == 'I') && IsCommandSeparator(p[2]))
	{
		p += 2;
		if(ParseKpX1000(&p, &first) != 0u)
		{
			cmd_quiet_ms = CMD_QUIET_MS;
			SetWheelKi(&left_wheel, first);
			SetWheelKi(&right_wheel, first);
			PrintParamSaved("KI", left_wheel.ki_x1000);
		}
		return;
	}

	if((c0 == 'K') && (c1 == 'D') && (c2 == 'L'))
	{
		p += 3;
		if(ParseKpX1000(&p, &first) != 0u)
		{
			cmd_quiet_ms = CMD_QUIET_MS;
			SetWheelKd(&left_wheel, first);
			PrintParamSaved("KDL", left_wheel.kd_x1000);
		}
		return;
	}

	if((c0 == 'K') && (c1 == 'D') && (c2 == 'R'))
	{
		p += 3;
		if(ParseKpX1000(&p, &first) != 0u)
		{
			cmd_quiet_ms = CMD_QUIET_MS;
			SetWheelKd(&right_wheel, first);
			PrintParamSaved("KDR", right_wheel.kd_x1000);
		}
		return;
	}

	if((c0 == 'K') && (c1 == 'D') && IsCommandSeparator(p[2]))
	{
		p += 2;
		if(ParseKpX1000(&p, &first) != 0u)
		{
			cmd_quiet_ms = CMD_QUIET_MS;
			SetWheelKd(&left_wheel, first);
			SetWheelKd(&right_wheel, first);
			PrintParamSaved("KD", left_wheel.kd_x1000);
		}
		return;
	}

	if((c0 == 'F') && (c1 == 'F') && (c2 == 'L'))
	{
		p += 3;
		if(ParseKpX1000(&p, &first) != 0u)
		{
			cmd_quiet_ms = CMD_QUIET_MS;
			SetWheelFf(&left_wheel, first);
			PrintParamSaved("FFL", left_wheel.ff_x1000);
		}
		return;
	}

	if((c0 == 'F') && (c1 == 'F') && (c2 == 'R'))
	{
		p += 3;
		if(ParseKpX1000(&p, &first) != 0u)
		{
			cmd_quiet_ms = CMD_QUIET_MS;
			SetWheelFf(&right_wheel, first);
			PrintParamSaved("FFR", right_wheel.ff_x1000);
		}
		return;
	}

	if((c0 == 'F') && (c1 == 'F') && IsCommandSeparator(p[2]))
	{
		p += 2;
		if(ParseKpX1000(&p, &first) != 0u)
		{
			cmd_quiet_ms = CMD_QUIET_MS;
			SetWheelFf(&left_wheel, first);
			SetWheelFf(&right_wheel, first);
			PrintParamSaved("FF", left_wheel.ff_x1000);
		}
		return;
	}

	if((c0 == 'T') && (c1 == 'R') && (c2 == 'K'))
	{
		p += 3;
		if(ParseInt32(&p, &first) != 0u)
		{
			cmd_quiet_ms = CMD_QUIET_MS;
			track_loop.mode = (uint8_t)LimitInt32(first, TRACK_MODE_OFF, TRACK_MODE_FOLLOW);
			ResetTrackLoop();
			if(track_loop.mode == TRACK_MODE_OFF)
			{
				SetBothSpeedTarget(0, 0);
			}
			printf("OK TRK %u\r\n", (unsigned int)track_loop.mode);
		}
		return;
	}

	if((c0 == 'T') && (c1 == 'K') && (c2 == 'P'))
	{
		p += 3;
		if(ParseKpX1000(&p, &first) != 0u)
		{
			cmd_quiet_ms = CMD_QUIET_MS;
			track_loop.kp_x1000 = LimitInt32(first, 0, TRACK_GAIN_MAX_X1000);
			ResetTrackLoop();
			printf("OK ");
			PrintFixed3("TKP", track_loop.kp_x1000);
			printf("\r\n");
		}
		return;
	}

	if((c0 == 'T') && (c1 == 'K') && (c2 == 'I'))
	{
		p += 3;
		if(ParseKpX1000(&p, &first) != 0u)
		{
			cmd_quiet_ms = CMD_QUIET_MS;
			track_loop.ki_x1000 = LimitInt32(first, 0, TRACK_GAIN_MAX_X1000);
			ResetTrackLoop();
			printf("OK ");
			PrintFixed3("TKI", track_loop.ki_x1000);
			printf("\r\n");
		}
		return;
	}

	if((c0 == 'T') && (c1 == 'K') && (c2 == 'D'))
	{
		p += 3;
		if(ParseKpX1000(&p, &first) != 0u)
		{
			cmd_quiet_ms = CMD_QUIET_MS;
			track_loop.kd_x1000 = LimitInt32(first, 0, TRACK_GAIN_MAX_X1000);
			ResetTrackLoop();
			printf("OK ");
			PrintFixed3("TKD", track_loop.kd_x1000);
			printf("\r\n");
		}
		return;
	}

	if((c0 == 'T') && (c1 == 'M') && (c2 == 'A'))
	{
		p += 4;
		if(ParseInt32(&p, &first) != 0u)
		{
			cmd_quiet_ms = CMD_QUIET_MS;
			track_loop.turn_max = LimitInt32(first, 0, SPEED_COUNT_MAX);
			printf("OK TMAX %ld\r\n", (long)track_loop.turn_max);
		}
		return;
	}

	if((c0 == 'T') && (c1 == 'M') && (c2 == 'I'))
	{
		p += 4;
		if(ParseInt32(&p, &first) != 0u)
		{
			cmd_quiet_ms = CMD_QUIET_MS;
			track_loop.turn_min = LimitInt32(first, 0, track_loop.turn_max);
			printf("OK TMIN %ld\r\n", (long)track_loop.turn_min);
		}
		return;
	}

	if((c0 == 'T') && (c1 == 'D') && (c2 == 'B'))
	{
		p += 3;
		if(ParseInt32(&p, &first) != 0u)
		{
			cmd_quiet_ms = CMD_QUIET_MS;
			track_loop.deadband = LimitInt32(first, 0, 4000);
			printf("OK TDB %ld\r\n", (long)track_loop.deadband);
		}
		return;
	}

	if((c0 == 'T') && (c1 == 'B') && (c2 == 'A'))
	{
		p += 5;
		if(ParseInt32(&p, &first) != 0u)
		{
			cmd_quiet_ms = CMD_QUIET_MS;
			track_loop.base_speed = LimitInt32(first, -SPEED_COUNT_MAX, SPEED_COUNT_MAX);
			printf("OK TBASE %ld\r\n", (long)track_loop.base_speed);
		}
		return;
	}

	if((c0 == 'R') && (c1 == 'U') && (c2 == 'N'))
	{
		p += 3;
		if(ParseInt32(&p, &first) != 0u)
		{
			track_loop.base_speed = LimitInt32(first, 0, SPEED_COUNT_MAX);
		}
		cmd_quiet_ms = CMD_QUIET_MS;
		track_loop.mode = TRACK_MODE_FOLLOW;
		ResetTrackLoop();
		printf("OK RUN TBASE %ld\r\n", (long)track_loop.base_speed);
		return;
	}

	if((c0 == 'M') && (c1 == 'I') && (c2 == 'N'))
	{
		if((ToUpperChar(p[3]) == 'P') &&
		   (ToUpperChar(p[4]) == 'W') &&
		   (ToUpperChar(p[5]) == 'M'))
		{
			p += 6;
		}
		else
		{
			p += 3;
		}
		if(ParseInt32(&p, &first) != 0u)
		{
			cmd_quiet_ms = CMD_QUIET_MS;
			min_pwm = LimitInt32(first, 0, PWM_MAX);
			SavePidConfig();
			printf("OK MINPWM %ld SAVED\r\n", (long)min_pwm);
		}
		return;
	}

	if((c0 == 'S') && (c1 == 'L') && (c2 == 'E'))
	{
		p += 4;
		if(ParseInt32(&p, &first) != 0u)
		{
			cmd_quiet_ms = CMD_QUIET_MS;
			pwm_slew = LimitInt32(first, 0, PWM_MAX);
			SavePidConfig();
			printf("OK SLEW %ld SAVED\r\n", (long)pwm_slew);
		}
		return;
	}

	if((c0 == 'V') && (c1 == 'O') && (c2 == 'F'))
	{
		p += 4;
		if(ParseInt32(&p, &first) != 0u)
		{
			cmd_quiet_ms = CMD_QUIET_MS;
			vofa_enabled = (first != 0) ? 1u : 0u;
			printf("OK VOFA %u\r\n", (unsigned int)vofa_enabled);
		}
		return;
	}

	if((c0 == 'R') && (c1 == 'A') && (c2 == 'T'))
	{
		p += 4;
		if(ParseInt32(&p, &first) != 0u)
		{
			cmd_quiet_ms = CMD_QUIET_MS;
			vofa_period_ms = (uint16_t)LimitInt32(first, SPEED_LOOP_PERIOD_MS, 10000);
			printf("OK RATE %u ms\r\n", (unsigned int)vofa_period_ms);
		}
		return;
	}

	if((c0 == 'S') && (c1 == 'T') && (c2 == 'O'))
	{
		cmd_quiet_ms = CMD_QUIET_MS;
		StopSpeedLoop();
		printf("OK STOP\r\n");
		return;
	}

	if((c0 == 'S') && (c1 == 'T') && (c2 == 'A'))
	{
		cmd_quiet_ms = CMD_QUIET_MS;
		PrintStatus();
		return;
	}

	if((c0 == 'L') && (c1 == 'I') && (c2 == 'N'))
	{
		cmd_quiet_ms = CMD_QUIET_MS;
		PrintLineStatus();
		return;
	}

	if((c0 == 'S') && IsCommandSeparator(p[1]))
	{
		p++;
		if((ParseInt32(&p, &first) != 0u) && (ParseInt32(&p, &second) != 0u))
		{
			cmd_quiet_ms = CMD_QUIET_MS;
			track_loop.mode = TRACK_MODE_OFF;
			ResetTrackLoop();
			SetBothSpeedTarget(first, second);
			printf("OK SPEED %ld,%ld\r\n", (long)left_wheel.target, (long)right_wheel.target);
		}
		return;
	}

	if((c0 == 'L') && IsCommandSeparator(p[1]))
	{
		p++;
		if(ParseInt32(&p, &first) != 0u)
		{
			cmd_quiet_ms = CMD_QUIET_MS;
			track_loop.mode = TRACK_MODE_OFF;
			ResetTrackLoop();
			SetSingleSpeedTarget(&left_wheel, &right_wheel, first);
			printf("OK L %ld, R 0\r\n", (long)left_wheel.target);
		}
		return;
	}

	if((c0 == 'R') && IsCommandSeparator(p[1]))
	{
		p++;
		if(ParseInt32(&p, &first) != 0u)
		{
			cmd_quiet_ms = CMD_QUIET_MS;
			track_loop.mode = TRACK_MODE_OFF;
			ResetTrackLoop();
			SetSingleSpeedTarget(&right_wheel, &left_wheel, first);
			printf("OK L 0, R %ld\r\n", (long)right_wheel.target);
		}
		return;
	}

	if((c0 == 'H') || (c0 == '?'))
	{
		cmd_quiet_ms = CMD_QUIET_MS;
		printf("CMD: RUN base | TRK 0/1/2 | TBASE/TMAX/TMIN/TDB n | TKP/TKI/TKD n | LINE | S l r | KP/KI/KD/FF n | VOFA 0/1 | RATE ms | STOP | STAT\r\n");
		return;
	}

	return;
}

static void ProcessUartLine(void)
{
	char line[UART_LINE_MAX];
	uint8_t i;

	if(uart_line_ready == 0u)
	{
		return;
	}

	__disable_irq();
	for(i = 0u; i < UART_LINE_MAX; i++)
	{
		line[i] = uart_line_buf[i];
		if(line[i] == '\0')
		{
			break;
		}
	}
	line[UART_LINE_MAX - 1u] = '\0';
	uart_line_ready = 0u;
	__enable_irq();

	ProcessCommand(line);
}

void Tracking_Init(void)
{
	int32_t encoder_a;
	int32_t encoder_b;

	__disable_irq();
	encoder_a = Get_Encoder_countA;
	encoder_b = Get_Encoder_countB;
	__enable_irq();

	left_wheel.last_count = encoder_b;
	right_wheel.last_count = encoder_a;
	LoadPidConfig();
	StopSpeedLoop();
	printf("Cascade loop ready. RUN 6 follow | TRK 1 spin align | LINE | STAT | STOP\r\n");
	PrintStatus();
}

void Tracking_Timer1ms_ISR(void)
{
	static uint8_t tick = 0u;

	tick++;
	if(tick >= SPEED_LOOP_PERIOD_MS)
	{
		tick = 0u;
		if(speed_loop_pending < 10u)
		{
			speed_loop_pending++;
		}
	}

	if(uart_idle_ms > 0u)
	{
		uart_idle_ms--;
		if((uart_idle_ms == 0u) &&
		   (uart_line_index > 0u) &&
		   (uart_line_ready == 0u))
		{
			uart_line_buf[uart_line_index] = '\0';
			uart_line_ready = 1u;
			uart_line_index = 0u;
		}
	}
}

void Tracking_Task(void)
{
	ProcessUartLine();

	while(speed_loop_pending > 0u)
	{
		__disable_irq();
		if(speed_loop_pending > 0u)
		{
			speed_loop_pending--;
		}
		__enable_irq();
		SpeedLoop10ms();
	}
}

void Tracking_UartRxByte(uint8_t data)
{
	if((data == '\r') || (data == '\n'))
	{
		if((uart_line_index > 0u) && (uart_line_ready == 0u))
		{
			uart_line_buf[uart_line_index] = '\0';
			uart_line_ready = 1u;
		}
		uart_line_index = 0u;
		uart_idle_ms = 0u;
		return;
	}

	if((data == '\b') || (data == 0x7Fu))
	{
		if(uart_line_index > 0u)
		{
			uart_line_index--;
		}
		return;
	}

	if((data < ' ') || (data > '~'))
	{
		return;
	}

	if(uart_line_ready != 0u)
	{
		return;
	}

	if(uart_line_index < (UART_LINE_MAX - 1u))
	{
		uart_line_buf[uart_line_index] = (char)data;
		uart_line_index++;
		uart_idle_ms = 30u;
	}
	else
	{
		uart_line_index = 0u;
	}
}
