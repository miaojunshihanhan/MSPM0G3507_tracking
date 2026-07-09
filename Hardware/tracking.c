/*
 * tracking.c
 *
 * 差速轮小车循迹控制模块 - 串级PID实现
 * =============================================
 * 控制架构：外环循迹PD + 内环速度PD（串级PID）
 * 传感器：8路灰度传感器（从左到右编号1~8）
 * 执行器：左右轮独立电机驱动（AO_Control / BO_Control）
 * 反馈：左右轮编码器（speedA / speedB，单位：脉冲数/控制周期）
 * =============================================
 * 调用方式：在定时中断中周期性调用 Tracking() 函数
 * 建议调用周期：5~10ms
 * =============================================
 */

/*
 * 参数单位约定：
 * - speedA/speedB：编码器脉冲数/控制周期（来自 empty.c 全局变量）
 * - BASE_SPEED：目标速度，单位与编码器一致（脉冲数/控制周期）
 * - KP_OUT/KD_OUT：外环PD系数，无单位（纯数值）
 * - Turn：转向修正量，单位与编码器速度一致
 * - KP_SPEED_L/R, KD_SPEED_L/R：内环速度PD系数
 * - MOTOR_PWM_MAX：电机PWM上限（0~999，对应 AO_Control/BO_Control 的 speed 参数）
 */

/*=============================================
 * 可调参数（集中在此处，方便调试）
 *=============================================*/

/* ---- 基础速度设置 ---- */
#define BASE_SPEED                  (250.0f)    /* 直行基础速度（脉冲数/周期） */
#define TARGET_SPEED_MAX            (999.0f)    /* 目标速度绝对上限 */

/* ---- 外环PD参数（循迹环） ---- */
#define KP_OUT                      (80.0f)     /* 外环比例系数，影响循迹灵敏度 */
#define KD_OUT                      (0.0f)      /* 外环微分系数，抑制超调和振荡 */
#define TURN_MAX                    (300.0f)    /* 转向修正量最大限幅，防止过度转向 */

/* ---- 内环PD参数（速度环，左右轮独立） ---- */
#define KP_SPEED_L                  (1.2f)      /* 左轮速度环比例系数 */
#define KD_SPEED_L                  (0.0f)      /* 左轮速度环微分系数 */
#define KP_SPEED_R                  (1.2f)      /* 右轮速度环比例系数 */
#define KD_SPEED_R                  (0.0f)      /* 右轮速度环微分系数 */

/* ---- 电机控制参数 ---- */
#define MOTOR_PWM_MAX               (999.0f)    /* 电机PWM输出上限（对应AO_Control的speed范围0~1000） */
#define TRACKING_FORWARD_DIR        (1u)        /* 前进方向 */
#define TRACKING_REVERSE_DIR        (0u)        /* 后退方向 */


#include "tracking.h"
#include "motor.h"
#include "key.h"

extern volatile int32_t speedA;   /* 左轮编码器速度（脉冲数/控制周期） */
extern volatile int32_t speedB;   /* 右轮编码器速度（脉冲数/控制周期） */

uint8_t line_lost = 0;                   /* 丢线标志：1=所有传感器未检测到黑线 */
uint8_t tracking_enabled = 0;            /* 循迹使能：上电默认关闭，B21按键切换 */
Tracking_State tracking_state = {0};     /* 循迹状态结构体，用于调试和监控 */

static Tracking_PDController left_speed_pid = {KP_SPEED_L, KD_SPEED_L, 0.0f, 0.0f, 0.0f};   /* 左轮速度PID控制器 */
static Tracking_PDController right_speed_pid = {KP_SPEED_R, KD_SPEED_R, 0.0f, 0.0f, 0.0f}; /* 右轮速度PID控制器 */
static float last_valid_tracking_error = 0.0f;       /* 上一次有效偏差值（丢线时复用） */
static float outer_error_previous = 0.0f;            /* 外环PD的上一次误差（用于微分计算） */

/*=============================================
 * 辅助函数：浮点限幅
 *=============================================*/
/*
 * 函数名：Limit_Float
 * 功  能：将浮点数限制在指定范围内
 * 参  数：value - 输入值
 *         min_value - 最小值
 *         max_value - 最大值
 * 返回值：限幅后的浮点值
 */
static float Limit_Float(float value, float min_value, float max_value)
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

/*=============================================
 * 辅助函数：PWM值转无符号32位整数
 *=============================================*/
/*
 * 函数名：Pwm_To_U32
 * 功  能：将浮点PWM值转换为AO_Control/BO_Control所需的uint32_t类型
 *         包含四舍五入和限幅处理
 * 参  数：pwm - 浮点PWM值
 * 返回值：0~MOTOR_PWM_MAX范围内的uint32_t值
 */
static uint32_t Pwm_To_U32(float pwm)
{
    pwm = Limit_Float(pwm, 0.0f, MOTOR_PWM_MAX);
    return (uint32_t)(pwm + 0.5f);
}

/*=============================================
 * 辅助函数：左轮电机输出
 *=============================================*/
/*
 * 函数名：Left_Motor_Output
 * 功  能：根据浮点PWM值控制左轮电机
 *         自动处理正负值对应的前进/后退方向
 * 参  数：pwm - 浮点PWM值（正值=前进，负值=后退）
 */
static void Left_Motor_Output(float pwm)
{
    uint8_t dir = TRACKING_FORWARD_DIR;

    if(pwm < 0.0f)
    {
        pwm = -pwm;
        dir = TRACKING_REVERSE_DIR;
    }

    AO_Control(dir, Pwm_To_U32(pwm));
}

/*=============================================
 * 辅助函数：右轮电机输出
 *=============================================*/
/*
 * 函数名：Right_Motor_Output
 * 功  能：根据浮点PWM值控制右轮电机
 *         自动处理正负值对应的前进/后退方向
 * 参  数：pwm - 浮点PWM值（正值=前进，负值=后退）
 */
static void Right_Motor_Output(float pwm)
{
    uint8_t dir = TRACKING_FORWARD_DIR;

    if(pwm < 0.0f)
    {
        pwm = -pwm;
        dir = TRACKING_REVERSE_DIR;
    }

    BO_Control(dir, Pwm_To_U32(pwm));
}

static void Tracking_Motor_Stop(void)
{
    AO_Control(TRACKING_FORWARD_DIR, 0u);
    BO_Control(TRACKING_FORWARD_DIR, 0u);
}

/*=============================================
 * 主控制函数：Tracking
 *=============================================*/
/*
 * 函数名：Tracking
 * 功  能：循迹主控制函数（串级PID执行入口）
 * 执行流程：
 *   1. 外环：get_error() 获取传感器加权平均偏差
 *   2. 外环：Outer_PD_Control() 计算转向修正量 Turn
 *   3. 速度分解：左/右轮目标速度 = BASE_SPEED ± Turn
 *   4. 内环：Speed_PD_Control() 对左右轮进行速度闭环控制
 *   5. 输出：Left_Motor_Output / Right_Motor_Output 驱动电机
 *   6. 更新：保存状态到 tracking_state 供调试使用
 * 调用要求：在定时中断中周期性调用（建议5~10ms）
 */
void Tracking(void)
{
    float error;
    float turn;
    float left_target;
    float right_target;
    float left_pwm;
    float right_pwm;

    Key_Scan_5ms();
    if(click())
    {
        tracking_enabled = (uint8_t)!tracking_enabled;
        Tracking_Reset();
    }

    if(tracking_enabled == 0u)
    {
        Tracking_Motor_Stop();
        tracking_state.left_target = 0.0f;
        tracking_state.right_target = 0.0f;
        tracking_state.left_pwm = 0.0f;
        tracking_state.right_pwm = 0.0f;
        return;
    }

    /* ---- 步骤1：外环循迹PD ---- */
    /* 获取当前传感器偏差（单位：传感器间距） */
    error = get_error();
    /* 根据偏差计算转向修正量 */
    turn = Outer_PD_Control(error);

    /* ---- 步骤2：速度分解（差速转向） ---- */
    /* 偏差error < 0：黑线在左侧，需向左转
     *   左轮减速（BASE_SPEED + turn，turn为负）
     *   右轮加速（BASE_SPEED - turn，turn为负）
     * 偏差error > 0：黑线在右侧，需向右转
     *   左轮加速（BASE_SPEED + turn，turn为正）
     *   右轮减速（BASE_SPEED - turn，turn为正）
     */
    left_target = Limit_Float(BASE_SPEED + turn, -TARGET_SPEED_MAX, TARGET_SPEED_MAX);
    right_target = Limit_Float(BASE_SPEED - turn, -TARGET_SPEED_MAX, TARGET_SPEED_MAX);

    /* ---- 步骤3：内环速度PD（左右轮独立闭环） ---- */
    /* 根据目标速度和实际编码器速度，计算电机PWM输出 */
    left_pwm = Speed_PD_Control(&left_speed_pid, left_target, (float)speedA);
    right_pwm = Speed_PD_Control(&right_speed_pid, right_target, (float)speedB);

    /* ---- 步骤4：电机输出 ---- */
    Left_Motor_Output(left_pwm);
    Right_Motor_Output(right_pwm);

    /* ---- 步骤5：保存状态（供调试监控） ---- */
    tracking_state.error = error;
    tracking_state.turn = turn;
    tracking_state.left_target = left_target;
    tracking_state.right_target = right_target;
    tracking_state.left_pwm = left_pwm;
    tracking_state.right_pwm = right_pwm;
}

/*=============================================
 * 传感器处理：get_error
 *=============================================*/
/*
 * 函数名：get_error
 * 功  能：加权平均法计算传感器偏差
 * 算法原理：
 *   1. 调用 Tracking_ReadSensors() 获取8路传感器状态
 *   2. 传感器返回值：bit=1表示白线，bit=0表示黑线
 *   3. 对检测到黑线（~sensors & 1）的传感器进行加权平均：
 *      position = Σ(i * weight) / Σ(weight)，其中i为传感器编号(0~7)，weight=1
 *   4. 连续偏差：error = position - 3.5（传感器0~7的理论中心）
 * 丢线处理：
 *   当所有传感器均未检测到黑线（black_mask == 0）时，
 *   返回上一次有效的error值，并设置 line_lost = 1
 * 返回值：偏差 error（单位：传感器间距）
 *         范围：理论上 -3.5 ~ +3.5（传感器全部偏左到全部偏右）
 */
float get_error(void)
{
    uint8_t sensors;
    uint8_t black_mask;
    uint8_t i;
    float weighted_sum = 0.0f;
    float detected_count = 0.0f;
    float position;
    float error;

    /* 读取8路传感器原始值 */
    sensors = Tracking_ReadSensors();
    tracking_state.sensors = sensors;

    /* Tracking_ReadSensors返回1为白线、0为黑线，取反得到黑线掩码 */
    black_mask = (uint8_t)(~sensors) & 0xFFu;

    /* 丢线处理：所有传感器均为白线 */
    if(black_mask == 0u)
    {
        line_lost = 1u;
        return last_valid_tracking_error;
    }

    line_lost = 0u;

    /* 遍历8路传感器，对检测到黑线的传感器进行加权求和 */
    for(i = 0u; i < 8u; i++)
    {
        if(((black_mask >> i) & 0x01u) != 0u)
        {
            weighted_sum += (float)i;
            detected_count += 1.0f;
        }
    }

    /* 计算黑线中心位置 */
    position = weighted_sum / detected_count;
    /* 计算偏差（相对于理论中心3.5） */
    error = position - 3.5f;
    /* 保存有效偏差（供丢线时使用） */
    last_valid_tracking_error = error;

    return error;
}

/*=============================================
 * 外环PD控制器：Outer_PD_Control
 *=============================================*/
/*
 * 函数名：Outer_PD_Control
 * 功  能：外环位置式PD控制器，根据传感器偏差计算转向修正量
 * 控制律：
 *   Turn = KP_OUT * error + KD_OUT * (error - error_prev)
 * 参数说明：
 *   error - 当前偏差（来自get_error，单位：传感器间距）
 * 返回值：
 *   Turn - 转向修正量（单位与编码器速度一致，脉冲数/周期）
 * 限幅：
 *   Turn 被限制在 [-TURN_MAX, TURN_MAX] 范围内
 */
float Outer_PD_Control(float error)
{
    float turn;

    /* 位置式PD控制律：比例项 + 微分项 */
    turn = (KP_OUT * error) + (KD_OUT * (error - outer_error_previous));
    /* 更新上一次误差（供下次微分计算） */
    outer_error_previous = error;

    /* 转向修正量限幅，防止过度转向 */
    return Limit_Float(turn, -TURN_MAX, TURN_MAX);
}

/*=============================================
 * 内环PD控制器：Speed_PD_Control
 *=============================================*/
/*
 * 函数名：Speed_PD_Control
 * 功  能：内环位置式PD控制器，单轮速度闭环控制
 * 控制律：
 *   output = target_speed + KP * error + KD * (error - last_error)
 *   其中 error = target_speed - actual_speed
 * 参数说明：
 *   pid - PID控制器状态结构体指针
 *   target_speed - 目标速度（脉冲数/周期）
 *   actual_speed - 实际速度（编码器反馈，脉冲数/周期）
 * 返回值：
 *   电机PWM输出值（已限幅在 [-MOTOR_PWM_MAX, MOTOR_PWM_MAX]）
 * 设计说明：
 *   加入 target_speed 作为前馈项，使稳态时输出能直接维持目标速度，
 *   PD部分仅用于纠正误差，有效消除静差并提高响应速度。
 */
float Speed_PD_Control(Tracking_PDController *pid, float target_speed, float actual_speed)
{
    if(pid == 0)
    {
        return 0.0f;
    }

    /* 计算速度误差：目标速度 - 实际速度 */
    pid->error = target_speed - actual_speed;

    /*
     * 位置式PD + 前馈控制：
     * output = target_speed（前馈） + KP*error（比例） + KD*(error - last_error)（微分）
     * 前馈项确保稳态时输出能直接维持目标速度，消除静差
     */
    pid->output = target_speed +
                  (pid->kp * pid->error) +
                  (pid->kd * (pid->error - pid->last_error));

    /* 更新上一次误差（供下次微分计算） */
    pid->last_error = pid->error;

    /* PWM输出限幅 */
    return Limit_Float(pid->output, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);
}

/*=============================================
 * 初始化/重置函数：Tracking_Reset
 *=============================================*/
/*
 * 函数名：Tracking_Reset
 * 功  能：重置所有循迹控制状态变量
 * 调用时机：系统上电初始化、手动复位、停车后重新启动
 */
void Tracking_Reset(void)
{
    /* 全局变量重置 */
    line_lost = 0u;
    tracking_state.sensors = 0u;
    tracking_state.error = 0.0f;
    tracking_state.turn = 0.0f;
    tracking_state.left_target = 0.0f;
    tracking_state.right_target = 0.0f;
    tracking_state.left_pwm = 0.0f;
    tracking_state.right_pwm = 0.0f;

    /* 左轮速度PID状态重置 */
    left_speed_pid.error = 0.0f;
    left_speed_pid.last_error = 0.0f;
    left_speed_pid.output = 0.0f;

    /* 右轮速度PID状态重置 */
    right_speed_pid.error = 0.0f;
    right_speed_pid.last_error = 0.0f;
    right_speed_pid.output = 0.0f;

    /* 外环状态重置 */
    last_valid_tracking_error = 0.0f;
    outer_error_previous = 0.0f;
}

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
