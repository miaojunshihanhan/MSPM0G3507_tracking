#include "motor.h"
#include "board.h"

/******************************************************************
 * 函 数 名 称：TB6612_Motor_Stop
 * 函 数 说 明：A端和B端电机停止
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LCKFB
 * 备       注：无
******************************************************************/
void TB6612_Motor_Stop(void)
{
    AIN1_OUT(1);
    AIN2_OUT(1);
    BIN1_OUT(1);
    BIN2_OUT(1);
}

/******************************************************************
 * 函 数 名 称：AO_Control
 * 函 数 说 明：A端口电机控制
 * 函 数 形 参：dir旋转方向 1正转0反转   speed旋转速度，范围（0 ~ per-1）
 * 函 数 返 回：无
 * 作       者：LCKFB
 * 备       注：speed 0-1000
******************************************************************/
void AO_Control(uint8_t dir, uint32_t speed)
{
    if(speed > 999 || dir > 1)
    {
        return;
    }

    if( dir == 1 )
    {
        AIN1_OUT(0);
        AIN2_OUT(1);
    }
    else
    {
        AIN1_OUT(1);
        AIN2_OUT(0);
    }

    DL_TimerG_setCaptureCompareValue(PWM_0_INST, speed, GPIO_PWM_0_C1_IDX);
}
/******************************************************************
 * 函 数 名 称：BO_Control
 * 函 数 说 明：B端口电机控制
 * 函 数 形 参：dir旋转方向 1正转0反转   speed旋转速度，范围（0 ~ per-1）
 * 函 数 返 回：无
 * 作       者：LCKFB
 * 备       注：speed 0-1000
******************************************************************/
void BO_Control(uint8_t dir, uint32_t speed)
{
    if(speed > 999 || dir > 1)
    {
        return;
    }

    if( dir == 1 )
    {
        BIN1_OUT(0);
        BIN2_OUT(1);
    }
    else
    {
        BIN1_OUT(1);
        BIN2_OUT(0);
    }

    DL_TimerG_setCaptureCompareValue(PWM_0_INST, speed, GPIO_PWM_0_C0_IDX);
}
