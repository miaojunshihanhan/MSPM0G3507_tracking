#ifndef _TRACKING_H
#define _TRACKING_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

/*
 * Default parameters. tracking.c defines the same names before including this
 * header so normal tuning only needs editing the parameter block at the top of
 * tracking.c.
 */
#ifndef BASE_SPEED
#define BASE_SPEED                  (250.0f)
#endif

#ifndef KP_OUT
#define KP_OUT                      (80.0f)
#endif

#ifndef KD_OUT
#define KD_OUT                      (120.0f)
#endif

#ifndef TURN_MAX
#define TURN_MAX                    (300.0f)
#endif

#ifndef KP_SPEED_L
#define KP_SPEED_L                  (1.2f)
#endif

#ifndef KD_SPEED_L
#define KD_SPEED_L                  (0.4f)
#endif

#ifndef KP_SPEED_R
#define KP_SPEED_R                  (1.2f)
#endif

#ifndef KD_SPEED_R
#define KD_SPEED_R                  (0.4f)
#endif

#ifndef MOTOR_PWM_MAX
#define MOTOR_PWM_MAX               (999.0f)
#endif

#ifndef TARGET_SPEED_MAX
#define TARGET_SPEED_MAX            (999.0f)
#endif

#ifndef TRACKING_FORWARD_DIR
#define TRACKING_FORWARD_DIR        (1u)
#endif

#ifndef TRACKING_REVERSE_DIR
#define TRACKING_REVERSE_DIR        (0u)
#endif

typedef struct
{
    float kp;
    float kd;
    float error;
    float last_error;
    float output;
} Tracking_PDController;

typedef struct
{
    uint8_t sensors;
    float error;
    float turn;
    float left_target;
    float right_target;
    float left_pwm;
    float right_pwm;
} Tracking_State;

extern uint8_t line_lost;
extern uint8_t tracking_enabled;
extern Tracking_State tracking_state;
extern volatile int32_t speedA;
extern volatile int32_t speedB;

void Tracking(void);
uint8_t Tracking_ReadSensors(void);
float get_error(void);
float Outer_PD_Control(float error);
float Speed_PD_Control(Tracking_PDController *pid, float target_speed, float actual_speed);
void Tracking_Reset(void);

#endif
