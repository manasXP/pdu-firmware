/**
 * @file    app_control.h
 * @brief   PFC dq-frame current control, LLC PFM, PI controllers
 */

#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include <stdint.h>

/* PI controller instance */
typedef struct
{
    float Kp;
    float Ki;
    float Tt;          /* Anti-windup tracking time constant */
    float out_min;
    float out_max;
    float integrator;
    float prev_error;
} PI_Controller_t;

void  App_Control_Init(void);
float PI_Update(PI_Controller_t *pi, float setpoint, float measurement, float dt);
void  PI_Reset(PI_Controller_t *pi);

/* PFC control */
void App_Control_PFC_ISR(void);

/* LLC control */
void App_Control_LLC_ISR(void);

#endif /* APP_CONTROL_H */
