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

/* PFC HRTIM configuration and control */
void App_Control_HRTIM_PFC_Init(void);
void App_Control_PFC_Start(void);
void App_Control_PFC_Stop(void);
void App_Control_PFC_SetDuty(float duty);
void App_Control_PFC_ISR(void);

/* LLC HRTIM configuration and control */
void    App_Control_HRTIM_LLC_Init(void);
void    App_Control_LLC_Start(void);
void    App_Control_LLC_Stop(void);
void    App_Control_LLC_SetFrequency(uint32_t freq_hz);
void    App_Control_LLC_SweepStart(void);
void    App_Control_LLC_Sweep(void);
uint8_t App_Control_LLC_SweepDone(void);
void    App_Control_LLC_ISR(void);

#endif /* APP_CONTROL_H */
