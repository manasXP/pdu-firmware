/**
 * @file    app_control.c
 * @brief   PFC dq-frame control, LLC PFM, PI with anti-windup
 */

#include "app_control.h"

/* PI controller instances */
static PI_Controller_t s_pi_id;    /* PFC d-axis current */
static PI_Controller_t s_pi_iq;    /* PFC q-axis current */
static PI_Controller_t s_pi_vbus;  /* PFC bus voltage outer loop */
static PI_Controller_t s_pi_vllc;  /* LLC output voltage */
static PI_Controller_t s_pi_illc;  /* LLC output current */

void App_Control_Init(void)
{
    /* PFC current loop: BW ~1.5 kHz, L=200 uH */
    s_pi_id = (PI_Controller_t){
        .Kp = 0.94f, .Ki = 1776.0f, .Tt = 0.001f,
        .out_min = -1.0f, .out_max = 1.0f,
        .integrator = 0.0f, .prev_error = 0.0f
    };
    s_pi_iq = s_pi_id;

    /* Bus voltage loop: BW ~30 Hz */
    s_pi_vbus = (PI_Controller_t){
        .Kp = 0.1f, .Ki = 10.0f, .Tt = 0.01f,
        .out_min = 0.0f, .out_max = 60.0f,
        .integrator = 0.0f, .prev_error = 0.0f
    };

    /* LLC voltage loop */
    s_pi_vllc = (PI_Controller_t){
        .Kp = 0.005f, .Ki = 5.0f, .Tt = 0.01f,
        .out_min = 100000.0f, .out_max = 300000.0f,
        .integrator = 0.0f, .prev_error = 0.0f
    };

    /* LLC current loop */
    s_pi_illc = (PI_Controller_t){
        .Kp = 0.01f, .Ki = 10.0f, .Tt = 0.01f,
        .out_min = 100000.0f, .out_max = 300000.0f,
        .integrator = 0.0f, .prev_error = 0.0f
    };
}

/**
 * @brief  PI controller with back-calculation anti-windup
 * @param  pi        Controller instance
 * @param  setpoint  Desired value
 * @param  measurement  Measured value
 * @param  dt        Time step (s)
 * @return Control output (clamped)
 */
float PI_Update(PI_Controller_t *pi, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    /* Proportional + integrator */
    float output = pi->Kp * error + pi->integrator;

    /* Clamp output */
    float output_sat = output;
    if (output_sat > pi->out_max)
        output_sat = pi->out_max;
    if (output_sat < pi->out_min)
        output_sat = pi->out_min;

    /* Anti-windup: back-calculation */
    float aw = (output_sat - output) * pi->Tt;
    pi->integrator += (pi->Ki * error + aw) * dt;

    pi->prev_error = error;
    return output_sat;
}

void PI_Reset(PI_Controller_t *pi)
{
    pi->integrator = 0.0f;
    pi->prev_error = 0.0f;
}

void App_Control_PFC_ISR(void)
{
    /* TODO: Clarke -> Park -> PI_id/PI_iq -> inv Park -> SVM -> HRTIM duty */
}

void App_Control_LLC_ISR(void)
{
    /* TODO: CC/CV mode select -> PI_vllc or PI_illc -> frequency -> HRTIM PERxR */
}
