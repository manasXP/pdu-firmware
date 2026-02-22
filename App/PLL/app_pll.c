/**
 * @file    app_pll.c
 * @brief   SRF-PLL implementation — grid angle and frequency estimation
 *
 * Algorithm (each call at 65 kHz):
 *   1. Clarke transform:  V_alpha, V_beta from V_a, V_b
 *   2. Park transform:    V_d, V_q using estimated theta
 *   3. PI on V_q:         drives V_q -> 0, output is omega correction
 *   4. Omega rate limiter: clamp |d_omega/dt| <= 500 rad/s^2
 *   5. Theta integration: theta += omega * dt, wrap to [0, 2*pi)
 *   6. Lock detection:    counter-based on |V_q| < threshold
 */

#include "app_pll.h"
#include "app_control.h"
#include <math.h>

/* ------------------------------------------------------------------ */
/*  Constants                                                          */
/* ------------------------------------------------------------------ */

#define TWO_PI          6.283185307f
#define INV_SQRT3       0.577350269f    /* 1 / sqrt(3) */

/* PI output limits: omega correction range (rad/s) */
#define PLL_PI_OUT_MIN  (-200.0f)
#define PLL_PI_OUT_MAX  200.0f

/* ------------------------------------------------------------------ */
/*  Module State                                                       */
/* ------------------------------------------------------------------ */

static float            s_theta_hat;
static float            s_omega_hat;
static float            s_vd;
static float            s_vq;
static uint32_t         s_lock_counter;
static uint8_t          s_locked;
static PI_Controller_t  s_pi_pll;

/* Previous omega for rate limiting */
static float            s_omega_prev;

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

void App_PLL_Init(void)
{
    App_PLL_Reset();
}

void App_PLL_Update(float v_a, float v_b, float dt)
{
    /* 1. Clarke transform (two-phase from three-phase, balanced assumption)
     *    V_alpha = V_a
     *    V_beta  = (V_a + 2 * V_b) / sqrt(3)
     */
    float v_alpha = v_a;
    float v_beta  = (v_a + 2.0f * v_b) * INV_SQRT3;

    /* 2. Park transform using estimated theta
     *    V_d =  V_alpha * cos(theta) + V_beta * sin(theta)
     *    V_q = -V_alpha * sin(theta) + V_beta * cos(theta)
     */
    float cos_theta = cosf(s_theta_hat);
    float sin_theta = sinf(s_theta_hat);

    s_vd =  v_alpha * cos_theta + v_beta * sin_theta;
    s_vq = -v_alpha * sin_theta + v_beta * cos_theta;

    /* 3. PI controller on V_q — drives V_q toward zero
     *    Output is the omega correction term
     */
    float omega_corr = PI_Update(&s_pi_pll, 0.0f, s_vq, dt);

    /* 4. Compute desired omega with feed-forward + PI correction */
    float omega_new = PLL_OMEGA_FF_50HZ + omega_corr;

    /* Rate limiter: clamp |d_omega/dt| <= PLL_OMEGA_RAMP_RATE */
    float d_omega = omega_new - s_omega_prev;
    float d_omega_max = PLL_OMEGA_RAMP_RATE * dt;

    if (d_omega > d_omega_max)
    {
        omega_new = s_omega_prev + d_omega_max;
    }
    else if (d_omega < -d_omega_max)
    {
        omega_new = s_omega_prev - d_omega_max;
    }

    s_omega_hat  = omega_new;
    s_omega_prev = omega_new;

    /* 5. Theta integration with wrap to [0, 2*pi) */
    s_theta_hat += s_omega_hat * dt;

    if (s_theta_hat >= TWO_PI)
    {
        s_theta_hat -= TWO_PI;
    }
    else if (s_theta_hat < 0.0f)
    {
        s_theta_hat += TWO_PI;
    }

    /* 6. Lock detection: |V_q| < threshold for N consecutive samples */
    if (fabsf(s_vq) < PLL_VQ_THRESHOLD)
    {
        if (s_lock_counter < PLL_LOCK_COUNT)
        {
            s_lock_counter++;
        }

        if (s_lock_counter >= PLL_LOCK_COUNT)
        {
            s_locked = 1U;
        }
    }
    else
    {
        s_lock_counter = 0U;
        s_locked = 0U;
    }
}

uint8_t App_PLL_IsLocked(void)
{
    return s_locked;
}

float App_PLL_GetTheta(void)
{
    return s_theta_hat;
}

float App_PLL_GetOmega(void)
{
    return s_omega_hat;
}

float App_PLL_GetVd(void)
{
    return s_vd;
}

float App_PLL_GetVq(void)
{
    return s_vq;
}

void App_PLL_Reset(void)
{
    s_theta_hat    = 0.0f;
    s_omega_hat    = 0.0f;
    s_omega_prev   = 0.0f;
    s_vd           = 0.0f;
    s_vq           = 0.0f;
    s_lock_counter = 0U;
    s_locked       = 0U;

    s_pi_pll = (PI_Controller_t){
        .Kp       = PLL_PI_KP,
        .Ki       = PLL_PI_KI,
        .Tt       = 1.0f / PLL_PI_KI,    /* Anti-windup tracking */
        .out_min  = PLL_PI_OUT_MIN,
        .out_max  = PLL_PI_OUT_MAX,
        .integrator = 0.0f,
        .prev_error = 0.0f
    };
}
