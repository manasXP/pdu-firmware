/**
 * @file    app_pll.h
 * @brief   Synchronous Reference Frame PLL for 3-phase grid synchronization
 *
 * Provides grid angle (theta) and frequency (omega) estimation using a
 * Clarke-Park transform and PI controller on the q-axis voltage.
 * Called at 65 kHz from the PFC ISR.
 */

#ifndef APP_PLL_H
#define APP_PLL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ------------------------------------------------------------------ */
/*  PLL Configuration Constants                                        */
/* ------------------------------------------------------------------ */

/** @brief  V_q lock threshold (V) — PLL is locked when |V_q| < this */
#define PLL_VQ_THRESHOLD       5.0f

/** @brief  Number of consecutive samples below threshold for lock */
#define PLL_LOCK_COUNT         20U

/** @brief  Maximum omega ramp rate (rad/s^2) */
#define PLL_OMEGA_RAMP_RATE    500.0f

/** @brief  Feed-forward omega for 50 Hz grid (rad/s) */
#define PLL_OMEGA_FF_50HZ      314.159265f

/* PI tuning for 30 Hz bandwidth, damping ratio 0.707 */
#define PLL_PI_KP              266.6f
#define PLL_PI_KI              35532.0f

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

/**
 * @brief  Initialize PLL state and PI controller
 */
void App_PLL_Init(void);

/**
 * @brief  Run one PLL iteration (Clarke -> Park -> PI -> integrate)
 * @param  v_a  Grid voltage phase A (V)
 * @param  v_b  Grid voltage phase B (V)
 * @param  dt   Time step (s), typically 1/65000
 */
void App_PLL_Update(float v_a, float v_b, float dt);

/**
 * @brief  Check if PLL has achieved lock
 * @return 1 if locked, 0 otherwise
 */
uint8_t App_PLL_IsLocked(void);

/**
 * @brief  Get estimated grid angle
 * @return theta_hat in [0, 2*pi) radians
 */
float App_PLL_GetTheta(void);

/**
 * @brief  Get estimated grid angular frequency
 * @return omega_hat in rad/s
 */
float App_PLL_GetOmega(void);

/**
 * @brief  Get d-axis voltage (grid voltage magnitude when locked)
 * @return V_d in volts
 */
float App_PLL_GetVd(void);

/**
 * @brief  Get q-axis voltage (should be ~0 when locked)
 * @return V_q in volts
 */
float App_PLL_GetVq(void);

/**
 * @brief  Reset PLL state for a fresh lock attempt
 */
void App_PLL_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_PLL_H */
