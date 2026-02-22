/**
 * @file    app_transforms.h
 * @brief   Clarke/Park transforms and SVM using CORDIC co-processor
 *
 * Shared transform functions for PFC dq-frame control. Uses the
 * STM32G474 CORDIC hardware accelerator for sin/cos computation
 * (< 50 ns, 6-cycle precision).
 */

#ifndef APP_TRANSFORMS_H
#define APP_TRANSFORMS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ------------------------------------------------------------------ */
/*  Types                                                              */
/* ------------------------------------------------------------------ */

/** @brief  Alpha-beta stationary frame quantities */
typedef struct
{
    float alpha;
    float beta;
} AlphaBeta_t;

/** @brief  Direct-quadrature rotating frame quantities */
typedef struct
{
    float d;
    float q;
} DQ_t;

/** @brief  Sine and cosine pair from CORDIC */
typedef struct
{
    float sin;
    float cos;
} SinCos_t;

/** @brief  Three-phase PWM duty cycles */
typedef struct
{
    float a;
    float b;
    float c;
} DutyABC_t;

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

/**
 * @brief  Initialize CORDIC for cosine function with 6-cycle precision
 *
 * Configures CSR: COSINE function, q1.31 format, 6 cycles, 1 write / 2 reads.
 * Must be called after MX_CORDIC_Init() and before any transform usage.
 */
void App_Transforms_Init(void);

/**
 * @brief  Compute sin/cos using CORDIC hardware accelerator
 * @param  theta  Angle in radians [0, 2*pi)
 * @return SinCos_t with sin and cos fields
 */
SinCos_t Transforms_CORDIC_SinCos(float theta);

/**
 * @brief  Clarke transform — 3-phase to alpha-beta (balanced assumption)
 * @param  a  Phase A quantity
 * @param  b  Phase B quantity
 * @return AlphaBeta_t result
 *
 * alpha = a
 * beta  = (a + 2*b) / sqrt(3)
 */
AlphaBeta_t Transforms_Clarke(float a, float b);

/**
 * @brief  Park transform — alpha-beta to dq rotating frame
 * @param  ab  Alpha-beta input
 * @param  sc  Sin/cos of grid angle
 * @return DQ_t result
 *
 * d =  alpha*cos + beta*sin
 * q = -alpha*sin + beta*cos
 */
DQ_t Transforms_Park(AlphaBeta_t ab, SinCos_t sc);

/**
 * @brief  Inverse Park transform — dq to alpha-beta
 * @param  dq  DQ input
 * @param  sc  Sin/cos of grid angle
 * @return AlphaBeta_t result
 *
 * alpha = d*cos - q*sin
 * beta  = d*sin + q*cos
 */
AlphaBeta_t Transforms_InversePark(DQ_t dq, SinCos_t sc);

/**
 * @brief  Space Vector Modulation — alpha-beta voltage to 3-phase duties
 * @param  v_alpha    Alpha-axis voltage command
 * @param  v_beta     Beta-axis voltage command
 * @param  v_dc       DC bus voltage
 * @param  max_duty   Maximum duty cycle (e.g. 0.95)
 * @return DutyABC_t  Per-phase duty cycles clamped to [0, max_duty]
 *
 * Implements 7-segment symmetric SVM with overmodulation clamp.
 */
DutyABC_t Transforms_SVM(float v_alpha, float v_beta, float v_dc,
                          float max_duty);

#ifdef __cplusplus
}
#endif

#endif /* APP_TRANSFORMS_H */
