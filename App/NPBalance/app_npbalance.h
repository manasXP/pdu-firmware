/**
 * @file    app_npbalance.h
 * @brief   Neutral point balancing — P-controller with zero-sequence SVM injection
 *
 * Monitors split-capacitor voltage imbalance (V_cap_top - V_cap_bot) and
 * computes a zero-sequence duty offset for the PFC SVM modulator.
 */

#ifndef APP_NPBALANCE_H
#define APP_NPBALANCE_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief  Initialize neutral point balancing module — zero state
 */
void App_NPBalance_Init(void);

/**
 * @brief  Update neutral point balance controller — call at 1 kHz
 *
 * Reads v_cap_top and v_cap_bot from ADC, computes P-controller offset,
 * and monitors for warning/fault conditions.
 */
void NP_Balance_Update(void);

/**
 * @brief  Get current zero-sequence duty offset
 * @return d_offset value in [-0.05, +0.05]
 */
float NP_Balance_GetOffset(void);

/**
 * @brief  Get current neutral point voltage error
 * @return v_cap_top - v_cap_bot (V)
 */
float NP_Balance_GetError(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_NPBALANCE_H */
