/**
 * @file    app_npbalance.c
 * @brief   Neutral point balancing — P-controller with zero-sequence injection
 *
 * The Vienna rectifier splits the DC bus into two series capacitors.
 * This module monitors V_cap_top - V_cap_bot and computes a zero-sequence
 * duty offset (d_offset) that is applied to the PFC SVM modulator to
 * correct midpoint drift.
 *
 * Runs at 1 kHz in thread mode (STATE_RUN and STATE_DERATE).
 * Only the result (g_np_zs_offset) is consumed in the 65 kHz PFC ISR.
 */

#include "app_npbalance.h"
#include "app_adc.h"
#include "app_diagnostics.h"
#include "app_protection.h"

/* ------------------------------------------------------------------ */
/*  Constants                                                          */
/* ------------------------------------------------------------------ */

/** @brief  Proportional gain for NP balance controller */
#define KNP_GAIN                0.005f

/** @brief  Maximum zero-sequence duty offset magnitude */
#define NP_OFFSET_MAX           0.05f

/** @brief  Warning threshold: |V_NP_err| > 20 V */
#define NP_WARN_THRESHOLD_V     20.0f

/** @brief  Warning debounce: 1 s at 1 kHz = 1000 ticks */
#define NP_WARN_DEBOUNCE_TICKS  1000U

/** @brief  Fault threshold: |V_NP_err| > 50 V */
#define NP_FAULT_THRESHOLD_V    50.0f

/** @brief  Fault debounce: 100 ms at 1 kHz = 100 ticks */
#define NP_FAULT_DEBOUNCE_TICKS 100U

/* ------------------------------------------------------------------ */
/*  Module state                                                       */
/* ------------------------------------------------------------------ */

/** @brief  Internal state for NP balance controller */
typedef struct
{
    float    d_offset;     /**< Zero-sequence duty offset */
    float    v_np_err;     /**< Voltage error: V_cap_top - V_cap_bot */
    uint32_t warn_timer;   /**< Warning debounce counter (ticks) */
    uint32_t fault_timer;  /**< Fault debounce counter (ticks) */
} NP_Balance_t;

static NP_Balance_t s_np;

/**
 * @brief  Shared zero-sequence offset — written here, read in PFC ISR.
 *
 * On Cortex-M4, aligned 32-bit float loads/stores are naturally atomic.
 */
volatile float g_np_zs_offset;

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

void App_NPBalance_Init(void)
{
    s_np.d_offset    = 0.0f;
    s_np.v_np_err    = 0.0f;
    s_np.warn_timer  = 0U;
    s_np.fault_timer = 0U;
    g_np_zs_offset   = 0.0f;
}

void NP_Balance_Update(void)
{
    const ADC_Readings_t *adc = App_ADC_GetReadings();

    /* Compute neutral point error */
    s_np.v_np_err = adc->v_cap_top - adc->v_cap_bot;

    /* P-controller: d_offset = K_NP * error */
    float offset = KNP_GAIN * s_np.v_np_err;

    /* Clamp to [-0.05, +0.05] */
    if (offset > NP_OFFSET_MAX)
    {
        offset = NP_OFFSET_MAX;
    }
    if (offset < -NP_OFFSET_MAX)
    {
        offset = -NP_OFFSET_MAX;
    }

    s_np.d_offset = offset;

    /* Atomic store — consumed by PFC ISR */
    g_np_zs_offset = offset;

    /* Absolute error for threshold checks */
    float abs_err = s_np.v_np_err;
    if (abs_err < 0.0f)
    {
        abs_err = -abs_err;
    }

    /* Warning monitor: |err| > 20 V for > 1 s */
    if (abs_err > NP_WARN_THRESHOLD_V)
    {
        s_np.warn_timer++;
        if (s_np.warn_timer == NP_WARN_DEBOUNCE_TICKS)
        {
            App_Diagnostics_Log("[NP] Imbalance warning");
        }
    }
    else
    {
        s_np.warn_timer = 0U;
    }

    /* Fault monitor: |err| > 50 V for > 100 ms */
    if (abs_err > NP_FAULT_THRESHOLD_V)
    {
        s_np.fault_timer++;
        if (s_np.fault_timer >= NP_FAULT_DEBOUNCE_TICKS)
        {
            Fault_Enter(FAULT_NP_IMBALANCE);
            s_np.fault_timer = NP_FAULT_DEBOUNCE_TICKS; /* Saturate */
        }
    }
    else
    {
        s_np.fault_timer = 0U;
    }
}

float NP_Balance_GetOffset(void)
{
    return s_np.d_offset;
}

float NP_Balance_GetError(void)
{
    return s_np.v_np_err;
}
