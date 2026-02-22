/**
 * @file    app_powerseq.c
 * @brief   PFC/LLC soft-start, shutdown, burst mode
 */

#include "app_powerseq.h"
#include "app_control.h"
#include "app_adc.h"
#include "app_diagnostics.h"
#include "app_protection.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/*  LLC Soft-Start Constants                                           */
/* ------------------------------------------------------------------ */

/** @brief  Phase stagger interval (ms) — 100 ms between each phase enable */
#define LLC_PHASE_STAGGER_MS   100U

/** @brief  Frequency ramp step (Hz) and settle time (ms) — reuse sweep params */
#define LLC_RAMP_STEP_HZ       2000U
#define LLC_RAMP_SETTLE_MS     50U

/* ------------------------------------------------------------------ */
/*  LLC Soft-Start State                                               */
/* ------------------------------------------------------------------ */

typedef struct
{
    uint32_t tick_count;        /* ms since Begin()              */
    uint32_t freq_hz;           /* current switching frequency   */
    uint32_t ramp_settle;       /* settle counter for freq ramp  */
    uint8_t  phase_d_on;        /* phase D enabled               */
    uint8_t  phase_e_on;        /* phase E enabled               */
    uint8_t  phase_f_on;        /* phase F enabled               */
    uint8_t  contactor_closed;  /* LLC contactor state           */
    uint8_t  complete;          /* soft-start done flag          */
    float    v_target;          /* target output voltage         */
} LLC_SoftStart_t;

static LLC_SoftStart_t s_llc_ss;

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

void App_PowerSeq_Init(void)
{
    (void)memset(&s_llc_ss, 0, sizeof(s_llc_ss));
}

void PFC_SoftStart_Begin(void)
{
    /* TODO: I_d* = 0, anti-windup pre-load, start ramp timer */
}

uint8_t PFC_SoftStart_Tick(void)
{
    /* TODO: Linear ramp I_d* over 200 ms, NTC bypass at 80% Vbus */
    return 0;
}

/**
 * @brief  Begin LLC soft-start sequence
 *
 * Zeroes state, sets frequency to 300 kHz, and enables phase D.
 * Phase E and F are staggered 100 ms apart by LLC_SoftStart_Tick().
 */
void LLC_SoftStart_Begin(void)
{
    (void)memset(&s_llc_ss, 0, sizeof(s_llc_ss));
    s_llc_ss.freq_hz    = LLC_FREQ_MAX_HZ;
    s_llc_ss.v_target   = LLC_SOFTSTART_TARGET_V;
    s_llc_ss.ramp_settle = LLC_RAMP_SETTLE_MS;

    App_Control_LLC_SetFrequency(LLC_FREQ_MAX_HZ);
    App_Control_LLC_StartPhase(0U);
    s_llc_ss.phase_d_on = 1U;

    App_Diagnostics_Log("[PS] LLC soft-start begin");
}

/**
 * @brief  Advance LLC soft-start — call at 1 kHz from state machine
 *
 * Sequence:
 *   tick 100: enable phase E
 *   tick 200: enable phase F
 *   After all phases on: ramp frequency down by 2 kHz every 50 ms
 *   When V_out within +/-5% of target: close contactor, done
 *   Safety: abort on OVP or OCP
 *
 * @return 1 when soft-start is complete, 0 otherwise
 */
uint8_t LLC_SoftStart_Tick(void)
{
    if (s_llc_ss.complete != 0U)
    {
        return 1U;
    }

    s_llc_ss.tick_count++;

    /* Stagger phase enables */
    if ((s_llc_ss.tick_count == LLC_PHASE_STAGGER_MS)
        && (s_llc_ss.phase_e_on == 0U))
    {
        App_Control_LLC_StartPhase(1U);
        s_llc_ss.phase_e_on = 1U;
        App_Diagnostics_Log("[PS] LLC phase E on");
    }

    if ((s_llc_ss.tick_count == (LLC_PHASE_STAGGER_MS * 2U))
        && (s_llc_ss.phase_f_on == 0U))
    {
        App_Control_LLC_StartPhase(2U);
        s_llc_ss.phase_f_on = 1U;
        App_Diagnostics_Log("[PS] LLC phase F on");
    }

    /* Read ADC */
    const ADC_Readings_t *adc = App_ADC_GetReadings();
    float v_out = adc->v_out;
    float i_out = adc->i_out;

    /* Safety check — abort on OVP or OCP */
    if ((v_out > OUT_OVP_THRESHOLD_V) || (i_out > PFC_OCP_THRESHOLD_A))
    {
        App_Control_LLC_Stop();
        HAL_GPIO_WritePin(RELAY_LLC_PORT, RELAY_LLC_PIN, GPIO_PIN_RESET);
        s_llc_ss.complete = 1U;
        App_Diagnostics_Log("[PS] LLC soft-start ABORT: safety limit");
        Fault_Enter(FAULT_OUT_OVP_SW);
        return 1U;
    }

    /* Frequency ramp — only after all phases are on */
    if ((s_llc_ss.phase_d_on != 0U) && (s_llc_ss.phase_e_on != 0U)
        && (s_llc_ss.phase_f_on != 0U))
    {
        if (s_llc_ss.ramp_settle > 0U)
        {
            s_llc_ss.ramp_settle--;
        }
        else
        {
            /* Check if V_out is within tolerance of target */
            float v_low  = s_llc_ss.v_target * (1.0f - LLC_SOFTSTART_TOLERANCE);
            float v_high = s_llc_ss.v_target * (1.0f + LLC_SOFTSTART_TOLERANCE);

            if ((v_out >= v_low) && (v_out <= v_high))
            {
                /* Target reached — close contactor */
                HAL_GPIO_WritePin(RELAY_LLC_PORT, RELAY_LLC_PIN,
                                  GPIO_PIN_SET);
                s_llc_ss.contactor_closed = 1U;
                s_llc_ss.complete         = 1U;
                App_Diagnostics_Log("[PS] LLC soft-start complete");
                return 1U;
            }

            /* Hit frequency floor without reaching target — fault */
            if (s_llc_ss.freq_hz <= (LLC_FREQ_MIN_HZ + LLC_RAMP_STEP_HZ))
            {
                App_Control_LLC_Stop();
                HAL_GPIO_WritePin(RELAY_LLC_PORT, RELAY_LLC_PIN,
                                  GPIO_PIN_RESET);
                s_llc_ss.complete = 1U;
                App_Diagnostics_Log("[PS] LLC soft-start ABORT: freq floor");
                Fault_Enter(FAULT_STARTUP_TIMEOUT);
                return 1U;
            }

            /* Step frequency down */
            s_llc_ss.freq_hz -= LLC_RAMP_STEP_HZ;
            App_Control_LLC_SetFrequency(s_llc_ss.freq_hz);
            s_llc_ss.ramp_settle = LLC_RAMP_SETTLE_MS;
        }
    }

    return 0U;
}

void Shutdown_Begin(void)
{
    /* TODO: Ramp I_out to 0, disable outputs, open contactor */
}

uint8_t Shutdown_Tick(void)
{
    /* TODO: Reverse startup sequence */
    return 0;
}

void Burst_Mode_Tick(void)
{
    /* TODO: HRTIM burst controller, integrator freeze during idle */
}
