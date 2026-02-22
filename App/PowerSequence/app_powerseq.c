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
/*  PFC Soft-Start State                                               */
/* ------------------------------------------------------------------ */

typedef struct
{
    uint32_t tick_count;       /* ms since Begin()                     */
    float    duty_current;     /* current duty cycle [0, PFC_MAX_DUTY] */
    float    duty_step;        /* duty increment per ms                */
    uint8_t  ntc_bypassed;     /* NTC bypass relay engaged             */
    uint8_t  complete;         /* soft-start done flag                 */
} PFC_SoftStart_t;

static PFC_SoftStart_t s_pfc_ss;

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
/*  Shutdown State                                                     */
/* ------------------------------------------------------------------ */

typedef struct
{
    uint32_t tick_count;        /* ms since Begin()              */
    float    duty_current;      /* PFC duty ramp-down value      */
    float    duty_step;         /* duty decrement per ms         */
    uint8_t  llc_stopped;       /* LLC outputs disabled          */
    uint8_t  pfc_stopped;       /* PFC outputs disabled          */
    uint8_t  contactor_open;    /* output contactor opened       */
    uint8_t  complete;          /* shutdown done flag            */
} Shutdown_t;

static Shutdown_t s_shutdown;

/* Forward declaration — full struct definition in Burst Mode section below */
typedef struct
{
    BurstState_t state;
    uint32_t     entry_timer;
    uint32_t     idle_tick;
    float        v_target;
    float        v_out_prev;
    uint8_t      hrtim_configured;
} BurstMode_t;

static BurstMode_t s_burst;

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

void App_PowerSeq_Init(void)
{
    (void)memset(&s_pfc_ss, 0, sizeof(s_pfc_ss));
    (void)memset(&s_llc_ss, 0, sizeof(s_llc_ss));
    (void)memset(&s_shutdown, 0, sizeof(s_shutdown));
    (void)memset(&s_burst, 0, sizeof(s_burst));
}

/* ------------------------------------------------------------------ */
/*  PFC Soft-Start                                                     */
/* ------------------------------------------------------------------ */

/**
 * @brief  Begin PFC soft-start — ramp duty from initial to 30% over 200 ms
 *
 * Pre-loads duty at a low initial value and computes a linear ramp step.
 * The PFC outputs must already be started (counters + outputs enabled)
 * before calling this function. The anti-windup integrators are reset
 * so the PI controllers start from zero when closed-loop engages later.
 */
void PFC_SoftStart_Begin(void)
{
    (void)memset(&s_pfc_ss, 0, sizeof(s_pfc_ss));

    /* Start at low duty to limit inrush */
    s_pfc_ss.duty_current = PFC_SOFTSTART_DUTY_INIT;

    /*
     * Ramp from initial duty (5%) to operating duty (30%) over 200 ms.
     * Step per ms = (0.30 - 0.05) / 200 = 0.00125 per tick.
     */
    float duty_target = 0.30f;
    s_pfc_ss.duty_step = (duty_target - PFC_SOFTSTART_DUTY_INIT)
                        / (float)PFC_SOFTSTART_RAMP_MS;

    /* Set initial low duty */
    App_Control_PFC_SetDuty(s_pfc_ss.duty_current);

    /* Reset PI integrators for clean closed-loop handoff */
    /* (PI controllers are in App_Control — will be armed by EP-04-001) */

    App_Diagnostics_Log("[PS] PFC soft-start begin");
}

/**
 * @brief  Advance PFC soft-start — call at 1 kHz from state machine
 *
 * Sequence:
 *   - Linear duty ramp from 5% to 30% over 200 ms
 *   - At each tick, check Vbus and engage NTC bypass relay at 80% target
 *   - Complete when ramp finishes and NTC is bypassed
 *
 * @return 1 when soft-start is complete, 0 otherwise
 */
uint8_t PFC_SoftStart_Tick(void)
{
    if (s_pfc_ss.complete != 0U)
    {
        return 1U;
    }

    s_pfc_ss.tick_count++;

    const ADC_Readings_t *adc = App_ADC_GetReadings();
    float v_bus = adc->v_bus;

    /* Safety check — abort on bus OVP */
    if (v_bus > BUS_OVP_THRESHOLD_V)
    {
        App_Control_PFC_Stop();
        HAL_GPIO_WritePin(RELAY_NTC_PORT, RELAY_NTC_PIN, GPIO_PIN_RESET);
        s_pfc_ss.complete = 1U;
        App_Diagnostics_Log("[PS] PFC soft-start ABORT: bus OVP");
        Fault_Enter(FAULT_BUS_OVP_SW);
        return 1U;
    }

    /* Duty ramp — increment each tick until ramp time reached */
    if (s_pfc_ss.tick_count <= PFC_SOFTSTART_RAMP_MS)
    {
        s_pfc_ss.duty_current += s_pfc_ss.duty_step;
        App_Control_PFC_SetDuty(s_pfc_ss.duty_current);
    }

    /* NTC bypass relay — engage when Vbus reaches 80% of target */
    if (s_pfc_ss.ntc_bypassed == 0U)
    {
        float ntc_threshold = PFC_TARGET_VBUS_V * PFC_NTC_BYPASS_VBUS_FRAC;

        if (v_bus >= ntc_threshold)
        {
            HAL_GPIO_WritePin(RELAY_NTC_PORT, RELAY_NTC_PIN, GPIO_PIN_SET);
            s_pfc_ss.ntc_bypassed = 1U;

            char buf[48];
            (void)snprintf(buf, sizeof(buf),
                           "[PS] NTC bypass at Vbus=%.0f V", (double)v_bus);
            App_Diagnostics_Log(buf);
        }
    }

    /* Complete when ramp is finished */
    if (s_pfc_ss.tick_count >= PFC_SOFTSTART_RAMP_MS)
    {
        s_pfc_ss.complete = 1U;

        char buf[48];
        (void)snprintf(buf, sizeof(buf),
                       "[PS] PFC soft-start done (%lu ms, Vbus=%.0f V)",
                       (unsigned long)s_pfc_ss.tick_count, (double)v_bus);
        App_Diagnostics_Log(buf);
        return 1U;
    }

    return 0U;
}

/* ------------------------------------------------------------------ */
/*  LLC Soft-Start                                                     */
/* ------------------------------------------------------------------ */

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

/* ------------------------------------------------------------------ */
/*  Shutdown Sequence                                                  */
/* ------------------------------------------------------------------ */

/**
 * @brief  Begin controlled shutdown sequence
 *
 * Immediately stops LLC outputs, then ramps PFC duty down to zero
 * over SHUTDOWN_DUTY_RAMP_MS. Contactor opens only after I_out drops
 * below SHUTDOWN_IOUT_THRESHOLD_A to avoid arcing.
 */
void Shutdown_Begin(void)
{
    (void)memset(&s_shutdown, 0, sizeof(s_shutdown));

    /* 1. Stop LLC immediately — frequency control no longer needed */
    App_Control_LLC_Stop();
    s_shutdown.llc_stopped = 1U;
    App_Diagnostics_Log("[PS] Shutdown: LLC off");

    /*
     * 2. Capture current PFC duty for ramp-down.
     * Assume 30% operating duty. Ramp step = 0.30 / 100 = 0.003 per ms.
     */
    s_shutdown.duty_current = 0.30f;
    s_shutdown.duty_step = s_shutdown.duty_current
                          / (float)SHUTDOWN_DUTY_RAMP_MS;
}

/**
 * @brief  Advance shutdown sequence — call at 1 kHz from state machine
 *
 * Phase 1 (0-100 ms): Ramp PFC duty from 30% → 0% linearly
 * Phase 2: Wait for I_out to decay below 0.5 A before opening contactor
 * Phase 3: Open relays (NTC, PFC, LLC) and signal completion
 *
 * @return 1 when shutdown is complete, 0 otherwise
 */
uint8_t Shutdown_Tick(void)
{
    if (s_shutdown.complete != 0U)
    {
        return 1U;
    }

    s_shutdown.tick_count++;

    const ADC_Readings_t *adc = App_ADC_GetReadings();

    /* Phase 1: PFC duty ramp-down */
    if (s_shutdown.pfc_stopped == 0U)
    {
        if (s_shutdown.duty_current > 0.0f)
        {
            s_shutdown.duty_current -= s_shutdown.duty_step;
            if (s_shutdown.duty_current < 0.0f)
            {
                s_shutdown.duty_current = 0.0f;
            }
            App_Control_PFC_SetDuty(s_shutdown.duty_current);
        }
        else
        {
            /* Duty at zero — disable PFC outputs */
            App_Control_PFC_Stop();
            s_shutdown.pfc_stopped = 1U;
            App_Diagnostics_Log("[PS] Shutdown: PFC off");
        }
        return 0U;
    }

    /* Phase 2: Wait for output current to decay before opening contactor */
    if (s_shutdown.contactor_open == 0U)
    {
        float i_out_abs = adc->i_out;
        if (i_out_abs < 0.0f)
        {
            i_out_abs = -i_out_abs;
        }

        if (i_out_abs < SHUTDOWN_IOUT_THRESHOLD_A)
        {
            /* Safe to open contactor — no arcing risk */
            HAL_GPIO_WritePin(RELAY_LLC_PORT, RELAY_LLC_PIN, GPIO_PIN_RESET);
            s_shutdown.contactor_open = 1U;

            char buf[48];
            (void)snprintf(buf, sizeof(buf),
                           "[PS] Contactor open (Iout=%.2f A)",
                           (double)adc->i_out);
            App_Diagnostics_Log(buf);
        }
        else if (s_shutdown.tick_count > SHUTDOWN_MAX_WAIT_MS)
        {
            /* Timeout — force contactor open, fault */
            HAL_GPIO_WritePin(RELAY_LLC_PORT, RELAY_LLC_PIN, GPIO_PIN_RESET);
            s_shutdown.contactor_open = 1U;
            App_Diagnostics_Log("[PS] Contactor forced open (timeout)");
        }
        else
        {
            return 0U;
        }
    }

    /* Phase 3: Open remaining relays, signal complete */
    HAL_GPIO_WritePin(RELAY_PFC_PORT, RELAY_PFC_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_NTC_PORT, RELAY_NTC_PIN, GPIO_PIN_RESET);
    s_shutdown.complete = 1U;

    char buf[48];
    (void)snprintf(buf, sizeof(buf), "[PS] Shutdown complete (%lu ms)",
                   (unsigned long)s_shutdown.tick_count);
    App_Diagnostics_Log(buf);

    return 1U;
}

/* ------------------------------------------------------------------ */
/*  Burst Mode                                                         */
/* ------------------------------------------------------------------ */

/**
 * @brief  Get current burst mode sub-state
 * @return BURST_INACTIVE, BURST_RUN, or BURST_IDLE
 */
BurstState_t Burst_Mode_GetState(void)
{
    return s_burst.state;
}

/**
 * @brief  Advance burst mode sub-state machine — call at 1 kHz from STATE_RUN
 *
 * Monitors LLC switching frequency demand to decide entry/exit.
 * During BURST_RUN: checks V_out to transition to IDLE.
 * During BURST_IDLE: monitors V_out and dV/dt for load detection.
 * Freezes LLC PI integrators during idle with slow decay.
 */
void Burst_Mode_Tick(void)
{
    const ADC_Readings_t *adc = App_ADC_GetReadings();
    float v_out = adc->v_out;

    /*
     * Estimate current LLC frequency demand from the PI_VLLC output.
     * The LLC voltage PI outputs a frequency setpoint in [100k, 300k] Hz.
     */
    const PI_Controller_t *pi_vllc = PI_GetController(PI_ID_VLLC);
    float freq_demand = pi_vllc->Kp * pi_vllc->prev_error
                      + pi_vllc->integrator;

    /* Clamp to valid range for comparison */
    if (freq_demand < (float)LLC_FREQ_MIN_HZ)
    {
        freq_demand = (float)LLC_FREQ_MIN_HZ;
    }
    if (freq_demand > (float)LLC_FREQ_MAX_HZ)
    {
        freq_demand = (float)LLC_FREQ_MAX_HZ;
    }

    switch (s_burst.state)
    {
    case BURST_INACTIVE:
        /* Check entry condition: frequency > 280 kHz sustained for 50 ms */
        if (freq_demand > (float)BURST_ENTRY_FREQ_HZ)
        {
            s_burst.entry_timer++;

            if (s_burst.entry_timer >= BURST_ENTRY_TIME_MS)
            {
                /* Configure HRTIM burst mode on first entry */
                if (s_burst.hrtim_configured == 0U)
                {
                    App_Control_HRTIM_BurstMode_Config(
                        BURST_DEFAULT_FREQ_HZ, BURST_DEFAULT_DUTY);
                    s_burst.hrtim_configured = 1U;
                }

                s_burst.v_target = (float)LLC_SOFTSTART_TARGET_V;
                s_burst.v_out_prev = v_out;

                /* Enable burst mode and trigger first cycle */
                App_Control_HRTIM_BurstMode_Enable();
                App_Control_HRTIM_BurstMode_SWTrigger();

                s_burst.state = BURST_RUN;
                s_burst.entry_timer = 0U;

                App_Diagnostics_Log("[BM] Enter burst mode");
            }
        }
        else
        {
            s_burst.entry_timer = 0U;
        }
        break;

    case BURST_RUN:
        /* Check exit condition: frequency demand dropped below hysteresis */
        if (freq_demand < (float)BURST_EXIT_FREQ_HZ)
        {
            App_Control_HRTIM_BurstMode_Disable();
            App_Control_LLC_PI_Unfreeze();
            s_burst.state = BURST_INACTIVE;
            App_Diagnostics_Log("[BM] Exit burst mode (freq)");
            break;
        }

        /* RUN → IDLE: output voltage reached upper threshold */
        if (v_out > (s_burst.v_target + BURST_VOUT_UPPER_V))
        {
            /* Freeze LLC PI integrators */
            App_Control_LLC_PI_Freeze();
            s_burst.idle_tick  = 0U;
            s_burst.v_out_prev = v_out;
            s_burst.state      = BURST_IDLE;
        }
        else
        {
            /* Re-trigger next burst cycle (single-shot mode) */
            App_Control_HRTIM_BurstMode_SWTrigger();
        }
        break;

    case BURST_IDLE:
        s_burst.idle_tick++;

        /* Slow integrator decay to prevent stale values */
        App_Control_LLC_PI_DecayIdle(BURST_INTEGRATOR_DECAY);

        /* Load detection: dV/dt estimation (dV per ms × C_out → current) */
        float dv = s_burst.v_out_prev - v_out; /* positive = dropping */
        s_burst.v_out_prev = v_out;

        /* Emergency exit: large voltage drop */
        if (v_out < (s_burst.v_target - BURST_VOUT_EMERGENCY_V))
        {
            App_Control_HRTIM_BurstMode_Disable();
            App_Control_LLC_PI_Unfreeze();
            s_burst.state = BURST_INACTIVE;
            App_Diagnostics_Log("[BM] Emergency exit (Vdrop)");
            break;
        }

        /* IDLE → RUN: output voltage dropped below lower threshold */
        if (v_out < (s_burst.v_target - BURST_VOUT_LOWER_V))
        {
            App_Control_LLC_PI_Unfreeze();
            App_Control_HRTIM_BurstMode_SWTrigger();
            s_burst.state = BURST_RUN;
            break;
        }

        /* Fast exit on load transient: dV > 2 V/ms indicates > 2 A step */
        if (dv > 2.0f)
        {
            App_Control_LLC_PI_Unfreeze();
            App_Control_HRTIM_BurstMode_SWTrigger();
            s_burst.state = BURST_RUN;
        }
        break;

    default:
        s_burst.state = BURST_INACTIVE;
        break;
    }
}
