/**
 * @file    app_control.c
 * @brief   PFC dq-frame control, LLC PFM, PI with anti-windup
 */

#include "app_control.h"
#include "app_npbalance.h"
#include "app_transforms.h"
#include "app_protection.h"
#include "app_pll.h"
#include "app_adc.h"
#include "app_can.h"
#include "app_diagnostics.h"
#include "main.h"
#include <math.h>
#include <stdio.h>

/* ------------------------------------------------------------------ */
/*  PFC HRTIM Constants                                                */
/* ------------------------------------------------------------------ */

/** @brief  HRTIM DLL clock: 170 MHz x 32 = 5.44 GHz */
#define PFC_HRTIM_PERIOD       83692U   /* 5,440,000,000 / 65,000 */
#define PFC_DUTY_30_CMP        25108U   /* 83,692 x 0.30 */
#define PFC_ADC_MIDPOINT_CMP   41846U   /* 83,692 / 2 */
#define PFC_MAX_DUTY           0.95f

/** @brief  Master timer phase offsets for 120-degree interleaving */
#define PFC_PHASE_OFFSET_0     1U       /* Timer A reset: ~0 deg */
#define PFC_PHASE_OFFSET_120   27897U   /* Timer B reset: PER/3 */
#define PFC_PHASE_OFFSET_240   55795U   /* Timer C reset: 2*PER/3 */

/**
 * @brief  Dead-time: prescaler MUL8 -> fDTG = 1.36 GHz
 *         300 ns x 1.36e9 = 408 ticks
 */
#define PFC_DEADTIME_TICKS     408U

/* ------------------------------------------------------------------ */
/*  LLC HRTIM Constants                                                */
/* ------------------------------------------------------------------ */

/** @brief  HRTIM period for LLC at 300 kHz: 5,440,000,000 / 300,000 = 18133 */
#define LLC_HRTIM_PERIOD_300K  18133U
/** @brief  HRTIM period for LLC at 100 kHz: 5,440,000,000 / 100,000 = 54400 */
#define LLC_HRTIM_PERIOD_100K  54400U

/** @brief  Sweep parameters: 300→100 kHz in 2 kHz steps, 50 ms settle */
#define LLC_SWEEP_STEP_HZ      2000U
#define LLC_SWEEP_SETTLE_MS    50U

/* ------------------------------------------------------------------ */
/*  ZVS Detection Constants                                            */
/* ------------------------------------------------------------------ */

/** @brief  Max sweep data points: (300k - 100k) / 2k + 1 = 101 */
#define SWEEP_MAX_POINTS       101U

/** @brief  Second derivative threshold for ZVS boundary inflection */
#define ZVS_D2GAIN_THRESHOLD   0.0005f

/* ------------------------------------------------------------------ */
/*  LLC Sweep State                                                    */
/* ------------------------------------------------------------------ */

/** @brief  Per-point sample recorded during sweep */
typedef struct
{
    uint32_t freq_hz;
    float    v_out;
    float    i_out;
    float    gain;
} SweepSample_t;

typedef struct
{
    uint32_t       freq_hz;
    uint32_t       step_hz;
    uint32_t       settle_ms;
    uint32_t       settle_count;
    uint8_t        active;
    uint8_t        done;
    LLC_SweepMode_t mode;
    SweepSample_t  samples[SWEEP_MAX_POINTS];
    uint16_t       sample_count;
    ZVS_Result_t   zvs_result;
} LLC_Sweep_t;

static LLC_Sweep_t s_llc_sweep;

/* ------------------------------------------------------------------ */
/*  PI Controller Instances                                             */
/* ------------------------------------------------------------------ */

/** @brief  Closed-loop PFC enable flag — set after soft-start completes */
static volatile uint8_t s_pfc_closed_loop;

/** @brief  I_d* setpoint in amps — set by voltage loop or soft-start ramp */
static volatile float s_pfc_id_ref;

/** @brief  Shared NP offset from neutral-point balancing module */
extern volatile float g_np_zs_offset;

/** @brief  PI controller array indexed by PI_Index_t */
static PI_Controller_t s_pi[PI_ID_COUNT];

/** @brief  Human-readable names for CLI/diagnostics */
static const char *const s_pi_names[PI_ID_COUNT] = {
    "id", "iq", "vbus", "vllc", "illc"
};

/**
 * @brief  Compute anti-windup tracking time constant
 *
 * Tt = sqrt(Kp / Ki) — geometric mean of proportional and integral
 * time constants.  For Kp/Ki <= 0, falls back to a safe default.
 */
static float pi_compute_tt(float Kp, float Ki)
{
    if ((Kp <= 0.0f) || (Ki <= 0.0f))
    {
        return 0.01f; /* Safe fallback */
    }
    return sqrtf(Kp / Ki);
}

void PI_SetGains(PI_Index_t idx, float Kp, float Ki,
                 float out_min, float out_max)
{
    if (idx >= PI_ID_COUNT)
    {
        return;
    }

    PI_Controller_t *pi = &s_pi[idx];
    pi->Kp      = Kp;
    pi->Ki      = Ki;
    pi->Tt      = pi_compute_tt(Kp, Ki);
    pi->out_min = out_min;
    pi->out_max = out_max;
    pi->integrator = 0.0f;
    pi->prev_error = 0.0f;
}

const PI_Controller_t *PI_GetController(PI_Index_t idx)
{
    if (idx >= PI_ID_COUNT)
    {
        return NULL;
    }
    return &s_pi[idx];
}

const char *PI_GetName(PI_Index_t idx)
{
    if (idx >= PI_ID_COUNT)
    {
        return "?";
    }
    return s_pi_names[idx];
}

void App_Control_Init(void)
{
    /* PFC current loop: BW ~1.5 kHz, L=200 uH, Tt = sqrt(0.94/1776) ≈ 0.023 */
    PI_SetGains(PI_ID_ID, 0.94f, 1776.0f, -1.0f, 1.0f);
    PI_SetGains(PI_ID_IQ, 0.94f, 1776.0f, -1.0f, 1.0f);

    /* Bus voltage loop: BW ~30 Hz, Tt = sqrt(0.1/10) = 0.1 */
    PI_SetGains(PI_ID_VBUS, 0.1f, 10.0f, 0.0f, 60.0f);

    /* LLC voltage loop: Tt = sqrt(0.005/5) ≈ 0.032 */
    PI_SetGains(PI_ID_VLLC, 0.005f, 5.0f, 100000.0f, 300000.0f);

    /* LLC current loop: Tt = sqrt(0.01/10) ≈ 0.032 */
    PI_SetGains(PI_ID_ILLC, 0.01f, 10.0f, 100000.0f, 300000.0f);

    /* Configure PFC HRTIM timers (does not start outputs) */
    App_Control_HRTIM_PFC_Init();

    /* Configure LLC HRTIM timers (does not start outputs) */
    App_Control_HRTIM_LLC_Init();
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
    {
        output_sat = pi->out_max;
    }
    if (output_sat < pi->out_min)
    {
        output_sat = pi->out_min;
    }

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

/* ------------------------------------------------------------------ */
/*  Bus Voltage Outer Loop (EP-04-002)                                 */
/* ------------------------------------------------------------------ */

/** @brief  Voltage loop enable flag — gated by state machine */
static volatile uint8_t s_vbus_loop_enabled = 0U;

/** @brief  Main-loop time step for 1 kHz execution */
#define VBUS_LOOP_DT  (1.0f / (float)MAIN_LOOP_FREQ_HZ)

/**
 * @brief  Run bus voltage PI controller at 1 kHz
 *
 * Reads V_bus from ADC, computes error vs. setpoint (from CAN command
 * or default PFC_TARGET_VBUS_V), and outputs I_d* reference clamped
 * to [0, PFC_SOFTSTART_ID_MAX].  The I_d* is consumed by the PFC
 * inner current loop ISR via App_Control_VBus_GetIdRef().
 *
 * Anti-windup is handled by the PI_Update back-calculation scheme.
 */
void App_Control_VBus_Update(void)
{
    if (s_vbus_loop_enabled == 0U)
    {
        return;
    }

    const ADC_Readings_t *adc = App_ADC_GetReadings();

    /* Determine setpoint: use CAN command if valid, else default */
    const CAN_Command_t *cmd = App_CAN_GetCommand();
    float v_setpoint;

    if (cmd->valid != 0U)
    {
        /* CAN v_ref is in 0.1 V/LSB */
        v_setpoint = (float)cmd->v_ref * 0.1f;

        /* Clamp to safe operating range */
        if (v_setpoint < 700.0f)
        {
            v_setpoint = 700.0f;
        }
        if (v_setpoint > (BUS_OVP_THRESHOLD_V - 50.0f))
        {
            v_setpoint = BUS_OVP_THRESHOLD_V - 50.0f;
        }
    }
    else
    {
        v_setpoint = PFC_TARGET_VBUS_V;
    }

    /* Run PI: output is I_d* in amps */
    float id_ref = PI_Update(&s_pi[PI_ID_VBUS], v_setpoint, adc->v_bus,
                             VBUS_LOOP_DT);

    /* Atomic 32-bit store — Cortex-M4 aligned float write is atomic */
    s_pfc_id_ref = id_ref;
}

/**
 * @brief  Enable the bus voltage outer loop
 *
 * Called when entering RUN state after soft-start completes.
 * The integrator should be pre-loaded before enabling for
 * bumpless transfer from open-loop soft-start.
 */
void App_Control_VBus_Enable(void)
{
    s_vbus_loop_enabled = 1U;
}

/**
 * @brief  Disable the bus voltage outer loop and zero the I_d* reference
 *
 * Called on FAULT, SHUTDOWN, or any state that disables PFC outputs.
 * Resets the PI integrator to prevent windup.
 */
void App_Control_VBus_Disable(void)
{
    s_vbus_loop_enabled = 0U;
    s_pfc_id_ref = 0.0f;
    PI_Reset(&s_pi[PI_ID_VBUS]);
}

/**
 * @brief  Get current I_d* reference from bus voltage loop
 * @return I_d* in amps (0 when loop is disabled)
 *
 * Called from PFC ISR (65 kHz) — single aligned float read is
 * atomic on Cortex-M4, no lock needed.
 */
float App_Control_VBus_GetIdRef(void)
{
    return s_pfc_id_ref;
}

/**
 * @brief  Pre-load bus voltage PI integrator for bumpless transfer
 * @param  value  Integrator pre-load value (typically the steady-state
 *                I_d* expected at the current bus voltage)
 *
 * Called just before enabling the loop when transitioning from
 * open-loop soft-start to closed-loop RUN.  Prevents a step change
 * in I_d* that would cause a voltage transient.
 */
void App_Control_VBus_PreloadIntegrator(float value)
{
    s_pi[PI_ID_VBUS].integrator = value;
    s_pi[PI_ID_VBUS].prev_error = 0.0f;
    s_pfc_id_ref = value;
}

/* ------------------------------------------------------------------ */
/*  PFC Closed-Loop Current Control API                                */
/* ------------------------------------------------------------------ */

/**
 * @brief  Set I_d* reference for PFC current loop
 * @param  id_ref_a  Desired d-axis current in amps
 */
void App_Control_PFC_SetIdRef(float id_ref_a)
{
    s_pfc_id_ref = id_ref_a;
}

/**
 * @brief  Enable closed-loop PFC current control
 *
 * Resets PI_ID and PI_IQ integrators for bumpless transfer,
 * then sets the closed-loop enable flag read by the ISR.
 */
void App_Control_PFC_EnableClosedLoop(void)
{
    PI_Reset(&s_pi[PI_ID_ID]);
    PI_Reset(&s_pi[PI_ID_IQ]);
    s_pfc_closed_loop = 1U;
}

/**
 * @brief  Disable closed-loop PFC current control
 */
void App_Control_PFC_DisableClosedLoop(void)
{
    s_pfc_closed_loop = 0U;
}

/* ------------------------------------------------------------------ */
/*  PFC HRTIM Configuration                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief  Helper — configure one PFC timer unit (A, B, or C)
 * @param  timer_idx  HRTIM_TIMERINDEX_TIMER_A / B / C
 * @param  reset_src  Master CMP event that resets this timer (phase offset)
 */
static void pfc_timer_config(uint32_t timer_idx, uint32_t reset_src)
{
    HRTIM_TimeBaseCfgTypeDef TimeBaseCfg = {0};
    HRTIM_TimerCfgTypeDef    TimerCfg    = {0};
    HRTIM_CompareCfgTypeDef  CompareCfg  = {0};
    HRTIM_DeadTimeCfgTypeDef DeadTimeCfg = {0};
    HRTIM_OutputCfgTypeDef   OutputCfg   = {0};

    /* Time base — same period as master */
    TimeBaseCfg.Period            = PFC_HRTIM_PERIOD;
    TimeBaseCfg.RepetitionCounter = 0U;
    TimeBaseCfg.PrescalerRatio    = HRTIM_PRESCALERRATIO_MUL32;
    TimeBaseCfg.Mode              = HRTIM_MODE_CONTINUOUS;
    HAL_HRTIM_TimeBaseConfig(&hhrtim1, timer_idx, &TimeBaseCfg);

    /* Timer config — preload, dead-time insertion, fault action */
    TimerCfg.InterleavedMode       = HRTIM_INTERLEAVED_MODE_DISABLED;
    TimerCfg.StartOnSync          = HRTIM_SYNCSTART_DISABLED;
    TimerCfg.ResetOnSync          = HRTIM_SYNCRESET_DISABLED;
    TimerCfg.DACSynchro           = HRTIM_DACSYNC_NONE;
    TimerCfg.PreloadEnable        = HRTIM_PRELOAD_ENABLED;
    TimerCfg.UpdateGating         = HRTIM_UPDATEGATING_INDEPENDENT;
    TimerCfg.BurstMode            = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
    TimerCfg.RepetitionUpdate     = HRTIM_UPDATEONREPETITION_ENABLED;
    TimerCfg.PushPull             = HRTIM_TIMPUSHPULLMODE_DISABLED;
    TimerCfg.FaultEnable          = HRTIM_TIMFAULTENABLE_FAULT1
                                  | HRTIM_TIMFAULTENABLE_FAULT3
                                  | HRTIM_TIMFAULTENABLE_FAULT5;
    TimerCfg.FaultLock            = HRTIM_TIMFAULTLOCK_READWRITE;
    TimerCfg.DeadTimeInsertion    = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
    TimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
    TimerCfg.UpdateTrigger        = HRTIM_TIMUPDATETRIGGER_NONE;
    TimerCfg.ResetTrigger         = reset_src;
    TimerCfg.ResetUpdate          = HRTIM_TIMUPDATEONRESET_ENABLED;
    HAL_HRTIM_WaveformTimerConfig(&hhrtim1, timer_idx, &TimerCfg);

    /* CMP1 — duty cycle compare (30% default) */
    CompareCfg.CompareValue       = PFC_DUTY_30_CMP;
    HAL_HRTIM_WaveformCompareConfig(&hhrtim1, timer_idx,
                                    HRTIM_COMPAREUNIT_1, &CompareCfg);

    /* CMP3 — ADC trigger at midpoint */
    CompareCfg.CompareValue       = PFC_ADC_MIDPOINT_CMP;
    HAL_HRTIM_WaveformCompareConfig(&hhrtim1, timer_idx,
                                    HRTIM_COMPAREUNIT_3, &CompareCfg);

    /* Dead-time: prescaler MUL8, rising = falling = 408 ticks (300 ns) */
    DeadTimeCfg.Prescaler         = HRTIM_TIMDEADTIME_PRESCALERRATIO_MUL8;
    DeadTimeCfg.RisingValue       = PFC_DEADTIME_TICKS;
    DeadTimeCfg.RisingSign        = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
    DeadTimeCfg.RisingLock        = HRTIM_TIMDEADTIME_RISINGLOCK_READONLY;
    DeadTimeCfg.RisingSignLock    = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_READONLY;
    DeadTimeCfg.FallingValue      = PFC_DEADTIME_TICKS;
    DeadTimeCfg.FallingSign       = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
    DeadTimeCfg.FallingLock       = HRTIM_TIMDEADTIME_FALLINGLOCK_READONLY;
    DeadTimeCfg.FallingSignLock   = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_READONLY;
    HAL_HRTIM_DeadTimeConfig(&hhrtim1, timer_idx, &DeadTimeCfg);

    /* Output 1: set on period (counter reset), reset on CMP1 match */
    OutputCfg.Polarity            = HRTIM_OUTPUTPOLARITY_HIGH;
    OutputCfg.SetSource           = HRTIM_OUTPUTSET_TIMPER;
    OutputCfg.ResetSource         = HRTIM_OUTPUTRESET_TIMCMP1;
    OutputCfg.IdleMode            = HRTIM_OUTPUTIDLEMODE_NONE;
    OutputCfg.IdleLevel           = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
    OutputCfg.FaultLevel          = HRTIM_OUTPUTFAULTLEVEL_INACTIVE;
    OutputCfg.ChopperModeEnable   = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
    OutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;

    /* Map timer index to output identifiers */
    uint32_t output1, output2;
    if (timer_idx == HRTIM_TIMERINDEX_TIMER_A)
    {
        output1 = HRTIM_OUTPUT_TA1;
        output2 = HRTIM_OUTPUT_TA2;
    }
    else if (timer_idx == HRTIM_TIMERINDEX_TIMER_B)
    {
        output1 = HRTIM_OUTPUT_TB1;
        output2 = HRTIM_OUTPUT_TB2;
    }
    else
    {
        output1 = HRTIM_OUTPUT_TC1;
        output2 = HRTIM_OUTPUT_TC2;
    }

    HAL_HRTIM_WaveformOutputConfig(&hhrtim1, timer_idx, output1, &OutputCfg);

    /* Output 2: complementary via dead-time (sources ignored when DT enabled) */
    OutputCfg.SetSource           = HRTIM_OUTPUTSET_NONE;
    OutputCfg.ResetSource         = HRTIM_OUTPUTRESET_NONE;
    HAL_HRTIM_WaveformOutputConfig(&hhrtim1, timer_idx, output2, &OutputCfg);
}

/**
 * @brief  Configure HRTIM Master + Timer A/B/C for 3-phase PFC at 65 kHz
 *
 * Master timer provides 120-degree phase offsets via CMP1/CMP2/CMP3.
 * Each timer unit runs at the same period with dead-time insertion.
 * ADC trigger 1 is sourced from Timer A CMP3 (midpoint sampling).
 * Does not start the counters — call App_Control_PFC_Start() for that.
 */
void App_Control_HRTIM_PFC_Init(void)
{
    HRTIM_TimeBaseCfgTypeDef MasterTimeBase = {0};
    HRTIM_TimerCfgTypeDef    MasterCfg      = {0};
    HRTIM_CompareCfgTypeDef  MasterCmp      = {0};
    HRTIM_ADCTriggerCfgTypeDef ADCTrigCfg   = {0};

    /* ---- Master timer ---- */
    MasterTimeBase.Period            = PFC_HRTIM_PERIOD;
    MasterTimeBase.RepetitionCounter = 0U;
    MasterTimeBase.PrescalerRatio    = HRTIM_PRESCALERRATIO_MUL32;
    MasterTimeBase.Mode              = HRTIM_MODE_CONTINUOUS;
    HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &MasterTimeBase);

    MasterCfg.InterleavedMode   = HRTIM_INTERLEAVED_MODE_DISABLED;
    MasterCfg.StartOnSync      = HRTIM_SYNCSTART_DISABLED;
    MasterCfg.ResetOnSync      = HRTIM_SYNCRESET_DISABLED;
    MasterCfg.DACSynchro       = HRTIM_DACSYNC_NONE;
    MasterCfg.PreloadEnable    = HRTIM_PRELOAD_ENABLED;
    MasterCfg.UpdateGating     = HRTIM_UPDATEGATING_INDEPENDENT;
    MasterCfg.BurstMode        = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
    MasterCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
    HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &MasterCfg);

    /* Master CMP1 — Timer A reset (0 deg) */
    MasterCmp.CompareValue = PFC_PHASE_OFFSET_0;
    HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER,
                                    HRTIM_COMPAREUNIT_1, &MasterCmp);

    /* Master CMP2 — Timer B reset (120 deg) */
    MasterCmp.CompareValue = PFC_PHASE_OFFSET_120;
    HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER,
                                    HRTIM_COMPAREUNIT_2, &MasterCmp);

    /* Master CMP3 — Timer C reset (240 deg) */
    MasterCmp.CompareValue = PFC_PHASE_OFFSET_240;
    HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER,
                                    HRTIM_COMPAREUNIT_3, &MasterCmp);

    /* ---- Timer A (phase A) — reset on Master CMP1 ---- */
    pfc_timer_config(HRTIM_TIMERINDEX_TIMER_A, HRTIM_TIMRESETTRIGGER_MASTER_CMP1);

    /* ---- Timer B (phase B) — reset on Master CMP2 ---- */
    pfc_timer_config(HRTIM_TIMERINDEX_TIMER_B, HRTIM_TIMRESETTRIGGER_MASTER_CMP2);

    /* ---- Timer C (phase C) — reset on Master CMP3 ---- */
    pfc_timer_config(HRTIM_TIMERINDEX_TIMER_C, HRTIM_TIMRESETTRIGGER_MASTER_CMP3);

    /* ---- ADC trigger 1: Timer A CMP3 event (midpoint sampling) ---- */
    ADCTrigCfg.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_A;
    ADCTrigCfg.Trigger      = HRTIM_ADCTRIGGEREVENT13_TIMERA_CMP3;
    HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1, &ADCTrigCfg);

    /* ---- NVIC: Master repetition interrupt for PFC ISR ---- */
    HAL_NVIC_SetPriority(HRTIM1_Master_IRQn, NVIC_PRIO_HRTIM_PFC, 0);
    HAL_NVIC_EnableIRQ(HRTIM1_Master_IRQn);
}

/**
 * @brief  Start PFC PWM outputs and master timer
 *
 * Enables Master + Timer A/B/C counters with master repetition interrupt,
 * then enables all 6 HRTIM outputs.
 */
void App_Control_PFC_Start(void)
{
    /* Start counters: Master + Timer A + B + C with master rep IT */
    HAL_HRTIM_WaveformCountStart_IT(&hhrtim1,
                                     HRTIM_TIMERID_MASTER
                                   | HRTIM_TIMERID_TIMER_A
                                   | HRTIM_TIMERID_TIMER_B
                                   | HRTIM_TIMERID_TIMER_C);

    /* Enable all 6 PFC outputs */
    HAL_HRTIM_WaveformOutputStart(&hhrtim1,
                                   HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2
                                 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2
                                 | HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2);
}

/**
 * @brief  Stop PFC PWM — disable outputs first (safe), then counters
 */
void App_Control_PFC_Stop(void)
{
    /* Disable closed-loop and reset PI integrators */
    s_pfc_closed_loop = 0U;
    PI_Reset(&s_pi[PI_ID_ID]);
    PI_Reset(&s_pi[PI_ID_IQ]);

    /* Disable outputs first for safe shutdown */
    HAL_HRTIM_WaveformOutputStop(&hhrtim1,
                                  HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2
                                | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2
                                | HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2);

    /* Stop counters */
    HAL_HRTIM_WaveformCountStop_IT(&hhrtim1,
                                    HRTIM_TIMERID_MASTER
                                  | HRTIM_TIMERID_TIMER_A
                                  | HRTIM_TIMERID_TIMER_B
                                  | HRTIM_TIMERID_TIMER_C);
}

/**
 * @brief  Set PFC duty cycle for all 3 timer units
 * @param  duty  Duty cycle 0.0 to 0.95
 */
void App_Control_PFC_SetDuty(float duty)
{
    /* Clamp duty */
    if (duty < 0.0f)
    {
        duty = 0.0f;
    }
    if (duty > PFC_MAX_DUTY)
    {
        duty = PFC_MAX_DUTY;
    }

    uint32_t cmp_val = (uint32_t)((float)PFC_HRTIM_PERIOD * duty);
    if (cmp_val < 1U)
    {
        cmp_val = 1U;
    }

    HRTIM_CompareCfgTypeDef cmp = {0};
    cmp.CompareValue = cmp_val;

    HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A,
                                    HRTIM_COMPAREUNIT_1, &cmp);
    HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,
                                    HRTIM_COMPAREUNIT_1, &cmp);
    HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C,
                                    HRTIM_COMPAREUNIT_1, &cmp);
}

/**
 * @brief  Set per-phase PFC duty cycles (independent per leg)
 * @param  da  Phase A duty [0, 0.95]
 * @param  db  Phase B duty [0, 0.95]
 * @param  dc  Phase C duty [0, 0.95]
 */
void App_Control_PFC_SetDutyABC(float da, float db, float dc)
{
    /* Clamp each phase independently */
    if (da < 0.0f) { da = 0.0f; }
    if (da > PFC_MAX_DUTY) { da = PFC_MAX_DUTY; }
    if (db < 0.0f) { db = 0.0f; }
    if (db > PFC_MAX_DUTY) { db = PFC_MAX_DUTY; }
    if (dc < 0.0f) { dc = 0.0f; }
    if (dc > PFC_MAX_DUTY) { dc = PFC_MAX_DUTY; }

    HRTIM_CompareCfgTypeDef cmp = {0};
    uint32_t cmp_a = (uint32_t)((float)PFC_HRTIM_PERIOD * da);
    uint32_t cmp_b = (uint32_t)((float)PFC_HRTIM_PERIOD * db);
    uint32_t cmp_c = (uint32_t)((float)PFC_HRTIM_PERIOD * dc);

    if (cmp_a < 1U) { cmp_a = 1U; }
    if (cmp_b < 1U) { cmp_b = 1U; }
    if (cmp_c < 1U) { cmp_c = 1U; }

    cmp.CompareValue = cmp_a;
    HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A,
                                    HRTIM_COMPAREUNIT_1, &cmp);
    cmp.CompareValue = cmp_b;
    HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,
                                    HRTIM_COMPAREUNIT_1, &cmp);
    cmp.CompareValue = cmp_c;
    HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C,
                                    HRTIM_COMPAREUNIT_1, &cmp);
}

/* ------------------------------------------------------------------ */
/*  ISR Callbacks                                                      */
/* ------------------------------------------------------------------ */

void App_Control_PFC_ISR(void)
{
    const ADC_Readings_t *adc = App_ADC_GetReadings();
    static const float dt = 1.0f / (float)PFC_ISR_FREQ_HZ;

    /* Compute sin/cos of PLL angle via CORDIC */
    float theta = App_PLL_GetTheta();
    SinCos_t sc = Transforms_CORDIC_SinCos(theta);

    /* Clarke transform on phase currents */
    AlphaBeta_t i_ab = Transforms_Clarke(adc->i_phase_a, adc->i_phase_b);

    /* Park transform to dq frame */
    DQ_t i_dq = Transforms_Park(i_ab, sc);

    /* Update PLL with pre-computed sin/cos (always runs for lock detect) */
    App_PLL_UpdateEx(adc->v_grid_a, adc->v_grid_b, dt, sc);

    /* Closed-loop current control — skip if not yet enabled */
    if (s_pfc_closed_loop == 0U)
    {
        return;
    }

    /* Read I_d* reference (from VBus outer loop or CLI override) */
    float id_ref = s_pfc_id_ref;

    /* d/q PI controllers */
    float v_d_pi = PI_Update(&s_pi[PI_ID_ID], id_ref, i_dq.d, dt);
    float v_q_pi = PI_Update(&s_pi[PI_ID_IQ], 0.0f, i_dq.q, dt);

    /* Decoupling feedforward: V_d* = V_d_PI - ω·L·I_q
     *                         V_q* = V_q_PI + ω·L·I_d   */
    float omega = App_PLL_GetOmega();
    float wL = omega * PFC_L_HENRY;

    DQ_t v_dq;
    v_dq.d = v_d_pi - wL * i_dq.q;
    v_dq.q = v_q_pi + wL * i_dq.d;

    /* Inverse Park → alpha-beta voltage commands */
    AlphaBeta_t v_ab = Transforms_InversePark(v_dq, sc);

    /* SVM → per-phase duty cycles */
    DutyABC_t duties = Transforms_SVM(v_ab.alpha, v_ab.beta,
                                       adc->v_bus, PFC_MAX_DUTY);

    /* Neutral-point zero-sequence offset (EP-03-013) */
    float np_offset = g_np_zs_offset;
    duties.a += np_offset;
    duties.b += np_offset;
    duties.c += np_offset;

    /* Write to HRTIM compare registers */
    App_Control_PFC_SetDutyABC(duties.a, duties.b, duties.c);
}

void App_Control_LLC_ISR(void)
{
    /* TODO: CC/CV mode select -> PI_vllc or PI_illc -> frequency -> HRTIM PERxR */
}

/* ------------------------------------------------------------------ */
/*  LLC HRTIM Configuration                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief  Helper — configure one LLC timer unit (D, E, or F)
 * @param  timer_idx   HRTIM_TIMERINDEX_TIMER_D / E / F
 * @param  reset_src   Cross-reset trigger (0 for Timer D — free-runs)
 * @param  fault_mask  Fault enable mask (FLT3 | FLT4 for LLC)
 */
static void llc_timer_config(uint32_t timer_idx, uint32_t reset_src,
                             uint32_t fault_mask)
{
    HRTIM_TimeBaseCfgTypeDef TimeBaseCfg = {0};
    HRTIM_TimerCfgTypeDef    TimerCfg    = {0};
    HRTIM_DeadTimeCfgTypeDef DeadTimeCfg = {0};
    HRTIM_OutputCfgTypeDef   OutputCfg   = {0};

    /* Time base — 300 kHz startup frequency, MUL32 prescaler */
    TimeBaseCfg.Period            = LLC_HRTIM_PERIOD_300K;
    TimeBaseCfg.RepetitionCounter = 0U;
    TimeBaseCfg.PrescalerRatio    = HRTIM_PRESCALERRATIO_MUL32;
    TimeBaseCfg.Mode              = HRTIM_MODE_CONTINUOUS;
    HAL_HRTIM_TimeBaseConfig(&hhrtim1, timer_idx, &TimeBaseCfg);

    /* Timer config — HALF mode for 50% duty, dead-time, fault action */
    TimerCfg.HalfModeEnable        = HRTIM_HALFMODE_ENABLED;
    TimerCfg.InterleavedMode       = HRTIM_INTERLEAVED_MODE_DISABLED;
    TimerCfg.StartOnSync           = HRTIM_SYNCSTART_DISABLED;
    TimerCfg.ResetOnSync           = HRTIM_SYNCRESET_DISABLED;
    TimerCfg.DACSynchro            = HRTIM_DACSYNC_NONE;
    TimerCfg.PreloadEnable         = HRTIM_PRELOAD_ENABLED;
    TimerCfg.UpdateGating          = HRTIM_UPDATEGATING_INDEPENDENT;
    TimerCfg.BurstMode             = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
    TimerCfg.RepetitionUpdate      = HRTIM_UPDATEONREPETITION_ENABLED;
    TimerCfg.PushPull              = HRTIM_TIMPUSHPULLMODE_DISABLED;
    TimerCfg.FaultEnable           = fault_mask;
    TimerCfg.FaultLock             = HRTIM_TIMFAULTLOCK_READWRITE;
    TimerCfg.DeadTimeInsertion     = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
    TimerCfg.UpdateTrigger         = HRTIM_TIMUPDATETRIGGER_NONE;
    TimerCfg.ResetTrigger          = reset_src;
    TimerCfg.ResetUpdate           = HRTIM_TIMUPDATEONRESET_ENABLED;

    /* Delayed protection — use correct define per timer group */
    if (timer_idx == HRTIM_TIMERINDEX_TIMER_F)
    {
        TimerCfg.DelayedProtectionMode =
            HRTIM_TIMER_F_DELAYEDPROTECTION_DISABLED;
    }
    else
    {
        TimerCfg.DelayedProtectionMode =
            HRTIM_TIMER_D_E_DELAYEDPROTECTION_DISABLED;
    }

    HAL_HRTIM_WaveformTimerConfig(&hhrtim1, timer_idx, &TimerCfg);

    /* Dead-time: prescaler MUL8, 272 ticks = 200 ns */
    DeadTimeCfg.Prescaler         = HRTIM_TIMDEADTIME_PRESCALERRATIO_MUL8;
    DeadTimeCfg.RisingValue       = LLC_DEADTIME_TICKS;
    DeadTimeCfg.RisingSign        = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
    DeadTimeCfg.RisingLock        = HRTIM_TIMDEADTIME_RISINGLOCK_READONLY;
    DeadTimeCfg.RisingSignLock    = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_READONLY;
    DeadTimeCfg.FallingValue      = LLC_DEADTIME_TICKS;
    DeadTimeCfg.FallingSign       = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
    DeadTimeCfg.FallingLock       = HRTIM_TIMDEADTIME_FALLINGLOCK_READONLY;
    DeadTimeCfg.FallingSignLock   = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_READONLY;
    HAL_HRTIM_DeadTimeConfig(&hhrtim1, timer_idx, &DeadTimeCfg);

    /* Output 1: set on period reset, reset on CMP1 (auto PER/2 via HALF) */
    OutputCfg.Polarity            = HRTIM_OUTPUTPOLARITY_HIGH;
    OutputCfg.SetSource           = HRTIM_OUTPUTSET_TIMPER;
    OutputCfg.ResetSource         = HRTIM_OUTPUTRESET_TIMCMP1;
    OutputCfg.IdleMode            = HRTIM_OUTPUTIDLEMODE_NONE;
    OutputCfg.IdleLevel           = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
    OutputCfg.FaultLevel          = HRTIM_OUTPUTFAULTLEVEL_INACTIVE;
    OutputCfg.ChopperModeEnable   = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
    OutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;

    uint32_t output1;
    uint32_t output2;
    if (timer_idx == HRTIM_TIMERINDEX_TIMER_D)
    {
        output1 = HRTIM_OUTPUT_TD1;
        output2 = HRTIM_OUTPUT_TD2;
    }
    else if (timer_idx == HRTIM_TIMERINDEX_TIMER_E)
    {
        output1 = HRTIM_OUTPUT_TE1;
        output2 = HRTIM_OUTPUT_TE2;
    }
    else
    {
        output1 = HRTIM_OUTPUT_TF1;
        output2 = HRTIM_OUTPUT_TF2;
    }

    HAL_HRTIM_WaveformOutputConfig(&hhrtim1, timer_idx, output1, &OutputCfg);

    /* Output 2: complementary via dead-time (sources ignored when DT enabled) */
    OutputCfg.SetSource   = HRTIM_OUTPUTSET_NONE;
    OutputCfg.ResetSource = HRTIM_OUTPUTRESET_NONE;
    HAL_HRTIM_WaveformOutputConfig(&hhrtim1, timer_idx, output2, &OutputCfg);
}

/**
 * @brief  Configure HRTIM Timers D, E, F for 3-phase LLC at 300 kHz
 *
 * Timer D is the LLC group master. CMP2/CMP4 cross-triggers reset
 * Timer E and F for 120-degree interleaving. HALF mode forces 50%
 * duty; only the period (frequency) is the control variable.
 * ADC trigger 2 is sourced from Timer D PERIOD event.
 */
void App_Control_HRTIM_LLC_Init(void)
{
    HRTIM_CompareCfgTypeDef  CompareCfg  = {0};
    HRTIM_ADCTriggerCfgTypeDef ADCTrigCfg = {0};

    uint32_t llc_faults = HRTIM_TIMFAULTENABLE_FAULT2
                        | HRTIM_TIMFAULTENABLE_FAULT3
                        | HRTIM_TIMFAULTENABLE_FAULT4
                        | HRTIM_TIMFAULTENABLE_FAULT5;

    /* ---- Timer D (LLC phase master) — free-running ---- */
    llc_timer_config(HRTIM_TIMERINDEX_TIMER_D,
                     HRTIM_TIMRESETTRIGGER_NONE, llc_faults);

    /* Timer D CMP2 = PER/3 → cross-reset Timer E (120 deg) */
    CompareCfg.CompareValue = LLC_HRTIM_PERIOD_300K / 3U;
    HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D,
                                    HRTIM_COMPAREUNIT_2, &CompareCfg);

    /* Timer D CMP4 = 2*PER/3 → cross-reset Timer F (240 deg) */
    CompareCfg.CompareValue = (LLC_HRTIM_PERIOD_300K * 2U) / 3U;
    HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D,
                                    HRTIM_COMPAREUNIT_4, &CompareCfg);

    /* ---- Timer E — reset by Timer D CMP2 (OTHER4 from E's view) ---- */
    llc_timer_config(HRTIM_TIMERINDEX_TIMER_E,
                     HRTIM_TIMRESETTRIGGER_OTHER4_CMP2, llc_faults);

    /* ---- Timer F — reset by Timer D CMP4 (OTHER4 from F's view) ---- */
    llc_timer_config(HRTIM_TIMERINDEX_TIMER_F,
                     HRTIM_TIMRESETTRIGGER_OTHER4_CMP4, llc_faults);

    /* ---- ADC trigger 2: Timer D PERIOD event (LLC sampling) ---- */
    ADCTrigCfg.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_D;
    ADCTrigCfg.Trigger      = HRTIM_ADCTRIGGEREVENT24_TIMERD_PERIOD;
    HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_2, &ADCTrigCfg);
}

/**
 * @brief  Start a single LLC phase (Timer D/E/F) with its two outputs
 * @param  phase  0 = Timer D (TD1/TD2), 1 = Timer E (TE1/TE2), 2 = Timer F (TF1/TF2)
 *
 * Timer D must always start first since Timer E and F are cross-reset
 * from Timer D CMP2/CMP4.
 */
void App_Control_LLC_StartPhase(uint32_t phase)
{
    uint32_t timer_id;
    uint32_t outputs;

    switch (phase)
    {
    case 0U:
        timer_id = HRTIM_TIMERID_TIMER_D;
        outputs  = HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2;
        break;
    case 1U:
        timer_id = HRTIM_TIMERID_TIMER_E;
        outputs  = HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2;
        break;
    case 2U:
        timer_id = HRTIM_TIMERID_TIMER_F;
        outputs  = HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2;
        break;
    default:
        return;
    }

    HAL_HRTIM_WaveformCountStart(&hhrtim1, timer_id);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, outputs);
}

/**
 * @brief  Start LLC PWM outputs — Timer D/E/F counters + all 6 outputs
 */
void App_Control_LLC_Start(void)
{
    HAL_HRTIM_WaveformCountStart(&hhrtim1,
                                  HRTIM_TIMERID_TIMER_D
                                | HRTIM_TIMERID_TIMER_E
                                | HRTIM_TIMERID_TIMER_F);

    HAL_HRTIM_WaveformOutputStart(&hhrtim1,
                                   HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2
                                 | HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2
                                 | HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);
}

/**
 * @brief  Stop LLC PWM — disable outputs first (safe), then counters
 */
void App_Control_LLC_Stop(void)
{
    HAL_HRTIM_WaveformOutputStop(&hhrtim1,
                                  HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2
                                | HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2
                                | HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2);

    HAL_HRTIM_WaveformCountStop(&hhrtim1,
                                 HRTIM_TIMERID_TIMER_D
                               | HRTIM_TIMERID_TIMER_E
                               | HRTIM_TIMERID_TIMER_F);
}

/**
 * @brief  Set LLC switching frequency (all 3 phases)
 * @param  freq_hz  Frequency in Hz, clamped to [100 kHz, 300 kHz]
 */
void App_Control_LLC_SetFrequency(uint32_t freq_hz)
{
    if (freq_hz < LLC_FREQ_MIN_HZ)
    {
        freq_hz = LLC_FREQ_MIN_HZ;
    }
    if (freq_hz > LLC_FREQ_MAX_HZ)
    {
        freq_hz = LLC_FREQ_MAX_HZ;
    }

    uint32_t period = (uint32_t)(HRTIM_DLL_FREQ_HZ / (uint64_t)freq_hz);

    HRTIM_CompareCfgTypeDef cmp = {0};

    /* Update period on all 3 timers */
    HRTIM_TimeBaseCfgTypeDef tb = {0};
    tb.Period            = period;
    tb.RepetitionCounter = 0U;
    tb.PrescalerRatio    = HRTIM_PRESCALERRATIO_MUL32;
    tb.Mode              = HRTIM_MODE_CONTINUOUS;
    HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, &tb);
    HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, &tb);
    HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, &tb);

    /* Update Timer D CMP2 = PER/3 (Timer E phase offset) */
    cmp.CompareValue = period / 3U;
    HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D,
                                    HRTIM_COMPAREUNIT_2, &cmp);

    /* Update Timer D CMP4 = 2*PER/3 (Timer F phase offset) */
    cmp.CompareValue = (period * 2U) / 3U;
    HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D,
                                    HRTIM_COMPAREUNIT_4, &cmp);
}

/* ------------------------------------------------------------------ */
/*  LLC Open-Loop Frequency Sweep                                      */
/* ------------------------------------------------------------------ */

/**
 * @brief  Analyze recorded gain curve to detect ZVS boundary
 *
 * Computes first and second derivatives of M(f) using central differences,
 * then scans from high-freq to low-freq for |d²M/df²| > threshold.
 * The inflection point marks the ZVS boundary (parallel resonant freq).
 *
 * @param  phase_idx  Index into zvs_result.phase[] (0=D, 1=E, 2=F)
 */
static void zvs_analyze_gain_curve(uint8_t phase_idx)
{
    ZVS_PhaseResult_t *result = &s_llc_sweep.zvs_result.phase[phase_idx];
    result->zvs_boundary_hz = 0U;
    result->zvs_margin_hz   = 0;
    result->zvs_lost        = 0U;
    result->valid           = 0U;

    uint16_t n = s_llc_sweep.sample_count;
    if (n < 5U)
    {
        return; /* Not enough points for derivative analysis */
    }

    const SweepSample_t *s = s_llc_sweep.samples;

    /*
     * Scan from index 2 to n-3 (need neighbors for central differences).
     * Samples are ordered high-freq (idx 0) to low-freq (idx n-1).
     * Frequency step df is constant = step_hz (negative direction).
     */
    float df = (float)s_llc_sweep.step_hz;
    uint32_t boundary_freq = 0U;

    for (uint16_t i = 2U; i < (n - 2U); i++)
    {
        /* First derivatives at i-1 and i+1 */
        float dm_prev = (s[i].gain - s[i - 2U].gain) / (2.0f * df);
        float dm_next = (s[i + 2U].gain - s[i].gain) / (2.0f * df);

        /* Second derivative at i */
        float d2m = (dm_next - dm_prev) / (2.0f * df);

        if (fabsf(d2m) > ZVS_D2GAIN_THRESHOLD)
        {
            boundary_freq = s[i].freq_hz;
            break;
        }
    }

    if (boundary_freq == 0U)
    {
        /* No inflection detected — ZVS maintained across full range */
        result->zvs_boundary_hz = LLC_FREQ_MIN_HZ;
        result->zvs_margin_hz   = (int32_t)LLC_FREQ_MIN_HZ
                                - (int32_t)LLC_FREQ_FR2_HZ;
        result->zvs_lost        = 0U;
        result->valid           = 1U;
        return;
    }

    result->zvs_boundary_hz = boundary_freq;
    result->zvs_margin_hz   = (int32_t)boundary_freq
                            - (int32_t)LLC_FREQ_FR2_HZ;
    result->valid            = 1U;

    /* Estimate resonant frequency as the detected inflection */
    s_llc_sweep.zvs_result.estimated_fr_hz = boundary_freq;

    /*
     * Check if sweep went below the boundary — if the lowest frequency
     * in the sweep data is below the boundary, ZVS was lost.
     */
    uint32_t lowest_freq = s[n - 1U].freq_hz;
    if (lowest_freq < boundary_freq)
    {
        result->zvs_lost = 1U;
    }
}

/**
 * @brief  Log ZVS analysis results and raise warning if ZVS lost
 */
static void zvs_log_results(void)
{
    static const char *const phase_names[3] = {"D", "E", "F"};
    char buf[80];
    uint8_t any_lost = 0U;

    for (uint8_t i = 0U; i < 3U; i++)
    {
        const ZVS_PhaseResult_t *p = &s_llc_sweep.zvs_result.phase[i];
        if (p->valid == 0U)
        {
            (void)snprintf(buf, sizeof(buf),
                           "ZVS Phase %s: no data", phase_names[i]);
        }
        else
        {
            (void)snprintf(buf, sizeof(buf),
                           "ZVS Phase %s: boundary=%lu Hz margin=%+ld Hz %s",
                           phase_names[i],
                           (unsigned long)p->zvs_boundary_hz,
                           (long)p->zvs_margin_hz,
                           (p->zvs_lost != 0U) ? "LOST" : "OK");
        }
        App_Diagnostics_Log(buf);

        if (p->zvs_lost != 0U)
        {
            any_lost = 1U;
        }
    }

    if (any_lost != 0U)
    {
        Fault_Enter(FAULT_ZVS_LOSS);
    }
}

/**
 * @brief  Start single-phase LLC outputs for per-phase sweep
 * @param  mode  Sweep mode selecting which phase to drive
 */
static void llc_start_single_phase(LLC_SweepMode_t mode)
{
    uint32_t timer_id;
    uint32_t outputs;

    switch (mode)
    {
    case SWEEP_MODE_PHASE_D:
        timer_id = HRTIM_TIMERID_TIMER_D;
        outputs  = HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2;
        break;
    case SWEEP_MODE_PHASE_E:
        timer_id = HRTIM_TIMERID_TIMER_D | HRTIM_TIMERID_TIMER_E;
        outputs  = HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2;
        break;
    case SWEEP_MODE_PHASE_F:
        timer_id = HRTIM_TIMERID_TIMER_D | HRTIM_TIMERID_TIMER_F;
        outputs  = HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2;
        break;
    default:
        return;
    }

    HAL_HRTIM_WaveformCountStart(&hhrtim1, timer_id);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, outputs);
}

/**
 * @brief  Stop single-phase LLC outputs
 * @param  mode  Sweep mode selecting which phase to stop
 */
static void llc_stop_single_phase(LLC_SweepMode_t mode)
{
    uint32_t timer_id;
    uint32_t outputs;

    switch (mode)
    {
    case SWEEP_MODE_PHASE_D:
        timer_id = HRTIM_TIMERID_TIMER_D;
        outputs  = HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2;
        break;
    case SWEEP_MODE_PHASE_E:
        timer_id = HRTIM_TIMERID_TIMER_D | HRTIM_TIMERID_TIMER_E;
        outputs  = HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2;
        break;
    case SWEEP_MODE_PHASE_F:
        timer_id = HRTIM_TIMERID_TIMER_D | HRTIM_TIMERID_TIMER_F;
        outputs  = HRTIM_OUTPUT_TF1 | HRTIM_OUTPUT_TF2;
        break;
    default:
        return;
    }

    HAL_HRTIM_WaveformOutputStop(&hhrtim1, outputs);
    HAL_HRTIM_WaveformCountStop(&hhrtim1, timer_id);
}

/**
 * @brief  Start extended open-loop frequency sweep with mode selection
 * @param  mode  SWEEP_MODE_ALL or SWEEP_MODE_PHASE_D/E/F
 */
void App_Control_LLC_SweepStartEx(LLC_SweepMode_t mode)
{
    s_llc_sweep.freq_hz      = LLC_FREQ_MAX_HZ;
    s_llc_sweep.step_hz      = LLC_SWEEP_STEP_HZ;
    s_llc_sweep.settle_ms    = LLC_SWEEP_SETTLE_MS;
    s_llc_sweep.settle_count = LLC_SWEEP_SETTLE_MS;
    s_llc_sweep.active       = 1U;
    s_llc_sweep.done         = 0U;
    s_llc_sweep.mode         = mode;
    s_llc_sweep.sample_count = 0U;

    /* Zero out ZVS results */
    for (uint8_t i = 0U; i < 3U; i++)
    {
        s_llc_sweep.zvs_result.phase[i].zvs_boundary_hz = 0U;
        s_llc_sweep.zvs_result.phase[i].zvs_margin_hz   = 0;
        s_llc_sweep.zvs_result.phase[i].zvs_lost         = 0U;
        s_llc_sweep.zvs_result.phase[i].valid            = 0U;
    }
    s_llc_sweep.zvs_result.estimated_fr_hz = 0U;

    App_Control_LLC_SetFrequency(LLC_FREQ_MAX_HZ);

    if (mode == SWEEP_MODE_ALL)
    {
        App_Control_LLC_Start();
    }
    else
    {
        llc_start_single_phase(mode);
    }
}

/**
 * @brief  Start open-loop frequency sweep: 300 kHz → 100 kHz (all phases)
 *
 * Initializes sweep state and starts LLC outputs. Call App_Control_LLC_Sweep()
 * from the main loop at 1 kHz to advance the sweep.
 */
void App_Control_LLC_SweepStart(void)
{
    App_Control_LLC_SweepStartEx(SWEEP_MODE_ALL);
}

/**
 * @brief  Advance LLC frequency sweep — call from main loop at 1 kHz
 *
 * Each step: wait for settling, record sample with gain, log data,
 * check safety limits, decrement frequency. On completion, runs
 * ZVS gain curve analysis and logs results.
 */
void App_Control_LLC_Sweep(void)
{
    if (s_llc_sweep.active == 0U)
    {
        return;
    }

    /* Wait for settling */
    if (s_llc_sweep.settle_count > 0U)
    {
        s_llc_sweep.settle_count--;
        return;
    }

    /* Read ADC values */
    const ADC_Readings_t *adc = App_ADC_GetReadings();
    float v_out = adc->v_out;
    float i_out = adc->i_out;
    float v_bus = adc->v_bus;

    /* Compute gain: M = V_out / (V_bus / (2 * n)) */
    float gain = 0.0f;
    float v_bus_reflected = v_bus / (2.0f * LLC_TURNS_RATIO);
    if (v_bus_reflected > 1.0f)
    {
        gain = v_out / v_bus_reflected;
    }

    /* Record sample */
    if (s_llc_sweep.sample_count < SWEEP_MAX_POINTS)
    {
        SweepSample_t *sp = &s_llc_sweep.samples[s_llc_sweep.sample_count];
        sp->freq_hz = s_llc_sweep.freq_hz;
        sp->v_out   = v_out;
        sp->i_out   = i_out;
        sp->gain    = gain;
        s_llc_sweep.sample_count++;
    }

    /* Log sweep data point with gain */
    char log_buf[80];
    (void)snprintf(log_buf, sizeof(log_buf),
                   "SWEEP %lu Hz Vout=%.1f Iout=%.1f M=%.3f",
                   (unsigned long)s_llc_sweep.freq_hz, (double)v_out,
                   (double)i_out, (double)gain);
    App_Diagnostics_Log(log_buf);

    /* Safety check — abort on OVP or OCP */
    if ((v_out > OUT_OVP_THRESHOLD_V) || (i_out > PFC_OCP_THRESHOLD_A))
    {
        if (s_llc_sweep.mode == SWEEP_MODE_ALL)
        {
            App_Control_LLC_Stop();
        }
        else
        {
            llc_stop_single_phase(s_llc_sweep.mode);
        }
        s_llc_sweep.active = 0U;
        s_llc_sweep.done   = 1U;
        App_Diagnostics_Log("SWEEP ABORT: safety limit");
        return;
    }

    /* Decrement frequency */
    if (s_llc_sweep.freq_hz <= (LLC_FREQ_MIN_HZ + s_llc_sweep.step_hz))
    {
        /* Sweep complete — stop outputs */
        if (s_llc_sweep.mode == SWEEP_MODE_ALL)
        {
            App_Control_LLC_Stop();
        }
        else
        {
            llc_stop_single_phase(s_llc_sweep.mode);
        }

        /* Run ZVS gain curve analysis */
        uint8_t phase_idx;
        switch (s_llc_sweep.mode)
        {
        case SWEEP_MODE_PHASE_D:
            phase_idx = 0U;
            break;
        case SWEEP_MODE_PHASE_E:
            phase_idx = 1U;
            break;
        case SWEEP_MODE_PHASE_F:
            phase_idx = 2U;
            break;
        default:
            /* All-phase sweep: analyze as phase D (shared tank) */
            phase_idx = 0U;
            break;
        }

        zvs_analyze_gain_curve(phase_idx);

        /*
         * For all-phase mode, copy result to phases E and F since
         * they share the same resonant tank and see the same V_out.
         */
        if (s_llc_sweep.mode == SWEEP_MODE_ALL)
        {
            s_llc_sweep.zvs_result.phase[1] =
                s_llc_sweep.zvs_result.phase[0];
            s_llc_sweep.zvs_result.phase[2] =
                s_llc_sweep.zvs_result.phase[0];
        }

        zvs_log_results();

        s_llc_sweep.active = 0U;
        s_llc_sweep.done   = 1U;
        App_Diagnostics_Log("SWEEP COMPLETE");
        return;
    }

    s_llc_sweep.freq_hz -= s_llc_sweep.step_hz;
    App_Control_LLC_SetFrequency(s_llc_sweep.freq_hz);
    s_llc_sweep.settle_count = s_llc_sweep.settle_ms;
}

/**
 * @brief  Check if LLC sweep has completed
 * @return 1 if sweep is done (completed or aborted), 0 otherwise
 */
uint8_t App_Control_LLC_SweepDone(void)
{
    return s_llc_sweep.done;
}

/**
 * @brief  Get ZVS analysis results from last completed sweep
 * @return Pointer to static ZVS result structure
 */
const ZVS_Result_t *App_Control_LLC_GetZVSResult(void)
{
    return &s_llc_sweep.zvs_result;
}

/* ------------------------------------------------------------------ */
/*  LLC PI Integrator Freeze / Unfreeze (Burst Mode)                   */
/* ------------------------------------------------------------------ */

/** @brief  Freeze flag — when set, LLC ISR skips PI integrator updates */
static volatile uint8_t s_llc_pi_frozen = 0U;

/**
 * @brief  Freeze LLC voltage and current PI integrators
 *
 * Called when entering BURST_IDLE. The integrator values are preserved
 * so they resume from the same point when switching resumes.
 */
void App_Control_LLC_PI_Freeze(void)
{
    s_llc_pi_frozen = 1U;
}

/**
 * @brief  Unfreeze LLC PI integrators
 *
 * Called when returning to BURST_RUN from BURST_IDLE.
 */
void App_Control_LLC_PI_Unfreeze(void)
{
    s_llc_pi_frozen = 0U;
}

/**
 * @brief  Apply slow decay to LLC PI integrators during burst idle
 * @param  factor  Multiplicative decay factor (e.g. 0.999 per ms)
 *
 * Prevents stale integrator windup if idle period is long.
 */
void App_Control_LLC_PI_DecayIdle(float factor)
{
    s_pi[PI_ID_VLLC].integrator *= factor;
    s_pi[PI_ID_ILLC].integrator *= factor;
}

/* ------------------------------------------------------------------ */
/*  HRTIM Burst Mode Controller                                        */
/* ------------------------------------------------------------------ */

/**
 * @brief  Configure HRTIM burst mode controller registers
 * @param  f_burst     Burst repetition frequency in Hz (e.g. 20000)
 * @param  duty_burst  Burst on-duty ratio 0.0–1.0 (e.g. 0.30)
 *
 * Uses prescaled fHRTIM clock (170 MHz / 16 = 10.625 MHz).
 * Single-shot mode with software trigger — firmware controls each burst cycle.
 * Minimum on-time enforced to guarantee at least BURST_MIN_LLC_CYCLES
 * LLC switching cycles per burst.
 */
void App_Control_HRTIM_BurstMode_Config(float f_burst, float duty_burst)
{
    HRTIM_BurstModeCfgTypeDef BurstCfg = {0};

    /* Compute period and compare from burst clock frequency */
    uint32_t bmper = (uint32_t)((float)BURST_HRTIM_CLK_HZ / f_burst);
    uint32_t bmcmpr = (uint32_t)((float)bmper * duty_burst);

    /* Enforce minimum on-time: at least BURST_MIN_LLC_CYCLES LLC periods */
    uint32_t min_on_ticks = (BURST_HRTIM_CLK_HZ / LLC_FREQ_MAX_HZ)
                          * BURST_MIN_LLC_CYCLES;
    if (bmcmpr < min_on_ticks)
    {
        bmcmpr = min_on_ticks;
    }

    /* Clamp to 16-bit register range */
    if (bmper > 0xFFFFU)
    {
        bmper = 0xFFFFU;
    }
    if (bmcmpr > bmper)
    {
        bmcmpr = bmper;
    }

    BurstCfg.Mode         = HRTIM_BURSTMODE_SINGLESHOT;
    BurstCfg.ClockSource  = HRTIM_BURSTMODECLOCKSOURCE_FHRTIM;
    BurstCfg.Prescaler    = HRTIM_BURSTMODEPRESCALER_DIV16;
    BurstCfg.PreloadEnable = HRIM_BURSTMODEPRELOAD_ENABLED;
    BurstCfg.Trigger      = HRTIM_BURSTMODETRIGGER_NONE;
    BurstCfg.IdleDuration = bmper - bmcmpr;
    BurstCfg.Period       = bmper;

    HAL_HRTIM_BurstModeConfig(&hhrtim1, &BurstCfg);
}

/**
 * @brief  Enable HRTIM burst mode controller
 */
void App_Control_HRTIM_BurstMode_Enable(void)
{
    HAL_HRTIM_BurstModeCtl(&hhrtim1, HRTIM_BURSTMODECTL_ENABLED);
}

/**
 * @brief  Disable HRTIM burst mode controller
 */
void App_Control_HRTIM_BurstMode_Disable(void)
{
    HAL_HRTIM_BurstModeCtl(&hhrtim1, HRTIM_BURSTMODECTL_DISABLED);
}

/**
 * @brief  Software-trigger one burst cycle
 */
void App_Control_HRTIM_BurstMode_SWTrigger(void)
{
    HAL_HRTIM_BurstModeSoftwareTrigger(&hhrtim1);
}
