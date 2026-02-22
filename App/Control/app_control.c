/**
 * @file    app_control.c
 * @brief   PFC dq-frame control, LLC PFM, PI with anti-windup
 */

#include "app_control.h"
#include "app_pll.h"
#include "app_adc.h"
#include "app_diagnostics.h"
#include "main.h"
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
/*  LLC Sweep State                                                    */
/* ------------------------------------------------------------------ */

typedef struct
{
    uint32_t freq_hz;
    uint32_t step_hz;
    uint32_t settle_ms;
    uint32_t settle_count;
    uint8_t  active;
    uint8_t  done;
} LLC_Sweep_t;

static LLC_Sweep_t s_llc_sweep;

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
                                  | HRTIM_TIMFAULTENABLE_FAULT2;
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

/* ------------------------------------------------------------------ */
/*  ISR Callbacks                                                      */
/* ------------------------------------------------------------------ */

void App_Control_PFC_ISR(void)
{
    const ADC_Readings_t *adc = App_ADC_GetReadings();
    static const float dt = 1.0f / (float)PFC_ISR_FREQ_HZ;

    App_PLL_Update(adc->v_grid_a, adc->v_grid_b, dt);

    /* Open-loop: static duty — no control action needed (EP-04-001) */
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

    uint32_t llc_faults = HRTIM_TIMFAULTENABLE_FAULT3
                        | HRTIM_TIMFAULTENABLE_FAULT4;

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
 * @brief  Start open-loop frequency sweep: 300 kHz → 100 kHz
 *
 * Initializes sweep state and starts LLC outputs. Call App_Control_LLC_Sweep()
 * from the main loop at 1 kHz to advance the sweep.
 */
void App_Control_LLC_SweepStart(void)
{
    s_llc_sweep.freq_hz      = LLC_FREQ_MAX_HZ;
    s_llc_sweep.step_hz      = LLC_SWEEP_STEP_HZ;
    s_llc_sweep.settle_ms    = LLC_SWEEP_SETTLE_MS;
    s_llc_sweep.settle_count = LLC_SWEEP_SETTLE_MS;
    s_llc_sweep.active       = 1U;
    s_llc_sweep.done         = 0U;

    App_Control_LLC_SetFrequency(LLC_FREQ_MAX_HZ);
    App_Control_LLC_Start();
}

/**
 * @brief  Advance LLC frequency sweep — call from main loop at 1 kHz
 *
 * Each step: wait for settling, log V_out/I_out, check safety limits,
 * decrement frequency by step_hz. Sweep completes at LLC_FREQ_MIN_HZ.
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

    /* Log sweep data point */
    char log_buf[64];
    (void)snprintf(log_buf, sizeof(log_buf),
                   "SWEEP %lu Hz Vout=%.1f Iout=%.1f",
                   (unsigned long)s_llc_sweep.freq_hz, (double)v_out,
                   (double)i_out);
    App_Diagnostics_Log(log_buf);

    /* Safety check — abort on OVP or OCP */
    if ((v_out > OUT_OVP_THRESHOLD_V) || (i_out > PFC_OCP_THRESHOLD_A))
    {
        App_Control_LLC_Stop();
        s_llc_sweep.active = 0U;
        s_llc_sweep.done   = 1U;
        App_Diagnostics_Log("SWEEP ABORT: safety limit");
        return;
    }

    /* Decrement frequency */
    if (s_llc_sweep.freq_hz <= (LLC_FREQ_MIN_HZ + s_llc_sweep.step_hz))
    {
        /* Sweep complete */
        App_Control_LLC_Stop();
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
