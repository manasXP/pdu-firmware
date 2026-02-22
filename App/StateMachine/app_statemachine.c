/**
 * @file    app_statemachine.c
 * @brief   10-state application state machine — runs at 1 kHz (TIM6)
 */

#include "app_statemachine.h"
#include "app_can.h"
#include "app_control.h"
#include "app_npbalance.h"
#include "app_diagnostics.h"
#include "app_pll.h"
#include "app_powerseq.h"
#include "app_protection.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/*  State name table                                                   */
/* ------------------------------------------------------------------ */

static const char *const s_state_names[STATE_COUNT] =
{
    "POWER_ON",
    "STANDBY",
    "PLL_LOCK",
    "SOFT_START_PFC",
    "SOFT_START_LLC",
    "RUN",
    "DERATE",
    "FAULT",
    "SHUTDOWN",
    "DISABLED"
};

/* ------------------------------------------------------------------ */
/*  Module state                                                       */
/* ------------------------------------------------------------------ */

static volatile AppState_t s_state          = STATE_POWER_ON;
static volatile uint8_t    s_tick_flag      = 0;
static uint32_t            s_state_entry_tick = 0;
static volatile uint8_t    s_enable_requested = 0;

/* ------------------------------------------------------------------ */
/*  LED timing constants                                               */
/* ------------------------------------------------------------------ */

/** @brief  Fault retry max — must match app_protection.c */
#define FAULT_RETRY_MAX        3U

/** @brief  LED toggle period in DISABLED state (ms) */
#define DISABLED_LED_BLINK_MS  500U

/** @brief  LED toggle period in PLL_LOCK state (ms) — fast blink */
#define PLL_LOCK_LED_BLINK_MS  100U

/** @brief  Shutdown failsafe timeout (ms) */
#define SHUTDOWN_TIMEOUT_MS    3000U

/* ------------------------------------------------------------------ */
/*  State transition helper                                            */
/* ------------------------------------------------------------------ */

static void State_Transition(AppState_t next)
{
    if (next != s_state)
    {
        char buf[48];
        (void)snprintf(buf, sizeof(buf), "[SM] %s -> %s",
                       s_state_names[s_state], s_state_names[next]);
        App_Diagnostics_Log(buf);

        s_state = next;
        s_state_entry_tick = HAL_GetTick();
        App_CAN_BroadcastState((uint8_t)s_state);
    }
}

/* ------------------------------------------------------------------ */
/*  State handlers                                                     */
/* ------------------------------------------------------------------ */

/**
 * @brief  POWER_ON — validate HRTIM DLL and ADC calibration
 *
 * Toggles debug LED each tick to signal init in progress.
 * Transitions to STANDBY on success, DISABLED on timeout.
 */
static void state_power_on(void)
{
    /* Toggle debug LED to signal init in progress */
    HAL_GPIO_TogglePin(DEBUG_LED_PORT, DEBUG_LED_PIN);

    /* Check HRTIM DLL ready */
    uint32_t dll_ready = HRTIM1->sCommonRegs.ISR & HRTIM_ISR_DLLRDY;

    /* Check ADC calibration completed (CALFACT != 0 after cal) */
    uint32_t adc_cal_ok = hadc1.Instance->CALFACT;

    if ((dll_ready != 0U) && (adc_cal_ok != 0U))
    {
        State_Transition(STATE_STANDBY);
        return;
    }

    /* Timeout check */
    uint32_t elapsed = HAL_GetTick() - s_state_entry_tick;

    if (elapsed >= STARTUP_TIMEOUT_MS)
    {
        App_Diagnostics_Log("[SM] POWER_ON timeout — init failed");
        State_Transition(STATE_DISABLED);
    }
}

/**
 * @brief  STANDBY — idle, ready for enable command
 *
 * Debug LED steady ON as ready indicator.
 * Waits for s_enable_requested flag (set by future CAN command).
 */
static void state_standby(void)
{
    /* Steady LED ON — ready indicator */
    HAL_GPIO_WritePin(DEBUG_LED_PORT, DEBUG_LED_PIN, GPIO_PIN_SET);

    if (s_enable_requested != 0U)
    {
        s_enable_requested = 0U;
        App_Control_PFC_Start();
        App_PLL_Reset();
        State_Transition(STATE_PLL_LOCK);
    }
}

/**
 * @brief  PLL_LOCK — wait for SRF-PLL to lock onto grid angle
 *
 * Fast-blinks LED (100 ms toggle) as visual indicator.
 * Transitions to SOFT_START_PFC on lock, FAULT on 2 s timeout.
 */
static void state_pll_lock(void)
{
    /* Fast LED blink — 100 ms toggle */
    uint32_t elapsed = HAL_GetTick() - s_state_entry_tick;

    if ((elapsed / PLL_LOCK_LED_BLINK_MS) % 2U == 0U)
    {
        HAL_GPIO_WritePin(DEBUG_LED_PORT, DEBUG_LED_PIN, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(DEBUG_LED_PORT, DEBUG_LED_PIN, GPIO_PIN_RESET);
    }

    /* Check PLL lock */
    if (App_PLL_IsLocked() != 0U)
    {
        State_Transition(STATE_SOFT_START_PFC);
        return;
    }

    /* Timeout check */
    if (elapsed >= PLL_LOCK_TIMEOUT_MS)
    {
        App_Diagnostics_Log("[SM] PLL lock timeout");
        Fault_Enter(FAULT_PLL_UNLOCK);
        State_Transition(STATE_FAULT);
    }
}

/**
 * @brief  SOFT_START_PFC — duty ramp + NTC bypass
 *
 * PFC outputs are already running (started in STANDBY→PLL_LOCK).
 * Ramps duty from 5% to 30% over 200 ms while monitoring Vbus.
 * NTC bypass relay engages at 80% of target bus voltage.
 * LED triple-blinks (150 ms period) to distinguish from PLL_LOCK.
 */
static void state_soft_start_pfc(void)
{
    uint32_t elapsed = HAL_GetTick() - s_state_entry_tick;

    /* First tick after entry — begin PFC soft-start */
    if (elapsed == 0U)
    {
        PFC_SoftStart_Begin();
    }

    /* LED triple-blink — 150 ms period */
    uint32_t phase = (elapsed / 50U) % 6U;
    if ((phase == 0U) || (phase == 2U) || (phase == 4U))
    {
        HAL_GPIO_WritePin(DEBUG_LED_PORT, DEBUG_LED_PIN, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(DEBUG_LED_PORT, DEBUG_LED_PIN, GPIO_PIN_RESET);
    }

    /* Advance PFC soft-start */
    if (PFC_SoftStart_Tick() != 0U)
    {
        App_Diagnostics_Log("[SM] PFC soft-start complete");
        State_Transition(STATE_SOFT_START_LLC);
        return;
    }

    /* Timeout */
    if (elapsed >= STARTUP_TIMEOUT_MS)
    {
        App_Diagnostics_Log("[SM] PFC soft-start timeout");
        App_Control_PFC_Stop();
        Fault_Enter(FAULT_STARTUP_TIMEOUT);
        State_Transition(STATE_FAULT);
    }
}

/**
 * @brief  SOFT_START_LLC — staggered phase enable + frequency ramp
 *
 * Uses PowerSequence API for controlled LLC soft-start with
 * staggered phase enables and frequency ramp to target voltage.
 * LED double-blinks (200 ms period) to distinguish from PLL_LOCK.
 */
static void state_soft_start_llc(void)
{
    uint32_t elapsed = HAL_GetTick() - s_state_entry_tick;

    /* First tick after entry — begin soft-start sequence */
    if (elapsed == 0U)
    {
        LLC_SoftStart_Begin();
    }

    /* LED double-blink — 200 ms period */
    uint32_t phase = (elapsed / 100U) % 4U;
    if ((phase == 0U) || (phase == 2U))
    {
        HAL_GPIO_WritePin(DEBUG_LED_PORT, DEBUG_LED_PIN, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(DEBUG_LED_PORT, DEBUG_LED_PIN, GPIO_PIN_RESET);
    }

    /* Advance soft-start (1 kHz tick rate) */
    if (LLC_SoftStart_Tick() != 0U)
    {
        /* ZVS results already logged by sweep completion handler */
        (void)App_Control_LLC_GetZVSResult();
        App_Diagnostics_Log("[SM] LLC soft-start complete");
        State_Transition(STATE_RUN);
        return;
    }

    /* Timeout — soft-start took too long */
    if (elapsed >= STARTUP_TIMEOUT_MS)
    {
        App_Diagnostics_Log("[SM] LLC soft-start timeout");
        App_Control_LLC_Stop();
        App_Control_PFC_Stop();
        Fault_Enter(FAULT_STARTUP_TIMEOUT);
        State_Transition(STATE_FAULT);
    }
}

/**
 * @brief  STATE_FAULT — outputs disabled, recovery or latch
 *
 * LED solid ON (fault indication).
 * Flushes fault log to flash, checks recovery eligibility.
 * Transitions: STANDBY (diag clear), PLL_LOCK (major recovery),
 *              DISABLED (latched + retries exhausted).
 */
static void state_fault(void)
{
    /* LED solid ON — fault indication */
    HAL_GPIO_WritePin(DEBUG_LED_PORT, DEBUG_LED_PIN, GPIO_PIN_SET);

    /* Deferred flash write */
    App_Protection_FlashSync();

    /* Check auto-recovery for major faults */
    Fault_Recovery_Check();

    const FaultStatus_t *fs = App_Protection_GetActiveFault();

    /* DiagClear was issued — fault cleared externally */
    if (fs->active_source >= FAULT_COUNT)
    {
        App_Diagnostics_Log("[SM] Fault cleared by diag — STANDBY");
        State_Transition(STATE_STANDBY);
        return;
    }

    /* Latched + retries exhausted → permanent disable */
    if ((fs->latched != 0U) && (fs->retry_count > FAULT_RETRY_MAX))
    {
        App_Diagnostics_Log("[SM] Fault latched — DISABLED");
        State_Transition(STATE_DISABLED);
        return;
    }

    /* Major fault auto-recovered (fault_active cleared by Recovery_Check) */
    if (App_Protection_IsFaultActive() == 0U)
    {
        App_Diagnostics_Log("[SM] Major fault recovered — PLL_LOCK");
        App_Control_PFC_Start();
        State_Transition(STATE_PLL_LOCK);
    }
}

/**
 * @brief  STATE_SHUTDOWN — controlled ramp-down sequence
 *
 * Uses Shutdown_Begin/Tick for smooth ramp-down:
 *   Phase 1: LLC stop (immediate)
 *   Phase 2: PFC duty ramp 30% → 0% over 100 ms
 *   Phase 3: Wait for I_out < 0.5 A, then open contactor
 *   Phase 4: Open all relays (NTC, PFC, LLC)
 *   Timeout: 3 s failsafe → FAULT
 */
static void state_shutdown(void)
{
    uint32_t elapsed = HAL_GetTick() - s_state_entry_tick;

    /* First tick after entry — begin shutdown sequence */
    if (elapsed == 0U)
    {
        Shutdown_Begin();
    }

    /* Advance shutdown */
    if (Shutdown_Tick() != 0U)
    {
        App_Diagnostics_Log("[SM] Shutdown complete — STANDBY");
        State_Transition(STATE_STANDBY);
        return;
    }

    /* Timeout failsafe */
    if (elapsed >= SHUTDOWN_TIMEOUT_MS)
    {
        /* Force all outputs and relays off */
        App_Control_LLC_Stop();
        App_Control_PFC_Stop();
        HAL_GPIO_WritePin(RELAY_PFC_PORT, RELAY_PFC_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RELAY_LLC_PORT, RELAY_LLC_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RELAY_NTC_PORT, RELAY_NTC_PIN, GPIO_PIN_RESET);
        App_Diagnostics_Log("[SM] Shutdown timeout — forced");
        Fault_Enter(FAULT_STARTUP_TIMEOUT);
        State_Transition(STATE_FAULT);
    }
}

/**
 * @brief  DISABLED — safe state, requires power cycle to exit
 *
 * Ensures relay GPIOs are driven low (safe outputs).
 * Slow LED blink (500 ms toggle) to indicate disabled/fault.
 */
static void state_disabled(void)
{
    /* Ensure relay outputs are safe */
    HAL_GPIO_WritePin(RELAY_PFC_PORT, RELAY_PFC_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_LLC_PORT, RELAY_LLC_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_NTC_PORT, RELAY_NTC_PIN, GPIO_PIN_RESET);

    /* Slow LED blink — 500 ms toggle */
    uint32_t elapsed = HAL_GetTick() - s_state_entry_tick;

    if ((elapsed / DISABLED_LED_BLINK_MS) % 2U == 0U)
    {
        HAL_GPIO_WritePin(DEBUG_LED_PORT, DEBUG_LED_PIN, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(DEBUG_LED_PORT, DEBUG_LED_PIN, GPIO_PIN_RESET);
    }

    /* No exit — requires power cycle */
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

void App_SM_Init(void)
{
    s_state = STATE_POWER_ON;
    s_state_entry_tick = HAL_GetTick();
    s_enable_requested = 0U;
    App_Diagnostics_Log("[SM] Init — entering POWER_ON");
}

AppState_t App_SM_GetState(void)
{
    return s_state;
}

const char *App_SM_GetStateName(void)
{
    AppState_t st = s_state;

    if (st < STATE_COUNT)
    {
        return s_state_names[st];
    }

    return "UNKNOWN";
}

void App_SM_Run(void)
{
    if (!s_tick_flag)
    {
        return;
    }
    s_tick_flag = 0;

    switch (s_state)
    {
    case STATE_POWER_ON:
        state_power_on();
        break;

    case STATE_STANDBY:
        state_standby();
        break;

    case STATE_PLL_LOCK:
        state_pll_lock();
        break;

    case STATE_SOFT_START_PFC:
        state_soft_start_pfc();
        break;

    case STATE_SOFT_START_LLC:
        state_soft_start_llc();
        break;

    case STATE_RUN:
        NP_Balance_Update();
        /* TODO: EP-03-006 — Normal operation — CC/CV charging */
        break;

    case STATE_DERATE:
        NP_Balance_Update();
        /* TODO: EP-03-007 — Thermal or CAN derate active */
        break;

    case STATE_FAULT:
        state_fault();
        break;

    case STATE_SHUTDOWN:
        state_shutdown();
        break;

    case STATE_DISABLED:
        state_disabled();
        break;

    default:
        State_Transition(STATE_FAULT);
        break;
    }
}

/**
 * @brief  Request enable — triggers STANDBY → PLL_LOCK transition
 *
 * Can be called from CLI or CAN command handler.
 */
void App_SM_RequestEnable(void)
{
    s_enable_requested = 1U;
}

/**
 * @brief  Called from TIM6 ISR to set tick flag
 */
void App_SM_TickISR(void)
{
    s_tick_flag = 1;
}
