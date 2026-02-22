/**
 * @file    app_statemachine.c
 * @brief   10-state application state machine — runs at 1 kHz (TIM6)
 */

#include "app_statemachine.h"
#include "app_can.h"
#include "app_diagnostics.h"
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

/** @brief  LED toggle period in DISABLED state (ms) */
#define DISABLED_LED_BLINK_MS  500U

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
        State_Transition(STATE_PLL_LOCK);
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
        /* TODO: EP-03-003 — SRF-PLL locking to grid */
        break;

    case STATE_SOFT_START_PFC:
        /* TODO: EP-03-004 — PFC I_d* ramp 0 -> rated over 200 ms */
        break;

    case STATE_SOFT_START_LLC:
        /* TODO: EP-03-005 — LLC frequency ramp 300 kHz -> operating point */
        break;

    case STATE_RUN:
        /* TODO: EP-03-006 — Normal operation — CC/CV charging */
        break;

    case STATE_DERATE:
        /* TODO: EP-03-007 — Thermal or CAN derate active */
        break;

    case STATE_FAULT:
        /* TODO: EP-03-008 — Fault active — outputs disabled */
        break;

    case STATE_SHUTDOWN:
        /* TODO: EP-03-009 — Controlled ramp-down */
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
 * @brief  Called from TIM6 ISR to set tick flag
 */
void App_SM_TickISR(void)
{
    s_tick_flag = 1;
}
