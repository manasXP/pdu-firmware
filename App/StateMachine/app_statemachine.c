/**
 * @file    app_statemachine.c
 * @brief   10-state application state machine — runs at 1 kHz (TIM6)
 */

#include "app_statemachine.h"
#include "app_can.h"

static volatile AppState_t s_state = STATE_POWER_ON;
static volatile uint8_t    s_tick_flag = 0;

void App_SM_Init(void)
{
    s_state = STATE_POWER_ON;
}

AppState_t App_SM_GetState(void)
{
    return s_state;
}

static void State_Transition(AppState_t next)
{
    if (next != s_state)
    {
        s_state = next;
        App_CAN_BroadcastState((uint8_t)s_state);
    }
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
        /* System_Init() complete -> STANDBY */
        State_Transition(STATE_STANDBY);
        break;

    case STATE_STANDBY:
        /* Wait for CAN enable command or local start */
        break;

    case STATE_PLL_LOCK:
        /* SRF-PLL locking to grid */
        break;

    case STATE_SOFT_START_PFC:
        /* PFC I_d* ramp 0 -> rated over 200 ms */
        break;

    case STATE_SOFT_START_LLC:
        /* LLC frequency ramp 300 kHz -> operating point */
        break;

    case STATE_RUN:
        /* Normal operation — CC/CV charging */
        break;

    case STATE_DERATE:
        /* Thermal or CAN derate active */
        break;

    case STATE_FAULT:
        /* Fault active — outputs disabled */
        break;

    case STATE_SHUTDOWN:
        /* Controlled ramp-down */
        break;

    case STATE_DISABLED:
        /* Init failure — requires power cycle */
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
