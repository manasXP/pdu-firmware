/**
 * @file    app_statemachine.h
 * @brief   10-state application state machine
 *
 * States: POWER_ON -> STANDBY -> PLL_LOCK -> SOFT_START_PFC ->
 *         SOFT_START_LLC -> RUN -> DERATE -> FAULT -> SHUTDOWN -> DISABLED
 */

#ifndef APP_STATEMACHINE_H
#define APP_STATEMACHINE_H

#include <stdint.h>

typedef enum
{
    STATE_POWER_ON = 0,
    STATE_STANDBY,
    STATE_PLL_LOCK,
    STATE_SOFT_START_PFC,
    STATE_SOFT_START_LLC,
    STATE_RUN,
    STATE_DERATE,
    STATE_FAULT,
    STATE_SHUTDOWN,
    STATE_DISABLED,
    STATE_COUNT
} AppState_t;

void        App_SM_Init(void);
void        App_SM_Run(void);
void        App_SM_TickISR(void);
AppState_t  App_SM_GetState(void);
const char *App_SM_GetStateName(void);

#endif /* APP_STATEMACHINE_H */
