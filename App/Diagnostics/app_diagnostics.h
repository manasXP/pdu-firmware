/**
 * @file    app_diagnostics.h
 * @brief   UART debug CLI, fault log dump, PFC open-loop test
 */

#ifndef APP_DIAGNOSTICS_H
#define APP_DIAGNOSTICS_H

void App_Diagnostics_Init(void);
void App_Diagnostics_Poll(void);
void App_Diagnostics_Log(const char *msg);
void App_Diagnostics_RxCallback(void);

#endif /* APP_DIAGNOSTICS_H */
