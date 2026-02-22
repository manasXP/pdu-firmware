/**
 * @file    app_diagnostics.h
 * @brief   UART debug CLI, fault log dump, PFC open-loop test, DMA TX
 */

#ifndef APP_DIAGNOSTICS_H
#define APP_DIAGNOSTICS_H

void App_Diagnostics_Init(void);
void App_Diagnostics_Poll(void);
void App_Diagnostics_Log(const char *msg);
void App_Diagnostics_RxCallback(void);
void App_Diagnostics_TxCpltCallback(void);

#endif /* APP_DIAGNOSTICS_H */
