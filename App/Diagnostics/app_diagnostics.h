/**
 * @file    app_diagnostics.h
 * @brief   UART debug CLI, fault log dump, calibration data
 */

#ifndef APP_DIAGNOSTICS_H
#define APP_DIAGNOSTICS_H

void App_Diagnostics_Init(void);
void App_Diagnostics_Poll(void);
void App_Diagnostics_Log(const char *msg);

#endif /* APP_DIAGNOSTICS_H */
