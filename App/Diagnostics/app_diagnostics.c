/**
 * @file    app_diagnostics.c
 * @brief   UART CLI — status, gain tuning, fault log, register peek/poke
 *
 * Commands:
 *   status  — Print state, Vbus, Iout, Vout, temperatures
 *   gain <loop> <kp> <ki> — Adjust PI gains at runtime
 *   fault   — Dump fault log ring buffer
 *   version — Print firmware version
 */

#include "app_diagnostics.h"

void App_Diagnostics_Init(void)
{
    /* TODO: Configure UART DMA Tx (non-blocking), Rx circular buffer */
}

void App_Diagnostics_Poll(void)
{
    /* TODO: Parse incoming UART commands, dispatch handlers */
}
