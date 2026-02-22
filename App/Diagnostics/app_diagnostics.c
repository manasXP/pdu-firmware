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
#include "main.h"
#include <string.h>

/** @brief  UART transmit timeout for blocking log calls (ms) */
#define DIAG_UART_TX_TIMEOUT_MS  50U

void App_Diagnostics_Init(void)
{
    /* TODO: Configure UART DMA Tx (non-blocking), Rx circular buffer */
}

void App_Diagnostics_Poll(void)
{
    /* TODO: Parse incoming UART commands, dispatch handlers */
}

/**
 * @brief  Blocking UART log — sends message + CRLF on USART2
 * @param  msg  Null-terminated string to transmit
 */
void App_Diagnostics_Log(const char *msg)
{
    if (msg == NULL)
    {
        return;
    }

    uint16_t len = (uint16_t)strlen(msg);

    (void)HAL_UART_Transmit(&huart2, (const uint8_t *)msg, len,
                            DIAG_UART_TX_TIMEOUT_MS);
    (void)HAL_UART_Transmit(&huart2, (const uint8_t *)"\r\n", 2U,
                            DIAG_UART_TX_TIMEOUT_MS);
}
