/**
 * @file    app_diagnostics.c
 * @brief   UART CLI — PFC open-loop test, status, fault log
 *
 * Commands:
 *   pfc start          — Start PFC outputs at current duty (open-loop)
 *   pfc stop           — Stop PFC outputs
 *   pfc duty <0-95>    — Set PFC duty cycle (percent)
 *   pfc status         — Print PFC switching parameters and bus voltage
 *   enable             — Request state machine enable (STANDBY → PLL_LOCK)
 *   status             — Print state, Vbus, Iout, Vout, temperatures
 *   fault              — Dump active fault info
 *   version            — Print firmware version
 */

#include "app_diagnostics.h"
#include "app_adc.h"
#include "app_control.h"
#include "app_protection.h"
#include "app_statemachine.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ------------------------------------------------------------------ */
/*  UART CLI Constants                                                  */
/* ------------------------------------------------------------------ */

/** @brief  UART transmit timeout for blocking log calls (ms) */
#define DIAG_UART_TX_TIMEOUT_MS  50U

/** @brief  Rx line buffer size (max command length) */
#define DIAG_RX_BUF_SIZE         64U

/** @brief  PFC open-loop test running flag */
static uint8_t s_pfc_test_active;

/* ------------------------------------------------------------------ */
/*  UART Rx state                                                       */
/* ------------------------------------------------------------------ */

static uint8_t  s_rx_buf[DIAG_RX_BUF_SIZE];
static uint16_t s_rx_idx;
static uint8_t  s_rx_byte;

/* ------------------------------------------------------------------ */
/*  Forward declarations                                                */
/* ------------------------------------------------------------------ */

static void diag_process_command(char *cmd);
static void diag_cmd_pfc(const char *args);
static void diag_cmd_status(void);
static void diag_cmd_fault(void);
static void diag_cmd_version(void);
static void diag_print(const char *msg);

/* ------------------------------------------------------------------ */
/*  Public API                                                          */
/* ------------------------------------------------------------------ */

void App_Diagnostics_Init(void)
{
    s_rx_idx = 0U;
    s_pfc_test_active = 0U;

    /* Start single-byte UART Rx interrupt */
    HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);

    diag_print("PDU CLI ready");
}

void App_Diagnostics_Poll(void)
{
    /* Command parsing is driven by UART Rx callback — nothing to poll */
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

/* ------------------------------------------------------------------ */
/*  UART Rx callback                                                    */
/* ------------------------------------------------------------------ */

/**
 * @brief  HAL UART Rx complete callback — accumulates bytes into line buffer
 *
 * Called from USART2 ISR context. On CR or LF, null-terminates the buffer
 * and dispatches the command. Re-arms single-byte Rx interrupt.
 */
void App_Diagnostics_RxCallback(void)
{
    uint8_t ch = s_rx_byte;

    /* Echo character */
    (void)HAL_UART_Transmit(&huart2, &ch, 1U, DIAG_UART_TX_TIMEOUT_MS);

    if ((ch == '\r') || (ch == '\n'))
    {
        if (s_rx_idx > 0U)
        {
            s_rx_buf[s_rx_idx] = '\0';
            diag_print("");  /* newline after echo */
            diag_process_command((char *)s_rx_buf);
            s_rx_idx = 0U;
        }
    }
    else if (s_rx_idx < (DIAG_RX_BUF_SIZE - 1U))
    {
        s_rx_buf[s_rx_idx] = ch;
        s_rx_idx++;
    }

    /* Re-arm Rx interrupt */
    HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);
}

/* ------------------------------------------------------------------ */
/*  Command dispatcher                                                  */
/* ------------------------------------------------------------------ */

static void diag_process_command(char *cmd)
{
    /* Skip leading whitespace */
    while (*cmd == ' ')
    {
        cmd++;
    }

    if (strncmp(cmd, "pfc ", 4U) == 0)
    {
        diag_cmd_pfc(cmd + 4U);
    }
    else if (strcmp(cmd, "pfc") == 0)
    {
        diag_cmd_pfc("");
    }
    else if (strcmp(cmd, "enable") == 0)
    {
        App_SM_RequestEnable();
        diag_print("Enable requested");
    }
    else if (strcmp(cmd, "status") == 0)
    {
        diag_cmd_status();
    }
    else if (strcmp(cmd, "fault") == 0)
    {
        diag_cmd_fault();
    }
    else if (strcmp(cmd, "version") == 0)
    {
        diag_cmd_version();
    }
    else if (strlen(cmd) > 0U)
    {
        diag_print("Unknown command. Try: pfc, enable, status, fault, version");
    }
}

/* ------------------------------------------------------------------ */
/*  PFC test commands                                                   */
/* ------------------------------------------------------------------ */

static void diag_cmd_pfc(const char *args)
{
    char buf[80];

    /* Skip leading whitespace */
    while (*args == ' ')
    {
        args++;
    }

    if (strncmp(args, "start", 5U) == 0)
    {
        if (s_pfc_test_active != 0U)
        {
            diag_print("PFC already running");
            return;
        }
        App_Control_PFC_Start();
        s_pfc_test_active = 1U;
        diag_print("PFC started (open-loop, 30% duty)");
    }
    else if (strncmp(args, "stop", 4U) == 0)
    {
        App_Control_PFC_Stop();
        s_pfc_test_active = 0U;
        diag_print("PFC stopped");
    }
    else if (strncmp(args, "duty ", 5U) == 0)
    {
        int pct = atoi(args + 5U);

        if ((pct < 0) || (pct > 95))
        {
            diag_print("Duty must be 0-95 (percent)");
            return;
        }
        float duty = (float)pct / 100.0f;
        App_Control_PFC_SetDuty(duty);
        (void)snprintf(buf, sizeof(buf), "PFC duty set to %d%%", pct);
        diag_print(buf);
    }
    else if (strncmp(args, "status", 6U) == 0)
    {
        const ADC_Readings_t *adc = App_ADC_GetReadings();

        (void)snprintf(buf, sizeof(buf), "PFC test: %s",
                       (s_pfc_test_active != 0U) ? "RUNNING" : "STOPPED");
        diag_print(buf);

        (void)snprintf(buf, sizeof(buf), "Vbus=%.1f V  Ia=%.2f A  Ib=%.2f A  Ic=%.2f A",
                       (double)adc->v_bus,
                       (double)adc->i_phase_a,
                       (double)adc->i_phase_b,
                       (double)adc->i_phase_c);
        diag_print(buf);

        (void)snprintf(buf, sizeof(buf), "Vgrid: A=%.1f  B=%.1f  C=%.1f V",
                       (double)adc->v_grid_a,
                       (double)adc->v_grid_b,
                       (double)adc->v_grid_c);
        diag_print(buf);
    }
    else
    {
        diag_print("Usage: pfc start|stop|duty <0-95>|status");
    }
}

/* ------------------------------------------------------------------ */
/*  Status / Fault / Version commands                                   */
/* ------------------------------------------------------------------ */

static void diag_cmd_status(void)
{
    char buf[80];
    const ADC_Readings_t *adc = App_ADC_GetReadings();

    (void)snprintf(buf, sizeof(buf), "State: %s", App_SM_GetStateName());
    diag_print(buf);

    (void)snprintf(buf, sizeof(buf), "Vbus=%.1f V  Vout=%.1f V  Iout=%.2f A",
                   (double)adc->v_bus, (double)adc->v_out,
                   (double)adc->i_out);
    diag_print(buf);

    (void)snprintf(buf, sizeof(buf), "Tsic_pfc=%.1f  Tsic_llc=%.1f  Tmag=%.1f  Tamb=%.1f C",
                   (double)adc->t_sic_pfc, (double)adc->t_sic_llc,
                   (double)adc->t_magnetics, (double)adc->t_ambient);
    diag_print(buf);

    (void)snprintf(buf, sizeof(buf), "V12aux=%.2f V  Vcap_top=%.1f V  Vcap_bot=%.1f V",
                   (double)adc->v_aux_12v, (double)adc->v_cap_top,
                   (double)adc->v_cap_bot);
    diag_print(buf);
}

static void diag_cmd_fault(void)
{
    char buf[80];

    if (App_Protection_IsFaultActive() == 0U)
    {
        diag_print("No active faults");
        return;
    }

    const FaultStatus_t *fs = App_Protection_GetActiveFault();

    (void)snprintf(buf, sizeof(buf), "Fault: src=%u sev=%u latched=%u retries=%u",
                   (unsigned)fs->active_source, (unsigned)fs->active_severity,
                   (unsigned)fs->latched, (unsigned)fs->retry_count);
    diag_print(buf);
}

static void diag_cmd_version(void)
{
    char buf[48];
    (void)snprintf(buf, sizeof(buf), "PDU FW v%d.%d.%d",
                   FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_BUILD);
    diag_print(buf);
}

/* ------------------------------------------------------------------ */
/*  Helper                                                              */
/* ------------------------------------------------------------------ */

static void diag_print(const char *msg)
{
    App_Diagnostics_Log(msg);
}
