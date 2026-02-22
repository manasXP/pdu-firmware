/**
 * @file    app_diagnostics.c
 * @brief   UART CLI — PFC open-loop test, PI tuning, status, fault log
 *
 * Commands:
 *   pfc start              — Start PFC outputs at current duty (open-loop)
 *   pfc stop               — Stop PFC outputs
 *   pfc duty <0-95>        — Set PFC duty cycle (percent)
 *   pfc status             — Print PFC switching parameters and bus voltage
 *   pi show                — Print all PI controller gains and Tt
 *   pi set <name> <Kp> <Ki> — Set PI gains (Tt auto-computed)
 *   pi step <name>         — Run offline step-response test (20 iterations)
 *   np                     — Print neutral point error and duty offset
 *   enable                 — Request state machine enable (STANDBY → PLL_LOCK)
 *   status                 — Print state, Vbus, Iout, Vout, temperatures
 *   fault                  — Dump active fault info
 *   version                — Print firmware version
 */

#include "app_diagnostics.h"
#include "app_adc.h"
#include "app_control.h"
#include "app_npbalance.h"
#include "app_protection.h"
#include "app_statemachine.h"
#include "main.h"
#include <math.h>
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
static void diag_cmd_pi(const char *args);
static void diag_cmd_np(void);
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
    else if (strncmp(cmd, "pi ", 3U) == 0)
    {
        diag_cmd_pi(cmd + 3U);
    }
    else if (strcmp(cmd, "pi") == 0)
    {
        diag_cmd_pi("");
    }
    else if (strcmp(cmd, "np") == 0)
    {
        diag_cmd_np();
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
        diag_print("Unknown command. Try: pfc, pi, np, enable, status, fault, version");
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
/*  PI tuning commands                                                  */
/* ------------------------------------------------------------------ */

/**
 * @brief  Resolve a controller name string to PI_Index_t
 * @return PI_ID_COUNT if name not recognized
 */
static PI_Index_t pi_parse_name(const char *name)
{
    for (PI_Index_t i = 0; i < PI_ID_COUNT; i++)
    {
        if (strcmp(name, PI_GetName(i)) == 0)
        {
            return i;
        }
    }
    return PI_ID_COUNT;
}

/**
 * @brief  'pi show' — print gains, Tt, and saturation limits for all instances
 */
static void diag_pi_show(void)
{
    char buf[96];

    for (PI_Index_t i = 0; i < PI_ID_COUNT; i++)
    {
        const PI_Controller_t *pi = PI_GetController(i);
        (void)snprintf(buf, sizeof(buf),
                       "  %-4s  Kp=%.4f  Ki=%.4f  Tt=%.4f  [%.1f, %.1f]",
                       PI_GetName(i),
                       (double)pi->Kp, (double)pi->Ki, (double)pi->Tt,
                       (double)pi->out_min, (double)pi->out_max);
        diag_print(buf);
    }
}

/**
 * @brief  'pi set <name> <Kp> <Ki>' — update gains at runtime
 */
static void diag_pi_set(const char *args)
{
    char buf[80];
    char name[8];
    float kp;
    float ki;

    if (sscanf(args, "%7s %f %f", name, &kp, &ki) != 3)
    {
        diag_print("Usage: pi set <id|iq|vbus|vllc|illc> <Kp> <Ki>");
        return;
    }

    PI_Index_t idx = pi_parse_name(name);
    if (idx >= PI_ID_COUNT)
    {
        diag_print("Unknown PI name. Try: id, iq, vbus, vllc, illc");
        return;
    }

    if ((kp <= 0.0f) || (ki <= 0.0f))
    {
        diag_print("Kp and Ki must be > 0");
        return;
    }

    /* Preserve existing saturation limits */
    const PI_Controller_t *old = PI_GetController(idx);
    PI_SetGains(idx, kp, ki, old->out_min, old->out_max);

    const PI_Controller_t *pi = PI_GetController(idx);
    (void)snprintf(buf, sizeof(buf),
                   "%s: Kp=%.4f Ki=%.4f Tt=%.4f",
                   PI_GetName(idx),
                   (double)pi->Kp, (double)pi->Ki, (double)pi->Tt);
    diag_print(buf);
}

/**
 * @brief  'pi step <name>' — offline step-response test
 *
 * Runs 20 iterations of PI_Update with setpoint=1.0, measurement=0.0,
 * then prints output trajectory.  Verifies overshoot < 5% and
 * settling within 10 cycles.  Uses a temporary PI copy so the live
 * controller state is not disturbed.
 */
static void diag_pi_step(const char *args)
{
    char buf[80];
    char name[8];

    if (sscanf(args, "%7s", name) != 1)
    {
        diag_print("Usage: pi step <id|iq|vbus|vllc|illc>");
        return;
    }

    PI_Index_t idx = pi_parse_name(name);
    if (idx >= PI_ID_COUNT)
    {
        diag_print("Unknown PI name. Try: id, iq, vbus, vllc, illc");
        return;
    }

    /* Work on a copy so we don't disturb live controller */
    const PI_Controller_t *src = PI_GetController(idx);
    PI_Controller_t test = *src;
    test.integrator = 0.0f;
    test.prev_error = 0.0f;

    /*
     * Simulate step response: setpoint = out_max (full-scale step),
     * measurement starts at 0 and tracks output as a simple first-order
     * plant: measurement += (output - measurement) * alpha
     */
    float setpoint = test.out_max;
    float measurement = 0.0f;
    float dt = 1.0f / (float)PFC_ISR_FREQ_HZ; /* Use PFC rate as reference */
    float peak = 0.0f;
    uint32_t settled_at = 0U;
    uint8_t settled = 0U;

    (void)snprintf(buf, sizeof(buf), "Step test: %s  setpoint=%.2f  dt=%.2e",
                   PI_GetName(idx), (double)setpoint, (double)dt);
    diag_print(buf);

    for (uint32_t k = 0U; k < 20U; k++)
    {
        float output = PI_Update(&test, setpoint, measurement, dt);

        /* Simple first-order plant model: tau ~= 5*dt */
        measurement += (output - measurement) * 0.2f;

        if (measurement > peak)
        {
            peak = measurement;
        }

        /* Check settling: within 2% of setpoint */
        float err_pct = fabsf(measurement - setpoint) / fabsf(setpoint);
        if ((settled == 0U) && (err_pct < 0.02f))
        {
            settled_at = k;
            settled = 1U;
        }
        else if (err_pct >= 0.02f)
        {
            settled = 0U;
        }

        (void)snprintf(buf, sizeof(buf), "  [%2lu] out=%.4f  meas=%.4f",
                       (unsigned long)k, (double)output, (double)measurement);
        diag_print(buf);
    }

    /* Report overshoot and settling */
    float overshoot_pct = 0.0f;
    if (fabsf(setpoint) > 1e-6f)
    {
        overshoot_pct = ((peak - setpoint) / fabsf(setpoint)) * 100.0f;
    }
    if (overshoot_pct < 0.0f)
    {
        overshoot_pct = 0.0f;
    }

    (void)snprintf(buf, sizeof(buf), "Overshoot=%.1f%%  Settled=%s (cycle %lu)",
                   (double)overshoot_pct,
                   (settled != 0U) ? "YES" : "NO",
                   (unsigned long)settled_at);
    diag_print(buf);

    if ((overshoot_pct < 5.0f) && (settled != 0U) && (settled_at < 10U))
    {
        diag_print("PASS: overshoot < 5%, settling < 10 cycles");
    }
    else
    {
        diag_print("FAIL: criteria not met");
    }

    /* Reset the test copy (live controller untouched) */
}

static void diag_cmd_pi(const char *args)
{
    /* Skip leading whitespace */
    while (*args == ' ')
    {
        args++;
    }

    if ((strncmp(args, "show", 4U) == 0) || (*args == '\0'))
    {
        diag_pi_show();
    }
    else if (strncmp(args, "set ", 4U) == 0)
    {
        diag_pi_set(args + 4U);
    }
    else if (strncmp(args, "step ", 5U) == 0)
    {
        diag_pi_step(args + 5U);
    }
    else
    {
        diag_print("Usage: pi show | pi set <name> <Kp> <Ki> | pi step <name>");
    }
}

/* ------------------------------------------------------------------ */
/*  NP balance command                                                  */
/* ------------------------------------------------------------------ */

static void diag_cmd_np(void)
{
    char buf[64];
    (void)snprintf(buf, sizeof(buf), "V_NP_err=%.1f d_offset=%.3f",
                   (double)NP_Balance_GetError(),
                   (double)NP_Balance_GetOffset());
    diag_print(buf);
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
