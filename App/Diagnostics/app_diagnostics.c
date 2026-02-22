/**
 * @file    app_diagnostics.c
 * @brief   UART CLI — PFC open-loop test, PI tuning, status, fault log
 *
 * Commands:
 *   pfc start              — Start PFC outputs at current duty (open-loop)
 *   pfc stop               — Stop PFC outputs
 *   pfc duty <0-95>        — Set PFC duty cycle (percent)
 *   pfc status             — Print PFC switching parameters and bus voltage
 *   pfc idref <amps>       — Set PFC I_d* reference for closed-loop testing
 *   pi show                — Print all PI controller gains and Tt
 *   pi set <name> <Kp> <Ki> — Set PI gains (Tt auto-computed)
 *   pi step <name>         — Run offline step-response test (20 iterations)
 *   gain <loop> <kp> <ki>  — Alias for 'pi set'
 *   np                     — Print neutral point error and duty offset
 *   enable                 — Request state machine enable (STANDBY -> PLL_LOCK)
 *   status                 — Print state, Vbus, Iout, Vout, temperatures
 *   fault                  — Dump fault log ring buffer
 *   version                — Print firmware version
 *
 * UART transmit uses DMA to avoid blocking ISR execution.
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
/*  UART DMA TX Constants                                              */
/* ------------------------------------------------------------------ */

/** @brief  DMA TX ring buffer size (must be power of 2) */
#define DIAG_TX_BUF_SIZE         1024U

/** @brief  Rx line buffer size (max command length) */
#define DIAG_RX_BUF_SIZE         64U

/** @brief  UART transmit timeout for echo (blocking, single byte) */
#define DIAG_ECHO_TIMEOUT_MS     5U

/** @brief  PFC open-loop test running flag */
static uint8_t s_pfc_test_active;

/* ------------------------------------------------------------------ */
/*  DMA TX ring buffer state                                           */
/* ------------------------------------------------------------------ */

static uint8_t  s_tx_buf[DIAG_TX_BUF_SIZE];
static volatile uint16_t s_tx_head;   /* Next write position (main context) */
static volatile uint16_t s_tx_tail;   /* Next DMA read position */
static volatile uint8_t  s_tx_busy;   /* 1 = DMA transfer in progress */

DMA_HandleTypeDef hdma_usart2_tx;

/* ------------------------------------------------------------------ */
/*  UART Rx state                                                       */
/* ------------------------------------------------------------------ */

static uint8_t  s_rx_buf[DIAG_RX_BUF_SIZE];
static uint16_t s_rx_idx;
static uint8_t  s_rx_byte;

/* ------------------------------------------------------------------ */
/*  Fault source name table                                            */
/* ------------------------------------------------------------------ */

static const char *const s_fault_names[] = {
    "PFC_OCP_HW",    "LLC_OCP_HW",    "BUS_OVP_HW",    "OUT_OVP_HW",
    "GROUND_FAULT",  "PFC_OCP_SW",    "LLC_OCP_SW",     "BUS_OVP_SW",
    "BUS_UVP_SW",    "OUT_OVP_SW",    "OUT_OCP_SW",     "OTP_SIC_PFC",
    "OTP_SIC_LLC",   "OTP_MAG",       "OTP_AMB",        "PHASE_LOSS",
    "PLL_UNLOCK",    "CAN_TIMEOUT",   "FAN_FAIL",       "NP_IMBAL",
    "SHORT_CKT",     "INRUSH_TMO",    "WATCHDOG",       "FLASH_ERR",
    "STARTUP_TMO",   "ZVS_LOSS"
};

static const char *const s_sev_names[] = {
    "NONE", "WARN", "MAJOR", "CRIT"
};

/* ------------------------------------------------------------------ */
/*  Forward declarations                                                */
/* ------------------------------------------------------------------ */

static void diag_process_command(char *cmd);
static void diag_cmd_pfc(const char *args);
static void diag_cmd_pi(const char *args);
static void diag_cmd_gain(const char *args);
static void diag_cmd_np(void);
static void diag_cmd_status(void);
static void diag_cmd_fault(void);
static void diag_cmd_version(void);
static void diag_print(const char *msg);
static void diag_tx_enqueue(const uint8_t *data, uint16_t len);
static void diag_tx_kick(void);

/* ------------------------------------------------------------------ */
/*  DMA TX setup                                                        */
/* ------------------------------------------------------------------ */

/**
 * @brief  Configure DMA1_Channel3 for USART2 TX
 *
 * Uses DMAMUX to route USART2_TX request to DMA1_Channel3.
 * Priority is low (diagnostic output, not time-critical).
 */
static void diag_dma_tx_init(void)
{
    hdma_usart2_tx.Instance                 = DMA1_Channel3;
    hdma_usart2_tx.Init.Request             = DMA_REQUEST_USART2_TX;
    hdma_usart2_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode                = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority            = DMA_PRIORITY_LOW;

    (void)HAL_DMA_Init(&hdma_usart2_tx);

    /* Link DMA handle to UART handle */
    __HAL_LINKDMA(&huart2, hdmatx, hdma_usart2_tx);

    /* Enable DMA1_Channel3 interrupt */
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, NVIC_PRIO_DMA, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

/* ------------------------------------------------------------------ */
/*  Public API                                                          */
/* ------------------------------------------------------------------ */

void App_Diagnostics_Init(void)
{
    s_rx_idx = 0U;
    s_pfc_test_active = 0U;
    s_tx_head = 0U;
    s_tx_tail = 0U;
    s_tx_busy = 0U;

    /* Configure DMA for USART2 TX */
    diag_dma_tx_init();

    /* Start single-byte UART Rx interrupt */
    HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);

    diag_print("PDU CLI ready");
}

void App_Diagnostics_Poll(void)
{
    /* Kick DMA TX if data pending and DMA idle */
    if ((s_tx_busy == 0U) && (s_tx_head != s_tx_tail))
    {
        diag_tx_kick();
    }
}

/**
 * @brief  DMA UART TX complete callback — called from DMA ISR
 *
 * Advances the tail pointer past the completed transfer.
 * If more data is pending in the ring buffer, starts the next
 * DMA transfer immediately.
 */
void App_Diagnostics_TxCpltCallback(void)
{
    s_tx_busy = 0U;

    /* If more data queued, kick the next transfer */
    if (s_tx_head != s_tx_tail)
    {
        diag_tx_kick();
    }
}

/**
 * @brief  Non-blocking UART log — enqueues message + CRLF into DMA TX buffer
 * @param  msg  Null-terminated string to transmit
 */
void App_Diagnostics_Log(const char *msg)
{
    if (msg == NULL)
    {
        return;
    }

    uint16_t len = (uint16_t)strlen(msg);

    if (len > 0U)
    {
        diag_tx_enqueue((const uint8_t *)msg, len);
    }
    diag_tx_enqueue((const uint8_t *)"\r\n", 2U);

    /* Kick DMA if idle */
    if (s_tx_busy == 0U)
    {
        diag_tx_kick();
    }
}

/* ------------------------------------------------------------------ */
/*  DMA TX ring buffer helpers                                         */
/* ------------------------------------------------------------------ */

/**
 * @brief  Enqueue bytes into the TX ring buffer (does not start DMA)
 * @param  data  Pointer to bytes
 * @param  len   Number of bytes
 *
 * Drops data silently if the ring buffer is full. This is acceptable
 * for diagnostic output — we never block the caller.
 */
static void diag_tx_enqueue(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0U; i < len; i++)
    {
        uint16_t next = (s_tx_head + 1U) % DIAG_TX_BUF_SIZE;
        if (next == s_tx_tail)
        {
            break;  /* Buffer full — drop remaining */
        }
        s_tx_buf[s_tx_head] = data[i];
        s_tx_head = next;
    }
}

/**
 * @brief  Start a DMA transfer from the ring buffer
 *
 * Transfers a contiguous block from tail to either head or end-of-buffer
 * (whichever comes first). The DMA complete callback will advance the tail
 * and kick the next segment if data wraps around.
 */
static void diag_tx_kick(void)
{
    if (s_tx_head == s_tx_tail)
    {
        return;
    }

    uint16_t len;

    if (s_tx_head > s_tx_tail)
    {
        len = s_tx_head - s_tx_tail;
    }
    else
    {
        /* Transfer up to end of buffer; next callback handles wrap */
        len = DIAG_TX_BUF_SIZE - s_tx_tail;
    }

    s_tx_busy = 1U;

    if (HAL_UART_Transmit_DMA(&huart2, &s_tx_buf[s_tx_tail], len) == HAL_OK)
    {
        s_tx_tail = (s_tx_tail + len) % DIAG_TX_BUF_SIZE;
    }
    else
    {
        s_tx_busy = 0U;  /* DMA start failed — retry on next poll */
    }
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

    /* Echo character (single byte, short timeout acceptable in ISR) */
    (void)HAL_UART_Transmit(&huart2, &ch, 1U, DIAG_ECHO_TIMEOUT_MS);

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
    else if (strncmp(cmd, "gain ", 5U) == 0)
    {
        diag_cmd_gain(cmd + 5U);
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
        diag_print("Unknown command. Try: pfc, pi, gain, np, enable, status, fault, version");
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
    else if (strncmp(args, "idref ", 6U) == 0)
    {
        float id_ref = (float)atof(args + 6U);

        if ((id_ref < -PFC_OCP_THRESHOLD_A) || (id_ref > PFC_OCP_THRESHOLD_A))
        {
            (void)snprintf(buf, sizeof(buf),
                           "I_d* out of range (max %.0f A)",
                           (double)PFC_OCP_THRESHOLD_A);
            diag_print(buf);
            return;
        }
        App_Control_PFC_SetIdRef(id_ref);
        (void)snprintf(buf, sizeof(buf), "PFC I_d* set to %.2f A",
                       (double)id_ref);
        diag_print(buf);
    }
    else
    {
        diag_print("Usage: pfc start|stop|duty <0-95>|idref <amps>|status");
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
/*  Gain command (alias for 'pi set')                                   */
/* ------------------------------------------------------------------ */

/**
 * @brief  'gain <loop> <kp> <ki>' — adjust PI gains at runtime
 *
 * Alias for 'pi set <loop> <kp> <ki>'. Accepts the same loop names:
 * id, iq, vbus, vllc, illc.
 */
static void diag_cmd_gain(const char *args)
{
    diag_pi_set(args);
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

/**
 * @brief  'fault' — dump fault log ring buffer
 *
 * Prints the active fault status (if any) followed by all entries in the
 * fault log ring buffer, oldest first. Each entry shows timestamp, source
 * name, severity, retry count, and context (Vbus, Iout at time of fault).
 */
static void diag_cmd_fault(void)
{
    char buf[96];

    /* Print active fault status */
    if (App_Protection_IsFaultActive() != 0U)
    {
        const FaultStatus_t *fs = App_Protection_GetActiveFault();
        const char *src_name = "UNKNOWN";
        if ((uint8_t)fs->active_source < (uint8_t)FAULT_COUNT)
        {
            src_name = s_fault_names[fs->active_source];
        }
        (void)snprintf(buf, sizeof(buf), "ACTIVE: %s sev=%s latched=%u retries=%u",
                       src_name,
                       s_sev_names[fs->active_severity],
                       (unsigned)fs->latched, (unsigned)fs->retry_count);
        diag_print(buf);
    }
    else
    {
        diag_print("No active fault");
    }

    /* Dump ring buffer */
    uint16_t count = App_Protection_GetLogCount();

    if (count == 0U)
    {
        diag_print("Fault log empty");
        return;
    }

    (void)snprintf(buf, sizeof(buf), "Fault log (%u entries):", (unsigned)count);
    diag_print(buf);

    for (uint16_t i = 0U; i < count; i++)
    {
        const FaultLogEntry_t *entry = App_Protection_GetLogEntry(i);
        if (entry == NULL)
        {
            break;
        }

        const char *src_name = "UNK";
        if (entry->source < (uint8_t)FAULT_COUNT)
        {
            src_name = s_fault_names[entry->source];
        }

        const char *sev_name = "?";
        if (entry->severity < 4U)
        {
            sev_name = s_sev_names[entry->severity];
        }

        uint16_t v_bus_x10 = (uint16_t)(entry->context >> 16);
        uint16_t i_out_x10 = (uint16_t)(entry->context & 0xFFFFU);

        (void)snprintf(buf, sizeof(buf),
                       "  [%3u] %lu ms  %-14s %-5s  retry=%u  Vbus=%.1f Iout=%.1f",
                       (unsigned)i,
                       (unsigned long)entry->timestamp_ms,
                       src_name, sev_name,
                       (unsigned)entry->retry_count,
                       (double)v_bus_x10 / 10.0,
                       (double)i_out_x10 / 10.0);
        diag_print(buf);
    }
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
