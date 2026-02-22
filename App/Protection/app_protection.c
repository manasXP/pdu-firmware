/**
 * @file    app_protection.c
 * @brief   Fault state machine, HRTIM fault ISR, ring buffer, thermal derating
 */

#include "app_protection.h"
#include "main.h"

#define FAULT_LOG_SIZE     256
#define FAULT_RETRY_MAX    3
#define FAULT_RETRY_COOLDOWN_MS 10000

/* Fault log entry (12 bytes) */
typedef struct
{
    uint32_t timestamp_ms;
    uint8_t  source;
    uint8_t  severity;
    uint8_t  retry_count;
    uint8_t  reserved;
    uint32_t context;      /* ADC reading or register snapshot */
} FaultLogEntry_t;

static FaultLogEntry_t s_fault_log[FAULT_LOG_SIZE];
static uint16_t        s_fault_log_head = 0;
static uint8_t         s_retry_count[FAULT_COUNT];
static uint8_t         s_fault_active = 0;

volatile uint32_t g_fault_pending = 0U;

/* Severity lookup table */
static const FaultSeverity_t s_severity_table[FAULT_COUNT] = {
    [FAULT_PFC_OCP_HW]     = FAULT_SEV_CRITICAL,
    [FAULT_LLC_OCP_HW]     = FAULT_SEV_CRITICAL,
    [FAULT_BUS_OVP_HW]     = FAULT_SEV_CRITICAL,
    [FAULT_OUT_OVP_HW]     = FAULT_SEV_CRITICAL,
    [FAULT_GROUND_FAULT]   = FAULT_SEV_CRITICAL,
    [FAULT_PFC_OCP_SW]     = FAULT_SEV_MAJOR,
    [FAULT_LLC_OCP_SW]     = FAULT_SEV_MAJOR,
    [FAULT_BUS_OVP_SW]     = FAULT_SEV_MAJOR,
    [FAULT_BUS_UVP_SW]     = FAULT_SEV_MAJOR,
    [FAULT_OUT_OVP_SW]     = FAULT_SEV_MAJOR,
    [FAULT_OUT_OCP_SW]     = FAULT_SEV_MAJOR,
    [FAULT_OTP_SIC_PFC]    = FAULT_SEV_MAJOR,
    [FAULT_OTP_SIC_LLC]    = FAULT_SEV_MAJOR,
    [FAULT_OTP_MAGNETICS]  = FAULT_SEV_MAJOR,
    [FAULT_OTP_AMBIENT]    = FAULT_SEV_MAJOR,
    [FAULT_PHASE_LOSS]     = FAULT_SEV_MAJOR,
    [FAULT_PLL_UNLOCK]     = FAULT_SEV_MAJOR,
    [FAULT_CAN_TIMEOUT]    = FAULT_SEV_MAJOR,
    [FAULT_FAN_FAILURE]    = FAULT_SEV_MAJOR,
    [FAULT_NP_IMBALANCE]   = FAULT_SEV_WARNING,
    [FAULT_SHORT_CIRCUIT]  = FAULT_SEV_CRITICAL,
    [FAULT_INRUSH_TIMEOUT] = FAULT_SEV_MAJOR,
    [FAULT_WATCHDOG]       = FAULT_SEV_CRITICAL,
    [FAULT_FLASH_ERROR]       = FAULT_SEV_WARNING,
    [FAULT_STARTUP_TIMEOUT]   = FAULT_SEV_MAJOR,
    [FAULT_ZVS_LOSS]          = FAULT_SEV_WARNING,
};

void App_Protection_Init(void)
{
    s_fault_log_head = 0;
    s_fault_active = 0;
    g_fault_pending = 0U;
    for (uint8_t i = 0; i < FAULT_COUNT; i++)
    {
        s_retry_count[i] = 0;
    }
    /* HRTIM FLT1-FLT5 and GPIO configured in MX_HRTIM1_Init / MX_GPIO_Init */
}

/**
 * @brief  HRTIM fault ISR — called from HRTIM1_FLT_IRQHandler
 *
 * Reads HRTIM common ISR register to identify which fault input(s) fired,
 * sets corresponding bits in g_fault_pending, clears the interrupt flags,
 * and logs each fault source.
 *
 * Runs at NVIC priority 0 (highest). Keep this fast — no HAL calls.
 */
void App_Protection_FaultISR(void)
{
    uint32_t isr = HRTIM1->sCommonRegs.ISR;

    if (isr & HRTIM_ISR_FLT1)
    {
        HRTIM1->sCommonRegs.ICR = HRTIM_ICR_FLT1C;
        g_fault_pending |= (1U << 0);
        Fault_Enter(FAULT_PFC_OCP_HW);
    }
    if (isr & HRTIM_ISR_FLT2)
    {
        HRTIM1->sCommonRegs.ICR = HRTIM_ICR_FLT2C;
        g_fault_pending |= (1U << 1);
        Fault_Enter(FAULT_LLC_OCP_HW);
    }
    if (isr & HRTIM_ISR_FLT3)
    {
        HRTIM1->sCommonRegs.ICR = HRTIM_ICR_FLT3C;
        g_fault_pending |= (1U << 2);
        Fault_Enter(FAULT_BUS_OVP_HW);
    }
    if (isr & HRTIM_ISR_FLT4)
    {
        HRTIM1->sCommonRegs.ICR = HRTIM_ICR_FLT4C;
        g_fault_pending |= (1U << 3);
        Fault_Enter(FAULT_OUT_OVP_HW);
    }
    if (isr & HRTIM_ISR_FLT5)
    {
        HRTIM1->sCommonRegs.ICR = HRTIM_ICR_FLT5C;
        g_fault_pending |= (1U << 4);
        Fault_Enter(FAULT_GROUND_FAULT);
    }
}

void Fault_Enter(FaultSource_t source)
{
    if (source >= FAULT_COUNT)
    {
        return;
    }

    /* Log to ring buffer */
    FaultLogEntry_t *entry = &s_fault_log[s_fault_log_head % FAULT_LOG_SIZE];
    entry->source       = (uint8_t)source;
    entry->severity     = (uint8_t)s_severity_table[source];
    entry->retry_count  = s_retry_count[source];
    entry->timestamp_ms = HAL_GetTick();
    entry->context      = HRTIM1->sCommonRegs.ISR;
    s_fault_log_head++;

    s_fault_active = 1;
}

/**
 * @brief  Clear fault latch — re-enable HRTIM outputs
 *
 * Disables then re-enables all 5 fault inputs to clear the hardware
 * latch. Outputs will not resume until the control module explicitly
 * restarts them.
 */
void Fault_Clear(void)
{
    /* Clear the hardware fault latch by toggling fault enable */
    HAL_HRTIM_FaultModeCtl(&hhrtim1,
                           HRTIM_FAULT_1 | HRTIM_FAULT_2
                         | HRTIM_FAULT_3 | HRTIM_FAULT_4
                         | HRTIM_FAULT_5,
                           HRTIM_FAULTMODECTL_DISABLED);

    HAL_HRTIM_FaultModeCtl(&hhrtim1,
                           HRTIM_FAULT_1 | HRTIM_FAULT_2
                         | HRTIM_FAULT_3 | HRTIM_FAULT_4
                         | HRTIM_FAULT_5,
                           HRTIM_FAULTMODECTL_ENABLED);

    g_fault_pending = 0U;
    s_fault_active = 0;
}

/**
 * @brief  Calculate power derating based on hottest component temperature
 * @param  t_hottest_deg_c  Temperature of hottest component
 * @return Power limit as fraction [0.0, 1.0]
 *         1.0 = full power (T <= 45°C)
 *         0.0 = shutdown (T >= 65°C)
 *         Linear derating between 45°C and 65°C
 */
float Thermal_Derate_Calc(float t_hottest_deg_c)
{
    const float T_FULL  = 45.0f;
    const float T_ZERO  = 65.0f;

    if (t_hottest_deg_c <= T_FULL)
        return 1.0f;
    if (t_hottest_deg_c >= T_ZERO)
        return 0.0f;

    return (T_ZERO - t_hottest_deg_c) / (T_ZERO - T_FULL);
}
