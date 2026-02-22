/**
 * @file    app_protection.c
 * @brief   Fault state machine, flash ring buffer, retry logic, thermal derating
 */

#include "app_protection.h"

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
    [FAULT_FLASH_ERROR]    = FAULT_SEV_WARNING,
};

void App_Protection_Init(void)
{
    s_fault_log_head = 0;
    s_fault_active = 0;
    for (uint8_t i = 0; i < FAULT_COUNT; i++)
    {
        s_retry_count[i] = 0;
    }
    /* TODO: Configure HRTIM FLT1-FLT5 and analog comparator thresholds */
}

void Fault_Enter(FaultSource_t source)
{
    if (source >= FAULT_COUNT)
        return;

    /* TODO: Disable all HRTIM outputs */

    /* Log to ring buffer */
    FaultLogEntry_t *entry = &s_fault_log[s_fault_log_head % FAULT_LOG_SIZE];
    entry->source      = (uint8_t)source;
    entry->severity    = (uint8_t)s_severity_table[source];
    entry->retry_count = s_retry_count[source];
    entry->timestamp_ms = 0; /* TODO: get system tick */
    entry->context     = 0;  /* TODO: capture relevant ADC */
    s_fault_log_head++;

    s_fault_active = 1;
}

void Fault_Clear(void)
{
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
