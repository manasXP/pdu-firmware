/**
 * @file    app_protection.h
 * @brief   Fault state machine, HRTIM fault inputs, thermal derating
 */

#ifndef APP_PROTECTION_H
#define APP_PROTECTION_H

#include <stdint.h>

/* Fault severity */
typedef enum
{
    FAULT_SEV_NONE = 0,
    FAULT_SEV_WARNING,     /* Log only, no action */
    FAULT_SEV_MAJOR,       /* Auto-retry up to 3 times, then latch */
    FAULT_SEV_CRITICAL     /* Immediate latch, no auto-recovery */
} FaultSeverity_t;

/* Fault source IDs (24 sources) */
typedef enum
{
    FAULT_PFC_OCP_HW = 0,
    FAULT_LLC_OCP_HW,
    FAULT_BUS_OVP_HW,
    FAULT_OUT_OVP_HW,
    FAULT_GROUND_FAULT,
    FAULT_PFC_OCP_SW,
    FAULT_LLC_OCP_SW,
    FAULT_BUS_OVP_SW,
    FAULT_BUS_UVP_SW,
    FAULT_OUT_OVP_SW,
    FAULT_OUT_OCP_SW,
    FAULT_OTP_SIC_PFC,
    FAULT_OTP_SIC_LLC,
    FAULT_OTP_MAGNETICS,
    FAULT_OTP_AMBIENT,
    FAULT_PHASE_LOSS,
    FAULT_PLL_UNLOCK,
    FAULT_CAN_TIMEOUT,
    FAULT_FAN_FAILURE,
    FAULT_NP_IMBALANCE,
    FAULT_SHORT_CIRCUIT,
    FAULT_INRUSH_TIMEOUT,
    FAULT_WATCHDOG,
    FAULT_FLASH_ERROR,
    FAULT_STARTUP_TIMEOUT,
    FAULT_ZVS_LOSS,
    FAULT_COUNT
} FaultSource_t;

void  App_Protection_Init(void);
void  Fault_Enter(FaultSource_t source);
void  Fault_Clear(void);
float Thermal_Derate_Calc(float t_hottest_deg_c);

#endif /* APP_PROTECTION_H */
