/**
 * @file    app_protection.h
 * @brief   Fault state machine, HRTIM fault inputs, thermal derating
 */

#ifndef APP_PROTECTION_H
#define APP_PROTECTION_H

#include <stdint.h>
#include "app_adc.h"

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

/* Fault status — tracks active fault state */
typedef struct
{
    FaultSource_t   active_source;   /* Currently active fault (FAULT_COUNT = none) */
    FaultSeverity_t active_severity; /* Severity of active fault */
    uint8_t         latched;         /* 1 = latched (no auto-recovery) */
    uint8_t         retry_count;     /* Retry attempts for active source */
} FaultStatus_t;

/**
 * @brief  Bitmask of pending hardware faults (set in HRTIM fault ISR).
 *
 * Bit positions match HRTIM_FAULT_x:
 *   bit 0 = FLT1 (PFC OCP), bit 1 = FLT2 (LLC OCP),
 *   bit 2 = FLT3 (bus OVP), bit 3 = FLT4 (output OVP),
 *   bit 4 = FLT5 (ground fault).
 */
extern volatile uint32_t g_fault_pending;

void  App_Protection_Init(void);
void  App_Protection_FaultISR(void);
void  Fault_Enter(FaultSource_t source);
void  Fault_Clear(void);
void  Fault_Recovery_Check(void);
float Thermal_Derate_Calc(const ADC_Readings_t *readings);

/* Fault status accessors */
uint8_t              Fault_IsLatched(void);
uint8_t              App_Protection_IsFaultActive(void);
const FaultStatus_t *App_Protection_GetActiveFault(void);

/* Flash persistence — call from main loop (NOT ISR) */
void App_Protection_FlashSync(void);

/* Diagnostic clear — resets latched faults and retry counts */
void App_Protection_DiagClear(void);

#endif /* APP_PROTECTION_H */
