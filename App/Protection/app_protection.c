/**
 * @file    app_protection.c
 * @brief   Fault state machine, HRTIM fault ISR, ring buffer, flash
 *          persistence, thermal derating
 */

#include "app_protection.h"
#include "app_adc.h"
#include "app_can.h"
#include "app_control.h"
#include "main.h"

#include <string.h>

/* ------------------------------------------------------------------ */
/*  Constants                                                          */
/* ------------------------------------------------------------------ */

#define FAULT_LOG_SIZE          256U
#define FAULT_RETRY_MAX         3U
#define FAULT_RETRY_COOLDOWN_MS 10000U

/* Flash fault log region — matches linker FAULTLOG origin */
#define FAULTLOG_FLASH_ADDR     0x0807F000U
#define FAULTLOG_FLASH_SIZE     0x1000U      /* 4 KB = 2 pages */
#define FLASH_PAGE_SIZE         0x800U       /* 2 KB per page (STM32G4) */

/* Flash magic for data integrity validation */
#define FAULTLOG_MAGIC          0xFA017L0GU
#define FAULTLOG_MAGIC_VAL      0xFA017A09U

/* Thermal derating thresholds (degC) */
#define DERATE_SIC_FULL         100.0f
#define DERATE_SIC_ZERO         115.0f
#define DERATE_MAG_FULL         120.0f
#define DERATE_MAG_ZERO         140.0f
#define DERATE_AMB_FULL         55.0f
#define DERATE_AMB_FLOOR        70.0f
#define DERATE_AMB_MIN_FRAC     0.5f
#define DERATE_HYSTERESIS       5.0f

/* Hysteresis bitmask positions */
#define DERATE_ZONE_SIC         (1U << 0)
#define DERATE_ZONE_MAG         (1U << 1)
#define DERATE_ZONE_AMB         (1U << 2)

/* ------------------------------------------------------------------ */
/*  Types                                                              */
/* ------------------------------------------------------------------ */

/* Fault log entry (12 bytes) */
typedef struct
{
    uint32_t timestamp_ms;
    uint8_t  source;
    uint8_t  severity;
    uint8_t  retry_count;
    uint8_t  reserved;
    uint32_t context;      /* v_bus×10 (upper 16) | i_out×10 (lower 16) */
} FaultLogEntry_t;

/* RAM shadow of flash fault log */
typedef struct
{
    uint32_t         magic;
    uint16_t         write_idx;
    uint16_t         count;
    FaultLogEntry_t  entries[FAULT_LOG_SIZE];
} FaultLogRam_t;

/* ------------------------------------------------------------------ */
/*  Module state                                                       */
/* ------------------------------------------------------------------ */

static FaultLogRam_t s_log_ram;
static uint8_t       s_retry_count[FAULT_COUNT];
static uint8_t       s_fault_active = 0;

static FaultStatus_t s_fault_status;
static uint32_t      s_fault_cooldown_start = 0;
static uint8_t       s_flash_dirty = 0;
static uint8_t       s_derate_active = 0;

volatile uint32_t g_fault_pending = 0U;

/* ------------------------------------------------------------------ */
/*  Severity lookup table                                              */
/* ------------------------------------------------------------------ */

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
    [FAULT_STARTUP_TIMEOUT] = FAULT_SEV_MAJOR,
    [FAULT_ZVS_LOSS]       = FAULT_SEV_WARNING,
};

/* ------------------------------------------------------------------ */
/*  Flash ring buffer helpers                                          */
/* ------------------------------------------------------------------ */

/**
 * @brief  Load fault log from flash into RAM shadow
 *
 * Reads the FAULTLOG flash region and validates magic. If valid, copies
 * into s_log_ram. Otherwise initialises a clean log.
 */
static void fault_log_load_from_flash(void)
{
    const FaultLogRam_t *flash_ptr =
        (const FaultLogRam_t *)FAULTLOG_FLASH_ADDR;

    if (flash_ptr->magic == FAULTLOG_MAGIC_VAL)
    {
        (void)memcpy(&s_log_ram, flash_ptr, sizeof(FaultLogRam_t));
    }
    else
    {
        (void)memset(&s_log_ram, 0, sizeof(FaultLogRam_t));
        s_log_ram.magic = FAULTLOG_MAGIC_VAL;
    }
}

/**
 * @brief  Flush RAM fault log to flash (deferred write)
 *
 * Erases 2 pages at the end of bank 2, then programs the entire
 * FaultLogRam_t struct as double-words. Takes ~25 ms due to flash
 * erase — MUST NOT be called from ISR context.
 */
void App_Protection_FlashSync(void)
{
    if (s_flash_dirty == 0U)
    {
        return;
    }

    HAL_FLASH_Unlock();

    /* Erase 2 pages at FAULTLOG region (bank 2 end) */
    FLASH_EraseInitTypeDef erase_cfg;
    uint32_t page_error = 0;

    erase_cfg.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_cfg.Banks     = FLASH_BANK_2;
    /* Page number within bank 2: (0x0807F000 - 0x08040000) / 0x800 = 254 */
    erase_cfg.Page      = (FAULTLOG_FLASH_ADDR - 0x08040000U) / FLASH_PAGE_SIZE;
    erase_cfg.NbPages   = 2U;

    if (HAL_FLASHEx_Erase(&erase_cfg, &page_error) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return;
    }

    /* Program as double-words (8 bytes at a time) */
    uint32_t dest_addr = FAULTLOG_FLASH_ADDR;
    const uint64_t *src = (const uint64_t *)&s_log_ram;
    uint32_t dwords = (sizeof(FaultLogRam_t) + 7U) / 8U;

    for (uint32_t i = 0; i < dwords; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                              dest_addr, src[i]) != HAL_OK)
        {
            break;
        }
        dest_addr += 8U;
    }

    HAL_FLASH_Lock();
    s_flash_dirty = 0;
}

/* ------------------------------------------------------------------ */
/*  Init                                                               */
/* ------------------------------------------------------------------ */

void App_Protection_Init(void)
{
    s_fault_active = 0;
    g_fault_pending = 0U;
    s_flash_dirty = 0;
    s_derate_active = 0;
    s_fault_cooldown_start = 0;

    s_fault_status.active_source   = FAULT_COUNT;
    s_fault_status.active_severity = FAULT_SEV_NONE;
    s_fault_status.latched         = 0;
    s_fault_status.retry_count     = 0;

    for (uint8_t i = 0; i < (uint8_t)FAULT_COUNT; i++)
    {
        s_retry_count[i] = 0;
    }

    fault_log_load_from_flash();

    /* HRTIM FLT1-FLT5 and GPIO configured in MX_HRTIM1_Init / MX_GPIO_Init */
}

/* ------------------------------------------------------------------ */
/*  HRTIM Fault ISR                                                    */
/* ------------------------------------------------------------------ */

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

/* ------------------------------------------------------------------ */
/*  Fault Enter                                                        */
/* ------------------------------------------------------------------ */

/**
 * @brief  Enter fault state — disable outputs, log, update status
 * @param  source  Fault source identifier
 */
void Fault_Enter(FaultSource_t source)
{
    if (source >= FAULT_COUNT)
    {
        return;
    }

    /* 1. Disable all HRTIM outputs immediately */
    App_Control_LLC_Stop();
    App_Control_PFC_Stop();

    /* 2. Look up severity */
    FaultSeverity_t sev = s_severity_table[source];

    /* 3. Capture ADC snapshot for context */
    const ADC_Readings_t *adc = App_ADC_GetReadings();
    uint16_t v_bus_x10 = (uint16_t)(adc->v_bus * 10.0f);
    uint16_t i_out_x10 = (uint16_t)(adc->i_out * 10.0f);
    uint32_t context = ((uint32_t)v_bus_x10 << 16) | (uint32_t)i_out_x10;

    /* 4. Log to RAM ring buffer */
    uint16_t idx = s_log_ram.write_idx % FAULT_LOG_SIZE;
    FaultLogEntry_t *entry = &s_log_ram.entries[idx];
    entry->timestamp_ms = HAL_GetTick();
    entry->source       = (uint8_t)source;
    entry->severity     = (uint8_t)sev;
    entry->retry_count  = s_retry_count[source];
    entry->context      = context;
    s_log_ram.write_idx++;
    if (s_log_ram.count < FAULT_LOG_SIZE)
    {
        s_log_ram.count++;
    }

    /* 5. Mark flash dirty for deferred write */
    s_flash_dirty = 1;

    /* 6. Update fault status */
    s_fault_status.active_source   = source;
    s_fault_status.active_severity = sev;

    /* 7. Severity-dependent latch / retry logic */
    if (sev == FAULT_SEV_CRITICAL)
    {
        s_fault_status.latched = 1;
    }
    else if (sev == FAULT_SEV_MAJOR)
    {
        s_retry_count[source]++;
        s_fault_status.retry_count = s_retry_count[source];
        if (s_retry_count[source] > FAULT_RETRY_MAX)
        {
            s_fault_status.latched = 1;
        }
    }
    else
    {
        /* WARNING — log only, don't set fault active */
        return;
    }

    /* 8. Set fault active and record cooldown start */
    s_fault_active = 1;
    s_fault_cooldown_start = HAL_GetTick();

    /* 9. Broadcast fault over CAN */
    App_CAN_BroadcastFault((uint8_t)source, (uint8_t)sev);
}

/* ------------------------------------------------------------------ */
/*  Fault Clear (hardware latch reset)                                 */
/* ------------------------------------------------------------------ */

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
}

/* ------------------------------------------------------------------ */
/*  Fault Recovery                                                     */
/* ------------------------------------------------------------------ */

/**
 * @brief  Check if a major fault can auto-recover (called at 1 kHz)
 *
 * - Latched faults: no action (wait for DiagClear)
 * - Major faults after 10 s cooldown: if retry_count <= 3, clear fault
 *   so state machine can restart. If > 3, latch permanently.
 */
void Fault_Recovery_Check(void)
{
    if (s_fault_status.latched != 0U)
    {
        return;
    }

    if (s_fault_status.active_severity != FAULT_SEV_MAJOR)
    {
        return;
    }

    uint32_t elapsed = HAL_GetTick() - s_fault_cooldown_start;

    if (elapsed < FAULT_RETRY_COOLDOWN_MS)
    {
        return;
    }

    /* Cooldown elapsed — check retry count */
    uint8_t src = (uint8_t)s_fault_status.active_source;
    if (s_retry_count[src] > FAULT_RETRY_MAX)
    {
        s_fault_status.latched = 1;
    }
    else
    {
        /* Clear fault — state machine will transition to PLL_LOCK */
        Fault_Clear();
        s_fault_active = 0;
    }
}

/* ------------------------------------------------------------------ */
/*  Diagnostic Clear                                                   */
/* ------------------------------------------------------------------ */

/**
 * @brief  Clear all faults and retry counts (called via CAN diagnostic)
 */
void App_Protection_DiagClear(void)
{
    Fault_Clear();
    s_fault_active = 0;
    s_fault_status.latched         = 0;
    s_fault_status.active_source   = FAULT_COUNT;
    s_fault_status.active_severity = FAULT_SEV_NONE;
    s_fault_status.retry_count     = 0;

    for (uint8_t i = 0; i < (uint8_t)FAULT_COUNT; i++)
    {
        s_retry_count[i] = 0;
    }
}

/* ------------------------------------------------------------------ */
/*  Accessors                                                          */
/* ------------------------------------------------------------------ */

uint8_t Fault_IsLatched(void)
{
    return s_fault_status.latched;
}

uint8_t App_Protection_IsFaultActive(void)
{
    return s_fault_active;
}

const FaultStatus_t *App_Protection_GetActiveFault(void)
{
    return &s_fault_status;
}

/* ------------------------------------------------------------------ */
/*  Thermal Derating                                                   */
/* ------------------------------------------------------------------ */

/**
 * @brief  Calculate power derating from all four temperature zones
 * @param  readings  Pointer to current ADC readings (all temperatures)
 * @return Power limit as fraction [0.0, 1.0]
 *
 * Three independent zones, return minimum:
 * - SiC:       100% at ≤100 C, linear to 0% at 115 C
 * - Magnetics: 100% at ≤120 C, linear to 0% at 140 C
 * - Ambient:   100% at ≤55 C,  linear to 50% floor at 70 C
 *
 * 5 C hysteresis on all zone entry/exit via s_derate_active bitmask.
 */
float Thermal_Derate_Calc(const ADC_Readings_t *readings)
{
    float sic_frac = 1.0f;
    float mag_frac = 1.0f;
    float amb_frac = 1.0f;

    /* --- SiC zone: use hotter of PFC and LLC --- */
    float t_sic = readings->t_sic_pfc;
    if (readings->t_sic_llc > t_sic)
    {
        t_sic = readings->t_sic_llc;
    }

    /* Hysteresis: enter derating at threshold, exit at threshold - 5 C */
    if ((s_derate_active & DERATE_ZONE_SIC) != 0U)
    {
        /* Already derating — exit when below (threshold - hysteresis) */
        if (t_sic <= (DERATE_SIC_FULL - DERATE_HYSTERESIS))
        {
            s_derate_active &= ~DERATE_ZONE_SIC;
        }
    }
    else
    {
        if (t_sic > DERATE_SIC_FULL)
        {
            s_derate_active |= DERATE_ZONE_SIC;
        }
    }

    if ((s_derate_active & DERATE_ZONE_SIC) != 0U)
    {
        if (t_sic >= DERATE_SIC_ZERO)
        {
            sic_frac = 0.0f;
        }
        else
        {
            sic_frac = (DERATE_SIC_ZERO - t_sic)
                      / (DERATE_SIC_ZERO - DERATE_SIC_FULL);
        }
    }

    /* --- Magnetics zone --- */
    float t_mag = readings->t_magnetics;

    if ((s_derate_active & DERATE_ZONE_MAG) != 0U)
    {
        if (t_mag <= (DERATE_MAG_FULL - DERATE_HYSTERESIS))
        {
            s_derate_active &= ~DERATE_ZONE_MAG;
        }
    }
    else
    {
        if (t_mag > DERATE_MAG_FULL)
        {
            s_derate_active |= DERATE_ZONE_MAG;
        }
    }

    if ((s_derate_active & DERATE_ZONE_MAG) != 0U)
    {
        if (t_mag >= DERATE_MAG_ZERO)
        {
            mag_frac = 0.0f;
        }
        else
        {
            mag_frac = (DERATE_MAG_ZERO - t_mag)
                      / (DERATE_MAG_ZERO - DERATE_MAG_FULL);
        }
    }

    /* --- Ambient zone --- */
    float t_amb = readings->t_ambient;

    if ((s_derate_active & DERATE_ZONE_AMB) != 0U)
    {
        if (t_amb <= (DERATE_AMB_FULL - DERATE_HYSTERESIS))
        {
            s_derate_active &= ~DERATE_ZONE_AMB;
        }
    }
    else
    {
        if (t_amb > DERATE_AMB_FULL)
        {
            s_derate_active |= DERATE_ZONE_AMB;
        }
    }

    if ((s_derate_active & DERATE_ZONE_AMB) != 0U)
    {
        if (t_amb >= DERATE_AMB_FLOOR)
        {
            amb_frac = DERATE_AMB_MIN_FRAC;
        }
        else
        {
            /* Linear from 1.0 at 55 C to 0.5 at 70 C */
            amb_frac = 1.0f - ((1.0f - DERATE_AMB_MIN_FRAC)
                      * (t_amb - DERATE_AMB_FULL)
                      / (DERATE_AMB_FLOOR - DERATE_AMB_FULL));
        }
    }

    /* Return minimum of all three zones (no fminf for MISRA) */
    float result = sic_frac;
    if (mag_frac < result)
    {
        result = mag_frac;
    }
    if (amb_frac < result)
    {
        result = amb_frac;
    }

    return result;
}
