/**
 * @file    main.h
 * @brief   Main program header — 30 kW PDU Firmware
 * @target  STM32G474RET6
 */

#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

/* ------------------------------------------------------------------ */
/*  Firmware Version                                                   */
/* ------------------------------------------------------------------ */

#define FW_VERSION_MAJOR  0
#define FW_VERSION_MINOR  1
#define FW_VERSION_BUILD  0

/* ------------------------------------------------------------------ */
/*  System Clock                                                       */
/* ------------------------------------------------------------------ */

#define SYSCLK_FREQ_HZ    170000000U
#define HRTIM_DLL_FREQ_HZ 5440000000ULL

/* ------------------------------------------------------------------ */
/*  Control Loop Rates                                                 */
/* ------------------------------------------------------------------ */

#define PFC_ISR_FREQ_HZ    65000U
#define LLC_ISR_FREQ_HZ    150000U
#define MAIN_LOOP_FREQ_HZ  1000U
#define CAN_STATUS_PERIOD_MS 10U

/* ------------------------------------------------------------------ */
/*  ADC                                                                */
/* ------------------------------------------------------------------ */

#define ADC_OVERSAMPLING_RATIO 16U
#define ADC_VREF_MV            3300U     /* VREF+ millivolts            */
#define ADC_FULL_SCALE         4095U     /* 12-bit ADC max code         */

/*
 * Per-signal scaling factors (placeholder — finalize at HW bring-up).
 * Each converts a 12-bit ADC code to engineering units (V / A / degC).
 * Formula: physical = (raw * ADC_VREF_MV / ADC_FULL_SCALE) * GAIN
 *
 * For bidirectional current sensors the zero-current offset is at
 * mid-scale (2048 counts); subtract before applying gain.
 */
#define ADC_LSB_MV             (3300.0f / 4095.0f)  /* ~0.8059 mV/LSB */

/* Voltage divider gains: physical_V = adc_V * (R_top + R_bot) / R_bot */
#define ADC_GAIN_VBUS          326.7f    /* DC bus  ≤ 966 V             */
#define ADC_GAIN_VGRID         220.0f    /* Grid    ≤ 530 Vpk           */
#define ADC_GAIN_VOUT          340.0f    /* Output  ≤ 1050 V            */
#define ADC_GAIN_VCAP          165.0f    /* Split-cap ≤ 500 V           */
#define ADC_GAIN_VAUX12V       4.0f     /* 12 V aux rail                */

/* Current sensor gains: physical_A = (adc - mid) * LSB * GAIN */
#define ADC_GAIN_IPHASE        50.0f    /* PFC phase current (A/V)      */
#define ADC_GAIN_IOUT          30.0f    /* LLC output current (A/V)     */

/* Convenience macros — raw 12-bit code to physical unit */
#define ADC_TO_VBUS(raw)   ((float)(raw) * ADC_LSB_MV * 0.001f * ADC_GAIN_VBUS)
#define ADC_TO_VGRID(raw)  ((float)(raw) * ADC_LSB_MV * 0.001f * ADC_GAIN_VGRID)
#define ADC_TO_VOUT(raw)   ((float)(raw) * ADC_LSB_MV * 0.001f * ADC_GAIN_VOUT)
#define ADC_TO_VCAP(raw)   ((float)(raw) * ADC_LSB_MV * 0.001f * ADC_GAIN_VCAP)
#define ADC_TO_VAUX12V(raw) ((float)(raw) * ADC_LSB_MV * 0.001f * ADC_GAIN_VAUX12V)
#define ADC_TO_IPHASE(raw) (((float)(raw) - 2048.0f) * ADC_LSB_MV * 0.001f * ADC_GAIN_IPHASE)
#define ADC_TO_IOUT(raw)   (((float)(raw) - 2048.0f) * ADC_LSB_MV * 0.001f * ADC_GAIN_IOUT)

/* ------------------------------------------------------------------ */
/*  Protection Thresholds                                              */
/* ------------------------------------------------------------------ */

#define PFC_OCP_THRESHOLD_A    78.0f
#define BUS_OVP_THRESHOLD_V    966.0f
#define OUT_OVP_THRESHOLD_V    1050.0f
#define OTP_DERATE_DEG_C       100.0f
#define OTP_SHUTDOWN_DEG_C     115.0f

/* ------------------------------------------------------------------ */
/*  Startup Timeouts                                                   */
/* ------------------------------------------------------------------ */

#define STARTUP_TIMEOUT_MS     6000U
#define PLL_LOCK_TIMEOUT_MS    2000U
#define INIT_TIMEOUT_DLL_MS    50U

/* ------------------------------------------------------------------ */
/*  PFC Soft-Start Parameters                                          */
/* ------------------------------------------------------------------ */

#define PFC_SOFTSTART_RAMP_MS  200U    /* I_d* ramp duration (ms)          */
#define PFC_SOFTSTART_ID_MAX   40.0f   /* Rated I_d* setpoint (A)          */
#define PFC_SOFTSTART_DUTY_INIT 0.05f  /* Initial duty on ramp start       */
#define PFC_NTC_BYPASS_VBUS_FRAC 0.80f /* NTC bypass at 80% of target Vbus */
#define PFC_TARGET_VBUS_V      800.0f  /* Target DC bus voltage (V)        */

/* ------------------------------------------------------------------ */
/*  Shutdown Parameters                                                */
/* ------------------------------------------------------------------ */

#define SHUTDOWN_IOUT_THRESHOLD_A 0.5f /* Open contactor below this (A)   */
#define SHUTDOWN_DUTY_RAMP_MS     100U /* PFC duty ramp-down duration (ms) */
#define SHUTDOWN_MAX_WAIT_MS      3000U /* Max wait for I_out to decay (ms)*/

/* ------------------------------------------------------------------ */
/*  LLC Frequency Range                                                */
/* ------------------------------------------------------------------ */

#define LLC_FREQ_MIN_HZ        100000U   /* 100 kHz — below resonance  */
#define LLC_FREQ_MAX_HZ        300000U   /* 300 kHz — safe startup     */
#define LLC_FREQ_RESONANT_HZ   150000U   /* ~150 kHz expected fr       */
#define LLC_DEADTIME_TICKS     272U      /* 200 ns @ MUL8 (1.36 GHz)  */
#define LLC_TURNS_RATIO        2.0f      /* Transformer turns ratio n  */
#define LLC_FREQ_FR2_HZ        139400U   /* Parallel resonant freq (ZVS boundary) */
#define LLC_SOFTSTART_TARGET_V 400.0f   /* Default output target (V) */
#define LLC_SOFTSTART_TOLERANCE 0.05f   /* 5% window for contactor close */

/* ------------------------------------------------------------------ */
/*  Burst Mode Parameters                                              */
/* ------------------------------------------------------------------ */

#define BURST_ENTRY_FREQ_HZ     280000U   /* Enter burst when f_sw > 280 kHz for 50 ms */
#define BURST_EXIT_FREQ_HZ      270000U   /* Exit burst when f_sw < 270 kHz (10 kHz hyst) */
#define BURST_ENTRY_TIME_MS     50U       /* Sustained high-freq time before entry       */
#define BURST_VOUT_UPPER_V      2.0f      /* RUN→IDLE: V_out > V_target + 2 V            */
#define BURST_VOUT_LOWER_V      5.0f      /* IDLE→RUN: V_out < V_target − 5 V            */
#define BURST_VOUT_EMERGENCY_V  10.0f     /* Immediate burst exit: V_out < V_target − 10 V*/
#define BURST_DEFAULT_FREQ_HZ   20000.0f  /* Default burst repetition frequency (20 kHz) */
#define BURST_DEFAULT_DUTY      0.30f     /* Default burst on-duty (30%)                 */
#define BURST_INTEGRATOR_DECAY  0.999f    /* Slow integrator decay per ms during idle    */
#define BURST_MIN_LLC_CYCLES    5U        /* Minimum LLC switching cycles per burst on   */

/** @brief  HRTIM burst clock: fHRTIM / 16 = 170 MHz / 16 = 10.625 MHz */
#define BURST_HRTIM_CLK_HZ      10625000U

/* ------------------------------------------------------------------ */
/*  NVIC Priorities                                                    */
/* ------------------------------------------------------------------ */

#define NVIC_PRIO_HRTIM_FLT    0U
#define NVIC_PRIO_HRTIM_PFC    1U
#define NVIC_PRIO_HRTIM_LLC    2U
#define NVIC_PRIO_TIM6         3U
#define NVIC_PRIO_FDCAN        5U
#define NVIC_PRIO_DMA          6U
#define NVIC_PRIO_USART        6U

/* ------------------------------------------------------------------ */
/*  GPIO Pin Definitions                                               */
/* ------------------------------------------------------------------ */

/* Debug LED */
#define DEBUG_LED_PORT         GPIOA
#define DEBUG_LED_PIN          GPIO_PIN_5

/* Debug tick output (TIM6 ISR toggle for scope verification) */
#define DEBUG_TICK_PORT        GPIOB
#define DEBUG_TICK_PIN         GPIO_PIN_0

/* HRTIM PFC outputs — Timer A (PA8/PA9), Timer B (PA10/PA11), Timer C (PB12/PB13) */
#define HRTIM_PFC_A1_PORT      GPIOA
#define HRTIM_PFC_A1_PIN       GPIO_PIN_8    /* HRTIM_CHA1 — Timer A output 1 */
#define HRTIM_PFC_A2_PORT      GPIOA
#define HRTIM_PFC_A2_PIN       GPIO_PIN_9    /* HRTIM_CHA2 — Timer A output 2 */
#define HRTIM_PFC_B1_PORT      GPIOA
#define HRTIM_PFC_B1_PIN       GPIO_PIN_10   /* HRTIM_CHB1 — Timer B output 1 */
#define HRTIM_PFC_B2_PORT      GPIOA
#define HRTIM_PFC_B2_PIN       GPIO_PIN_11   /* HRTIM_CHB2 — Timer B output 2 */
#define HRTIM_PFC_C1_PORT      GPIOB
#define HRTIM_PFC_C1_PIN       GPIO_PIN_12   /* HRTIM_CHC1 — Timer C output 1 */
#define HRTIM_PFC_C2_PORT      GPIOB
#define HRTIM_PFC_C2_PIN       GPIO_PIN_13   /* HRTIM_CHC2 — Timer C output 2 */

/* Relay / contactor control — PC0/PC1 (relocated from PC8/PC9 for LLC HRTIM) */
#define RELAY_PFC_PORT         GPIOC
#define RELAY_PFC_PIN          GPIO_PIN_0
#define RELAY_LLC_PORT         GPIOC
#define RELAY_LLC_PIN          GPIO_PIN_1

/* NTC bypass relay — PC2 (shorts inrush NTC once bus is charged) */
#define RELAY_NTC_PORT         GPIOC
#define RELAY_NTC_PIN          GPIO_PIN_2

/* HRTIM LLC outputs — Timer D (PB14/PB15), Timer E (PC8/PC9), Timer F (PC6/PC7) */
#define HRTIM_LLC_D1_PORT      GPIOB
#define HRTIM_LLC_D1_PIN       GPIO_PIN_14   /* HRTIM_CHD1 — Timer D output 1, AF13 */
#define HRTIM_LLC_D2_PORT      GPIOB
#define HRTIM_LLC_D2_PIN       GPIO_PIN_15   /* HRTIM_CHD2 — Timer D output 2, AF13 */
#define HRTIM_LLC_E1_PORT      GPIOC
#define HRTIM_LLC_E1_PIN       GPIO_PIN_8    /* HRTIM_CHE1 — Timer E output 1, AF3  */
#define HRTIM_LLC_E2_PORT      GPIOC
#define HRTIM_LLC_E2_PIN       GPIO_PIN_9    /* HRTIM_CHE2 — Timer E output 2, AF3  */
#define HRTIM_LLC_F1_PORT      GPIOC
#define HRTIM_LLC_F1_PIN       GPIO_PIN_6    /* HRTIM_CHF1 — Timer F output 1, AF13 */
#define HRTIM_LLC_F2_PORT      GPIOC
#define HRTIM_LLC_F2_PIN       GPIO_PIN_7    /* HRTIM_CHF2 — Timer F output 2, AF13 */

/* FDCAN1 pins (remapped to PB8/PB9 — PA11/PA12 used by HRTIM Timer B) */
#define FDCAN1_RX_PORT         GPIOB
#define FDCAN1_RX_PIN          GPIO_PIN_8    /* AF9 */
#define FDCAN1_TX_PORT         GPIOB
#define FDCAN1_TX_PIN          GPIO_PIN_9    /* AF9 */

/* HRTIM fault inputs — hardware OCP/OVP protection */
#define HRTIM_FLT1_PORT        GPIOA
#define HRTIM_FLT1_PIN         GPIO_PIN_12   /* PFC OCP, AF13 */
#define HRTIM_FLT2_PORT        GPIOA
#define HRTIM_FLT2_PIN         GPIO_PIN_15   /* LLC OCP, AF13 */
#define HRTIM_FLT3_PORT        GPIOB
#define HRTIM_FLT3_PIN         GPIO_PIN_10   /* DC bus OVP, AF13 */
#define HRTIM_FLT4_PORT        GPIOB
#define HRTIM_FLT4_PIN         GPIO_PIN_11   /* Output OVP, AF13 */
#define HRTIM_FLT5_PORT        GPIOB
#define HRTIM_FLT5_PIN         GPIO_PIN_2    /* Ground fault, AF13 */

/* USART2 pins */
#define USART2_TX_PORT         GPIOA
#define USART2_TX_PIN          GPIO_PIN_2
#define USART2_RX_PORT         GPIOA
#define USART2_RX_PIN          GPIO_PIN_3

/* DIP switch — node ID selection (active-low, internal pull-up) */
#define DIP_SW_PORT            GPIOC
#define DIP_SW0_PIN            GPIO_PIN_10
#define DIP_SW1_PIN            GPIO_PIN_11
#define DIP_SW2_PIN            GPIO_PIN_12

/* ------------------------------------------------------------------ */
/*  Peripheral Handle Externs                                          */
/* ------------------------------------------------------------------ */

extern HRTIM_HandleTypeDef   hhrtim1;
extern ADC_HandleTypeDef     hadc1;
extern ADC_HandleTypeDef     hadc2;
extern ADC_HandleTypeDef     hadc3;
extern ADC_HandleTypeDef     hadc4;
extern ADC_HandleTypeDef     hadc5;
extern FDCAN_HandleTypeDef   hfdcan1;
extern TIM_HandleTypeDef     htim6;
extern UART_HandleTypeDef    huart2;
extern CORDIC_HandleTypeDef  hcordic;
extern FMAC_HandleTypeDef    hfmac;

/* ------------------------------------------------------------------ */
/*  Public Functions                                                   */
/* ------------------------------------------------------------------ */

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
