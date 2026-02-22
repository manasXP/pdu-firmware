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

/* Firmware version */
#define FW_VERSION_MAJOR  0
#define FW_VERSION_MINOR  1
#define FW_VERSION_BUILD  0

/* System clock: 170 MHz from HSE via PLL */
#define SYSCLK_FREQ_HZ   170000000U

/* HRTIM DLL frequency */
#define HRTIM_DLL_FREQ_HZ 5440000000ULL

/* Control loop rates */
#define PFC_ISR_FREQ_HZ   65000U    /* PFC switching frequency */
#define LLC_ISR_FREQ_HZ   150000U   /* LLC nominal switching frequency */
#define MAIN_LOOP_FREQ_HZ 1000U     /* 1 kHz state machine tick (TIM6) */
#define CAN_STATUS_PERIOD_MS 10U    /* CAN status broadcast interval */

/* ADC */
#define ADC_OVERSAMPLING_RATIO 16U

/* Protection thresholds */
#define PFC_OCP_THRESHOLD_A   78.0f
#define BUS_OVP_THRESHOLD_V   966.0f
#define OUT_OVP_THRESHOLD_V   1050.0f
#define OTP_DERATE_DEG_C      100.0f
#define OTP_SHUTDOWN_DEG_C    115.0f

/* Startup */
#define STARTUP_TIMEOUT_MS    6000U
#define PLL_LOCK_TIMEOUT_MS   2000U

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
