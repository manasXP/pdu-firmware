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

/* HRTIM outputs — placeholder assignments (finalize from schematic) */
#define HRTIM_PFC_A_PORT       GPIOA
#define HRTIM_PFC_A_PIN        GPIO_PIN_8    /* HRTIM_CHA1 */
#define HRTIM_PFC_B_PORT       GPIOA
#define HRTIM_PFC_B_PIN        GPIO_PIN_9    /* HRTIM_CHA2 */

/* Relay / contactor control — placeholders */
#define RELAY_PFC_PORT         GPIOC
#define RELAY_PFC_PIN          GPIO_PIN_8
#define RELAY_LLC_PORT         GPIOC
#define RELAY_LLC_PIN          GPIO_PIN_9

/* FDCAN1 pins */
#define FDCAN1_TX_PORT         GPIOA
#define FDCAN1_TX_PIN          GPIO_PIN_12
#define FDCAN1_RX_PORT         GPIOA
#define FDCAN1_RX_PIN          GPIO_PIN_11

/* USART2 pins */
#define USART2_TX_PORT         GPIOA
#define USART2_TX_PIN          GPIO_PIN_2
#define USART2_RX_PORT         GPIOA
#define USART2_RX_PIN          GPIO_PIN_3

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
