/**
 * @file    stm32g4xx_hal_conf.h
 * @brief   HAL configuration — 30 kW PDU Firmware
 * @target  STM32G474RET6
 */

#ifndef STM32G4XX_HAL_CONF_H
#define STM32G4XX_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  Module Selection                                                   */
/* ------------------------------------------------------------------ */
#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_CORDIC_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_FDCAN_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_FMAC_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_HRTIM_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

/* ------------------------------------------------------------------ */
/*  Oscillator Values                                                  */
/* ------------------------------------------------------------------ */

/** @brief  HSE crystal frequency (Hz) */
#if !defined(HSE_VALUE)
#define HSE_VALUE    8000000U
#endif

/** @brief  HSE startup timeout (ms) */
#if !defined(HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT    100U
#endif

/** @brief  HSI frequency (Hz) */
#if !defined(HSI_VALUE)
#define HSI_VALUE    16000000U
#endif

/** @brief  HSI48 frequency (Hz) */
#if !defined(HSI48_VALUE)
#define HSI48_VALUE  48000000U
#endif

/** @brief  LSI frequency (Hz) */
#if !defined(LSI_VALUE)
#define LSI_VALUE    32000U
#endif

/** @brief  LSE frequency (Hz) */
#if !defined(LSE_VALUE)
#define LSE_VALUE    32768U
#endif

/** @brief  LSE startup timeout (ms) */
#if !defined(LSE_STARTUP_TIMEOUT)
#define LSE_STARTUP_TIMEOUT    5000U
#endif

/** @brief  External clock source for I2S (Hz) */
#if !defined(EXTERNAL_CLOCK_VALUE)
#define EXTERNAL_CLOCK_VALUE    12288000U
#endif

/* ------------------------------------------------------------------ */
/*  System Configuration                                               */
/* ------------------------------------------------------------------ */

#define VDD_VALUE                    3300U   /* mV */
#define TICK_INT_PRIORITY            15U     /* Lowest priority for SysTick */
#define USE_RTOS                     0U
#define PREFETCH_ENABLE              1U
#define INSTRUCTION_CACHE_ENABLE     1U
#define DATA_CACHE_ENABLE            1U

/* ------------------------------------------------------------------ */
/*  Assert / Debug                                                     */
/* ------------------------------------------------------------------ */

/* #define USE_FULL_ASSERT    1U */

/* ------------------------------------------------------------------ */
/*  HAL Module Includes                                                */
/* ------------------------------------------------------------------ */

#ifdef HAL_RCC_MODULE_ENABLED
#include "stm32g4xx_hal_rcc.h"
#include "stm32g4xx_hal_rcc_ex.h"
#endif

#ifdef HAL_GPIO_MODULE_ENABLED
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_gpio_ex.h"
#endif

#ifdef HAL_DMA_MODULE_ENABLED
#include "stm32g4xx_hal_dma.h"
#include "stm32g4xx_hal_dma_ex.h"
#endif

#ifdef HAL_CORTEX_MODULE_ENABLED
#include "stm32g4xx_hal_cortex.h"
#endif

#ifdef HAL_ADC_MODULE_ENABLED
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_adc_ex.h"
#endif

#ifdef HAL_CORDIC_MODULE_ENABLED
#include "stm32g4xx_hal_cordic.h"
#endif

#ifdef HAL_FDCAN_MODULE_ENABLED
#include "stm32g4xx_hal_fdcan.h"
#endif

#ifdef HAL_FLASH_MODULE_ENABLED
#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_hal_flash_ex.h"
#include "stm32g4xx_hal_flash_ramfunc.h"
#endif

#ifdef HAL_FMAC_MODULE_ENABLED
#include "stm32g4xx_hal_fmac.h"
#endif

#ifdef HAL_HRTIM_MODULE_ENABLED
#include "stm32g4xx_hal_hrtim.h"
#endif

#ifdef HAL_PWR_MODULE_ENABLED
#include "stm32g4xx_hal_pwr.h"
#include "stm32g4xx_hal_pwr_ex.h"
#endif

#ifdef HAL_TIM_MODULE_ENABLED
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_hal_tim_ex.h"
#endif

#ifdef HAL_UART_MODULE_ENABLED
#include "stm32g4xx_hal_uart.h"
#include "stm32g4xx_hal_uart_ex.h"
#endif

/* ------------------------------------------------------------------ */
/*  Assert Macro                                                       */
/* ------------------------------------------------------------------ */

#ifdef USE_FULL_ASSERT
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
void assert_failed(uint8_t *file, uint32_t line);
#else
#define assert_param(expr) ((void)0U)
#endif

#ifdef __cplusplus
}
#endif

#endif /* STM32G4XX_HAL_CONF_H */
