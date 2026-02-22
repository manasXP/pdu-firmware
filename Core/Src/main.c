/**
 * @file    main.c
 * @brief   Main entry point — 30 kW PDU Firmware
 * @target  STM32G474RET6
 *
 * System init sequence: HAL_Init → SystemClock_Config (170 MHz) →
 * peripheral init (GPIO, DMA, HRTIM, ADC×5, FDCAN, TIM6, UART,
 * CORDIC, FMAC) → App module init → main loop.
 */

#include "main.h"
#include "app_statemachine.h"
#include "app_adc.h"
#include "app_protection.h"
#include "app_can.h"
#include "app_control.h"
#include "app_powerseq.h"
#include "app_diagnostics.h"
#include "app_pll.h"

/* ------------------------------------------------------------------ */
/*  Peripheral Handles (file-scope)                                    */
/* ------------------------------------------------------------------ */

HRTIM_HandleTypeDef  hhrtim1;
ADC_HandleTypeDef    hadc1;
ADC_HandleTypeDef    hadc2;
ADC_HandleTypeDef    hadc3;
ADC_HandleTypeDef    hadc4;
ADC_HandleTypeDef    hadc5;
FDCAN_HandleTypeDef  hfdcan1;
TIM_HandleTypeDef    htim6;
UART_HandleTypeDef   huart2;
CORDIC_HandleTypeDef hcordic;
FMAC_HandleTypeDef   hfmac;

/* ------------------------------------------------------------------ */
/*  Private Function Prototypes                                        */
/* ------------------------------------------------------------------ */

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_HRTIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC4_Init(void);
static void MX_ADC5_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CORDIC_Init(void);
static void MX_FMAC_Init(void);

/* ------------------------------------------------------------------ */
/*  Main                                                               */
/* ------------------------------------------------------------------ */

/**
 * @brief  Main entry point
 */
int main(void)
{
    /* HAL init (SysTick at 1 ms, NVIC priority grouping) */
    HAL_Init();

    /* Configure system clock: HSE 8 MHz → PLL → 170 MHz SYSCLK */
    SystemClock_Config();

    /* Peripheral init — order matters: GPIO/DMA before HRTIM/ADC */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_HRTIM1_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_ADC3_Init();
    MX_ADC4_Init();
    MX_ADC5_Init();
    MX_FDCAN1_Init();
    MX_TIM6_Init();
    MX_USART2_UART_Init();
    MX_CORDIC_Init();
    MX_FMAC_Init();

    /* Application init */
    App_ADC_Init();
    App_Protection_Init();
    App_CAN_Init();
    App_PLL_Init();
    App_Control_Init();
    App_PowerSeq_Init();
    App_Diagnostics_Init();
    App_SM_Init();

    /* Start HRTIM-triggered injected conversions on all ADCs */
    App_ADC_StartInjected();

    /* Main loop — state machine runs at 1 kHz via TIM6 flag */
    while (1)
    {
        App_SM_Run();
        App_ADC_RegularProcess();
        App_Control_LLC_Sweep();
        App_Diagnostics_Poll();
    }
}

/* ================================================================== */
/*  Peripheral Init Implementations                                    */
/* ================================================================== */

/**
 * @brief  System clock configuration
 *
 * HSE 8 MHz → PLL (M=2, N=85, R=2) → SYSCLK 170 MHz
 * AHB = 170 MHz, APB1 = 170 MHz, APB2 = 170 MHz
 * Flash latency: 8 wait states (boost mode, range 1)
 *
 * Timing: ~2 ms (HSE startup + PLL lock)
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Configure voltage scaling for 170 MHz operation */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    /* HSE oscillator + PLL configuration */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = RCC_PLLM_DIV2;   /* 8 / 2 = 4 MHz */
    RCC_OscInitStruct.PLL.PLLN       = 85U;              /* 4 × 85 = 340 MHz VCO */
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR       = RCC_PLLR_DIV2;    /* 340 / 2 = 170 MHz */

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* Select PLL as SYSCLK source; configure bus dividers */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    /* Flash latency 8 WS for 170 MHz in boost mode */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_8) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  GPIO initialization
 *
 * Enables clocks for GPIOA, GPIOB, GPIOC.
 * Configures debug LED (PA5) and debug tick (PB0) as push-pull outputs.
 * All HRTIM output pins start in a safe (low) state.
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Debug LED — PA5 push-pull, low speed, start OFF */
    HAL_GPIO_WritePin(DEBUG_LED_PORT, DEBUG_LED_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = DEBUG_LED_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DEBUG_LED_PORT, &GPIO_InitStruct);

    /* Debug tick — PB0 push-pull for scope verification */
    HAL_GPIO_WritePin(DEBUG_TICK_PORT, DEBUG_TICK_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = DEBUG_TICK_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DEBUG_TICK_PORT, &GPIO_InitStruct);

    /* Relay/contactor outputs — start OFF (safe state) */
    HAL_GPIO_WritePin(RELAY_PFC_PORT, RELAY_PFC_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_LLC_PORT, RELAY_LLC_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = RELAY_PFC_PIN | RELAY_LLC_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RELAY_PFC_PORT, &GPIO_InitStruct);

    /* HRTIM PFC outputs — Timer A (PA8/PA9), Timer B (PA10/PA11) */
    GPIO_InitStruct.Pin   = HRTIM_PFC_A1_PIN | HRTIM_PFC_A2_PIN
                          | HRTIM_PFC_B1_PIN | HRTIM_PFC_B2_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* HRTIM PFC outputs — Timer C (PB12/PB13) + LLC Timer D (PB14/PB15) */
    GPIO_InitStruct.Pin   = HRTIM_PFC_C1_PIN | HRTIM_PFC_C2_PIN
                          | HRTIM_LLC_D1_PIN | HRTIM_LLC_D2_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* HRTIM LLC outputs — Timer E (PC8/PC9), AF3 */
    GPIO_InitStruct.Pin   = HRTIM_LLC_E1_PIN | HRTIM_LLC_E2_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_HRTIM1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* HRTIM LLC outputs — Timer F (PC6/PC7), AF13 */
    GPIO_InitStruct.Pin   = HRTIM_LLC_F1_PIN | HRTIM_LLC_F2_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* HRTIM fault inputs — FLT1 (PA12), FLT2 (PA15), AF13 */
    GPIO_InitStruct.Pin   = HRTIM_FLT1_PIN | HRTIM_FLT2_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* HRTIM fault inputs — FLT3 (PB10), FLT4 (PB11), FLT5 (PB2), AF13 */
    GPIO_InitStruct.Pin   = HRTIM_FLT3_PIN | HRTIM_FLT4_PIN
                          | HRTIM_FLT5_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* FDCAN1 — PB8 (RX), PB9 (TX), AF9 */
    GPIO_InitStruct.Pin   = FDCAN1_RX_PIN | FDCAN1_TX_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
 * @brief  DMA initialization
 *
 * Enables DMA1, DMA2, DMAMUX1 clocks and configures NVIC priorities.
 * Channels are configured by individual peripheral init functions.
 */
static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();

    /* DMA1 channel interrupts */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_PRIO_DMA, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, NVIC_PRIO_DMA, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

    /* DMA2 channel interrupts */
    HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, NVIC_PRIO_DMA, 0);
    HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
    HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, NVIC_PRIO_DMA, 0);
    HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);
    HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, NVIC_PRIO_DMA, 0);
    HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
}

/**
 * @brief  HRTIM1 initialization
 *
 * Enables HRTIM clock, performs DLL calibration (14-20 ms), and configures
 * fault inputs FLT1-FLT4 for hardware overcurrent/overvoltage protection.
 * Timer units (PFC/LLC) are configured later by App_Control_Init().
 *
 * Timing: 14-20 ms (DLL calibration)
 */
static void MX_HRTIM1_Init(void)
{
    HRTIM_FaultCfgTypeDef FaultCfg = {0};

    __HAL_RCC_HRTIM1_CLK_ENABLE();

    hhrtim1.Instance = HRTIM1;
    hhrtim1.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
    hhrtim1.Init.SyncOptions             = HRTIM_SYNCOPTION_NONE;

    if (HAL_HRTIM_Init(&hhrtim1) != HAL_OK)
    {
        Error_Handler();
    }

    /* DLL calibration — required before any timer operation */
    if (HAL_HRTIM_DLLCalibrationStart(&hhrtim1, HRTIM_CALIBRATIONRATE_3)
        != HAL_OK)
    {
        Error_Handler();
    }

    /* Poll DLLRDY with timeout */
    uint32_t tickstart = HAL_GetTick();
    while (__HAL_HRTIM_GET_FLAG(&hhrtim1, HRTIM_FLAG_DLLRDY) == 0U)
    {
        if ((HAL_GetTick() - tickstart) > INIT_TIMEOUT_DLL_MS)
        {
            Error_Handler();
        }
    }

    /*
     * Fault inputs FLT1-FLT4: active-low, no filter (<200 ns response).
     * These provide hardware-level protection that disables HRTIM outputs
     * independently of software.
     */
    FaultCfg.Source    = HRTIM_FAULTSOURCE_DIGITALINPUT;
    FaultCfg.Polarity  = HRTIM_FAULTPOLARITY_LOW;
    FaultCfg.Filter    = HRTIM_FAULTFILTER_NONE;
    FaultCfg.Lock      = HRTIM_FAULTLOCK_READWRITE;

    HAL_HRTIM_FaultConfig(&hhrtim1, HRTIM_FAULT_1, &FaultCfg);
    HAL_HRTIM_FaultConfig(&hhrtim1, HRTIM_FAULT_2, &FaultCfg);
    HAL_HRTIM_FaultConfig(&hhrtim1, HRTIM_FAULT_3, &FaultCfg);
    HAL_HRTIM_FaultConfig(&hhrtim1, HRTIM_FAULT_4, &FaultCfg);
    HAL_HRTIM_FaultConfig(&hhrtim1, HRTIM_FAULT_5, &FaultCfg);

    /* Enable all 5 fault inputs */
    HAL_HRTIM_FaultModeCtl(&hhrtim1,
                           HRTIM_FAULT_1 | HRTIM_FAULT_2
                         | HRTIM_FAULT_3 | HRTIM_FAULT_4
                         | HRTIM_FAULT_5,
                           HRTIM_FAULTMODECTL_ENABLED);

    /* Enable fault interrupt sources (FLT1–FLT5) */
    __HAL_HRTIM_ENABLE_IT(&hhrtim1,
                          HRTIM_IT_FLT1 | HRTIM_IT_FLT2
                        | HRTIM_IT_FLT3 | HRTIM_IT_FLT4
                        | HRTIM_IT_FLT5);

    /* HRTIM fault ISR — highest priority */
    HAL_NVIC_SetPriority(HRTIM1_FLT_IRQn, NVIC_PRIO_HRTIM_FLT, 0);
    HAL_NVIC_EnableIRQ(HRTIM1_FLT_IRQn);
}

/**
 * @brief  ADC helper — common injected-group init for one ADC instance
 * @param  hadc       Pointer to ADC handle
 * @param  instance   ADC peripheral instance (ADC1..ADC5)
 */
static void adc_common_init(ADC_HandleTypeDef *hadc, ADC_TypeDef *instance)
{
    hadc->Instance                   = instance;
    hadc->Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;
    hadc->Init.Resolution            = ADC_RESOLUTION_12B;
    hadc->Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc->Init.GainCompensation      = 0U;
    hadc->Init.ScanConvMode          = ADC_SCAN_ENABLE;
    hadc->Init.EOCSelection          = ADC_EOC_SEQ_CONV;
    hadc->Init.LowPowerAutoWait      = DISABLE;
    hadc->Init.ContinuousConvMode    = DISABLE;
    hadc->Init.NbrOfConversion       = 1U;
    hadc->Init.DiscontinuousConvMode = DISABLE;
    hadc->Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc->Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc->Init.DMAContinuousRequests = DISABLE;
    hadc->Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
    hadc->Init.OversamplingMode      = DISABLE;

    if (HAL_ADC_Init(hadc) != HAL_OK)
    {
        Error_Handler();
    }

    /* Single-ended calibration */
    if (HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  ADC1 init — INJ: I_A (phase A current), V_DC_bus
 */
static void MX_ADC1_Init(void)
{
    ADC_InjectionConfTypeDef InjConfig = {0};

    __HAL_RCC_ADC12_CLK_ENABLE();

    adc_common_init(&hadc1, ADC1);

    /* Injected channel: I_A */
    InjConfig.InjectedChannel               = ADC_CHANNEL_1;
    InjConfig.InjectedRank                   = ADC_INJECTED_RANK_1;
    InjConfig.InjectedSamplingTime           = ADC_SAMPLETIME_6CYCLES_5;
    InjConfig.InjectedSingleDiff             = ADC_SINGLE_ENDED;
    InjConfig.InjectedOffsetNumber           = ADC_OFFSET_NONE;
    InjConfig.InjectedOffset                 = 0U;
    InjConfig.InjectedNbrOfConversion        = 2U;
    InjConfig.InjectedDiscontinuousConvMode  = DISABLE;
    InjConfig.AutoInjectedConv               = DISABLE;
    InjConfig.QueueInjectedContext           = DISABLE;
    InjConfig.ExternalTrigInjecConv          = ADC_EXTERNALTRIGINJEC_HRTIM_TRG1;
    InjConfig.ExternalTrigInjecConvEdge      = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &InjConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* Injected channel: V_DC_bus */
    InjConfig.InjectedChannel = ADC_CHANNEL_2;
    InjConfig.InjectedRank    = ADC_INJECTED_RANK_2;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &InjConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  ADC2 init — INJ: I_B (phase B current), V_A (phase A voltage)
 */
static void MX_ADC2_Init(void)
{
    ADC_InjectionConfTypeDef InjConfig = {0};

    /* ADC12 clock already enabled by ADC1 init */

    adc_common_init(&hadc2, ADC2);

    InjConfig.InjectedChannel               = ADC_CHANNEL_3;
    InjConfig.InjectedRank                   = ADC_INJECTED_RANK_1;
    InjConfig.InjectedSamplingTime           = ADC_SAMPLETIME_6CYCLES_5;
    InjConfig.InjectedSingleDiff             = ADC_SINGLE_ENDED;
    InjConfig.InjectedOffsetNumber           = ADC_OFFSET_NONE;
    InjConfig.InjectedOffset                 = 0U;
    InjConfig.InjectedNbrOfConversion        = 2U;
    InjConfig.InjectedDiscontinuousConvMode  = DISABLE;
    InjConfig.AutoInjectedConv               = DISABLE;
    InjConfig.QueueInjectedContext           = DISABLE;
    InjConfig.ExternalTrigInjecConv          = ADC_EXTERNALTRIGINJEC_HRTIM_TRG1;
    InjConfig.ExternalTrigInjecConvEdge      = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &InjConfig) != HAL_OK)
    {
        Error_Handler();
    }

    InjConfig.InjectedChannel = ADC_CHANNEL_4;
    InjConfig.InjectedRank    = ADC_INJECTED_RANK_2;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &InjConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  ADC3 init — INJ: I_C (phase C current), V_B (phase B voltage)
 */
static void MX_ADC3_Init(void)
{
    ADC_InjectionConfTypeDef InjConfig = {0};

    __HAL_RCC_ADC345_CLK_ENABLE();

    adc_common_init(&hadc3, ADC3);

    InjConfig.InjectedChannel               = ADC_CHANNEL_1;
    InjConfig.InjectedRank                   = ADC_INJECTED_RANK_1;
    InjConfig.InjectedSamplingTime           = ADC_SAMPLETIME_6CYCLES_5;
    InjConfig.InjectedSingleDiff             = ADC_SINGLE_ENDED;
    InjConfig.InjectedOffsetNumber           = ADC_OFFSET_NONE;
    InjConfig.InjectedOffset                 = 0U;
    InjConfig.InjectedNbrOfConversion        = 2U;
    InjConfig.InjectedDiscontinuousConvMode  = DISABLE;
    InjConfig.AutoInjectedConv               = DISABLE;
    InjConfig.QueueInjectedContext           = DISABLE;
    InjConfig.ExternalTrigInjecConv          = ADC_EXTERNALTRIGINJEC_HRTIM_TRG2;
    InjConfig.ExternalTrigInjecConvEdge      = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &InjConfig) != HAL_OK)
    {
        Error_Handler();
    }

    InjConfig.InjectedChannel = ADC_CHANNEL_2;
    InjConfig.InjectedRank    = ADC_INJECTED_RANK_2;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &InjConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  ADC4 init — INJ: V_C (phase C voltage), V_out (output voltage)
 */
static void MX_ADC4_Init(void)
{
    ADC_InjectionConfTypeDef InjConfig = {0};

    /* ADC345 clock already enabled by ADC3 init */

    adc_common_init(&hadc4, ADC4);

    InjConfig.InjectedChannel               = ADC_CHANNEL_3;
    InjConfig.InjectedRank                   = ADC_INJECTED_RANK_1;
    InjConfig.InjectedSamplingTime           = ADC_SAMPLETIME_6CYCLES_5;
    InjConfig.InjectedSingleDiff             = ADC_SINGLE_ENDED;
    InjConfig.InjectedOffsetNumber           = ADC_OFFSET_NONE;
    InjConfig.InjectedOffset                 = 0U;
    InjConfig.InjectedNbrOfConversion        = 2U;
    InjConfig.InjectedDiscontinuousConvMode  = DISABLE;
    InjConfig.AutoInjectedConv               = DISABLE;
    InjConfig.QueueInjectedContext           = DISABLE;
    InjConfig.ExternalTrigInjecConv          = ADC_EXTERNALTRIGINJEC_HRTIM_TRG2;
    InjConfig.ExternalTrigInjecConvEdge      = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc4, &InjConfig) != HAL_OK)
    {
        Error_Handler();
    }

    InjConfig.InjectedChannel = ADC_CHANNEL_4;
    InjConfig.InjectedRank    = ADC_INJECTED_RANK_2;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc4, &InjConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  ADC5 init — INJ: I_out (output current)
 */
static void MX_ADC5_Init(void)
{
    ADC_InjectionConfTypeDef InjConfig = {0};

    /* ADC345 clock already enabled by ADC3 init */

    adc_common_init(&hadc5, ADC5);

    InjConfig.InjectedChannel               = ADC_CHANNEL_1;
    InjConfig.InjectedRank                   = ADC_INJECTED_RANK_1;
    InjConfig.InjectedSamplingTime           = ADC_SAMPLETIME_6CYCLES_5;
    InjConfig.InjectedSingleDiff             = ADC_SINGLE_ENDED;
    InjConfig.InjectedOffsetNumber           = ADC_OFFSET_NONE;
    InjConfig.InjectedOffset                 = 0U;
    InjConfig.InjectedNbrOfConversion        = 1U;
    InjConfig.InjectedDiscontinuousConvMode  = DISABLE;
    InjConfig.AutoInjectedConv               = DISABLE;
    InjConfig.QueueInjectedContext           = DISABLE;
    InjConfig.ExternalTrigInjecConv          = ADC_EXTERNALTRIGINJEC_HRTIM_TRG2;
    InjConfig.ExternalTrigInjecConvEdge      = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc5, &InjConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  FDCAN1 initialization
 *
 * 500 kbps classical CAN (from 170 MHz PCLK1):
 *   Prescaler=20, TS1=13, TS2=3 → Tq = 17, bit time = 20 ns × 17 = 340 ns
 *   Bit rate = 1 / (20 × 17 / 170e6) = 500 kHz
 *
 * Accept-all filter for initial bring-up.
 */
static void MX_FDCAN1_Init(void)
{
    FDCAN_FilterTypeDef FilterConfig = {0};

    __HAL_RCC_FDCAN_CLK_ENABLE();

    hfdcan1.Instance                  = FDCAN1;
    hfdcan1.Init.ClockDivider         = FDCAN_CLOCK_DIV1;
    hfdcan1.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode                 = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission   = ENABLE;
    hfdcan1.Init.TransmitPause        = ENABLE;
    hfdcan1.Init.ProtocolException    = DISABLE;
    hfdcan1.Init.NominalPrescaler     = 20U;
    hfdcan1.Init.NominalSyncJumpWidth = 1U;
    hfdcan1.Init.NominalTimeSeg1      = 13U;
    hfdcan1.Init.NominalTimeSeg2      = 3U;
    hfdcan1.Init.StdFiltersNbr        = 1U;
    hfdcan1.Init.ExtFiltersNbr        = 0U;
    hfdcan1.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;

    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }

    /* Accept-all standard ID filter for bring-up */
    FilterConfig.IdType       = FDCAN_STANDARD_ID;
    FilterConfig.FilterIndex  = 0U;
    FilterConfig.FilterType   = FDCAN_FILTER_MASK;
    FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FilterConfig.FilterID1    = 0x000U;
    FilterConfig.FilterID2    = 0x000U;    /* Mask = 0 → accept all */

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &FilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* Reject frames not matching any filter into Rx FIFO 0 */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    /* Start FDCAN */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }

    /* Enable Rx FIFO 0 new message interrupt */
    if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                       0U) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, NVIC_PRIO_FDCAN, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
}

/**
 * @brief  TIM6 initialization — 1 kHz state-machine tick
 *
 * From 170 MHz APB1 timer clock:
 *   Prescaler = 169 → 1 MHz count frequency
 *   Period    = 999 → 1 kHz overflow rate
 */
static void MX_TIM6_Init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();

    htim6.Instance               = TIM6;
    htim6.Init.Prescaler         = 169U;
    htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim6.Init.Period            = 999U;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        Error_Handler();
    }

    /* Start TIM6 with update interrupt */
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, NVIC_PRIO_TIM6, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/**
 * @brief  USART2 initialization — 115200 baud, 8N1 (diagnostics CLI)
 */
static void MX_USART2_UART_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();

    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200U;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling        = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler        = UART_PRESCALER_DIV1;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(USART2_IRQn, NVIC_PRIO_USART, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/**
 * @brief  CORDIC initialization
 *
 * Configured for cosine function, q1.31 format, 6-cycle precision (~20-bit),
 * 2 output reads (cos + sin). Used by PFC dq-frame transforms.
 */
static void MX_CORDIC_Init(void)
{
    __HAL_RCC_CORDIC_CLK_ENABLE();

    hcordic.Instance = CORDIC;

    if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  FMAC initialization
 *
 * Enables clock and performs reset. Filter coefficients are loaded later
 * by App_ADC_Init() for signal conditioning.
 */
static void MX_FMAC_Init(void)
{
    __HAL_RCC_FMAC_CLK_ENABLE();

    hfmac.Instance = FMAC;

    if (HAL_FMAC_Init(&hfmac) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ------------------------------------------------------------------ */
/*  Error Handler                                                      */
/* ------------------------------------------------------------------ */

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
