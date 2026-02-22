/**
 * @file    main.c
 * @brief   Main entry point — 30 kW PDU Firmware
 * @target  STM32G474RET6
 */

#include "main.h"
#include "app_statemachine.h"
#include "app_adc.h"
#include "app_protection.h"
#include "app_can.h"
#include "app_control.h"
#include "app_powerseq.h"
#include "app_diagnostics.h"

/* Private function prototypes */
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

/**
 * @brief  Main entry point
 */
int main(void)
{
    /* HAL init */
    HAL_Init();

    /* Configure system clock: 170 MHz from HSE via PLL */
    SystemClock_Config();

    /* Peripheral init (CubeMX generated) */
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
    App_Control_Init();
    App_PowerSeq_Init();
    App_Diagnostics_Init();
    App_SM_Init();

    /* Main loop — state machine runs at 1 kHz via TIM6 flag */
    while (1)
    {
        App_SM_Run();
        App_Diagnostics_Poll();
    }
}

/* Stub implementations — replaced by CubeMX generated code */

static void SystemClock_Config(void)
{
    /* TODO: CubeMX generated — 170 MHz from HSE, HRTIM DLL 5.44 GHz */
}

static void MX_GPIO_Init(void) { /* TODO: CubeMX generated */ }
static void MX_DMA_Init(void) { /* TODO: CubeMX generated */ }
static void MX_HRTIM1_Init(void) { /* TODO: CubeMX generated */ }
static void MX_ADC1_Init(void) { /* TODO: CubeMX generated */ }
static void MX_ADC2_Init(void) { /* TODO: CubeMX generated */ }
static void MX_ADC3_Init(void) { /* TODO: CubeMX generated */ }
static void MX_ADC4_Init(void) { /* TODO: CubeMX generated */ }
static void MX_ADC5_Init(void) { /* TODO: CubeMX generated */ }
static void MX_FDCAN1_Init(void) { /* TODO: CubeMX generated */ }
static void MX_TIM6_Init(void) { /* TODO: CubeMX generated */ }
static void MX_USART2_UART_Init(void) { /* TODO: CubeMX generated */ }
static void MX_CORDIC_Init(void) { /* TODO: CubeMX generated */ }
static void MX_FMAC_Init(void) { /* TODO: CubeMX generated */ }

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
