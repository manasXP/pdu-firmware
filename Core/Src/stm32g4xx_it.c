/**
 * @file    stm32g4xx_it.c
 * @brief   Interrupt service routines — 30 kW PDU Firmware
 * @target  STM32G474RET6
 */

#include "main.h"
#include "stm32g4xx_it.h"

/* ------------------------------------------------------------------ */
/*  External references                                                */
/* ------------------------------------------------------------------ */

extern TIM_HandleTypeDef htim6;
extern HRTIM_HandleTypeDef hhrtim1;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_adc3;
extern DMA_HandleTypeDef hdma_adc4;
extern DMA_HandleTypeDef hdma_adc5;

/** @brief  Provided by App/StateMachine — sets 1 kHz tick flag */
extern void App_SM_TickISR(void);

/** @brief  Provided by App/Control — PFC control loop at 65 kHz */
extern void App_Control_PFC_ISR(void);

/* ------------------------------------------------------------------ */
/*  Cortex-M4 System Exceptions                                        */
/* ------------------------------------------------------------------ */

void NMI_Handler(void)
{
    while (1)
    {
    }
}

void HardFault_Handler(void)
{
    while (1)
    {
    }
}

void MemManage_Handler(void)
{
    while (1)
    {
    }
}

void BusFault_Handler(void)
{
    while (1)
    {
    }
}

void UsageFault_Handler(void)
{
    while (1)
    {
    }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

/**
 * @brief  SysTick — 1 ms HAL timebase
 */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* ------------------------------------------------------------------ */
/*  Peripheral Interrupts                                              */
/* ------------------------------------------------------------------ */

/**
 * @brief  TIM6 update — 1 kHz state-machine tick
 *
 * Optional: toggle DEBUG_TICK_PIN for scope verification.
 */
void TIM6_DAC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim6);

    App_SM_TickISR();

#ifdef DEBUG_TICK_GPIO
    HAL_GPIO_TogglePin(DEBUG_TICK_PORT, DEBUG_TICK_PIN);
#endif
}

/**
 * @brief  HRTIM1 Master repetition interrupt — PFC control loop at 65 kHz
 *
 * Direct register access for minimum latency (ISR budget <3 us).
 */
void HRTIM1_Master_IRQHandler(void)
{
    HRTIM1->sCommonRegs.ICR = HRTIM_MICR_MREP;
    App_Control_PFC_ISR();
}

/**
 * @brief  HRTIM1 Timer D interrupt — LLC control loop (future EP-03-007)
 *
 * Skeleton for closed-loop LLC ISR. Currently unused by the open-loop
 * frequency sweep (ADC trigger 2 fires from hardware, no ISR needed).
 */
void HRTIM1_TIMD_IRQHandler(void)
{
    /* Clear Timer D repetition interrupt flag */
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_D].TIMxICR = HRTIM_TIMICR_REPC;
    /* Reserved for closed-loop LLC control (EP-03-007) */
}

/**
 * @brief  HRTIM1 fault interrupt — hardware OCP/OVP
 *
 * Fault inputs disable HRTIM outputs in <200 ns via hardware.
 * This ISR is for software-side notification and logging.
 */
void HRTIM1_FLT_IRQHandler(void)
{
    HAL_HRTIM_IRQHandler(&hhrtim1, HRTIM_TIMERINDEX_COMMON);
}

/* ------------------------------------------------------------------ */
/*  DMA Interrupts (ADC regular groups)                                */
/* ------------------------------------------------------------------ */

void DMA1_Channel1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_adc1);
}

void DMA1_Channel2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_adc2);
}

void DMA2_Channel1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_adc3);
}

void DMA2_Channel2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_adc4);
}

void DMA2_Channel3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_adc5);
}
