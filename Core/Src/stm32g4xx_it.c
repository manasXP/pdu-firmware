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

/** @brief  Provided by App/StateMachine — sets 1 kHz tick flag */
extern void App_SM_TickISR(void);

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
 * @brief  HRTIM1 fault interrupt — hardware OCP/OVP
 *
 * Fault inputs disable HRTIM outputs in <200 ns via hardware.
 * This ISR is for software-side notification and logging.
 */
void HRTIM1_FLT_IRQHandler(void)
{
    HAL_HRTIM_IRQHandler(&hhrtim1, HRTIM_TIMERINDEX_COMMON);
}
