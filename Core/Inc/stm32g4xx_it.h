/**
 * @file    stm32g4xx_it.h
 * @brief   Interrupt handler prototypes — 30 kW PDU Firmware
 * @target  STM32G474RET6
 */

#ifndef STM32G4XX_IT_H
#define STM32G4XX_IT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Cortex-M4 system exceptions */
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

/* Peripheral interrupts */
void TIM6_DAC_IRQHandler(void);
void HRTIM1_FLT_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* STM32G4XX_IT_H */
