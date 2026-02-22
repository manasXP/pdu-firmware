/**
 * @file    app_powerseq.h
 * @brief   Power-on/shutdown sequences, soft-start, burst mode
 */

#ifndef APP_POWERSEQ_H
#define APP_POWERSEQ_H

#include <stdint.h>

void App_PowerSeq_Init(void);

/* PFC soft-start: I_d* ramp 0 -> rated over 200 ms */
void PFC_SoftStart_Begin(void);
uint8_t PFC_SoftStart_Tick(void);  /* Returns 1 when complete */

/* LLC soft-start: frequency ramp 300 kHz -> operating, staggered phase enable */
void LLC_SoftStart_Begin(void);
uint8_t LLC_SoftStart_Tick(void);  /* Returns 1 when complete */

/* Shutdown: reverse sequence */
void Shutdown_Begin(void);
uint8_t Shutdown_Tick(void);       /* Returns 1 when complete */

/* Burst mode sub-state */
typedef enum
{
    BURST_INACTIVE = 0,  /* Normal continuous operation    */
    BURST_RUN      = 1,  /* Burst on — LLC switching       */
    BURST_IDLE     = 2   /* Burst off — LLC outputs gated  */
} BurstState_t;

void         Burst_Mode_Tick(void);
BurstState_t Burst_Mode_GetState(void);

#endif /* APP_POWERSEQ_H */
