/**
 * @file    app_powerseq.c
 * @brief   PFC/LLC soft-start, shutdown, burst mode
 */

#include "app_powerseq.h"

void App_PowerSeq_Init(void)
{
    /* TODO: Init soft-start state variables */
}

void PFC_SoftStart_Begin(void)
{
    /* TODO: I_d* = 0, anti-windup pre-load, start ramp timer */
}

uint8_t PFC_SoftStart_Tick(void)
{
    /* TODO: Linear ramp I_d* over 200 ms, NTC bypass at 80% Vbus */
    return 0;
}

void LLC_SoftStart_Begin(void)
{
    /* TODO: Start at 300 kHz, stagger phases (100 ms spacing) */
}

uint8_t LLC_SoftStart_Tick(void)
{
    /* TODO: Frequency ramp down to operating point, close contactor */
    return 0;
}

void Shutdown_Begin(void)
{
    /* TODO: Ramp I_out to 0, disable outputs, open contactor */
}

uint8_t Shutdown_Tick(void)
{
    /* TODO: Reverse startup sequence */
    return 0;
}

void Burst_Mode_Tick(void)
{
    /* TODO: HRTIM burst controller, integrator freeze during idle */
}
