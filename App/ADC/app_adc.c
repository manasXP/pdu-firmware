/**
 * @file    app_adc.c
 * @brief   ADC pipeline — injected (HRTIM-triggered), regular (DMA), filters
 */

#include "app_adc.h"

static ADC_Readings_t s_readings;

void App_ADC_Init(void)
{
    /* TODO: ADC self-calibration (5 instances x 2 ms) */
    /* TODO: Configure HRTIM ADC trigger routing */
    /* TODO: Configure DMA circular double-buffer for regular group */
    /* TODO: Init software filters: biquad IIR (Vbus), EMA (temp), 1st-order (LLC) */
}

const ADC_Readings_t *App_ADC_GetReadings(void)
{
    return &s_readings;
}
