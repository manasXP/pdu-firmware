/**
 * @file    app_adc.h
 * @brief   ADC pipeline — HRTIM-triggered injected + DMA regular groups
 */

#ifndef APP_ADC_H
#define APP_ADC_H

#include <stdint.h>

/* Processed ADC readings */
typedef struct
{
    float v_bus;          /* DC bus voltage (V) */
    float v_out;          /* LLC output voltage (V) */
    float i_phase_a;      /* PFC phase A current (A) */
    float i_phase_b;      /* PFC phase B current (A) */
    float i_phase_c;      /* PFC phase C current (A) */
    float i_out;          /* LLC output current (A) */
    float t_sic_pfc;      /* PFC MOSFET temperature (degC) */
    float t_sic_llc;      /* LLC MOSFET temperature (degC) */
    float t_magnetics;    /* Transformer core temperature (degC) */
    float t_ambient;      /* Inlet air temperature (degC) */
    float v_grid_a;       /* Grid voltage phase A (V) */
    float v_grid_b;       /* Grid voltage phase B (V) */
    float v_grid_c;       /* Grid voltage phase C (V) */
} ADC_Readings_t;

void                  App_ADC_Init(void);
const ADC_Readings_t *App_ADC_GetReadings(void);

#endif /* APP_ADC_H */
