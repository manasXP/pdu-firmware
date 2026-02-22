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
    /* Fast path — updated from injected ISRs at 48-65 kHz */
    float v_bus;          /* DC bus voltage (V)                */
    float v_out;          /* LLC output voltage (V)            */
    float i_phase_a;      /* PFC phase A current (A)           */
    float i_phase_b;      /* PFC phase B current (A)           */
    float i_phase_c;      /* PFC phase C current (A)           */
    float i_out;          /* LLC output current (A)            */
    float v_grid_a;       /* Grid voltage phase A (V)          */
    float v_grid_b;       /* Grid voltage phase B (V)          */
    float v_grid_c;       /* Grid voltage phase C (V)          */

    /* Slow path — updated from regular+DMA at 1 kHz */
    float t_sic_pfc;      /* PFC MOSFET temperature (degC)     */
    float t_sic_llc;      /* LLC MOSFET temperature (degC)     */
    float t_magnetics;    /* Transformer core temperature (degC) */
    float t_ambient;      /* Inlet air temperature (degC)      */
    float v_cap_top;      /* Split-cap top voltage (V)         */
    float v_cap_bot;      /* Split-cap bottom voltage (V)      */
    float v_aux_12v;      /* 12 V auxiliary rail (V)           */
} ADC_Readings_t;

void                  App_ADC_Init(void);
const ADC_Readings_t *App_ADC_GetReadings(void);

/* Start HRTIM-triggered injected conversions on all 5 ADCs */
void App_ADC_StartInjected(void);

/* Process regular (slow) group — call from main loop at 1 kHz */
void App_ADC_RegularProcess(void);

/* Read injected results — called from HRTIM period ISRs */
void App_ADC_ReadInjected_PFC(void);
void App_ADC_ReadInjected_LLC(void);

#endif /* APP_ADC_H */
