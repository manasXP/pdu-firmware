/**
 * @file    app_control.h
 * @brief   PFC dq-frame current control, LLC PFM, PI controllers
 */

#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include <stdint.h>

/* PI controller instance index */
typedef enum
{
    PI_ID_ID = 0,   /* PFC d-axis current */
    PI_ID_IQ,       /* PFC q-axis current */
    PI_ID_VBUS,     /* PFC bus voltage outer loop */
    PI_ID_VLLC,     /* LLC output voltage */
    PI_ID_ILLC,     /* LLC output current */
    PI_ID_COUNT
} PI_Index_t;

/* LLC sweep mode — selects which phase(s) to drive */
typedef enum
{
    SWEEP_MODE_ALL = 0,    /* All 3 phases (D + E + F) */
    SWEEP_MODE_PHASE_D,    /* Timer D only */
    SWEEP_MODE_PHASE_E,    /* Timer E only */
    SWEEP_MODE_PHASE_F     /* Timer F only */
} LLC_SweepMode_t;

/* Per-phase ZVS analysis result */
typedef struct
{
    uint32_t zvs_boundary_hz;  /* Frequency where ZVS boundary detected */
    int32_t  zvs_margin_hz;    /* Margin above parallel resonant freq   */
    uint8_t  zvs_lost;         /* 1 if sweep went below ZVS boundary    */
    uint8_t  valid;            /* 1 if analysis produced a result       */
} ZVS_PhaseResult_t;

/* Aggregate ZVS result for all phases */
typedef struct
{
    ZVS_PhaseResult_t phase[3];        /* D=0, E=1, F=2 */
    uint32_t          estimated_fr_hz; /* Estimated resonant frequency  */
} ZVS_Result_t;

/* PI controller instance */
typedef struct
{
    float Kp;
    float Ki;
    float Tt;          /* Anti-windup tracking time constant */
    float out_min;
    float out_max;
    float integrator;
    float prev_error;
} PI_Controller_t;

void  App_Control_Init(void);
float PI_Update(PI_Controller_t *pi, float setpoint, float measurement, float dt);
void  PI_Reset(PI_Controller_t *pi);

/**
 * @brief  Set PI gains at runtime; auto-computes Tt = sqrt(Kp / Ki)
 * @param  idx      Controller index (PI_ID_ID .. PI_ID_ILLC)
 * @param  Kp       Proportional gain (must be > 0)
 * @param  Ki       Integral gain (must be > 0)
 * @param  out_min  Output saturation minimum
 * @param  out_max  Output saturation maximum
 */
void PI_SetGains(PI_Index_t idx, float Kp, float Ki,
                 float out_min, float out_max);

/**
 * @brief  Get read-only pointer to a PI controller instance
 * @param  idx  Controller index
 * @return Pointer to controller, or NULL if idx is out of range
 */
const PI_Controller_t *PI_GetController(PI_Index_t idx);

/**
 * @brief  Get human-readable name for a PI controller index
 * @param  idx  Controller index
 * @return Static string name (e.g. "id", "iq", "vbus", "vllc", "illc")
 */
const char *PI_GetName(PI_Index_t idx);

/* Bus voltage outer loop (1 kHz, called from state machine) */
void  App_Control_VBus_Update(void);
void  App_Control_VBus_Enable(void);
void  App_Control_VBus_Disable(void);
float App_Control_VBus_GetIdRef(void);
void  App_Control_VBus_PreloadIntegrator(float value);

/* PFC HRTIM configuration and control */
void App_Control_HRTIM_PFC_Init(void);
void App_Control_PFC_Start(void);
void App_Control_PFC_Stop(void);
void App_Control_PFC_SetDuty(float duty);
void App_Control_PFC_SetDutyABC(float da, float db, float dc);
void App_Control_PFC_ISR(void);

/* PFC closed-loop current control */
void  App_Control_PFC_SetIdRef(float id_ref_a);
void  App_Control_PFC_EnableClosedLoop(void);
void  App_Control_PFC_DisableClosedLoop(void);

/* LLC HRTIM configuration and control */
void    App_Control_HRTIM_LLC_Init(void);
void    App_Control_LLC_Start(void);
void    App_Control_LLC_Stop(void);
void    App_Control_LLC_SetFrequency(uint32_t freq_hz);
void    App_Control_LLC_StartPhase(uint32_t phase);
void    App_Control_LLC_SweepStart(void);
void    App_Control_LLC_SweepStartEx(LLC_SweepMode_t mode);
void    App_Control_LLC_Sweep(void);
uint8_t App_Control_LLC_SweepDone(void);
void    App_Control_LLC_ISR(void);

/* ZVS boundary detection */
const ZVS_Result_t *App_Control_LLC_GetZVSResult(void);

/* LLC PI integrator freeze/unfreeze for burst mode idle */
void App_Control_LLC_PI_Freeze(void);
void App_Control_LLC_PI_Unfreeze(void);
void App_Control_LLC_PI_DecayIdle(float factor);

/* HRTIM burst mode controller configuration */
void App_Control_HRTIM_BurstMode_Config(float f_burst, float duty_burst);
void App_Control_HRTIM_BurstMode_Enable(void);
void App_Control_HRTIM_BurstMode_Disable(void);
void App_Control_HRTIM_BurstMode_SWTrigger(void);

#endif /* APP_CONTROL_H */
