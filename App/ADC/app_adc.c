/**
 * @file    app_adc.c
 * @brief   ADC pipeline — injected (HRTIM-triggered), regular (DMA), filters
 *
 * Two conversion paths:
 *  - Injected (fast): HRTIM-triggered at 48-65 kHz for PFC/LLC control.
 *    Results read directly from JDRx registers in ISR context.
 *  - Regular (slow): Software-triggered at 1 kHz via DMA for temperatures,
 *    split-cap voltages, and auxiliary rail monitoring.
 *
 * ES0430 errata guard: regular conversions are not started while an
 * injected sequence is in progress (JADSTART check).
 */

#include "app_adc.h"
#include "main.h"

/* ------------------------------------------------------------------ */
/*  Biquad filter (Direct Form II Transposed)                          */
/* ------------------------------------------------------------------ */

typedef struct
{
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float d1;    /* delay element 1 */
    float d2;    /* delay element 2 */
} Biquad_t;

/**
 * @brief  Update biquad filter with new sample
 * @param  f   Pointer to biquad state
 * @param  x   New input sample
 * @return Filtered output
 */
static inline float biquad_update(Biquad_t *f, float x)
{
    float y = (f->b0 * x) + f->d1;
    f->d1   = (f->b1 * x) - (f->a1 * y) + f->d2;
    f->d2   = (f->b2 * x) - (f->a2 * y);
    return y;
}

/**
 * @brief  Exponential moving average update
 * @param  prev  Previous filtered value
 * @param  x     New sample
 * @param  alpha Smoothing factor (0..1, smaller = more filtering)
 * @return Updated filtered value
 */
static inline float ema_update(float prev, float x, float alpha)
{
    return prev + alpha * (x - prev);
}

/**
 * @brief  NTC thermistor ADC-to-degC conversion (placeholder)
 * @param  raw  12-bit ADC code
 * @return Temperature in degrees Celsius
 *
 * Linear approximation — replace with Steinhart-Hart or lookup table
 * once NTC parameters are characterized.
 */
static float ntc_adc_to_degc(uint16_t raw)
{
    /* Placeholder linear mapping: 0 counts → -40 degC, 4095 → 150 degC */
    return -40.0f + ((float)raw * (190.0f / 4095.0f));
}

/* ------------------------------------------------------------------ */
/*  Module state                                                       */
/* ------------------------------------------------------------------ */

static ADC_Readings_t s_readings;

/*
 * Biquad for V_bus — 2nd-order Butterworth low-pass
 * Designed for Fs = 65 kHz, Fc = 3 kHz (120 Hz ripple rejection)
 * Coefficients: b0=0.0675, b1=0.1349, b2=0.0675, a1=-1.143, a2=0.4128
 */
static Biquad_t s_biquad_vbus = {
    .b0 = 0.0675f,
    .b1 = 0.1349f,
    .b2 = 0.0675f,
    .a1 = -1.143f,
    .a2 = 0.4128f,
    .d1 = 0.0f,
    .d2 = 0.0f,
};

/* 1st-order IIR smoothing factor for V_out (LLC ISR path) */
#define IIR_ALPHA_VOUT  0.3f

/* EMA smoothing factor for temperature channels */
#define EMA_ALPHA_TEMP  0.01f

/* ------------------------------------------------------------------ */
/*  DMA handles and buffers                                            */
/* ------------------------------------------------------------------ */

DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;
DMA_HandleTypeDef hdma_adc4;
DMA_HandleTypeDef hdma_adc5;

/*
 * Regular group channel counts per ADC (slow path):
 *   ADC1: T_SiC_PFC                    (1 ch)
 *   ADC2: T_SiC_LLC                    (1 ch)
 *   ADC3: T_magnetics, T_ambient       (2 ch)
 *   ADC4: V_cap_top, V_cap_bot         (2 ch)
 *   ADC5: V_aux_12v                    (1 ch)
 */
#define ADC1_REG_CH_COUNT  1U
#define ADC2_REG_CH_COUNT  1U
#define ADC3_REG_CH_COUNT  2U
#define ADC4_REG_CH_COUNT  2U
#define ADC5_REG_CH_COUNT  1U

static uint16_t s_dma_buf_adc1[ADC1_REG_CH_COUNT] __attribute__((aligned(4)));
static uint16_t s_dma_buf_adc2[ADC2_REG_CH_COUNT] __attribute__((aligned(4)));
static uint16_t s_dma_buf_adc3[ADC3_REG_CH_COUNT] __attribute__((aligned(4)));
static uint16_t s_dma_buf_adc4[ADC4_REG_CH_COUNT] __attribute__((aligned(4)));
static uint16_t s_dma_buf_adc5[ADC5_REG_CH_COUNT] __attribute__((aligned(4)));

/* DMA completion flags — set in HAL callback, cleared after processing */
static volatile uint8_t s_dma_done_adc1;
static volatile uint8_t s_dma_done_adc2;
static volatile uint8_t s_dma_done_adc3;
static volatile uint8_t s_dma_done_adc4;
static volatile uint8_t s_dma_done_adc5;

/* ------------------------------------------------------------------ */
/*  DMA helper                                                         */
/* ------------------------------------------------------------------ */

/**
 * @brief  Configure a DMA channel for ADC peripheral-to-memory transfer
 * @param  hdma       DMA handle to configure
 * @param  hadc       ADC handle to link
 * @param  instance   DMA channel instance (e.g. DMA1_Channel1)
 * @param  request    DMAMUX request ID
 */
static void dma_channel_init(DMA_HandleTypeDef *hdma,
                             ADC_HandleTypeDef *hadc,
                             DMA_Channel_TypeDef *instance,
                             uint32_t request)
{
    hdma->Instance                 = instance;
    hdma->Init.Request             = request;
    hdma->Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma->Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma->Init.MemInc              = DMA_MINC_ENABLE;
    hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma->Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma->Init.Mode                = DMA_NORMAL;
    hdma->Init.Priority            = DMA_PRIORITY_LOW;

    if (HAL_DMA_Init(hdma) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_LINKDMA(hadc, DMA_Handle, *hdma);
}

/* ------------------------------------------------------------------ */
/*  Regular channel configuration helper                               */
/* ------------------------------------------------------------------ */

/**
 * @brief  Configure one regular-group channel on an ADC
 * @param  hadc     ADC handle
 * @param  channel  ADC_CHANNEL_x
 * @param  rank     Sequence rank (1-based)
 */
static void adc_config_regular_channel(ADC_HandleTypeDef *hadc,
                                       uint32_t channel,
                                       uint32_t rank)
{
    ADC_ChannelConfTypeDef cfg = {0};

    cfg.Channel      = channel;
    cfg.Rank         = rank;
    cfg.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;  /* Slow path — maximize SNR */
    cfg.SingleDiff   = ADC_SINGLE_ENDED;
    cfg.OffsetNumber = ADC_OFFSET_NONE;
    cfg.Offset       = 0U;

    if (HAL_ADC_ConfigChannel(hadc, &cfg) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ------------------------------------------------------------------ */
/*  Hardware oversampling helper                                       */
/* ------------------------------------------------------------------ */

/**
 * @brief  Enable 16x hardware oversampling on regular group
 * @param  hadc  ADC handle (must be stopped)
 *
 * Configures CFGR2: OVSR = 0b0011 (16x), OVSS = 0b0100 (>> 4),
 * ROVSE = 1 (regular oversampling enable).
 * Net effect: 16 conversions averaged, result still 12-bit.
 */
static void adc_enable_hw_oversampling(ADC_HandleTypeDef *hadc)
{
    /* Disable ADC to modify CFGR2 */
    if (HAL_ADC_Stop(hadc) != HAL_OK)
    {
        /* ADC may not be running yet — ignore */
    }

    ADC_TypeDef *adc = hadc->Instance;

    /* Clear oversampling fields */
    adc->CFGR2 &= ~(ADC_CFGR2_OVSR | ADC_CFGR2_OVSS | ADC_CFGR2_ROVSE);

    /* OVSR = 0b0011 (16x), OVSS = 0b0100 (shift right 4), ROVSE = 1 */
    adc->CFGR2 |= (3U << ADC_CFGR2_OVSR_Pos)
                 | (4U << ADC_CFGR2_OVSS_Pos)
                 | ADC_CFGR2_ROVSE;
}

/* ------------------------------------------------------------------ */
/*  Init                                                               */
/* ------------------------------------------------------------------ */

void App_ADC_Init(void)
{
    /*
     * DMA channel assignments (STM32G474 DMAMUX):
     *   ADC1 → DMA1_CH1  (request 5)
     *   ADC2 → DMA1_CH2  (request 36)
     *   ADC3 → DMA2_CH1  (request 37)
     *   ADC4 → DMA2_CH2  (request 38)
     *   ADC5 → DMA2_CH3  (request 39)
     */
    dma_channel_init(&hdma_adc1, &hadc1, DMA1_Channel1, DMA_REQUEST_ADC1);
    dma_channel_init(&hdma_adc2, &hadc2, DMA1_Channel2, DMA_REQUEST_ADC2);
    dma_channel_init(&hdma_adc3, &hadc3, DMA2_Channel1, DMA_REQUEST_ADC3);
    dma_channel_init(&hdma_adc4, &hadc4, DMA2_Channel2, DMA_REQUEST_ADC4);
    dma_channel_init(&hdma_adc5, &hadc5, DMA2_Channel3, DMA_REQUEST_ADC5);

    /*
     * Configure regular-group channels for slow-path monitoring.
     * Channel assignments are placeholders — finalize from schematic.
     */

    /* ADC1 regular: T_SiC_PFC */
    hadc1.Init.NbrOfConversion = ADC1_REG_CH_COUNT;
    adc_config_regular_channel(&hadc1, ADC_CHANNEL_3, ADC_REGULAR_RANK_1);

    /* ADC2 regular: T_SiC_LLC */
    hadc2.Init.NbrOfConversion = ADC2_REG_CH_COUNT;
    adc_config_regular_channel(&hadc2, ADC_CHANNEL_5, ADC_REGULAR_RANK_1);

    /* ADC3 regular: T_magnetics, T_ambient */
    hadc3.Init.NbrOfConversion = ADC3_REG_CH_COUNT;
    adc_config_regular_channel(&hadc3, ADC_CHANNEL_3, ADC_REGULAR_RANK_1);
    adc_config_regular_channel(&hadc3, ADC_CHANNEL_4, ADC_REGULAR_RANK_2);

    /* ADC4 regular: V_cap_top, V_cap_bot */
    hadc4.Init.NbrOfConversion = ADC4_REG_CH_COUNT;
    adc_config_regular_channel(&hadc4, ADC_CHANNEL_5, ADC_REGULAR_RANK_1);
    adc_config_regular_channel(&hadc4, ADC_CHANNEL_6, ADC_REGULAR_RANK_2);

    /* ADC5 regular: V_aux_12v */
    hadc5.Init.NbrOfConversion = ADC5_REG_CH_COUNT;
    adc_config_regular_channel(&hadc5, ADC_CHANNEL_2, ADC_REGULAR_RANK_1);

    /* Enable 16x hardware oversampling on all regular groups */
    adc_enable_hw_oversampling(&hadc1);
    adc_enable_hw_oversampling(&hadc2);
    adc_enable_hw_oversampling(&hadc3);
    adc_enable_hw_oversampling(&hadc4);
    adc_enable_hw_oversampling(&hadc5);
}

/* ------------------------------------------------------------------ */
/*  Injected path — called from HRTIM period ISRs                      */
/* ------------------------------------------------------------------ */

void App_ADC_StartInjected(void)
{
    HAL_ADCEx_InjectedStart(&hadc1);
    HAL_ADCEx_InjectedStart(&hadc2);
    HAL_ADCEx_InjectedStart(&hadc3);
    HAL_ADCEx_InjectedStart(&hadc4);
    HAL_ADCEx_InjectedStart(&hadc5);
}

/**
 * @brief  Read PFC injected results — called from PFC HRTIM ISR (~65 kHz)
 *
 * ADC1 INJ: rank1 = I_A, rank2 = V_bus
 * ADC2 INJ: rank1 = I_B, rank2 = V_grid_A
 * ADC3 INJ: rank1 = I_C, rank2 = V_grid_B
 */
void App_ADC_ReadInjected_PFC(void)
{
    /* Direct JDRx register reads — fastest path, no HAL overhead */
    uint32_t adc1_jdr1 = hadc1.Instance->JDR1;
    uint32_t adc1_jdr2 = hadc1.Instance->JDR2;
    uint32_t adc2_jdr1 = hadc2.Instance->JDR1;
    uint32_t adc2_jdr2 = hadc2.Instance->JDR2;
    uint32_t adc3_jdr1 = hadc3.Instance->JDR1;
    uint32_t adc3_jdr2 = hadc3.Instance->JDR2;

    /* Scale to engineering units */
    s_readings.i_phase_a = ADC_TO_IPHASE(adc1_jdr1);
    s_readings.i_phase_b = ADC_TO_IPHASE(adc2_jdr1);
    s_readings.i_phase_c = ADC_TO_IPHASE(adc3_jdr1);

    /* V_bus with biquad filtering (120 Hz ripple rejection) */
    float v_bus_raw = ADC_TO_VBUS(adc1_jdr2);
    s_readings.v_bus = biquad_update(&s_biquad_vbus, v_bus_raw);

    /* Grid voltages — unfiltered (used by PLL) */
    s_readings.v_grid_a = ADC_TO_VGRID(adc2_jdr2);
    s_readings.v_grid_b = ADC_TO_VGRID(adc3_jdr2);
}

/**
 * @brief  Read LLC injected results — called from LLC HRTIM ISR (~150 kHz)
 *
 * ADC4 INJ: rank1 = V_grid_C, rank2 = V_out
 * ADC5 INJ: rank1 = I_out
 */
void App_ADC_ReadInjected_LLC(void)
{
    uint32_t adc4_jdr1 = hadc4.Instance->JDR1;
    uint32_t adc4_jdr2 = hadc4.Instance->JDR2;
    uint32_t adc5_jdr1 = hadc5.Instance->JDR1;

    s_readings.v_grid_c = ADC_TO_VGRID(adc4_jdr1);

    /* V_out with 1st-order IIR smoothing */
    float v_out_raw = ADC_TO_VOUT(adc4_jdr2);
    s_readings.v_out = s_readings.v_out
                     + IIR_ALPHA_VOUT * (v_out_raw - s_readings.v_out);

    s_readings.i_out = ADC_TO_IOUT(adc5_jdr1);
}

/* ------------------------------------------------------------------ */
/*  Regular path — DMA callbacks                                       */
/* ------------------------------------------------------------------ */

/**
 * @brief  HAL DMA transfer-complete callback
 *
 * Called by HAL_DMA_IRQHandler when a regular-group DMA transfer finishes.
 * Sets the per-ADC completion flag for processing in main loop context.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        s_dma_done_adc1 = 1U;
    }
    else if (hadc->Instance == ADC2)
    {
        s_dma_done_adc2 = 1U;
    }
    else if (hadc->Instance == ADC3)
    {
        s_dma_done_adc3 = 1U;
    }
    else if (hadc->Instance == ADC4)
    {
        s_dma_done_adc4 = 1U;
    }
    else if (hadc->Instance == ADC5)
    {
        s_dma_done_adc5 = 1U;
    }
    else
    {
        /* Unexpected ADC instance — do nothing */
    }
}

/* ------------------------------------------------------------------ */
/*  Regular path — 1 kHz processing (main loop context)                */
/* ------------------------------------------------------------------ */

void App_ADC_RegularProcess(void)
{
    /*
     * ES0430 errata guard: do not start regular conversions while an
     * injected sequence is in progress.  Check JADSTART on all ADCs.
     */
    if ((hadc1.Instance->CR & ADC_CR_JADSTART) != 0U)
    {
        return;
    }
    if ((hadc2.Instance->CR & ADC_CR_JADSTART) != 0U)
    {
        return;
    }
    if ((hadc3.Instance->CR & ADC_CR_JADSTART) != 0U)
    {
        return;
    }
    if ((hadc4.Instance->CR & ADC_CR_JADSTART) != 0U)
    {
        return;
    }
    if ((hadc5.Instance->CR & ADC_CR_JADSTART) != 0U)
    {
        return;
    }

    /* Start DMA transfers for regular groups */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_dma_buf_adc1, ADC1_REG_CH_COUNT);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)s_dma_buf_adc2, ADC2_REG_CH_COUNT);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t *)s_dma_buf_adc3, ADC3_REG_CH_COUNT);
    HAL_ADC_Start_DMA(&hadc4, (uint32_t *)s_dma_buf_adc4, ADC4_REG_CH_COUNT);
    HAL_ADC_Start_DMA(&hadc5, (uint32_t *)s_dma_buf_adc5, ADC5_REG_CH_COUNT);

    /*
     * Process completed DMA transfers from previous cycle.
     * Non-circular mode: each flag means the full sequence completed.
     */

    /* ADC1 regular: T_SiC_PFC */
    if (s_dma_done_adc1 != 0U)
    {
        s_dma_done_adc1 = 0U;
        float t_raw = ntc_adc_to_degc(s_dma_buf_adc1[0]);
        s_readings.t_sic_pfc = ema_update(s_readings.t_sic_pfc, t_raw,
                                          EMA_ALPHA_TEMP);
    }

    /* ADC2 regular: T_SiC_LLC */
    if (s_dma_done_adc2 != 0U)
    {
        s_dma_done_adc2 = 0U;
        float t_raw = ntc_adc_to_degc(s_dma_buf_adc2[0]);
        s_readings.t_sic_llc = ema_update(s_readings.t_sic_llc, t_raw,
                                          EMA_ALPHA_TEMP);
    }

    /* ADC3 regular: T_magnetics, T_ambient */
    if (s_dma_done_adc3 != 0U)
    {
        s_dma_done_adc3 = 0U;
        float t_mag = ntc_adc_to_degc(s_dma_buf_adc3[0]);
        float t_amb = ntc_adc_to_degc(s_dma_buf_adc3[1]);
        s_readings.t_magnetics = ema_update(s_readings.t_magnetics, t_mag,
                                            EMA_ALPHA_TEMP);
        s_readings.t_ambient = ema_update(s_readings.t_ambient, t_amb,
                                          EMA_ALPHA_TEMP);
    }

    /* ADC4 regular: V_cap_top, V_cap_bot */
    if (s_dma_done_adc4 != 0U)
    {
        s_dma_done_adc4 = 0U;
        s_readings.v_cap_top = ADC_TO_VCAP(s_dma_buf_adc4[0]);
        s_readings.v_cap_bot = ADC_TO_VCAP(s_dma_buf_adc4[1]);
    }

    /* ADC5 regular: V_aux_12v */
    if (s_dma_done_adc5 != 0U)
    {
        s_dma_done_adc5 = 0U;
        s_readings.v_aux_12v = ADC_TO_VAUX12V(s_dma_buf_adc5[0]);
    }
}

/* ------------------------------------------------------------------ */
/*  Accessor                                                           */
/* ------------------------------------------------------------------ */

const ADC_Readings_t *App_ADC_GetReadings(void)
{
    return &s_readings;
}
