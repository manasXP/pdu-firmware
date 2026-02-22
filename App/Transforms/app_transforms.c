/**
 * @file    app_transforms.c
 * @brief   Clarke/Park transforms and SVM using CORDIC co-processor
 *
 * CORDIC sin/cos uses direct register access for minimum latency (~50 ns).
 * All functions are designed for ISR context at 65 kHz.
 */

#include "app_transforms.h"
#include "main.h"

/* ------------------------------------------------------------------ */
/*  Constants                                                          */
/* ------------------------------------------------------------------ */

#define PI_F          3.14159265f
#define TWO_PI_F      6.28318531f
#define INV_PI_F      0.31830989f     /* 1 / PI */
#define INV_SQRT3_F   0.57735027f     /* 1 / sqrt(3) */

/**
 * @brief  CORDIC CSR configuration for COSINE function
 *
 * Bits:  FUNC=0 (COSINE), PRECISION=6 (6 iterations, ~20-bit accuracy),
 *        NRES=1 (2 results: cos + sin), NARGS=0 (1 argument),
 *        RESSIZE=0 (32-bit), ARGSIZE=0 (32-bit), RRDY=0
 */
#define CORDIC_CSR_CONFIG  (0U          /* FUNC = COSINE (0x00) */       \
    | (6U << 4U)                        /* PRECISION = 6 cycles */       \
    | (0U << 8U)                        /* SCALE = 0 */                  \
    | (0U << 16U)                       /* IEN = 0 (polling) */          \
    | (0U << 17U)                       /* DMAREN = 0 */                 \
    | (0U << 18U)                       /* DMAWEN = 0 */                 \
    | (0U << 19U)                       /* NRES = 0 (2 results) */       \
    | (0U << 20U)                       /* NARGS = 0 (1 arg) */          \
    | (0U << 21U)                       /* RESSIZE = 0 (32-bit) */       \
    | (0U << 22U))                      /* ARGSIZE = 0 (32-bit) */

/**
 * @brief  Scale factor for float-to-q1.31 conversion
 *
 * CORDIC input range: [-1, +1) maps to [-PI, +PI) radians.
 * Conversion: q31 = (theta - PI) / PI, scaled to 2^31.
 * We precompute 2^31 / PI = 683565276.
 */
#define Q31_SCALE     2147483648.0f    /* 2^31 */

/* ------------------------------------------------------------------ */
/*  CORDIC Init                                                        */
/* ------------------------------------------------------------------ */

void App_Transforms_Init(void)
{
    /* Configure CORDIC CSR for cosine, 6-cycle precision, q1.31 */
    CORDIC->CSR = CORDIC_CSR_CONFIG;
}

/* ------------------------------------------------------------------ */
/*  CORDIC Sin/Cos                                                     */
/* ------------------------------------------------------------------ */

SinCos_t Transforms_CORDIC_SinCos(float theta)
{
    SinCos_t sc;

    /*
     * Normalize theta from [0, 2*PI) to [-1, +1) in q1.31 format.
     * CORDIC expects angle in [-1, +1) representing [-PI, +PI).
     * Map: q31_val = (theta - PI) * (1/PI)  => range [-1, +1)
     * Then scale to q1.31: multiply by 2^31.
     */
    float normalized = (theta - PI_F) * INV_PI_F;    /* [-1, +1) */
    int32_t q31_angle = (int32_t)(normalized * Q31_SCALE);

    /* Write angle to CORDIC — starts computation */
    CORDIC->WDATA = (uint32_t)q31_angle;

    /* Read cos result (first read from COSINE function) */
    int32_t cos_q31 = (int32_t)CORDIC->RDATA;

    /* Read sin result (second read) */
    int32_t sin_q31 = (int32_t)CORDIC->RDATA;

    /* Convert q1.31 back to float: divide by 2^31 */
    sc.cos = (float)cos_q31 * (1.0f / Q31_SCALE);
    sc.sin = (float)sin_q31 * (1.0f / Q31_SCALE);

    return sc;
}

/* ------------------------------------------------------------------ */
/*  Clarke Transform                                                   */
/* ------------------------------------------------------------------ */

AlphaBeta_t Transforms_Clarke(float a, float b)
{
    AlphaBeta_t ab;

    ab.alpha = a;
    ab.beta  = (a + 2.0f * b) * INV_SQRT3_F;

    return ab;
}

/* ------------------------------------------------------------------ */
/*  Park Transform                                                     */
/* ------------------------------------------------------------------ */

DQ_t Transforms_Park(AlphaBeta_t ab, SinCos_t sc)
{
    DQ_t dq;

    dq.d =  ab.alpha * sc.cos + ab.beta * sc.sin;
    dq.q = -ab.alpha * sc.sin + ab.beta * sc.cos;

    return dq;
}

/* ------------------------------------------------------------------ */
/*  Inverse Park Transform                                             */
/* ------------------------------------------------------------------ */

AlphaBeta_t Transforms_InversePark(DQ_t dq, SinCos_t sc)
{
    AlphaBeta_t ab;

    ab.alpha = dq.d * sc.cos - dq.q * sc.sin;
    ab.beta  = dq.d * sc.sin + dq.q * sc.cos;

    return ab;
}

/* ------------------------------------------------------------------ */
/*  Space Vector Modulation (7-segment symmetric)                      */
/* ------------------------------------------------------------------ */

DutyABC_t Transforms_SVM(float v_alpha, float v_beta, float v_dc,
                          float max_duty)
{
    DutyABC_t duties;

    /* Avoid division by zero */
    if (v_dc < 1.0f)
    {
        duties.a = 0.0f;
        duties.b = 0.0f;
        duties.c = 0.0f;
        return duties;
    }

    float inv_vdc = 1.0f / v_dc;

    /*
     * Compute auxiliary variables for sector determination.
     * Using the 60-degree reference frame projections:
     *   X = V_beta
     *   Y = (sqrt(3)/2) * V_alpha - (1/2) * V_beta
     *   Z = -(sqrt(3)/2) * V_alpha - (1/2) * V_beta
     */
    float x = v_beta;
    float y = 0.86602540f * v_alpha - 0.5f * v_beta;     /* sqrt(3)/2 */
    float z = -0.86602540f * v_alpha - 0.5f * v_beta;

    /* Sector determination from signs of X, Y, Z */
    uint8_t sector;
    if (x >= 0.0f)
    {
        if (y >= 0.0f)
        {
            sector = 1U;    /* X >= 0, Y >= 0 → sector 1 */
        }
        else if (z >= 0.0f)
        {
            sector = 5U;    /* X >= 0, Y < 0, Z >= 0 → sector 5 */
        }
        else
        {
            sector = 6U;    /* X >= 0, Y < 0, Z < 0 → sector 6 */
        }
    }
    else
    {
        if (y >= 0.0f)
        {
            if (z >= 0.0f)
            {
                sector = 2U;    /* X < 0, Y >= 0, Z >= 0 → sector 2 */
            }
            else
            {
                sector = 3U;    /* X < 0, Y >= 0, Z < 0 → sector 3 */
            }
        }
        else
        {
            sector = 4U;    /* X < 0, Y < 0 → sector 4 */
        }
    }

    /*
     * Compute T1 and T2 switching times (normalized to PWM period).
     * Uses the sector-dependent formulas with Ts = 1 (normalized).
     * Factor: 2 / sqrt(3) * inv_vdc = 1.1547 * inv_vdc
     */
    float t1;
    float t2;

    switch (sector)
    {
    case 1U:
        t1 =  (0.86602540f * v_alpha - 0.5f * v_beta) * inv_vdc;    /* Y */
        t2 =  v_beta * inv_vdc;                                       /* X */
        break;
    case 2U:
        t1 =  v_beta * inv_vdc;                                       /* X */
        t2 = -(0.86602540f * v_alpha + 0.5f * v_beta) * inv_vdc;     /* -Z */
        break;
    case 3U:
        t1 = -(0.86602540f * v_alpha - 0.5f * v_beta) * inv_vdc;    /* -Y */
        t2 =  (0.86602540f * v_alpha + 0.5f * v_beta) * inv_vdc;    /* Z */
        break;
    case 4U:
        t1 = -v_beta * inv_vdc;                                       /* -X */
        t2 =  (0.86602540f * v_alpha - 0.5f * v_beta) * inv_vdc;    /* Y */
        break;
    case 5U:
        t1 =  (0.86602540f * v_alpha + 0.5f * v_beta) * inv_vdc;    /* Z */
        t2 = -v_beta * inv_vdc;                                       /* -X */
        break;
    default: /* sector 6 */
        t1 = -(0.86602540f * v_alpha + 0.5f * v_beta) * inv_vdc;    /* -Z */
        t2 = -(0.86602540f * v_alpha - 0.5f * v_beta) * inv_vdc;    /* -Y */
        break;
    }

    /* Overmodulation clamp: if T1 + T2 > 1, scale proportionally */
    float t_sum = t1 + t2;
    if (t_sum > 1.0f)
    {
        float scale = 1.0f / t_sum;
        t1 *= scale;
        t2 *= scale;
        t_sum = 1.0f;
    }

    /* Zero-vector time, split equally for symmetric (7-segment) pattern */
    float t0_half = (1.0f - t_sum) * 0.5f;

    /*
     * Per-sector duty assignment (centered PWM pattern).
     * ta, tb, tc are the per-phase on-times normalized to [0, 1].
     */
    float ta;
    float tb;
    float tc;

    switch (sector)
    {
    case 1U:
        ta = t1 + t2 + t0_half;
        tb = t2 + t0_half;
        tc = t0_half;
        break;
    case 2U:
        ta = t1 + t0_half;
        tb = t1 + t2 + t0_half;
        tc = t0_half;
        break;
    case 3U:
        ta = t0_half;
        tb = t1 + t2 + t0_half;
        tc = t2 + t0_half;
        break;
    case 4U:
        ta = t0_half;
        tb = t1 + t0_half;
        tc = t1 + t2 + t0_half;
        break;
    case 5U:
        ta = t2 + t0_half;
        tb = t0_half;
        tc = t1 + t2 + t0_half;
        break;
    default: /* sector 6 */
        ta = t1 + t2 + t0_half;
        tb = t0_half;
        tc = t1 + t0_half;
        break;
    }

    /* Clamp duties to [0, max_duty] */
    if (ta < 0.0f) { ta = 0.0f; }
    if (ta > max_duty) { ta = max_duty; }
    if (tb < 0.0f) { tb = 0.0f; }
    if (tb > max_duty) { tb = max_duty; }
    if (tc < 0.0f) { tc = 0.0f; }
    if (tc > max_duty) { tc = max_duty; }

    duties.a = ta;
    duties.b = tb;
    duties.c = tc;

    return duties;
}
