/**
  ******************************************************************************
  * @file    fixpmath.c
  * @author  Piak Electronic Design B.V.
  * @brief   This file is part of the Motor Control SDK.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 Piak Electronic Design B.V.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* fixpmath.c */

#include "fixpmath.h"

#include <math.h> /* fabsf() */

#include "mathlib.h"
#include "mc_stm_types.h" /* Required for CORDIC */

#if defined(CORDIC)
#define FIXPMATH_USE_CORDIC
#endif

#ifdef FIXPMATH_USE_CORDIC
// Number of cycles can be tuned as a compromise between calculation time and precision
#define CORDIC_CONFIG_BASE_COSSIN    (LL_CORDIC_PRECISION_6CYCLES | LL_CORDIC_SCALE_0 | LL_CORDIC_NBWRITE_2 |\
				 LL_CORDIC_INSIZE_32BITS | LL_CORDIC_OUTSIZE_32BITS)

/* CORDIC FUNCTION: COSINE only q1.31 */
#define CORDIC_CONFIG_COSINE    (CORDIC_CONFIG_BASE_COSSIN | LL_CORDIC_FUNCTION_COSINE)

/* CORDIC FUNCTION: SINE only q1.31 */
#define CORDIC_CONFIG_SINE		(CORDIC_CONFIG_BASE_COSSIN | LL_CORDIC_FUNCTION_SINE)

/* CORDIC FUNCTION: COSINE and SINE q1.31 */
#define CORDIC_CONFIG_COSINE_AND_SINE	(CORDIC_CONFIG_BASE_COSSIN | LL_CORDIC_FUNCTION_COSINE | LL_CORDIC_NBREAD_2)

/* CORDIC FUNCTION: PHASE q1.31 (Angle and magnitude computation) */
#define CORDIC_CONFIG_PHASE     (LL_CORDIC_FUNCTION_PHASE | LL_CORDIC_PRECISION_15CYCLES | LL_CORDIC_SCALE_0 |\
				 LL_CORDIC_NBWRITE_2 | LL_CORDIC_NBREAD_2 |\
				 LL_CORDIC_INSIZE_32BITS | LL_CORDIC_OUTSIZE_32BITS)

/* CORDIC FUNCTION: SQUAREROOT q1.31 */
#define CORDIC_CONFIG_SQRT      (LL_CORDIC_FUNCTION_SQUAREROOT | LL_CORDIC_PRECISION_3CYCLES | LL_CORDIC_SCALE_1 |\
				 LL_CORDIC_NBWRITE_1 | LL_CORDIC_NBREAD_1 |\
				 LL_CORDIC_INSIZE_32BITS | LL_CORDIC_OUTSIZE_32BITS)

// ToDo: Use LL_CORDIC_FUNCTION_HCOSINE to calculate EXP

#define ANGLE_CORDIC_TO_PU(angle_cordic) (angle_cordic >> 2) & (FIXP30(1.0f)-1) /* scale and wrap cordic angle to per unit */
#define ANGLE_PU_TO_CORDIC(angle_pu) (angle_pu << 2)
#define ANGLE_FP24_PU_TO_CORDIC(angle_pu) (angle_pu << 8)
#endif /* FIXPMATH_USE_CORDIC */

/* fixptable is a table of pre-calculated powers of 2 as float value */
const float FIXPSCALED_fixptable[] =
{
    (float) (1UL << 0),   /* 1.0f, which is the maximum value which will fit in an fixp30_t */
    (float) (1UL << 1),
    (float) (1UL << 2),
    (float) (1UL << 3),
    (float) (1UL << 4),
    (float) (1UL << 5),
    (float) (1UL << 6),
    (float) (1UL << 7),   /* i = 7, fixptable[i] == 128.0f, fixpFmt = 31 - 7 = 24 */
    (float) (1UL << 8),
    (float) (1UL << 9),
    (float) (1UL << 10),
    (float) (1UL << 11),
    (float) (1UL << 12),
    (float) (1UL << 13),
    (float) (1UL << 14),
    (float) (1UL << 15),
    (float) (1UL << 16),
    (float) (1UL << 17),
    (float) (1UL << 18),
    (float) (1UL << 19),
    (float) (1UL << 20),
    (float) (1UL << 21),
    (float) (1UL << 22),
    (float) (1UL << 23),
    (float) (1UL << 24),
    (float) (1UL << 25),
    (float) (1UL << 26),
    (float) (1UL << 27),
    (float) (1UL << 28),
    (float) (1UL << 29),
    (float) (1UL << 30),   /* 1073741824.0f, which is the maximum value which will fit in an fixp1_t */
    (float) (1UL << 31),   /* 2147483648.0f, which is the maximum value which will fit in an fixp0_t */
    (float) (1ULL << 32),
    (float) (1ULL << 33),
    (float) (1ULL << 34),
    (float) (1ULL << 35),
    (float) (1ULL << 36),
    (float) (1ULL << 37),
    (float) (1ULL << 38),
    (float) (1ULL << 39),
    (float) (1ULL << 40),
    (float) (1ULL << 41),
    (float) (1ULL << 42),
    (float) (1ULL << 43),
};

void FIXPMATH_init(void)
{
	MATHLIB_init();
}

void FIXPSCALED_floatToFIXPscaled(const float value, FIXP_scaled_t *pFps)
{
    int i;
    fixpFmt_t fixpFmt = 0;   /* use fixp0_t if number will not fit at all */

    /* the absolute value is used for comparisons, but the actual value for the final calculation */
    float absvalue = fabsf(value);

    /* Figure out which scale will fit the float provided */
    for (i = 0; i <= 31; i++)
    {
        if (FIXPSCALED_fixptable[i] > absvalue) /* check if it will fit */
        {
            /* the qFmt for the result */
        	fixpFmt = 31 - i;
            break;
        }
        /* We either find a fit, or use _iq0 by default */
    }

    pFps->fixpFmt = fixpFmt;
    pFps->value = (long) (value * FIXPSCALED_fixptable[fixpFmt]); /* may be negative */
} /* end of FIXPSCALED_floatToFIXPscaled() function */

void FIXPSCALED_floatToFIXPscaled_exp(const float value, FIXP_scaled_t *pFps, fixpFmt_t exponent)
{
    pFps->fixpFmt = exponent;
    pFps->value = (long) (value * FIXPSCALED_fixptable[exponent]); /* may be negative */
} /* end of FIXPSCALED_floatToFIXPscaled_exp() function */

float_t FIXPSCALED_FIXPscaledToFloat(const FIXP_scaled_t *pFps)
{
	return (float) ((float) pFps->value / FIXPSCALED_fixptable[pFps->fixpFmt]);
} /* end of FIXPSCALED_FIXPscaledToFloat() function */

void FIXPSCALED_doubleToFIXPscaled(const double value, FIXP_scaled_t *pFps)
{
    int i;
    fixpFmt_t fixpFmt = 0;   /* use fixp0_t if number will not fit at all */

    /* the absolute value is used for comparisons, but the actual value for the final calculation */
    double absvalue = fabs(value);

    /* Figure out which scale will fit the float provided */
    for (i = 0; i <= 31; i++)
    {
        if ((double)FIXPSCALED_fixptable[i] > absvalue) /* check if it will fit */
        {
            /* the qFmt for the result */
        	fixpFmt = 31 - i;
            break;
        }
        /* We either find a fit, or use fixp0_t by default */
    }

    pFps->fixpFmt = fixpFmt;
    pFps->value = (long) (value * (double)FIXPSCALED_fixptable[fixpFmt]); /* may be negative */
} /* end of FIXPSCALED_floatToFIXPscaled() function */

void FIXPSCALED_calculateScaleFactor(
    const fixpFmt_t source_fixpFmt,
    const fixpFmt_t target_fixpFmt,
    const float source_fullscale,
    const float target_fullscale,
    const float datascale_factor,
	FIXP_scaled_t* pFps)
{
    /* fixed point scaling factor */
    /* Calculated using bitshifts, to avoid using the pow function */
    float fixpFmt_factor;
    int_least8_t lshift = target_fixpFmt - source_fixpFmt;
    if (lshift >= 0)
    {
        /* Positive shift, giving 2^n */
        fixpFmt_factor = (float) (1ul << lshift);
    }
    else
    {
        /* Negative shift, where we need to divide to calculate the corresponding factor 1/2^abs(n) */
        fixpFmt_factor = (float) (1.0f / (1ul << (-lshift)));
    }

    /* full scale scaling factor */
    float fullscale_factor = source_fullscale / target_fullscale;

    /* data scaling factor */
    /* as given */

    /* total scaling factor is the product of these three factors */
    float ds = fixpFmt_factor * fullscale_factor * datascale_factor;

    /* Convert float to fixed point in optimal scale */
    FIXPSCALED_floatToFIXPscaled(ds, pFps);
} /* end of FIXPSCALED_calculateScaleFactor() function */

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
void FIXP30_CosSinPU(fixp30_t angle_pu, FIXP_CosSin_t *pCosSin)
{
#if defined(FIXPMATH_USE_CORDIC)
	__disable_irq();
	WRITE_REG(CORDIC->CSR, CORDIC_CONFIG_COSINE_AND_SINE);
	LL_CORDIC_WriteData(CORDIC, ANGLE_PU_TO_CORDIC(angle_pu));
	LL_CORDIC_WriteData(CORDIC, FIXP30(1.0f));
	while (HAL_IS_BIT_CLR(CORDIC->CSR, CORDIC_CSR_RRDY)); /* Wait for result */
	pCosSin->cos = LL_CORDIC_ReadData(CORDIC);
	pCosSin->sin = LL_CORDIC_ReadData(CORDIC);
	__enable_irq();
#else /* FIXPMATH_USE_CORDIC */
    angle_pu = (angle_pu & 0x3FFFFFFFul) >> (30-15) ; /* Wrap the angle by ANDing */
    Vector_cossin_s cossin = MATHLIB_cossin(angle_pu);
    pCosSin->cos = cossin.cos << (30-15);
    pCosSin->sin = cossin.sin << (30-15);
#endif /* FIXPMATH_USE_CORDIC */
}
#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
void FIXP30_polar(const fixp30_t x, const fixp30_t y, fixp30_t *pAngle_pu, fixp30_t *pMagnitude)
{
#ifdef FIXPMATH_USE_CORDIC
	__disable_irq();
	WRITE_REG(CORDIC->CSR, CORDIC_CONFIG_PHASE);
	LL_CORDIC_WriteData(CORDIC, x);
	LL_CORDIC_WriteData(CORDIC, y);
	while (HAL_IS_BIT_CLR(CORDIC->CSR, CORDIC_CSR_RRDY)); /* Wait for result */
	*pAngle_pu = ANGLE_CORDIC_TO_PU(LL_CORDIC_ReadData(CORDIC));
	*pMagnitude = LL_CORDIC_ReadData(CORDIC);
	__enable_irq();
#else
	MATHLIB_polar(x, y, pAngle_pu, pMagnitude);
#endif
}

fixp_t FIXP_mag(const fixp_t a, const fixp_t b)
{
	fixp_t dummy;
	fixp_t magnitude;
	FIXP30_polar(a, b, &dummy, &magnitude);
	return magnitude;
}

fixp30_t FIXP30_mag(const fixp30_t a, const fixp30_t b)
{
	fixp_t dummy;
	fixp_t magnitude;
	FIXP30_polar(a, b, &dummy, &magnitude);
	return magnitude;
}

fixp30_t FIXP30_atan2_PU(fixp30_t beta, fixp30_t alpha)
{
	fixp30_t angle_pu;
	fixp30_t dummy;
	FIXP30_polar(alpha, beta, &angle_pu, &dummy);
	return angle_pu;
}

fixp24_t FIXP24_atan2_PU(fixp24_t beta, fixp24_t alpha)
{
	fixp24_t angle_pu;
	fixp24_t dummy;
	FIXP30_polar(alpha, beta, &angle_pu, &dummy);
	return (angle_pu >> (30 - FIXP_FMT));
}

fixp29_t FIXP29_atan2_PU(fixp30_t beta, fixp30_t alpha)
{
	fixp29_t angle_pu;
	fixp29_t dummy;
	FIXP30_polar(alpha, beta, &angle_pu, &dummy);
	return (angle_pu >> (30 - 29));
}

fixp_t FIXP_cos(fixp_t angle_rad)
{
#ifdef FIXPMATH_USE_CORDIC
	fixp30_t angle_pu = FIXP24_mpy(angle_rad, FIXP30(1.0f/M_TWOPI));
	__disable_irq();
	WRITE_REG(CORDIC->CSR, CORDIC_CONFIG_COSINE);
	LL_CORDIC_WriteData(CORDIC, ANGLE_PU_TO_CORDIC(angle_pu));
	LL_CORDIC_WriteData(CORDIC, FIXP24(1.0f));
	while (HAL_IS_BIT_CLR(CORDIC->CSR, CORDIC_CSR_RRDY)); /* Wait for result */
	fixp_t data = LL_CORDIC_ReadData(CORDIC);
	__enable_irq();
	return data;
#else
	fixp_t angle_pu = FIXP30_mpy(angle_rad, FIXP30(1.0f/M_TWOPI));
    fixp15_t angle_pu_q15 = (angle_pu & (FIXP(1.0f)-1)) >> (FIXP_FMT-15) ; /* Wrap the angle by ANDing, shift to q15 format */
    Vector_cossin_s cossin = MATHLIB_cossin(angle_pu_q15);
    return (cossin.cos << (FIXP_FMT-15));
#endif
}
#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
fixp_t FIXP_cos_PU(fixp_t angle_pu)
{
#ifdef FIXPMATH_USE_CORDIC
	__disable_irq();
	WRITE_REG(CORDIC->CSR, CORDIC_CONFIG_COSINE);
	LL_CORDIC_WriteData(CORDIC, ANGLE_FP24_PU_TO_CORDIC(angle_pu));
	LL_CORDIC_WriteData(CORDIC, FIXP24(1.0f));
	while (HAL_IS_BIT_CLR(CORDIC->CSR, CORDIC_CSR_RRDY)); /* Wait for result */
	fixp_t data = LL_CORDIC_ReadData(CORDIC);
	__enable_irq();
	return data;
#else
    fixp15_t angle_pu_q15 = (angle_pu & (FIXP(1.0f)-1)) >> (FIXP_FMT-15) ; /* Wrap the angle by ANDing, shift to q15 format */
    Vector_cossin_s cossin = MATHLIB_cossin(angle_pu_q15);
    return (cossin.cos << (FIXP_FMT-15));
#endif
}
#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
fixp30_t FIXP30_cos_PU(fixp30_t angle_pu)
{
#ifdef FIXPMATH_USE_CORDIC
	__disable_irq();
	WRITE_REG(CORDIC->CSR, CORDIC_CONFIG_COSINE);
	LL_CORDIC_WriteData(CORDIC, ANGLE_PU_TO_CORDIC(angle_pu));
	LL_CORDIC_WriteData(CORDIC, FIXP30(1.0f));
	while (HAL_IS_BIT_CLR(CORDIC->CSR, CORDIC_CSR_RRDY)); /* Wait for result */
	fixp30_t data = LL_CORDIC_ReadData(CORDIC);
	__enable_irq();
	return data;
#else
    fixp_t angle_pu_q15 = (angle_pu & (FIXP30(1.0f)-1)) >> (30-15) ; /* Wrap the angle by ANDing, shift to q15 format */
    Vector_cossin_s cossin = MATHLIB_cossin(angle_pu_q15);
    return (cossin.cos << (30-15));
#endif
}
#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
fixp30_t FIXP30_sin_PU(fixp30_t angle_pu)
{
#ifdef FIXPMATH_USE_CORDIC
	__disable_irq();
	WRITE_REG(CORDIC->CSR, CORDIC_CONFIG_SINE);
	LL_CORDIC_WriteData(CORDIC, ANGLE_PU_TO_CORDIC(angle_pu));
	LL_CORDIC_WriteData(CORDIC, FIXP30(1.0f));
	while (HAL_IS_BIT_CLR(CORDIC->CSR, CORDIC_CSR_RRDY)); /* Wait for result */
	fixp30_t data = LL_CORDIC_ReadData(CORDIC);
	__enable_irq();
	return data;
#else
    fixp_t angle_pu_q15 = (angle_pu & (FIXP30(1.0f)-1)) >> (30-15) ; /* Wrap the angle by ANDing, shift to q15 format */
    Vector_cossin_s cossin = MATHLIB_cossin(angle_pu_q15);
    return (cossin.sin << (30-15));
#endif
}

fixp_t FIXP_exp(fixp_t power)
{
	// ToDo: Use CORDIC when supported (The exponential function, exp x, can be obtained as the sum of sinh x and cosh x.)
	float_t power_flt = FIXP_toF(power);
	return FIXP((float_t)expf(power_flt));
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
fixp30_t FIXP30_sqrt(const fixp30_t value)
{
	/* Return zero for negative/zero inputs */
	if (value <= 0) return 0;

#ifdef FIXPMATH_USE_CORDIC_disabled	/* Not reliable over the required range yet */
	__disable_irq();
	WRITE_REG(CORDIC->CSR, CORDIC_CONFIG_SQRT);
	LL_CORDIC_WriteData(CORDIC, value);
	while (HAL_IS_BIT_CLR(CORDIC->CSR, CORDIC_CSR_RRDY)); /* Wait for result */
	fixp30_t data = LL_CORDIC_ReadData(CORDIC);
	__enable_irq();
	return data;
#else
	/* Floating point placeholder calculation */
	return FIXP30(sqrtf(FIXP30_toF(value)));
#endif
}

fixp24_t FIXP24_sqrt(const fixp24_t value)
{
	// ToDo: Use CORDIC when supported
	/* Floating point placeholder calculation */
	return FIXP24(sqrtf(FIXP24_toF(value)));
}

/* end of fixpmath.c */

/************************ (C) COPYRIGHT 2023 Piak Electronic Design B.V. *****END OF FILE****/
