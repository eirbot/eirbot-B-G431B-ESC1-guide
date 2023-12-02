/**
  ******************************************************************************
  * @file    fixpmath.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the fixed-point format definitions used 
  *           in Motor Control SDK.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup FixpMath
  */

#ifndef _FIXPMATH_H_
#define _FIXPMATH_H_

/* Global default format */
#define FIXP_FMT			(24)			/* Per unit internal format is q24 */
#ifdef FIXP_PRECISION_FLOAT
#define FIXP_FMT_MPY		(16777216.0f)	/* Must be 2^FIXP_FMT */
#else
#define FIXP_FMT_MPY		(16777216.0L)	/* Must be 2^FIXP_FMT */
#endif

#ifdef FIXP_CORDIC_INLINE
#include "mc_cordic.h"
#endif /* FIXP_CORDIC_INLINE */

#include <stdint.h> /* int32_t */
#include <math.h> /* float_t, 2PI */

#ifndef M_TWOPI
#define M_TWOPI 6.283185307179586476925286766559L
#endif

#define MATH_TWO_PI (M_TWOPI)

typedef int32_t fixp_t;

/* Internal multiply macro */
#define FIXP_MPY(a, b, n)	( (fixp_t)(((int64_t) (a) * (b)) >> (n)) )

/* Conversion from floating point to fixed point */
#ifdef FIXP_PRECISION_FLOAT
#define FIXP(A)	(fixp_t) ((A) * FIXP_FMT_MPY)
#define FIXP0(A)	(fixp0_t) ((A) * 1.0f)
#define FIXP1(A)	(fixp1_t) ((A) * 2.0f)
#define FIXP2(A)	(fixp2_t) ((A) * 4.0f)
#define FIXP3(A)	(fixp3_t) ((A) * 8.0f)
#define FIXP4(A)	(fixp4_t) ((A) * 16.0f)
#define FIXP5(A)	(fixp5_t) ((A) * 32.0f)
#define FIXP6(A)	(fixp6_t) ((A) * 64.0f)
#define FIXP7(A)	(fixp7_t) ((A) * 128.0f)
#define FIXP8(A)	(fixp8_t) ((A) * 256.0f)
#define FIXP9(A)	(fixp9_t) ((A) * 512.0f)
#define FIXP10(A)	(fixp10_t) ((A) * 1024.0f)
#define FIXP11(A)	(fixp11_t) ((A) * 2048.0f)
#define FIXP12(A)	(fixp10_t) ((A) * 4096.0f)
#define FIXP13(A)	(fixp13_t) ((A) * 8192.0f)
#define FIXP14(A)	(fixp14_t) ((A) * 16384.0f)
#define FIXP15(A)	(fixp15_t) ((A) * 32768.0f)
#define FIXP16(A)	(fixp16_t) ((A) * 65536.0f)
#define FIXP17(A)	(fixp17_t) ((A) * 131072.0f)
#define FIXP18(A)	(fixp18_t) ((A) * 262144.0f)
#define FIXP19(A)	(fixp19_t) ((A) * 524288.0f)
#define FIXP20(A)	(fixp20_t) ((A) * 1048576.0f)
#define FIXP21(A)	(fixp21_t) ((A) * 2097152.0f)
#define FIXP22(A)	(fixp22_t) ((A) * 4194304.0f)
#define FIXP23(A)	(fixp23_t) ((A) * 8388608.0f)
#define FIXP24(A)	(fixp24_t) ((A) * 16777216.0f)
#define FIXP25(A)	(fixp25_t) ((A) * 33554432.0f)
#define FIXP26(A)	(fixp26_t) ((A) * 67108864.0f)
#define FIXP27(A)	(fixp27_t) ((A) * 134217728.0f)
#define FIXP28(A)	(fixp28_t) ((A) * 268435456.0f)
#define FIXP29(A)	(fixp29_t) ((A) * 536870912.0f)
#define FIXP30(A)	(fixp30_t) ((A) * 1073741824.0f)
#define FIXP31(A)	(fixp31_t) ((A) * 2147483648.0f)
#else /* FIXP_PRECISION_FLOAT */
#define FIXP(A)	(fixp_t) ((A) * FIXP_FMT_MPY)
#define FIXP0(A)	(fixp0_t) ((A) * 1.0L)
#define FIXP1(A)	(fixp1_t) ((A) * 2.0L)
#define FIXP2(A)	(fixp2_t) ((A) * 4.0L)
#define FIXP3(A)	(fixp3_t) ((A) * 8.0L)
#define FIXP4(A)	(fixp4_t) ((A) * 16.0L)
#define FIXP5(A)	(fixp5_t) ((A) * 32.0L)
#define FIXP6(A)	(fixp6_t) ((A) * 64.0L)
#define FIXP7(A)	(fixp7_t) ((A) * 128.0L)
#define FIXP8(A)	(fixp8_t) ((A) * 256.0L)
#define FIXP9(A)	(fixp9_t) ((A) * 512.0L)
#define FIXP10(A)	(fixp10_t) ((A) * 1024.0L)
#define FIXP11(A)	(fixp11_t) ((A) * 2048.0L)
#define FIXP12(A)	(fixp10_t) ((A) * 4096.0L)
#define FIXP13(A)	(fixp13_t) ((A) * 8192.0L)
#define FIXP14(A)	(fixp14_t) ((A) * 16384.0L)
#define FIXP15(A)	(fixp15_t) ((A) * 32768.0L)
#define FIXP16(A)	(fixp16_t) ((A) * 65536.0L)
#define FIXP17(A)	(fixp17_t) ((A) * 131072.0L)
#define FIXP18(A)	(fixp18_t) ((A) * 262144.0L)
#define FIXP19(A)	(fixp19_t) ((A) * 524288.0L)
#define FIXP20(A)	(fixp20_t) ((A) * 1048576.0L)
#define FIXP21(A)	(fixp21_t) ((A) * 2097152.0L)
#define FIXP22(A)	(fixp22_t) ((A) * 4194304.0L)
#define FIXP23(A)	(fixp23_t) ((A) * 8388608.0L)
#define FIXP24(A)	(fixp24_t) ((A) * 16777216.0L)
#define FIXP25(A)	(fixp25_t) ((A) * 33554432.0L)
#define FIXP26(A)	(fixp26_t) ((A) * 67108864.0L)
#define FIXP27(A)	(fixp27_t) ((A) * 134217728.0L)
#define FIXP28(A)	(fixp28_t) ((A) * 268435456.0L)
#define FIXP29(A)	(fixp29_t) ((A) * 536870912.0L)
#define FIXP30(A)	(fixp30_t) ((A) * 1073741824.0L)
#define FIXP31(A)	(fixp31_t) ((A) * 2147483648.0L)
#endif /* FIXP_PRECISION_FLOAT */

/* Conversion between two fixed point scales */
#define FIXPtoFIXP29(A)	((long) (A) << (29 - FIXP_FMT))	/* ToDo: take sign of shift into account */
#define FIXPtoFIXP30(A)	((long) (A) << (30 - FIXP_FMT))
#define FIXPtoFIXP31(A)	((long) (A) << (31 - FIXP_FMT))
#define FIXP30_toFIXP(A)	((long) (A) >> (30 - FIXP_FMT))

/* Conversion to float */
#define FIXP_toF(A) 	((float_t)(((float_t) (A)) / FIXP_FMT_MPY))
#ifdef FIXP_PRECISION_FLOAT
#define FIXP8_toF(A)	((float_t)(((float_t) (A)) / 256.0f))
#define FIXP11_toF(A)	((float_t)(((float_t) (A)) / 2048.0f))
#define FIXP15_toF(A)	((float_t)(((float_t) (A)) / 32768.0f))
#define FIXP16_toF(A)	((float_t)(((float_t) (A)) / 65536.0f))
#define FIXP20_toF(A)	((float_t)(((float_t) (A)) / 1048576.0f))
#define FIXP24_toF(A)	((float_t)(((float_t) (A)) / 16777216.0f))
#define FIXP29_toF(A)	((float_t)(((float_t) (A)) / 536870912.0f))
#define FIXP30_toF(A)	((float_t)(((float_t) (A)) / 1073741824.0f))
#define FIXP31_toF(A)	((float_t)(((float_t) (A)) / 2147483648.0f))
#else /* FIXP_PRECISION_FLOAT */
#define FIXP8_toF(A)	((float_t)(((float_t) (A)) / 256.0L))
#define FIXP11_toF(A)	((float_t)(((float_t) (A)) / 2048.0L))
#define FIXP15_toF(A)	((float_t)(((float_t) (A)) / 32768.0L))
#define FIXP16_toF(A)	((float_t)(((float_t) (A)) / 65536.0L))
#define FIXP20_toF(A)	((float_t)(((float_t) (A)) / 1048576.0L))
#define FIXP24_toF(A)	((float_t)(((float_t) (A)) / 16777216.0L))
#define FIXP29_toF(A)	((float_t)(((float_t) (A)) / 536870912.0L))
#define FIXP30_toF(A)	((float_t)(((float_t) (A)) / 1073741824.0L))
#define FIXP31_toF(A)	((float_t)(((float_t) (A)) / 2147483648.0L))
#endif /* FIXP_PRECISION_FLOAT */

/* Multiplication */
#define FIXP_mpy(a, b)		FIXP_MPY((a), (b), FIXP_FMT)
#define FIXP1_mpy(a, b)		FIXP_MPY((a), (b), 1)
#define FIXP2_mpy(a, b)		FIXP_MPY((a), (b), 2)
#define FIXP3_mpy(a, b)		FIXP_MPY((a), (b), 3)
#define FIXP4_mpy(a, b)		FIXP_MPY((a), (b), 4)
#define FIXP5_mpy(a, b)		FIXP_MPY((a), (b), 5)
#define FIXP6_mpy(a, b)		FIXP_MPY((a), (b), 6)
#define FIXP7_mpy(a, b)		FIXP_MPY((a), (b), 7)
#define FIXP8_mpy(a, b)		FIXP_MPY((a), (b), 8)
#define FIXP9_mpy(a, b)		FIXP_MPY((a), (b), 9)
#define FIXP10_mpy(a, b)	FIXP_MPY((a), (b), 10)
#define FIXP11_mpy(a, b)	FIXP_MPY((a), (b), 11)
#define FIXP12_mpy(a, b)	FIXP_MPY((a), (b), 12)
#define FIXP13_mpy(a, b)	FIXP_MPY((a), (b), 13)
#define FIXP14_mpy(a, b)	FIXP_MPY((a), (b), 14)
#define FIXP15_mpy(a, b)	FIXP_MPY((a), (b), 15)
#define FIXP16_mpy(a, b)	FIXP_MPY((a), (b), 16)
#define FIXP17_mpy(a, b)	FIXP_MPY((a), (b), 17)
#define FIXP18_mpy(a, b)	FIXP_MPY((a), (b), 18)
#define FIXP19_mpy(a, b)	FIXP_MPY((a), (b), 19)
#define FIXP20_mpy(a, b)	FIXP_MPY((a), (b), 20)
#define FIXP21_mpy(a, b)	FIXP_MPY((a), (b), 21)
#define FIXP22_mpy(a, b)	FIXP_MPY((a), (b), 22)
#define FIXP23_mpy(a, b)	FIXP_MPY((a), (b), 23)
#define FIXP24_mpy(a, b)	FIXP_MPY((a), (b), 24)
#define FIXP25_mpy(a, b)	FIXP_MPY((a), (b), 25)
#define FIXP26_mpy(a, b)	FIXP_MPY((a), (b), 26)
#define FIXP27_mpy(a, b)	FIXP_MPY((a), (b), 27)
#define FIXP28_mpy(a, b)	FIXP_MPY((a), (b), 28)
#define FIXP29_mpy(a, b)	FIXP_MPY((a), (b), 29)
#define FIXP30_mpy(a, b)	FIXP_MPY((a), (b), 30)
#define FIXP31_mpy(a, b)	FIXP_MPY((a), (b), 31)

/* Division */
#define FIXP_DIV(a, b, n)	((fixp_t)((((int64_t) (a)) << (n)) / (b)))
#define FIXP_div(a, b)		FIXP_DIV((a), (b), FIXP_FMT)
#define FIXP30_div(a, b)	FIXP_DIV((a), (b), 30)

/* ToDo Saturating mpy not implemented, using standard mpy instead */
#define FIXP_rsmpy(a, b)	FIXP_MPY((a), (b), FIXP_FMT)
#define FIXP1_rsmpy(a, b)	FIXP_MPY((a), (b), 1)
#define FIXP2_rsmpy(a, b)	FIXP_MPY((a), (b), 2)
#define FIXP3_rsmpy(a, b)	FIXP_MPY((a), (b), 3)
#define FIXP4_rsmpy(a, b)	FIXP_MPY((a), (b), 4)
#define FIXP5_rsmpy(a, b)	FIXP_MPY((a), (b), 5)
#define FIXP6_rsmpy(a, b)	FIXP_MPY((a), (b), 6)
#define FIXP7_rsmpy(a, b)	FIXP_MPY((a), (b), 7)
#define FIXP8_rsmpy(a, b)	FIXP_MPY((a), (b), 8)
#define FIXP9_rsmpy(a, b)	FIXP_MPY((a), (b), 9)
#define FIXP10_rsmpy(a, b)	FIXP_MPY((a), (b), 10)
#define FIXP11_rsmpy(a, b)	FIXP_MPY((a), (b), 11)
#define FIXP12_rsmpy(a, b)	FIXP_MPY((a), (b), 12)
#define FIXP13_rsmpy(a, b)	FIXP_MPY((a), (b), 13)
#define FIXP14_rsmpy(a, b)	FIXP_MPY((a), (b), 14)
#define FIXP15_rsmpy(a, b)	FIXP_MPY((a), (b), 15)
#define FIXP16_rsmpy(a, b)	FIXP_MPY((a), (b), 16)
#define FIXP17_rsmpy(a, b)	FIXP_MPY((a), (b), 17)
#define FIXP18_rsmpy(a, b)	FIXP_MPY((a), (b), 18)
#define FIXP19_rsmpy(a, b)	FIXP_MPY((a), (b), 19)
#define FIXP20_rsmpy(a, b)	FIXP_MPY((a), (b), 20)
#define FIXP21_rsmpy(a, b)	FIXP_MPY((a), (b), 21)
#define FIXP22_rsmpy(a, b)	FIXP_MPY((a), (b), 22)
#define FIXP23_rsmpy(a, b)	FIXP_MPY((a), (b), 23)
#define FIXP24_rsmpy(a, b)	FIXP_MPY((a), (b), 24)
#define FIXP25_rsmpy(a, b)	FIXP_MPY((a), (b), 25)
#define FIXP26_rsmpy(a, b)	FIXP_MPY((a), (b), 26)
#define FIXP27_rsmpy(a, b)	FIXP_MPY((a), (b), 27)
#define FIXP28_rsmpy(a, b)	FIXP_MPY((a), (b), 28)
#define FIXP29_rsmpy(a, b)	FIXP_MPY((a), (b), 29)
#define FIXP30_rsmpy(a, b)	FIXP_MPY((a), (b), 30)

/* Division by bitshift */
#define _FIXPdiv2(a)		((a) >> 1)
#define _FIXPdiv4(a)		((a) >> 2)
#define _FIXPdiv8(a)		((a) >> 3)

/* Included after all the _FIXPNrsmpy() macros */
#include "fixpmpyxufloat.h"

/* Trigonometric
 * (All implemented as functions, below) */

/* Saturation */
#define FIXP_sat(A, V_MAX, V_MIN)	((A) > (V_MAX) ? (V_MAX) : (A) < (V_MIN) ? (V_MIN) : (A))

/* Absolute */
#define FIXP_abs(A)					((A) >= 0 ? (A) : -(A))
#define FIXP30_abs(A) FIXP_abs(A)

typedef fixp_t fixp1_t;
typedef fixp_t fixp2_t;
typedef fixp_t fixp3_t;
typedef fixp_t fixp4_t;
typedef fixp_t fixp5_t;
typedef fixp_t fixp6_t;
typedef fixp_t fixp7_t;
typedef fixp_t fixp8_t;
typedef fixp_t fixp9_t;
typedef fixp_t fixp10_t;
typedef fixp_t fixp11_t;
typedef fixp_t fixp12_t;
typedef fixp_t fixp13_t;
typedef fixp_t fixp14_t;
typedef fixp_t fixp15_t;
typedef fixp_t fixp16_t;    /*!< @brief fixed-point 16 Bit format */
typedef fixp_t fixp17_t;
typedef fixp_t fixp18_t;
typedef fixp_t fixp19_t;
typedef fixp_t fixp20_t;
typedef fixp_t fixp21_t;
typedef fixp_t fixp22_t;
typedef fixp_t fixp23_t;
typedef fixp_t fixp24_t;    /*!< @brief fixed-point 24 Bit format */
typedef fixp_t fixp25_t;
typedef fixp_t fixp26_t;
typedef fixp_t fixp27_t;
typedef fixp_t fixp28_t;
typedef fixp_t fixp29_t;
typedef fixp_t fixp30_t;    /*!< @brief fixed-point 30 Bit format */
typedef fixp_t fixp31_t;
typedef int_least8_t fixpFmt_t;

typedef struct
{
    fixp_t    	value;
    fixpFmt_t	fixpFmt;
} FIXP_scaled_t;

#include "fixpmath_types.h"

void FIXPSCALED_floatToFIXPscaled(const float value, FIXP_scaled_t *pFps);
void FIXPSCALED_floatToFIXPscaled_exp(const float value, FIXP_scaled_t *pFps, fixpFmt_t exponent);
float FIXPSCALED_FIXPscaledToFloat(const FIXP_scaled_t *pFps);
void FIXPSCALED_doubleToFIXPscaled(const double value, FIXP_scaled_t *pFps);

void FIXPSCALED_calculateScaleFactor(
    const fixpFmt_t source_fixpFmt,
    const fixpFmt_t target_fixpFmt,
    const float source_fullscale,
    const float target_fullscale,
    const float datascale_factor,
	FIXP_scaled_t *pFps);

static inline fixp_t FIXP_mpyFIXPscaled(const fixp_t a, const FIXP_scaled_t *pFps)
{
	/* Returned value is in the same format as parameter a */
    return FIXP_MPY(a, pFps->value, pFps->fixpFmt);
}

void FIXP30_CosSinPU(const fixp30_t angle_pu, FIXP_CosSin_t *pCosSin);
void FIXP30_polar(const fixp30_t x, const fixp30_t y, fixp30_t *pAngle_pu, fixp30_t *pMagnitude);

/* Function declarations */
void FIXPMATH_init(void);
fixp_t FIXP_mag(const fixp_t a, const fixp_t b);
fixp30_t FIXP30_mag(const fixp30_t a, const fixp30_t b);
fixp24_t FIXP24_atan2_PU(fixp24_t beta, fixp24_t alpha);
fixp29_t FIXP29_atan2_PU(fixp30_t beta, fixp30_t alpha);
fixp30_t FIXP30_atan2_PU(fixp30_t beta, fixp30_t alpha);
fixp_t FIXP_cos(fixp_t angle_rad);
fixp_t FIXP_cos_PU(fixp_t angle_pu);
fixp30_t FIXP30_cos_PU(fixp30_t angle_pu);
fixp30_t FIXP30_sin_PU(fixp30_t angle_pu);
fixp_t FIXP_exp(fixp_t power);
fixp30_t FIXP30_sqrt(const fixp30_t value);
fixp24_t FIXP24_sqrt(const fixp24_t value);

#endif /* _FIXPMATH_H_ */

/* end of fixpmath.h */
