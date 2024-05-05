/**
  ******************************************************************************
  * @file    mc_cordic.h
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
/* mc_cordic.h */

#ifndef _MC_CORDIC_H_
#define _MC_CORDIC_H_

#include <stdint.h>

#include "cordic_defs.h"

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

#define _FIXP30local(A)	(int32_t) ((A) * 1073741824.0f) // local macro

#define C_ANGLE_CORDIC_TO_PU(angle_cordic) (angle_cordic >> 2) & (_FIXP30local(1.0f)-1) /* scale and wrap cordic angle to per unit */
#define C_ANGLE_PU_TO_CORDIC(angle_pu) (angle_pu << 2)
#define C_ANGLE_FP24_PU_TO_CORDIC(angle_pu) (angle_pu << 8)

static inline void _CORDIC_polar(const int32_t x, const int32_t y, int32_t *pAngle_pu, int32_t *pMagnitude)
{
	// __disable_irq();
	__asm volatile ("cpsid i" : : : "memory");

	// WRITE_REG(LCORDIC->CSR, CORDIC_CONFIG_PHASE);
	LCORDIC->CSR = CORDIC_CONFIG_PHASE;

	// LL_CORDIC_WriteData(CORDIC, x);
	// WRITE_REG(CORDICx->WDATA, InData);
	LCORDIC->WDATA = x;

	// LL_CORDIC_WriteData(CORDIC, y);
	// WRITE_REG(CORDICx->WDATA, InData);
	LCORDIC->WDATA = y;

	// while (HAL_IS_BIT_CLR(LCORDIC->CSR, CORDIC_CSR_RRDY)); /* Wait for result */
	while ((LCORDIC->CSR & CORDIC_CSR_RRDY) == 0U); /* Wait for result */

	// *pAngle_pu = ANGLE_CORDIC_TO_PU(LL_CORDIC_ReadData(CORDIC));
	*pAngle_pu = C_ANGLE_CORDIC_TO_PU(LCORDIC->RDATA);

	// *pMagnitude = LL_CORDIC_ReadData(CORDIC);
	*pMagnitude = LCORDIC->RDATA;

	//	__enable_irq();
	__asm volatile ("cpsie i" : : : "memory");
}

// bit of a mutual inclusion problem for cossin struct type
typedef struct
{
	int32_t cos;
	int32_t sin;
} int32_cossin_s;

static inline void _CORDIC_30CosSinPU(const int32_t angle_pu, int32_cossin_s *pCosSin)
{
	// __disable_irq();
	__asm volatile ("cpsid i" : : : "memory");

	// WRITE_REG(LCORDIC->CSR, CORDIC_CONFIG_COSINE_AND_SINE);
	LCORDIC->CSR = CORDIC_CONFIG_COSINE_AND_SINE;

	// LL_CORDIC_WriteData(CORDIC, ANGLE_PU_TO_CORDIC(angle_pu));
	LCORDIC->WDATA = C_ANGLE_PU_TO_CORDIC(angle_pu);

	// LL_CORDIC_WriteData(CORDIC, FIXP30(1.0f));
	LCORDIC->WDATA = _FIXP30local(1.0f);

	// while (HAL_IS_BIT_CLR(LCORDIC->CSR, CORDIC_CSR_RRDY)); /* Wait for result */
	while ((LCORDIC->CSR & CORDIC_CSR_RRDY) == 0U); /* Wait for result */

	// pCosSin->cos = LL_CORDIC_ReadData(CORDIC);
	pCosSin->cos = LCORDIC->RDATA;

	// pCosSin->sin = LL_CORDIC_ReadData(CORDIC);
	pCosSin->sin = LCORDIC->RDATA;

	// __enable_irq();
	__asm volatile ("cpsie i" : : : "memory");
}

#if 0 // Not implemented yet; will be used in DEMODULATOR_calculateParameters, angle in radians q24, output in q24
static inline void _CORDIC_cos(const int32_t angle_pu, int32_cossin_s *pCosSin)
{
#if 0
	fixp30_t angle_pu = FIXP24_mpy(angle_rad, FIXP30(1.0/M_TWOPI));
	WRITE_REG(LCORDIC->CSR, CORDIC_CONFIG_COSINE);
	LL_CORDIC_WriteData(CORDIC, ANGLE_PU_TO_CORDIC(angle_pu));
	LL_CORDIC_WriteData(CORDIC, FIXP24(1.0f));
	while (HAL_IS_BIT_CLR(LCORDIC->CSR, CORDIC_CSR_RRDY)); /* Wait for result */
	return LL_CORDIC_ReadData(CORDIC);
#else

#warning unchanged cossin code

	// __disable_irq();
	__asm volatile ("cpsid i" : : : "memory");

	// WRITE_REG(LCORDIC->CSR, CORDIC_CONFIG_COSINE_AND_SINE);
	LCORDIC->CSR = CORDIC_CONFIG_COSINE_AND_SINE;

	// LL_CORDIC_WriteData(CORDIC, ANGLE_PU_TO_CORDIC(angle_pu));
	LCORDIC->WDATA = C_ANGLE_PU_TO_CORDIC(angle_pu);

	// LL_CORDIC_WriteData(CORDIC, FIXP30(1.0f));
	LCORDIC->WDATA = _FIXP30local(1.0f);

	// while (HAL_IS_BIT_CLR(LCORDIC->CSR, CORDIC_CSR_RRDY)); /* Wait for result */
	while ((LCORDIC->CSR & CORDIC_CSR_RRDY) == 0U); /* Wait for result */

	// pCosSin->cos = LL_CORDIC_ReadData(CORDIC);
	pCosSin->cos = LCORDIC->RDATA;

	// pCosSin->sin = LL_CORDIC_ReadData(CORDIC);
	pCosSin->sin = LCORDIC->RDATA;

	// __enable_irq();
	__asm volatile ("cpsie i" : : : "memory");
#endif
}
#endif

#endif /* _MC_CORDIC_H_ */

/* end of mc_cordic.h */

/************************ (C) COPYRIGHT 2023 Piak Electronic Design B.V. *****END OF FILE****/
