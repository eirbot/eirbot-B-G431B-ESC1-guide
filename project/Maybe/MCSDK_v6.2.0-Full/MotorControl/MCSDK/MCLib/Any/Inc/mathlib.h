/**
  ******************************************************************************
  * @file    mathlib.h
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
/* mathlib.h */

#ifndef _MATHLIB_H_
#define _MATHLIB_H_

#include <stdint.h>

typedef struct _Vector_cossin_s_
{
	int32_t cos;
	int32_t sin;
} Vector_cossin_s;


/* Initialize mathlib by calculating/loading tables */
void MATHLIB_init(void);

/* Convert to polar form, calculating angle and magnitude of vector */
void MATHLIB_polar(const int32_t x, const int32_t y, int32_t *angle_pu, int32_t *magnitude);

/* Calculate cosine and sine from angle in per-unit */
Vector_cossin_s MATHLIB_cossin(int32_t angle_pu);

#endif /* _MATHLIB_H_ */

/* end of mathlib.h */

/************************ (C) COPYRIGHT 2023 Piak Electronic Design B.V. *****END OF FILE****/
