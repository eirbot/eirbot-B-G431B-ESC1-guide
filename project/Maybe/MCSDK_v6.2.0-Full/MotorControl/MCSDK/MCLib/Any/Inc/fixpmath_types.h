/**
  ******************************************************************************
  * @file    fixpmath_types.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the structure definitions of the vectors 
  *          used in Motor Control SDK.
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

#ifndef _FIXPMATH_TYPES_H_
#define _FIXPMATH_TYPES_H_

#include <stdbool.h>

typedef struct 
{
    fixp30_t cos;
    fixp30_t sin;
} FIXP_CosSin_t;

typedef struct 
{
    fixp30_t A;        /*!< @brief value of @f$\alpha@f$ encoded in fixed-point 30Bit format */
    fixp30_t B;        /*!< @brief value of @f$\beta@f$ encoded in fixed-point 30Bit format */
} Vector_ab_t;

typedef struct 
{
    fixp30_t D;        /*!< @brief value in D direction encoded in fixed-point 30Bit format */
    fixp30_t Q;        /*!< @brief value in D direction encoded in fixed-point 30Bit format */
} Vector_dq_t;

typedef struct 
{
    fixp30_t R;        /*!< @brief value of phase R encoded in fixed-point 30Bit format */
    fixp30_t S;        /*!< @brief value of phase S encoded in fixed-point 30Bit format */
    fixp30_t T;        /*!< @brief value of phase T encoded in fixed-point 30Bit format */
} Vector_rst;

typedef Vector_rst Currents_Irst_t;

typedef struct 
{
    uint_least8_t	numValid;
    bool	validR;
    bool	validS;
    bool	validT;
} CurrentReconstruction_t;

typedef Vector_rst Voltages_Urst_t;

typedef Vector_ab_t Currents_Iab_t;

typedef Vector_dq_t Currents_Idq_t;

typedef Vector_dq_t Voltages_Udq_t;

typedef Vector_ab_t Voltages_Uab_t;

typedef Vector_dq_t Duty_Ddq_t;

typedef Vector_ab_t Duty_Dab_t;

typedef Vector_rst Duty_Drst_t;

#endif /* _FIXPMATH_TYPES_H_ */

/* end of fixpmath_types.h */
