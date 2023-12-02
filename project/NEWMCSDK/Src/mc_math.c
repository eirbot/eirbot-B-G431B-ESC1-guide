
/**
  ******************************************************************************
  * @file    mc_math.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides mathematics functions useful for and specific to
  *          Motor Control.
  *
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
  */
/* Includes ------------------------------------------------------------------*/
#include "mc_math.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup MC_Math Motor Control Math functions
  * @brief Motor Control Mathematic functions of the Motor Control SDK
  *
  * @todo Document the Motor Control Math "module".
  *
  * @{
  */

/* Private macro -------------------------------------------------------------*/

#define divSQRT_3 (int32_t)0x49E6    /* 1/sqrt(3) in q1.15 format=0.5773315 */

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  This function transforms stator values a and b (which are
  *         directed along axes each displaced by 120 degrees) into values
  *         alpha and beta in a stationary qd reference frame.
  *                               alpha = a
  *                       beta = -(2*b+a)/sqrt(3)
  * @param  Input: stator values a and b in ab_t format.
  * @retval Stator values alpha and beta in alphabeta_t format.
  */
__weak alphabeta_t MCM_Clarke(ab_t Input)
{
  alphabeta_t Output;

  int32_t a_divSQRT3_tmp;
  int32_t b_divSQRT3_tmp;
  int32_t wbeta_tmp;
  int16_t hbeta_tmp;

  /* qIalpha = qIas*/
  Output.alpha = Input.a;

  a_divSQRT3_tmp = divSQRT_3 * ((int32_t)Input.a);

  b_divSQRT3_tmp = divSQRT_3 * ((int32_t)Input.b);

  /* qIbeta = -(2*qIbs+qIas)/sqrt(3) */
#ifndef FULL_MISRA_C_COMPLIANCY_MC_MATH
  /* WARNING: the below instruction is not MISRA compliant, user should verify
    that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
    the compiler to perform the shift (instead of LSR logical shift right) */
  //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
  wbeta_tmp = (-(a_divSQRT3_tmp) - (b_divSQRT3_tmp) - (b_divSQRT3_tmp)) >> 15;
#else
  wbeta_tmp = (-(a_divSQRT3_tmp) - (b_divSQRT3_tmp) - (b_divSQRT3_tmp)) / 32768;
#endif

  /* Check saturation of Ibeta */
  if (wbeta_tmp > INT16_MAX)
  {
    hbeta_tmp = INT16_MAX;
  }
  else if (wbeta_tmp < (-32768))
  {
    hbeta_tmp =  ((int16_t)-32768);
  }
  else
  {
    hbeta_tmp = ((int16_t)wbeta_tmp);
  }

  Output.beta = hbeta_tmp;

  if (((int16_t )-32768) == Output.beta)
  {
    Output.beta = -32767;
  }
  else
  {
    /* Nothing to do */
  }

  return (Output);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  This function transforms stator values alpha and beta, which
  *         belong to a stationary qd reference frame, to a rotor flux
  *         synchronous reference frame (properly oriented), so as q and d.
  *                   d= alpha *sin(theta)+ beta *cos(Theta)
  *                   q= alpha *cos(Theta)- beta *sin(Theta)
  * @param  Input: stator values alpha and beta in alphabeta_t format.
  * @param  Theta: rotating frame angular position in q1.15 format.
  * @retval Stator values q and d in qd_t format
  */
__weak qd_t MCM_Park(alphabeta_t Input, int16_t Theta)
{
  qd_t Output;
  int32_t d_tmp_1;
  int32_t d_tmp_2;
  int32_t q_tmp_1;
  int32_t q_tmp_2;
  int32_t wqd_tmp;
  int16_t hqd_tmp;
  Trig_Components Local_Vector_Components;

  Local_Vector_Components = MCM_Trig_Functions(Theta);

  /* No overflow guaranteed */
  q_tmp_1 = Input.alpha * ((int32_t )Local_Vector_Components.hCos);

  /* No overflow guaranteed */
  q_tmp_2 = Input.beta * ((int32_t)Local_Vector_Components.hSin);

  /* Iq component in Q1.15 Format */
#ifndef FULL_MISRA_C_COMPLIANCY_MC_MATH
  /* WARNING: the below instruction is not MISRA compliant, user should verify
    that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
    the compiler to perform the shift (instead of LSR logical shift right) */
  wqd_tmp = (q_tmp_1 - q_tmp_2) >> 15; //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
#else
  wqd_tmp = (q_tmp_1 - q_tmp_2) / 32768;
#endif

  /* Check saturation of Iq */
  if (wqd_tmp > INT16_MAX)
  {
    hqd_tmp = INT16_MAX;
  }
  else if (wqd_tmp < (-32768))
  {
    hqd_tmp = ((int16_t)-32768);
  }
  else
  {
    hqd_tmp = ((int16_t)wqd_tmp);
  }

  Output.q = hqd_tmp;

  if (((int16_t)-32768) == Output.q)
  {
    Output.q = -32767;
  }
  else
  {
    /* Nothing to do */
  }

  /* No overflow guaranteed */
  d_tmp_1 = Input.alpha * ((int32_t )Local_Vector_Components.hSin);

  /* No overflow guaranteed */
  d_tmp_2 = Input.beta * ((int32_t )Local_Vector_Components.hCos);

  /* Id component in Q1.15 Format */
#ifndef FULL_MISRA_C_COMPLIANCY_MC_MATH
  /* WARNING: the below instruction is not MISRA compliant, user should verify
    that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
    the compiler to perform the shift (instead of LSR logical shift right) */
  wqd_tmp = (d_tmp_1 + d_tmp_2) >> 15; //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
#else
  wqd_tmp = (d_tmp_1 + d_tmp_2) / 32768;
#endif

  /* Check saturation of Id */
  if (wqd_tmp > INT16_MAX)
  {
    hqd_tmp = INT16_MAX;
  }
  else if (wqd_tmp < (-32768))
  {
    hqd_tmp = ((int16_t)-32768);
  }
  else
  {
    hqd_tmp = ((int16_t)wqd_tmp);
  }

  Output.d = hqd_tmp;

  if (((int16_t)-32768) == Output.d)
  {
    Output.d = -32767;
  }
  else
  {
    /* Nothing to do */
  }

  return (Output);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  This function transforms stator voltage qVq and qVd, that belong to
  *         a rotor flux synchronous rotating frame, to a stationary reference
  *         frame, so as to obtain qValpha and qVbeta:
  *                  Valfa= Vq*Cos(theta)+ Vd*Sin(theta)
  *                  Vbeta=-Vq*Sin(theta)+ Vd*Cos(theta)
  * @param  Input: stator voltage Vq and Vd in qd_t format.
  * @param  Theta: rotating frame angular position in q1.15 format.
  * @retval Stator voltage Valpha and Vbeta in qd_t format.
  */
__weak alphabeta_t MCM_Rev_Park(qd_t Input, int16_t Theta)
{
  int32_t alpha_tmp1;
  int32_t alpha_tmp2;
  int32_t beta_tmp1;
  int32_t beta_tmp2;
  Trig_Components Local_Vector_Components;
  alphabeta_t Output;

  Local_Vector_Components = MCM_Trig_Functions(Theta);

  /* No overflow guaranteed */
  alpha_tmp1 = Input.q * ((int32_t)Local_Vector_Components.hCos);
  alpha_tmp2 = Input.d * ((int32_t)Local_Vector_Components.hSin);

#ifndef FULL_MISRA_C_COMPLIANCY_MC_MATH
  /* WARNING: the below instruction is not MISRA compliant, user should verify
    that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
    the compiler to perform the shift (instead of LSR logical shift right) */
  //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
  Output.alpha = (int16_t)(((alpha_tmp1) + (alpha_tmp2)) >> 15);
#else
  Output.alpha = (int16_t)(((alpha_tmp1) + (alpha_tmp2)) / 32768);
#endif

  beta_tmp1 = Input.q * ((int32_t)Local_Vector_Components.hSin);
  beta_tmp2 = Input.d * ((int32_t)Local_Vector_Components.hCos);

#ifndef FULL_MISRA_C_COMPLIANCY_MC_MATH
  /* WARNING: the below instruction is not MISRA compliant, user should verify
  that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
  the compiler to perform the shift (instead of LSR logical shift right) */
  //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
  Output.beta = (int16_t)((beta_tmp2 - beta_tmp1) >> 15);
#else
  Output.beta = (int16_t)((beta_tmp2 - beta_tmp1) / 32768);
#endif

  return (Output);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  This function returns cosine and sine functions of the angle fed in input.
  * @param  hAngle: angle in q1.15 format.
  * @retval Sin(angle) and Cos(angle) in Trig_Components format.
  */

__weak Trig_Components MCM_Trig_Functions(int16_t hAngle)
{
  /* MISRAC2012-violation Rule 19.2. The union keyword should not be used.
   * If this rule is not followed, the kinds of behavior that need to be determined
   * are:
   * Padding — how much padding is inserted at the end of the union;
   * Alignment — how are members of any structures within the union aligned;
   * Endianness — is the most significant byte of a word stored at the lowest or
   *              highest memory address;
   * Bit-order — how are bits numbered within bytes and how are bits allocated to
   *             bit fields.
   * Low. Use of union (u32toi16x2). */
  //cstat -MISRAC2012-Rule-19.2
  union u32toi16x2 {
    uint32_t CordicRdata;
    Trig_Components Components;
  } CosSin;
  //cstat +MISRAC2012-Rule-19.2
  /* Configure CORDIC */
  /* Misra  violation Rule 11.4 A�Conversion�should�not�be�performed�between�a�
   * pointer�to�object and an integer type */
  WRITE_REG(CORDIC->CSR, CORDIC_CONFIG_COSINE);
  /* Misra  violation Rule�11.4 A�Conversion�should�not�be�performed�between�a
   * pointer�to�object and an integer type */
  LL_CORDIC_WriteData(CORDIC, ((uint32_t)0x7FFF0000) + ((uint32_t)hAngle));
  /* Read angle */
  /* Misra  violation Rule�11.4 A�Conversion�should�not�be�performed between�a
   * pointer�to object and an integer type */
  CosSin.CordicRdata = LL_CORDIC_ReadData(CORDIC);
  return (CosSin.Components); //cstat !UNION-type-punning
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  It calculates the square root of a non-negative int32_t. It returns 0 for negative int32_t.
  * @param  Input int32_t number.
  * @retval int32_t Square root of Input (0 if Input<0).
  */
__weak int32_t MCM_Sqrt(int32_t wInput)
{
  int32_t wtemprootnew;

  if (wInput > 0)
  {
    uint32_t retVal;
    /* Disable Irq as sqrt is used in MF and HF task */
    __disable_irq();
    /* Configure CORDIC */
    WRITE_REG(CORDIC->CSR, CORDIC_CONFIG_SQRT);
    LL_CORDIC_WriteData(CORDIC, ((uint32_t)wInput));
    /* Read sqrt and return */
#ifndef FULL_MISRA_C_COMPLIANCY_MC_MATH
    retVal = (LL_CORDIC_ReadData(CORDIC)) >> 15; //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
#else
    retVal = (LL_CORDIC_ReadData(CORDIC)) / 32768U;
#endif
    wtemprootnew = (int32_t)retVal;
    __enable_irq();

  }
  else
  {
    wtemprootnew = (int32_t)0;
  }

  return (wtemprootnew);
}

/**
  * @brief  This function codify a floating point number into the relative 32bit integer.
  * @param  float Floating point number to be coded.
  * @retval uint32_t Coded 32bit integer.
  */
__weak uint32_t MCM_floatToIntBit( float_t x ) //cstat !MISRAC2012-Dir-4.6_a
{
  const uint32_t *pInt;
  pInt = (uint32_t *)(&x); //cstat !MISRAC2012-Rule-11.3
  return (*pInt);
}

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
