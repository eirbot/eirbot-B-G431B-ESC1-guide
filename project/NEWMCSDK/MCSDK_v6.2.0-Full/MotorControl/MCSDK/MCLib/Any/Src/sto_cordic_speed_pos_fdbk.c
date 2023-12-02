/**
  ******************************************************************************
  * @file    sto_cordic_speed_pos_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the State Observer + CORDIC Speed & Position Feedback component of the
  *          Motor Control SDK.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sto_cordic_speed_pos_fdbk.h"
#include "mc_math.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */

/** @defgroup STO_CORDIC_SpeednPosFdbk State Observer CORDIC Speed & Position Feedback
  * @brief State Observer with CORDIC Speed & Position Feedback implementation
  *
  * This component uses a State Observer coupled with a CORDIC (COordinate Rotation DIgital
  * Computer) to provide an estimation of the speed and the position of the rotor of the motor.
  *
  * @todo Document the State Observer + CORDIC Speed & Position Feedback "module".
  *
  * @{
  */

/* Private defines -----------------------------------------------------------*/

#define C6_COMP_CONST1  (int32_t)1043038
#define C6_COMP_CONST2  (int32_t)10430

/* Private functions prototypes ----------------------------------------------*/
static void STO_CR_Store_Rotor_Speed(STO_CR_Handle_t *pHandle, int16_t hRotor_Speed, int16_t hOrRotor_Speed);
static void STO_CR_InitSpeedBuffer(STO_CR_Handle_t *pHandle);

/**
  * @brief  It initializes the state observer object
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  * @retval none
  */
__weak void STO_CR_Init(STO_CR_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  if (NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    int32_t wAux;
    int16_t htempk;

    pHandle->ConsistencyCounter = pHandle->StartUpConsistThreshold;
    pHandle->EnableDualCheck = true;

    wAux = (int32_t)1;
    pHandle->F3POW2 = 0u;

    htempk = (int16_t)(C6_COMP_CONST1 / (pHandle->hF2));

    while (htempk != 0)
    {
      htempk /= (int16_t)2;
      wAux *= (int32_t)2;
      pHandle->F3POW2++;
    }

    pHandle->hF3 = (int16_t)wAux;
    wAux = (int32_t)(pHandle->hF2) * pHandle->hF3;
    pHandle->hC6 = (int16_t)(wAux / C6_COMP_CONST2);

    STO_CR_Clear(pHandle);

    /* Acceleration measurement set to zero */
    pHandle->_Super.hMecAccelUnitP = 0;
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  }
#endif
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
  * @brief  This method executes Luenberger state observer equations and calls
  *         CORDIC with the purpose of computing a new speed estimation and
  *         updating the estimated electrical angle.
  * @param  pHandle: handler of the current instance of the STO CORDIC component.
  *         pInputs pointer to the observer inputs structure.
  * @retval int16_t rotor electrical angle (s16Degrees)
  */
//cstat !MISRAC2012-Rule-8.13
__weak int16_t STO_CR_CalcElAngle(STO_CR_Handle_t *pHandle, Observer_Inputs_t *pInputs)
{
  int16_t returnvalue;
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  if ((NULL == pHandle) || (NULL == pInputs))
  {
    returnvalue = 0;
  }
  else
  {
#endif
    int32_t wAux;
    int32_t wDirection;
    int32_t wAux_Alpha;
    int32_t wAux_Beta;
    int32_t wIalfa_est_Next;
    int32_t wIbeta_est_Next;
    int32_t wBemf_alfa_est_Next;
    int32_t wBemf_beta_est_Next;

    int16_t hAux;
    int16_t hAux_Alfa;
    int16_t hAux_Beta;
    int16_t hIalfa_err;
    int16_t hIbeta_err;
    int16_t hRotor_Speed;
    int16_t hOrRotor_Speed;
    int16_t hRotor_Acceleration;
    int16_t hRotor_Angle;
    int16_t hValfa;
    int16_t hVbeta;

    int16_t hPrev_Rotor_Angle = pHandle->_Super.hElAngle;
    int16_t hPrev_Rotor_Speed = pHandle->_Super.hElSpeedDpp;
    int16_t hMax_Instant_Accel = pHandle->MaxInstantElAcceleration;

    if (pHandle->wBemf_alfa_est > ((int32_t)pHandle->hF2 * INT16_MAX))
    {
      pHandle->wBemf_alfa_est = INT16_MAX * (int32_t)(pHandle->hF2);
    }
    else if (pHandle->wBemf_alfa_est <= (-INT16_MAX * (int32_t)(pHandle->hF2)))
    {
      pHandle->wBemf_alfa_est = -INT16_MAX * (int32_t)(pHandle->hF2);
    }
    else
    {
      /* Nothing to do */
    }
#ifndef FULL_MISRA_C_COMPLIANCY_STO_CORDIC
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
      hAux_Alfa = (int16_t)(pHandle->wBemf_alfa_est >> pHandle->F2LOG);
#else
      hAux_Alfa = (int16_t)(pHandle->wBemf_alfa_est / pHandle->hF2);
#endif

    if (pHandle->wBemf_beta_est > (INT16_MAX * (int32_t)(pHandle->hF2)))
    {
      pHandle->wBemf_beta_est = INT16_MAX * (int32_t)(pHandle->hF2);
    }
    else if (pHandle->wBemf_beta_est <= (-INT16_MAX * (int32_t)(pHandle->hF2)))
    {
      pHandle->wBemf_beta_est = -INT16_MAX * (int32_t)(pHandle->hF2);
    }
    else
    {
      /* Nothing to do */
    }
#ifndef FULL_MISRA_C_COMPLIANCY_STO_CORDIC
      //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
      hAux_Beta = (int16_t)(pHandle->wBemf_beta_est >> pHandle->F2LOG);
#else
      hAux_Beta = (int16_t)(pHandle->wBemf_beta_est / pHandle->hF2);
#endif

    if (pHandle->Ialfa_est > (INT16_MAX * (int32_t)(pHandle->hF1)))
    {
      pHandle->Ialfa_est = INT16_MAX * (int32_t)(pHandle->hF1);
    }
    else if (pHandle->Ialfa_est <= (-INT16_MAX * (int32_t)(pHandle->hF1)))
    {
      pHandle->Ialfa_est = -INT16_MAX * (int32_t)(pHandle->hF1);
    }
    else
    {
      /* Nothing to do */
    }

    if (pHandle->Ibeta_est > (INT16_MAX * (int32_t)(pHandle->hF1)))
    {
      pHandle->Ibeta_est = INT16_MAX * (int32_t)(pHandle->hF1);
    }
    else if (pHandle->Ibeta_est <= (-INT16_MAX * (int32_t)(pHandle->hF1)))
    {
      pHandle->Ibeta_est = -INT16_MAX * (int32_t)(pHandle->hF1);
    }
    else
    {
      /* Nothing to do */
    }

#ifndef FULL_MISRA_C_COMPLIANCY_STO_CORDIC
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
    hIalfa_err = (int16_t)(pHandle->Ialfa_est >> pHandle->F1LOG);
#else
    hIalfa_err = (int16_t)(pHandle->Ialfa_est / pHandle->hF1);
#endif

    hIalfa_err = hIalfa_err - pInputs->Ialfa_beta.alpha;

#ifndef FULL_MISRA_C_COMPLIANCY_STO_CORDIC
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
    hIbeta_err = (int16_t)(pHandle->Ibeta_est >> pHandle->F1LOG);
#else
    hIbeta_err = (int16_t)(pHandle->Ibeta_est / pHandle->hF1);
#endif

    hIbeta_err = hIbeta_err - pInputs->Ialfa_beta.beta;

    wAux = (int32_t)(pInputs->Vbus) * pInputs->Valfa_beta.alpha;
#ifndef FULL_MISRA_C_COMPLIANCY_STO_CORDIC
    hValfa = (int16_t)(wAux >> 16); //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
#else
    hValfa = (int16_t)(wAux / 65536);
#endif

    wAux = (int32_t)(pInputs->Vbus) * pInputs->Valfa_beta.beta;

#ifndef FULL_MISRA_C_COMPLIANCY_STO_CORDIC
    hVbeta = (int16_t)(wAux >> 16); //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
#else
    hVbeta = (int16_t)(wAux / 65536);
#endif

    /* Alfa axes observer */
#ifndef FULL_MISRA_C_COMPLIANCY_STO_CORDIC
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
    hAux = (int16_t)(pHandle->Ialfa_est >> pHandle->F1LOG);
#else
    hAux = (int16_t)(pHandle->Ialfa_est / pHandle->hF1);
#endif

    wAux = (int32_t)(pHandle->hC1) * hAux;
    wIalfa_est_Next = pHandle->Ialfa_est - wAux;

    wAux = (int32_t)(pHandle->hC2) * hIalfa_err;
    wIalfa_est_Next += wAux;

    wAux = (int32_t)(pHandle->hC5) * hValfa;
    wIalfa_est_Next += wAux;

    wAux = (int32_t)(pHandle->hC3) * hAux_Alfa;
    wIalfa_est_Next -= wAux;

    wAux = (int32_t)(pHandle->hC4) * hIalfa_err;
    wBemf_alfa_est_Next = pHandle->wBemf_alfa_est + wAux;

#ifndef FULL_MISRA_C_COMPLIANCY_STO_CORDIC
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
    wAux = (int32_t) hAux_Beta >> pHandle->F3POW2;
#else
    wAux = (int32_t)hAux_Beta / pHandle->hF3;
#endif

    wAux = wAux * pHandle->hC6;
    wAux = hPrev_Rotor_Speed * wAux;
    wBemf_alfa_est_Next += wAux;

    /* Beta axes observer */
#ifndef FULL_MISRA_C_COMPLIANCY_STO_CORDIC
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
    hAux = (int16_t)(pHandle->Ibeta_est >> pHandle->F1LOG);
#else
    hAux = (int16_t)(pHandle->Ibeta_est / pHandle->hF1);
#endif

    wAux = (int32_t)(pHandle->hC1) * hAux;
    wIbeta_est_Next = pHandle->Ibeta_est - wAux;

    wAux = (int32_t)(pHandle->hC2) * hIbeta_err;
    wIbeta_est_Next += wAux;

    wAux = (int32_t)(pHandle->hC5) * hVbeta;
    wIbeta_est_Next += wAux;

    wAux = (int32_t)(pHandle->hC3) * hAux_Beta;
    wIbeta_est_Next -= wAux;

    wAux = (int32_t)(pHandle->hC4) * hIbeta_err;
    wBemf_beta_est_Next = pHandle->wBemf_beta_est + wAux;

#ifndef FULL_MISRA_C_COMPLIANCY_STO_CORDIC
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
    wAux = (int32_t) hAux_Alfa >> pHandle->F3POW2;
#else
    wAux = (int32_t)hAux_Alfa / pHandle->hF3;
#endif

    wAux = wAux * pHandle->hC6;
    wAux = hPrev_Rotor_Speed * wAux;
    wBemf_beta_est_Next -= wAux;

    if (pHandle->Orig_ElSpeedDpp >= 0)
    {
      wDirection = 1;
    }
    else
    {
      wDirection = -1;
    }

    /* Stores observed b-emfs */
    pHandle->hBemf_alfa_est = hAux_Alfa;
    pHandle->hBemf_beta_est = hAux_Beta;

    /* Calls the CORDIC blockset */
    wAux_Alpha = pHandle->wBemf_alfa_est * wDirection;
    wAux_Beta = pHandle->wBemf_beta_est * wDirection;

    hRotor_Angle = MCM_PhaseComputation(wAux_Alpha, -wAux_Beta);

    hOrRotor_Speed = (int16_t)(hRotor_Angle - hPrev_Rotor_Angle);
    hRotor_Acceleration = hOrRotor_Speed - hPrev_Rotor_Speed;

    hRotor_Speed = hOrRotor_Speed;

    if (wDirection == 1)
    {
      if (hRotor_Speed < 0)
      {
        hRotor_Speed = 0;
      }
      else
      {
        if (hRotor_Acceleration > hMax_Instant_Accel)
        {
          hRotor_Speed = hPrev_Rotor_Speed + hMax_Instant_Accel;

          pHandle->_Super.hElAngle = hPrev_Rotor_Angle + hRotor_Speed;
        }
        else
        {
          pHandle->_Super.hElAngle = hRotor_Angle;
        }
      }
    }
    else
    {
      if (hRotor_Speed > 0)
      {
        hRotor_Speed = 0;
      }
      else
      {
        if (hRotor_Acceleration < (-hMax_Instant_Accel))
        {
          hRotor_Speed = hPrev_Rotor_Speed - hMax_Instant_Accel;

          pHandle->_Super.hElAngle = hPrev_Rotor_Angle + hRotor_Speed;
        }
        else
        {
          pHandle->_Super.hElAngle = hRotor_Angle;
        }
      }
    }


    if (hRotor_Acceleration > hMax_Instant_Accel)
    {
      hOrRotor_Speed = hPrev_Rotor_Speed + hMax_Instant_Accel;
    }
    else if (hRotor_Acceleration < (-hMax_Instant_Accel))
    {
      hOrRotor_Speed = hPrev_Rotor_Speed - hMax_Instant_Accel;
    }
    else
    {
      /* Nothing to do */
    }


    STO_CR_Store_Rotor_Speed(pHandle, hRotor_Speed, hOrRotor_Speed);

    /* Storing previous values of currents and bemfs */
    pHandle->Ialfa_est = wIalfa_est_Next;
    pHandle->wBemf_alfa_est = wBemf_alfa_est_Next;

    pHandle->Ibeta_est = wIbeta_est_Next;
    pHandle->wBemf_beta_est = wBemf_beta_est_Next;
    returnvalue = pHandle->_Super.hElAngle;
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  }
#endif
  return (returnvalue);
}

/**
  * @brief Computes the rotor average mechanical speed in the unit defined by
  *        #SPEED_UNIT and writes it in pMecSpeedUnit
  *
  *  This method must be called - at least - with the same periodicity
  * on which the speed control is executed. It computes and returns - through
  * parameter pMecSpeedUnit - the rotor average mechanical speed, expressed in
  * the unit defined by #SPEED_UNIT. Average is computed considering a FIFO depth
  * equal to STO_CR_Handle_t::SpeedBufferSizeUnit.
  *
  * Moreover it also computes and returns the reliability state of the sensor,
  * measured with reference to parameters STO_CR_Handle_t::Reliability_hysteresys,
  * STO_CR_Handle_t::VariancePercentage and STO_CR_Handle_t::SpeedBufferSizeUnit of
  * the STO_CR_Handle_t handle.
  *
  * @param  pHandle handler of the current instance of the STO CORDIC component
  * @param  pMecSpeedUnit pointer to int16_t, used to return the rotor average
  *         mechanical speed
  * @retval true if the sensor information is reliable, false otherwise
  */

__weak bool STO_CR_CalcAvrgMecSpeedUnit(STO_CR_Handle_t *pHandle, int16_t *pMecSpeedUnit)
{
  bool bAux = false;
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  if ((NULL == pHandle) || (NULL == pMecSpeedUnit))
  {
    /* Nothing to do */
  }
  else
  {
#endif
    int32_t wAvrSpeed_dpp = (int32_t)0;
    int32_t wError;
    int32_t wAux;
    int32_t wAvrSquareSpeed;
    int32_t wAvrQuadraticError = 0;
    int32_t wObsBemf, wEstBemf;
    int32_t wObsBemfSq = 0;
    int32_t wEstBemfSq = 0;
    int32_t wEstBemfSqLo;

    uint8_t i;
    uint8_t bSpeedBufferSizeUnit = pHandle->SpeedBufferSizeUnit;
    bool bIs_Speed_Reliable = false;
    bool bIs_Bemf_Consistent = false;

    for (i = 0U; i < bSpeedBufferSizeUnit; i++)
    {
      wAvrSpeed_dpp += (int32_t)(pHandle->Speed_Buffer[i]);
    }

    if ((uint8_t)0 == bSpeedBufferSizeUnit)
    {
      /* Nothing to do */
    }
    else
    {
      wAvrSpeed_dpp = wAvrSpeed_dpp / (int16_t)bSpeedBufferSizeUnit;
    }


    /* Value is written during init and computed by MC Workbench and always >= 1 */
    //cstat -MISRAC2012-Rule-1.3_d
    for (i = 0U; i < bSpeedBufferSizeUnit; i++)
    {
      wError = (int32_t)(pHandle->Speed_Buffer[i]) - wAvrSpeed_dpp;
      wError = (wError * wError);
      wAvrQuadraticError += wError;
    }


    /* It computes the measurement variance */
    wAvrQuadraticError = wAvrQuadraticError / (int16_t)bSpeedBufferSizeUnit;
    //cstat +MISRAC2012-Rule-1.3_d
    /* The maximum variance acceptable is here calculated as a function of average speed */
    wAvrSquareSpeed = wAvrSpeed_dpp * wAvrSpeed_dpp;
    int64_t lAvrSquareSpeed = (int64_t)(wAvrSquareSpeed) * (int64_t)pHandle->VariancePercentage;
    wAvrSquareSpeed = (int32_t)(lAvrSquareSpeed / (int64_t)128);
    
    if (wAvrQuadraticError < wAvrSquareSpeed)
    {
      bIs_Speed_Reliable = true;
    }
    else
    {
      /* Nothing to do */
    }

    /* Computation of Mechanical speed unit */
    wAux = wAvrSpeed_dpp * (int32_t)(pHandle->_Super.hMeasurementFrequency);
    wAux = wAux * (int32_t) (pHandle->_Super.SpeedUnit);
    wAux = wAux / (int32_t)(pHandle->_Super.DPPConvFactor);
    wAux = wAux / (int16_t)(pHandle->_Super.bElToMecRatio);

    *pMecSpeedUnit = (int16_t)wAux;
    pHandle->_Super.hAvrMecSpeedUnit = (int16_t)wAux;

    pHandle->IsSpeedReliable = bIs_Speed_Reliable;

    /* Bemf Consistency Check algorithm */
    if (true == pHandle->EnableDualCheck) /*do algorithm if it's enabled*/
    {
      /* wAux abs value */
      /* False positive */
      wAux = ((wAux < 0) ? (-wAux) : (wAux));  //cstat !MISRAC2012-Rule-14.3_b !RED-cond-never !RED-cmp-never

      if (wAux < (int32_t)(pHandle->MaxAppPositiveMecSpeedUnit))
      {
        /* Computation of Observed back-emf */
        wObsBemf = (int32_t)(pHandle->hBemf_alfa_est);
        wObsBemfSq = wObsBemf * wObsBemf;
        wObsBemf = (int32_t)(pHandle->hBemf_beta_est);
        wObsBemfSq += wObsBemf * wObsBemf;

        /* Computation of Estimated back-emf */
        wEstBemf = (wAux * 32767) / (int16_t)(pHandle->_Super.hMaxReliableMecSpeedUnit);
        wEstBemfSq = (wEstBemf * (int32_t)(pHandle->BemfConsistencyGain)) / 64;
        wEstBemfSq *= wEstBemf;

        /* Computation of threshold */
        wEstBemfSqLo = wEstBemfSq - ((wEstBemfSq / 64) * (int32_t)(pHandle->BemfConsistencyCheck));

        /* Check */
        if (wObsBemfSq > wEstBemfSqLo)
        {
          bIs_Bemf_Consistent = true;
        }
        else
        {
          /* Nothing to do */
        }
      }
      else
      {
        /* Nothing to do */
      }

      pHandle->IsBemfConsistent = bIs_Bemf_Consistent;
      pHandle->Obs_Bemf_Level = wObsBemfSq;
      pHandle->Est_Bemf_Level = wEstBemfSq;
    }
    else
    {
      bIs_Bemf_Consistent = true;
    }

    /* Decision making */
    if (pHandle->IsAlgorithmConverged == false)
    {
      bAux = SPD_IsMecSpeedReliable(&pHandle->_Super, pMecSpeedUnit);
    }
    else
    {
      if ((pHandle->IsSpeedReliable == false) || (bIs_Bemf_Consistent == false))
      {
        pHandle->ReliabilityCounter++;
        if (pHandle->ReliabilityCounter >= pHandle->Reliability_hysteresys)
        {
          pHandle->ReliabilityCounter = 0U;
          pHandle->_Super.bSpeedErrorNumber = pHandle->_Super.bMaximumSpeedErrorsNumber;
          bAux = false;
        }
        else
        {
          bAux = SPD_IsMecSpeedReliable (&pHandle->_Super, pMecSpeedUnit);
        }
      }
      else
      {
        pHandle->ReliabilityCounter = 0U;
        bAux = SPD_IsMecSpeedReliable (&pHandle->_Super, pMecSpeedUnit);
      }
    }
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  }
#endif
  return (bAux);
}

/**
  * @brief  It clears state observer object by re-initializing private variables
  * @param  pHandle pointer on the component instance to work on.
  */
__weak void STO_CR_Clear(STO_CR_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  if (NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->Ialfa_est = (int32_t)0;
    pHandle->Ibeta_est = (int32_t)0;
    pHandle->wBemf_alfa_est = (int32_t)0;
    pHandle->wBemf_beta_est = (int32_t)0;
    pHandle->_Super.hElAngle = (int16_t)0;
    pHandle->_Super.hElSpeedDpp = (int16_t)0;
    pHandle->Orig_ElSpeedDpp = (int16_t)0;
    pHandle->ConsistencyCounter = 0u;
    pHandle->ReliabilityCounter = 0u;
    pHandle->IsAlgorithmConverged = false;
    pHandle->IsBemfConsistent = false;
    pHandle->Obs_Bemf_Level = (int32_t)0;
    pHandle->Est_Bemf_Level = (int32_t)0;
    pHandle->DppBufferSum = (int32_t)0;
    pHandle->DppOrigBufferSum = (int32_t)0;
    pHandle->ForceConvergency = false;
    pHandle->ForceConvergency2 = false;

   STO_CR_InitSpeedBuffer(pHandle);
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  It stores in estimated speed FIFO latest calculated value of motor
  *         speed
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  * @retval none
  */
inline static void STO_CR_Store_Rotor_Speed(STO_CR_Handle_t *pHandle, int16_t hRotor_Speed, int16_t hOrRotor_Speed)
{
  uint8_t bBuffer_index = pHandle->Speed_Buffer_Index;

  bBuffer_index++;
  if (bBuffer_index == pHandle->SpeedBufferSizeUnit)
  {
    bBuffer_index = 0U;
  }
  else
  {
    /* Nothing too do */
  }

  pHandle->SpeedBufferOldestEl = pHandle->Speed_Buffer[bBuffer_index];
  pHandle->OrigSpeedBufferOldestEl = pHandle->Orig_Speed_Buffer[bBuffer_index];

  pHandle->Speed_Buffer[bBuffer_index] = hRotor_Speed;
  pHandle->Orig_Speed_Buffer[bBuffer_index] = hOrRotor_Speed;
  pHandle->Speed_Buffer_Index = bBuffer_index;
}


/**
  * @brief  It clears the estimated speed buffer
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  * @retval none
  */
static void STO_CR_InitSpeedBuffer(STO_CR_Handle_t *pHandle)
{
  uint8_t b_i;
  uint8_t bSpeedBufferSizeUnit = pHandle->SpeedBufferSizeUnit;

  /*init speed buffer*/
  for (b_i = 0U; b_i < bSpeedBufferSizeUnit; b_i++)
  {
    pHandle->Speed_Buffer[b_i] = (int16_t)0;
    pHandle->Orig_Speed_Buffer[b_i] = (int16_t)0;
  }

  pHandle->Speed_Buffer_Index = 0u;
  pHandle->SpeedBufferOldestEl = (int16_t)0;
  pHandle->OrigSpeedBufferOldestEl = (int16_t)0;
}


/**
  * @brief Returns true if the Observer has converged or false otherwise.
  *
  *  Internally performs a set of checks necessary to state whether
  * the state observer algorithm has converged or not.
  *
  * This function is to be periodically called during the motor rev-up procedure
  * at the same frequency as the *_CalcElAngle() functions.
  *
  * It returns true if the estimated angle and speed can be considered reliable,
  * false otherwise.
  *
  * @param  pHandle pointer on the component instance
  * @param  hForcedMecSpeedUnit Mechanical speed as forced by VSS, in the unit defined by #SPEED_UNIT
  */
__weak bool STO_CR_IsObserverConverged(STO_CR_Handle_t *pHandle, int16_t hForcedMecSpeedUnit)
{
  bool bAux = false;
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  if (NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    int32_t wAux;
    int32_t wtemp;
    int16_t hEstimatedSpeedUnit;
    int16_t hUpperThreshold;
    int16_t hLowerThreshold;
    int16_t lForcedMecSpeedUnit;


    if (pHandle->ForceConvergency2 == true)
    {
      lForcedMecSpeedUnit = pHandle->_Super.hAvrMecSpeedUnit;
    }
    else
    {
      lForcedMecSpeedUnit = hForcedMecSpeedUnit;
    }

    if (pHandle->ForceConvergency == true)
    {
      bAux = true;
      pHandle->IsAlgorithmConverged = true;
      pHandle->_Super.bSpeedErrorNumber = 0U;
    }
    else
    {
      hEstimatedSpeedUnit = pHandle->_Super.hAvrMecSpeedUnit;
      wtemp = (int32_t)hEstimatedSpeedUnit * (int32_t)lForcedMecSpeedUnit;

      if (wtemp > 0)
      {
        if (hEstimatedSpeedUnit < 0)
        {
          hEstimatedSpeedUnit = -hEstimatedSpeedUnit;
        }
        else
        {
          /* Nothing to do */
        }

        if (lForcedMecSpeedUnit < 0)
        {
          lForcedMecSpeedUnit = -lForcedMecSpeedUnit;
        }
        else
        {
          /* Nothing to do */
        }

        wAux = (int32_t) (lForcedMecSpeedUnit) * (int16_t)pHandle->SpeedValidationBand_H;
        hUpperThreshold = (int16_t)(wAux / (int32_t)16);

        wAux = (int32_t) (lForcedMecSpeedUnit) * (int16_t)pHandle->SpeedValidationBand_L;
        hLowerThreshold = (int16_t)(wAux / (int32_t)16);

        /* If the variance of the estimated speed is low enough... */
        if (true == pHandle->IsSpeedReliable)
        {
          if ((uint16_t)hEstimatedSpeedUnit > pHandle->MinStartUpValidSpeed)
          {
            /* ...and the estimated value is quite close to the expected value... */
            if (hEstimatedSpeedUnit >= hLowerThreshold)
            {
              if (hEstimatedSpeedUnit <= hUpperThreshold)
              {
                pHandle->ConsistencyCounter++;

                /* ...for hConsistencyThreshold consecutive times... */
                if (pHandle->ConsistencyCounter >= pHandle->StartUpConsistThreshold)
                {
                  /* The algorithm converged */
                  bAux = true;
                  pHandle->IsAlgorithmConverged = true;
                  pHandle->_Super.bSpeedErrorNumber = 0U;
                }
              }
              else
              {
                pHandle->ConsistencyCounter = 0U;
              }
            }
            else
            {
              pHandle->ConsistencyCounter = 0U;
            }
          }
          else
          {
            pHandle->ConsistencyCounter = 0U;
          }
        }
        else
        {
          pHandle->ConsistencyCounter = 0U;
        }
      }
    }
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  }
#endif
  return (bAux);
}

  /* false positive */
  //cstat -MISRAC2012-Rule-2.2_b
/**
  * @brief  It exports estimated Bemf alpha-beta in qd_t format
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  * @retval alphabeta_t Bemf alpha-beta
  */
//cstat !MISRAC2012-Rule-8.13
__weak alphabeta_t STO_CR_GetEstimatedBemf(STO_CR_Handle_t *pHandle)
{
  alphabeta_t Vaux;
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  if (NULL == pHandle)
  {
    Vaux.alpha = 0;
    Vaux.beta = 0;
  }
  else
  {
#endif
    Vaux.alpha = pHandle->hBemf_alfa_est;
    Vaux.beta = pHandle->hBemf_beta_est;
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  }
#endif
  return (Vaux);
}

/**
  * @brief  It exports the stator current alpha-beta as estimated by state
  *         observer
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  * @retval alphabeta_t State observer estimated stator current Ialpha-beta
  */
//cstat !MISRAC2012-Rule-8.13
__weak alphabeta_t STO_CR_GetEstimatedCurrent(STO_CR_Handle_t *pHandle)
{
  alphabeta_t Iaux;
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  if (NULL == pHandle)
  {
    Iaux.alpha = 0;
    Iaux.beta = 0;
  }
  else
  {
#endif
#ifndef FULL_MISRA_C_COMPLIANCY_STO_CORDIC
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
    Iaux.alpha = (int16_t)(pHandle->Ialfa_est >> pHandle->F1LOG);
#else
    Iaux.alpha = (int16_t)(pHandle->Ialfa_est / (pHandle->hF1));
#endif

#ifndef FULL_MISRA_C_COMPLIANCY_STO_CORDIC
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
    Iaux.beta = (int16_t)(pHandle->Ibeta_est >> pHandle->F1LOG);
#else
    Iaux.beta = (int16_t)(pHandle->Ibeta_est / (pHandle->hF1));
#endif
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  }
#endif
  return (Iaux);
}
//cstat +MISRAC2012-Rule-2.2_b

/**
  * @brief  It exports current observer gains through parameters hhC2 and hhC4
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  * @param  phC2 pointer to int16_t used to return parameters hhC2
  * @param  phC4 pointer to int16_t used to return parameters hhC4
  * @retval none
  */
__weak void STO_CR_GetObserverGains(STO_CR_Handle_t *pHandle, int16_t *phC2, int16_t *phC4)
{
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  if ((NULL == pHandle) || (NULL == phC2) || (NULL == phC4))
  {
    /* Nothing to do */
  }
  else
  {
#endif
    *phC2 = pHandle->hC2;
    *phC4 = pHandle->hC4;
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  It allows setting new values for observer gains
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  * @param  wK1 new value for observer gain hhC1
  * @param  wK2 new value for observer gain hhC2
  * @retval none
  */
__weak void STO_CR_SetObserverGains(STO_CR_Handle_t *pHandle, int16_t hhC1, int16_t hhC2)
{
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  if (NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->hC2 = hhC1;
    pHandle->hC4 = hhC2;
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  }
#endif
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
  * @brief  This method must be called - at least - with the same periodicity
  *         on which speed control is executed. It computes and update object
  *         variable hElSpeedDpp that is estimated average electrical speed
  *         expressed in dpp used for instance in observer equations.
  *         Average is computed considering a FIFO depth equal to
  *         bSpeedBufferSizedpp.
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  * @retval none
  */
__weak void STO_CR_CalcAvrgElSpeedDpp(STO_CR_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  if (NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    int32_t wSum = pHandle->DppBufferSum;
    int32_t wSumOrig = pHandle->DppOrigBufferSum;
    int32_t wAvrSpeed_dpp;
    int16_t hSpeedBufferSizedpp = (int16_t)(pHandle->SpeedBufferSizedpp);
    int16_t hSpeedBufferSizeUnit = (int16_t)(pHandle->SpeedBufferSizeUnit);
    int16_t hBufferSizeDiff;
    int16_t hIndexNew = (int16_t)pHandle->Speed_Buffer_Index;
    int16_t hIndexOld;
    int16_t hIndexOldTemp;

    hBufferSizeDiff = hSpeedBufferSizeUnit - hSpeedBufferSizedpp;

    if (0 == hBufferSizeDiff)
    {
      wSum = wSum + pHandle->Speed_Buffer[hIndexNew] - pHandle->SpeedBufferOldestEl;
      wSumOrig = wSumOrig + pHandle->Orig_Speed_Buffer[hIndexNew] - pHandle->OrigSpeedBufferOldestEl;
    }
    else
    {
      hIndexOldTemp = hIndexNew + hBufferSizeDiff;

      if (hIndexOldTemp >= hSpeedBufferSizeUnit)
      {
        hIndexOld = hIndexOldTemp - hSpeedBufferSizeUnit;
      }
      else
      {
        hIndexOld = hIndexOldTemp;
      }

      wSum = wSum + pHandle->Speed_Buffer[hIndexNew] - pHandle->Speed_Buffer[hIndexOld];

      wSumOrig = wSumOrig + pHandle->Orig_Speed_Buffer[hIndexNew] - pHandle->Orig_Speed_Buffer[hIndexOld];
    }

#ifndef FULL_MISRA_C_COMPLIANCY_STO_CORDIC
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
    wAvrSpeed_dpp = (int32_t)(wSum >> pHandle->SpeedBufferSizedppLOG);
    pHandle->_Super.hElSpeedDpp = (int16_t)wAvrSpeed_dpp;
    //cstat !MISRAC2012-Rule-1.3_n !ATH-shift-neg !MISRAC2012-Rule-10.1_R6
    wAvrSpeed_dpp = (int32_t)(wSumOrig >> pHandle->SpeedBufferSizedppLOG);
#else
    if (hSpeedBufferSizedpp != 0)
    {
      wAvrSpeed_dpp = wSum / hSpeedBufferSizedpp;
      pHandle->_Super.hElSpeedDpp = (int16_t)wAvrSpeed_dpp;
      wAvrSpeed_dpp = wSumOrig / hSpeedBufferSizedpp;
    }
    else
    {
      wAvrSpeed_dpp = (int32_t)0;
    }
#endif

    pHandle->Orig_ElSpeedDpp = (int16_t)wAvrSpeed_dpp;

    pHandle->DppBufferSum = wSum;

    pHandle->DppOrigBufferSum = wSumOrig;
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  It exports estimated Bemf squared level
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  * @retval int32_t
  */
__weak int32_t STO_CR_GetEstimatedBemfLevel(const STO_CR_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  return ((NULL == pHandle) ? 0 : pHandle->Est_Bemf_Level);
#else
  return (pHandle->Est_Bemf_Level);
#endif
}

/**
  * @brief  It exports observed Bemf squared level
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  * @retval int32_t
  */
__weak int32_t STO_CR_GetObservedBemfLevel(const STO_CR_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  return ((NULL == pHandle) ? 0 : pHandle->Obs_Bemf_Level);
#else
  return (pHandle->Obs_Bemf_Level);
#endif
}

/**
  * @brief  It enables/disables the bemf consistency check
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  * @param  bSel boolean; true enables check; false disables check
  */
__weak void STO_CR_BemfConsistencyCheckSwitch(STO_CR_Handle_t *pHandle, bool bSel)
{
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  if (NULL == pHandle)
  {
    /* Noything to do */
  }
  else
  {
#endif
    pHandle->EnableDualCheck = bSel;
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  }
#endif
}

/**
  * @brief  It returns the result of the Bemf consistency check
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  * @retval bool Bemf consistency state
  */
__weak bool STO_CR_IsBemfConsistent(const STO_CR_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  return ((NULL == pHandle) ? false : pHandle->IsBemfConsistent);
#else
  return (pHandle->IsBemfConsistent);
#endif
}

/**
  * @brief  This method returns the reliability of the speed sensor
  * @param  pHandle: handler of the current instance of the STO CORDIC component
  * @retval bool speed sensor reliability, measured with reference to parameters
  *         bReliability_hysteresys, hVariancePercentage and bSpeedBufferSize
  *         true = sensor information is reliable
  *         false = sensor information is not reliable
  */
__weak bool STO_CR_IsSpeedReliable(const STO_Handle_t *pHandle)
{
  bool returnbool;
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  if (NULL == pHandle)
  {
    returnbool = false;
  }
  else
  {
#endif
    const  STO_CR_Handle_t *pHdl = (STO_CR_Handle_t *)pHandle->_Super; //cstat !MISRAC2012-Rule-11.3
    returnbool = pHdl->IsSpeedReliable;
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  }
#endif
  return (returnbool);
}

/* @brief  It forces the state-observer to declare converged
 * @param  pHandle: handler of the current instance of the STO CORDIC component
 */
__weak void STO_CR_ForceConvergency1(STO_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  if (NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    STO_CR_Handle_t *pHdl = (STO_CR_Handle_t *)pHandle->_Super; //cstat !MISRAC2012-Rule-11.3
  
    pHdl->ForceConvergency = true;
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  }
#endif
}

/* @brief  It forces the state-observer to declare converged
 * @param  pHandle: handler of the current instance of the STO CORDIC component
 */
__weak void STO_CR_ForceConvergency2(STO_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  if (NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    STO_CR_Handle_t *pHdl = (STO_CR_Handle_t *)pHandle->_Super; //cstat !MISRAC2012-Rule-11.3
  
    pHdl->ForceConvergency2 = true;
#ifdef NULL_PTR_CHECK_STO_COR_SPD_POS_FDB
  }
#endif
}

/**
  * @}
  */

/**
  * @}
  */

/** @} */
/************************ (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
