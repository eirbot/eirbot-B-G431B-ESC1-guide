/**
  ******************************************************************************
  * @file    speed_pos_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the  features
  *          of the Speed & Position Feedback component of the Motor Control SDK.
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
#include "speed_pos_fdbk.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup SpeednPosFdbk Speed & Position Feedback
  *
  * @brief Speed & Position Feedback components of the Motor Control SDK
  *
  * These components provide the speed and the angular position of the rotor of a motor (both
  * electrical and mechanical). These informations are expressed in units defined into [measurement units](measurement_units.md).
  *
  * Several implementations of the Speed and Position Feedback feature are provided by the Motor
  * to account for the specificities of the motor used on the application:
  *
  * - @ref hall_speed_pos_fdbk "Hall Speed & Position Feedback" for motors with Hall effect sensors.
  * - @ref Encoder "Encoder Speed & Position Feedback" for motors with a quadrature encoder.
  * - Two general purpose sensorless implementations are provided:
  *   @ref SpeednPosFdbk_STO "State Observer with PLL" and
  *   @ref STO_CORDIC_SpeednPosFdbk "State Observer with CORDIC"
  * - A @ref VirtualSpeedSensor "Virtual Speed & Position Feedback" implementation used during the
  *   @ref RevUpCtrl "Rev-Up Control" phases of the motor in a sensorless subsystem.
  * - @ref In the future a High Frequency Injection for anisotropic I-PMSM motors will be supported.
  *
  *   For more information see the user manual [Speed position and sensorless BEMF reconstruction](speed_pos_sensorless_bemf_reconstruction.md).
  * @{
  */

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
  * @brief  Returns the last computed rotor electrical angle, expressed in [s16degrees](measurement_units.md).
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component.
  * @retval int16_t rotor electrical angle.
  */
__weak int16_t SPD_GetElAngle(const SpeednPosFdbk_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_SPD_POS_FBK
  return ((MC_NULL == pHandle) ? 0 : pHandle->hElAngle);
#else
  return (pHandle->hElAngle);
#endif
}

/**
  * @brief  Returns the last computed rotor mechanical angle, expressed in [s16degrees](measurement_units.md).
  * @note   Both Hall sensor and Sensor-less application do not implement either mechanical angle computation or
  *         acceleration computation.
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component.
  * @retval int16_t rotor mechanical angle.
  *
  * - Mechanical angle frame is based on parameter @ref SpeednPosFdbk_Handle_t::bElToMecRatio "bElToMecRatio"
  * and, if occasionally provided through Encoder function of a measured mechanical angle, on information computed
  * thereof.
  * - Called to set a mechanical position ot the rotor.
  */
__weak int32_t SPD_GetMecAngle(const SpeednPosFdbk_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_SPD_POS_FBK
  return ((MC_NULL == pHandle) ? 0 : pHandle->wMecAngle);
#else
  return (pHandle->wMecAngle);
#endif
}

/**
  * @brief  Returns the last computed average mechanical speed, expressed in
  *         the unit defined by [SPEED_UNIT](measurement_units.md).
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component.
  */
__weak int16_t SPD_GetAvrgMecSpeedUnit(const SpeednPosFdbk_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_SPD_POS_FBK
  return ((MC_NULL == pHandle) ? 0 : pHandle->hAvrMecSpeedUnit);
#else
  return (pHandle->hAvrMecSpeedUnit);
#endif
}

/**
  * @brief  Returns the last computed electrical speed, expressed in [dpp](measurement_units.md).
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component.
  * @retval int16_t rotor electrical speed ([dpp](measurement_units.md)).
  *
  * - The control period is the period on which the rotor electrical angle is computed thanks
  *   to HALL effect sensor functions.
  * - Called during feed-forward controller computation and during Motor profiling.
  */
__weak int16_t SPD_GetElSpeedDpp(const SpeednPosFdbk_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_SPD_POS_FBK
  return ((MC_NULL == pHandle) ? 0 : pHandle->hElSpeedDpp);
#else
  return (pHandle->hElSpeedDpp);
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
  * @brief  Returns the last instantaneous computed electrical speed, expressed in [dpp](measurement_units.md).
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component.
  * @retval int16_t rotor instantaneous electrical speed ([dpp](measurement_units.md)).
  *
  * - The control period is the period on which the rotor electrical angle is computed thanks to HALL effect sensor
  *   functions.
  * - Called during FOC drive control for Iqd currents regulation.
  */
__weak int16_t SPD_GetInstElSpeedDpp(const SpeednPosFdbk_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_SPD_POS_FBK
  return ((MC_NULL == pHandle) ? 0 : pHandle->InstantaneousElSpeedDpp);
#else
  return (pHandle->InstantaneousElSpeedDpp);
#endif
}

/**
  * @brief  Returns the result of the last reliability check performed.
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component.
  * @retval bool sensor reliability state.
  *
  * - Reliability is measured with reference to parameters
  * @ref SpeednPosFdbk_Handle_t::hMaxReliableMecSpeedUnit "hMaxReliableMecSpeedUnit",
  * @ref SpeednPosFdbk_Handle_t::hMinReliableMecSpeedUnit "hMaxReliableMecSpeedUnit",
  * @ref SpeednPosFdbk_Handle_t::bMaximumSpeedErrorsNumber "bMaximumSpeedErrorsNumber".
  * - If the number of time the average mechanical speed is not valid matches the
  *  maximum value of not valid speed measurements, sensor information is not reliable.
  * - Embedded into construction of the MC_GetSpeedSensorReliabilityMotor API.
  * - The return value is a boolean that expresses:\n
  * -- true  = sensor information is reliable.\n
  * -- false = sensor information is not reliable.
  */
__weak bool SPD_Check(const SpeednPosFdbk_Handle_t *pHandle)
{
  bool SpeedSensorReliability = true;
#ifdef NULL_PTR_CHECK_SPD_POS_FBK
  if ((MC_NULL == pHandle) || (pHandle->bSpeedErrorNumber == pHandle->bMaximumSpeedErrorsNumber))
#else
  if (pHandle->bSpeedErrorNumber == pHandle->bMaximumSpeedErrorsNumber)
#endif
  {
    SpeedSensorReliability = false;
  }
  else
  {
    /* Nothing to do */
  }
  return (SpeedSensorReliability);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif

/**
  * @brief  Computes and returns through parameter @ref SpeednPosFdbk_Handle_t::pMecSpeedUnit "pMecSpeedUnit",
  *         the rotor average mechanical speed, expressed in the unit defined by
  *         @ref SpeednPosFdbk_Handle_t::SpeedUnit "SpeedUnit".
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component.
  * @param  pMecSpeedUnit: pointer to int16_t, used to return the rotor average
  *         mechanical speed (expressed in the unit defined by [SPEED_UNIT](measurement_units.md).
  * @retval none
  *
  * - Computes and returns the reliability state of the sensor. Reliability is measured with
  * reference to parameters @ref SpeednPosFdbk_Handle_t::hMinReliableMecSpeedUnit "hMinReliableMecSpeedUnit",
  * @ref SpeednPosFdbk_Handle_t::hMaxReliableMecSpeedUnit "hMaxReliableMecSpeedUnit",
  * @ref SpeednPosFdbk_Handle_t::bMaximumSpeedErrorsNumber "bMaximumSpeedErrorsNumber"\n
  * -- true  = sensor information is reliable.\n
  * -- false = sensor information is not reliable.\n
  * - Called at least with the same periodicity on which speed control is executed.
  *         -

  */
__weak bool SPD_IsMecSpeedReliable(SpeednPosFdbk_Handle_t *pHandle, const int16_t *pMecSpeedUnit)
{
  bool SpeedSensorReliability = true;
#ifdef NULL_PTR_CHECK_SPD_POS_FBK
  if ((MC_NULL == pHandle) || (MC_NULL == pMecSpeedUnit))
  {
    SpeedSensorReliability = false;
  }
  else
  {
#endif
    uint16_t hAbsMecSpeedUnit;
    uint16_t hAbsMecAccelUnitP;
    int16_t hAux;
    uint8_t bSpeedErrorNumber;
    uint8_t bMaximumSpeedErrorsNumber = pHandle->bMaximumSpeedErrorsNumber;
    bool SpeedError = false;

    bSpeedErrorNumber = pHandle->bSpeedErrorNumber;

    /* Compute absoulte value of mechanical speed */
    if (*pMecSpeedUnit < 0)
    {
      hAux = -(*pMecSpeedUnit);
      hAbsMecSpeedUnit = (uint16_t)hAux;
    }
    else
    {
      hAbsMecSpeedUnit = (uint16_t)(*pMecSpeedUnit);
    }

    if (hAbsMecSpeedUnit > pHandle->hMaxReliableMecSpeedUnit)
    {
      SpeedError = true;
    }
    else
    {
      /* Nothing to do */
    }

    if (hAbsMecSpeedUnit < pHandle->hMinReliableMecSpeedUnit)
    {
      SpeedError = true;
    }
    else
    {
      /* Nothing to do */
    }

    /* Compute absoulte value of mechanical acceleration */
    if (pHandle->hMecAccelUnitP < 0)
    {
      hAux = -(pHandle->hMecAccelUnitP);
      hAbsMecAccelUnitP = (uint16_t)hAux;
    }
    else
    {
      hAbsMecAccelUnitP = (uint16_t)pHandle->hMecAccelUnitP;
    }

    if (hAbsMecAccelUnitP > pHandle->hMaxReliableMecAccelUnitP)
    {
      SpeedError = true;
    }
    else
    {
      /* Nothing to do */
    }

    if (true == SpeedError)
    {
      if (bSpeedErrorNumber < bMaximumSpeedErrorsNumber)
      {
        bSpeedErrorNumber++;
      }
      else
      {
        /* Nothing to do */
      }
    }
    else
    {
      if (bSpeedErrorNumber < bMaximumSpeedErrorsNumber)
      {
        bSpeedErrorNumber = 0u;
      }
      else
      {
        /* Nothing to do */
      }
    }

    if (bSpeedErrorNumber == bMaximumSpeedErrorsNumber)
    {
      SpeedSensorReliability = false;
    }
    else
    {
      /* Nothing to do */
    }

    pHandle->bSpeedErrorNumber = bSpeedErrorNumber;
#ifdef NULL_PTR_CHECK_SPD_POS_FBK
  }
#endif
  return (SpeedSensorReliability);
}

/**
  * @brief  Returns the average mechanical rotor speed expressed in [S16Speed](measurement_units.md).
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component
  * @retval int16_t The average mechanical rotor speed.
  *
  * - The value equals:\n
  * -- zero      when the average mechanical speed is equal zero,\n
  * -- INT16_MAX when the average mechanical speed is equal to
  * @ref SpeednPosFdbk_Handle_t::hMaxReliableMecSpeedUnit "hMaxReliableMecSpeedUnit" ,\n
  * - Called for speed monitoring through MotorPilote.
  */
__weak int16_t SPD_GetS16Speed(const SpeednPosFdbk_Handle_t *pHandle)
{
  int16_t tempValue;
#ifdef NULL_PTR_CHECK_SPD_POS_FBK
  if (MC_NULL == pHandle)
  {
    tempValue = 0;
  }
  else
  {
#endif
    int32_t wAux = (int32_t)pHandle->hAvrMecSpeedUnit;
    wAux *= INT16_MAX;
    wAux /= (int16_t)pHandle->hMaxReliableMecSpeedUnit;
    tempValue = (int16_t)wAux;
#ifdef NULL_PTR_CHECK_SPD_POS_FBK
  }
#endif
  return (tempValue);
}

/**
  * @brief  Returns the coefficient used to transform electrical to
  *         mechanical quantities and viceversa. It usually coincides with motor
  *         pole pairs number.
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component.
  * @retval uint8_t The motor pole pairs number.
  *
  * - Called by motor profiling functions and for monitoring through motorPilote.
  */
__weak uint8_t SPD_GetElToMecRatio(const SpeednPosFdbk_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_SPD_POS_FBK
  return ((MC_NULL == pHandle) ? 0U : pHandle->bElToMecRatio);
#else
  return (pHandle->bElToMecRatio);
#endif
}

/**
  * @brief  Sets the coefficient used to transform electrical to
  *         mechanical quantities and viceversa. It usually coincides with motor
  *         pole pairs number.
  * @param  pHandle: handler of the current instance of the SpeednPosFdbk component.
  * @param  bPP The motor pole pairs number to be set.
  *
  * - Called only for monitoring through motorPilote.
  */
__weak void SPD_SetElToMecRatio(SpeednPosFdbk_Handle_t *pHandle, uint8_t bPP)
{
#ifdef NULL_PTR_CHECK_SPD_POS_FBK
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->bElToMecRatio = bPP;
#ifdef NULL_PTR_CHECK_SPD_POS_FBK
  }
#endif
}


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
