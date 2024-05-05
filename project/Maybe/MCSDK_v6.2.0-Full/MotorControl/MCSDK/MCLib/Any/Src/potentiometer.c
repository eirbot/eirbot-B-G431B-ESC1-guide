/**
  ******************************************************************************
  * @file    potentiometer.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides the functions that implement the potentiometer
  *          component of the Motor Control SDK.
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
  * @ingroup Potentiometer
  */

/* Includes ------------------------------------------------------------------*/
#include "potentiometer.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup Potentiometer Linear Potentiometer
  * @brief Linear Potentiometer reading component of the Motor Control SDK
  *
  *  The Potentiometer component aims at measuring the voltage set on a linear 
  * potentiometer. It uses the services of the @ref RCM component to get measures
  * from an ADC channel connected to the potentiometer. 
  * 
  *  The component is designed to take potentiometer measures periodically. It 
  * computes a moving average on a number of these measures in order to reduce
  * the reading noise. This moving average is the potentiometer value produced 
  * by the component. It is retrieved by calling the POT_GetValue() function.
  * 
  *  The measures are taken by the POT_TakeMeasurement() function. This 
  * function must then be called periodically.
  * 
  *  At startup or after a call to the POT_Clear() function, a valid average  
  * potentiometer value is not immediately available. Enough measures need to be
  * taken so that the moving average can be computed. The POT_ValidValueAvailable()
  * function can be used to check whether a valid potentiometer value is returned
  * when calling the POT_GetValue() function.
  * 
  *  The state of a Potentiometer component is maintained in a Potentiometer_Handle_t
  * structure. To use the Potentiometer component, a Potentiometer_Handle_t structure
  * needs to be instanciated and initialized. The initialization is performed thanks 
  * to the POT_Init() function. Prior to calling this function, some of the fields of
  * this structure need to be given a value. See the Potentiometer_Handle_t and below
  * for more details on this.
  * 
  *  The Potentiometer component accumulates a number of measures on which it 
  * computes a moving average. For performance reasons, this number must be a 
  * power of two. This is set in the Potentiometer_Handle_t::LPFilterBandwidthPOW2 
  * field of the potentiometer handle structure.
  * 
  * @note In the current version of the Potentiometer component, the periodic ADC 
  * measures **must** be performed on the Medium Frequency Task. This can be done by using 
  * the MC_APP_PostMediumFrequencyHook_M1() function of the @ref MCAppHooks service 
  * for instance.
  * 
  * @{
  */

/* Public functions ----------------------------------------------------------*/
/**
 * @brief Initializes a Potentiometer component
 * 
 * This function must be called once before starting to use the component.
 * 
 * @param pHandle Handle on the Potentiometer component to initialize
 */
void POT_Init(Potentiometer_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_POT
  if (MC_NULL ==  pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif /* NULL_PTR_CHECK_POT */
    /* Potentiometer component init */
    pHandle->LPFilterBandwidth = (uint16_t)(1 << pHandle->LPFilterBandwidthPOW2);
    POT_Clear(pHandle);
#ifdef NULL_PTR_CHECK_POT
  }
#endif /* NULL_PTR_CHECK_POT */
}

/**
 * @brief Clears the state of a Potentiometer component
 * 
 *  After this function has been called, the potentiometer value
 * becomes invalid. See the @ref Potentiometer documentation for more
 * details.
 * 
 * @param pHandle Handle on the Potentiometer component
 */
void POT_Clear(Potentiometer_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_POT
  if (MC_NULL ==  pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif /* NULL_PTR_CHECK_POT */
    for (uint8_t i = 0; i < pHandle->LPFilterBandwidth; i++)
    {
      pHandle->PotMeasArray[i] = (uint16_t) 0;
    }

    pHandle->PotValueAccumulator = (uint32_t)0;
    pHandle->Index = (uint16_t)0;
    pHandle->Valid = (bool) false;
#ifdef NULL_PTR_CHECK_POT
  }
#endif /* NULL_PTR_CHECK_POT */
}

/**
 * @brief Measures the voltage of the potentiometer of a Potentiometer component
 * 
 *  This function needs to be called periodically. See the @ref Potentiometer 
 * documentation for more details.
 *
 * @param pHandle Handle on the Potentiometer component
 */
void POT_TakeMeasurement(Potentiometer_Handle_t *pHandle, uint16_t rawValue)
{
#ifdef NULL_PTR_CHECK_POT
  if (MC_NULL ==  pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif /* NULL_PTR_CHECK_POT */
    /* Update the Accumulator */
    pHandle->PotValueAccumulator -= (uint32_t) pHandle->PotMeasArray[pHandle->Index];
    pHandle->PotMeasArray[pHandle->Index] = rawValue;
    pHandle->PotValueAccumulator += (uint32_t) pHandle->PotMeasArray[pHandle->Index];

    /* Update the Index */
    pHandle->Index++;
    if (pHandle->Index == pHandle->LPFilterBandwidth)
    {
      pHandle->Index = 0;
      /* The Index has reached the end of the measurement array. 
       * So, the accumulator is only made of actual measure. 
       * Hence, its value is considered valid. */
      pHandle->Valid = true;
    }
    else
    {
      /* Nothing to do */
    }
#ifdef NULL_PTR_CHECK_POT
  }
#endif /* NULL_PTR_CHECK_POT */
}

/**
 * @brief Returns the current value of a Potentiometer component
 * 
 * @param pHandle Handle on the Potentiometer component
 */
uint16_t POT_GetValue(Potentiometer_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_POT
    return ((NULL == pHandle) ? 0U : (uint16_t)(pHandle->PotValueAccumulator >> pHandle->LPFilterBandwidthPOW2));
#else /* NULL_PTR_CHECK_POT */
    return ((uint16_t)(pHandle->PotValueAccumulator >> pHandle->LPFilterBandwidthPOW2));
#endif /* NULL_PTR_CHECK_POT */
}

/**
 * @brief Returns true if the current value of a Potentiometer component is valid 
 *        and false otherwise
 * 
 * @param pHandle Handle on the Potentiometer component
 */
bool POT_ValidValueAvailable(Potentiometer_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_POT
    return ((NULL == pHandle) ? 0U : pHandle->Valid);
#else /* NULL_PTR_CHECK_POT */
    return (pHandle->Valid);
#endif /* NULL_PTR_CHECK_POT */
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
