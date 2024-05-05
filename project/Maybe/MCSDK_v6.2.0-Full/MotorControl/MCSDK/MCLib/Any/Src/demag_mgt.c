/**
  ******************************************************************************
  * @file    f0xx_bemf_ADC_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement Bemf sensing
  *          class to be stantiated when the six-step sensorless driving mode
  *          topology is used. It is specifically designed for STM32F0XX
  *          microcontrollers and implements the sensing using one ADC with
  *          DMA support.
  *           + MCU peripheral and handle initialization fucntion
  *           + ADC sampling function
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
#include "demag_mgt.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/**
 * @defgroup Demag_management Six-Step Demagnetization time management
 *
 * @brief Demagnetization time management
 *
 * This component is used in applications based on Six-Step algorithm
 * using either sensorless or sensored position feedback.
 *
 * @todo: TODO: complete documentation.
 * @{
 */

/**
  * @brief  It initializes ADC1, DMA and NVIC for three bemf voltages reading
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @retval none
  */
__weak void DMG_Init( Demag_Handle_t *pHandle )
{

  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    pHandle->PWMCycles = 0;
    pHandle->DemagCounterThreshold  = pHandle->DemagMinimumThreshold;
  }
}

/**
  * @brief  Reset the ADC status
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @retval none
  */
__weak void DMG_Clear( Demag_Handle_t *pHandle )
{
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    pHandle->PWMCycles = 0;
  }
}

/**
  * @brief  Reset the ADC status
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @retval none
  */
__weak void DMG_IncreaseDemagCounter(Demag_Handle_t *pHandle)
{

  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    pHandle->PWMCycles = pHandle->PWMCycles + pHandle->PWMScaling ;
  }
}

/**
  * @brief  Reset the ADC status
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @retval none
  */
__weak uint16_t DMG_GetDemagCounter(Demag_Handle_t *pHandle)
{
  return (pHandle->PWMCycles);
}

/**
  * @brief  Reset the ADC status
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @retval none
  */
__weak bool DMG_IsDemagTElapsed(Demag_Handle_t *pHandle )
{
  bool DemagElapsed = false;
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    if (pHandle->PWMCycles >=  pHandle->DemagCounterThreshold)
    {
      DemagElapsed = true;
    }
    else
    {
    }
  }
  return DemagElapsed;
}

/**
  * @brief  It calculates and stores in the corresponding variable the demagnetization 
  *         time in open loop operation  
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @param  pHandleSTC: handler of the current instance of the Speed Control component
  * @retval none
  */
__weak void DMG_CalcRevUpDemagT(Demag_Handle_t *pHandle, SpeednTorqCtrl_Handle_t *pHandleSTC )
{
  int16_t hSpeed;
  SpeednPosFdbk_Handle_t *speedHandle;
  speedHandle = STC_GetSpeedSensor(pHandleSTC);
  hSpeed = SPD_GetAvrgMecSpeedUnit(speedHandle);
  if (hSpeed == 0)
  {
    pHandle->DemagCounterThreshold = pHandle->DemagMinimumThreshold;;
  }	  
  else
  {
    if (hSpeed < 0)
    {
      hSpeed = - hSpeed;
    }
    pHandle->DemagCounterThreshold = (uint16_t) (pHandle->RevUpDemagSpeedConv / hSpeed);		
  }
  if (pHandle->DemagCounterThreshold < pHandle->DemagMinimumThreshold)
  {
    pHandle->DemagCounterThreshold = pHandle->DemagMinimumThreshold;
  }  
}

/**
  * @brief  It calculates and stores in the corresponding variable the demagnetization 
  *         time in closed loop operation  
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @param  pHandleSTC: handler of the current instance of the Speed Control component
  * @retval none
  */
__weak void DMG_CalcRunDemagT(Demag_Handle_t *pHandle, SpeednTorqCtrl_Handle_t *pHandleSTC )
{
  int16_t hSpeed;
  SpeednPosFdbk_Handle_t *speedHandle;
  speedHandle = STC_GetSpeedSensor(pHandleSTC);
  hSpeed = SPD_GetAvrgMecSpeedUnit(speedHandle);
  if (hSpeed < 0) hSpeed = - hSpeed; 
  if (hSpeed < pHandle->DemagMinimumSpeedUnit)
  {   
    pHandle->DemagCounterThreshold = (uint16_t) (pHandle->RunDemagSpeedConv / hSpeed);
    if (pHandle->DemagCounterThreshold < pHandle->DemagMinimumThreshold)
    {
      pHandle->DemagCounterThreshold = pHandle->DemagMinimumThreshold;
    }   
  } 
  else
  {   
    pHandle->DemagCounterThreshold = pHandle->DemagMinimumThreshold;
  }     
}


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
