/**
  ******************************************************************************
  * @file    f0xx_bemf_ADC_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          Sensorless Bemf acquisition with ADC component of the Motor Control SDK.
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
  * @ingroup SpeednPosFdbk_Bemf
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEMAGMGT_H
#define DEMAGMGT_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "speed_ctrl.h"
  
/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup Demag_management
  * @{
  */

/**
  * @brief This structure is used to handle the data of an instance of the Demagnetization Management component
  *
  */
typedef struct
{
  uint16_t PWMCycles;       /*!< Counter of the PWM cycles elapsed from the last step change */  
  uint16_t DemagCounterThreshold;  /*!< Number of PWM cycles for phase demagnetization */  
  uint16_t DemagMinimumSpeedUnit;    /*!< Speed threshold for minimum demagnetization time */
  uint16_t RevUpDemagSpeedConv;      /*!< Convertion factor between speed and demagnetization time */
  uint16_t RunDemagSpeedConv;        /*!< Open loop convertion factor between speed and demagnetization time during */
  uint16_t DemagMinimumThreshold;    /*!< Minimum demagnetization time */
  uint8_t PWMScaling;
} Demag_Handle_t;

/* Exported functions --------------------------------------------------------*/

/* It initializes all the object variables. */
void DMG_Init( Demag_Handle_t *pHandle );

/* It resets the ADC status and empties arrays. */
void DMG_Clear( Demag_Handle_t *pHandle );

/* It configures the sensorless parameters for the following step. */
void DMG_IncreaseDemagCounter(Demag_Handle_t *pHandle);

/* It configures the sensorless parameters for the following step. */
uint16_t DMG_GetDemagCounter(Demag_Handle_t *pHandle);

/* It computes the demagnetization time during revup procedure. */
bool DMG_IsDemagTElapsed(Demag_Handle_t *pHandle );

/* It computes the demagnetization time in closed loop operation.*/
void DMG_CalcRevUpDemagT(Demag_Handle_t *pHandle, SpeednTorqCtrl_Handle_t *pHandleSTC );

/* It computes the demagnetization time in closed loop operation.*/
void DMG_CalcRunDemagT(Demag_Handle_t *pHandle, SpeednTorqCtrl_Handle_t *pHandleSTC );

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* BEMFADCFDBK_H */

/******************* (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
