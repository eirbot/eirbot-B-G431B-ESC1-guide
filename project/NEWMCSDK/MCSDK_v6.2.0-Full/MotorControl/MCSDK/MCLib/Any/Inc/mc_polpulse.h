/**
  ******************************************************************************
  * @file    mc_polpulse.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          PolPulse application component of the Motor Control SDK.
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
  * @ingroup MC_PolPulse
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MC_POLPULSE_H_
#define _MC_POLPULSE_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "polpulse.h"
#include "pwm_curr_fdbk.h"
#include "speed_pos_fdbk_hso.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MC_PolPulse
  * @{
  */

/* Exported defines ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
  
/**
  * @brief  PolPulse application handler structure
  * 
  * This structure holds the components and parameters needed to implement the PolPulse 
  * 
  * A pointer on a structure of this type is passed to each 
  * function of the @ref MC_PolPulse.
  */  
typedef struct
{
  POLPULSE_Obj PolpulseObj;        /*!< @brief pointer on polPulse component handler.*/
  PWMC_Handle_t * pPWM;            /*!< @brief pointer PWM current feedback component handler .*/
  SPD_Handle_t *pSPD;    /*!< @brief pointer on sensorless component handler.*/
  TIM_TypeDef * TIMx;              /*!< @brief timer used for PWM generation.*/
  int16_t pulse_countdown;         /*!< @brief pulse counter.*/
  bool flagPolePulseActivation;    /*!< @brief PolPulse activation flag.*/
}MC_PolPulse_Handle_t;

void MC_POLPULSE_init(MC_PolPulse_Handle_t *pHandle,
                      POLPULSE_Params *params,
                      FLASH_Params_t const *flashParams);
void MC_POLPULSE_SetAngle(MC_PolPulse_Handle_t *pHandle, const fixp30_t angle_pu);
void MC_POLPULSE_run(MC_PolPulse_Handle_t *pHandle, const fixp30_t angle_pu);
void PulseCountDown(MC_PolPulse_Handle_t *pHandle);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* _MC_POLPULSE_H_ */

/******************* (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
