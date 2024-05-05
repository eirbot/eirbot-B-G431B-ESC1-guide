/**
  ******************************************************************************
  * @file    profiler_types.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions for the
  *          profiler component of the Motor Control SDK.
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
  * @ingroup Profiler
  */

#ifndef _PROFILER_TYPES_H_
#define _PROFILER_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h" 
#include "speed_torq_ctrl_hso.h"
#include "mc_curr_ctrl.h"
#include "speed_pos_fdbk_hso.h"
#include "mc_polpulse.h"
#include "bus_voltage.h"  

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup Profiler
  * @{
  */
  
 /**
  * @brief  Handle of profiler motor component
  * 
  * This structure holds all the components needed to run the profiler 
  * 
  * A pointer on a structure of this type is passed to main 
  * functions of the @ref Profiler.
  */ 
typedef struct
{
  FOCVars_t* pFocVars;                         /*!< @brief todo */
  SPD_Handle_t *pSPD;                /*!< @brief todo */
  STC_Handle_t *pSTC;                          /*!< @brief todo */
  POLPULSE_Obj *pPolpulse;                     /*!< @brief todo */
  CurrCtrl_Handle_t *pCurrCtrl;                /*!< @brief todo */
  PWMC_Handle_t * pPWM;                        /*!< @brief todo */
  BusVoltageSensor_Handle_t *pVBus;            /*!< @brief todo */
}ProfilerMotor_Handle_t;

/**
  * @brief  todo
  */ 
typedef ProfilerMotor_Handle_t* MOTOR_Handle; 

/**
  * @brief  todo
  */ 
typedef struct _PROFILER_Params_
{
	/* General parameters */
  float_t isrFreq_Hz;
	float_t background_rate_hz;                  /*!< @brief todo */
  float_t fullScaleFreq_Hz;                    /*!< @brief todo */
  float_t fullScaleCurrent_A;                  /*!< @brief todo */
  float_t fullScaleVoltage_V;                  /*!< @brief todo */
  float_t voltagefilterpole_rps;               /*!< @brief todo */
  float_t PolePairs;                           /*!< @brief todo */

  /* DC / AC measurments */
  float_t dcac_PowerDC_goal_W;                  /*!< @brief todo */
  float_t dcac_DCmax_current_A;                 /*!< @brief todo */
  float_t dcac_duty_maxgoal;                    /*!< @brief todo */
  float_t dcac_dc_measurement_time_s;           /*!< @brief todo */
  float_t dcac_ac_measurement_time_s;           /*!< @brief todo */
  float_t dcac_duty_ramping_time_s;             /*!< @brief todo */
  float_t dcac_current_ramping_time_s;          /*!< @brief todo */
	float_t dcac_ac_freq_Hz;                      /*!< @brief todo */

	bool    glue_dcac_fluxestim_on;               /*!< @brief skip ramp down and ramp up */

	/* Flux estimation */
  float_t fluxestim_measurement_time_s;          /*!< @brief todo */
	float_t fluxestim_Idref_A;                     /*!< @brief todo */
	float_t fluxestim_current_ramping_time_s;      /*!< @brief todo */
	float_t fluxestim_speed_Hz;                    /*!< @brief todo */
	float_t fluxestim_speed_ramping_time_s;        /*!< @brief todo */

} PROFILER_Params;

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* _PROFILER_TYPES_H_ */

/* end of profiler_types.h */
