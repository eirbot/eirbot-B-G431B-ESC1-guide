/**
  ******************************************************************************
  * @file    profiler_fluxestim.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          profiler flux estimator component of the Motor Control SDK.
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
  * 
  */

#ifndef _PROFILER_FLUXESTIM_H_
#define _PROFILER_FLUXESTIM_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
  
/* Includes ------------------------------------------------------------------*/
#include "profiler_types.h"

  
/**
  * @brief  todo
  */ 
typedef enum _PROFILER_FLUXESTIM_State_e_
{
	PROFILER_FLUXESTIM_STATE_Idle,            /*!< @brief todo */
	PROFILER_FLUXESTIM_STATE_RampUpCur,       /*!< @brief todo */
	PROFILER_FLUXESTIM_STATE_RampUpFreq,      /*!< @brief todo */
	PROFILER_FLUXESTIM_STATE_FluxRefAuto,     /*!< @brief todo */
	PROFILER_FLUXESTIM_STATE_RampDownFreq,    /*!< @brief todo */
	PROFILER_FLUXESTIM_STATE_RampDownCur,     /*!< @brief todo */
	PROFILER_FLUXESTIM_STATE_Complete,        /*!< @brief todo */
	PROFILER_FLUXESTIM_STATE_Error,           /*!< @brief todo */
} PROFILER_FLUXESTIM_State_e;

/**
  * @brief  todo
  */
typedef enum _PROFILER_FLUXESTIM_Error_e_
{
	PROFILER_FLUXESTIM_ERROR_None,            /*!< @brief todo */
	PROFILER_FLUXESTIM_ERROR_FluxNotValid,    /*!< @brief todo */
} PROFILER_FLUXESTIM_Error_e;

/**
  * @brief  todo
  */
typedef struct _PROFILER_FLUXESTIM_Obj_
{
	PROFILER_FLUXESTIM_State_e	state;     /*!< @brief todo */
	PROFILER_FLUXESTIM_Error_e	error;     /*!< @brief todo */

	/* inputs */

	/* outputs */
	fixp30_t 	flux_Wb;                   /*!< @brief todo */

	/* configuration */
  bool		glue_dcac_fluxestim_on;                 /*!< @brief todo */
	uint32_t	measurement_counts;                 /*!< @brief todo */
	float_t		measurement_time_s;                 /*!< @brief todo */
	fixp30_t     CurToRamp_sf_pu;                   /*!< @brief todo */
	fixp30_t		FreqToRamp_sf_pu;               /*!< @brief todo */
	fixp30_t		Id_ref_pu;                      /*!< @brief todo */
	fixp30_t     FluxEstFreq_pu;                    /*!< @brief todo */
	fixp30_t		freqEst_pu;                     /*!< @brief todo */
	fixp30_t		freqHSO_pu;                     /*!< @brief todo */
	fixp30_t		ramp_rate_A_pu_per_cycle;       /*!< @brief todo */
	fixp30_t		ramp_rate_freq_pu_per_cycle;    /*!< @brief todo */
	fixp30_t		fluxEstimPoleTs;                /*!< @brief todo */
	float_t		background_rate_hz;                 /*!< @brief todo */
  float_t 	fullScaleCurrent_A;                     /*!< @brief todo */
  float_t 	fullScaleVoltage_V;                     /*!< @brief todo */
  float_t		fullScaleFreq_Hz;                   /*!< @brief todo */
  float_t     voltagefilterpole_rps;                /*!< @brief todo */

	/* FluxEstim state data */
	uint32_t counter;                               /*!< @brief todo */

} PROFILER_FLUXESTIM_Obj;

/**
  * @brief  todo
  */
typedef PROFILER_FLUXESTIM_Obj* PROFILER_FLUXESTIM_Handle;

void PROFILER_FLUXESTIM_init(PROFILER_FLUXESTIM_Handle handle);

void PROFILER_FLUXESTIM_setParams(PROFILER_FLUXESTIM_Handle handle, PROFILER_Params* pParams);

void PROFILER_FLUXESTIM_run(PROFILER_FLUXESTIM_Handle handle, MOTOR_Handle motor);

void PROFILER_FLUXESTIM_runBackground(PROFILER_FLUXESTIM_Handle handle, MOTOR_Handle motor);

void PROFILER_FLUXESTIM_reset(PROFILER_FLUXESTIM_Handle handle, MOTOR_Handle motor);

/* Accessors */

float_t PROFILER_FLUXESTIM_getFluxEstFreq_Hz(const PROFILER_FLUXESTIM_Handle handle);
float_t PROFILER_FLUXESTIM_getMeasurementTime(PROFILER_FLUXESTIM_Handle handle);

void PROFILER_FLUXESTIM_setFluxEstFreq_Hz(PROFILER_FLUXESTIM_Handle handle, const float_t fluxEstFreq_Hz);
void PROFILER_FLUXESTIM_setMeasurementTime(PROFILER_FLUXESTIM_Handle handle, const float_t time_seconds);



#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* _PROFILER_FLUXESTIM_H_ */

/* end of profiler_fluxestim.h */
