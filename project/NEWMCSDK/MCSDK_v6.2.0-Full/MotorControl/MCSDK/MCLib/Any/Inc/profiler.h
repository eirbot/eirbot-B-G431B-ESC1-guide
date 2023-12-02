/**
  ******************************************************************************
  * @file    profiler.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
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

#ifndef _PROFILER_H_
#define _PROFILER_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "profiler_types.h"
#include "profiler_dcac.h"
#include "profiler_fluxestim.h"
#include "flash_parameters.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup Profiler
  * @{
  */
  
/**
  * @brief  Profiler States, lists all the possible profiler state machine states
  * 
  */  
typedef enum _PROFILER_State_e_
{
  PROFILER_STATE_Idle,            /*!< @brief Profiler Idle state */
  PROFILER_STATE_DCandACcheck,    /*!< @brief DC and AC measurement step */
  PROFILER_STATE_FluxEstim,       /*!< @brief Flux estimation step after DCAC step complete whitout error */
  PROFILER_STATE_Complete,        /*!< @brief Profiler complete state set when procedure is finished */
  PROFILER_STATE_Error,           /*!< @brief Error occurs when profiler procedure is not ready to start, mainly in two cases:
                                              - Control Mode is not in Profiling mode
                                              - or PWM are not in state forced to zero */
  PROFILER_STATE_DCAC_Error,      /*!< @brief Error occurs when DCAC profiler step failed, when measured Rs or Ls are out of expected ranges. */
  PROFILER_STATE_FluxEstim_Error, /*!< @brief Error occurs when motor fails to spin during profiling */
} PROFILER_State_e;

/**
  * @brief  Profiler errors list
  * 
  */  
typedef enum _PROFILER_Error_e_
{
  PROFILER_ERROR_None,          /*!< @brief No error detected */
  PROFILER_ERROR_DCAC,          /*!< @brief Measured motor parameters are out of expected ranges */
  PROFILER_ERROR_FluxEstim,     /*!< @brief Resulting speed from the flux observer (HSO) and the open loop frequency does not match */
  PROFILER_ERROR_NotReady,      /*!< @brief Profiling not ready to start, the procedure is not in a suitable state */
  } PROFILER_Error_e;


/**
  * @brief  Profiler user commands list
  * 
  */  
typedef enum _PROFILER_Command_e_
{
  PROFILER_COMMAND_None,       /*!< @brief Default state when no user command */
  PROFILER_COMMAND_Start,      /*!< @brief User command to start the profiler */
  PROFILER_COMMAND_Reset,      /*!< @brief User command to reset the profiler */
} PROFILER_Command_e;


/**
  * @brief  Handle of profiler component
  * 
  * This structure holds all the parameters needed to implement the profiler 
  * 
  * A pointer on a structure of this type is passed to each 
  * function of the @ref Profiler.
  */  
typedef struct _PROFILER_Obj_
{
  PROFILER_State_e	state;         /*!< @brief Profiler state machine */
  PROFILER_Command_e	command;     /*!< @brief User command state */
  PROFILER_Error_e	error;         /*!< @brief Error message to report */

  /* Profiler components */
  PROFILER_DCAC_Obj 	dcacObj;        /*!< @brief Handle of DCAC object */
  PROFILER_FLUXESTIM_Obj fluxestimObj;  /*!< @brief Handle of FluxEstim object */

  /* Configuration */
  float_t				PolePairs;			    /*!< @brief Number of polepairs, for motors without gearbox this is the number of magnets divided by two.
                                         For motors with gearbox, this is the number of magnets divided by two, multiplied by the reduction. */
  float_t				PowerDC_goal_W;	  	/*!< @brief Profiler attempts to reach this power goal during DC ramp-up */
  float_t				dutyDC;             /*!< @brief DC duty applied during profiling */
  float_t				Idc_A;              /*!< @brief DC current (A) used during profiling */
  float_t				freqEst_Hz;         /*!< @brief Open-loop estimated speed */
  float_t				freqHSO_Hz;         /*!< @brief Estimated observer (HSO) speed */
  float_t				debug_Flux_VpHz;    /*!< @brief Estimated flux in V/Hz (for debug only) */
  float_t				debug_kV;           /*!< @brief rpm per Volt DC (for debug only) */
  float_t				CritFreq_Hz;        /*!< @brief Frequency where Rs voltage drop would equal emf at used current level (for debug only) */
  float_t				injectFreq_Hz;      /*!< @brief Injected AC frequency (Hz) during profiling */
  float_t				fullScaleFreq_Hz;   /*!< @brief Full scale frequency (Hz) */
  float_t				fullScaleVoltage_V; /*!< @brief Full scale voltage (V) */

  /* Levels reached */
  float_t				PowerDC_W;			/*!< @brief Power actually reached during DC phase */

  /* Identified values */
  float_t				Rs_dc;				/*!< @brief Stator resistance measured using DC duty */
  float_t				Rs_ac;				/*!< @brief Stator resistance measured using AC duty */
  float_t				R_factor;     /*!< @brief Resitance factor = Rs_dc/Rs_ac */
  float_t				Ld_H;			  	/*!< @brief D-axis inductance measured using AC duty */
  float_t				Lq_H;				  /*!< @brief Q-axis inductance is not seperately measured */
  float_t				Flux_Wb;			/*!< @brief Flux measured while running motor */

  /* Motor characteristics */
  float_t				KT_Nm_A;			/*!< @brief Torque per Amp (Nm/Apeak) */
  float_t				Isc_A;				/*!< @brief Short circuit current, Flux/Inductance */
  /* inital value */
  fixp30_t				injectFreq_HzZest;   /*!< @brief ZeST injected frequency stored before to start profiling */
  fixp30_t				injectIdZest;        /*!< @brief ZeST Injection D current stored before to start profiling */
  fixp30_t				gainDZest;           /*!< @brief ZeST Gain D stored before to start profiling */
  fixp30_t				gainQZest;           /*!< @brief ZeST Gain Q stored before to start profiling */
} PROFILER_Obj;

#include "profiler_handle.h"

void PROFILER_init(PROFILER_Handle handle, 
                    PROFILER_Params* pParams, 
                    FLASH_Params_t const *flashParams);

void PROFILER_reset(PROFILER_Handle handle, MOTOR_Handle motor);

void PROFILER_run(PROFILER_Handle handle, MOTOR_Handle motor);

void PROFILER_runBackground(PROFILER_Handle handle, MOTOR_Handle motor);

/* Accessors */

float_t PROFILER_getDcAcMeasurementTime(PROFILER_Handle handle);
float_t PROFILER_getFlux_Wb(const PROFILER_Handle handle);
float_t PROFILER_getFluxEstFreq_Hz(const PROFILER_Handle handle);
float_t PROFILER_getFluxEstMeasurementTime(PROFILER_Handle handle);
float_t PROFILER_getLd_H(const PROFILER_Handle handle);
float_t PROFILER_getPowerGoal_W(const PROFILER_Handle handle);
float_t PROFILER_getRs_ac(const PROFILER_Handle handle);
float_t PROFILER_getRs_dc(const PROFILER_Handle handle);

void PROFILER_setCommand(PROFILER_Handle handle, const PROFILER_Command_e command);
void PROFILER_setDcAcMeasurementTime(PROFILER_Handle handle, const float_t time_seconds);
void PROFILER_setFluxEstFreq_Hz(PROFILER_Handle handle, const float_t fluxEstFreq_Hz);
void PROFILER_setFluxEstMeasurementTime(PROFILER_Handle handle, const float_t time_seconds);
void PROFILER_setPolePairs(PROFILER_Handle handle, const float_t polepairs);
void PROFILER_setPowerGoal_W(PROFILER_Handle handle, const float_t powerGoal_W);
void PROFILER_resetEstimates(PROFILER_Handle handle);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* _PROFILER_H_ */

