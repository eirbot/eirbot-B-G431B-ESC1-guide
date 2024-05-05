/**
  ******************************************************************************
  * @file    profiler.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement 
  *          the profiler component of the Motor Control SDK.
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
  * @ingroup Profiler
  */

/* Includes ------------------------------------------------------------------*/
#include "profiler.h"
#include "drive_parameters.h"
#include "parameters_conversion.h"

/** @addtogroup MCSDK
  * @{
  */

/**
  * @defgroup Profiler 
  *
  * @brief Profiler component of the Motor Control SDK
  *
  * This procedure can be used to identify any PMSM.
  * 
  * Once the profiler complete successfully the motor parameters @f$R_s,\ L_s@f$ and motor flux @f$\psi_{mag} @f$ are provided.\n
  * The estimated @f$R_s@f$ and @f$L_s@f$ are then used to define the PI settings (@f$K_p,\ K_i@f$) for current controllers.
  *
  * [Profiler](Profiler.md)
  * 
  * @{
  */
  
void PROFILER_stateMachine(PROFILER_Handle handle, MOTOR_Handle motor);

  
/**
  * @brief  Initializes Profiler component.
  * @param  handle Profiler handler
  * @param  pParams pointer on profiler parameters
  * @param  flashParams pointer on flash parameters handler
  *
  */
void PROFILER_init(PROFILER_Handle handle, 
                   PROFILER_Params *pParams,
                   FLASH_Params_t const *flashParams)
{
  
  PROFILER_DCAC_init(&handle->dcacObj);
  PROFILER_FLUXESTIM_init(&handle->fluxestimObj);
  
  pParams->fullScaleFreq_Hz = flashParams->scale.frequency;
  pParams->fullScaleCurrent_A = flashParams->scale.current;
  pParams->fullScaleVoltage_V = flashParams->scale.voltage;
  pParams->PolePairs = flashParams->motor.polePairs;
  
  handle->PolePairs = pParams->PolePairs;
  handle->PowerDC_goal_W = pParams->dcac_PowerDC_goal_W;

  handle->fullScaleFreq_Hz = pParams->fullScaleFreq_Hz;
  handle->fullScaleVoltage_V = pParams->fullScaleVoltage_V;
        
        
  PROFILER_DCAC_setParams(&handle->dcacObj, pParams);
  PROFILER_FLUXESTIM_setParams(&handle->fluxestimObj, pParams);
}

  
/**
  * Resets Profiler to restart new measure
  * param  pHandle Profiler handler
  * param  motor handler
  *
  */
void PROFILER_reset(PROFILER_Handle handle, MOTOR_Handle motor)
{

  handle->state = PROFILER_STATE_Idle;
  handle->command = PROFILER_COMMAND_None;
  handle->error = PROFILER_ERROR_None;

  PROFILER_DCAC_reset(&handle->dcacObj, motor);
  PROFILER_FLUXESTIM_reset(&handle->fluxestimObj, motor);

  /* Clear result variables */
  PROFILER_resetEstimates(handle);

}

/**
  * @brief  Executes profiler procedure
  * @param  handle Profiler handler
  * @param  motor motor handler
  *
  */
void PROFILER_run(PROFILER_Handle handle, MOTOR_Handle motor)
{
  /* Called from the motor control interrupt */
  /* Required because some data can only be calculated in the interrupt (or at least synchronous to isr)
   * Slow/demanding calculations are not allowed
   */

  switch (handle->state)
  {
  case PROFILER_STATE_DCandACcheck:
    PROFILER_DCAC_run(&handle->dcacObj, motor);
    break;

  case PROFILER_STATE_FluxEstim:
    PROFILER_FLUXESTIM_run(&handle->fluxestimObj, motor);
    break;

  case PROFILER_STATE_Idle:
    /* No break */
  case PROFILER_STATE_Complete:
    /* No break */
  case PROFILER_STATE_Error:
  case PROFILER_STATE_DCAC_Error:
  case PROFILER_STATE_FluxEstim_Error:
    /* Nothing to do in these states */
    break;
  }
}


/**
  * @brief  Executes profiling in background task.
  * @param  handle Profiler handler
  * @param  motor parameters
  *
  */
void PROFILER_runBackground(PROFILER_Handle handle, MOTOR_Handle motor)
{
  /* Called from slow control loop, outside of ISR */

  /* Does nothing when profiler is not active */
  if ((handle->state == PROFILER_STATE_Idle || handle->state == PROFILER_STATE_Complete) &&
    (handle->command != PROFILER_COMMAND_Start) && handle->command != PROFILER_COMMAND_Reset) return;

  /* Run the profiler top-level state machine */
  PROFILER_stateMachine(handle, motor);

  /* Run the lower level profilers */
  switch(handle->state)
  {
  case PROFILER_STATE_DCandACcheck:
    PROFILER_DCAC_runBackground(&handle->dcacObj, motor);
    break;

  case PROFILER_STATE_FluxEstim:
    PROFILER_FLUXESTIM_runBackground(&handle->fluxestimObj, motor);
    break;

  case PROFILER_STATE_Complete:
  case PROFILER_STATE_Error:
  case PROFILER_STATE_DCAC_Error:
  case PROFILER_STATE_FluxEstim_Error:
    motor->pSPD->zestControl.injectFreq_kHz = FIXP30(80.0f / 1000.0f); // switch to 80Hz injection frequency after profiling;
    motor->pSPD->zestControl.injectId_A_pu = 0; // switch to 0 injection after profiling;
    motor->pSPD->zestControl.feedbackGainD = 0; // reset gainD after profiling
    motor->pSPD->zestControl.feedbackGainQ = 0; // reset gainQ after profiling
    motor->pCurrCtrl->Ddq_ref_pu.D = 0 ;
    break;

  case PROFILER_STATE_Idle:
    /* No break */

    /* Nothing to do in these states */
    break;
  }
}

/**
  * @brief  Set PWM off and disable current controller
  * @param  handle Profiler handler
  * @param  motor parameters
  *
  */
void PROFILER_setMotorToIdle(PROFILER_Handle handle, MOTOR_Handle motor)
{
  motor->pCurrCtrl->forceZeroPwm = true;
  motor->pCurrCtrl->currentControlEnabled = false;
}

 /**
  * @brief  Check profiler state before to start profiling procedure
  * @param  handle Profiler handler
  * @param  motor parameters
  *
  */
PROFILER_Error_e PROFILER_isReadyToStart(PROFILER_Handle handle, MOTOR_Handle motor)
{
  // ToDo: Check PWM is active (Requires reference to hardware layer from motorHandle)

  /* Must be in control mode 'None' */ // ToDo: Allow Profiler start in more controlModes
  if (motor->pFocVars->controlMode != MCM_PROFILING_MODE) return PROFILER_ERROR_NotReady;

  // ToDo: Check offset measurement performed (or known-good from EEPROM)
  if (motor->pCurrCtrl->forceZeroPwm == false) return PROFILER_ERROR_NotReady;	/* Must be in ForceZeroPwm mode */

  return PROFILER_ERROR_None;
}

 /**
  * @brief  Profiler top-level state machine
  * @param  handle Profiler handler
  * @param  motor parameters
  *
  */
void PROFILER_stateMachine(PROFILER_Handle handle, MOTOR_Handle motor)
{
  PROFILER_State_e state = handle->state;
  PROFILER_State_e newState = state;
  PROFILER_Command_e command = handle->command;
  handle->command = PROFILER_COMMAND_None;

  switch (state)
  {
  case PROFILER_STATE_Idle:
    if (command == PROFILER_COMMAND_Start)
    {
      PROFILER_Error_e isReadyResult = PROFILER_isReadyToStart(handle, motor);

      if (isReadyResult == PROFILER_ERROR_None)
      {
        handle->error = PROFILER_ERROR_None;
        /* Reset sub-profilers */
        PROFILER_DCAC_reset(&handle->dcacObj, motor);
        PROFILER_FLUXESTIM_reset(&handle->fluxestimObj, motor);
        PROFILER_resetEstimates(handle);

        /* Set up for DCAC measurement */
        handle->dcacObj.PowerDC_goal_W = handle->PowerDC_goal_W;
        motor->pFocVars->controlMode = MCM_PROFILING_MODE;
        newState = PROFILER_STATE_DCandACcheck;
      }
      else
      {
        /* Cannot start profiling now */
        handle->error = isReadyResult;
        newState = PROFILER_STATE_Error;
      }
    }
    break;

  case PROFILER_STATE_DCandACcheck:
    if ((handle->dcacObj.state == PROFILER_DCAC_STATE_Complete) || (handle->dcacObj.state == PROFILER_DCAC_STATE_Error))
    {
      /* Fetch measured Rs and Ls */
      handle->PowerDC_W = handle->dcacObj.PowerDC_W;
      handle->dutyDC = FIXP30_toF(handle->dcacObj.dc_duty_pu);
      handle->Idc_A  = FIXP30_toF(handle->dcacObj.Id_dc_ref_pu) * handle->dcacObj.fullScaleCurrent_A;
      handle->Rs_dc  = handle->dcacObj.Rs_dc;
      handle->Rs_ac  = handle->dcacObj.Rs_inject;
      handle->Ld_H   = handle->dcacObj.Ls_inject;
      handle->Lq_H   = 0.0f; //todo
      handle->R_factor = handle->dcacObj.Rs_inject / handle->dcacObj.Rs_dc;
      handle->injectFreq_Hz = FIXP30_toF(handle->dcacObj.ac_freqkHz_pu) * 1000.0f;

      /* Copy value from DCAC test */
      handle->fluxestimObj.Id_ref_pu = handle->dcacObj.Id_dc_ref_pu;
      handle->fluxestimObj.ramp_rate_A_pu_per_cycle  = FIXP30_mpy(handle->fluxestimObj.Id_ref_pu, handle->fluxestimObj.CurToRamp_sf_pu);

      if (handle->dcacObj.state == PROFILER_DCAC_STATE_Error)
      {
        handle->error = PROFILER_ERROR_DCAC;
        newState = PROFILER_STATE_DCAC_Error;

        /* Clear profiling mode */
        motor->pFocVars->controlMode = MCM_PROFILING_MODE;
      }
      else
      {
        newState = PROFILER_STATE_FluxEstim;
      }
    }
    break;

  case PROFILER_STATE_FluxEstim:
    if ((handle->fluxestimObj.state == PROFILER_FLUXESTIM_STATE_Complete) || (handle->fluxestimObj.state == PROFILER_FLUXESTIM_STATE_Error))
    {
      /* Fetch measured Flux */
      handle->freqEst_Hz = FIXP30_toF(handle->fluxestimObj.freqEst_pu) * handle->fullScaleFreq_Hz;
      handle->freqHSO_Hz = FIXP30_toF(handle->fluxestimObj.freqHSO_pu) * handle->fullScaleFreq_Hz;

      if (handle->fluxestimObj.state == PROFILER_FLUXESTIM_STATE_Complete)
      {
        handle->Flux_Wb = FIXP30_toF(handle->fluxestimObj.flux_Wb);
        handle->debug_Flux_VpHz = M_TWOPI * handle->Flux_Wb; 		 /* Flux V/Hz, just for check*/
        handle->debug_kV =  30.0f / (1.0000f * handle->PolePairs * handle->debug_Flux_VpHz); //rpm per Volt DC (drone motors)
        handle->KT_Nm_A = 1.5f * handle->PolePairs * handle->Flux_Wb; /* Torque per Amp (Nm/Apeak) */
        handle->Isc_A   = handle->Flux_Wb / handle->Ld_H;            /* machine's short-circuit current */
        
        /* Update PolePulse parameters */
        POLPULSE_setCurrentGoal(motor->pPolpulse, 0.25f*handle->Isc_A);
        POLPULSE_setLsd(motor->pPolpulse, handle->Ld_H);

        handle->CritFreq_Hz = (handle->Rs_dc * handle->Idc_A) / handle->debug_Flux_VpHz; /* freq where Rs voltage drop would equal emf at used current level */
        // suggested Kp_mech = 5 * rated current / rated speed: rated current is about
          float FullScaleFlux = handle->fullScaleVoltage_V * (float)(1.0f/TF_REGULATION_RATE);
          float RatedFlux_pu = (M_TWOPI * handle->Flux_Wb) / FullScaleFlux;
          float oneOverRatedFlux_pu = 1.0f / RatedFlux_pu;
          FIXPSCALED_floatToFIXPscaled(oneOverRatedFlux_pu, &motor->pSPD->oneOverFlux_pu_fps);
            motor->pFocVars->Kt = (1.0f / (1.5f * handle->PolePairs * handle->Flux_Wb));
            motor->pSPD->flagDynamicQgain = false;
        newState = PROFILER_STATE_Complete;
      }
      else  /* PROFILER_FLUXESTIM_STATE_Error */
      {
        handle->Flux_Wb = 0.0;
        handle->debug_Flux_VpHz = 0.0;
        handle->debug_kV =  0.0;
        handle->KT_Nm_A = 0.0;
        handle->Isc_A   = 0.0;
        handle->CritFreq_Hz = 0.0;
        handle->error = PROFILER_ERROR_FluxEstim;
        newState = PROFILER_STATE_FluxEstim_Error;
      }

        /* PWM off and related settings */
        PROFILER_setMotorToIdle(handle, motor);

        /* Next state */
        motor->pFocVars->controlMode = MCM_PROFILING_MODE;
    }
    break;

  case PROFILER_STATE_Complete:
    {
    PROFILER_DCAC_reset(&handle->dcacObj, motor);
    PROFILER_FLUXESTIM_reset(&handle->fluxestimObj, motor);

    /* Allow starting from Complete state */
    if (command == PROFILER_COMMAND_Start)
    {
      newState = PROFILER_STATE_Idle;
    }
    }
    break;

  case PROFILER_STATE_Error:
  case PROFILER_STATE_DCAC_Error:
  case PROFILER_STATE_FluxEstim_Error:
    /* PWM off and related settings */
    PROFILER_setMotorToIdle(handle, motor);
    break;
  }

  /* Reset is allowed in any state */
  if (command == PROFILER_COMMAND_Reset)
  {
    PROFILER_reset(handle, motor);
    newState = PROFILER_STATE_Idle;
  }

  if (newState != state)
  {
    handle->state = newState;
  }

}

/* Accessors */

float_t PROFILER_getDcAcMeasurementTime(PROFILER_Handle handle)
{
  return PROFILER_DCAC_getMeasurementTime(&handle->dcacObj);
}

/**
  * @brief  Returns measured flux amplitude (in Weber)
  * @param  pHandle Handle on the Profiler component
  */
float_t PROFILER_getFlux_Wb(const PROFILER_Handle handle)
{
  return handle->Flux_Wb;
}

/**
  * @brief  Returns estimated flux amplitude (in Hz)
  * @param  pHandle Handle on the Profiler component
  */
float_t PROFILER_getFluxEstFreq_Hz(const PROFILER_Handle handle)
{
  return PROFILER_FLUXESTIM_getFluxEstFreq_Hz(&handle->fluxestimObj);
}

/**
  * @brief  Returns measurement time allowed to estimate the flux
  * @param  pHandle Handle on the Profiler component
  */
float_t PROFILER_getFluxEstMeasurementTime(PROFILER_Handle handle)
{
  return PROFILER_FLUXESTIM_getMeasurementTime(&handle->fluxestimObj);
}

/**
  * @brief  Returns estimated inductance (in Henry)
  * @param  pHandle Handle on the Profiler component
  */
float_t PROFILER_getLd_H(const PROFILER_Handle handle)
{
  return handle->Ld_H;
}

/**
  * @brief  Returns power goal used for DCAC measurement
  * @param  pHandle Handle on the Profiler component
  */
float_t PROFILER_getPowerGoal_W(const PROFILER_Handle handle)
{
  return handle->PowerDC_goal_W;
}

/**
  * @brief  Returns estimated resistance Rs_ac from AC-duty injection
  * @param  pHandle Handle on the Profiler component
  */
float_t PROFILER_getRs_ac(const PROFILER_Handle handle)
{
  return handle->Rs_ac;
}

/**
  * @brief  Returns estimated resistance Rs_dc once current ramp-up complete
  * @param  pHandle Handle on the Profiler component
  */
float_t PROFILER_getRs_dc(const PROFILER_Handle handle)
{
  return handle->Rs_dc;
}

/**
  * @brief  Sets user command to start or reset the profiler
  * @param  pHandle Handle on the Profiler component
  * @param  command User command: Start or Reset
  */
void PROFILER_setCommand(PROFILER_Handle handle, const PROFILER_Command_e command)
{
  handle->command = command;
}

/**
  * @brief  Sets measurement time allowed to DCAC steps
  * @param  pHandle Handle on the Profiler component
  * @param  time_seconds time in seconds allowed to execute each one of steps: DC, AC measuring
  */
void PROFILER_setDcAcMeasurementTime(PROFILER_Handle handle, const float_t time_seconds)
{
  PROFILER_DCAC_setMeasurementTime(&handle->dcacObj, time_seconds);
}

/**
  * @brief  Sets flux estimation frequency
  * @param  pHandle Handle on the Profiler component
  * @param  fluxEstFreq_Hz Flux estimate frequency (speed in Hz)
  */  
void PROFILER_setFluxEstFreq_Hz(PROFILER_Handle handle, const float_t fluxEstFreq_Hz)
{
  PROFILER_FLUXESTIM_setFluxEstFreq_Hz(&handle->fluxestimObj, fluxEstFreq_Hz);
}

/**
  * @brief  Sets measurement time allowed for flux estimate step
  * @param  pHandle Handle on the Profiler component
  * @param  time_seconds time in seconds allowed to execute the flux measurement step
  */
void PROFILER_setFluxEstMeasurementTime(PROFILER_Handle handle, const float_t time_seconds)
{
  PROFILER_FLUXESTIM_setMeasurementTime(&handle->fluxestimObj, time_seconds);
}

/**
  * @brief  Sets the number of polepairs
  * @param  pHandle Handle on the Profiler component
  * @param  polepairs number of polepairs
  */
void PROFILER_setPolePairs(PROFILER_Handle handle, const float_t polepairs)
{
  handle->PolePairs = polepairs;
}

/**
  * @brief  Sets the power goal for DCAC measurement
  * @param  pHandle Handle on the Profiler component
  * @param  powerGoal_W Level of power allowed to identify the motor
  */
void PROFILER_setPowerGoal_W(PROFILER_Handle handle, const float_t powerGoal_W)
{
  if (handle->state == PROFILER_STATE_Idle || handle->state == PROFILER_STATE_Complete || handle->state == PROFILER_STATE_Error)
  {
    handle->PowerDC_goal_W = powerGoal_W;
  }
}

/**
  * @brief  Clears result variables before each profiling
  * @param  pHandle Handle on the Profiler component
  */
void PROFILER_resetEstimates(PROFILER_Handle handle)
{
  handle->PowerDC_W = 0.0;
  handle->dutyDC = 0.0;
  handle->Idc_A  = 0.0;
  handle->Rs_dc  = 0.0;
  handle->Rs_ac  = 0.0;
  handle->Ld_H   = 0.0;
  handle->Lq_H   = 0.0f;
  handle->freqEst_Hz = 0.0;
  handle->freqHSO_Hz = 0.0;
  handle->Flux_Wb = 0.0;
  handle->debug_Flux_VpHz = 0.0;
  handle->debug_kV =  0.0;
  handle->KT_Nm_A = 0.0;
  handle->Isc_A   = 0.0;
  handle->CritFreq_Hz = 0.0;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
