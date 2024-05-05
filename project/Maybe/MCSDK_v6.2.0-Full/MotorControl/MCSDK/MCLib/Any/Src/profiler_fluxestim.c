/**
 ******************************************************************************
 * @file    profiler_fluxestim.c
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions of profiler 
 *          component
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
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "profiler_fluxestim.h"

  
void PROFILER_FLUXESTIM_stateMachine(PROFILER_FLUXESTIM_Handle handle, MOTOR_Handle motor);

/**
  * @brief  todo
  */
void PROFILER_FLUXESTIM_init(PROFILER_FLUXESTIM_Handle handle)
{
	handle->state = PROFILER_FLUXESTIM_STATE_Idle;
}

/**
  * @brief  todo
  */
void PROFILER_FLUXESTIM_setParams(PROFILER_FLUXESTIM_Handle handle, PROFILER_Params* pParams)
{
	/* Scaling */
	handle->fullScaleVoltage_V	= pParams->fullScaleVoltage_V;
	handle->fullScaleCurrent_A	= pParams->fullScaleCurrent_A;
	handle->fullScaleFreq_Hz	= pParams->fullScaleFreq_Hz;

	/* Number of medium frequency task cycles in the dc measurement time */
	handle->background_rate_hz = pParams->background_rate_hz;
	PROFILER_FLUXESTIM_setMeasurementTime(handle, pParams->fluxestim_measurement_time_s);
	handle->CurToRamp_sf_pu    = FIXP30(1.0f / pParams->fluxestim_current_ramping_time_s / pParams->background_rate_hz);
	handle->FreqToRamp_sf_pu   = FIXP30(1.0f / pParams->fluxestim_speed_ramping_time_s   / pParams->background_rate_hz);

	/* Current setpoint */
	handle->Id_ref_pu = FIXP30(pParams->fluxestim_Idref_A / pParams->fullScaleCurrent_A);

	/* Flux estimation frequency */
	PROFILER_FLUXESTIM_setFluxEstFreq_Hz(handle, pParams->fluxestim_speed_Hz);

	/* Ramp rate */
	handle->ramp_rate_freq_pu_per_cycle = FIXP30_mpy(handle->FluxEstFreq_pu, handle->FreqToRamp_sf_pu);
	handle->ramp_rate_A_pu_per_cycle    = FIXP30_mpy(handle->Id_ref_pu, handle->CurToRamp_sf_pu);
	handle->voltagefilterpole_rps  = pParams->voltagefilterpole_rps;
	handle->glue_dcac_fluxestim_on = pParams->glue_dcac_fluxestim_on;
	/* tune fluxEstimPole to 1/7 of measurement time to get 0.999 expectation ratio */
	handle->fluxEstimPoleTs        = FIXP30(7.0f /(pParams->fluxestim_measurement_time_s * pParams->background_rate_hz));
}

/**
  * @brief  todo
  */
void PROFILER_FLUXESTIM_run(PROFILER_FLUXESTIM_Handle handle, MOTOR_Handle motor)
{
	/* This function is called from the high frequency task / motor control interrupt */
	/* After Sensorless and CurrentControl, so the DQ-values are available */
}

/**
  * @brief  todo
  */
void PROFILER_FLUXESTIM_runBackground(PROFILER_FLUXESTIM_Handle handle, MOTOR_Handle motor)
{
	/* This function is called from the medium frequency task */

	/* Counter counts down */
	if (handle->counter > 0) handle->counter--;

	switch (handle->state)
	{
		case PROFILER_FLUXESTIM_STATE_Idle:
			handle->flux_Wb = FIXP30(0.0f); /* clear flux filter before new estimate */
			break;

		/* Current ramping */
		case PROFILER_FLUXESTIM_STATE_RampUpCur:
			{
				fixp30_t ramp_rate = handle->ramp_rate_A_pu_per_cycle;
				fixp30_t delta_pu = handle->Id_ref_pu - motor->pSTC->Idq_ref_pu.D; /* delta = target - current */
				motor->pSTC->Idq_ref_pu.D += FIXP_sat(delta_pu, ramp_rate, -ramp_rate);
			}
			break;

		case PROFILER_FLUXESTIM_STATE_RampUpFreq:
			{
				fixp30_t ramp_rate = handle->ramp_rate_freq_pu_per_cycle;
				fixp30_t delta_pu  = handle->FluxEstFreq_pu - motor->pSTC->speed_ref_pu; /* delta = target - current */
				motor->pSTC->speed_ref_pu += FIXP_sat(delta_pu, ramp_rate, -ramp_rate);
			}
			break;

		case PROFILER_FLUXESTIM_STATE_FluxRefAuto:
			{
				/* read and filter actual flux amplitude and copy to reference value: equilibrium will be found, use 7tau for accuracy */
				handle->flux_Wb += FIXP30_mpy((HSO_getFluxAmpl_Wb(motor->pSPD->pHSO) - handle->flux_Wb), handle->fluxEstimPoleTs);
				HSO_setFluxRef_Wb(motor->pSPD->pHSO, handle->flux_Wb);

				/* Update user visible rated flux */
				motor->pFocVars->Flux_Rated_VpHz = FIXP30_mpy(handle->flux_Wb, FIXP(M_TWOPI));
			}
			break;

		case PROFILER_FLUXESTIM_STATE_RampDownFreq:
			{
				fixp30_t ramp_rate = handle->ramp_rate_freq_pu_per_cycle;
				fixp30_t delta_pu  = 0 - motor->pSTC->speed_ref_pu; /* delta = target - present */
				motor->pSTC->speed_ref_pu += FIXP_sat(delta_pu, ramp_rate, -ramp_rate);
			}
			break;

		case PROFILER_FLUXESTIM_STATE_RampDownCur:
			{
				fixp30_t ramp_rate = handle->ramp_rate_A_pu_per_cycle;
				fixp30_t delta_pu = 0 - motor->pSTC->Idq_ref_pu.D; /* delta = target - current */
				motor->pSTC->Idq_ref_pu.D += FIXP_sat(delta_pu, ramp_rate, -ramp_rate);
			}
			break;

		case PROFILER_FLUXESTIM_STATE_Error:
			break;

		case PROFILER_FLUXESTIM_STATE_Complete:
			break;
	}

	PROFILER_FLUXESTIM_stateMachine(handle, motor);
}

/**
  * @brief  todo
  */
void PROFILER_FLUXESTIM_reset(PROFILER_FLUXESTIM_Handle handle, MOTOR_Handle motor)
{
	handle->state = PROFILER_FLUXESTIM_STATE_Idle;
	handle->error = PROFILER_FLUXESTIM_ERROR_None;

	motor->pSTC->Idq_ref_pu.D = 0;
	motor->pSTC->Idq_ref_pu.Q = 0;
	motor->pSTC->speed_ref_pu = 0;
}

/**
  * @brief  todo
  */
void PROFILER_FLUXESTIM_stateMachine(PROFILER_FLUXESTIM_Handle handle, MOTOR_Handle motor)
{
	PROFILER_FLUXESTIM_State_e state = handle->state;
	PROFILER_FLUXESTIM_State_e newState = state;

	switch (state)
	{
	case PROFILER_FLUXESTIM_STATE_Idle:
		/* Only called when profiler active */

		/* Set motor to required state */
		{
			/* Open loop current mode */
			motor->pSTC->SpeedControlEnabled = false;
			motor->pCurrCtrl->currentControlEnabled = true;
			motor->pSPD->closedLoopAngleEnabled = false;
			motor->pCurrCtrl->forceZeroPwm = false;
			if (handle->glue_dcac_fluxestim_on == false)
			{
				motor->pCurrCtrl->Ddq_ref_pu.D = 0;
			}
			motor->pCurrCtrl->Ddq_ref_pu.Q = 0;

			motor->pSTC->speedref_source = FOC_SPEED_SOURCE_Default;
			motor->pSTC->speed_ref_active_pu = 0;
			motor->pSTC->speed_ref_ramped_pu = 0;
			motor->pSTC->speed_ref_pu = FIXP30(0.0f);

			motor->pSPD->zestControl.enableControlUpdate = false; /* Disable MC_ZEST_Control_Update() */
			motor->pSPD->flagHsoAngleEqualsOpenLoopAngle = false; /* enable hso to follow EMF */
			ZEST_setInjectMode(motor->pSPD->pZeST, ZEST_INJECTMODE_None);

			if (handle->glue_dcac_fluxestim_on)
			{
				motor->pSTC->Idq_ref_pu.D = motor->pCurrCtrl->Idq_in_pu.D; /* Copy present current to reference */
				PIDREGDQX_CURRENT_setUiD_pu(&motor->pCurrCtrl->pid_IdIqX_obj,motor->pCurrCtrl->Ddq_ref_pu.D); /* Copy present duty to current controller integrator */
			}
		}
		{
			float_t myfreq = 100.0f; /* Set Flux observer cross-over frequency */
			HSO_setMinCrossOver_Hz(motor->pSPD->pHSO, myfreq);
			HSO_setMaxCrossOver_Hz(motor->pSPD->pHSO, myfreq);
			handle->flux_Wb = FIXP30(0.0f); /* Set Ref-flux to zero before speed rampup */
			HSO_setFluxRef_Wb(motor->pSPD->pHSO, handle->flux_Wb);
		}
		newState = PROFILER_FLUXESTIM_STATE_RampUpCur;
		break;

	case PROFILER_FLUXESTIM_STATE_RampUpCur:
		/* open-loop current mode */
		if (motor->pSTC->Idq_ref_pu.D == handle->Id_ref_pu)
		{
			motor->pSTC->I_max_pu = FIXP24_mpy(handle->Id_ref_pu, FIXP24(2.0f));
			newState = PROFILER_FLUXESTIM_STATE_RampUpFreq;
		}
		break;

	case PROFILER_FLUXESTIM_STATE_RampUpFreq:
		{
			Voltages_Uab_t Uab = motor->pPWM->Uab_in_pu;
			fixp30_t Umag = FIXP_mag(Uab.A, Uab.B);
			/* ramp speed up till set frequency or 50% of available duty */
			if ((motor->pSTC->speed_ref_pu == handle->FluxEstFreq_pu) || (Umag > (motor->pVBus->Udcbus_in_pu >> 2)))
			{
				/* Prepare the Flux measurement */
				handle->freqEst_pu = motor->pSTC->speed_ref_pu; //copy actual OL speed
				/* set new Flux observer cross-over frequency was 100Hz up to now*/
				float_t myfreq = 2 * FIXP30_toF(handle->freqEst_pu) * handle->fullScaleFreq_Hz;
				HSO_setMinCrossOver_Hz(motor->pSPD->pHSO, myfreq);
				HSO_setMaxCrossOver_Hz(motor->pSPD->pHSO, myfreq);
				handle->counter = handle->measurement_counts;
				newState = PROFILER_FLUXESTIM_STATE_FluxRefAuto;
			}
		}
		break;

	case PROFILER_FLUXESTIM_STATE_FluxRefAuto:
		if (handle->counter == 0)
		{
			handle->freqHSO_pu = HSO_getSpeedLP_pu(motor->pSPD->pHSO); //copy actual HSO speed
			fixp30_t freqHSO = FIXP_abs(handle->freqHSO_pu);
			fixp30_t freqRef = FIXP_abs(handle->freqEst_pu);
			if ((freqHSO < FIXP_mpy(freqRef, FIXP(0.60))) || (freqHSO > FIXP_mpy(freqRef, FIXP(1.50))))
			{
				handle->error = PROFILER_FLUXESTIM_ERROR_FluxNotValid;
			}
			newState = PROFILER_FLUXESTIM_STATE_RampDownFreq;
		}
		break;

	case PROFILER_FLUXESTIM_STATE_RampDownFreq:
		if (motor->pSTC->speed_ref_pu == FIXP30(0.0f))
		{
			HSO_setMinCrossOver_Hz(motor->pSPD->pHSO, 20.0f); /* return to previous settings */
			HSO_setMaxCrossOver_Hz(motor->pSPD->pHSO, 500.0f);
			//set Kp_speed to Imax / speed:(voltage/flux)
			newState = PROFILER_FLUXESTIM_STATE_RampDownCur;
		}
		break;

	case PROFILER_FLUXESTIM_STATE_RampDownCur:
		// Reduce current reference by ramp rate
		if (motor->pSTC->Idq_ref_pu.D == FIXP30(0.0f))
		{
			/* Done, set motor control back to normal functioning */

			ZEST_setInjectMode(motor->pSPD->pZeST, ZEST_INJECTMODE_Auto);
			motor->pSPD->flagDisableImpedanceCorrection = false; /* Enable Impedance Correction */
			motor->pSPD->zestControl.enableControlUpdate = true; /* Back to default situation */

			if (handle->error != PROFILER_FLUXESTIM_ERROR_None)
			{
				newState = PROFILER_FLUXESTIM_STATE_Error;
			}
			else
			{
				newState = PROFILER_FLUXESTIM_STATE_Complete;
			}
		}
		break;

	case PROFILER_FLUXESTIM_STATE_Complete:
		/* No action */
		break;

	case PROFILER_FLUXESTIM_STATE_Error:
		/* No action */
		break;
	}

	if (state != newState)
	{
		handle->state = newState;
	}
}

/* Accessors */
/**
  * @brief  todo
  */
float_t PROFILER_FLUXESTIM_getFluxEstFreq_Hz(const PROFILER_FLUXESTIM_Handle handle)
{
	return FIXP30_toF(handle->FluxEstFreq_pu) * handle->fullScaleFreq_Hz;
}

/**
  * @brief  todo
  */
float_t PROFILER_FLUXESTIM_getMeasurementTime(PROFILER_FLUXESTIM_Handle handle)
{
	return handle->measurement_time_s;
}

/**
  * @brief  todo
  */
void PROFILER_FLUXESTIM_setFluxEstFreq_Hz(PROFILER_FLUXESTIM_Handle handle, const float_t fluxEstFreq_Hz)
{
	handle->FluxEstFreq_pu = FIXP30(fluxEstFreq_Hz / handle->fullScaleFreq_Hz);
}

/**
  * @brief  todo
  */
void PROFILER_FLUXESTIM_setMeasurementTime(PROFILER_FLUXESTIM_Handle handle, const float_t time_seconds)
{
	handle->measurement_time_s = time_seconds;
	handle->measurement_counts = (uint32_t) (time_seconds * handle->background_rate_hz);
}


/************************ (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/

