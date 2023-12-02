/**
 ******************************************************************************
 * @file    profiler_dcac.c
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   This file provides firmware functions of profiler DC/AC 
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
#include "profiler_dcac.h"
#include "mc_math.h"

  
void PROFILER_DCAC_stateMachine(PROFILER_DCAC_Handle handle, MOTOR_Handle motor);

/**
  * @brief  todo
  */
void PROFILER_DCAC_init(PROFILER_DCAC_Handle handle)
{
	handle->state = PROFILER_DCAC_STATE_Idle;
}

/**
  * @brief  todo
  */
void PROFILER_DCAC_setParams(PROFILER_DCAC_Handle handle, PROFILER_Params* pParams)
{
	handle->background_rate_hz = pParams->background_rate_hz;

	/* Number of medium frequency task cycles in the dc measurement time */
	PROFILER_DCAC_setMeasurementTime(handle, pParams->dcac_measurement_time_s);
	handle->CurToRamp_sf_pu    = FIXP30(1.0f / pParams->dcac_current_ramping_time_s / pParams->background_rate_hz);

	/* duty setpoint */
	handle->duty_maxgoal_pu = FIXP30(pParams->dcac_duty_maxgoal);
	handle->ac_freqkHz_pu   = FIXP30(pParams->dcac_ac_freq_Hz / 1000.0f);
	handle->dc_duty_pu      = FIXP30(0.0f);
	handle->ac_duty_pu      = FIXP30(0.0f);
	handle->PowerDC_goal_W  = pParams->dcac_PowerDC_goal_W;

	/* Current setpoint */
	handle->Id_dc_max_pu 	= FIXP30(pParams->dcac_DCmax_current_A / pParams->fullScaleCurrent_A);

	/* Ramp rate */
	handle->ramp_rate_duty_per_cycle = FIXP30(pParams->dcac_duty_maxgoal / pParams->dcac_duty_ramping_time_s  / pParams->background_rate_hz);

	handle->fullScaleVoltage_V     = pParams->fullScaleVoltage_V;
	handle->fullScaleCurrent_A     = pParams->fullScaleCurrent_A;
	handle->voltagefilterpole_rps  = pParams->voltagefilterpole_rps;
	handle->CurToRamp_sf_pu        = FIXP30(1.0f / pParams->fullScaleCurrent_A / pParams->dcac_current_ramping_time_s / pParams->background_rate_hz);
	handle->glue_dcac_fluxestim_on = pParams->glue_dcac_fluxestim_on;
}

/**
  * @brief  todo
  */
void PROFILER_DCAC_run(PROFILER_DCAC_Handle handle, MOTOR_Handle motor)
{
	/* This function is called from the high frequency task / motor control interrupt */
	/* After Sensorless and CurrentControl, so the DQ-values are available */

	{
		/* Calculate filtered voltages and currents */

		Voltages_Udq_t Udq = motor->pCurrCtrl->Udq_in_pu;
		Currents_Idq_t Idq = motor->pCurrCtrl->Idq_in_pu;

		/* Run filters with wLP = fs/(2^filtshift) (rad/sec) */
		int filtshift = 10;
		handle->UdqLP.D += (Udq.D - handle->UdqLP.D) >> filtshift;
		handle->UdqLP.Q += (Udq.Q - handle->UdqLP.Q) >> filtshift;
		handle->IdqLP.D += (Idq.D - handle->IdqLP.D) >> filtshift;
		handle->IdqLP.Q += (Idq.Q - handle->IdqLP.Q) >> filtshift;
	}

	switch (handle->state)
	{
	case PROFILER_DCAC_STATE_AC_Measuring:
		{
			/* During AC Measuring state, use the ZEST injection as a duty injection */
			Duty_Ddq_t  Ddq;
			ZEST_getIdq_ref_inject_pu(motor->pSPD->pZeST, &Ddq); /* AC component for injection */
			motor->pCurrCtrl->Ddq_ref_pu.D = handle->dc_duty_pu + Ddq.D;
		}
		break;

	default:
		/* Nothing to do */
		break;
	}
}

/**
  * @brief  todo
  */
void PROFILER_DCAC_runBackground(PROFILER_DCAC_Handle handle, MOTOR_Handle motor)
{
	/* This function is called from the medium frequency task */

	/* Counter counts down */
	if (handle->counter > 0) handle->counter--;

	switch (handle->state)
	{
	/* Duty ramping */
	case PROFILER_DCAC_STATE_RampUp:
		{
			fixp30_t ramp_rate = handle->ramp_rate_duty_per_cycle;
			fixp30_t delta_pu = handle->duty_maxgoal_pu - handle->dc_duty_pu; /* delta = target - current */
			handle->dc_duty_pu += FIXP_sat(delta_pu, ramp_rate, -ramp_rate);
			motor->pCurrCtrl->Ddq_ref_pu.D = handle->dc_duty_pu;
		}
		break;

	case PROFILER_DCAC_STATE_DC_Measuring:
		motor->pCurrCtrl->Ddq_ref_pu.D = handle->dc_duty_pu;
		break;

	case PROFILER_DCAC_STATE_AC_Measuring:
		/* update duty performed in PROFILER_DCAC_run() function */
		break;

	case PROFILER_DCAC_STATE_DConly:
		motor->pCurrCtrl->Ddq_ref_pu.D = handle->dc_duty_pu;
		break;

	case PROFILER_DCAC_STATE_RampDown:
		{
			fixp30_t ramp_rate = handle->ramp_rate_duty_per_cycle;
			fixp30_t delta_pu = 0 - handle->dc_duty_pu; /* delta = target - current */
			handle->dc_duty_pu += FIXP_sat(delta_pu, ramp_rate, -ramp_rate);
			motor->pCurrCtrl->Ddq_ref_pu.D = handle->dc_duty_pu;
		}
		break;

	case PROFILER_DCAC_STATE_Idle:
		/* No break */
	case PROFILER_DCAC_STATE_Complete:
		/* No break */
	case PROFILER_DCAC_STATE_Error:
		/* Nothing to do in these states */
		break;
	}

	PROFILER_DCAC_stateMachine(handle, motor);
}

/**
  * @brief  todo
  */
void PROFILER_DCAC_reset(PROFILER_DCAC_Handle handle, MOTOR_Handle motor)
{
	/* This is called before starting the DCAC process, and when the user aborts the Profiling */

	/* Reset the DCAC state */
	handle->state = PROFILER_DCAC_STATE_Idle;
	handle->error = PROFILER_DCAC_ERROR_None;

	/* Stop any ongoing actions */
	motor->pSPD->zestControl.enableControlUpdate = true;
	motor->pSPD->flagHsoAngleEqualsOpenLoopAngle = false;
	motor->pCurrCtrl->Ddq_ref_pu.D = 0;
	motor->pCurrCtrl->Ddq_ref_pu.Q = 0;

	/* Reset duty reached */
	handle->dc_duty_pu = 0;

}

/**
  * @brief  todo
  */
void PROFILER_DCAC_stateMachine(PROFILER_DCAC_Handle handle, MOTOR_Handle motor)
{
	PROFILER_DCAC_State_e state = handle->state;
	PROFILER_DCAC_State_e newState = state;

	switch (state)
	{
	case PROFILER_DCAC_STATE_Idle:
		/* Only called when profiler active */

		/* Set motor to required state */
		{
			/* Open loop angle, using duty reference Ddq_ref_pu */
			//motor->flagEnableClosedLoopAngle = false;
			motor->pSTC->SpeedControlEnabled = false;
			motor->pCurrCtrl->forceZeroPwm = false;
			motor->pSPD->OpenLoopAngle = FIXP30(0.0f);
			motor->pCurrCtrl->Ddq_ref_pu.D = 0;
			motor->pCurrCtrl->Ddq_ref_pu.Q = 0;

			motor->pSTC->speedref_source = FOC_SPEED_SOURCE_Default;
			motor->pSTC->speed_ref_active_pu = 0;
			motor->pSTC->speed_ref_ramped_pu = 0;
			motor->pSTC->speed_ref_pu = FIXP30(0.0f);

			motor->pSPD->zestControl.enableControlUpdate = false; /* Disable MC_ZEST_Control_Update() */
			motor->pSPD->flagHsoAngleEqualsOpenLoopAngle = true;  /* theta, cosTh and sinTh of HSO equal to openloopAngle */
			ZEST_setInjectMode(motor->pSPD->pZeST, ZEST_INJECTMODE_Auto);
			motor->pSPD->flagDisableImpedanceCorrection = true;
			IMPEDCORR_setRs(motor->pSPD->pImpedCorr, 0, 20); //set R and L to zero
			IMPEDCORR_setLs(motor->pSPD->pImpedCorr, 0, 20);
			handle->dc_duty_pu = 0;
			handle->ac_duty_pu = 0;
			handle->Id_dc_ref_pu = handle->Id_dc_max_pu; //reset initial current setpoint to max.
			handle->PowerDC_goal_pu = FIXP30(1.0f/1.5f * handle->PowerDC_goal_W / (handle->fullScaleCurrent_A * handle->fullScaleVoltage_V));
		}
		newState = PROFILER_DCAC_STATE_RampUp;
		break;

	case PROFILER_DCAC_STATE_RampUp:
		{
			Currents_Idq_t Idq = motor->pCurrCtrl->Idq_in_pu;
			FIXP_CosSin_t cossin_park;
			FIXP30_CosSinPU(motor->pCurrCtrl->angle_park_pu, &cossin_park);
			Voltages_Udq_t Udq = MCM_Park_Voltage(motor->pPWM->Uab_in_pu, &cossin_park);
			fixp30_t Umag = FIXP_mag(Udq.D, Udq.Q);
			fixp30_t Imag = FIXP_mag(Idq.D, Idq.Q);
			fixp30_t Pmag = FIXP30_mpy(Umag, Imag);
			handle->PowerDC_W = 1.5f * FIXP30_toF(Umag) * handle->fullScaleVoltage_V * FIXP30_toF(Imag) * handle->fullScaleCurrent_A;

			if ((Imag > handle->Id_dc_ref_pu) || (Pmag > handle->PowerDC_goal_pu) || (handle->dc_duty_pu == handle->duty_maxgoal_pu))
			{
				/* Current ramp-up complete. Actual duty in dc_duty_pu, prepare for the next phase */
				if (handle->dc_duty_pu == handle->duty_maxgoal_pu) /* motor with high resistance and low current, use filtered values */
				{
					handle->Id_dc_ref_pu = FIXP_mag(handle->IdqLP.D, handle->IdqLP.Q);
				}
				else
				{
					handle->Id_dc_ref_pu = FIXP_mag(Idq.D, Idq.Q); //make positive reference always
				}

				/* Set timer counter */
				handle->counter = handle->measurement_counts;
				newState = PROFILER_DCAC_STATE_DC_Measuring;
			}
		}
		break;

	case PROFILER_DCAC_STATE_DC_Measuring:
		// if measurement time is done, save values and move on
		if (handle->counter == 0)
		{
			/* Calculate resistance value from filtered values */
			fixp30_t Umag = FIXP_mag(handle->UdqLP.D, handle->UdqLP.Q);
			fixp30_t Imag = FIXP_mag(handle->IdqLP.D, handle->IdqLP.Q);
			if (Imag != 0)
			{
				handle->Rs_dc = (Umag * handle->fullScaleVoltage_V) / (Imag * handle->fullScaleCurrent_A);
			}
			handle->Umag_pu = Umag;
			handle->Imag_pu = Imag;
			handle->PowerDC_W = 1.5f * FIXP30_toF(Umag) * handle->fullScaleVoltage_V * FIXP30_toF(Imag) * handle->fullScaleCurrent_A;

			/* Start the AC measurement based on DC effective duty (keeping dead-time out of equation) */
			float_t dutyDC = 2 * FIXP30_toF(handle->Umag_pu) / FIXP30_toF(motor->pVBus->Udcbus_in_pu);
			handle->ac_duty_pu = FIXP30(dutyDC * 0.5f);
			handle->counter = handle->measurement_counts;

			ZEST_setFreqInjectkHz(motor->pSPD->pZeST, handle->ac_freqkHz_pu); //inject AC frequency
			ZEST_setIdInject_pu(motor->pSPD->pZeST, handle->ac_duty_pu);      //inject duty as if it was current...
			ZEST_setThresholdInjectActiveCurrent_A(motor->pSPD->pZeST, 0.0f); //remove threshold based on maxmotorcurrent
			newState = PROFILER_DCAC_STATE_AC_Measuring;
		}
		break;

	case PROFILER_DCAC_STATE_AC_Measuring:
		{
			if (handle->counter == 0)
			{
				handle->Rs_inject = ZEST_getR(motor->pSPD->pZeST);
				handle->Ls_inject = ZEST_getL(motor->pSPD->pZeST);
				fixp20_t Rsdc     = FIXP20(handle->Rs_dc);
				fixp20_t Rsinject = FIXP20(handle->Rs_inject);
				fixp30_t Lsinject = FIXP30(handle->Ls_inject);
				/* Check AC and DC resistance to match */
				if ((Rsinject  < FIXP_mpy(Rsdc, FIXP(0.8f))) || (Rsinject > FIXP_mpy(Rsdc, FIXP(1.4f))))
				{
					handle->error = PROFILER_DCAC_ERROR_ResistanceAC;
				}
				/* Check Ls: negative Ls can result at too low injection levels */
				if (Lsinject <= FIXP(0.0f))
				{
					handle->error = PROFILER_DCAC_ERROR_Inductance;
				}

				if (handle->error != PROFILER_DCAC_ERROR_None)
				{
					newState = PROFILER_DCAC_STATE_Error;
				}
				else
				{
					/* Set correct currentcontroller-parameters */
					PIDREGDQX_CURRENT_setKpWiRLmargin_si(&motor->pCurrCtrl->pid_IdIqX_obj, handle->Rs_inject, handle->Ls_inject, 5.0f);
					/* Set correct ImpedanceCorrection-parameters */
					PROFILER_DCAC_setImpedCorr_RsLs(handle, motor);
					ZEST_setIdInject_pu(motor->pSPD->pZeST, FIXP30(0.0f)); //stop injection.
					ZEST_setThresholdInjectActiveCurrent_A(motor->pSPD->pZeST, 0.02 * handle->Rs_dc); //set threshold to intended level
					if (handle->glue_dcac_fluxestim_on)
					{
						handle->counter = 20;
						newState = PROFILER_DCAC_STATE_DConly;
					}
					else
					{
						/* Start the ramp-down */
						newState = PROFILER_DCAC_STATE_RampDown;
					}
				}
			}
		}
		break;

	case PROFILER_DCAC_STATE_DConly:
		if (handle->counter == 0)
		{
			newState = PROFILER_DCAC_STATE_Complete;
		}
		break;

	case PROFILER_DCAC_STATE_RampDown:
		// Reduce current reference by ramp rate
		if (handle->dc_duty_pu == FIXP30(0.0f))
		{
			newState = PROFILER_DCAC_STATE_Complete;
		}
		break;

	case PROFILER_DCAC_STATE_Complete:
		/* No action */
		break;

	case PROFILER_DCAC_STATE_Error:
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
float_t PROFILER_DCAC_getMeasurementTime(PROFILER_DCAC_Handle handle)
{
	return handle->measurement_time_sec;
}

/**
  * @brief  todo
  */
void PROFILER_DCAC_setIdDCMax(PROFILER_DCAC_Handle handle, const float_t id_dc_max)
{
	handle->Id_dc_max_pu = FIXP30(id_dc_max / handle->fullScaleCurrent_A);
}

/**
  * @brief  todo
  */
void PROFILER_DCAC_setImpedCorr_RsLs(PROFILER_DCAC_Handle handle, MOTOR_Handle motor)
{
	{
		/* write estimated values of R and L to impedance correction */
		IMPEDCORR_setRs_si(motor->pSPD->pImpedCorr, handle->Rs_dc);
		RSEST_setRsRatedOhm(motor->pSPD->pRsEst, handle->Rs_dc);

		IMPEDCORR_setLs_si(motor->pSPD->pImpedCorr, handle->Ls_inject);
	}
	{
		/* Set other inductance variables */

		float_t Ls_pu_flt = handle->Ls_inject * handle->voltagefilterpole_rps * handle->fullScaleCurrent_A / handle->fullScaleVoltage_V;
		FIXPSCALED_floatToFIXPscaled(Ls_pu_flt, &motor->pFocVars->Ls_Rated_pu_fps);
		FIXPSCALED_floatToFIXPscaled(Ls_pu_flt, &motor->pSPD->Ls_Active_pu_fps);
		fixp24_t Ls_henry_fixp24 = FIXP24(handle->Ls_inject);
		motor->pFocVars->Ls_Rated_H  = Ls_henry_fixp24;
		motor->pFocVars->Ls_Active_H = Ls_henry_fixp24;
	}
}

/**
  * @brief  todo
  */
void PROFILER_DCAC_setMeasurementTime(PROFILER_DCAC_Handle handle, const float_t time_seconds)
{
	handle->measurement_time_sec = time_seconds;
	handle->measurement_counts = (uint32_t) (time_seconds * handle->background_rate_hz);
}


/************************ (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
