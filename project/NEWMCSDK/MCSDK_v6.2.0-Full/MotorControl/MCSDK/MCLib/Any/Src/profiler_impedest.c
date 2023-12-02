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
#include "profiler_impedest.h"

/**
  * @brief  todo
  *
  */
void PROFILER_IMPEDEST_init(PROFILER_IMPEDEST_Handle handle)
{
	handle->flag_inject = false;
}

/**
  * @brief  todo
  *
  */
void PROFILER_IMPEDEST_setParams(PROFILER_IMPEDEST_Obj *obj, PROFILER_Params *pParams)
{
	obj->isrPeriod_ms = FIXP30(1000.0f / pParams->isrFreq_Hz);
	obj->fullScaleCurrent_A = pParams->fullScaleCurrent_A;
	obj->fullScaleVoltage_V = pParams->fullScaleVoltage_V;

	float_t inject_freq_hz = 80.0f;
	obj->injectFreqKhz = FIXP30(inject_freq_hz / 1000.0f);
	obj->flag_inject = false;
	obj->inject_ref = 0;
	float_t filt_freq_radps = inject_freq_hz / 10.0f * M_TWOPI;
	obj->filt = FIXP30(filt_freq_radps / pParams->isrFreq_Hz);

	obj->angle_inject_pu = 0;
	obj->inject_out = 0;
	obj->dither = 0;
}

/**
  * @brief  todo
  *
  */
void PROFILER_IMPEDEST_run(PROFILER_IMPEDEST_Obj *obj, fixp30_t Ua_pu, fixp30_t Ia_pu)
{
	if (obj->flag_inject)
	{
		/* Rotate angle */
		fixp30_t angle_inject_pu = obj->angle_inject_pu;
		angle_inject_pu += FIXP30_mpy(obj->injectFreqKhz, obj->isrPeriod_ms);
		angle_inject_pu = angle_inject_pu & (FIXP30(1.0f) - 1);
		obj->angle_inject_pu = angle_inject_pu;

		/* Generate inject signal */
		FIXP_CosSin_t cossin;
		FIXP30_CosSinPU(angle_inject_pu, &cossin);
		obj->inject_out = FIXP30_mpy(obj->inject_ref, cossin.cos);

		/* Dithering */
		fixp30_t dither = obj->dither;
		dither = dither ^ 1;
		obj->dither = dither;

		Ua_pu += dither;
		Ia_pu += dither;

		/* Demodulate U */
		Vector_dq_t demod;
		demod.D = FIXP30_mpy(Ua_pu, cossin.cos);
		demod.Q = FIXP30_mpy(Ua_pu, cossin.sin);

		/* Filter U */
		fixp30_t filt = obj->filt;

		obj->U_demod_lp1.D += FIXP30_mpy((demod.D - obj->U_demod_lp1.D), filt);
		obj->U_demod_lp1.Q += FIXP30_mpy((demod.Q - obj->U_demod_lp1.Q), filt);

		obj->U_demod_lp2.D += FIXP30_mpy((obj->U_demod_lp1.D - obj->U_demod_lp2.D), filt);
		obj->U_demod_lp2.Q += FIXP30_mpy((obj->U_demod_lp1.Q - obj->U_demod_lp2.Q), filt);

		/* Demodulate I */
		demod.D = FIXP30_mpy(Ia_pu, cossin.cos);
		demod.Q = FIXP30_mpy(Ia_pu, cossin.sin);

		/* Filter I */
		obj->I_demod_lp1.D += FIXP30_mpy((demod.D - obj->I_demod_lp1.D), filt);
		obj->I_demod_lp1.Q += FIXP30_mpy((demod.Q - obj->I_demod_lp1.Q), filt);

		obj->I_demod_lp2.D += FIXP30_mpy((obj->I_demod_lp1.D - obj->I_demod_lp2.D), filt);
		obj->I_demod_lp2.Q += FIXP30_mpy((obj->I_demod_lp1.Q - obj->I_demod_lp2.Q), filt);
	}
}

/**
  * @brief  todo
  *
  */
void PROFILER_IMPEDEST_calculate(PROFILER_IMPEDEST_Obj *obj)
{
	/* Determine angle and magnitude of demodulated current vector */
	fixp30_t I_angle_pu, I_magn_pu;
	FIXP30_polar(obj->I_demod_lp2.D, obj->I_demod_lp2.Q, &I_angle_pu, &I_magn_pu);

	/* Rotate demodulated voltage vector to align current vector with real axis */
	FIXP_CosSin_t cossin;
	Vector_dq_t U_demod_aligned_lp;
	FIXP30_CosSinPU(I_angle_pu, &cossin);
	U_demod_aligned_lp.D = FIXP30_mpy(obj->U_demod_lp2.D, cossin.cos) + FIXP30_mpy(obj->U_demod_lp2.Q, cossin.sin);
	U_demod_aligned_lp.Q = FIXP30_mpy(obj->U_demod_lp2.Q, cossin.cos) - FIXP30_mpy(obj->U_demod_lp2.D, cossin.sin);

	/* Calculate R and L */

	float_t I_magn_A = FIXP30_toF(I_magn_pu) * obj->fullScaleCurrent_A;
	float_t tc = 1.0f / (FIXP30_toF(obj->injectFreqKhz) * 1000.0f * M_TWOPI);
	obj->Rs = (FIXP30_toF(U_demod_aligned_lp.D) * obj->fullScaleVoltage_V) / I_magn_A;
	obj->Ls = -(FIXP30_toF(U_demod_aligned_lp.Q) * obj->fullScaleVoltage_V) / I_magn_A * tc;
}

/* Accessors */

/* Getters */

/**
  * @brief  todo
  *
  */
fixp30_t PROFILER_IMPEDEST_getInject(PROFILER_IMPEDEST_Obj *obj)
{
	return obj->inject_out;
}

/**
  * @brief  todo
  *
  */
float_t PROFILER_IMPEDEST_getLs(PROFILER_IMPEDEST_Obj *obj)
{
	return obj->Ls;
}

/**
  * @brief  todo
  *
  */
float_t PROFILER_IMPEDEST_getRs(PROFILER_IMPEDEST_Obj *obj)
{
	return obj->Rs;
}

/* Setters */

/**
  * @brief  todo
  *
  */
void PROFILER_IMPEDEST_setFlagInject(PROFILER_IMPEDEST_Obj *obj, const bool value)
{
	obj->flag_inject = value;

	if (obj->flag_inject == false)
	{
		obj->angle_inject_pu = 0;
		obj->inject_out = 0;
	}
}

/**
  * @brief  todo
  *
  */
void PROFILER_IMPEDEST_setInjectFreq_kHz(PROFILER_IMPEDEST_Obj *obj, const fixp30_t value)
{
	obj->injectFreqKhz = value;
}

/**
  * @brief  todo
  *
  */
void PROFILER_IMPEDEST_setInjectRef(PROFILER_IMPEDEST_Obj *obj, const fixp30_t value)
{
	obj->inject_ref = value;
}

/************************ (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
