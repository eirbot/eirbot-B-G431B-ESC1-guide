/**
  ******************************************************************************
  * @file    profiler_impedest.h
  * @author  Piak Electronic Design B.V.
  * @brief   This file is part of the Motor Control SDK.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 Piak Electronic Design B.V.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* profiler_impedest.h */

#ifndef _PROFILER_IMPEDEST_H_
#define _PROFILER_IMPEDEST_H_

#include "profiler_types.h"

typedef struct _PROFILER_IMPEDEST_Obj_
{
	/* Configuration */
	fixp30_t			isrPeriod_ms;			/* ISR period in milliseconds, always < 1.0 */
	float_t			fullScaleCurrent_A;
	float_t			fullScaleVoltage_V;

	fixp30_t			injectFreqKhz;			/* Frequency to inject, in kHz */
	bool			flag_inject;
	fixp30_t			inject_ref;				/* Reference for magnitude of injected signal */
	fixp30_t			filt;					/* Filter constant for demodulated signal */

	/* State */
	fixp30_t			angle_inject_pu;		/* Angle of the injected signal */
	fixp30_t			inject_out;				/* Injected signal */
	fixp30_t			dither;

	Vector_dq_t		U_demod_lp1;
	Vector_dq_t		I_demod_lp1;

	Vector_dq_t		U_demod_lp2;
	Vector_dq_t		I_demod_lp2;

	/* Output */
	float_t			Rs;
	float_t			Ls;
} PROFILER_IMPEDEST_Obj;

typedef PROFILER_IMPEDEST_Obj *PROFILER_IMPEDEST_Handle;

void PROFILER_IMPEDEST_init(PROFILER_IMPEDEST_Handle handle);

void PROFILER_IMPEDEST_setParams(PROFILER_IMPEDEST_Obj *obj, PROFILER_Params *pParams);

void PROFILER_IMPEDEST_run(PROFILER_IMPEDEST_Obj *obj, fixp30_t Ua_pu, fixp30_t Ia_pu);

void PROFILER_IMPEDEST_calculate(PROFILER_IMPEDEST_Obj *obj);

/* Accessors */

/* Getters */

fixp30_t PROFILER_IMPEDEST_getInject(PROFILER_IMPEDEST_Obj *obj);

float_t PROFILER_IMPEDEST_getLs(PROFILER_IMPEDEST_Obj *obj);

float_t PROFILER_IMPEDEST_getRs(PROFILER_IMPEDEST_Obj *obj);

/* Setters */

void PROFILER_IMPEDEST_setFlagInject(PROFILER_IMPEDEST_Obj *obj, const bool value);

void PROFILER_IMPEDEST_setInjectFreq_kHz(PROFILER_IMPEDEST_Obj *obj, const fixp30_t value);

void PROFILER_IMPEDEST_setInjectRef(PROFILER_IMPEDEST_Obj *obj, const fixp30_t value);

#endif /* _PROFILER_IMPEDEST_H_ */

/* end of profiler_impedest.h */

/************************ (C) COPYRIGHT 2023 Piak Electronic Design B.V. *****END OF FILE****/
