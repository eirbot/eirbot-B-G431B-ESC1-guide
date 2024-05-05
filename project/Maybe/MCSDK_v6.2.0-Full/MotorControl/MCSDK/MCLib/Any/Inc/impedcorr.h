/**
  ******************************************************************************
  * @file    impedcorr.h
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
/* impedcorr.h */

#ifndef _IMPEDCORR_H_
#define _IMPEDCORR_H_

#include "fixpmath.h"
#include <stddef.h>

typedef struct _IMPEDCORR_Params_
{
    float_t		voltageFilterPole_rps;
    float_t		cycleFreq_Hz;
    float_t		fullScaleFreq_Hz;
    float_t     KsampleDelay;
	float_t		fullScaleCurrent_A;
	float_t		fullScaleVoltage_V;
} IMPEDCORR_Params;


#define IMPEDCORR_SIZE_WORDS		(50)	/* Size in words, plus a margin */

typedef struct _IMPEDCORR_Obj_
{
    /* Contents of this structure is not public */
    uint16_t reserved[IMPEDCORR_SIZE_WORDS];
} IMPEDCORR_Obj;

typedef IMPEDCORR_Obj* IMPEDCORR_Handle;

/* Initialization */

extern IMPEDCORR_Handle IMPEDCORR_init(void *pMemory, const size_t size);

extern void IMPEDCORR_setParams(IMPEDCORR_Handle pHandle, const IMPEDCORR_Params *pParams);

/* Functional */
extern Voltages_Uab_t IMPEDCORR_run(IMPEDCORR_Handle pHandle,
    const Currents_Iab_t* pIab_pu,
    const Voltages_Uab_t* pUab_pu,
    const fixp30_t       Fe_pu
);

/* Accessors */
extern void IMPEDCORR_getEmf_ab_pu(const IMPEDCORR_Handle handle, Voltages_Uab_t* pEmf_ab);
extern void IMPEDCORR_getEmf_ab_filt_pu(const IMPEDCORR_Handle handle, Voltages_Uab_t* pE_ab);
extern Voltages_Uab_t IMPEDCORR_getEmf_ab_filt(const IMPEDCORR_Handle handle);
extern void IMPEDCORR_getIab_filt_pu(const IMPEDCORR_Handle handle, Currents_Iab_t* pIab_filt);
extern Currents_Iab_t IMPEDCORR_getIab_filt(const IMPEDCORR_Handle handle);
extern float_t IMPEDCORR_getKSampleDelay(const IMPEDCORR_Handle handle);
extern float_t IMPEDCORR_getLs_si(const IMPEDCORR_Handle pHandle);
extern FIXP_scaled_t IMPEDCORR_getRs_pu_fps(const IMPEDCORR_Handle handle);
extern float_t IMPEDCORR_getRs_si(const IMPEDCORR_Handle pHandle);

extern void IMPEDCORR_setKSampleDelay(IMPEDCORR_Handle handle, const float_t value);
extern void IMPEDCORR_setLs(IMPEDCORR_Handle handle, const fixp_t Ls_pu, const fixpFmt_t fmt);
extern void IMPEDCORR_setRs(IMPEDCORR_Handle handle, const fixp_t Rs_pu, const fixpFmt_t fmt);
extern void IMPEDCORR_setLs_si(IMPEDCORR_Handle pHandle, const float_t Ls_H);
extern void IMPEDCORR_setRs_si(IMPEDCORR_Handle pHandle, const float_t Rs_Ohm);

#endif /* _IMPEDCORR_H_ */

/* end of impedcorr.h */

/************************ (C) COPYRIGHT 2023 Piak Electronic Design B.V. *****END OF FILE****/
