/**
  ******************************************************************************
  * @file    hso.h
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
/* hso.h */

#ifndef _HSO_H_
#define _HSO_H_

#include "fixpmath.h"
#include <stdbool.h>
#include <stddef.h>
#include <string.h> /* memset */

#define HSO_setflag_TrackAngle HSO_setFlag_TrackAngle		/* HSO_setflag_TrackAngle is deprecated */

typedef struct _HSO_Params_
{
    float_t         Flux_Wb;
    float_t         CrossOver_Hz;
    float_t         isrTime_s;
    float_t         FullScaleVoltage_V;
    float_t         FullScaleFreq_Hz;
    float_t         speedPole_rps;
    bool            flag_FilterAdaptive;
    float_t         Filter_adapt_fmin_Hz;
    float_t         Filter_adapt_fmax_Hz;
    float_t         CheckDirBW_Hz;
} HSO_Params;

#define HSO_SIZE_WORDS		(120)	/* Size in words, plus a margin */

typedef struct _HSO_Obj_
{
    /* Contents of this structure is not public */
    uint16_t reserved[HSO_SIZE_WORDS];
} HSO_Obj;

typedef HSO_Obj* HSO_Handle;

/* initialization */

static inline HSO_Handle HSO_init(void *pMemory, const size_t size)
{
    HSO_Handle handle = (HSO_Handle) NULL;

    if (size >= sizeof(HSO_Obj))
    {
        handle = (HSO_Handle) pMemory;
        memset(pMemory, 0, size);
    }

    return (handle);
}

extern void HSO_setParams(HSO_Handle handle, const HSO_Params *pParams);

/* functional */

extern void HSO_run(HSO_Handle handle, Voltages_Uab_t *pVback_ab, const fixp30_t Correction);

extern void HSO_adjustAngle_pu(HSO_Handle handle, const fixp30_t rotation_pu);

extern void HSO_clear(HSO_Handle handle);

extern void HSO_flip_angle(HSO_Handle handle);

/* accessors */

extern fixp30_t HSO_getAngle_pu(const HSO_Handle handle);

extern fixp30_t HSO_getCheckDir(const HSO_Handle handle);

extern void HSO_getCosSinTh_ab(const HSO_Handle handle, FIXP_CosSin_t *pCosSinTh);

extern FIXP_CosSin_t HSO_getCosSin(const HSO_Handle handle);

extern fixp30_t HSO_getDelta_theta_pu(const HSO_Handle handle);

extern fixp30_t HSO_getEmfSpeed_pu(const HSO_Handle handle);

extern Vector_ab_t HSO_getFlux_ab_Wb(const HSO_Handle handle);

extern fixp30_t HSO_getFluxAmpl_Wb(const HSO_Handle handle);

extern fixp30_t HSO_getFluxRef_Wb(const HSO_Handle handle);

extern fixp30_t HSO_getKoffset(const HSO_Handle handle);

extern fixp30_t HSO_getSpeedLP_pu(const HSO_Handle handle);

extern float_t HSO_getSpeedPole_rps(const HSO_Handle handle);

extern void HSO_setAngle_pu(HSO_Handle handle, const fixp30_t theta);

extern void HSO_setCrossOver_Hz(HSO_Handle handle, const float_t CrossOver_Hz);

extern void HSO_setCrossOver_pu(HSO_Handle handle, const fixp30_t CrossOver_pu);

extern void HSO_setFlag_TrackAngle(HSO_Handle handle, const bool value);

extern void HSO_setFluxPolar_pu(HSO_Handle handle, const fixp30_t angle_pu, const fixp30_t magn_pu);

extern void HSO_setFluxRef_Wb(HSO_Handle handle, const fixp30_t FluxWb);

extern void HSO_setKoffset(HSO_Handle handle, const fixp30_t value);

extern void HSO_setMaxCrossOver_Hz(HSO_Handle handle, const float_t value);

extern void HSO_setMinCrossOver_Hz(HSO_Handle handle, const float_t value);

extern void HSO_setObserverRatio(HSO_Handle handle, const float_t value);

extern void HSO_setSpeedPole_rps(HSO_Handle handle, const float_t value);

#endif /* _HSO_H_ */

/* end of hso.h */

/************************ (C) COPYRIGHT 2023 Piak Electronic Design B.V. *****END OF FILE****/
