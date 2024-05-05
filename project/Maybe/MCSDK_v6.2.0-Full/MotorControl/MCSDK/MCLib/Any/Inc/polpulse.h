/**
  ******************************************************************************
  * @file    polpulse.h
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
/* polpulse.h */

#ifndef _POLPULSE_H_
#define _POLPULSE_H_

#include "fixpmath.h"

#include <stddef.h>

#define POLPULSE_NUMSAMPLES 6

typedef enum _POLPULSE_State_e_
{
	POLPULSE_STATE_Idle,
	POLPULSE_STATE_Clear,
	POLPULSE_STATE_Precalcs,
	POLPULSE_STATE_WaitDecay,
	POLPULSE_STATE_WaitPulse,
	POLPULSE_STATE_Postcalcs,
	POLPULSE_STATE_PostcalcsComplete,
} POLPULSE_State_e;

typedef struct _POLPULSE_Params_
{
    float_t         VoltageScale;
    float_t         Ts;
    float_t         Lsd;
    float_t         PulseCurrentGoal_A;
    uint16_t        N;                   /* Number of pulse-periods */
    uint16_t        Nd;                  /* Number of decay-periods */
    uint16_t        N_Angles;            /* Number of directions to pulse in */
} POLPULSE_Params;

typedef struct _POLPULSE_Angles_
{
	/* Planned pulse angle, in per-unit and cos/sin */
	FIXP_CosSin_t       Phasor;
	fixp30_t             PulseAngle;
} POLPULSE_Angles;

typedef struct _POLPULSE_Data_
{
	/* Sampled data */
	Currents_Iab_t        IabSample;		/* Iab at end of pulse */
} POLPULSE_Data;


#define POLPULSE_SIZE_WORDS		(120)	/* Size in words, plus a margin */

typedef struct _POLPULSE_Obj_
{
    /* Contents of this structure is not public */
    uint16_t reserved[POLPULSE_SIZE_WORDS];
} POLPULSE_Obj;

typedef POLPULSE_Obj* POLPULSE_Handle;

/* Initialization */

POLPULSE_Handle POLPULSE_init(void *pMemory, const size_t size);
void POLPULSE_setParams(POLPULSE_Obj *obj, const POLPULSE_Params *pParams);

/* Functional */

void POLPULSE_run(POLPULSE_Obj *obj, const Currents_Iab_t	*pIab_pu, const Voltages_Uab_t *pUab_pu, const fixp30_t angle);
void POLPULSE_runBackground(POLPULSE_Obj *obj, const fixp_t oneoverUdc_pu, const fixp30_t Udc_pu);

void POLPULSE_clearTriggerPulse(POLPULSE_Obj *obj);
bool POLPULSE_isBusy(POLPULSE_Obj *obj);
void POLPULSE_resetState(POLPULSE_Obj *obj);
void POLPULSE_stopPulse(POLPULSE_Obj *obj);
void POLPULSE_trigger(POLPULSE_Obj *obj);

/* Accessors */

fixp30_t				POLPULSE_getAngleEstimated(const POLPULSE_Obj *obj);
float_t             POLPULSE_getCurrentGoal(const POLPULSE_Obj *obj);
uint16_t            POLPULSE_getDecayPeriods(const POLPULSE_Obj *obj);
Duty_Dab_t			POLPULSE_getDutyAB(const POLPULSE_Obj *obj);
uint16_t            POLPULSE_getNumAngles(const POLPULSE_Obj *obj);
bool				POLPULSE_getOverruleDuty(const POLPULSE_Obj *obj);
fixp30_t				POLPULSE_getPulseDuty(const POLPULSE_Obj *obj);
uint16_t            POLPULSE_getPulsePeriods(const POLPULSE_Obj *obj);
POLPULSE_State_e	POLPULSE_getState(const POLPULSE_Obj *obj);
bool				POLPULSE_getTriggerPulse(const POLPULSE_Obj *obj);

void                POLPULSE_setCurrentGoal(POLPULSE_Obj *obj, const float_t currentgoal);
void                POLPULSE_setDecayPeriods(POLPULSE_Obj *obj, const uint16_t periods);
void                POLPULSE_setLsd(POLPULSE_Obj *obj, const float_t Lsd);
void                POLPULSE_setNumAngles(POLPULSE_Obj *obj, const uint16_t angles);
void				POLPULSE_setPulseDone(POLPULSE_Obj *obj);
void                POLPULSE_setPulsePeriods(POLPULSE_Obj *obj, const uint16_t periods);

#endif /* _POLPULSE_H_ */

/* end of polpulse.h */

/************************ (C) COPYRIGHT 2023 Piak Electronic Design B.V. *****END OF FILE****/
