/**
  ******************************************************************************
  * @file    pidreg_current.h
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
/* pidreg_current.h */

#ifndef _PIDREG_CURRENT_H_
#define _PIDREG_CURRENT_H_

#include "fixpmath.h"

typedef struct _PIDREG_CURRENT_s_
{
	/* configuration */
	FIXP_scaled_t	Kp_fps;
	FIXP_scaled_t	Wi_fps;
	float			current_scale;
	float			voltage_scale;
	float			pid_freq_hz;

	/* limits */
	fixp24_t			Max;
	fixp24_t			Min;

	/* process */
	fixp24_t			Err;
	fixp24_t			Up;
	fixp24_t			Ui;
	fixp30_t			Out;
	fixp24_t			compensation;	/* bus voltage compensation */
	bool			clipped;
} PIDREG_CURRENT_s;

void PIDREG_CURRENT_init(
		PIDREG_CURRENT_s * pHandle,
		const float current_scale,
		const float voltage_scale,
		const float	pid_freq_hz
	);
fixp30_t PIDREG_CURRENT_run(PIDREG_CURRENT_s* pHandle, const fixp30_t err);
bool PIDREF_CURRENT_getClipped( PIDREG_CURRENT_s* pHandle );
float PIDREF_CURRENT_getKp_si( PIDREG_CURRENT_s* pHandle );
float PIDREF_CURRENT_getWi_si( PIDREG_CURRENT_s* pHandle );
void PIDREG_CURRENT_setKp_si(PIDREG_CURRENT_s* pHandle, const float Kp_si);
void PIDREG_CURRENT_setWi_si(PIDREG_CURRENT_s* pHandle, const float Wi_si);
void PIDREG_CURRENT_setUi_pu( PIDREG_CURRENT_s* pHandle, const fixp30_t Ui);
void PIDREG_CURRENT_setCompensation(PIDREG_CURRENT_s* pHandle, const fixp24_t compensation);

void PIDREG_CURRENT_setOutputLimits(PIDREG_CURRENT_s* pHandle, const fixp30_t max_pu, const fixp30_t min_pu);

#endif /* _PIDREG_CURRENT_H_ */

/* end of pidreg_current.h */

/************************ (C) COPYRIGHT 2023 Piak Electronic Design B.V. *****END OF FILE****/
