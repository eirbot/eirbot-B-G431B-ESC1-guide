/**
  ******************************************************************************
  * @file    pidreg_speed.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          PID speed regulator of the Motor Control SDK.
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
  * @ingroup PIDRegSpeed
  */

#ifndef _PIDREG_SPEED_H_
#define _PIDREG_SPEED_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "fixpmath.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup PIDRegSpeed
  * @{
  */
  
/**
  * @brief  PID speed regulator handler definition
  */  
typedef struct _PIDREG_SPEED_s_
{
	/* configuration */
	FIXP_scaled_t	Kp_fps;             /*!< @brief Kp gain expressed in fps */
	FIXP_scaled_t	Ki_fps;             /*!< @brief Ki gain expressed in fps */
    fixp24_t         dither;            /*!< @brief dithering variable */
	float			current_scale;      /*!< @brief current scaling factor */
	float			frequency_scale;    /*!< @brief frequency scalimg factor */
	float			pid_freq_hz;		/*!< @brief Frequency at which the PID is called */

	/* limits */
	fixp24_t			Max;	        /*!< @brief upper PID limit in A_pu */
	fixp24_t			Min;			/*!< @brief lower PID limit in A_pu */

	/* process */
	fixp24_t			Err;            /*!< @brief speed error */
	fixp24_t			Up;             /*!< @brief proportional result */
	fixp24_t			Ui;             /*!< @brief integral result */
	fixp30_t			Out;            /*!< @brief PID output result */
	bool			clipped;            /*!< @brief clipping status flag */
} PIDREG_SPEED_s;

void PIDREG_SPEED_init(
		PIDREG_SPEED_s* pHandle,
		const float current_scale,
		const float frequency_scale,
		const float	pid_freq_hz
	);
fixp30_t PIDREG_SPEED_run(PIDREG_SPEED_s* pHandle, const fixp30_t err);
fixp30_t PIDREG_SPEED2_run( PIDREG_SPEED_s* pHandle, const fixp30_t err, const fixp30_t errIncSpd);
bool PIDREG_SPEED_getClipped( PIDREG_SPEED_s* pHandle );
float PIDREG_SPEED_getKp_si( PIDREG_SPEED_s* pHandle );
float PIDREG_SPEED_getKi_si( PIDREG_SPEED_s* pHandle );
void PIDREG_SPEED_setKp_si(PIDREG_SPEED_s* pHandle, const float Kp_si);
void PIDREG_SPEED_setKi_si(PIDREG_SPEED_s* pHandle, const float Ki_si);
void PIDREG_SPEED_setUi_pu( PIDREG_SPEED_s* pHandle, const fixp30_t Ui);

void PIDREG_SPEED_setOutputLimits(PIDREG_SPEED_s* pHandle, const fixp30_t max_pu, const fixp30_t min_pu);
fixp30_t PIDREG_SPEED_getOutputLimitMax(PIDREG_SPEED_s* pHandle);
fixp30_t PIDREG_SPEED_getOutputLimitMin(PIDREG_SPEED_s* pHandle);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* _PIDREG_SPEED_H_ */

