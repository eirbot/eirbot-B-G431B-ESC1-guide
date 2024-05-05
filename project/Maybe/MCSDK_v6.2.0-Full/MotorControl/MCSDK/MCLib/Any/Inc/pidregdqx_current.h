/**
  ******************************************************************************
  * @file    pidregdqx_current.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          PID current regulator of the Motor Control SDK.
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
  * @ingroup PIDRegdqx
  */

#ifndef _PIDREGDQX_CURRENT_H_
#define _PIDREGDQX_CURRENT_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "fixpmath.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup PIDRegdqx
  * @{
  */

/**
  * @brief  PID current regulator handler definition
  */
typedef struct _PIDREGDQX_CURRENT_s_
{
	/* configuration */
	FIXP_scaled_t	Kp_fps;                       /*!< @brief Kp gain expressed in fps */
	FIXP_scaled_t	Wi_fps;                       /*!< @brief Ki gain expressed in fps */
	float	KpAlign;                              /*!< @brief Kp gain applied during Rs DC estmation procedure */
	float	Kp;                                   /*!< @brief Kp gain */
	float			current_scale;                /*!< @brief current scaling factor  */
	float			voltage_scale;                /*!< @brief voltage scaling factor */
	float			pid_freq_hz;                  /*!< @brief frequency at which is called the PID regulator */
	float 			freq_scale_hz;                /*!< @brief frequecy scaling factor */
	float			duty_limit;                   /*!< @brief duty cycle limit */
	/* limits */                                  
	fixp24_t			maxModulation_squared;    /*!< @brief maximum modulation squared value */
	fixp24_t			MaxD;                     /*!< @brief D upper limit */
	fixp24_t			MinD;                     /*!< @brief D lower limit */
	fixp24_t			MaxQ;                     /*!< @brief Q upper limit */
	fixp24_t			MinQ;                     /*!< @brief Q lower limit */
	/* process */                                 
	fixp24_t			wfsT;                     /*!< @brief conversion factor */
	fixp24_t			ErrD;                     /*!< @brief D-current error */
	fixp24_t			ErrQ;                     /*!< @brief Q-current error */
	fixp24_t			UpD;                      /*!< @brief D proportional result  */
	fixp24_t			UpQ;                      /*!< @brief Q proportional result */
	fixp24_t			UiD;                      /*!< @brief D integral term */
	fixp24_t			UiQ;                      /*!< @brief Q integral term */
	fixp30_t			OutD;                     /*!< @brief PID output on d-axis */
	fixp30_t			OutQ;                     /*!< @brief PID output on q-axis */
	fixp24_t			compensation;	          /*!< @brief bus voltage compensation */
	bool			clippedD;                     /*!< @brief clipping status flag on D output */
	bool			clippedQ;                     /*!< @brief clipping status flag on Q output */
	bool			clipped;                      /*!< @brief overvall clipping status flag */
	bool			crosscompON;                  /*!< @brief cross coupling compensation activation flag */
} PIDREGDQX_CURRENT_s;

void PIDREGDQX_CURRENT_init(
		PIDREGDQX_CURRENT_s * pHandle,
		const float current_scale,
		const float voltage_scale,
		const float	pid_freq_hz,
		const float freq_scale_hz,
		const float duty_limit
	);
void PIDREGDQX_CURRENT_run( PIDREGDQX_CURRENT_s* pHandle, const fixp30_t errD, const fixp30_t errQ, const fixp30_t felec_pu);
bool PIDREGDQX_CURRENT_getClipped( PIDREGDQX_CURRENT_s* pHandle );
float PIDREGDQX_CURRENT_getKp_si( PIDREGDQX_CURRENT_s* pHandle );
fixp_t PIDREGDQX_CURRENT_getOutD(PIDREGDQX_CURRENT_s* pHandle);
fixp_t PIDREGDQX_CURRENT_getOutQ(PIDREGDQX_CURRENT_s* pHandle);
float PIDREGDQX_CURRENT_getWi_si( PIDREGDQX_CURRENT_s* pHandle );

void PIDREGDQX_CURRENT_setKp_si(PIDREGDQX_CURRENT_s* pHandle, const float Kp_si);
void PIDREGDQX_CURRENT_setKpWiRLmargin_si( PIDREGDQX_CURRENT_s* pHandle, const float Rsi, const float Lsi, const float margin);
void PIDREGDQX_CURRENT_setWi_si(PIDREGDQX_CURRENT_s* pHandle, const float Wi_si);
void PIDREGDQX_CURRENT_setUiD_pu( PIDREGDQX_CURRENT_s* pHandle, const fixp30_t Ui);
void PIDREGDQX_CURRENT_setUiQ_pu( PIDREGDQX_CURRENT_s* pHandle, const fixp30_t Ui);
void PIDREGDQX_CURRENT_setCompensation(PIDREGDQX_CURRENT_s* pHandle, const fixp24_t compensation);
void PIDREGDQX_CURRENT_setOutputLimitsD( PIDREGDQX_CURRENT_s* pHandle, const fixp30_t max_pu, const fixp30_t min_pu);
void PIDREGDQX_CURRENT_setOutputLimitsQ( PIDREGDQX_CURRENT_s* pHandle, const fixp30_t max_pu, const fixp30_t min_pu);
void PIDREGDQX_CURRENT_setMaxModulation_squared( PIDREGDQX_CURRENT_s* pHandle, const float  duty_limit);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* _PIDREGDQX_CURRENT_H_ */

/* end of pidregdqx_current.h */
