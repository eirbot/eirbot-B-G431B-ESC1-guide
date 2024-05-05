/**
  ******************************************************************************
  * @file    pidregdqx_current.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the PID current regulator component of the Motor Control SDK
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
  * @ingroup PIDRegdqx
  */
#include "pidregdqx_current.h"

/** @addtogroup MCSDK
  * @{
  */

/**
  * @defgroup PIDRegdqx PID current regulator
  *
  * @brief PID current regulator component of the Motor Control SDK
  *
  * The PID current regulator component implements the following control functions:
  * 
  * * A proportional-integral controller, implemented by the PIDREGDQX_CURRENT_run() function:
  * * A circle limitation feature that limits the output duty vector according to the maximum 
  * modulation value configured by the user thanks to the function PIDREGDQX_CURRENT_setMaxModulation_squared().
  * * A DC bus compensation to mitigate DC bus ripple.
  *
  * ![](pidreg_current.png)
  *
  * * Input:
  *    * `I_dq_Ref`: currents vector references
  *    * `I_dq` : currents vector measured
  * * Output:
  *    * `Duty_dq:` Duty vector
  *
  * If needed the PID current regulator can apply a cross-coupling compensation feature to increase stability at high speed.
  *   
  * ![](pidreg_crosscomp_current.png) 
  *  
  * * Input:
  *    * `I_dq_Ref`: currents vector references
  *    * `I_dq` : currents vector measured
  *    * `?`: Electrical speed from delta angle (low pass filtered)
  * * Output:
  *    * `Duty_dq`: Duty vector
  *
  * Each of the gain parameters, can be set, at run time and independently, via the PIDREGDQX_CURRENT_setKp_si(),
  * PIDREGDQX_CURRENT_setWi_si(). 
  * 
  * A PID Current Regulator component needs to be initialized before it can be used. This is done with the PIDREGDQX_CURRENT_init() 
  * function that sets the intergral term to 0 and initializes the component data structure.
  * 
  * To keep the computed values within limits, the component features the possibility to constrain the integral term 
  * within a range of values bounded by the PIDREGDQX_CURRENT_setOutputLimitsD() and  PIDREGDQX_CURRENT_setOutputLimitsQ() functions.
  * 
  * Handling a process with a PID Controller may require some adjustment to cope with specific situations. To that end, the 
  * PID speed regulator component provides functions to set the integral term (PIDREGDQX_CURRENT_setUiD_pu() and PIDREGDQX_CURRENT_setUiQ_pu()).
  * @{
  */
 
#define PU_FMT				(30)		/*!< @brief Per unit format, external format */
#define SUM_FTM				(24)		/*!< @brief Summing format, internal format */ 

/**
  * @brief  Initializes PID current regulator component.
  *         It Should be called during Motor control middleware initialization
  * @param  pHandle PID current regulator handler
  * @param  current_scale current scaling factor
  * @param  voltage_scale voltage scaling factor
  * @param  pid_freq_hz PID regulator execution frequency
  * @param  freq_scale_hz frequency scaling factor
  * @param  duty_limit duty cycle limit
  */
void PIDREGDQX_CURRENT_init(
		PIDREGDQX_CURRENT_s * pHandle,
		const float current_scale,
		const float voltage_scale,
		const float	pid_freq_hz,
		const float freq_scale_hz,
		const float	duty_limit
	)
{
	pHandle->crosscompON = false; /* only needed ON when stability at highest speeds is an issue */
	pHandle->Kp_fps.value = 0;
	pHandle->Kp_fps.fixpFmt = 30;
	pHandle->Wi_fps.value = 0;
	pHandle->Wi_fps.fixpFmt = 30;
	pHandle->maxModulation_squared = FIXP((double)(duty_limit * duty_limit));
	pHandle->MaxD = FIXP24(0.7f);
	pHandle->MinD = FIXP24(-0.7f);
	pHandle->UpD = FIXP24(0.0f);
	pHandle->UiD = FIXP24(0.0f);
	pHandle->OutD = FIXP30(0.0f);
	pHandle->MaxQ = FIXP24(0.7f);
	pHandle->MinQ = FIXP24(-0.7f);
	pHandle->UpQ = FIXP24(0.0f);
	pHandle->UiD = FIXP24(0.0f);
	pHandle->compensation = FIXP24(1.0f);

	pHandle->current_scale = current_scale;
	pHandle->voltage_scale = voltage_scale;
	pHandle->freq_scale_hz = freq_scale_hz;
	pHandle->pid_freq_hz = pid_freq_hz;
	pHandle->wfsT = FIXP((MATH_TWO_PI * (double)freq_scale_hz / (double)pid_freq_hz));
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Computes the output of a PID current regulator component, sum of its proportional
  *         and integral terms
  * @param  pHandle PID current regulator handler
  * @param  errD D current error
  * @param  errQ Q current error
  * @param  felec_pu motor electrical frequency 
  */
void PIDREGDQX_CURRENT_run( PIDREGDQX_CURRENT_s* pHandle, const fixp30_t errD, const fixp30_t errQ, const fixp30_t felec_pu)
{
	fixp24_t maxD = pHandle->MaxD;
	fixp24_t minD = pHandle->MinD;
	fixp24_t maxQ = pHandle->MaxQ;
	fixp24_t minQ = pHandle->MinQ;
	fixp24_t uiD  = pHandle->UiD;
	fixp24_t uiQ  = pHandle->UiQ;
	fixp24_t welecT = FIXP(0.0f);

	if(pHandle->crosscompON)
	{
		welecT = FIXP30_mpy(felec_pu, pHandle->wfsT);
	}

	/* Error */
	pHandle->ErrD = (errD >> (PU_FMT-SUM_FTM));
	pHandle->ErrQ = (errQ >> (PU_FMT-SUM_FTM));

	/* Proportional term D*/
	fixp24_t upD = FIXP_mpyFIXPscaled(pHandle->ErrD, &pHandle->Kp_fps);
	upD = FIXP_rsmpy(upD, pHandle->compensation);
	/* Proportional term Q*/
	fixp24_t upQ = FIXP_mpyFIXPscaled(pHandle->ErrQ, &pHandle->Kp_fps);
	upQ = FIXP_rsmpy(upQ, pHandle->compensation);

	/* Integral term including cross-term D*/
	uiD += FIXP_mpyFIXPscaled(upD, &pHandle->Wi_fps) - FIXP_mpy(upQ,welecT);
	pHandle->clippedD = ((uiD <= minD) || (uiD >= maxD));
	uiD = FIXP_sat(uiD, maxD, minD);
	pHandle->UiD = uiD;
	/* Summing term D*/
	fixp24_t sumD =FIXP_sat(upD + uiD, maxD, minD);

   // Circle limitation based on sumD
	fixp24_t d_duty_squared = FIXP_mpy(sumD, sumD);
	fixp24_t q_duty_squared = pHandle->maxModulation_squared - d_duty_squared;
	maxQ = FIXP24_sqrt(q_duty_squared);
	minQ = -maxQ;
	pHandle->MaxQ = maxQ;
	pHandle->MinQ = minQ;

	/* Integral term Q*/
	uiQ += FIXP_mpyFIXPscaled(upQ, &pHandle->Wi_fps) + FIXP_mpy(upD,welecT);
	pHandle->clippedQ = ((uiQ <= minQ) || (uiQ >= maxQ));
	uiQ = FIXP_sat(uiQ, maxQ, minQ);
	pHandle->UiQ = uiQ;

	/* Summing term Q*/
	fixp24_t sumQ =FIXP_sat(upQ + uiQ, maxQ, minQ);
	/*back to 30 */
	fixp30_t outD = (sumD << (PU_FMT-SUM_FTM));
	fixp30_t outQ = (sumQ << (PU_FMT-SUM_FTM));

	/* Store DQ values for monitoring */
	pHandle->UpD = upD;
	pHandle->OutD = outD;

	pHandle->UpQ = upQ;
	pHandle->OutQ = outQ;
	pHandle->clipped = pHandle->clippedD || pHandle->clippedQ;
}

/**
  * @brief  Gets overall clipping status
  * @param  pHandle PID current regulator handler
  * @retval  true if clipped false otherwise 
  */
bool PIDREGDQX_CURRENT_getClipped( PIDREGDQX_CURRENT_s* pHandle )
{
	return pHandle->clipped;
}

/**
  * @brief  Gets Kp gain in SI unit
  * @param  pHandle PID current regulator handler
  * @retval Kp in float format
  */
float PIDREGDQX_CURRENT_getKp_si( PIDREGDQX_CURRENT_s* pHandle )
{
	float Kp_pu = FIXPSCALED_FIXPscaledToFloat(&pHandle->Kp_fps);

	float Kp_si =  Kp_pu / pHandle->current_scale * pHandle->voltage_scale;

	return Kp_si;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Gets the PID output on D axis
  * @param  pHandle PID current regulator handler
  * @retval outD signal
  */
fixp_t PIDREGDQX_CURRENT_getOutD(PIDREGDQX_CURRENT_s* pHandle)
{
	return pHandle->OutD;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Gets the PID output on Q axis
  * @param  pHandle PID current regulator handler
  * @retval outD signal
  */
fixp_t PIDREGDQX_CURRENT_getOutQ(PIDREGDQX_CURRENT_s* pHandle)
{
	return pHandle->OutQ;
}

/**
  * @brief  Gets Ki gain in SI unit
  * @param  pHandle PID current regulator handler
  * @retval Ki gain
  */
float PIDREGDQX_CURRENT_getWi_si( PIDREGDQX_CURRENT_s* pHandle )
{
	float Wi_pu = FIXPSCALED_FIXPscaledToFloat(&pHandle->Wi_fps);

	float Wi_si = Wi_pu * pHandle->pid_freq_hz;

	return Wi_si;
}

/**
  * @brief  Sets Kp gain expressed in SI unit
  * @param  pHandle PID current regulator handler
  */
void PIDREGDQX_CURRENT_setKp_si( PIDREGDQX_CURRENT_s* pHandle, const float Kp)
{
	// Parameter Kp is in V/A (or Ohm)

	/* Convert to per unit, in full scale voltage per full scale current */
	float Kp_pu = Kp * pHandle->current_scale / pHandle->voltage_scale;

	/* Convert Kp_pu to scaled value, and store */
	FIXPSCALED_floatToFIXPscaled(Kp_pu, &pHandle->Kp_fps);
}

/**
  * @brief  Sets Kp and Ki gain according to Rs and Ls value of the motor
  * @param  pHandle PID current regulator handler
  * @param  Rsi motor resistance expressed in Ohm
  * @param  Lsi motor inductance expressed in Henry
  * @param  margin margin to apply for Kp computation  
  */
void PIDREGDQX_CURRENT_setKpWiRLmargin_si( PIDREGDQX_CURRENT_s* pHandle, const float Rsi, const float Lsi, const float margin)
{
  float kp_idq = (2.0f / margin) * Lsi * pHandle->pid_freq_hz;	/* using duty_scale=2 unit V/A take gain-margin (margin about 5) */
  float wi_idq = (Rsi / Lsi);					/* unit rad/s */
  pHandle->Kp = kp_idq;
  PIDREGDQX_CURRENT_setKp_si(pHandle, kp_idq);
  PIDREGDQX_CURRENT_setWi_si(pHandle, wi_idq);
}


/**
  * @brief  Sets Ki gain in SI unit
  * @param  pHandle PID current regulator handler
  * @param  Wi Ki gain expressed in rad/s 
  */
void PIDREGDQX_CURRENT_setWi_si( PIDREGDQX_CURRENT_s* pHandle, const float Wi)
{
	// Parameter Wi is in 1/s (or rad/s)
	// Wi unit is frequency rad/s in series with Kp, dimension of Kp*Wi = V/As (voltage per charge, being 1/Farad)

	/* Convert to unit per interrupt frequency */
	float Wi_pu = Wi / pHandle->pid_freq_hz;
	FIXPSCALED_floatToFIXPscaled(Wi_pu, &pHandle->Wi_fps);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Sets D integral term
  * @param  pHandle PID current regulator handler
  * @param  Ui integral term value
  */
void PIDREGDQX_CURRENT_setUiD_pu( PIDREGDQX_CURRENT_s* pHandle, const fixp30_t Ui)
{
	// Parameter Ui is in the same unit as the output, per unit duty
	// Internally the Ui is stored in a different format

	pHandle->UiD = (Ui >> (PU_FMT-SUM_FTM));
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Sets Q integral term
  * @param  pHandle PID current regulator handler
  * @param  Ui integral term value
  */
void PIDREGDQX_CURRENT_setUiQ_pu( PIDREGDQX_CURRENT_s* pHandle, const fixp30_t Ui)
{
	// Parameter Ui is in the same unit as the output, per unit duty
	// Internally the Ui is stored in a different format

	pHandle->UiQ = (Ui >> (PU_FMT-SUM_FTM));
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Sets bus voltage compensation factor
  * @param  pHandle PID current regulator handler
  * @param  compensation bus voltage compensation factor
  */
void PIDREGDQX_CURRENT_setCompensation( PIDREGDQX_CURRENT_s* pHandle, const fixp24_t compensation)
{
	pHandle->compensation = compensation;
}

/**
  * @brief  Sets the D upper and lower output limit of a PID component
  * @param  pHandle PID current regulator handler
  * @param  max_pu upper limit in per-unit
  * @param  mix_pu lower limit in per-unit
  */
void PIDREGDQX_CURRENT_setOutputLimitsD( PIDREGDQX_CURRENT_s* pHandle, const fixp30_t max_pu, const fixp30_t min_pu)
{
	pHandle->MaxD = (max_pu >> (PU_FMT-SUM_FTM));
	pHandle->MinD = (min_pu >> (PU_FMT-SUM_FTM));
}

/**
  * @brief  Sets the Q upper and lower output limit of a PID component
  * @param  pHandle PID current regulator handler
  * @param  max_pu upper limit in per-unit
  * @param  mix_pu lower limit in per-unit
  */
void PIDREGDQX_CURRENT_setOutputLimitsQ( PIDREGDQX_CURRENT_s* pHandle, const fixp30_t max_pu, const fixp30_t min_pu)
{
	pHandle->MaxQ = (max_pu >> (PU_FMT-SUM_FTM));
	pHandle->MinQ = (min_pu >> (PU_FMT-SUM_FTM));
}

/**
  * @brief  Sets squared maximum modulation value
  * @param  pHandle PID current regulator handler
  * @param  duty_limit maximum modulation  
  */
void PIDREGDQX_CURRENT_setMaxModulation_squared( PIDREGDQX_CURRENT_s* pHandle, const float  duty_limit)
{
  pHandle->maxModulation_squared = FIXP((double)(duty_limit * duty_limit));
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
