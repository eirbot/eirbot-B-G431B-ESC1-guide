/**
  ******************************************************************************
  * @file    pidreg_speed.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the PID speed regulator component of the Motor Control SDK
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
  * @ingroup PIDRegSpeed
  */

#include "pidreg_speed.h"

/** @addtogroup MCSDK
  * @{
  */

/**
  * @defgroup PIDRegSpeed PID speed regulator
  *
  * @brief PID speed regulator component of the Motor Control SDK
  *
  * The PID speed regulator component implements the following control functions:
  *
  * * A proportional-integral controller, implemented by the PIDREG_SPEED2_run() function:
  *
  * ![](pidreg_speed.png)
  *
  * * Input:
  *    * `Speed_user_ref`: Electrical reference speed (in Hz) from user
  *    * `?`: Electrical speed from delta angle (low pass filtered)
  *    * `?Emf`: Electrical speed from angular speed minus correction signal (low pass filtered)
  * * Output:
  *    * `I_q_ref`: Iq current reference
  *
  * Note:
  * Input `err` represents the speed error without zest-correction and is used for proportional gain Kp only. Without zest-correction in the feedback signal, higher proportional gains can be used while keeping the system stable.
  * Input `errIncSpd` is the discrete derivative of the estimated angle that also includes the zest-correction. While standing still, zest will do something to compensate for any system imperfections and offsets, hence `err` will
  * contain a non-zero signal that represents a non-zero speed. Hence a corrected angle is required for integral speed (position) control, otherwise position-drift will occur.
  * In case a Ki is commanded, the shaft should act as a mechanical torsional spring. Standstill should result in a fixed position without drift at any constant load. Hence `errIncSpd` is used for the integration part.  
  *   
  * Each of the gain parameters, can be set, at run time and independently, via the PIDREG_SPEED_setKp_si(),
  * PIDREG_SPEED_setKp_si(). 
  * 
  * A PID Speed Regulator component needs to be initialized before it can be used. This is done with the PIDREG_SPEED_init() 
  * function that sets the intergral term to 0 and initializes the component data structure.
  * 
  * To keep the computed values within limits, the component features the possibility to constrain the integral term 
  * within a range of values bounded by the PIDREG_SPEED_setOutputLimits() function.
  * 
  * Handling a process with a PID Controller may require some adjustment to cope with specific situations. To that end, the 
  * PID speed regulator component provides functions to set the integral term (PIDREG_SPEED_setUi_pu()).
  * 
  *
  * @{
  */
 
#define PU_FMT				(30)		/*!< @brief Per unit format, external format */
#define SUM_FTM				(24)		/*!< @brief Summing format, internal format */
#define HALF_BIT                ((1L << (PU_FMT-SUM_FTM-1))-1) /*!< @brief equal to 2^(PU_FMT-SUM_FTM -1) -1 = 32-1 = 31 in standard case*/

/**
  * @brief  Initializes PID speed regulator component.
  *
  *         It Should be called during Motor control middleware initialization.
  * @param  pHandle PID current regulator handler
  * @param  current_scale current scaling factor
  * @param  frequency_scale frequency scaling factor
  * @param  pid_freq_PID regulator execution frequency
  */
void PIDREG_SPEED_init(
		PIDREG_SPEED_s* pHandle,
		const float current_scale,
		const float frequency_scale,
		const float	pid_freq_hz
	)
{
	pHandle->Kp_fps.value = 0;
	pHandle->Kp_fps.fixpFmt = 30;
	pHandle->Ki_fps.value = 0;
	pHandle->Ki_fps.fixpFmt = 30;
	pHandle->Max = FIXP24(0.7f);
	pHandle->Min = FIXP24(-0.7f);
	pHandle->Up = FIXP24(0.0f);
	pHandle->Ui = FIXP24(0.0f);
	pHandle->Out = FIXP30(0.0f);
    pHandle->dither = 0;

	pHandle->current_scale = current_scale;
	pHandle->frequency_scale = frequency_scale;
	pHandle->pid_freq_hz = pid_freq_hz;
}

/**
  * @brief  Computes the output of a PID speed regulator component, sum of its proportional
  *         and integral terms
  * @param  pHandle PID current regulator handler
  * @param  err speed error
  */
fixp30_t PIDREG_SPEED_run( PIDREG_SPEED_s* pHandle, const fixp30_t err )
{
	fixp24_t max = pHandle->Max;
	fixp24_t min = pHandle->Min;
	fixp24_t ui = pHandle->Ui;

	/* Error */
	fixp24_t err_24 = (err >> (PU_FMT-SUM_FTM));

	/* Proportional term */
	fixp24_t up = FIXP_mpyFIXPscaled(err_24, &pHandle->Kp_fps);

	/* Integral term */
	ui += FIXP_mpyFIXPscaled(err_24, &pHandle->Ki_fps);
	ui  = FIXP_sat(ui, max, min);
	pHandle->Ui = ui;

	fixp24_t sum = FIXP_sat(up + ui, max, min);
	fixp30_t out = (sum << (PU_FMT-SUM_FTM));

	/* Store values for monitoring */
	pHandle->Err = err_24;
	pHandle->Up = up;
	pHandle->Out = out;

	return (out);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Computes the output of a PID speed regulator component, sum of its proportional
  *         and integral terms
  * @param  pHandle PID current regulator handler
  * @param  err speed error
  * @param  err speed error
  */
fixp30_t PIDREG_SPEED2_run( PIDREG_SPEED_s* pHandle, const fixp30_t err, const fixp30_t errIncSpd)
{
	fixp24_t max = pHandle->Max;
	fixp24_t min = pHandle->Min;
	fixp24_t ui = pHandle->Ui;

	/* Error */
	 fixp24_t err_24       = ((err + HALF_BIT) >> (PU_FMT-SUM_FTM));
	 fixp24_t errIncSpd_24 = ((errIncSpd + HALF_BIT) >> (PU_FMT-SUM_FTM));

	/* Proportional term */
	fixp24_t up = FIXP_mpyFIXPscaled(err_24, &pHandle->Kp_fps);

	/* Integral term */
	pHandle->dither = (pHandle->dither + 1) & 1;        
        ui += FIXP_mpyFIXPscaled(errIncSpd_24, &pHandle->Ki_fps) + pHandle->dither;

        pHandle->clipped = ((ui <= min) || (ui >= max));
	ui  = FIXP_sat(ui, max, min);
	pHandle->Ui = ui;

	fixp24_t sum = FIXP_sat(up + ui, max, min);
	fixp30_t out = (sum << (PU_FMT-SUM_FTM));

	/* Store values for monitoring */
	pHandle->Err = err_24;
	pHandle->Up = up;
	pHandle->Out = out;

	return (out);
}

/**
  * @brief  Gets overall clipping status
  * @param  pHandle PID current regulator handler
  * @retval bool clipping status
  */
bool PIDREG_SPEED_getClipped( PIDREG_SPEED_s* pHandle )
{
	return pHandle->clipped;
}
/* end of PIDREG_SPEED_getClipped( PIDREG_SPEED_s* pHandle )*/

/**
  * @brief  Gets Kp gain in SI unit
  * @param  pHandle PID speed regulator handler
  * @retval float Kp gain in SI unit
  */
float PIDREG_SPEED_getKp_si( PIDREG_SPEED_s* pHandle )
{
	float Kp_pu = FIXPSCALED_FIXPscaledToFloat(&pHandle->Kp_fps);

	float Kp_si = Kp_pu / pHandle->frequency_scale * pHandle->current_scale;

	return Kp_si;
}

/**
  * @brief  Gets Ki gain in SI unit
  * @param  pHandle PID speed regulator handler
  * @retval float Ki gain in SI unit
  */
float PIDREG_SPEED_getKi_si( PIDREG_SPEED_s* pHandle )
{
	float Ki_pu = FIXPSCALED_FIXPscaledToFloat(&pHandle->Ki_fps);

	float Ki_si = Ki_pu / pHandle->frequency_scale * pHandle->current_scale * pHandle->pid_freq_hz;

	return Ki_si;
}

/**
  * @brief  Sets Kp gain in SI unit
  * @param  pHandle PID current regulator handler
  * @param  Kp Kp gain
  */
void PIDREG_SPEED_setKp_si( PIDREG_SPEED_s* pHandle, const float Kp)
{
	// Parameter Kp is in current per speed, A/Hz (electrical)

	// User may want to set using different scale, like A/rad*s^-1, that is done external to this function

	/* Convert to per unit, in full scale current per full scale frequency */
	float Kp_pu = Kp * pHandle->frequency_scale / pHandle->current_scale;

	/* Convert Kp_pu to scaled value, and store */
	FIXPSCALED_floatToFIXPscaled(Kp_pu, &pHandle->Kp_fps);
}

/**
  * @brief  Sets Ki gain in SI unit
  * @param  pHandle PID current regulator handler
  * @param  Ki Ki gain
  */
void PIDREG_SPEED_setKi_si( PIDREG_SPEED_s* pHandle, const float Ki)
{
	/* Parameter Ki is in current per angle, A/rad; */

	/* Convert to per unit, in full scale voltage per full scale current per pid_freq_hz */
	float Ki_pu = Ki * pHandle->frequency_scale / pHandle->current_scale / pHandle->pid_freq_hz;

	FIXPSCALED_floatToFIXPscaled(Ki_pu, &pHandle->Ki_fps);
}

/**
  * @brief  Sets integral term
  * @param  pHandle PID current regulator handler
  * @param  Ui integral term
  */
void PIDREG_SPEED_setUi_pu( PIDREG_SPEED_s* pHandle, const fixp30_t Ui)
{
	// Parameter Ui is in the same unit as the output, per unit duty
	// Internally the Ui is stored in a different format

	pHandle->Ui = (Ui >> (PU_FMT-SUM_FTM));
}

/**
  * @brief  Sets PID regulator output limits 
  * @param  pHandle PID current regulator handler
  * @param  max_pu upper limit in per-unit
  * @param  mix_pu lower limit in per unit
  */
void PIDREG_SPEED_setOutputLimits(PIDREG_SPEED_s* pHandle, const fixp30_t max_pu, const fixp30_t min_pu)
{
	pHandle->Max = (max_pu >> (PU_FMT-SUM_FTM));
	pHandle->Min = (min_pu >> (PU_FMT-SUM_FTM));
}

/**
  * @brief  Gets PID regulator maximum output limits 
  * @param  pHandle PID current regulator handler
  * @retval fixp30_t maximum output limits
  */
fixp30_t PIDREG_SPEED_getOutputLimitMax(PIDREG_SPEED_s* pHandle)
{
	return ((pHandle->Max) << (PU_FMT-SUM_FTM));
}

/**
  * @brief  Gets PID regulator minimum output limits 
  * @param  pHandle PID current regulator handler
  * @retval fixp30_t minimum output limits
  */
fixp30_t PIDREG_SPEED_getOutputLimitMin(PIDREG_SPEED_s* pHandle)
{
	return ((pHandle->Min) << (PU_FMT-SUM_FTM));
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
