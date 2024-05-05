/**
  ******************************************************************************
  * @file    mc_app_hooks.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements default motor control app hooks.
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
  * @ingroup MCAppHooks
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "mc_app_hooks.h"
#include "mc_config.h"
#include "esc.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MCTasks
  * @{
  */

/**
 * @defgroup MCAppHooks Motor Control Applicative hooks
 * @brief User defined functions that are called in the Motor Control tasks.
 *
 *
 * @{
 */

/**
 * @brief Hook function called right before the end of the MCboot function.
 *
 *
 *
 */
__weak void MC_APP_BootHook(void)
{
  esc_boot(&ESC_M1);
/* USER CODE BEGIN BootHook */

/* USER CODE END BootHook */
}

/**
 * @brief Hook function called right after the Medium Frequency Task for Motor 1.
 *
 *
 *
 */
__weak void MC_APP_PostMediumFrequencyHook_M1(void)
{
  esc_pwm_control(&ESC_M1);
/* USER SECTION BEGIN PostMediumFrequencyHookM1 */

/* USER SECTION END PostMediumFrequencyHookM1 */
}

/** @} */

/** @} */

/** @} */

/************************ (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
