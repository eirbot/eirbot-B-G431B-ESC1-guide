/**
  ******************************************************************************
  * @file    mc_tasks.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implementes tasks definition.
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
  * @ingroup MCTasks
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MCTASKS_H
#define MCTASKS_H

/* Includes ------------------------------------------------------------------*/
#include "mc_parameters.h"

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup MCTasks Motor Control Tasks
  *
  * @brief Motor Control subsystem configuration and operation routines.
  *
  * @{
  */

/* Initializes the Motor subsystem core according to user defined parameters */
void MCboot(MCI_Handle_t *pMCIList[NBR_OF_MOTORS]);

/* Runs all the Tasks of the Motor Control cockpit */
void MC_RunMotorControlTasks(void);

/* Executes the Medium Frequency Task functions for each drive instance */
void MC_Scheduler(void);

/* Executes safety checks (e.g. bus voltage and temperature) for all drive instances */
void TSK_SafetyTask(void);

/* Executes the Motor Control duties that require a high frequency rate and a precise timing */
uint8_t TSK_HighFrequencyTask(void);

void UI_HandleStartStopButton_cb(void);

/* Reserves FOC execution on ADC ISR half a PWM period in advance */
void TSK_DualDriveFIFOUpdate(uint8_t Motor);

/* Puts the Motor Control subsystem in in safety conditions on a Hard Fault */
void TSK_HardwareFaultTask(void);

 /* Locks GPIO pins used for Motor Control to prevent accidental reconfiguration */
void mc_lock_pins(void);
/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* MCTASKS_H */
/******************* (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
