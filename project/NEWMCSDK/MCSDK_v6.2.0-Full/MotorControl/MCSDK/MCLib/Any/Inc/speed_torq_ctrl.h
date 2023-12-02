/**
  ******************************************************************************
  * @file    speed_torq_ctrl.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          Speed & Torque Control component of the Motor Control SDK.
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
  * @ingroup SpeednTorqCtrlClassic
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SPEEDNTORQCTRLCLASS_H
#define SPEEDNTORQCTRLCLASS_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "pid_regulator.h"
#include "speed_pos_fdbk.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednTorqCtrl
  * @{
  */

/** @addtogroup SpeednTorqCtrlClassic
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  Speed & Torque Control parameters definition
  */
typedef struct
{
  MC_ControlMode_t Mode;               /*!< Modality of STC. It can be one of these two settings:
                                           MCM_TORQUE_MODE to enable the Torque mode or
                                           MCM_SPEED_MODE to enable the Speed mode. */
  int16_t TargetFinal;                 /*!< Backup of #hTargetFinal to be applied in the last step. */
  int32_t SpeedRefUnitExt;             /*!< Current mechanical rotor speed reference expressed in
                                            [SPEED_UNIT](measurement_units.md) multiplied by 65536.*/
  int32_t TorqueRef;                   /*!< Current motor torque reference. This value represents actually
                                            the Iq current expressed in digit multiplied by 65536. */
  uint32_t RampRemainingStep;          /*!< Number of steps remaining to complete the ramp. */
  PID_Handle_t *PISpeed;               /*!< The regulator used to perform the speed control loop. */
  SpeednPosFdbk_Handle_t *SPD;         /*!< The speed sensor used to perform the speed regulation. */
  int32_t IncDecAmount;                /*!< Increment/decrement amount to be applied to the reference value at each
                                            #CalcTorqueReference. */
  uint16_t STCFrequencyHz;             /*!< Frequency on which the user updates the torque reference calling
                                            #STC_CalcTorqueReference method expressed in Hz */
  uint16_t MaxAppPositiveMecSpeedUnit; /*!< Application maximum positive value of the rotor mechanical speed.
                                            Expressed in the unit defined by [SPEED_UNIT](measurement_units.md). */
  uint16_t MinAppPositiveMecSpeedUnit; /*!< Application minimum positive value of the rotor mechanical speed.
                                            Expressed in the unit defined by [SPEED_UNIT](measurement_units.md). */
  int16_t MaxAppNegativeMecSpeedUnit;  /*!< Application maximum negative value of the rotor mechanical speed.
                                            Expressed in the unit defined by [SPEED_UNIT](measurement_units.md). */
  int16_t MinAppNegativeMecSpeedUnit;  /*!< Application minimum negative value of the rotor mechanical speed.
                                            Expressed in the unit defined by [SPEED_UNIT](measurement_units.md). */
  uint16_t MaxPositiveTorque;          /*!< Maximum positive value of motor torque. This value represents actually
                                            the maximum Iq current expressed in digit. */
  int16_t MinNegativeTorque;           /*!< Minimum negative value of motor torque. This value represents actually
                                            the maximum Iq current expressed in digit. */
  MC_ControlMode_t ModeDefault;        /*!< Default STC modality. */
  int16_t MecSpeedRefUnitDefault;      /*!< Default mechanical rotor speed reference expressed in the unit defined by
                                            [SPEED_UNIT](measurement_units.md). */
  int16_t TorqueRefDefault;            /*!< Default motor torque reference. This value represents actually the Iq
                                            current reference expressed in digit. */
  int16_t IdrefDefault;                /*!< Default Id current reference expressed in digit. */
} SpeednTorqCtrl_Handle_t;



/* Initializes all the object variables */
void STC_Init(SpeednTorqCtrl_Handle_t *pHandle, PID_Handle_t *pPI, SpeednPosFdbk_Handle_t *SPD_Handle);

/* Resets the integral term of speed regulator */
void STC_Clear(SpeednTorqCtrl_Handle_t *pHandle);

/* Gets the current mechanical rotor speed reference */
int16_t STC_GetMecSpeedRefUnit(SpeednTorqCtrl_Handle_t *pHandle);

/* Gets the current motor torque reference */
int16_t STC_GetTorqueRef(SpeednTorqCtrl_Handle_t *pHandle);

/* Sets the mode of the speed and torque controller (Torque mode or Speed mode) */
void STC_SetControlMode(SpeednTorqCtrl_Handle_t *pHandle, MC_ControlMode_t bMode);

/* Gets the mode of the speed and torque controller */
MC_ControlMode_t STC_GetControlMode(SpeednTorqCtrl_Handle_t *pHandle);

/* Starts the execution of a ramp using new target and duration */
bool STC_ExecRamp(SpeednTorqCtrl_Handle_t *pHandle, int16_t hTargetFinal, uint32_t hDurationms);

/* Interrupts the execution of any previous ramp command */
void STC_StopRamp(SpeednTorqCtrl_Handle_t *pHandle);

/* Computes the new value of motor torque reference */
int16_t STC_CalcTorqueReference(SpeednTorqCtrl_Handle_t *pHandle);

/* Gets the Default mechanical rotor speed reference */
int16_t STC_GetMecSpeedRefUnitDefault(SpeednTorqCtrl_Handle_t *pHandle);

/* Returns the Application maximum positive rotor mechanical speed */
uint16_t STC_GetMaxAppPositiveMecSpeedUnit(SpeednTorqCtrl_Handle_t *pHandle);

/* Returns the Application minimum negative rotor mechanical speed */
int16_t STC_GetMinAppNegativeMecSpeedUnit(SpeednTorqCtrl_Handle_t *pHandle);

/* Checks if the settled speed or torque ramp has been completed */
bool STC_RampCompleted(SpeednTorqCtrl_Handle_t *pHandle);

/* Stops the execution of speed ramp */
bool STC_StopSpeedRamp(SpeednTorqCtrl_Handle_t *pHandle);

/* Sets in real time the speed sensor utilized by the FOC */
void STC_SetSpeedSensor(SpeednTorqCtrl_Handle_t *pHandle, SpeednPosFdbk_Handle_t *SPD_Handle);

/* Returns the speed sensor utilized by the FOC */
SpeednPosFdbk_Handle_t *STC_GetSpeedSensor(SpeednTorqCtrl_Handle_t *pHandle);

/* It returns the default values of Iqdref */
qd_t STC_GetDefaultIqdref(SpeednTorqCtrl_Handle_t *pHandle);

/* Sets the nominal current */
void STC_SetNominalCurrent(SpeednTorqCtrl_Handle_t *pHandle, uint16_t hNominalCurrent);

/* Forces the speed reference to the current speed */
void STC_ForceSpeedReferenceToCurrentSpeed(SpeednTorqCtrl_Handle_t *pHandle);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* SPEEDNTORQCTRLCLASS_H */

/************************ (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/

