
/**
  ******************************************************************************
  * @file    mc_config.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Motor Control Subsystem components configuration and handler
  *          structures declarations.
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
  */

#ifndef MC_CONFIG_H
#define MC_CONFIG_H

#include "pid_regulator.h"
#include "speed_torq_ctrl.h"
#include "virtual_speed_sensor.h"
#include "ntc_temperature_sensor.h"
#include "revup_ctrl.h"
#include "pwm_curr_fdbk.h"
#include "mc_interface.h"
#include "mc_configuration_registers.h"
#include "regular_conversion_manager.h"
#include "r_divider_bus_voltage_sensor.h"
#include "virtual_bus_voltage_sensor.h"
#include "pqd_motor_power_measurement.h"

#include "r3_2_g4xx_pwm_curr_fdbk.h"

#include "esc.h"

#include "ramp_ext_mngr.h"
#include "circle_limitation.h"

#include "sto_speed_pos_fdbk.h"
#include "sto_pll_speed_pos_fdbk.h"
/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

extern RevUpCtrl_Handle_t RevUpControlM1;

extern PID_Handle_t PIDSpeedHandle_M1;
extern PID_Handle_t PIDIqHandle_M1;
extern PID_Handle_t PIDIdHandle_M1;
extern RegConv_t TempRegConv_M1;
extern NTC_Handle_t TempSensor_M1;

extern PWMC_R3_2_Handle_t PWM_Handle_M1;
extern SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1;
extern PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM1;
extern PQD_MotorPowMeas_Handle_t *pPQD_MotorPowMeasM1;
extern VirtualSpeedSensor_Handle_t VirtualSpeedSensorM1;
extern STO_Handle_t STO_M1;
extern STO_PLL_Handle_t STO_PLL_M1;
extern RegConv_t VbusRegConv_M1;
extern RDivider_Handle_t BusVoltageSensor_M1;
extern CircleLimitation_Handle_t CircleLimitationM1;
extern RampExtMngr_Handle_t RampExtMngrHFParamsM1;

extern MCI_Handle_t Mci[NBR_OF_MOTORS];
extern SpeednTorqCtrl_Handle_t *pSTC[NBR_OF_MOTORS];
extern PID_Handle_t *pPIDIq[NBR_OF_MOTORS];
extern PID_Handle_t *pPIDId[NBR_OF_MOTORS];
extern NTC_Handle_t *pTemperatureSensor[NBR_OF_MOTORS];
extern PQD_MotorPowMeas_Handle_t *pMPM[NBR_OF_MOTORS];
extern MCI_Handle_t* pMCI[NBR_OF_MOTORS];
extern ESC_Handle_t ESC_M1;
/* USER CODE BEGIN Additional extern */

/* USER CODE END Additional extern */

#endif /* MC_CONFIG_H */
/******************* (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
