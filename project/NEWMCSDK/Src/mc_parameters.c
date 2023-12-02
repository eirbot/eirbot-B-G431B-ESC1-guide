
/**
  ******************************************************************************
  * @file    mc_parameters.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides definitions of HW parameters specific to the
  *          configuration of the subsystem.
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
  */

/* Includes ------------------------------------------------------------------*/
//cstat -MISRAC2012-Rule-21.1
#include "main.h" //cstat !MISRAC2012-Rule-21.1
//cstat +MISRAC2012-Rule-21.1
#include "parameters_conversion.h"

#include "r3_2_g4xx_pwm_curr_fdbk.h"

#include "esc.h"
/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

#define FREQ_RATIO 1                /* Dummy value for single drive */
#define FREQ_RELATION HIGHEST_FREQ  /* Dummy value for single drive */

/**
  * @brief  Internal OPAMP parameters Motor 1 - three shunt - G4xx - Shared Resources
  *
  */
R3_3_OPAMPParams_t R3_3_OPAMPParamsM1 =
{

  .OPAMPSelect_1 = {
                     OPAMP2
                    ,OPAMP1
                    ,OPAMP1
                    ,OPAMP1
                    ,OPAMP1
                    ,OPAMP2
                   },
  .OPAMPSelect_2 = {
                     OPAMP3
                    ,OPAMP3
                    ,OPAMP3
                    ,OPAMP2
                    ,OPAMP2
                    ,OPAMP3
                   },

  .OPAMPConfig1 = {
                    OPAMP_UNCHANGED
                   ,OPAMP_UNCHANGED
                   ,OPAMP_UNCHANGED
                   ,OPAMP_UNCHANGED
                   ,OPAMP_UNCHANGED
                   ,OPAMP_UNCHANGED
  },
  .OPAMPConfig2 = {PIN_CONNECT|OPAMP3_NonInvertingInput_PB0
                  ,DIRECT_CONNECT|OPAMP3_NonInvertingInput_PB0
                  ,DIRECT_CONNECT|OPAMP3_NonInvertingInput_PB0
                  ,OPAMP_UNCHANGED
                  ,OPAMP_UNCHANGED
                  ,PIN_CONNECT|OPAMP3_NonInvertingInput_PB0
                 },
};
/**
  * @brief  Current sensor parameters Motor 1 - three shunt - G4
  */
//cstat !MISRAC2012-Rule-8.4
const R3_2_Params_t R3_2_ParamsM1 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio             = FREQ_RATIO,
  .IsHigherFreqTim       = FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCConfig1 = {
                  (uint32_t)(3U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(3U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(3U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(3U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(3U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(3U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                },
  .ADCConfig2 = {
                  (uint32_t)(12U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(18U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(18U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(3U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(3U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 ,(uint32_t)(12U << ADC_JSQR_JSQ1_Pos)
                | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                },

  .ADCDataReg1 = {
                   ADC2
                  ,ADC1
                  ,ADC1
                  ,ADC1
                  ,ADC1
                  ,ADC2
                 },
  .ADCDataReg2 = {
                   ADC1
                  ,ADC2
                  ,ADC2
                  ,ADC2
                  ,ADC2
                  ,ADC1
                  },
 //cstat +MISRAC2012-Rule-12.1 +MISRAC2012-Rule-10.1_R6

  /* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter     = REP_COUNTER,
  .Tafter                = TW_AFTER,
  .Tbefore               = TW_BEFORE,
  .Tsampling             = (uint16_t)SAMPLING_TIME,
  .Tcase2                = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3                = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME) / 2u,
  .TIMx                  = TIM1,

/* Internal OPAMP common settings --------------------------------------------*/
  .OPAMPParams           = &R3_3_OPAMPParamsM1,

/* Internal COMP settings ----------------------------------------------------*/
  .CompOCPASelection     = COMP1,
  .CompOCPAInvInput_MODE = INT_MODE,
  .CompOCPBSelection     = COMP2,
  .CompOCPBInvInput_MODE = INT_MODE,
  .CompOCPCSelection     = COMP4,
  .CompOCPCInvInput_MODE = INT_MODE,
  .DAC_OCP_ASelection    = DAC3,
  .DAC_OCP_BSelection    = DAC3,
  .DAC_OCP_CSelection    = DAC3,
  .DAC_Channel_OCPA      = LL_DAC_CHANNEL_1,
  .DAC_Channel_OCPB      = LL_DAC_CHANNEL_2,
  .DAC_Channel_OCPC      = LL_DAC_CHANNEL_2,
  .CompOVPSelection      = MC_NULL,
  .CompOVPInvInput_MODE  = NONE,
  .DAC_OVP_Selection     = MC_NULL,
  .DAC_Channel_OVP       = (uint32_t)0,

/* DAC settings --------------------------------------------------------------*/
  .DAC_OCP_Threshold     = 4083,
  .DAC_OVP_Threshold     = 23830,

};

ScaleParams_t scaleParams_M1 =
{
 .voltage = NOMINAL_BUS_VOLTAGE_V/(1.73205 * 32767), /* sqrt(3) = 1.73205 */
 .current = CURRENT_CONV_FACTOR_INV,
 .frequency = (1.15 * MAX_APPLICATION_SPEED_UNIT * U_RPM)/(32768* SPEED_UNIT)
};

const ESC_Params_t ESC_ParamsM1 =
{
  .Command_TIM        = TIM2,
  .Motor_TIM          = TIM1,
  .ARMING_TIME        = 200,
  .PWM_TURNOFF_MAX    = 500,
  .TURNOFF_TIME_MAX   = 500,
  .Ton_max            = ESC_TON_MAX,               /*!<  Maximum ton value for PWM (by default is 1800 us) */
  .Ton_min            = ESC_TON_MIN,               /*!<  Minimum ton value for PWM (by default is 1080 us) */
  .Ton_arming         = ESC_TON_ARMING,            /*!<  Minimum value to start the arming of PWM */
  .delta_Ton_max      = ESC_TON_MAX - ESC_TON_MIN,
  .speed_max_valueRPM = MOTOR_MAX_SPEED_RPM,       /*!< Maximum value for speed reference from Workbench */
  .speed_min_valueRPM = 1000,                      /*!< Set the minimum value for speed reference */
  .motor              = M1,
};

/* USER CODE BEGIN Additional parameters */

/* USER CODE END Additional parameters */

/******************* (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/

