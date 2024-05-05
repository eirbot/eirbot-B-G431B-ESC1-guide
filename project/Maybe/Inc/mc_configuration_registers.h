
/**
  ******************************************************************************
  * @file    mc_configuration_registers.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides the definitions needed to build the project
  *          configuration information registers.
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

#ifndef MC_CONFIGURATION_REGISTERS_H
#define MC_CONFIGURATION_REGISTERS_H

#include "mc_type.h"

typedef struct
{
  uint32_t SDKVersion;
  uint8_t MotorNumber;
  uint8_t MCP_Flag;
  uint8_t MCPA_UARTA_LOG;
  uint8_t MCPA_UARTB_LOG;
  uint8_t MCPA_STLNK_LOG;
  uint8_t Padding;
} __attribute__ ((packed)) GlobalConfig_reg_t;

typedef struct
{
    float_t polePairs;
    float_t ratedFlux;
    float_t rs;
    float_t rsSkinFactor;
    float_t ls;
    float_t ld;
    float_t maxCurrent;
    float_t mass_copper_kg;
    float_t cooling_tau_s;
    char_t name[24];
} __attribute__ ((packed)) MotorConfig_reg_t;

typedef  struct
{
  uint32_t maxMechanicalSpeed;
  float_t maxReadableCurrent;
  float_t nominalCurrent;
  uint16_t nominalVoltage;
  uint8_t driveType;
  uint8_t padding;
} __attribute__ ((packed)) ApplicationConfig_reg_t;

typedef struct
{
  uint8_t primarySensor;
  uint8_t auxiliarySensor;
  uint8_t topology;
  uint8_t FOCRate;
  uint32_t PWMFrequency;
  uint16_t MediumFrequency;
  uint16_t configurationFlag1;
  uint16_t configurationFlag2;
} __attribute__ ((packed)) FOCFwConfig_reg_t;

#define ENO_SENSOR                0
#define EPLL                      1
#define ECORDIC                   2
#define EENCODER                  3
#define EHALL                     4
#define EHSO                      5
#define EZEST                     6

#define SDK_VERSION_MAIN   (0x6) /*!< [31:24] main version */
#define SDK_VERSION_SUB1   (0x2) /*!< [23:16] sub1 version */
#define SDK_VERSION_SUB2   (0x0) /*!< [15:8]  sub2 version */
#define SDK_VERSION_RC     (0x0) /*!< [7:0]  release candidate */
#define SDK_VERSION               ((SDK_VERSION_MAIN << 24U)\
                                  |(SDK_VERSION_SUB1 << 16U)\
                                  |(SDK_VERSION_SUB2 << 8U )\
                                  |(SDK_VERSION_RC))
/* configurationFlag1 definition */
#define FLUX_WEAKENING_FLAG       (1U)
#define FEED_FORWARD_FLAG         (1U << 1U)
#define MTPA_FLAG                 (1U << 2U)
#define PFC_FLAG                  (1U << 3U)
#define ICL_FLAG                  (1U << 4U)
#define RESISTIVE_BREAK_FLAG      (1U << 5U)
#define OCP_DISABLE_FLAG          (1U << 6U)
#define STGAP_FLAG                (1U << 7U)
#define POSITION_CTRL_FLAG        (1U << 8U)
#define VBUS_SENSING_FLAG         (1U << 9U)
#define TEMP_SENSING_FLAG         (1U << 10U)
#define VOLTAGE_SENSING_FLAG      (1U << 11U)
#define FLASH_CONFIG_FLAG         (1U << 12U)
#define DAC_CH1_FLAG              (1U << 13U)
#define DAC_CH2_FLAG              (1U << 14U)
#define OTF_STARTUP_FLAG          (1U << 15U)

/* configurationFlag2 definition */
#define OVERMODULATION_FLAG       (1U)
#define DISCONTINUOUS_PWM_FLAG    (1U << 1U)
#define PROFILER_FLAG             (1U << 13U)
#define DBG_MCU_LOAD_MEASURE_FLAG (1U << 14U)
#define DBG_OPEN_LOOP_FLAG        (1U << 15U)

/* MCP_Flag definition */
#define FLAG_MCP_OVER_STLINK       0U
#define FLAG_MCP_OVER_UARTA        (1U << 1U)
#define FLAG_MCP_OVER_UARTB        0U

#define configurationFlag1_M1     (FLUX_WEAKENING_FLAG|VBUS_SENSING_FLAG|TEMP_SENSING_FLAG)
#define configurationFlag2_M1     (DISCONTINUOUS_PWM_FLAG)

#define DRIVE_TYPE_M1              0
#define PRIM_SENSOR_M1            EHALL
#define AUX_SENSOR_M1             ENO_SENSOR
#define TOPOLOGY_M1               0
#define FOC_RATE_M1               1
#define PWM_FREQ_M1               40000

extern const char_t FIRMWARE_NAME[]; //cstat !MISRAC2012-Rule-18.8 !MISRAC2012-Rule-8.11
extern const char_t CTL_BOARD[]; //cstat !MISRAC2012-Rule-18.8 !MISRAC2012-Rule-8.11
extern const char_t *PWR_BOARD_NAME[NBR_OF_MOTORS];
extern const char_t *MOTOR_NAME[NBR_OF_MOTORS];
extern const GlobalConfig_reg_t globalConfig_reg;
extern const FOCFwConfig_reg_t *FOCConfig_reg[NBR_OF_MOTORS];
extern const MotorConfig_reg_t *MotorConfig_reg[NBR_OF_MOTORS];
extern const ApplicationConfig_reg_t *ApplicationConfig_reg[NBR_OF_MOTORS];

#endif /* MC_CONFIGURATION_REGISTERS_H */
/************************ (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
