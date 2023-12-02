/**
  ******************************************************************************
  * @file    r3_2_g4xx_pwm_curr_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          R3_2_G4XX_pwm_curr_fdbk component of the Motor Control SDK.
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
  * @ingroup R3_2_G4XX_pwm_curr_fdbk
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef R3_2_G4XX_PWMNCURRFDBK_H
#define R3_2_G4XX_PWMNCURRFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "pwm_curr_fdbk.h"

/* Exported defines --------------------------------------------------------*/
#define GPIO_NoRemap_TIM1 ((uint32_t)(0))
#define SHIFTED_TIMs      ((uint8_t) 1)
#define NO_SHIFTED_TIMs   ((uint8_t) 0)

#define NONE    ((uint8_t)(0x00))
#define EXT_MODE  ((uint8_t)(0x01))
#define INT_MODE  ((uint8_t)(0x02))

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @addtogroup R3_2_pwm_curr_fdbk
  * @{
  */

/* Exported types ------------------------------------------------------- */

#ifndef R3_3_OPAMP
#define R3_3_OPAMP
/**
  * @brief  Current feedback component 3-OPAMP parameters structure definition. Specific to G4XX.
  */
typedef const struct
{
  /* First OPAMP settings ------------------------------------------------------*/
  OPAMP_TypeDef *OPAMPSelect_1 [6] ;  /*!< Define for each sector first conversion which OPAMP is involved - Null otherwise */
  OPAMP_TypeDef *OPAMPSelect_2 [6] ;  /*!< Define for each sector second conversion which OPAMP is involved - Null otherwise */
  uint32_t OPAMPConfig1 [6]; /*!< Define the OPAMP_CSR_OPAMPINTEN and the OPAMP_CSR_VPSEL config for each ADC conversions*/
  uint32_t OPAMPConfig2 [6]; /*!< Define the OPAMP_CSR_OPAMPINTEN and the OPAMP_CSR_VPSEL config for each ADC conversions*/
} R3_3_OPAMPParams_t;
#endif

/*
  * @brief R3_2_G4XX_pwm_curr_fdbk component parameters structure definition.
  */
typedef const struct
{
  /* HW IP involved -----------------------------*/
  TIM_TypeDef *TIMx;               /* timer used for PWM generation.*/
  R3_3_OPAMPParams_t *OPAMPParams;  /*!< Pointer to the 3-OPAMP params struct.
                                       It must be #MC_NULL if internal OPAMP are not used. Specific to G4XX.*/
  COMP_TypeDef *CompOCPASelection;  /* Internal comparator used for Phase A protection.*/
  COMP_TypeDef *CompOCPBSelection;  /* Internal comparator used for Phase B protection.*/
  COMP_TypeDef *CompOCPCSelection;  /* Internal comparator used for Phase C protection.*/
  COMP_TypeDef *CompOVPSelection;   /* Internal comparator used for Over Voltage protection.*/
  DAC_TypeDef   *DAC_OCP_ASelection; /*!< DAC used for Phase A protection. Specific to G4XX.*/
  DAC_TypeDef   *DAC_OCP_BSelection; /*!< DAC used for Phase B protection. Specific to G4XX.*/
  DAC_TypeDef   *DAC_OCP_CSelection; /*!< DAC used for Phase C protection. Specific to G4XX.*/
  DAC_TypeDef   *DAC_OVP_Selection; /*!< DAC used for Over Voltage protection. Specific to G4XX.*/
  uint32_t DAC_Channel_OCPA;      /*!< DAC channel used for Phase A current protection.*/
  uint32_t DAC_Channel_OCPB;      /*!< DAC channel used for Phase B current protection.*/
  uint32_t DAC_Channel_OCPC;      /*!< DAC channel used for Phase C current protection.*/
  uint32_t DAC_Channel_OVP;       /*!< DAC channel used for Over Voltage protection.*/
  ADC_TypeDef *ADCDataReg1[6];
  ADC_TypeDef *ADCDataReg2[6];
  uint32_t ADCConfig1 [6] ; /* Values of JSQR for first ADC for 6 sectors */
  uint32_t ADCConfig2 [6] ; /* Values of JSQR for Second ADC for 6 sectors */

  /* PWM generation parameters --------------------------------------------------*/

  uint16_t Tafter;                      /* Sum of dead time plus max
                                            value between rise time and noise time
                                            expressed in number of TIM clocks.*/
  uint16_t Tsampling;                   /* Sampling time expressed in
                                            number of TIM clocks.*/
  uint16_t Tbefore;                     /* Sampling time expressed in
                                            number of TIM clocks.*/
  uint16_t Tcase2;                      /* Sampling time expressed in
                                            number of TIM clocks.*/
  uint16_t Tcase3;                      /* Sampling time expressed in
                                            number of TIM clocks.*/

  /* DAC settings --------------------------------------------------------------*/
  uint16_t DAC_OCP_Threshold;           /* Value of analog reference expressed
                                            as 16bit unsigned integer.
                                            Ex. 0 = 0V ; 65536 = VDD_DAC.*/
  uint16_t DAC_OVP_Threshold;           /* Value of analog reference expressed
                                            as 16bit unsigned integer.
                                            Ex. 0 = 0V ; 65536 = VDD_DAC.*/
  /* PWM Driving signals initialization ----------------------------------------*/
  uint8_t  RepetitionCounter;           /* Expresses the number of PWM
                                            periods to be elapsed before compare
                                            registers are updated again. In
                                            particular:
                                            RepetitionCounter= (2* #PWM periods)-1*/

  /* Internal COMP settings ----------------------------------------------------*/
  uint8_t       CompOCPAInvInput_MODE;      /* COMPx inverting input mode. It must be either
                                                equal to EXT_MODE or INT_MODE. */
  uint8_t       CompOCPBInvInput_MODE;      /* COMPx inverting input mode. It must be either
                                                equal to EXT_MODE or INT_MODE. */
  uint8_t       CompOCPCInvInput_MODE;      /* COMPx inverting input mode. It must be either
                                                equal to EXT_MODE or INT_MODE. */
  uint8_t       CompOVPInvInput_MODE;       /* COMPx inverting input mode. It must be either
                                                equal to EXT_MODE or INT_MODE. */

  /* Dual MC parameters --------------------------------------------------------*/
  uint8_t  FreqRatio;               /* Used in case of dual MC to
                                        synchronize TIM1 and TIM8. It has
                                        effect only on the second instanced
                                        object and must be equal to the
                                        ratio between the two PWM frequencies
                                        (higher/lower). Supported values are
                                        1, 2 or 3 */
  uint8_t  IsHigherFreqTim;         /* When bFreqRatio is greather than 1
                                        this param is used to indicate if this
                                        instance is the one with the highest
                                        frequency. Allowed value are: HIGHER_FREQ
                                        or LOWER_FREQ */

} R3_2_Params_t, *pR3_2_Params_t;

/*
  * @brief  Handles an instance of the R3_2_G4XX_pwm_curr_fdbk component.
  */
typedef struct
{
  PWMC_Handle_t _Super;     /*   */
  uint32_t PhaseAOffset;    /* Offset of Phase A current sensing network. */
  uint32_t PhaseBOffset;    /* Offset of Phase B current sensing network. */
  uint32_t PhaseCOffset;    /* Offset of Phase C current sensing network. */
  uint16_t Half_PWMPeriod;  /* Half PWM Period in timer clock counts. */
  uint16_t ADC_ExternalPolarityInjected; /* External trigger selection for ADC peripheral. */
  volatile uint8_t PolarizationCounter;
  uint8_t PolarizationSector; /* Sector selected during calibration phase. */

  pR3_2_Params_t pParams_str;
  bool ADCRegularLocked;    /*!< This flag is set when regular conversions are locked. Specific to G4XX. */     /* Cut 2.2 patch*/
} PWMC_R3_2_Handle_t;

/* Exported functions ------------------------------------------------------- */

/*
  * Initializes TIMx, ADC, GPIO, DMA1 and NVIC for current reading
  * in three shunt topology using STM32F30X and shared ADC.
  */
void R3_2_Init(PWMC_R3_2_Handle_t *pHandle);

/*
  * Stores into the handle the voltage present on Ia and
  * Ib current feedback analog channels when no current is flowing into the
  * motor.
  */
void R3_2_CurrentReadingPolarization(PWMC_Handle_t *pHdl);

/*
  * Computes and returns latest converted motor phase currents.
  */
void R3_2_GetPhaseCurrents(PWMC_Handle_t *pHdl, ab_t *Iab);

/*
  * Computes and return latest converted motor phase currents.
  */
void R3_2_GetPhaseCurrents_OVM(PWMC_Handle_t *pHdl, ab_t *Iab);

/*
  * Turns on low sides switches.
  */
void R3_2_TurnOnLowSides(PWMC_Handle_t *pHdl, uint32_t ticks);

/*
  * Enables PWM generation on the proper Timer peripheral acting on MOE bit.
  */
void R3_2_SwitchOnPWM(PWMC_Handle_t *pHdl);

/*
  * Disables PWM generation on the proper Timer peripheral acting on MOE bit.
  */
void R3_2_SwitchOffPWM(PWMC_Handle_t *pHdl);

/*
  * Configures the ADC for the current sampling related to sector 1.
  */
uint16_t R3_2_SetADCSampPointSectX(PWMC_Handle_t *pHdl);

/*
  * Configures the ADC for the current sampling related to sector X (X = [1..6] ).
  */
uint16_t R3_2_SetADCSampPointSectX_OVM(PWMC_Handle_t *pHdl);

/*
  * Contains the TIMx Update event interrupt.
  */
void *R3_2_TIMx_UP_IRQHandler(PWMC_R3_2_Handle_t *pHandle);

/*
  * Sets the PWM mode for R/L detection.
  */
void R3_2_RLDetectionModeEnable(PWMC_Handle_t *pHdl);

/*
  * Disables the PWM mode for R/L detection.
  */
void R3_2_RLDetectionModeDisable(PWMC_Handle_t *pHdl);

/*
  * Sets the PWM dutycycle for R/L detection.
  */
uint16_t R3_2_RLDetectionModeSetDuty(PWMC_Handle_t *pHdl, uint16_t hDuty);

/*
 * Turns on low sides switches and start ADC triggering.
 * This function is specific for MP phase.
 */
void R3_2_RLTurnOnLowSidesAndStart(PWMC_Handle_t *pHdl);

/*
  * Sets the calibrated offset.
  */
void R3_2_SetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets);

/*
  * Reads the calibrated offsets.
  */
void R3_2_GetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets);

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

#endif /*R3_2_G4XX_PWMNCURRFDBK_H*/

/************************ (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
