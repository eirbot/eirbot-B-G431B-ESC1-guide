/**
  ******************************************************************************
  * @file    r1_g4xx_pwm_curr_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          R1_G4XX_pwm_curr_fdbk component of the Motor Control SDK.
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
  * @ingroup R1_G4XX_pwm_curr_fdbk
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef R1_G4XX_PWMNCURRFDBK_H
#define R1_G4XX_PWMNCURRFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "pwm_curr_fdbk.h"

/* Exported defines --------------------------------------------------------*/

#define NONE     ((uint8_t)(0x00))
#define EXT_MODE ((uint8_t)(0x01))
#define INT_MODE ((uint8_t)(0x02))
#define STBD3    0x02u /*!< Flag to indicate which phase has been distorted
                           in boudary 3 zone (A or B).*/
#define DSTEN    0x04u /*!< Flag to indicate if the distortion must be performed
                           or not (in case of charge of bootstrap capacitor phase
                           is not required).*/


/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */



/* Exported types ------------------------------------------------------- */

/**
  * @brief Current feedback component parameters structure definition for 1 Shunt configurations. Common to G0XX and G4XX MCUs.
  */
typedef const struct
{
  /* HW IP involved -----------------------------*/
  ADC_TypeDef *ADCx;                        /*!< ADC peripheral to be used. */
  TIM_TypeDef *TIMx;                        /*!< Timer used for PWM generation. */
  OPAMP_TypeDef *OPAMP_Selection;           /*!< Selected Opamp for 1 shunt configuration. */
  COMP_TypeDef *CompOCPSelection;           /*!< Internal comparator used for Phases protection. */
  COMP_TypeDef *CompOVPSelection;           /*!< Internal comparator used for Over Voltage protection. */
  GPIO_TypeDef *pwm_en_u_port;              /*!< Channel 1N (low side) GPIO output. */
  GPIO_TypeDef *pwm_en_v_port;              /*!< Channel 2N (low side) GPIO output. */
  GPIO_TypeDef *pwm_en_w_port;              /*!< Channel 3N (low side)  GPIO output. */
  DAC_TypeDef  *DAC_OCP_Selection;          /*!< DAC used for Over Current protection. */
  DAC_TypeDef  *DAC_OVP_Selection;          /*!< DAC used for Over Voltage protection. */
  uint32_t DAC_Channel_OCP;                 /*!< DAC channel used for Phase A current protection. */
  uint32_t DAC_Channel_OVP;                 /*!< DAC channel used for over voltage protection. */


 /* PWM generation parameters --------------------------------------------------*/

  uint16_t TMin;                            /* */
  uint16_t HTMin;                           /* */
  uint16_t CHTMin;                          /* */
  uint16_t Tbefore;                         /*!< Sampling time expressed in number of TIM clocks. */
  uint16_t Tafter;                          /*!< Sum of dead time plus max value between rise time and noise time
                                                 expressed in number of TIM clocks. */
  uint16_t TSample;                         /*!< Sampling time expressed in number of TIM clocks. */ 

  /* DAC settings --------------------------------------------------------------*/
  uint16_t DAC_OCP_Threshold;               /*!< Value of analog reference expressed as 16bit unsigned integer.
                                                 Ex. 0 = 0V 65536 = VDD_DAC. */
  uint16_t DAC_OVP_Threshold;               /*!< Value of analog reference expressed as 16bit unsigned integer.
                                                 Ex. 0 = 0V 65536 = VDD_DAC. */
  /* PWM Driving signals initialization ----------------------------------------*/
  uint8_t  IChannel;

  uint8_t  RepetitionCounter;               /*!< It expresses the number of PWM periods to be elapsed before compare
                                                 registers are updated again. In particular:
                                                 RepetitionCounter= (2* #PWM periods)-1*/
 
  /* Internal COMP settings ----------------------------------------------------*/
  uint8_t       CompOCPInvInput_MODE;       /*!< COMPx inverting input mode. It must be either equal to EXT_MODE or
                                                 INT_MODE. */
  uint8_t       CompOVPInvInput_MODE;       /*!< COMPx inverting input mode. It must be either equal to EXT_MODE or
                                                 INT_MODE. */
  
  /* Dual MC parameters --------------------------------------------------------*/
  uint8_t  FreqRatio;                       /*!< It is used in case of dual MC to
                                                 effect only on the second instanced object and must be equal to the
                                                 ratio between the two PWM frequencies (higher/lower).
                                                 Supported values are 1, 2 or 3. */
  uint8_t  IsHigherFreqTim;                 /*!< When bFreqRatio is greather than 1
                                                 this instance is the one with the highest frequency.
                                                 Allowed value are: HIGHER_FREQ or LOWER_FREQ */
} R1_Params_t;

/**
  * @brief  This structure is used to handle an instance of the
  *         Current feedback component for 1 Shunt configurations. Common to G0XX and G4XX MCUs.
  */
typedef struct
{
  PWMC_Handle_t _Super;                     /*!< Base component handler. */
  DMA_Channel_TypeDef * DistortionDMAy_Chx; /*!< DMA resource used for doing the distortion. */
  uint32_t ADCConfig;                       /*!< Values of JSQR register for ADC. */
  uint32_t PhaseOffset;                     /*!< Offset of Phase A current sensing network. */
  uint16_t Half_PWMPeriod;                  /*!< Half PWM Period in timer clock counts. */
  uint16_t DmaBuff[2]; 
  uint16_t CntSmp1; 
  uint16_t CntSmp2;
  uint16_t CurrAOld;
  uint16_t CurrBOld;
  uint8_t bIChannel;                        /*!< ADC channel used for current conversion. */
  uint8_t  PolarizationCounter;             /*!< Number of conversions performed during the calibration phase. */
  uint8_t sampCur1;
  uint8_t sampCur2;
  uint8_t Inverted_pwm_new;                 /* */
  uint8_t Flags; /* */
  bool UpdateFlagBuffer;                    /*!< Buffered version of Timer update IT flag. */
  /* Trigger selection for ADC peripheral.*/
  bool ADCRegularLocked;                    /*!< This flag is set when regular conversions are locked. */
  R1_Params_t * pParams_str;
} PWMC_R1_Handle_t;                     


/** @addtogroup R1_G4XX_pwm_curr_fdbk
  * @{
  */

/*
  * Initializes TIMx, ADC, GPIO, DMA1 and NVIC for current reading
  * in single shunt configuration using STM3G4XX
  */
void R1_Init(PWMC_R1_Handle_t *pHandle);

/*
  * Stores into the handle the voltage present on Ia and
  * Ib current feedback analog channels when no current is flowin into the
  * motor
  */
void R1_CurrentReadingPolarization(PWMC_Handle_t *pHdl);

/*
  * Computes and return latest converted motor phase currents.
  */
void R1_GetPhaseCurrents(PWMC_Handle_t *pHdl, ab_t *Iab);

/*
  * Turns on low sides switches
  */
void R1_TurnOnLowSides(PWMC_Handle_t *pHdl);

/*
  * Enables PWM generation on the proper Timer peripheral acting on MOE bit
  */
void R1_SwitchOnPWM(PWMC_Handle_t *pHdl);

/*
  * Disables PWM generation on the proper Timer peripheral acting on MOE bit
  */
void R1_SwitchOffPWM(PWMC_Handle_t *pHdl);

/*
  * Configures the ADC for the current sampling
  */
uint16_t R1_CalcDutyCycles(PWMC_Handle_t *pHdl);


/*
  * Contains the TIMx Update event interrupt
  */
void *R1_TIMx_UP_IRQHandler(PWMC_R1_Handle_t *pHdl);

/*
  * Contains the TIMx Break2 event interrupt
  */
void *R1_BRK2_IRQHandler(PWMC_R1_Handle_t *pHdl);

/*
  * Contains the TIMx Break1 event interrupt
  */
void *R1_BRK_IRQHandler(PWMC_R1_Handle_t *pHdl);


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

#endif /*R1_G4XX_PWMNCURRFDBK_H*/

/************************ (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
