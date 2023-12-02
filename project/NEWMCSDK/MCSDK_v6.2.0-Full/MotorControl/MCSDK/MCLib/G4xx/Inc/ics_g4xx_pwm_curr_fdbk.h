/**
  ******************************************************************************
  * @file    ics_g4xx_pwm_curr_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          ICS_G4XX_pwm_curr_fdbk component of the Motor Control SDK.
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
  * @ingroup ICS_G4XX_pwm_curr_fdbk
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ICS_G4XX_PWMNCURRFDBK_H
#define ICS_G4XX_PWMNCURRFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "pwm_curr_fdbk.h"

/* Exported defines --------------------------------------------------------*/
#define NONE    ((uint8_t)(0x00))
#define EXT_MODE  ((uint8_t)(0x01))
#define INT_MODE  ((uint8_t)(0x02))

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @addtogroup ICS_pwm_curr_fdbk
  * @{
  */


/* Exported types ------------------------------------------------------- */

/*
  * @brief  ICS_G4XX_pwm_curr_fdbk component parameters definition.
  */
typedef const struct
{
  /* HW IP involved -----------------------------*/
  ADC_TypeDef *ADCx_1;             /* First ADC peripheral to be used.*/
  ADC_TypeDef *ADCx_2;             /* Second ADC peripheral to be used.*/
  TIM_TypeDef *TIMx;               /* Timer used for PWM generation.*/

  uint32_t ADCConfig1;      /*!< Value for ADC CR2 to properly configure
                                 current sampling during the context switching. 
                                 Either defined in PWMC_ICS_Handle_t or in 
                                 ICS_Params_t. Absent in F4XX. */
  uint32_t ADCConfig2;      /*!< Value for ADC CR2 to properly configure
                                 current sampling during the context switching. 
                                 Either defined in PWMC_ICS_Handle_t or in 
                                 ICS_Params_t. Absent in F4XX. */

  uint8_t  RepetitionCounter;            /* Expresses the number of PWM
                                            periods to be elapsed before compare
                                            registers are updated again. In
                                            particular:
                                            RepetitionCounter= (2* #PWM periods)-1*/

  /* Dual MC parameters --------------------------------------------------------*/
  uint8_t  FreqRatio;                /* Used in case of dual MC to
                                        synchronize TIM1 and TIM8. It has
                                        effect only on the second instanced
                                        object and must be equal to the
                                        ratio between the two PWM frequencies
                                        (higher/lower). Supported values are
                                        1, 2 or 3 */
  uint8_t  IsHigherFreqTim;          /* When FreqRatio is greater than 1
                                        this param is used to indicate if this
                                        instance is the one with the highest
                                        frequency. Allowed values are: HIGHER_FREQ
                                        or LOWER_FREQ */

} ICS_Params_t, *pICS_Params_t;

/*
  * This structure is used to handle an instance of the ICS_G4XX_pwm_curr_fdbk component.
  */
typedef struct
{
  PWMC_Handle_t _Super;     /* Base component handler. */
  uint32_t PhaseAOffset;    /* Offset of Phase A current sensing network. */
  uint32_t PhaseBOffset;    /* Offset of Phase B current sensing network. */
  uint16_t Half_PWMPeriod;  /* Half PWM Period in timer clock counts. */
  volatile uint8_t PolarizationCounter; /* Number of conversions performed during the calibration phase. */
  bool ADCRegularLocked; /*!< This flag is set when regular conversions are locked. Specific to G4XX. */     /* Cut 2.2 patch*/
  pICS_Params_t pParams_str;
} PWMC_ICS_Handle_t;

/* Exported functions ------------------------------------------------------- */

/*
  * Initializes TIMx, ADC, GPIO and NVIC for current reading
  * in ICS configuration using STM32G4XX.
  */
void ICS_Init(PWMC_ICS_Handle_t *pHandle);

/*
  * Stores into the handler the voltage present on Ia and Ib current
  * feedback analog channels when no current is flowing into the motor.
  */
void ICS_CurrentReadingPolarization(PWMC_Handle_t *pHdl);

/*
  * Computes and stores in the handler the latest converted motor phase currents in ab_t format.
  */
void ICS_GetPhaseCurrents(PWMC_Handle_t *pHdl, ab_t *Iab);

/*
  * Turns on low sides switches.
  */
void ICS_TurnOnLowSides(PWMC_Handle_t *pHdl, uint32_t ticks);

/*
  * Enables PWM generation on the proper Timer peripheral acting on MOE bit.
  */
void ICS_SwitchOnPWM(PWMC_Handle_t *pHdl);

/*
  * Disables PWM generation on the proper Timer peripheral acting on MOE bit.
  */
void ICS_SwitchOffPWM(PWMC_Handle_t *pHdl);

/*
  * Writes into peripheral registers the new duty cycle and sampling point.
  */
uint16_t ICS_WriteTIMRegisters(PWMC_Handle_t *pHdl);


/*
  * Contains the TIMx Update event interrupt.
  */
void *ICS_TIMx_UP_IRQHandler(PWMC_ICS_Handle_t *pHdl);

/*
  * Contains the TIMx Break2 event interrupt.
  */
void *ICS_BRK2_IRQHandler(PWMC_ICS_Handle_t *pHdl);

/*
  * Contains the TIMx Break1 event interrupt.
  */
void *ICS_BRK_IRQHandler(PWMC_ICS_Handle_t *pHdl);

/*
  * Stores in the handler the calibrated offsets.
  */
void ICS_SetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets);

/*
  * Reads the calibrated offsets stored in the handler.
  */
void ICS_GetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets);

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

#endif /*ICS_G4XX_PWMNCURRFDBK_H*/

/************************ (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
