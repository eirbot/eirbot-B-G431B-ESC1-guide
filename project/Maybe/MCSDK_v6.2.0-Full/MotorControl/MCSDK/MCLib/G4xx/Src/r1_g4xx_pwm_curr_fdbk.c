/**
  ******************************************************************************
  * @file    r1_g4xx_pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement current sensor
  *          class to be stantiated when the single shunt current sensing
  *          topology is used.
  * 
  *          It is specifically designed for STM32G4XX microcontrollers and
  *          implements the successive sampling of motor current using only one ADC.
  *           + initializes MCU peripheral for 1 shunt topology and G4 family
  *           + performs PWM duty cycle computation and generation
  *           + performs current sensing
  *
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
  * @ingroup R1_G4XX_pwm_curr_fdbk
  */

/* Includes ------------------------------------------------------------------*/
#include "r1_g4xx_pwm_curr_fdbk.h"
#include "pwm_common.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/**
 * @defgroup R1_G4XX_pwm_curr_fdbk G4 R1 1 ADC PWM & Current Feedback
 *
 * @brief STM32G4, 1-Shunt, 1 ADC, PWM & Current Feedback implementation
 *
 * This component is used in applications based on an STM32G4 MCU
 * and using a single shunt resistor current sensing topology.
 *
 * @{
 */

/* Constant values -----------------------------------------------------------*/
#define TIMxCCER_MASK_CH123 (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E |\
                             TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE)

/* boundary zone definition */
#define REGULAR             ((uint8_t)0u)
#define BOUNDARY_1          ((uint8_t)1u)  /* Two small, one big */
#define BOUNDARY_2          ((uint8_t)2u)  /* Two big, one small */
#define BOUNDARY_3          ((uint8_t)3u)  /* Three equal        */

#define INVERT_NONE         0u
#define INVERT_A            1u
#define INVERT_B            2u
#define INVERT_C            3u

#define SAMP_NO             0u
#define SAMP_IA             1u
#define SAMP_IB             2u
#define SAMP_IC             3u
#define SAMP_NIA            4u
#define SAMP_NIB            5u
#define SAMP_NIC            6u
#define SAMP_OLDA           7u
#define SAMP_OLDB           8u
#define SAMP_OLDC           9u

static const uint8_t REGULAR_SAMP_CUR1[6] = {SAMP_NIC, SAMP_NIC, SAMP_NIA, SAMP_NIA, SAMP_NIB, SAMP_NIB};
static const uint8_t REGULAR_SAMP_CUR2[6] = {SAMP_IA, SAMP_IB, SAMP_IB, SAMP_IC, SAMP_IC, SAMP_IA};
static const uint8_t BOUNDR1_SAMP_CUR2[6] = {SAMP_IB, SAMP_IB, SAMP_IC, SAMP_IC, SAMP_IA, SAMP_IA};
static const uint8_t BOUNDR2_SAMP_CUR1[6] = {SAMP_IA, SAMP_IB, SAMP_IB, SAMP_IC, SAMP_IC, SAMP_IA};
static const uint8_t BOUNDR2_SAMP_CUR2[6] = {SAMP_IC, SAMP_IA, SAMP_IA, SAMP_IB, SAMP_IB, SAMP_IC};

/* Private typedef -----------------------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/
void R1_HFCurrentsPolarization(PWMC_Handle_t *pHdl, ab_t *pStator_Currents);
void R1_SetAOReferenceVoltage(uint32_t DAC_Channel, DAC_TypeDef *DACx, uint16_t hDACVref);
void R1_1ShuntMotorVarsInit(PWMC_Handle_t *pHdl);
void R1_TIMxInit(TIM_TypeDef *TIMx, PWMC_R1_Handle_t *pHdl);
uint16_t R1_SetADCSampPointPolarization(PWMC_Handle_t *pHdl);

/**
  * @brief  Initializes TIMx, ADC, GPIO, DMA1 and NVIC for current reading
  *         in three shunt topology using STM32F30X and shared ADC.
  * 
  * @param  pHandle: Handler of the current instance of the PWM component.
  */
__weak void R1_Init(PWMC_R1_Handle_t *pHandle)
{
  OPAMP_TypeDef *OPAMPx = pHandle->pParams_str->OPAMP_Selection;
  COMP_TypeDef *COMP_OCPx = pHandle->pParams_str->CompOCPSelection;
  COMP_TypeDef *COMP_OVPx = pHandle->pParams_str->CompOVPSelection;
  DAC_TypeDef *DAC_OCPx = pHandle->pParams_str->DAC_OCP_Selection;
  DAC_TypeDef *DAC_OVP = pHandle->pParams_str->DAC_OVP_Selection;
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef *ADCx = pHandle->pParams_str->ADCx;

  R1_1ShuntMotorVarsInit(&pHandle->_Super);

  /* Disable IT and flags in case of LL driver usage
   * workaround for unwanted interrupt enabling done by LL driver */
  LL_ADC_DisableIT_EOC(ADCx);
  LL_ADC_ClearFlag_EOC(ADCx);
  LL_ADC_DisableIT_JEOC(ADCx);
  LL_ADC_ClearFlag_JEOC(ADCx);


  /* Enable TIM1 - TIM8 clock */
  if (TIM1 == TIMx)
  {
    /* DMA Event related to TIM1 Channel 4 */
    /* DMA1 Channel4 configuration ----------------------------------------------*/
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)pHandle->DmaBuff);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t) & (TIM1->CCR1));
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 2);

    /* Enable DMA1 Channel4 */
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
    pHandle->DistortionDMAy_Chx = DMA1_Channel1;

  }
#if (defined(TIM8) && defined(DMA2))
  else
  {
    /* DMA Event related to TIM8 Channel 4 */
    /* DMA2 Channel2 configuration ----------------------------------------------*/
    LL_DMA_SetMemoryAddress(DMA2, LL_DMA_CHANNEL_1, (uint32_t)pHandle->DmaBuff);
    LL_DMA_SetPeriphAddress(DMA2, LL_DMA_CHANNEL_1, (uint32_t)&TIMx->CCR1);
    LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_1, 2);
    /* Enable DMA2 Channel2 */
    LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_1);
    pHandle->DistortionDMAy_Chx = DMA2_Channel1;

  }
#else
  else
  {
    /* Nothing to do */
  }
#endif

  R1_TIMxInit(TIMx, pHandle);

  if (OPAMPx)
  {
    /* Enable OPAMP */
    LL_OPAMP_Enable(OPAMPx);
    LL_OPAMP_Lock(OPAMPx);
  }
  else
  {
    /* Nothing to do */
  }

  /* Over current protection */
  if (COMP_OCPx)
  {
    /* Inverting input*/
    if (pHandle->pParams_str->CompOCPInvInput_MODE != EXT_MODE)
    {
      R1_SetAOReferenceVoltage(pHandle->pParams_str->DAC_Channel_OCP, DAC_OCPx, (uint16_t)(pHandle->pParams_str->DAC_OCP_Threshold));
    }
    else
    {
      /* Nothing to do */
    }

    /* Enable comparator */
#if defined(COMP_CSR_COMPxHYST)
    LL_COMP_SetInputHysteresis(COMP_OCPx, LL_COMP_HYSTERESIS_LOW);
#endif
    LL_COMP_Enable(COMP_OCPx);
    LL_COMP_Lock(COMP_OCPx);
  }
  else
  {
    /* Nothing to do */
  }

  /* Over voltage protection */
  if (COMP_OVPx)
  {
    /* Inverting input*/
    if (pHandle->pParams_str->CompOCPInvInput_MODE != EXT_MODE)
    {
      R1_SetAOReferenceVoltage(pHandle->pParams_str->DAC_Channel_OVP, DAC_OVP, (uint16_t)(pHandle->pParams_str->DAC_OVP_Threshold));
    }
    /* Enable comparator */
#if defined(COMP_CSR_COMPxHYST)
    LL_COMP_SetInputHysteresis(COMP_OCPx, LL_COMP_HYSTERESIS_LOW);
#endif
    LL_COMP_Enable(COMP_OVPx);
    LL_COMP_Lock(COMP_OVPx);
  }
  else
  {
    /* Nothing to do */
  }

  if (TIM1 == pHandle->pParams_str->TIMx)
  {
    /* TIM1 Counter Clock stopped when the core is halted */
    LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_TIM1_STOP);
  }
#if defined(TIM8)
  else
  {
    /* TIM8 Counter Clock stopped when the core is halted */
    LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_TIM8_STOP);
  }
#else
  else
  {
    /* Nothing to do */
  }
#endif
    /* - Exit from deep-power-down mode */
  LL_ADC_DisableDeepPowerDown(ADCx);
 
  if (0U == LL_ADC_IsInternalRegulatorEnabled(ADCx))
  {
    /* Enable ADC internal voltage regulator */
    LL_ADC_EnableInternalRegulator(ADCx);
  
    /* Wait for Regulator Startup time, once for both */
    /* Note: Variable divided by 2 to compensate partially              */
    /*       CPU processing cycles, scaling in us split to not          */
    /*       exceed 32 bits register capacity and handle low frequency */
    volatile uint32_t wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US / 10UL) * (SystemCoreClock / (100000UL * 2UL)));
    while(wait_loop_index != 0UL)
    {
      wait_loop_index--;
    }
  }
  else
  {
    /* Nothing to do */
  }

  LL_ADC_StartCalibration(ADCx, LL_ADC_SINGLE_ENDED);
  while (1U == LL_ADC_IsCalibrationOnGoing(ADCx))
  {
    /* Nothing to do */
  }
  /* ADC Enable (must be done after calibration) */
  /* ADC5-140924: Enabling the ADC by setting ADEN bit soon after polling ADCAL=0 
  * following a calibration phase, could have no effect on ADC 
  * within certain AHB/ADC clock ratio
  */
  while (0U == LL_ADC_IsActiveFlag_ADRDY(ADCx))
  { 
    LL_ADC_Enable(ADCx);
  }

  /* Flushing JSQR queue of context by setting JADSTP = 1 (JQM)=1 */
  LL_ADC_INJ_StopConversion(ADCx);

  if (TIM1 == pHandle->pParams_str->TIMx)
  {
    LL_ADC_INJ_ConfigQueueContext(ADCx,
                                  LL_ADC_INJ_TRIG_EXT_TIM1_TRGO2,
                                  LL_ADC_INJ_TRIG_EXT_RISING,
                                  LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS,
                                  __LL_ADC_DECIMAL_NB_TO_CHANNEL(pHandle->pParams_str->IChannel),
                                  __LL_ADC_DECIMAL_NB_TO_CHANNEL(pHandle->pParams_str->IChannel),
                                  __LL_ADC_DECIMAL_NB_TO_CHANNEL(pHandle->pParams_str->IChannel),
                                  __LL_ADC_DECIMAL_NB_TO_CHANNEL(pHandle->pParams_str->IChannel));
  }
#if defined(TIM8)
  else
  {
    LL_ADC_INJ_ConfigQueueContext(ADCx,
                                  LL_ADC_INJ_TRIG_EXT_TIM8_TRGO2,
                                  LL_ADC_INJ_TRIG_EXT_RISING,
                                  LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS,
                                  __LL_ADC_DECIMAL_NB_TO_CHANNEL(pHandle->pParams_str->IChannel),
                                  __LL_ADC_DECIMAL_NB_TO_CHANNEL(pHandle->pParams_str->IChannel),
                                  __LL_ADC_DECIMAL_NB_TO_CHANNEL(pHandle->pParams_str->IChannel),
                                  __LL_ADC_DECIMAL_NB_TO_CHANNEL(pHandle->pParams_str->IChannel));
  }
#else
  else
  {
    /* Nothing to do */
  }
#endif


  /* Store register value in the handle to be used later during SVPWM for re-init */
  pHandle->ADCConfig = ADCx->JSQR;

  LL_ADC_INJ_StopConversion(ADCx);
  
  /* Enable injected end of sequence conversions interrupt */
  LL_ADC_EnableIT_JEOS(ADCx);
  
  if (pHandle->pParams_str->RepetitionCounter > 1)
  {
    /* Enable DMA distortion window insertion interrupt */
    if (TIM1 == TIMx)
    {
      LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
    }
    else
    {
      LL_DMA_EnableIT_TC(DMA2, LL_DMA_CHANNEL_1);
    }
  }
  else
  {
    /* Nothing to do */
  }

  LL_ADC_INJ_SetQueueMode(ADCx, LL_ADC_INJ_QUEUE_2CONTEXTS_END_EMPTY);
  LL_ADC_INJ_SetSequencerDiscont(ADCx, LL_ADC_INJ_SEQ_DISCONT_1RANK);

  /* Clear the flags */
  pHandle->_Super.DTTest = 0u;

}

/**
  * @brief Initializes @p TIMx peripheral with @p pHandle handler for PWM generation.
  * 
  */
__weak void R1_TIMxInit(TIM_TypeDef *TIMx, PWMC_R1_Handle_t *pHandle)
{

  /* Disable main TIM counter to ensure
   * a synchronous start by TIM2 trigger */
  LL_TIM_DisableCounter(TIMx);

  /* Enables the TIMx Preload on CC1 Register */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  /* Enables the TIMx Preload on CC2 Register */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  /* Enables the TIMx Preload on CC3 Register */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  /* Enables the TIMx Preload on CC4 Register */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH4);
  /* Enables the TIMx Preload on CC5 Register */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH5);
  /* Enables the TIMx Preload on CC6 Register */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH6);

  /* Always enable BKIN for safety feature */
  LL_TIM_ClearFlag_BRK(TIMx);
  LL_TIM_ClearFlag_BRK2(TIMx);
  LL_TIM_EnableIT_BRK(TIMx);

  /* Prepare timer for synchronization */
  LL_TIM_GenerateEvent_UPDATE(TIMx);
  if (2U == pHandle->pParams_str->FreqRatio)
  {
    if (HIGHER_FREQ == pHandle->pParams_str->IsHigherFreqTim)
    {
      if (3U == pHandle->pParams_str->RepetitionCounter)
      {
        /* Set TIMx repetition counter to 1 */
        LL_TIM_SetRepetitionCounter(TIMx, 1);
        LL_TIM_GenerateEvent_UPDATE(TIMx);
        /* Repetition counter will be set to 3 at next Update */
        LL_TIM_SetRepetitionCounter(TIMx, 3);
      }
      else
      {
        /* Nothing to do */
      }
    }
    else
    {
      /* Nothing to do */
    }
    LL_TIM_SetCounter(TIMx, (uint32_t)(pHandle->Half_PWMPeriod) - 1u);
  }
  else /* bFreqRatio equal to 1 or 3 */
  {
    if (M1 == pHandle->_Super.Motor)
    {
      if (1U ==  pHandle->pParams_str->RepetitionCounter)
      {
        LL_TIM_SetCounter(TIMx, (uint32_t)(pHandle->Half_PWMPeriod) - 1u);
      }
      else if (3U == pHandle->pParams_str->RepetitionCounter)
      {
        /* Set TIMx repetition counter to 1 */
        LL_TIM_SetRepetitionCounter(TIMx, 1);
        LL_TIM_GenerateEvent_UPDATE(TIMx);
        /* Repetition counter will be set to 3 at next Update */
        LL_TIM_SetRepetitionCounter(TIMx, 3);
      }
      else
      {
        /* Nothing to do */
      }
    }
    else
    {
      /* Nothing to do */
    }
  }

  /* Enable PWM channel */
  LL_TIM_CC_EnableChannel(TIMx, TIMxCCER_MASK_CH123);

}

/**
  * @brief  First initialization of the @p pHdl handler.
  * 
  */
__weak void R1_1ShuntMotorVarsInit(PWMC_Handle_t *pHdl)
{
  PWMC_R1_Handle_t *pHandle = (PWMC_R1_Handle_t *)pHdl;

  /* Init motor vars */
  pHandle->Inverted_pwm_new = INVERT_NONE;
  pHandle->Flags &= (~STBD3); /* STBD3 cleared */

  /* After reset, value of DMA buffers for distortion */
  pHandle->DmaBuff[0] =  pHandle->Half_PWMPeriod + 1u;
  pHandle->DmaBuff[1] =  pHandle->Half_PWMPeriod >> 1; /* Dummy */

  /* Default value of sampling points */
  pHandle->CntSmp1 = (pHandle->Half_PWMPeriod >> 1) + (pHandle->pParams_str->Tafter);
  pHandle->CntSmp2 = pHandle->Half_PWMPeriod - 1u;
  
  /* Set the default previous value of Phase A,B,C current */
  pHandle->CurrAOld = 0;
  pHandle->CurrBOld = 0;

  pHandle->_Super.BrakeActionLock = false;

}

/**
  * @brief  Stores into the @p pHdl the voltage present on Ia and
  *         Ib current feedback analog channels when no current is flowing into the
  *         motor.
  * 
  */
__weak void R1_CurrentReadingPolarization(PWMC_Handle_t *pHdl)
{

  PWMC_R1_Handle_t * pHandle = (PWMC_R1_Handle_t *)pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  pHandle->PhaseOffset = 0u;
  pHandle->PolarizationCounter = 0u;


  /* It forces inactive level on TIMx CHy and CHyN */
  TIMx->CCER &= (~TIMxCCER_MASK_CH123);

  /* Offset Polarization */
  /* Change function to be executed in ADCx_ISR */
  pHandle->_Super.pFctGetPhaseCurrents = &R1_HFCurrentsPolarization;
  
  pHandle->_Super.pFctSetADCSampPointSectX = &R1_SetADCSampPointPolarization;
      
  R1_SwitchOnPWM(&pHandle->_Super);

  /* Wait for NB_CONVERSIONS to be executed */
  waitForPolarizationEnd(TIMx,
                         &pHandle->_Super.SWerror,
                         pHandle->pParams_str->RepetitionCounter,
                         &pHandle->PolarizationCounter);

  R1_SwitchOffPWM(&pHandle->_Super);

  pHandle->PhaseOffset >>= 4;

  /* Change back function to be executed in ADCx_ISR */
  pHandle->_Super.pFctGetPhaseCurrents = &R1_GetPhaseCurrents;
  pHandle->_Super.pFctSetADCSampPointSectX = &R1_CalcDutyCycles;
  
  /* It re-enable drive of TIMx CHy and CHyN by TIMx CHyRef */
  TIMx->CCER |= TIMxCCER_MASK_CH123;

  R1_1ShuntMotorVarsInit(&pHandle->_Super);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif

/**
  * @brief  Computes and stores in @p pHdl handler the latest converted motor phase currents in @p pStator_Currents ab_t format.
  *
  */
__weak void R1_GetPhaseCurrents(PWMC_Handle_t *pHdl, ab_t *pStator_Currents)
{
  int32_t wAux;
  int16_t hCurrA = 0;
  int16_t hCurrB = 0;
  int16_t hCurrC = 0;
  uint8_t bCurrASamp = 0u;
  uint8_t bCurrBSamp = 0u;
  uint8_t bCurrCSamp = 0u;

  PWMC_R1_Handle_t *pHandle = (PWMC_R1_Handle_t *)pHdl;
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx = pHandle->pParams_str->ADCx;

  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH3);

  /* Reset the buffered version of update flag to indicate the start of FOC algorithm */
  pHandle->UpdateFlagBuffer = false;

  /* First sampling point */
  wAux = (int32_t)(ADCx->JDR1);

  wAux -= (int32_t)(pHandle->PhaseOffset);

  /* Check saturation */
  if (wAux > -INT16_MAX)
  {
    if (wAux < INT16_MAX)
    {
      /* Nothing to do */
    }
    else
    {
      wAux = INT16_MAX;
    }
  }
  else
  {
    wAux = -INT16_MAX;
  }

  switch (pHandle->sampCur1)
  {
    case SAMP_IA:
    {
      hCurrA = (int16_t)(wAux);
      bCurrASamp = 1u;
      break;
    }
    case SAMP_IB:
    {
      hCurrB = (int16_t)(wAux);
      bCurrBSamp = 1u;
      break;
    }
    case SAMP_IC:
    {
      hCurrC = (int16_t)(wAux);
      bCurrCSamp = 1u;
      break;
    }
    case SAMP_NIA:
    {
      wAux = -wAux;
      hCurrA = (int16_t)(wAux);
      bCurrASamp = 1u;
      break;
    }
    case SAMP_NIB:
    {
      wAux = -wAux;
      hCurrB = (int16_t)(wAux);
      bCurrBSamp = 1u;
      break;
    }
    case SAMP_NIC:
    {
      wAux = -wAux;
      hCurrC = (int16_t)(wAux);
      bCurrCSamp = 1u;
      break;
    }
    case SAMP_OLDA:
    {
      hCurrA = pHandle->CurrAOld;
      bCurrASamp = 1u;
      break;
    }
    case SAMP_OLDB:
    {
      hCurrB = pHandle->CurrBOld;
      bCurrBSamp = 1u;
      break;
    }
    default:
      break;
  }

  /* Second sampling point */
  wAux = (int32_t)(ADCx->JDR2);

  wAux -= (int32_t)(pHandle->PhaseOffset);

  /* Check saturation */
  if (wAux > -INT16_MAX)
  {
    if (wAux < INT16_MAX)
    {
      /* Nothing to do */
    }
    else
    {
      wAux = INT16_MAX;
    }
  }
  else
  {
    wAux = -INT16_MAX;
  }

  switch (pHandle->sampCur2)
  {
    case SAMP_IA:
    {
      hCurrA = (int16_t)(wAux);
      bCurrASamp = 1u;
      break;
    }
    case SAMP_IB:
    {
      hCurrB = (int16_t)(wAux);
      bCurrBSamp = 1u;
      break;
    }
    case SAMP_IC:
    {
      hCurrC = (int16_t)(wAux);
      bCurrCSamp = 1u;
      break;
    }
    case SAMP_NIA:
    {
      wAux = -wAux;
      hCurrA = (int16_t)(wAux);
      bCurrASamp = 1u;
      break;
    }
    case SAMP_NIB:
    {
      wAux = -wAux;
      hCurrB = (int16_t)(wAux);
      bCurrBSamp = 1u;
      break;
    }
    case SAMP_NIC:
    {
      wAux = -wAux;
      hCurrC = (int16_t)(wAux);
      bCurrCSamp = 1u;
      break;
    }
    default:
      break;
  }

  /* Computation of the third value */
  if (0U == bCurrASamp)
  {
    wAux = -((int32_t)(hCurrB)) - ((int32_t)(hCurrC));

    /* Check saturation */
    if (wAux > -INT16_MAX)
    {
      if (wAux < INT16_MAX)
      {
        /* Nothing to do */
      }
      else
      {
        wAux = INT16_MAX;
      }
    }
    else
    {
      wAux = -INT16_MAX;
    }

    hCurrA = (int16_t)wAux;
  }
  if (bCurrBSamp == 0u)
  {
    wAux = -((int32_t)(hCurrA)) - ((int32_t)(hCurrC));

    /* Check saturation */
    if (wAux > -INT16_MAX)
    {
      if (wAux < INT16_MAX)
      {
        /* Nothing to do */
      }
      else
      {
        wAux = INT16_MAX;
      }
    }
    else
    {
      wAux = -INT16_MAX;
    }

    hCurrB = (int16_t)wAux;
  }
  if (bCurrCSamp == 0u)
  {
    wAux = -((int32_t)(hCurrA)) - ((int32_t)(hCurrB));

    /* Check saturation */
    if (wAux > -INT16_MAX)
    {
      if (wAux < INT16_MAX)
      {
        /* Nothing to do */
      }
      else
      {
        wAux = INT16_MAX;
      }
    }
    else
    {
      wAux = -INT16_MAX;
    }

    hCurrC = (int16_t)wAux;
  }

  /* hCurrA, hCurrB, hCurrC values are the sampled values */
  pHandle->CurrAOld = hCurrA;
  pHandle->CurrBOld = hCurrB;

  pStator_Currents->a = hCurrA;
  pStator_Currents->b = hCurrB;

  pHandle->_Super.Ia = hCurrA;
  pHandle->_Super.Ib = hCurrB;
  pHandle->_Super.Ic = -hCurrA - hCurrB;
}

/**
  * @brief  Implementation of PWMC_GetPhaseCurrents to be performed during polarization.
  * 
  * It sums up injected conversion data into PhaseOffset to compute the
  * offset introduced in the current feedback network. It is required 
  * to properly configure ADC inputs before enabling the offset computation.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @param  pStator_Currents: Pointer to the structure that will receive motor current
  *         of phase A and B in ab_t format.
  */
__weak void R1_HFCurrentsPolarization(PWMC_Handle_t *pHdl, ab_t *pStator_Currents)
{
  /* Derived class members container */
  PWMC_R1_Handle_t *pHandle = (PWMC_R1_Handle_t *)pHdl;
  ADC_TypeDef *ADCx = pHandle->pParams_str->ADCx;

  /* Reset the buffered version of update flag to indicate the start of FOC algorithm */
  pHandle->UpdateFlagBuffer = false;

  if (pHandle->PolarizationCounter < NB_CONVERSIONS)
  {
    pHandle->PhaseOffset += ADCx->JDR2;
    pHandle->PolarizationCounter++;
  }
  else
  {
    /* Nothing to do */
  }

  /* During offset Polarization no current is flowing in the phases */
  pStator_Currents->a = 0;
  pStator_Currents->b = 0;

}

/**
  * @brief  Configures the ADC for the current sampling during calibration.
  * 
  * It sets the ADC sequence length and channels, and the sampling point via TIMx_Ch4 value and polarity.
  * It then calls the WriteTIMRegisters method.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @retval uint16_t Return value of R3_1_WriteTIMRegisters.
  */
uint16_t R1_SetADCSampPointPolarization(PWMC_Handle_t * pHdl)
{
  PWMC_R1_Handle_t *pHandle = (PWMC_R1_Handle_t *)pHdl;
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;
  uint16_t hAux;
    
  LL_TIM_OC_SetCompareCH5(TIMx, ((uint32_t)(pHandle->Half_PWMPeriod) >> 1) + (uint32_t)(pHandle->pParams_str->Tafter));
  LL_TIM_OC_SetCompareCH6(TIMx, ((uint32_t)(pHandle->Half_PWMPeriod) - 1u));
  
  if (LL_TIM_IsActiveFlag_UPDATE(TIMx))
  {
    hAux = MC_DURATION;
  }
  else
  {
    hAux = MC_NO_ERROR;
  }
  
  return (hAux);
}

/**
  * @brief  Turns on low sides switches.
  * 
  * This function is intended to be used for charging boot capacitors of driving section. It has to be
  * called on each motor start-up when using high voltage drivers.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
__weak void R1_TurnOnLowSides(PWMC_Handle_t *pHdl)
{
  PWMC_R1_Handle_t *pHandle = (PWMC_R1_Handle_t *)pHdl;
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.TurnOnLowSidesAction = true;
  /* Turn on the three low side switches */
  LL_TIM_OC_SetCompareCH1(TIMx, 0);
  LL_TIM_OC_SetCompareCH2(TIMx, 0);
  LL_TIM_OC_SetCompareCH3(TIMx, 0);

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(TIMx);

  /* Wait until next update */
  while (RESET == LL_TIM_IsActiveFlag_UPDATE(TIMx))
  {
    /* Nothing to do */
  }

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs(TIMx);

  if ((pHandle->_Super.LowSideOutputs) == ES_GPIO)
  {
    LL_GPIO_SetOutputPin(pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin);
    LL_GPIO_SetOutputPin(pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin);
    LL_GPIO_SetOutputPin(pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin);
  }
  else
  {
    /* Nothing to do */
  }
}


/**
  * @brief  Enables PWM generation on the proper Timer peripheral acting on MOE bit.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
__weak void R1_SwitchOnPWM(PWMC_Handle_t *pHdl)
{
  PWMC_R1_Handle_t *pHandle = (PWMC_R1_Handle_t *)pHdl;
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;
  
  /* We forbid ADC usage for regular conversion on Systick */
  pHandle->ADCRegularLocked=true;
  
  /* Enables the TIMx Preload on CC1 Register */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  /* Enables the TIMx Preload on CC2 Register */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  /* Enables the TIMx Preload on CC3 Register */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH3);

  /* TIM output trigger 2 for ADC */
  LL_TIM_SetTriggerOutput2(TIMx, LL_TIM_TRGO2_OC5_RISING_OC6_RISING);

  pHandle->_Super.TurnOnLowSidesAction = false;
  /* Set all duty to 50% */
  /* Set ch5 ch6 for triggering */
  /* Clear Update Flag */
  /* TIM ch4 DMA request enable */

  pHandle->DmaBuff[1] = pHandle->Half_PWMPeriod >> 1;
  LL_TIM_OC_SetCompareCH1(TIMx, (uint32_t)(pHandle->Half_PWMPeriod >> 1));
  LL_TIM_OC_SetCompareCH2(TIMx, (uint32_t)(pHandle->Half_PWMPeriod >> 1));
  LL_TIM_OC_SetCompareCH3(TIMx, (uint32_t)(pHandle->Half_PWMPeriod >> 1));

  LL_TIM_OC_SetCompareCH5(TIMx, (((uint32_t)(pHandle->Half_PWMPeriod >> 1)) + (uint32_t)pHandle->pParams_str->Tafter));
  LL_TIM_OC_SetCompareCH6(TIMx, (uint32_t)(pHandle->Half_PWMPeriod - 1u));

  /* Wait for a new PWM period */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  while (RESET == LL_TIM_IsActiveFlag_UPDATE(TIMx))
  {
    /* Nothing to do */
  }
  LL_TIM_ClearFlag_UPDATE(TIMx);
  
  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs(TIMx);

  if ((pHandle->_Super.LowSideOutputs) == ES_GPIO)
  {
    if ((TIMx->CCER & TIMxCCER_MASK_CH123) != 0u)
    {
      LL_GPIO_SetOutputPin(pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin);
      LL_GPIO_SetOutputPin(pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin);
      LL_GPIO_SetOutputPin(pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin);
    }
    else
    {
      /* It is executed during Polarization phase the EN signal shall stay off */
      LL_GPIO_ResetOutputPin(pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin);
      LL_GPIO_ResetOutputPin(pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin);
      LL_GPIO_ResetOutputPin(pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin);
    }
  }
  else
  {
    /* Nothing to do */
  }
  
  /* Enable DMA request for distortion window insertion */
  LL_TIM_EnableDMAReq_CC4(TIMx);
  
  /* Enable TIMx update IRQ */
  LL_TIM_EnableIT_UPDATE(TIMx);

}


/**
  * @brief  Disables PWM generation on the proper Timer peripheral acting on MOE bit.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
__weak void R1_SwitchOffPWM(PWMC_Handle_t *pHdl)
{
  PWMC_R1_Handle_t *pHandle = (PWMC_R1_Handle_t *)pHdl;
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef *ADCx = pHandle->pParams_str->ADCx;

  /* Disable TIMx update IRQ */
  LL_TIM_DisableIT_UPDATE(TIMx);

  /* Flushing JSQR queue of context by setting JADSTP = 1 (JQM)=1 */
  LL_ADC_INJ_StopConversion(ADCx);
  
  pHandle->_Super.TurnOnLowSidesAction = false;
  
  /* Main PWM Output Disable */
  LL_TIM_DisableAllOutputs(TIMx);
  if (pHandle->_Super.BrakeActionLock == true)
  {
    /* Nothing to do */
  }
  else
  {
    if (ES_GPIO == (pHandle->_Super.LowSideOutputs))
    {
      LL_GPIO_ResetOutputPin(pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin);
      LL_GPIO_ResetOutputPin(pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin);
      LL_GPIO_ResetOutputPin(pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin);
    }
    else
    {
      /* Nothing to do */
    }
  }

  /* Wait for a new PWM period */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  while (RESET == LL_TIM_IsActiveFlag_UPDATE(TIMx))
  {
    /* Nothing to do */
  }
  LL_TIM_ClearFlag_UPDATE(TIMx);

  /* Clear injected end of sequence conversions flag */
  LL_ADC_ClearFlag_JEOS(ADCx);

  /* Disable TIMx DMA requests enable */
  LL_TIM_DisableDMAReq_CC4(TIMx);
  
 /* We allow ADC usage for regular conversion on Systick*/
  pHandle->ADCRegularLocked = false;

}

/**
  * @brief  Configures the analog output used for protection thresholds.
  *
  * @param  DAC_Channel: the selected DAC channel.
  *          This parameter can be:
  *            @arg DAC_Channel_1: DAC Channel1 selected.
  *            @arg DAC_Channel_2: DAC Channel2 selected.
  * @param  DACx: DAC to be configured.
  * @param  hDACVref: Value of DAC reference expressed as 16bit unsigned integer. \n
  *         Ex. 0 = 0V ; 65536 = VDD_DAC.
  */
__weak void R1_SetAOReferenceVoltage(uint32_t DAC_Channel, DAC_TypeDef *DACx, uint16_t hDACVref)
{
  LL_DAC_ConvertData12LeftAligned (DACx, DAC_Channel, hDACVref);

  /* Enable DAC Channel */
  LL_DAC_TrigSWConversion (DACx, DAC_Channel);

  if (LL_DAC_IsEnabled (DACx, DAC_Channel) == 1u)
  { /* If DAC is already enable, we wait LL_DAC_DELAY_VOLTAGE_SETTLING_US */
    uint32_t wait_loop_index = ((LL_DAC_DELAY_VOLTAGE_SETTLING_US) * (SystemCoreClock / (1000000UL * 2UL)));
    while(wait_loop_index != 0UL)
    {
      wait_loop_index--;
    }
  }
  else
  {
    /* If DAC is not enabled, we must wait LL_DAC_DELAY_STARTUP_VOLTAGE_SETTLING_US */
    LL_DAC_Enable(DACx, DAC_Channel);
    uint32_t wait_loop_index = ((LL_DAC_DELAY_STARTUP_VOLTAGE_SETTLING_US) * (SystemCoreClock / (1000000UL * 2UL)));
    while(wait_loop_index != 0UL)
    {
      wait_loop_index--;
    }
  }
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif

/**
  * @brief  Implementation of the single shunt algorithm to setup the
  *         TIM1 register and DMA buffers values for the next PWM period.
  * 
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @retval Returns #MC_NO_ERROR if no error occurred or #MC_DURATION if the duty cycles were
  *         set too late for being taken into account in the next PWM cycle.
  */
__weak uint16_t R1_CalcDutyCycles(PWMC_Handle_t *pHdl)
{
  PWMC_R1_Handle_t *pHandle = (PWMC_R1_Handle_t *)pHdl;
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;
  int16_t hDeltaDuty_0;
  int16_t hDeltaDuty_1;
  uint16_t hAux;
  register uint16_t lowDuty = pHandle->_Super.lowDuty;
  register uint16_t midDuty = pHandle->_Super.midDuty;
  register uint16_t highDuty = pHandle->_Super.highDuty;
  uint8_t bSector;
  uint8_t bStatorFluxPos;


  bSector = (uint8_t)(pHandle->_Super.Sector);

  /* Compute delta duty */
  hDeltaDuty_0 = (int16_t)(midDuty) - (int16_t)(highDuty);
  hDeltaDuty_1 = (int16_t)(lowDuty) - (int16_t)(midDuty);

  /* Check region */
  if ((uint16_t)hDeltaDuty_0 <= pHandle->pParams_str->TMin)
  {
    if ((uint16_t)hDeltaDuty_1 <= pHandle->pParams_str->TMin)
    {
      bStatorFluxPos = BOUNDARY_3;
    }
    else
    {
      bStatorFluxPos = BOUNDARY_2;
    }
  }
  else
  {
    if ((uint16_t)hDeltaDuty_1 > pHandle->pParams_str->TMin)
    {
      bStatorFluxPos = REGULAR;
    }
    else
    {
      bStatorFluxPos = BOUNDARY_1;
    }
  }
  if (REGULAR == bStatorFluxPos)
  {
    pHandle->Inverted_pwm_new = INVERT_NONE;
  }
  else if (BOUNDARY_1 == bStatorFluxPos) /* Adjust the lower */
    {
      switch (bSector)
      {
        case SECTOR_5:
        case SECTOR_6:
        {
          if ((pHandle->_Super.CntPhA - pHandle->pParams_str->CHTMin - highDuty) > pHandle->pParams_str->TMin)
          {
            pHandle->Inverted_pwm_new = INVERT_A;
            pHandle->_Super.CntPhA -= pHandle->pParams_str->CHTMin;
            if (pHandle->_Super.CntPhA < midDuty)
            {
              midDuty = pHandle->_Super.CntPhA;
            }
            else
            {
              /* Nothing to do */
            }
          }
          else
          {
            bStatorFluxPos = BOUNDARY_3;
            if (0U == (pHandle->Flags & STBD3))
            {
              pHandle->Inverted_pwm_new = INVERT_A;
              pHandle->_Super.CntPhA -= pHandle->pParams_str->CHTMin;
              pHandle->Flags |= STBD3;
            }
            else
            {
              pHandle->Inverted_pwm_new = INVERT_B;
              pHandle->_Super.CntPhB -= pHandle->pParams_str->CHTMin;
              pHandle->Flags &= (~STBD3);
            }
          }
          break;
        }

        case SECTOR_2:
        case SECTOR_1:
        {
          if ((pHandle->_Super.CntPhB - pHandle->pParams_str->CHTMin - highDuty) > pHandle->pParams_str->TMin)
          {
            pHandle->Inverted_pwm_new = INVERT_B;
            pHandle->_Super.CntPhB -= pHandle->pParams_str->CHTMin;
            if (pHandle->_Super.CntPhB < midDuty)
            {
              midDuty = pHandle->_Super.CntPhB;
            }
            else
            {
              /* Nothing to do */
            }
          }
          else
          {
            bStatorFluxPos = BOUNDARY_3;
            if (0U == (pHandle->Flags & STBD3))
            {
              pHandle->Inverted_pwm_new = INVERT_A;
              pHandle->_Super.CntPhA -= pHandle->pParams_str->CHTMin;
              pHandle->Flags |= STBD3;
            }
            else
            {
              pHandle->Inverted_pwm_new = INVERT_B;
              pHandle->_Super.CntPhB -= pHandle->pParams_str->CHTMin;
              pHandle->Flags &= (~STBD3);
            }
          }
          break;
        }

        case SECTOR_4:
        case SECTOR_3:
        {
          if ((pHandle->_Super.CntPhC - pHandle->pParams_str->CHTMin - highDuty) > pHandle->pParams_str->TMin)
          {
            pHandle->Inverted_pwm_new = INVERT_C;
            pHandle->_Super.CntPhC -= pHandle->pParams_str->CHTMin;
            if (pHandle->_Super.CntPhC < midDuty)
            {
              midDuty = pHandle->_Super.CntPhC;
            }
            else
            {
              /* Nothing to do */
            }
          }
          else
          {
            bStatorFluxPos = BOUNDARY_3;
            if (0U == (pHandle->Flags & STBD3))
            {
              pHandle->Inverted_pwm_new = INVERT_A;
              pHandle->_Super.CntPhA -= pHandle->pParams_str->CHTMin;
              pHandle->Flags |= STBD3;
            }
            else
            {
              pHandle->Inverted_pwm_new = INVERT_B;
              pHandle->_Super.CntPhB -= pHandle->pParams_str->CHTMin;
              pHandle->Flags &= (~STBD3);
            }
          }
          break;
        }

        default:
          break;
      }
    }
    else if (BOUNDARY_2 == bStatorFluxPos) /* Adjust the middler */
    {
      switch (bSector)
      {
        case SECTOR_4:
        case SECTOR_5: /* Invert B */
        {
          pHandle->Inverted_pwm_new = INVERT_B;
          pHandle->_Super.CntPhB -= pHandle->pParams_str->CHTMin;
          if (pHandle->_Super.CntPhB > 0xEFFFu)
          {
            pHandle->_Super.CntPhB = 0u;
          }
          else
          {
            /* Nothing to do */
          }
          break;
        }

        case SECTOR_2:
        case SECTOR_3: /* Invert A */
        {
          pHandle->Inverted_pwm_new = INVERT_A;
          pHandle->_Super.CntPhA -= pHandle->pParams_str->CHTMin;
          if (pHandle->_Super.CntPhA > 0xEFFFu)
          {
            pHandle->_Super.CntPhA = 0u;
          }
          else
          {
            /* Nothing to do */
          }
          break;
        }

        case SECTOR_6:
        case SECTOR_1: /* Invert C */
        {
          pHandle->Inverted_pwm_new = INVERT_C;
          pHandle->_Super.CntPhC -= pHandle->pParams_str->CHTMin;
          if (pHandle->_Super.CntPhC > 0xEFFFu)
          {
            pHandle->_Super.CntPhC = 0u;
          }
          else
          {
            /* Nothing to do */
          }
          break;
        }

        default:
          break;
      }
    }
    else if (bStatorFluxPos == BOUNDARY_3)
    {
      if (0U == (pHandle->Flags & STBD3))
      {
        pHandle->Inverted_pwm_new = INVERT_A;
        pHandle->_Super.CntPhA -= pHandle->pParams_str->CHTMin;
        pHandle->Flags |= STBD3;
      }
      else
      {
        pHandle->Inverted_pwm_new = INVERT_B;
        pHandle->_Super.CntPhB -= pHandle->pParams_str->CHTMin;
        pHandle->Flags &= (~STBD3);
      }
    }
    else
    {
      /* Nothing to do */
    }

    if (REGULAR == bStatorFluxPos) /* Regular zone */
    {
      /* First point */
      pHandle->CntSmp1 = midDuty - pHandle->pParams_str->Tbefore;

      /* Second point */
      pHandle->CntSmp2 = lowDuty - pHandle->pParams_str->Tbefore;
    }
    else
    {
      /* Nothing to do */
    }

    if (BOUNDARY_1 == bStatorFluxPos) /* Two small, one big */
    {
      /* First point */
      pHandle->CntSmp1 = midDuty - pHandle->pParams_str->Tbefore;

      /* Second point */
      pHandle->CntSmp2 = pHandle->Half_PWMPeriod - pHandle->pParams_str->HTMin + pHandle->pParams_str->TSample;
    }
    else
    {
      /* Nothing to do */
    }

    if (BOUNDARY_2 == bStatorFluxPos) /* Two big, one small */
    {
      /* First point */
      pHandle->CntSmp1 = lowDuty - pHandle->pParams_str->Tbefore;

      /* Second point */
      pHandle->CntSmp2 = pHandle->Half_PWMPeriod - pHandle->pParams_str->HTMin + pHandle->pParams_str->TSample;
    }
    else
    {
      /* Nothing to do */
    }

    if (BOUNDARY_3 == bStatorFluxPos)
    {
      /* First point */
      pHandle->CntSmp1 = highDuty - pHandle->pParams_str->Tbefore; /* Dummy trigger */
      /* Second point */
      pHandle->CntSmp2 = pHandle->Half_PWMPeriod - pHandle->pParams_str->HTMin + pHandle->pParams_str->TSample;
    }
    else
    {
      /* Nothing to do */
    }

  LL_TIM_OC_SetCompareCH5(TIMx, pHandle->CntSmp1);
  LL_TIM_OC_SetCompareCH6(TIMx, pHandle->CntSmp2);

  if (REGULAR == bStatorFluxPos)
  {
    LL_TIM_SetTriggerOutput2(TIMx, LL_TIM_TRGO2_OC5_RISING_OC6_RISING);
  }
  else
  {
    LL_TIM_SetTriggerOutput2(TIMx, LL_TIM_TRGO2_OC5_RISING_OC6_FALLING);
  }

  /* Update Timer Ch 1,2,3 (These value are required before update event) */
  LL_TIM_OC_SetCompareCH1(TIMx, pHandle->_Super.CntPhA);
  LL_TIM_OC_SetCompareCH2(TIMx, pHandle->_Super.CntPhB);
  LL_TIM_OC_SetCompareCH3(TIMx, pHandle->_Super.CntPhC);
  
  /* End of FOC */
  /* Check software error */
  if (true == pHandle->UpdateFlagBuffer)
  {
    hAux = MC_DURATION;
  }
  else
  {
    hAux = MC_NO_ERROR;
  }
  if (pHandle->_Super.SWerror == 1u)
  {
    hAux = MC_DURATION;
    pHandle->_Super.SWerror = 0u;
  }

  /* Set the current sampled */
  if (REGULAR == bStatorFluxPos) /* Regual zone */
  {
    pHandle->sampCur1 = REGULAR_SAMP_CUR1[bSector];
    pHandle->sampCur2 = REGULAR_SAMP_CUR2[bSector];
  }
  else
  {
    /* Nothing to do */
  }

  if (BOUNDARY_1 == bStatorFluxPos) /* Two small, one big */
  {
    pHandle->sampCur1 = REGULAR_SAMP_CUR1[bSector];
    pHandle->sampCur2 = BOUNDR1_SAMP_CUR2[bSector];
  }
  else
  {
    /* Nothing to do */
  }

  if (BOUNDARY_2 == bStatorFluxPos) /* Two big, one small */
  {
    pHandle->sampCur1 = BOUNDR2_SAMP_CUR1[bSector];
    pHandle->sampCur2 = BOUNDR2_SAMP_CUR2[bSector];
  }
  else
  {
    /* Nothing to do */
  }

  if (BOUNDARY_3 == bStatorFluxPos)
  {
    if (pHandle->Inverted_pwm_new == INVERT_A)
    {
      pHandle->sampCur1 = SAMP_OLDB;
      pHandle->sampCur2 = SAMP_IA;
    }
    else
    {
      /* Nothing to do */
    }
    if (pHandle->Inverted_pwm_new == INVERT_B)
    {
      pHandle->sampCur1 = SAMP_OLDA;
      pHandle->sampCur2 = SAMP_IB;
    }
    else
    {
      /* Nothing to do */
    }
  }
  else
  {
    /* Nothing to do */
  }
  
  return (hAux);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif

/**
  * @brief  Contains the TIMx Update event interrupt.
  *
  * @param  pHandle: Handler of the current instance of the PWM component.
  */
__weak void * R1_TIMx_UP_IRQHandler(PWMC_R1_Handle_t *pHandle)
{
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef *ADCx = pHandle->pParams_str->ADCx;
  
  pHandle->UpdateFlagBuffer = true;
  
  switch (pHandle->Inverted_pwm_new)
  {
    case INVERT_A:
    {
      /* Active window insertion phase A:
         - Set duty cycle A value to be restored by DMA */
      pHandle->DmaBuff[1] = pHandle->_Super.CntPhA;
      /* - Set DMA dest. address to phase A capture/compare register */
      pHandle->DistortionDMAy_Chx->CPAR = (uint32_t) &TIMx->CCR1;
      /* - Disable phase preload register to take into account immediately the values written by DMA */
      LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH1);
      LL_TIM_EnableDMAReq_CC4(TIMx);
      break;
    }

    case INVERT_B:
    {
      /* Active window insertion phase B:
      - Set duty cycle B value to be restored by DMA */
      pHandle->DmaBuff[1] = pHandle->_Super.CntPhB;
      /* - Set DMA dest. address to phase B capture/compare register */
      pHandle->DistortionDMAy_Chx->CPAR = (uint32_t) &TIMx->CCR2;
      /* - Disable phase preload register to take into account immediately the values written by DMA */
      LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH2);
      LL_TIM_EnableDMAReq_CC4(TIMx);
      break;
    }

    case INVERT_C:
    {
      /* Active window insertion phase C:
         - Set duty cycle C value to be restored by DMA */
      pHandle->DmaBuff[1] = pHandle->_Super.CntPhC;
     /* - Set DMA dest. address to phase C capture/compare register */
      pHandle->DistortionDMAy_Chx->CPAR = (uint32_t) &TIMx->CCR3;
      /* - Disable phase preload register to take into account immediately the values written by DMA */
      LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH3);
      LL_TIM_EnableDMAReq_CC4(TIMx);
      break;
    }

    default:
    {
      /* Disable DMA requests done by TIMx as no active window insertion needed on current PWM cycle */
      LL_TIM_DisableDMAReq_CC4(TIMx);
      break;
    }
 
  /* Write ADC sequence */
  pHandle->pParams_str->ADCx->JSQR = pHandle->ADCConfig;
  /* Start injected conversion */
  LL_ADC_INJ_StartConversion(ADCx);
  
  return (&(pHandle->_Super.Motor));
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
