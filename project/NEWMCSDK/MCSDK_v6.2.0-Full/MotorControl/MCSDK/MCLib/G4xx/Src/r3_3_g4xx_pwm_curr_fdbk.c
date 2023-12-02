/**
  ******************************************************************************
  * @file    r3_3_g4xx_pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement current sensor
  *          class to be stantiated when the three shunts current sensing
  *          topology is used.
  * 
  *          It is specifically designed for STM32G4XX microcontrollers and
  *          implements the successive sampling of motor current using three ADCs.
  *           + MCU peripheral and handle initialization function
  *           + three shunts current sensing
  *           + space vector modulation function
  *           + ADC sampling function
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  * @ingroup R3_3_G4XX_pwm_curr_fdbk
  */

/* Includes ------------------------------------------------------------------*/
#include "r3_3_g4xx_pwm_curr_fdbk.h"
#include "pwm_common.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/* Private defines -----------------------------------------------------------*/
#define TIMxCCER_MASK_CH123          (LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH2N |\
                                      LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N)

/* Private typedef -----------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/

/* These function overloads the TIM_BDTRConfig and TIM_BDTRStructInit
   of the standard library */
void R3_3_G4XX_TIMxInit(TIM_TypeDef *TIMx, PWMC_Handle_t *pHdl);
uint16_t R3_3_G4XX_WriteTIMRegisters(PWMC_Handle_t *pHdl, uint16_t hCCR4Reg);
void R3_3_G4XX_HFCurrentsCalibrationABC(PWMC_Handle_t *pHdl, Curr_Components *pStator_Currents);
void R3_3_G4XX_SetAOReferenceVoltage(uint32_t DAC_Channel, uint16_t hDACVref);

/* Private functions ---------------------------------------------------------*/

/* Local redefinition of both LL_TIM_OC_EnablePreload & LL_TIM_OC_DisablePreload */
__STATIC_INLINE void __LL_TIM_OC_EnablePreload(TIM_TypeDef *TIMx, uint32_t Channel)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  register volatile uint32_t *pReg = (uint32_t *)((uint32_t)((uint32_t)(&TIMx->CCMR1) + OFFSET_TAB_CCMRx[iChannel]));
  SET_BIT(*pReg, (TIM_CCMR1_OC1PE << SHIFT_TAB_OCxx[iChannel]));
}

__STATIC_INLINE void __LL_TIM_OC_DisablePreload(TIM_TypeDef *TIMx, uint32_t Channel)
{
  register uint8_t iChannel = TIM_GET_CHANNEL_INDEX(Channel);
  register volatile uint32_t *pReg = (uint32_t *)((uint32_t)((uint32_t)(&TIMx->CCMR1) + OFFSET_TAB_CCMRx[iChannel]));
  CLEAR_BIT(*pReg, (TIM_CCMR1_OC1PE << SHIFT_TAB_OCxx[iChannel]));
}

/**
  * @defgroup R3_3_G4XX_pwm_curr_fdbk R3 3 ADCs PWM & Current Feedback
  *
  * @brief 3-Shunt, 3 ADCs, PWM & Current Feedback implementation for G4XX MCU
  *
  * This component is used in applications based on an STM32G4 MCU, using a three
  * shunt resistors current sensing topology and 3 ADC peripherals to acquire the current
  * values.
  *
  * @{
  */

/**
  * @brief  Initializes TIMx, ADC, GPIO, DMA1 and NVIC for current reading
  *         in three shunt topology using STM32G4XX and shared ADC.
  * 
  * @param  pHandle: handler of the current instance of the PWM component.
  */
__weak void R3_3_G4XX_Init(PWMC_R3_3_G4_Handle_t *pHandle)
{
  pR3_3_G4XXOPAMPParams_t pDOPAMPParams_str = pHandle->pParams_str->pOPAMPParams;
  COMP_TypeDef *COMP_OCPAx = pHandle->pParams_str->wCompOCPASelection;
  COMP_TypeDef *COMP_OCPBx = pHandle->pParams_str->wCompOCPBSelection;
  COMP_TypeDef *COMP_OCPCx = pHandle->pParams_str->wCompOCPCSelection;
  COMP_TypeDef *COMP_OVPx = pHandle->pParams_str->wCompOVPSelection;
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;

  if ((uint32_t)pHandle == (uint32_t)&pHandle->_Super)
  {
    /* disable IT and flags in case of LL driver usage
     * workaround for unwanted interrupt enabling done by LL driver */
    LL_ADC_DisableIT_EOC(pHandle->pParams_str->ADCx_A);
    LL_ADC_ClearFlag_EOC(pHandle->pParams_str->ADCx_A);
    LL_ADC_DisableIT_JEOC(pHandle->pParams_str->ADCx_A);
    LL_ADC_ClearFlag_JEOC(pHandle->pParams_str->ADCx_A);
    LL_ADC_DisableIT_EOC(pHandle->pParams_str->ADCx_B);
    LL_ADC_ClearFlag_EOC(pHandle->pParams_str->ADCx_B);
    LL_ADC_DisableIT_JEOC(pHandle->pParams_str->ADCx_B);
    LL_ADC_ClearFlag_JEOC(pHandle->pParams_str->ADCx_B);
    LL_ADC_DisableIT_EOC(pHandle->pParams_str->ADCx_C);
    LL_ADC_ClearFlag_EOC(pHandle->pParams_str->ADCx_C);
    LL_ADC_DisableIT_JEOC(pHandle->pParams_str->ADCx_C);
    LL_ADC_ClearFlag_JEOC(pHandle->pParams_str->ADCx_C);

    R3_3_G4XX_TIMxInit(TIMx, &pHandle->_Super);

    if (TIMx == TIM1)
    {
      /* TIM1 Counter Clock stopped when the core is halted */
      LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_TIM1_STOP);
    }
    else
    {
      /* TIM8 Counter Clock stopped when the core is halted */
      LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_TIM8_STOP);
    }

    if (pDOPAMPParams_str)
    {
      LL_OPAMP_Enable(pDOPAMPParams_str->OPAMP_PHA);
      LL_OPAMP_Enable(pDOPAMPParams_str->OPAMP_PHB);
      LL_OPAMP_Enable(pDOPAMPParams_str->OPAMP_PHC);

    }

    /* Over current protection phase A */
    if (COMP_OCPAx)
    {
      /* Inverting input*/
      if (pHandle->pParams_str->bCompOCPAInvInput_MODE != EXT_MODE)
      {
        if (LL_COMP_GetInputMinus(COMP_OCPAx) == LL_COMP_INPUT_MINUS_DAC1_CH1)
        {
          R3_3_G4XX_SetAOReferenceVoltage(LL_DAC_CHANNEL_1, (uint16_t)(pHandle->pParams_str->hDAC_OCP_Threshold));
        }
#if defined(DAC_CHANNEL2_SUPPORT)
        else if (LL_COMP_GetInputMinus(COMP_OCPAx) == LL_COMP_INPUT_MINUS_DAC1_CH2)
        {
          R3_3_G4XX_SetAOReferenceVoltage(LL_DAC_CHANNEL_2, (uint16_t)(pHandle->pParams_str->hDAC_OCP_Threshold));
        }
#endif
        else
        {
          /* Nothing to do */
        }
      }
      else
      {
        /* Nothing to do */
      }

      /* Wait to stabilize DAC voltage */
      volatile uint16_t waittime = 0u;
      for (waittime = 0u; waittime < 1000u; waittime++)
      {}

      /* Output */
      LL_COMP_Enable(COMP_OCPAx);
      LL_COMP_Lock(COMP_OCPAx);
    }

    /* Over current protection phase B */
    if (COMP_OCPBx)
    {
      LL_COMP_Enable(COMP_OCPBx);
      LL_COMP_Lock(COMP_OCPBx);
    }
    else
    {
      /* Nothing to do */
    }

    /* Over current protection phase C */
    if (COMP_OCPCx)
    {
      LL_COMP_Enable(COMP_OCPCx);
      LL_COMP_Lock(COMP_OCPCx);
    }
    else
    {
      /* Nothing to do */
    }

    /* Over voltage protection */
    if (COMP_OVPx)
    {
      /* Inverting input*/
      if (pHandle->pParams_str->bCompOVPInvInput_MODE != EXT_MODE)
      {
        if (LL_COMP_GetInputMinus(COMP_OVPx) == LL_COMP_INPUT_MINUS_DAC1_CH1)
        {
          R3_3_G4XX_SetAOReferenceVoltage(LL_DAC_CHANNEL_1, (uint16_t)(pHandle->pParams_str->hDAC_OVP_Threshold));
        }
#if defined(DAC_CHANNEL2_SUPPORT)
        else if (LL_COMP_GetInputMinus(COMP_OVPx) == LL_COMP_INPUT_MINUS_DAC1_CH2)
        {
          R3_3_G4XX_SetAOReferenceVoltage(LL_DAC_CHANNEL_2, (uint16_t)(pHandle->pParams_str->hDAC_OVP_Threshold));
        }
#endif
        else
        {
          /* Nothing to do */
        }
      }

      /* Wait to stabilize DAC voltage */
      volatile uint16_t waittime = 0u;
      for (waittime = 0u; waittime < 1000u; waittime++)
      {}

      /* Output */
      LL_COMP_Enable(COMP_OVPx);
      LL_COMP_Lock(COMP_OVPx);
    }

    if (pHandle->_Super.bMotor == M1)
    {
      /* - Exit from deep-power-down mode */
      LL_ADC_DisableDeepPowerDown(pHandle->pParams_str->ADCx_A);
      LL_ADC_DisableDeepPowerDown(pHandle->pParams_str->ADCx_B);
      LL_ADC_DisableDeepPowerDown(pHandle->pParams_str->ADCx_C);

      if (LL_ADC_IsInternalRegulatorEnabled(pHandle->pParams_str->ADCx_A) == 0)
      {
        /* Enable ADC internal voltage regulator */
        LL_ADC_EnableInternalRegulator(pHandle->pParams_str->ADCx_A);
        LL_ADC_EnableInternalRegulator(pHandle->pParams_str->ADCx_B);
        LL_ADC_EnableInternalRegulator(pHandle->pParams_str->ADCx_C);

        /* Wait for Regulator Startup time */
        /* Note: Variable divided by 2 to compensate partially              */
        /*       CPU processing cycles, scaling in us split to not          */
        /*       exceed 32 bits register capacity and handle low frequency. */
        uint32_t wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US / 10UL) * (SystemCoreClock / (100000UL * 2UL)));
        while (wait_loop_index != 0UL)
        {
          wait_loop_index--;
        }
      }
      else
      {
        /* Nothing to do */
      }

      LL_ADC_StartCalibration(pHandle->pParams_str->ADCx_A, LL_ADC_SINGLE_ENDED);
      while (LL_ADC_IsCalibrationOnGoing(pHandle->pParams_str->ADCx_A))
      {}

      LL_ADC_StartCalibration(pHandle->pParams_str->ADCx_B, LL_ADC_SINGLE_ENDED);
      while (LL_ADC_IsCalibrationOnGoing(pHandle->pParams_str->ADCx_B))
      {}

      LL_ADC_StartCalibration(pHandle->pParams_str->ADCx_C, LL_ADC_SINGLE_ENDED);
      while (LL_ADC_IsCalibrationOnGoing(pHandle->pParams_str->ADCx_C))
      {}

      if ((pHandle->pParams_str->regconvADCx != pHandle->pParams_str->ADCx_A)
          && (pHandle->pParams_str->regconvADCx != pHandle->pParams_str->ADCx_B)
          && (pHandle->pParams_str->regconvADCx != pHandle->pParams_str->ADCx_C))
      {
        LL_ADC_EnableInternalRegulator(pHandle->pParams_str->regconvADCx);

        /* Wait for Regulator Startup time, once for both */
        uint16_t waittime = 0u;
        for (waittime = 0u; waittime < 65000u; waittime++)
        {}

        LL_ADC_StartCalibration(pHandle->pParams_str->regconvADCx, LL_ADC_SINGLE_ENDED);
        while (LL_ADC_IsCalibrationOnGoing(pHandle->pParams_str->regconvADCx))
        {}
      }
      else
      {
        /* Nothing to do */
      }

      /* Enable ADCx_A, ADCx_B and ADCx_C*/
      LL_ADC_Enable(pHandle->pParams_str->ADCx_A);
      LL_ADC_Enable(pHandle->pParams_str->ADCx_B);
      LL_ADC_Enable(pHandle->pParams_str->ADCx_C);
    }
    else
    {
      /* already done by the first motor */
    }

    if ((pHandle->pParams_str->regconvADCx != pHandle->pParams_str->ADCx_A)
        && (pHandle->pParams_str->regconvADCx != pHandle->pParams_str->ADCx_B)
        && (pHandle->pParams_str->regconvADCx != pHandle->pParams_str->ADCx_C))
    {
      LL_ADC_Enable(pHandle->pParams_str->regconvADCx);
    }

    /* reset regular conversion sequencer length set by cubeMX */
    LL_ADC_REG_SetSequencerLength(pHandle->pParams_str->regconvADCx, LL_ADC_REG_SEQ_SCAN_DISABLE);

    pHandle->wADC_JSQR_phA =  pHandle->pParams_str->ADCx_A->JSQR;
    pHandle->wADC_JSQR_phB =  pHandle->pParams_str->ADCx_B->JSQR;
    pHandle->wADC_JSQR_phC =  pHandle->pParams_str->ADCx_C->JSQR;
    CLEAR_BIT(pHandle->wADC_JSQR_phA, ADC_JSQR_JEXTEN);
    CLEAR_BIT(pHandle->wADC_JSQR_phB, ADC_JSQR_JEXTEN);
    CLEAR_BIT(pHandle->wADC_JSQR_phC, ADC_JSQR_JEXTEN);

    LL_ADC_INJ_SetQueueMode(pHandle->pParams_str->ADCx_A, LL_ADC_INJ_QUEUE_2CONTEXTS_END_EMPTY);
    LL_ADC_INJ_SetQueueMode(pHandle->pParams_str->ADCx_B, LL_ADC_INJ_QUEUE_2CONTEXTS_END_EMPTY);
    LL_ADC_INJ_SetQueueMode(pHandle->pParams_str->ADCx_C, LL_ADC_INJ_QUEUE_2CONTEXTS_END_EMPTY);

#if NOT_IMPLEMENTED
    pHandle->pParams_str->ADCx_A->JSQR = pHandle->wADC_JSQR_phA + LL_ADC_INJ_TRIG_EXT_RISING;
    pHandle->pParams_str->ADCx_B->JSQR = pHandle->wADC_JSQR_phB + LL_ADC_INJ_TRIG_EXT_RISING;
    pHandle->pParams_str->ADCx_C->JSQR = pHandle->wADC_JSQR_phC + LL_ADC_INJ_TRIG_EXT_RISING;
#endif
    LL_ADC_INJ_StartConversion(pHandle->pParams_str->ADCx_A);
    LL_ADC_INJ_StartConversion(pHandle->pParams_str->ADCx_B);
    LL_ADC_INJ_StartConversion(pHandle->pParams_str->ADCx_C);
#if NOT_IMPLEMENTED
    LL_TIM_OC_DisablePreload( TIMx, LL_TIM_CHANNEL_CH4 );
    LL_TIM_OC_SetCompareCH4( TIMx, 0xFFFFu );
    LL_TIM_OC_SetCompareCH4( TIMx, 0x0u );
    LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH4 );

    while ( LL_ADC_IsActiveFlag_JEOS( pHandle->pParams_str->ADCx_A ) == RESET )
    {}
    while ( LL_ADC_IsActiveFlag_JEOS( pHandle->pParams_str->ADCx_B ) == RESET )
    {}
    while ( LL_ADC_IsActiveFlag_JEOS( pHandle->pParams_str->ADCx_C ) == RESET )
    {}

    /* ADCx_ Injected conversions end interrupt enabling */
    LL_ADC_ClearFlag_JEOS( pHandle->pParams_str->ADCx_A );
    LL_ADC_ClearFlag_JEOS( pHandle->pParams_str->ADCx_B );
    LL_ADC_ClearFlag_JEOS( pHandle->pParams_str->ADCx_C );
#endif


    /* TODO: check this It pending */
    NVIC_ClearPendingIRQ(ADC3_IRQn);
    LL_ADC_EnableIT_JEOS(pHandle->pParams_str->ADCx_C);

    /* Clear the flags */
    pHandle->_Super.DTTest = 0u;
    pHandle->_Super.DTCompCnt = pHandle->_Super.hDTCompCnt;
  }
}

/**
  * @brief  Initializes @p TIMx peripheral with @p pHdl handler for PWM generation.
  * 
  */
__weak void R3_3_G4XX_TIMxInit(TIM_TypeDef *TIMx, PWMC_Handle_t *pHdl)
{
  PWMC_R3_3_G4_Handle_t *pHandle = (PWMC_R3_3_G4_Handle_t *)pHdl;

  /* disable main TIM counter to ensure
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

  LL_TIM_ClearFlag_BRK(TIMx);
  LL_TIM_ClearFlag_BRK2(TIMx);
  LL_TIM_EnableIT_BRK(TIMx);

  /* Prepare timer for synchronization */
  LL_TIM_GenerateEvent_UPDATE(TIMx);
  if (pHandle->pParams_str->bFreqRatio == 2u)
  {
    if (pHandle->pParams_str->bIsHigherFreqTim == HIGHER_FREQ)
    {
      if (pHandle->pParams_str->RepetitionCounter == 3u)
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
    if (pHandle->_Super.bMotor == M1)
    {
      if (pHandle->pParams_str->RepetitionCounter == 1u)
      {
        LL_TIM_SetCounter(TIMx, (uint32_t)(pHandle->Half_PWMPeriod) - 1u);
      }
      else if (pHandle->pParams_str->RepetitionCounter == 3u)
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
  * @brief  Sets the calibrated offsets.
  *
  * @param pHdl: Handler of the current instance of the PWM component.
  * @param offsets: Pointer to the structure that contains the offsets for each phase.
  */
__weak void R3_3_SetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets)
{
  PWMC_R3_3_Handle_t *pHandle = (PWMC_R3_3_Handle_t *)pHdl; //cstat !MISRAC2012-Rule-11.3

  pHandle->PhaseAOffset = offsets->phaseAOffset;
  pHandle->PhaseBOffset = offsets->phaseBOffset;
  pHandle->PhaseCOffset = offsets->phaseCOffset;
  pHdl->offsetCalibStatus = true;
}

/**
  * @brief Reads the calibrated @p offsets stored in @p pHdl.
  * 
  */
__weak void R3_3_GetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets)
{
  PWMC_R3_3_Handle_t *pHandle = (PWMC_R3_3_Handle_t *)pHdl; //cstat !MISRAC2012-Rule-11.3

  offsets->phaseAOffset = pHandle->PhaseAOffset;
  offsets->phaseBOffset = pHandle->PhaseBOffset;
  offsets->phaseCOffset = pHandle->PhaseCOffset;
}

/**
  * @brief  Stores into the @p pHdl handler the voltage present on Ia and
  *         Ib current feedback analog channels when no current is flowing into the
  *         motor.
  * 
  */
__weak void R3_3_G4XX_CurrentReadingCalibration(PWMC_Handle_t *pHdl)
{
  PWMC_R3_3_G4_Handle_t *pHandle = (PWMC_R3_3_G4_Handle_t *)pHdl;
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;

  if (false == pHandle->_Super.offsetCalibStatus)
  {
    pHandle-> wPhaseAOffset = 0u;
    pHandle-> wPhaseBOffset = 0u;
    pHandle-> wPhaseCOffset = 0u;

    pHandle->bIndex = 0u;

    /* It forces inactive level on TIMx CHy and CHyN */
    LL_TIM_CC_DisableChannel(TIMx, TIMxCCER_MASK_CH123);

    /* Offset calibration for all phases */
    /* Change function to be executed in ADCx_ISR */
    pHandle->_Super.pFctGetPhaseCurrents = &R3_3_G4XX_HFCurrentsCalibrationABC;

    R3_3_G4XX_SwitchOnPWM( &pHandle->_Super );

    /* Wait for NB_CONVERSIONS to be executed */
    waitForPolarizationEnd(TIMx,
                           &pHandle->_Super.SWerror,
                           pHandle->pParams_str->RepetitionCounter,
                           &pHandle->bIndex);

    R3_3_G4XX_SwitchOffPWM( &pHandle->_Super );

    pHandle->wPhaseAOffset >>= 4;
    pHandle->wPhaseBOffset >>= 4;
    pHandle->wPhaseCOffset >>= 4;
    if (0U == pHandle->_Super.SWerror)
    {
      pHandle->_Super.offsetCalibStatus = true;
    }
    else
    {
      /* nothing to do */
    }

    /* Change back function to be executed in ADCx_ISR */
    pHandle->_Super.pFctGetPhaseCurrents = &R3_3_G4XX_GetPhaseCurrents;
  }
  else
  {
    /* Nothing to do */
  }
  /* It over write TIMx CCRy wrongly written by FOC during calibration so as to
     force 50% duty cycle on the three inverer legs */
  /* Disable TIMx preload */
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_SetCompareCH1(TIMx, pHandle->Half_PWMPeriod);
  LL_TIM_OC_SetCompareCH2(TIMx, pHandle->Half_PWMPeriod);
  LL_TIM_OC_SetCompareCH3(TIMx, pHandle->Half_PWMPeriod);
  /* Enable TIMx preload */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH3);

  /* It re-enable drive of TIMx CHy and CHyN by TIMx CHyRef*/
  LL_TIM_CC_EnableChannel(TIMx, TIMxCCER_MASK_CH123);

  pHandle->_Super.BrakeActionLock = false;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
  * @brief  Computes and stores in @p pHdl handler the latest converted motor phase currents in @p pStator_Currents Curr_Components format.
  *
  */
__weak void R3_3_G4XX_GetPhaseCurrents(PWMC_Handle_t *pHdl, Curr_Components *pStator_Currents)
{
  int32_t wAux;
  uint16_t phaseA;
  uint16_t phaseB;
  uint16_t phaseC;
  uint8_t bSector;
  static uint16_t i;

  PWMC_R3_3_G4_Handle_t *pHandle = (PWMC_R3_3_G4_Handle_t *)pHdl;

  /* Reset the SOFOC flag to indicate the start of FOC algorithm*/
  pHandle->bSoFOC = 0u;

  phaseA = (uint16_t)(pHandle->pParams_str->ADCx_A->JDR1);
  phaseB = (uint16_t)(pHandle->pParams_str->ADCx_B->JDR1);
  phaseC = (uint16_t)(pHandle->pParams_str->ADCx_C->JDR1);

  bSector = (uint8_t)pHandle->_Super.hSector;

  switch (bSector)
  {
    case SECTOR_4:
    case SECTOR_5:
    {
      /* Current on Phase C is not accessible     */
      /* Ia = PhaseAOffset - ADC converted value) */
      wAux = (int32_t)(pHandle->wPhaseAOffset) - (int32_t)(phaseA);

      /* Saturation of Ia */
      if (wAux < -INT16_MAX)
      {
        pStator_Currents->qI_Component1 = -INT16_MAX;
      }
      else  if (wAux > INT16_MAX)
      {
        pStator_Currents->qI_Component1 = INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component1 = (int16_t)wAux;
      }

      /* Ib = PhaseBOffset - ADC converted value) */
      wAux = (int32_t)(pHandle->wPhaseBOffset) - (int32_t)(phaseB);

      /* Saturation of Ib */
      if (wAux < -INT16_MAX)
      {
        pStator_Currents->qI_Component2 = -INT16_MAX;
      }
      else  if (wAux > INT16_MAX)
      {
        pStator_Currents->qI_Component2 = INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component2 = (int16_t)wAux;
      }
      break;
    }

    case SECTOR_6:
    case SECTOR_1:
    {
      /* Current on Phase A is not accessible     */
      /* Ib = PhaseBOffset - ADC converted value) */
      wAux = (int32_t)(pHandle->wPhaseBOffset) - (int32_t)(phaseB);

      /* Saturation of Ib */
      if (wAux < -INT16_MAX)
      {
        pStator_Currents->qI_Component2 = -INT16_MAX;
      }
      else  if (wAux > INT16_MAX)
      {
        pStator_Currents->qI_Component2 = INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component2 = (int16_t)wAux;
      }

      /* Ia = -Ic -Ib */
      wAux = (int32_t)(phaseC) - (int32_t)(pHandle->wPhaseCOffset);         /* -Ic */
      wAux -= (int32_t)pStator_Currents->qI_Component2;               /* Ia  */

      /* Saturation of Ia */
      if (wAux > INT16_MAX)
      {
        pStator_Currents->qI_Component1 = INT16_MAX;
      }
      else  if (wAux < -INT16_MAX)
      {
        pStator_Currents->qI_Component1 = -INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component1 = (int16_t)wAux;
      }
      break;
    }

    case SECTOR_2:
    case SECTOR_3:
    {
      /* Current on Phase B is not accessible     */
      /* Ia = PhaseAOffset - ADC converted value) */
      wAux = (int32_t)(pHandle->wPhaseAOffset) - (int32_t)(phaseA);

      /* Saturation of Ia */
      if (wAux < -INT16_MAX)
      {
        pStator_Currents->qI_Component1 = -INT16_MAX;
      }
      else  if (wAux > INT16_MAX)
      {
        pStator_Currents->qI_Component1 = INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component1 = (int16_t)wAux;
      }

      /* Ib = -Ic -Ia */
      wAux = (int32_t)(phaseC) - (int32_t)(pHandle->wPhaseCOffset);         /* -Ic */
      wAux -= (int32_t)pStator_Currents->qI_Component1;               /* Ib */

      /* Saturation of Ib */
      if (wAux > INT16_MAX)
      {
        pStator_Currents->qI_Component2 = INT16_MAX;
      }
      else  if (wAux < -INT16_MAX)
      {
        pStator_Currents->qI_Component2 = -INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component2 = (int16_t)wAux;
      }
      break;
    }

    default:
      break;
  }

  pHandle->_Super.hIa = pStator_Currents->qI_Component1;
  pHandle->_Super.hIb = pStator_Currents->qI_Component2;
  pHandle->_Super.hIc = -pStator_Currents->qI_Component1 - pStator_Currents->qI_Component2;
}

/**
  * @brief  Implementation of PWMC_GetPhaseCurrents to be performed during calibration.
  *         
  * It sums up injected conversion data into PhaseAOffset, wPhaseBOffset and wPhaseCOffset
  * to compute the offset introduced in the current feedback network. It is required to 
  * properly configure ADC inputs before in order to enable offset computation.
  * 
  * @param  pHdl: Pointer on the target component instance.
  * @param  pStator_Currents: Pointer to the structure that will receive motor current
  *         of phase A and B in ab_t format.
  */
__weak void R3_3_G4XX_HFCurrentsCalibrationABC(PWMC_Handle_t *pHdl, Curr_Components *pStator_Currents)
{
  PWMC_R3_3_G4_Handle_t *pHandle = (PWMC_R3_3_G4_Handle_t *)pHdl;

  /* Reset the SOFOC flag to indicate the start of FOC algorithm*/
  pHandle->bSoFOC = 0u;

  if (pHandle->bIndex < NB_CONVERSIONS)
  {
    pHandle-> wPhaseAOffset += pHandle->pParams_str->ADCx_A->JDR1;
    pHandle-> wPhaseBOffset += pHandle->pParams_str->ADCx_B->JDR1;
    pHandle-> wPhaseCOffset += pHandle->pParams_str->ADCx_C->JDR1;
    pHandle->bIndex++;
  }
  else
  {
    /* Nothing to do */
  }

  /* during offset calibration no current is flowing in the phases */
  pStator_Currents->qI_Component1 = 0;
  pStator_Currents->qI_Component2 = 0;
}

/**
  * @brief  Turns on low sides switches.
  * 
  * This function is intended to be used for charging boot capacitors of driving section. It has to be
  * called each motor start-up when using high voltage drivers.
  * 
  * @param  pHdl: Handler of the current instance of the PWM component
  * @param  ticks: Timer ticks value to be applied
  *                Min value: 0 (low sides ON)
  *                Max value: PWM_PERIOD_CYCLES/2 (low sides OFF)
  */
__weak void R3_3_G4XX_TurnOnLowSides(PWMC_Handle_t *pHdl, uint32_t ticks)
{
  PWMC_R3_3_G4_Handle_t *pHandle = (PWMC_R3_3_G4_Handle_t *)pHdl;
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.bTurnOnLowSidesAction = true;

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(pHandle->pParams_str->TIMx);

  /*Turn on the three low side switches */
  LL_TIM_OC_SetCompareCH1(TIMx, ticks);
  LL_TIM_OC_SetCompareCH2(TIMx, ticks);
  LL_TIM_OC_SetCompareCH3(TIMx, ticks);

  /* Wait until next update */
  while (0U == LL_TIM_IsActiveFlag_UPDATE(TIMx))
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
}


/**
  * @brief  Enables PWM generation on the proper Timer peripheral acting on MOE bit.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
__weak void R3_3_G4XX_SwitchOnPWM(PWMC_Handle_t *pHdl)
{
  PWMC_R3_3_G4_Handle_t *pHandle = (PWMC_R3_3_G4_Handle_t *)pHdl;
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.bTurnOnLowSidesAction = false;

  /* Set all duty to 50% */
  /* Set ch4 for triggering */
  /* Clear Update Flag */
  LL_TIM_OC_SetCompareCH1(TIMx, (uint32_t)(pHandle->Half_PWMPeriod) >> 1);
  LL_TIM_OC_SetCompareCH2(TIMx, (uint32_t)(pHandle->Half_PWMPeriod) >> 1);
  LL_TIM_OC_SetCompareCH3(TIMx, (uint32_t)(pHandle->Half_PWMPeriod) >> 1);
  LL_TIM_OC_SetCompareCH4(TIMx, (uint32_t)(pHandle->Half_PWMPeriod) - 5u);

  /* wait for a new PWM period */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  while (0U == LL_TIM_IsActiveFlag_UPDATE(TIMx))
  {
    /* Nothing to do */
  }
  LL_TIM_ClearFlag_UPDATE(TIMx);

  /* Main PWM Output Enable */
  TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
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
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_ResetOutputPin(pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin);
      LL_GPIO_ResetOutputPin(pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin);
      LL_GPIO_ResetOutputPin(pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin);
    }
  }
  else
  {
    /* Nothing to do */
  }

  pHandle->pParams_str->ADCx_A->JSQR = pHandle->wADC_JSQR_phA + LL_ADC_INJ_TRIG_EXT_RISING;
  pHandle->pParams_str->ADCx_B->JSQR = pHandle->wADC_JSQR_phB + LL_ADC_INJ_TRIG_EXT_RISING;
  pHandle->pParams_str->ADCx_C->JSQR = pHandle->wADC_JSQR_phC + LL_ADC_INJ_TRIG_EXT_RISING;

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  /* Enable Update IRQ */
  LL_TIM_EnableIT_UPDATE(TIMx);
}


/**
  * @brief  Disables PWM generation on the proper Timer peripheral acting on MOE bit.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
__weak void R3_3_G4XX_SwitchOffPWM(PWMC_Handle_t *pHdl)
{
  PWMC_R3_3_G4_Handle_t *pHandle = (PWMC_R3_3_G4_Handle_t *)pHdl;
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;

  /* Disable UPDATE ISR */
  LL_TIM_DisableIT_UPDATE(TIMx);

  pHandle->_Super.bTurnOnLowSidesAction = false;

  /* Main PWM Output Disable */
  LL_TIM_DisableAllOutputs(TIMx);
  if (pHandle->_Super.BrakeActionLock == true)
  {
    /* Nothing to do */
  }
  else
  {
    if ((pHandle->_Super.LowSideOutputs) == ES_GPIO)
    {
      LL_GPIO_ResetOutputPin(pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin);
      LL_GPIO_ResetOutputPin(pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin);
      LL_GPIO_ResetOutputPin(pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin);
    }
    else
    {
      /* nothing to do */
    }
  }

  /* Clear the JSAQR register queue */
  LL_ADC_INJ_StopConversion(pHandle->pParams_str->ADCx_A);
  LL_ADC_INJ_StopConversion(pHandle->pParams_str->ADCx_B);
  LL_ADC_INJ_StopConversion(pHandle->pParams_str->ADCx_C);

  LL_ADC_INJ_StartConversion(pHandle->pParams_str->ADCx_A);
  LL_ADC_INJ_StartConversion(pHandle->pParams_str->ADCx_B);
  LL_ADC_INJ_StartConversion(pHandle->pParams_str->ADCx_C);

  /* wait for a new PWM period to flush last HF task */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  while (0U == LL_TIM_IsActiveFlag_UPDATE(TIMx))
  {
    /* Nothing to do */
  }
  LL_TIM_ClearFlag_UPDATE(TIMx);

}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
  * @brief Writes into peripheral registers the new duty cycles and sampling point.
  * 
  * @param pHdl: Handler of the current instance of the PWM component.
  * @param hCCR4Reg: New capture/compare register value, written in timer clock counts.
  * @retval uint16_t Returns #MC_NO_ERROR if no error occurred or #MC_DURATION if the duty cycles were
  *         set too late for being taken into account in the next PWM cycle.
  */
__weak uint16_t R3_3_G4XX_WriteTIMRegisters(PWMC_Handle_t *pHdl, uint16_t hCCR4Reg)
{
  uint16_t hAux;
  PWMC_R3_3_G4_Handle_t *pHandle = (PWMC_R3_3_G4_Handle_t *)pHdl;
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;

  LL_TIM_OC_SetCompareCH1(TIMx, pHandle->_Super.hCntPhA);
  LL_TIM_OC_SetCompareCH2(TIMx, pHandle->_Super.hCntPhB);
  LL_TIM_OC_SetCompareCH3(TIMx, pHandle->_Super.hCntPhC);

  __LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH4);
  LL_TIM_OC_SetCompareCH4(TIMx, 0xFFFFu);
  __LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH4);
  LL_TIM_OC_SetCompareCH4(TIMx, hCCR4Reg);

  pHandle->pParams_str->ADCx_A->JSQR = pHandle->wADC_JSQR_phA + pHandle->ADC_ExternalPolarityInjected;
  pHandle->pParams_str->ADCx_B->JSQR = pHandle->wADC_JSQR_phB + pHandle->ADC_ExternalPolarityInjected;
  pHandle->pParams_str->ADCx_C->JSQR = pHandle->wADC_JSQR_phC + pHandle->ADC_ExternalPolarityInjected;

  /* Limit for update event */
  /* Check the status of SOFOC flag. If it is set, an update event has occurred
  and thus the FOC rate is too high */
  if (pHandle->bSoFOC != 0u)
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
__attribute__((section(".ccmram")))
#endif
#endif
/**
  * @brief  Configures the ADC for the current sampling related to sector 1.
  * 
  * It sets the ADC sequence length and channels, and the sampling point via TIMx_Ch4 value and polarity.
  * It then calls the WriteTIMRegisters method.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @retval uint16_t Return value of R3_1_WriteTIMRegisters.
  */
__weak uint16_t R3_3_G4XX_SetADCSampPointSect1(PWMC_Handle_t *pHdl)
{
  uint16_t hCntSmp;
  uint16_t hDeltaDuty;
  PWMC_R3_3_G4_Handle_t *pHandle = (PWMC_R3_3_G4_Handle_t *)pHdl;
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;
  pHandle->ADC_ExternalPolarityInjected = LL_ADC_INJ_TRIG_EXT_RISING;

  /* Verify that sampling is possible in the middle of PWM by checking the smallest duty cycle
   * in the sector 1 (i.e phase A duty cycle) */
  if ((uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhA) > pHandle->pParams_str->hTafter)
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4  */
    pHandle->_Super.hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;

  }
  else
  {
    /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/

    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
        duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
        one with minimum complementary duty and one with variable complementary duty. In this case, phases
        with variable complementary duty and with maximum duty are converted and the first will be always
        the phase with variable complementary duty cycle */
    hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhA - pHandle->_Super.hCntPhB);

    /* Definition of crossing point */
    if (hDeltaDuty > ((uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhA) * 2u))
    {
      hCntSmp = pHandle->_Super.hCntPhA - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHandle->_Super.hCntPhA + pHandle->pParams_str->hTafter;

      if (hCntSmp >= pHandle->Half_PWMPeriod)
      {
        /* Set CC4 as PWM mode 1 */
        pHandle->ADC_ExternalPolarityInjected = LL_ADC_INJ_TRIG_EXT_FALLING;
        hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
      }
      else
      {
        /* Nothing to do */
      }
    }
  }

  return (R3_3_G4XX_WriteTIMRegisters(&pHandle->_Super, hCntSmp));
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
  * @brief  Configures the ADC for the current sampling related to sector 2.
  * 
  * It sets the ADC sequence length and channels, and the sampling point via TIMx_Ch4 value and polarity.
  * It then calls the WriteTIMRegisters method.
  *
  * @param  pHandle: Handler of the current instance of the PWM component.
  * @retval uint16_t Return value of R3_1_WriteTIMRegisters.
  */
__weak uint16_t R3_3_G4XX_SetADCSampPointSect2(PWMC_Handle_t *pHdl)
{
  uint16_t hCntSmp;
  uint16_t hDeltaDuty;
  PWMC_R3_3_G4_Handle_t *pHandle = (PWMC_R3_3_G4_Handle_t *)pHdl;
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;
  pHandle->ADC_ExternalPolarityInjected = LL_ADC_INJ_TRIG_EXT_RISING;

  /* Verify that sampling is possible in the middle of PWM by checking the smallest duty cycle
   * in the sector 2 (i.e phase B duty cycle) */
  if ((uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhB) > pHandle->pParams_str->hTafter)
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4  */
    pHandle->_Super.hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;

  }
  else
  {
    /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/

    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
        duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
        one with minimum complementary duty and one with variable complementary duty. In this case, phases
        with variable complementary duty and with maximum duty are converted and the first will be always
        the phase with variable complementary duty cycle */
    hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhB - pHandle->_Super.hCntPhA);

    /* Definition of crossing point */
    if (hDeltaDuty > ((uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhB) * 2u))
    {
      hCntSmp = pHandle->_Super.hCntPhB - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHandle->_Super.hCntPhB + pHandle->pParams_str->hTafter;

      if (hCntSmp >= pHandle->Half_PWMPeriod)
      {
        /* Set CC4 as PWM mode 1 */
        pHandle->ADC_ExternalPolarityInjected = LL_ADC_INJ_TRIG_EXT_FALLING;
        hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
      }
      else
      {
        /* Nothing to do */
      }
    }
  }

  return (R3_3_G4XX_WriteTIMRegisters(&pHandle->_Super, hCntSmp));
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
  * @brief  Configures the ADC for the current sampling related to sector 3.
  * 
  * It sets the ADC sequence length and channels, and the sampling point via TIMx_Ch4 value and polarity.
  * It then calls the WriteTIMRegisters method.
  *
  * @param  pHandle: Handler of the current instance of the PWM component.
  * @retval uint16_t Return value of R3_1_WriteTIMRegisters.
  */
__weak uint16_t R3_3_G4XX_SetADCSampPointSect3(PWMC_Handle_t *pHdl)
{
  uint16_t hCntSmp;
  uint16_t hDeltaDuty;
  PWMC_R3_3_G4_Handle_t *pHandle = (PWMC_R3_3_G4_Handle_t *)pHdl;
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;
  pHandle->ADC_ExternalPolarityInjected = LL_ADC_INJ_TRIG_EXT_RISING;


  /* Verify that sampling is possible in the middle of PWM by checking the smallest duty cycle
   * in the sector 3 (i.e phase B duty cycle) */
  if ((uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhB) > pHandle->pParams_str->hTafter)
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4  */
    pHandle->_Super.hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;
  }
  else
  {
    /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/

    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
        duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
        one with minimum complementary duty and one with variable complementary duty. In this case, phases
        with variable complementary duty and with maximum duty are converted and the first will be always
        the phase with variable complementary duty cycle */
    hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhB - pHandle->_Super.hCntPhC);

    /* Definition of crossing point */
    if (hDeltaDuty > ((uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhB) * 2u))
    {
      hCntSmp = pHandle->_Super.hCntPhB - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHandle->_Super.hCntPhB + pHandle->pParams_str->hTafter;

      if (hCntSmp >= pHandle->Half_PWMPeriod)
      {
        /* Set CC4 as PWM mode 1 */
        pHandle->ADC_ExternalPolarityInjected = LL_ADC_INJ_TRIG_EXT_FALLING;
        hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
      }
      else
      {
        /* Nothing to do */
      }
    }
  }

  return (R3_3_G4XX_WriteTIMRegisters(&pHandle->_Super, hCntSmp));
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
  * @brief  Configures the ADC for the current sampling related to sector 4.
  * 
  * It sets the ADC sequence length and channels, and the sampling point via TIMx_Ch4 value and polarity.
  * It then calls the WriteTIMRegisters method.
  *
  * @param  pHandle: Handler of the current instance of the PWM component.
  * @retval uint16_t Return value of R3_1_WriteTIMRegisters.
  */
__weak uint16_t R3_3_G4XX_SetADCSampPointSect4(PWMC_Handle_t *pHdl)
{
  uint16_t hCntSmp;
  uint16_t hDeltaDuty;
  PWMC_R3_3_G4_Handle_t *pHandle = (PWMC_R3_3_G4_Handle_t *)pHdl;
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;
  pHandle->ADC_ExternalPolarityInjected = LL_ADC_INJ_TRIG_EXT_RISING;


  /* Verify that sampling is possible in the middle of PWM by checking the smallest duty cycle
   * in the sector 4 (i.e phase C duty cycle) */
  if ((uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhC) > pHandle->pParams_str->hTafter)
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4  */
    pHandle->_Super.hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;

  }
  else
  {
    /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/

    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
        duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
        one with minimum complementary duty and one with variable complementary duty. In this case, phases
        with variable complementary duty and with maximum duty are converted and the first will be always
        the phase with variable complementary duty cycle */
    hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhC - pHandle->_Super.hCntPhB);

    /* Definition of crossing point */
    if (hDeltaDuty > ((uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhC) * 2u))
    {
      hCntSmp = pHandle->_Super.hCntPhC - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHandle->_Super.hCntPhC + pHandle->pParams_str->hTafter;

      if (hCntSmp >= pHandle->Half_PWMPeriod)
      {
        /* Set CC4 as PWM mode 1 */
        pHandle->ADC_ExternalPolarityInjected = LL_ADC_INJ_TRIG_EXT_FALLING;
        hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
      }
      else
      {
        /* Nothing to do */
      }
    }
  }

  return (R3_3_G4XX_WriteTIMRegisters(&pHandle->_Super, hCntSmp));
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
  * @brief  Configures the ADC for the current sampling related to sector 5.
  * 
  * It sets the ADC sequence length and channels, and the sampling point via TIMx_Ch4 value and polarity.
  * It then calls the WriteTIMRegisters method.
  *
  * @param  pHandle: Handler of the current instance of the PWM component.
  * @retval uint16_t Return value of R3_1_WriteTIMRegisters.
  */
__weak uint16_t R3_3_G4XX_SetADCSampPointSect5(PWMC_Handle_t *pHdl)
{
  uint16_t hCntSmp;
  uint16_t hDeltaDuty;
  PWMC_R3_3_G4_Handle_t *pHandle = (PWMC_R3_3_G4_Handle_t *)pHdl;
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;
  pHandle->ADC_ExternalPolarityInjected = LL_ADC_INJ_TRIG_EXT_RISING;

  /* Verify that sampling is possible in the middle of PWM by checking the smallest duty cycle
   * in the sector 5 (i.e phase C duty cycle) */
  if ((uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhC) > pHandle->pParams_str->hTafter)
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB, AC  or BC) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4  */
    pHandle->_Super.hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;
  }
  else
  {
    /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/

    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
        duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
        one with minimum complementary duty and one with variable complementary duty. In this case, phases
        with variable complementary duty and with maximum duty are converted and the first will be always
        the phase with variable complementary duty cycle */
    hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhC - pHandle->_Super.hCntPhA);

    /* Definition of crossing point */
    if (hDeltaDuty > ((uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhC) * 2u))
    {
      hCntSmp = pHandle->_Super.hCntPhC - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHandle->_Super.hCntPhC + pHandle->pParams_str->hTafter;

      if (hCntSmp >= pHandle->Half_PWMPeriod)
      {
        /* Set CC4 as PWM mode 1 */
        pHandle->ADC_ExternalPolarityInjected = LL_ADC_INJ_TRIG_EXT_FALLING;
        hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
      }
      else
      {
        /* nothing to do */
      }
    }
  }

  return (R3_3_G4XX_WriteTIMRegisters(&pHandle->_Super, hCntSmp));
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
  * @brief  Configures the ADC for the current sampling related to sector 6.
  * 
  * It sets the ADC sequence length and channels, and the sampling point via TIMx_Ch4 value and polarity.
  * It then calls the WriteTIMRegisters method.
  *
  * @param  pHandle: Handler of the current instance of the PWM component.
  * @retval uint16_t Return value of R3_1_WriteTIMRegisters.
  */
__weak uint16_t R3_3_G4XX_SetADCSampPointSect6(PWMC_Handle_t *pHdl)
{
  uint16_t hCntSmp;
  uint16_t hDeltaDuty;
  PWMC_R3_3_G4_Handle_t *pHandle = (PWMC_R3_3_G4_Handle_t *)pHdl;
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;
  pHandle->ADC_ExternalPolarityInjected = LL_ADC_INJ_TRIG_EXT_RISING;

  /* Verify that sampling is possible in the middle of PWM by checking the smallest duty cycle
   * in the sector 6 (i.e phase A duty cycle) */
  if ((uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhA) > pHandle->pParams_str->hTafter)
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4  */
    pHandle->_Super.hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = (uint32_t)(pHandle->Half_PWMPeriod) - 1u;

  }
  else
  {
    /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/

    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
        duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
        one with minimum complementary duty and one with variable complementary duty. In this case, phases
        with variable complementary duty and with maximum duty are converted and the first will be always
        the phase with variable complementary duty cycle */
    hDeltaDuty = (uint16_t)(pHandle->_Super.hCntPhA - pHandle->_Super.hCntPhC);

    /* Definition of crossing point */
    if (hDeltaDuty > ((uint16_t)(pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhA) * 2u))
    {
      hCntSmp = pHandle->_Super.hCntPhA - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHandle->_Super.hCntPhA + pHandle->pParams_str->hTafter;

      if (hCntSmp >= pHandle->Half_PWMPeriod)
      {
        /* Set CC4 as PWM mode 1 */
        pHandle->ADC_ExternalPolarityInjected = LL_ADC_INJ_TRIG_EXT_FALLING;
        hCntSmp = (2u * pHandle->Half_PWMPeriod) - hCntSmp - 1u;
      }
      else
      {
        /* Nothing to do */
      }
    }
  }
  return (R3_3_G4XX_WriteTIMRegisters(&pHandle->_Super, hCntSmp));
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section(".ccmram")))
#endif
#endif
/**
  * @brief  Contains the TIMx Update event interrupt.
  *
  * @param  pHandle: Handler of the current instance of the PWM component.
  */
__weak void *R3_3_G4XX_TIMx_UP_IRQHandler(PWMC_R3_3_G4_Handle_t *pHandle)
{

  /* Set the SOFOC flag to indicate the execution of Update IRQ*/
  pHandle->bSoFOC = 1u;
  return (&(pHandle->_Super.bMotor));
}

/**
  * @brief  Configures the analog output used for protection thresholds.
  *
  * @param  DAC_Channel: Selected DAC channel.
  *          This parameter can be:
  *            @arg DAC_Channel_1: DAC Channel1 selected.
  *            @arg DAC_Channel_2: DAC Channel2 selected.
  * @param  hDACVref: Value of DAC reference expressed as 16bit unsigned integer. \n
  *         Ex. 0 = 0V ; 65536 = VDD_DAC.
  */
__weak void R3_3_G4XX_SetAOReferenceVoltage(uint32_t DAC_Channel, uint16_t hDACVref)
{

  if (DAC_Channel == LL_DAC_CHANNEL_2)
  {
    LL_DAC_ConvertData12LeftAligned(DAC1, LL_DAC_CHANNEL_2, hDACVref);
  }
  else
  {
    LL_DAC_ConvertData12LeftAligned(DAC1, LL_DAC_CHANNEL_1, hDACVref);
  }

  /* Enable DAC Channel */
  LL_DAC_TrigSWConversion(DAC1, DAC_Channel);
  LL_DAC_Enable(DAC1, DAC_Channel);

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
