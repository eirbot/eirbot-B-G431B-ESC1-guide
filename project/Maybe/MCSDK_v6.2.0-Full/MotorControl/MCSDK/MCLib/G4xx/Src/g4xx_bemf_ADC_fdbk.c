/**
  ******************************************************************************
  * @file    g4xx_bemf_ADC_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement Bemf sensing
  *          class to be stantiated when the six-step sensorless driving mode
  *          topology is used.
  * 
  *          It is specifically designed for STM32G4XX microcontrollers and
  *          implements the sensing using one, two or three independent
  *          ADCs with injected channels.
  *           + MCU peripheral and handle initialization fucntion
  *           + ADC sampling function
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
  * @ingroup SpeednPosFdbk_Bemf
  */

/* Includes ------------------------------------------------------------------*/
#include "g4xx_bemf_ADC_fdbk.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */

/** @addtogroup SpeednPosFdbk_Bemf
  * @{
  */

/* Private defines -----------------------------------------------------------*/
#define MAX_PSEUDO_SPEED        ((int16_t)0x7FFF)

#define PHASE_U 0u
#define PHASE_V 1u
#define PHASE_W 2u
/* Private function prototypes -----------------------------------------------*/
void BADC_CalcAvrgElSpeedDpp( Bemf_ADC_Handle_t * pHandle );
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes ADC and NVIC for three bemf voltages reading
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  */
__weak void BADC_Init( Bemf_ADC_Handle_t *pHandle)
{
  ADC_TypeDef * ADCx_u = pHandle->pParams_str->pAdc[0];
  ADC_TypeDef * ADCx_v = pHandle->pParams_str->pAdc[1];
  ADC_TypeDef * ADCx_w = pHandle->pParams_str->pAdc[2];

  if ( ( uint32_t )pHandle == ( uint32_t )&pHandle->_Super )
  {

    /* disable IT and flags in case of LL driver usage
     * workaround for unwanted interrupt enabling done by LL driver */
    LL_ADC_DisableIT_EOC( ADCx_u );
    LL_ADC_DisableIT_EOC( ADCx_v );
    LL_ADC_DisableIT_EOC( ADCx_w );
    LL_ADC_ClearFlag_EOC( ADCx_u );
    LL_ADC_ClearFlag_EOC( ADCx_v );
    LL_ADC_ClearFlag_EOC( ADCx_w );
    LL_ADC_DisableIT_JEOC( ADCx_u );
    LL_ADC_DisableIT_JEOC( ADCx_v );
    LL_ADC_DisableIT_JEOC( ADCx_w );
    LL_ADC_ClearFlag_JEOC( ADCx_u );
    LL_ADC_ClearFlag_JEOC( ADCx_v );
    LL_ADC_ClearFlag_JEOC( ADCx_w );
    /* - Exit from deep-power-down mode */
    LL_ADC_DisableDeepPowerDown(ADCx_u);
    LL_ADC_DisableDeepPowerDown(ADCx_v);
    LL_ADC_DisableDeepPowerDown(ADCx_w);
    
    LL_ADC_EnableInternalRegulator( ADCx_u );
    LL_ADC_EnableInternalRegulator( ADCx_v );
    LL_ADC_EnableInternalRegulator( ADCx_w );
    volatile uint32_t wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US / 10UL) * (SystemCoreClock / (100000UL * 2UL)));
    while(wait_loop_index != 0UL)
    {
      wait_loop_index--;
    }

    LL_ADC_StartCalibration( ADCx_u, LL_ADC_SINGLE_ENDED );
    while ( LL_ADC_IsCalibrationOnGoing( ADCx_u ) )
    {
    }
    if (ADCx_u != ADCx_v)
    {
      LL_ADC_StartCalibration( ADCx_v, LL_ADC_SINGLE_ENDED );
      while ( LL_ADC_IsCalibrationOnGoing( ADCx_v ) )
      {
      }
    }
    if ((ADCx_w != ADCx_u) && (ADCx_w != ADCx_v))
    {
      LL_ADC_StartCalibration( ADCx_w, LL_ADC_SINGLE_ENDED );
      while ( LL_ADC_IsCalibrationOnGoing( ADCx_w ) )
      {
      }
    }
    /* ADC Enable (must be done after calibration) */
    /* ADC5-140924: Enabling the ADC by setting ADEN bit soon after polling ADCAL=0
    * following a calibration phase, could have no effect on ADC
    * within certain AHB/ADC clock ratio.
    */
    LL_ADC_SetChannelSamplingTime (ADCx_u, pHandle->pParams_str->AdcChannel[0], LL_ADC_SAMPLINGTIME_2CYCLES_5 );
    LL_ADC_SetChannelSamplingTime (ADCx_v, pHandle->pParams_str->AdcChannel[1], LL_ADC_SAMPLINGTIME_2CYCLES_5 );
    LL_ADC_SetChannelSamplingTime (ADCx_w, pHandle->pParams_str->AdcChannel[2], LL_ADC_SAMPLINGTIME_2CYCLES_5 );
    
    while (  LL_ADC_IsActiveFlag_ADRDY( ADCx_u ) == 0u)
    {
      LL_ADC_Enable(  ADCx_u );
    }    
    while (  LL_ADC_IsActiveFlag_ADRDY( ADCx_v ) == 0u)
    {
      LL_ADC_Enable(  ADCx_v );
    }
    while (  LL_ADC_IsActiveFlag_ADRDY( ADCx_w ) == 0u)
    {
      LL_ADC_Enable(  ADCx_w );
    } 

    pHandle->ADCRegularLocked=false;
	
    uint16_t hMinReliableElSpeedUnit = pHandle->_Super.hMinReliableMecSpeedUnit * pHandle->_Super.bElToMecRatio;
    uint16_t hMaxReliableElSpeedUnit = pHandle->_Super.hMaxReliableMecSpeedUnit * pHandle->_Super.bElToMecRatio;
    uint8_t bSpeedBufferSize;
    uint8_t bIndex;
    uint16_t MinBemfTime;
	
    /* Adjustment factor: minimum measurable speed is x time less than the minimum
    reliable speed */
    hMinReliableElSpeedUnit /= 4U;

    /* Adjustment factor: maximum measurable speed is x time greater than the
    maximum reliable speed */
    hMaxReliableElSpeedUnit *= 2U;

    pHandle->OvfFreq = (uint16_t)(pHandle->TIMClockFreq / 65536U);

    /* SW Init */
    if (0U == hMinReliableElSpeedUnit)
    {

      /* Set fixed to 150 ms */
      pHandle->BemfTimeout = 150U;
    }
    else
    {
      /* Set accordingly the min reliable speed */
      /* 1000 comes from mS
      * 6 comes from the fact that sensors are toggling each 60 deg = 360/6 deg */
      pHandle->BemfTimeout = (1000U * (uint16_t)SPEED_UNIT) / (6U * hMinReliableElSpeedUnit);
    }

    /* Align MaxPeriod and MinPeriod to a multiple of Overflow.*/
    pHandle->MaxPeriod = (pHandle->BemfTimeout * pHandle->OvfFreq) / 1000U * 65536UL;

    MinBemfTime = ((1000U * (uint16_t)SPEED_UNIT) << 8) / (6U * hMaxReliableElSpeedUnit);
    pHandle->MinPeriod = ((MinBemfTime * pHandle->OvfFreq) >> 8) / 1000U * 65536UL;
	
    pHandle->SatSpeed = hMaxReliableElSpeedUnit;

    pHandle->PseudoPeriodConv = ((pHandle->TIMClockFreq / 6U) / pHandle->_Super.hMeasurementFrequency)
                              * pHandle->_Super.DPPConvFactor;

    if (0U == hMaxReliableElSpeedUnit)
    {
      pHandle->MinPeriod = ((uint32_t)SPEED_UNIT * (pHandle->TIMClockFreq / 6UL));
    }
    else
    {
      pHandle->MinPeriod = (((uint32_t)SPEED_UNIT * (pHandle->TIMClockFreq / 6UL)) / hMaxReliableElSpeedUnit);
    }
    pHandle->PWMNbrPSamplingFreq = ((pHandle->_Super.hMeasurementFrequency * pHandle->PWMFreqScaling) /
                                    pHandle->SpeedSamplingFreqHz) - 1U;

    pHandle->pSensing_Params = &(pHandle->Pwm_OFF);
    pHandle->IsOnSensingEnabled = false;
    pHandle->ElPeriodSum = 0;
    pHandle->ZcEvents = 0;
    pHandle->DemagCounterThreshold  = pHandle->DemagParams.DemagMinimumThreshold;	
    
    /* Erase speed buffer */
    bSpeedBufferSize = pHandle->SpeedBufferSize;
    for (bIndex = 0u; bIndex < bSpeedBufferSize; bIndex++)
    {
      pHandle->SpeedBufferDpp[bIndex]  = (int32_t)pHandle->MaxPeriod;
    }

    LL_TIM_EnableCounter(pHandle->pParams_str->LfTim);
  }
}

/**
  * @brief  Resets the parameter values of the component
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  */
__weak void BADC_Clear( Bemf_ADC_Handle_t *pHandle )
{
  pHandle->ZcEvents = 0;
  pHandle->ElPeriodSum = 0;

  /* Acceleration measurement not implemented.*/
  pHandle->_Super.hMecAccelUnitP = 0;
  pHandle->BufferFilled = 0U;
  pHandle->CompSpeed = 0;

  /* Initialize speed buffer index */
  pHandle->SpeedFIFOIdx = 0U;
  pHandle->_Super.hElAngle  = 0;
  
  /* Clear speed error counter */
  pHandle->_Super.bSpeedErrorNumber = 0;
  pHandle->IsLoopClosed=false;
  pHandle->IsAlgorithmConverged = false;
}

/**
 * @brief  Starts bemf ADC conversion of the phase depending on current step
 * @param  pHandle: handler of the current instance of the Bemf_ADC component
 * @param  step: current step of the six-step sequence
 */
__weak void BADC_Start(Bemf_ADC_Handle_t *pHandle, uint8_t step)
{
  ADC_TypeDef * ADCx_u = pHandle->pParams_str->pAdc[0];
  ADC_TypeDef * ADCx_v = pHandle->pParams_str->pAdc[1];
  ADC_TypeDef * ADCx_w = pHandle->pParams_str->pAdc[2];
  if (true == pHandle->ADCRegularLocked)
  {
    LL_ADC_DisableIT_JEOC( ADCx_u );
    LL_ADC_DisableIT_JEOC( ADCx_v );
    LL_ADC_DisableIT_JEOC( ADCx_w );
  }
  else
  {
    pHandle->ADCRegularLocked = true;
  }

  switch (step)
  {
    case STEP_1:
    case STEP_4:
        BADC_SelectAdcChannel(pHandle, PHASE_W);
          /*start injected conversion */
        LL_ADC_EnableIT_JEOC( ADCx_w );
        LL_ADC_INJ_StartConversion( ADCx_w );
      break;
    case STEP_2:
    case STEP_5:
        BADC_SelectAdcChannel(pHandle, PHASE_V);
          /*start injected conversion */
        LL_ADC_EnableIT_JEOC( ADCx_v );
        LL_ADC_INJ_StartConversion( ADCx_v );
      break;
    case STEP_3:
    case STEP_6: 
        BADC_SelectAdcChannel(pHandle, PHASE_U);
          /*start injected conversion */
        LL_ADC_EnableIT_JEOC( ADCx_u );
        LL_ADC_INJ_StartConversion( ADCx_u );
      break;
    default:
      break;
  }
}

/**
 * @brief  Stops bemf ADC conversion
 * @param  pHandle: handler of the current instance of the Bemf_ADC component
 */
__weak void BADC_Stop(Bemf_ADC_Handle_t *pHandle)
{
  ADC_TypeDef * ADCx_u = pHandle->pParams_str->pAdc[0];
  ADC_TypeDef * ADCx_v = pHandle->pParams_str->pAdc[1];
  ADC_TypeDef * ADCx_w = pHandle->pParams_str->pAdc[2];

  /* Disable JEOC */
  LL_ADC_DisableIT_JEOC( ADCx_u );
  LL_ADC_DisableIT_JEOC( ADCx_v );
  LL_ADC_DisableIT_JEOC( ADCx_w );
  /* Clear JEOC */
  LL_ADC_ClearFlag_JEOC( ADCx_u );
  LL_ADC_ClearFlag_JEOC( ADCx_v );
  LL_ADC_ClearFlag_JEOC( ADCx_w );
  if (true == pHandle->ADCRegularLocked)
  {
  /* Stop ADC injected conversion */
    LL_ADC_INJ_StopConversion( ADCx_u );
    LL_ADC_INJ_StopConversion( ADCx_v );
    LL_ADC_INJ_StopConversion( ADCx_w );
    pHandle->ADCRegularLocked=false;    
  }
  else
  {
    /* Nothing to do */
  }
}

/**
 * @brief  Enables the speed  loop (low frequency) timer
 * @param  pHandle: handler of the current instance of the Bemf_ADC component
 */
__weak void BADC_SpeedMeasureOn(Bemf_ADC_Handle_t *pHandle)
{
  LL_TIM_ClearFlag_UPDATE(pHandle->pParams_str->LfTim);
  LL_TIM_EnableIT_UPDATE(pHandle->pParams_str->LfTim);
}

/**
 * @brief  Disables the speed  loop (low frequency) timer
 * @param  pHandle: handler of the current instance of the Bemf_ADC component
 */
__weak void BADC_SpeedMeasureOff(Bemf_ADC_Handle_t *pHandle)
{
  LL_TIM_DisableIT_UPDATE(pHandle->pParams_str->LfTim);
}

/**
  * @brief  Configures the ADC for the current sampling.
  *         It sets the sampling point via TIM1_Ch4 value, the ADC sequence
  *         and channels.
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @param  pHandlePWMC: handler of the current instance of the PWMC component
  * @param  pHandleSTC: handler of the current instance of the Speed Control component
  */
__weak void BADC_SetSamplingPoint(Bemf_ADC_Handle_t *pHandle, PWMC_Handle_t *pHandlePWMC, SpeednTorqCtrl_Handle_t *pHandleSTC )
{
  if ((pHandleSTC->ModeDefault == MCM_SPEED_MODE) && (pHandle->DriveMode == VM))
  {
    if (!(pHandle->IsOnSensingEnabled) && (pHandlePWMC->CntPh > pHandle->OnSensingEnThres))
    {
      pHandle->IsOnSensingEnabled=true;
      pHandle->pSensing_Params = &(pHandle->Pwm_ON);
    }
    else if ((pHandle->IsOnSensingEnabled) && (pHandlePWMC->CntPh < pHandle->OnSensingDisThres))
    {
      pHandle->IsOnSensingEnabled=false;
      pHandle->pSensing_Params = &(pHandle->Pwm_OFF);
    }
    else
    {
    }
  }
  else
  {
    pHandle->IsOnSensingEnabled=false;
    pHandle->pSensing_Params = &(pHandle->Pwm_OFF);
  }	
  if (true == pHandle->pParams_str->gpio_divider_available)  LL_GPIO_ResetOutputPin( pHandle->pParams_str->bemf_divider_port, pHandle->pParams_str->bemf_divider_pin );
  PWMC_SetADCTriggerChannel( pHandlePWMC, pHandle->pSensing_Params->SamplingPoint);
}

/**
 * @brief  Gets last bemf value and checks for zero crossing detection.
 *         It updates speed loop timer and electrical angle accordingly.
 * @param  pHandle: handler of the current instance of the Bemf_ADC component
 * @param  pHandlePWMC: handler of the current instance of the PWMC component
 * @retval True if zero has been crossed, false otherwise
 */
__weak bool BADC_IsZcDetected( Bemf_ADC_Handle_t *pHandle, PWMC_Handle_t *pHandlePWMC)
{
  ADC_TypeDef * ADCx_u = pHandle->pParams_str->pAdc[0];
  ADC_TypeDef * ADCx_v = pHandle->pParams_str->pAdc[1];
  ADC_TypeDef * ADCx_w = pHandle->pParams_str->pAdc[2];
  uint16_t AdcValue;
  bool ZcDetection = false;
  pHandle->DemagCounter++;
  if ( pHandle->DemagCounter > pHandle->DemagCounterThreshold)
  {
    if (pHandle->ZcDetected == false)
    {
      switch(pHandlePWMC->Step)
      {
      case STEP_1:
        AdcValue = ADCx_w->JDR1;
        pHandle->BemfLastValues[2] = AdcValue;
        if(pHandle->Direction == 1)
        {
          if (AdcValue < pHandle->pSensing_Params->AdcThresholdDown)
          {
            ZcDetection = true;
            pHandle->MeasuredElAngle = - S16_60_PHASE_SHIFT / 2;
          }
        }
        else
        {
          if (AdcValue > pHandle->pSensing_Params->AdcThresholdUp)
          {
            ZcDetection = true;
            pHandle->MeasuredElAngle = S16_120_PHASE_SHIFT + S16_60_PHASE_SHIFT / 2;
          }
        }
        break;
      case STEP_2:
        AdcValue = ADCx_v->JDR1;
        pHandle->BemfLastValues[1] = AdcValue;
        if(pHandle->Direction == 1)
        {     
          if (AdcValue > pHandle->pSensing_Params->AdcThresholdUp)
          {
            ZcDetection = true;
            pHandle->MeasuredElAngle = S16_60_PHASE_SHIFT / 2; 
          }
        }
        else
        {
          if (AdcValue < pHandle->pSensing_Params->AdcThresholdDown)
          {
            ZcDetection = true;
            pHandle->MeasuredElAngle = - S16_120_PHASE_SHIFT - S16_60_PHASE_SHIFT / 2;         
          }
        }
        break;      
      case STEP_3:
        AdcValue = ADCx_u->JDR1;
        pHandle->BemfLastValues[0] = AdcValue;
        if(pHandle->Direction == 1)
        {
          if (AdcValue < pHandle->pSensing_Params->AdcThresholdDown)
          {
            ZcDetection = true;
            pHandle->MeasuredElAngle = S16_60_PHASE_SHIFT + S16_60_PHASE_SHIFT / 2;
          }
        }
        else
        {
          if (AdcValue > pHandle->pSensing_Params->AdcThresholdUp)
          {
            ZcDetection = true;
            pHandle->MeasuredElAngle = - S16_60_PHASE_SHIFT - S16_60_PHASE_SHIFT / 2;       
          }
        }
        break;
      case STEP_4:
        AdcValue = ADCx_w->JDR1;
        pHandle->BemfLastValues[2] = AdcValue;          
        if(pHandle->Direction == 1)
        {     
          if (AdcValue > pHandle->pSensing_Params->AdcThresholdUp)
          {
            ZcDetection = true;
            pHandle->MeasuredElAngle = S16_120_PHASE_SHIFT + S16_60_PHASE_SHIFT / 2;
          }
        }
        else
        {
          if (AdcValue < pHandle->pSensing_Params->AdcThresholdDown)
          {
            ZcDetection = true;
            pHandle->MeasuredElAngle = - S16_60_PHASE_SHIFT / 2;          
          }            
        }
        break;
      case STEP_5:
        AdcValue = ADCx_v->JDR1;
        pHandle->BemfLastValues[1] = AdcValue;   
        if(pHandle->Direction == 1)
        {
          if (AdcValue < pHandle->pSensing_Params->AdcThresholdDown)
          {
            ZcDetection = true;
            pHandle->MeasuredElAngle = - S16_120_PHASE_SHIFT - S16_60_PHASE_SHIFT / 2;
          }
        }
        else
        {
          if (AdcValue > pHandle->pSensing_Params->AdcThresholdUp)
          {
            ZcDetection = true;
            pHandle->MeasuredElAngle = S16_60_PHASE_SHIFT / 2;        
          }
        }
        break;
      case STEP_6:
        AdcValue = ADCx_u->JDR1;
        pHandle->BemfLastValues[0] = AdcValue;   
        if(pHandle->Direction == 1)
        {     
          if (AdcValue > pHandle->pSensing_Params->AdcThresholdUp)
          {
            ZcDetection = true;
            pHandle->MeasuredElAngle = - S16_60_PHASE_SHIFT - S16_60_PHASE_SHIFT / 2;
          }
        }
        else
        {
          if (AdcValue < pHandle->pSensing_Params->AdcThresholdDown)
          {
            ZcDetection = true;
            pHandle->MeasuredElAngle = S16_60_PHASE_SHIFT + S16_60_PHASE_SHIFT / 2;            
          }
        }
        break;
      }
      if (true == ZcDetection)
      {
        pHandle->MeasuredElAngle += (S16_60_PHASE_SHIFT/2) - (int16_t)((uint32_t)(pHandle->Zc2CommDelay * S16_60_PHASE_SHIFT)>>9);
        if (pHandle->ZcEvents > pHandle->StartUpConsistThreshold) 
        {
          pHandle->IsAlgorithmConverged = true;
        }		
        pHandle->ZcDetected = true;
        pHandle->ZcEvents++;
        switch(pHandlePWMC->Step)
        {
        case STEP_1:
        case STEP_3:
        case STEP_5:
          if(pHandle->Direction == 1)
          {     
            pHandle->ZC_Counter_Down = LL_TIM_GetCounter(pHandle->pParams_str->LfTim);
            pHandle->ZC_Counter_Last = pHandle->ZC_Counter_Down;
          }
          else
          {
            pHandle->ZC_Counter_Up = LL_TIM_GetCounter(pHandle->pParams_str->LfTim);
            pHandle->ZC_Counter_Last = pHandle->ZC_Counter_Up;
          }
          break;
        case STEP_2:
        case STEP_4:
        case STEP_6:
          if(pHandle->Direction == 1)
          {     
            pHandle->ZC_Counter_Up = LL_TIM_GetCounter(pHandle->pParams_str->LfTim);
            pHandle->ZC_Counter_Last = pHandle->ZC_Counter_Up;
          }
          else
          {
            pHandle->ZC_Counter_Down = LL_TIM_GetCounter(pHandle->pParams_str->LfTim);
            pHandle->ZC_Counter_Last = pHandle->ZC_Counter_Down;
          }
          break;
        }
        if (true == pHandle->IsAlgorithmConverged)
        {
          uint32_t tempReg = (uint32_t )(pHandle->PseudoPeriodConv / ((pHandle->LowFreqTimerPsc + 1) * (pHandle->AvrElSpeedDpp * pHandle->Direction)));		  
          LL_TIM_SetAutoReload(pHandle->pParams_str->LfTim,pHandle->ZC_Counter_Last + (((uint32_t)((pHandle->Zc2CommDelay) * tempReg)) >> 9) );
        }
      }
    }
  }
  return ZcDetection;
}

/**
  * @brief  This method must be called - at least - with the same periodicity
  *         on which speed control is executed. It computes and returns - through
  *         parameter hMecSpeedUnit - the rotor average mechanical speed,
  *         expressed in Unit. Average is computed considering a FIFO depth
  *         equal to SpeedBufferSizeUnit. Moreover it also computes and returns
  *         the reliability state of the sensor.
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @param  pMecSpeedUnit pointer to int16_t, used to return the rotor average
  *         mechanical speed (expressed in the unit defined by SPEED_UNIT)
  * @retval bool speed sensor reliability, measured with reference to parameters
  *         Reliability_hysteresys, VariancePercentage and SpeedBufferSize
  *         true = sensor information is reliable
  *         false = sensor information is not reliable
  */
__weak bool BADC_CalcAvrgMecSpeedUnit( Bemf_ADC_Handle_t * pHandle, int16_t * pMecSpeedUnit )
{
  bool bReliability;

  pHandle->_Super.hElSpeedDpp =  pHandle->AvrElSpeedDpp;
  if (0 ==  pHandle->AvrElSpeedDpp)
  {
    /* Speed is too low */
    *pMecSpeedUnit = 0;
  }
  else
  {
    /* Check if speed is not to fast */
    if (pHandle->AvrElSpeedDpp != MAX_PSEUDO_SPEED)
    {
      pHandle->DeltaAngle = pHandle->MeasuredElAngle - pHandle->_Super.hElAngle;
      pHandle->CompSpeed = (int16_t)((int32_t)(pHandle->DeltaAngle) / (int32_t)(pHandle->PWMNbrPSamplingFreq));

      /* Convert el_dpp to MecUnit */
      *pMecSpeedUnit = (int16_t)((pHandle->AvrElSpeedDpp * (int32_t)pHandle->_Super.hMeasurementFrequency
                       * (int32_t)SPEED_UNIT )
                       / ((int32_t)(pHandle->_Super.DPPConvFactor) * (int32_t)(pHandle->_Super.bElToMecRatio)) );
    }
    else
    {
      *pMecSpeedUnit = (int16_t)pHandle->SatSpeed;
    }
  }      
  bReliability = SPD_IsMecSpeedReliable(&pHandle->_Super, pMecSpeedUnit);

  pHandle->_Super.hAvrMecSpeedUnit = *pMecSpeedUnit;

  BADC_CalcAvrgElSpeedDpp (pHandle);
  return (bReliability);
}

/**
  * @brief  Returns false if calculated speed is out of reliability ranges
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @retval bool: reliability flag
  */
__weak bool BADC_IsSpeedReliable( Bemf_ADC_Handle_t *pHandle )
{
  bool SpeedError = false;

  if ( pHandle->_Super.hAvrMecSpeedUnit > pHandle->_Super.hMaxReliableMecSpeedUnit )
  {
    SpeedError = true;
  }

  if ( pHandle->_Super.hAvrMecSpeedUnit < pHandle->_Super.hMinReliableMecSpeedUnit )
  {
    SpeedError = true;
  }

  return ( SpeedError );
}

/**
  * @brief  Forces the rotation direction
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @param  direction: imposed direction
  */
__weak void BADC_SetDirection( Bemf_ADC_Handle_t * pHandle, uint8_t direction )
{
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
    pHandle->Direction = direction;
  }
}

/**
  * @brief  Internally performs a checks necessary to state whether
  *         the bemf algorithm converged. To be periodically called
  *         during motor open-loop ramp-up (e.g. at the same frequency of
  *         SPD_CalcElAngle), it returns true if the estimated angle and speed
  *         can be considered reliable, false otherwise
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  *
  * @retval bool sensor reliability state
  */
__weak bool BADC_IsObserverConverged( Bemf_ADC_Handle_t * pHandle )
{
  return pHandle->IsAlgorithmConverged;
}

/**
  * @brief  Configures the proper ADC channel according to the current 
  *         step corresponding to the floating phase. To be periodically called
  *         at least at every step change. 
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @param  Phase: Floating phase for bemf acquisition
  */
void BADC_SelectAdcChannel(Bemf_ADC_Handle_t * pHandle, uint8_t Phase)
{
  LL_ADC_INJ_SetSequencerRanks(pHandle->pParams_str->pAdc[Phase], LL_ADC_INJ_RANK_1, __LL_ADC_DECIMAL_NB_TO_CHANNEL(pHandle->pParams_str->AdcChannel[Phase]));
}

/**
  * @brief  Updates the estimated electrical angle.
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @retval int16_t rotor electrical angle (s16Degrees)
  */
__weak int16_t BADC_CalcElAngle(Bemf_ADC_Handle_t * pHandle)
{
  int16_t retValue;
    if (pHandle->_Super.hElSpeedDpp != MAX_PSEUDO_SPEED)
    {
      if (false == pHandle->IsLoopClosed)
      {
        pHandle->MeasuredElAngle += pHandle->_Super.hElSpeedDpp;
        pHandle->PrevRotorFreq = pHandle->_Super.hElSpeedDpp;
        pHandle->_Super.hElAngle += pHandle->_Super.hElSpeedDpp + pHandle->CompSpeed;
      }
      else
      {
        pHandle->_Super.hElAngle = pHandle->MeasuredElAngle;      
      }       
    }
    else
    {
      pHandle->_Super.hElAngle += pHandle->PrevRotorFreq;
    }
    retValue = pHandle->_Super.hElAngle;
  return (retValue);
}

/**
  * @brief  Must be called - at least - with the same periodicity
  *         on which speed control is executed. It computes and update component
  *         variable hElSpeedDpp that is estimated average electrical speed
  *         expressed in dpp used for instance in observer equations.
  *         Average is computed considering a FIFO depth equal to
  *         bSpeedBufferSizedpp.
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  */
void BADC_CalcAvrgElSpeedDpp( Bemf_ADC_Handle_t * pHandle )
{
  uint32_t wCaptBuf;
  
  /* used to validate the average speed measurement */
  if (pHandle->BufferFilled < pHandle->SpeedBufferSize)
  {
    pHandle->BufferFilled++;
  }
  else
  {
    /* Nothing to do */
  }
  if (false == pHandle->IsLoopClosed)
  {
    if (pHandle->VirtualElSpeedDpp == 0) pHandle->Counter_Period = 0xFFFF;
    else pHandle->Counter_Period = ( uint32_t )(pHandle->PseudoPeriodConv / ((pHandle->LowFreqTimerPsc + 1) * pHandle->VirtualElSpeedDpp));
  }
  else
  {
    pHandle->Counter_Period =  pHandle->ZC_Counter_Last + pHandle->Last_Zc2Comm_Delay;
  }
  wCaptBuf = pHandle->Counter_Period * (pHandle->LowFreqTimerPsc + 1);

  /* Filtering to fast speed... could be a glitch  ? */
  /* the MAX_PSEUDO_SPEED is temporary in the buffer, and never included in average computation*/
  if (wCaptBuf < pHandle->MinPeriod)
  {
    /* Nothing to do */
  }
  else
  {
    pHandle->ElPeriodSum -= pHandle->SpeedBufferDpp[pHandle->SpeedFIFOIdx]; /* value we gonna removed from the accumulator */
    if (wCaptBuf >= pHandle->MaxPeriod)
    {
      pHandle->SpeedBufferDpp[pHandle->SpeedFIFOIdx] = (int32_t)pHandle->MaxPeriod * pHandle->Direction;
    }
    else
    {
      pHandle->SpeedBufferDpp[pHandle->SpeedFIFOIdx] = (int32_t)wCaptBuf ;
      pHandle->SpeedBufferDpp[pHandle->SpeedFIFOIdx] *= pHandle->Direction;
      pHandle->ElPeriodSum += pHandle->SpeedBufferDpp[pHandle->SpeedFIFOIdx];
    }
    /* Update pointers to speed buffer */
    pHandle->SpeedFIFOIdx++;
    if (pHandle->SpeedFIFOIdx == pHandle->SpeedBufferSize)
    {
      pHandle->SpeedFIFOIdx = 0U;
    }
    if (((pHandle->BufferFilled < pHandle->SpeedBufferSize) && (wCaptBuf != 0U)) 
        || (false == pHandle->IsLoopClosed))
    {
      uint32_t tempReg = (pHandle->PseudoPeriodConv / wCaptBuf) * (uint32_t)pHandle->Direction;
      pHandle->AvrElSpeedDpp = (int16_t)tempReg;
    }
    else
    {
      /* Average speed allow to smooth the mechanical sensors misalignement */
      int32_t  tElPeriodSum = 0;
      uint8_t i;
      for (i=0; i < pHandle->SpeedBufferSize; i++)
      {
        tElPeriodSum += pHandle->SpeedBufferDpp[i];
      }             
      pHandle->AvrElSpeedDpp = (int16_t)((int32_t)pHandle->PseudoPeriodConv /
                                               (tElPeriodSum / (int32_t)pHandle->SpeedBufferSize)); /* Average value */
    }
  }
}

/**
  * @brief  Used to calculate instant speed during revup, to
  *         initialize parameters at step change and to select proper ADC channel
  *         for next Bemf acquisitions
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @param  hElSpeedDpp: Mechanical speed imposed by virtual speed component
  * @param  pHandlePWMC: handler of the current instance of the PWMC component
  */
void BADC_StepChangeEvent(Bemf_ADC_Handle_t * pHandle, int16_t hElSpeedDpp, PWMC_Handle_t *pHandlePWMC)
{
  pHandle->DemagCounter = 0;
  pHandle->Last_Zc2Comm_Delay = LL_TIM_GetAutoReload(pHandle->pParams_str->LfTim) - pHandle->ZC_Counter_Last;
  if (false == pHandle->IsLoopClosed)
  {
    if (hElSpeedDpp < 0)
    {
      pHandle->VirtualElSpeedDpp = - hElSpeedDpp;
    }
    else
    {
      pHandle->VirtualElSpeedDpp = hElSpeedDpp;
    }
    pHandle->ZcDetected = false;
  }
  else
  {
    int16_t ElAngleUpdate;
    if(pHandle->Direction == -1)
    {
      ElAngleUpdate = -S16_60_PHASE_SHIFT  ;
    }
    else
    {
      ElAngleUpdate = S16_60_PHASE_SHIFT  ;
    }
    pHandle->MeasuredElAngle += ElAngleUpdate;
    if ( false == pHandle->ZcDetected)
    {
      LL_TIM_SetAutoReload(pHandle->pParams_str->LfTim,LL_TIM_GetAutoReload(pHandle->pParams_str->LfTim) * 150 / 100);
    }
    else
    {
      pHandle->ZcDetected = false;
    }
  }
  pHandle->StepUpdate = true;
}

/**
  * @brief  Calculates and stores in the corresponding variable the demagnetization 
  *         time in open loop operation  
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  */
void BADC_CalcRevUpDemagTime(Bemf_ADC_Handle_t *pHandle)
{
  int16_t hSpeed;
  hSpeed = (int16_t)((pHandle->VirtualElSpeedDpp * (int32_t)pHandle->_Super.hMeasurementFrequency
                       * (int32_t)SPEED_UNIT )
                       / ((int32_t)(pHandle->_Super.DPPConvFactor) * (int32_t)(pHandle->_Super.bElToMecRatio)) );;
  if (hSpeed == 0)
  {
    pHandle->DemagCounterThreshold = pHandle->DemagParams.DemagMinimumThreshold;;
  }	  
  else
  {
    pHandle->DemagCounterThreshold = (uint16_t) (pHandle->DemagParams.RevUpDemagSpeedConv / hSpeed);		
  }
  if (pHandle->DemagCounterThreshold < pHandle->DemagParams.DemagMinimumThreshold)
  {
    pHandle->DemagCounterThreshold = pHandle->DemagParams.DemagMinimumThreshold;
  }  
}

/**
  * @brief  Calculates and stores in the corresponding variable the demagnetization 
  *         time in closed loop operation  
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  */
void BADC_CalcRunDemagTime(Bemf_ADC_Handle_t *pHandle)
{
  int16_t hSpeed;
  hSpeed = pHandle->_Super.hAvrMecSpeedUnit;
  if (hSpeed < 0) hSpeed = - hSpeed; 
  if (hSpeed < pHandle->DemagParams.DemagMinimumSpeedUnit)
  {   
    pHandle->DemagCounterThreshold = (uint16_t) (pHandle->DemagParams.RunDemagSpeedConv / hSpeed);
    if (pHandle->DemagCounterThreshold < pHandle->DemagParams.DemagMinimumThreshold)
    {
      pHandle->DemagCounterThreshold = pHandle->DemagParams.DemagMinimumThreshold;
    }   
  } 
  else
  {   
    pHandle->DemagCounterThreshold = pHandle->DemagParams.DemagMinimumThreshold;
  }
}

/**
  * @brief  Must be called after switch-over procedure when
  *         virtual speed sensor transition is ended.  
  * @param  pHandle: handler of the current instance of the STO component
  */
void BADC_SetLoopClosed(Bemf_ADC_Handle_t *pHandle)
{
  pHandle->IsLoopClosed=true;
}

/**
  * @brief  Returns the last acquired bemf value
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @param  phase: motor phase under investigation
  * @retval uint16_t: Bemf value
  */
uint16_t BADC_GetLastBemfValue(Bemf_ADC_Handle_t *pHandle, uint8_t phase)
{   
  return ((MC_NULL == pHandle) ? 0U : pHandle->BemfLastValues[phase]);
}

/**
  * @brief  Returns the zero crossing detection flag
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @retval bool: zero crossing detection flag
  */
bool BADC_GetBemfZcrFlag(Bemf_ADC_Handle_t *pHandle)
{   
  return ((MC_NULL == pHandle) ? 0U : pHandle->ZcDetected);
}  

/**
  * @brief  Returns true if the step needs to be updated
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @retval bool: step update request
  */
bool BADC_ClearStepUpdate(Bemf_ADC_Handle_t *pHandle)
{ 
  bool retValue = (((pHandle->IsLoopClosed == true) && (pHandle->StepUpdate == true)) || (pHandle->IsLoopClosed == false));
  pHandle->StepUpdate = false;
  return retValue;
}

/**
  * @brief  Configures the parameters for bemf sensing during pwm off-time
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @param  BemfAdcConfig: thresholds and sampling time parameters
  * @param  Zc2CommDelay: delay between bemf zero crossing parameter and step commutation
  * @param  bemfAdcDemagConfig: demagnetization parameters
  */
void BADC_SetBemfSensorlessParam(Bemf_ADC_Handle_t *pHandle, Bemf_Sensing_Params *BemfAdcConfig, uint16_t *Zc2CommDelay,
                                   Bemf_Demag_Params *bemfAdcDemagConfig)         
{  
  pHandle->Pwm_OFF.AdcThresholdUp = BemfAdcConfig->AdcThresholdUp;
  pHandle->Pwm_OFF.AdcThresholdDown = BemfAdcConfig->AdcThresholdDown;
  pHandle->Pwm_OFF.SamplingPoint = BemfAdcConfig->SamplingPoint;
  pHandle->Zc2CommDelay = *Zc2CommDelay;
  pHandle->DemagParams.DemagMinimumSpeedUnit = bemfAdcDemagConfig->DemagMinimumSpeedUnit;
  pHandle->DemagParams.DemagMinimumThreshold = bemfAdcDemagConfig->DemagMinimumThreshold;
}

/**
  * @brief  Configures the parameters for bemf sensing during pwm on-time
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @param  BemfOnAdcConfig: thresholds and sampling time parameters
  * @param  OnSensingEnThres: Minimum dudty cycle for on-sensing activation
  * @param  OnSensingDisThres: Minimum duty cycle for on-sensing Deactivationg
  */
void BADC_SetBemfOnTimeSensorlessParam(Bemf_ADC_Handle_t *pHandle, Bemf_Sensing_Params *BemfOnAdcConfig, uint16_t *OnSensingEnThres,
                                   uint16_t *OnSensingDisThres)
{  
  pHandle->Pwm_ON.AdcThresholdUp = BemfOnAdcConfig->AdcThresholdUp;
  pHandle->Pwm_ON.AdcThresholdDown = BemfOnAdcConfig->AdcThresholdDown;
  pHandle->Pwm_ON.SamplingPoint = BemfOnAdcConfig->SamplingPoint;
  pHandle->OnSensingEnThres = *OnSensingEnThres;
  pHandle->OnSensingDisThres = *OnSensingDisThres;
}

/**
  * @brief  Gets the parameters for bemf sensing during pwm off-time
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @param  BemfAdcConfig: thresholds and sampling time parameters
  * @param  Zc2CommDelay: delay between bemf zero crossing parameter and step commutation
  * @param  BemfAdcDemagConfig: demagnetization parameters
  */
void BADC_GetBemfSensorlessParam(Bemf_ADC_Handle_t *pHandle, Bemf_Sensing_Params *BemfAdcConfig, uint16_t *Zc2CommDelay,
                                   Bemf_Demag_Params *BemfAdcDemagConfig)         
{  
  BemfAdcConfig->AdcThresholdUp = pHandle->Pwm_OFF.AdcThresholdUp;
  BemfAdcConfig->AdcThresholdDown = pHandle->Pwm_OFF.AdcThresholdDown;
  BemfAdcConfig->SamplingPoint = pHandle->Pwm_OFF.SamplingPoint;
  *Zc2CommDelay = pHandle->Zc2CommDelay;
  BemfAdcDemagConfig->DemagMinimumSpeedUnit = pHandle->DemagParams.DemagMinimumSpeedUnit;
  BemfAdcDemagConfig->DemagMinimumThreshold = pHandle->DemagParams.DemagMinimumThreshold;
}

/**
  * @brief  Gets the parameters for bemf sensing during pwm on-time
  * @param  pHandle: handler of the current instance of the Bemf_ADC component
  * @param  BemfOnAdcConfig: thresholds and sampling time parameters
  * @param  OnSensingEnThres: Minimum duty cycle for on-sensing activation
  * @param  OnSensingDisThres: Minimum duty cycle for on-sensing Deactivationg
  */
void BADC_GetBemfOnTimeSensorlessParam(Bemf_ADC_Handle_t *pHandle, Bemf_Sensing_Params *BemfOnAdcConfig, uint16_t *OnSensingEnThres,
                                   uint16_t *OnSensingDisThres)
{  
  BemfOnAdcConfig->AdcThresholdUp = pHandle->Pwm_ON.AdcThresholdUp;
  BemfOnAdcConfig->AdcThresholdDown = pHandle->Pwm_ON.AdcThresholdDown;
  BemfOnAdcConfig->SamplingPoint = pHandle->Pwm_ON.SamplingPoint;
  *OnSensingEnThres = pHandle->OnSensingEnThres;
  *OnSensingDisThres = pHandle->OnSensingDisThres;
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
