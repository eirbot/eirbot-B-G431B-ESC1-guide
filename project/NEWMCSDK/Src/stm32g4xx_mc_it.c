
/**
  ******************************************************************************
  * @file    stm32g4xx_mc_it.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Main Interrupt Service Routines.
  *          This file provides exceptions handler and peripherals interrupt
  *          service routine related to Motor Control for the STM32G4 Family.
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
  * @ingroup STM32G4xx_IRQ_Handlers
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_config.h"
#include "mc_type.h"
//cstat -MISRAC2012-Rule-3.1
#include "mc_tasks.h"
//cstat +MISRAC2012-Rule-3.1
#include "motorcontrol.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx.h"
#include "mcp_config.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup STM32G4xx_IRQ_Handlers STM32G4xx IRQ Handlers
  * @{
  */

/* USER CODE BEGIN PRIVATE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define SYSTICK_DIVIDER (SYS_TICK_FREQUENCY/1000U)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USER CODE END PRIVATE */

/* Public prototypes of IRQ handlers called from assembly code ---------------*/
void ADC1_2_IRQHandler(void);
void TIMx_UP_M1_IRQHandler(void);
void TIMx_BRK_M1_IRQHandler(void);

void HardFault_Handler(void);
void SysTick_Handler(void);
void EXTI15_10_IRQHandler(void);

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  This function handles ADC1/ADC2 interrupt request.
  * @param  None
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */

    /* Clear Flags M1 */
    LL_ADC_ClearFlag_JEOS(ADC2);

  (void)TSK_HighFrequencyTask();

 /* USER CODE BEGIN HighFreq */

 /* USER CODE END HighFreq  */

 /* USER CODE BEGIN ADC1_2_IRQn 1 */

 /* USER CODE END ADC1_2_IRQn 1 */
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  This function handles first motor TIMx Update interrupt request.
  * @param  None
  */
void TIMx_UP_M1_IRQHandler(void)
{
 /* USER CODE BEGIN TIMx_UP_M1_IRQn 0 */

 /* USER CODE END  TIMx_UP_M1_IRQn 0 */

  LL_TIM_ClearFlag_UPDATE(TIM1);
  (void)R3_2_TIMx_UP_IRQHandler(&PWM_Handle_M1);
 /* USER CODE BEGIN TIMx_UP_M1_IRQn 1 */

 /* USER CODE END  TIMx_UP_M1_IRQn 1 */
}

void TIMx_BRK_M1_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_BRK_M1_IRQn 0 */

  /* USER CODE END TIMx_BRK_M1_IRQn 0 */
  if (0U == LL_TIM_IsActiveFlag_BRK(TIM1))
  {
    /* Nothing to do */
  }
  else
  {
    LL_TIM_ClearFlag_BRK(TIM1);
    PWMC_OCP_Handler(&PWM_Handle_M1._Super);

  }

  if (0U == LL_TIM_IsActiveFlag_BRK2(TIM1))
  {
    /* Nothing to do */
  }
  else
  {
    LL_TIM_ClearFlag_BRK2(TIM1);
    PWMC_OVP_Handler(&PWM_Handle_M1._Super, TIM1);

  }
  /* Systick is not executed due low priority so is necessary to call MC_Scheduler here */
  MC_Scheduler();

  /* USER CODE BEGIN TIMx_BRK_M1_IRQn 1 */

  /* USER CODE END TIMx_BRK_M1_IRQn 1 */
}

/**
  * @brief This function handles DMA_RX_A channel DMACH_RX_A global interrupt.
  */
//cstat !MISRAC2012-Rule-8.4
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQHandler 0 */

  /* USER CODE BEGIN DMA1_Channel1_IRQHandler 0 */

  /* Buffer is ready by the HW layer to be processed */
  if (0U == LL_DMA_IsActiveFlag_TC(DMA_RX_A, DMACH_RX_A))
  {
    /* Nothing to do */
  }
  else
  {
    LL_DMA_ClearFlag_TC (DMA_RX_A, DMACH_RX_A);
    ASPEP_HWDataReceivedIT (&aspepOverUartA);
  }
  /* USER CODE BEGIN DMA1_Channel1_IRQHandler 1 */

  /* USER CODE BEGIN DMA1_Channel1_IRQHandler 1 */

}

/* This section is present only when MCP over UART_A is used */
/**
  * @brief  This function handles USART interrupt request.
  * @param  None
  */
//cstat !MISRAC2012-Rule-8.4
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQHandler 0 */

  /* USER CODE END USART2_IRQHandler 0 */

  if (0U == LL_USART_IsActiveFlag_TC(USARTA))
  {
    /* Nothing to do */
  }
  else
  {
    /* LL_GPIO_SetOutputPin(GPIOC , LL_GPIO_PIN_6) */
    /* Disable the DMA channel to prepare the next chunck of data */
    LL_DMA_DisableChannel(DMA_TX_A, DMACH_TX_A);
    LL_USART_ClearFlag_TC (USARTA);
    /* Data Sent by UART */
    /* Need to free the buffer, and to check pending transfer */
    ASPEP_HWDataTransmittedIT (&aspepOverUartA);
    /* LL_GPIO_ResetOutputPin(GPIOC , LL_GPIO_PIN_6) */
  }

  uint32_t flags;
  uint32_t oreFlag;
  uint32_t feFlag;
  uint32_t neFlag;
  uint32_t errorMask;
  uint32_t activeIdleFlag;
  uint32_t isEnabledIdelFlag;
  oreFlag = LL_USART_IsActiveFlag_ORE(USARTA);
  feFlag = LL_USART_IsActiveFlag_FE(USARTA);
  neFlag = LL_USART_IsActiveFlag_NE(USARTA);
  errorMask = LL_USART_IsEnabledIT_ERROR(USARTA);

  flags = ((oreFlag | feFlag | neFlag) & errorMask);
  if (0U == flags)
  {
    /* Nothing to do */
  }
  else
  { /* Stopping the debugger will generate an OverRun error */
    WRITE_REG(USARTA->ICR, USART_ICR_FECF|USART_ICR_ORECF|USART_ICR_NECF);
    /* We disable ERROR interrupt to avoid to trig one Overrun IT per additional byte recevied */
    LL_USART_DisableIT_ERROR (USARTA);
    LL_USART_EnableIT_IDLE (USARTA);
  }

  activeIdleFlag = LL_USART_IsActiveFlag_IDLE (USARTA);
  isEnabledIdelFlag = LL_USART_IsEnabledIT_IDLE (USARTA);
  flags = activeIdleFlag & isEnabledIdelFlag;
  if (0U == flags)
  {
    /* Nothing to do */
  }
  else
  { /* Stopping the debugger will generate an OverRun error */
    LL_USART_DisableIT_IDLE (USARTA);
    /* Once the complete unexpected data are received, we enable back the error IT */
    LL_USART_EnableIT_ERROR (USARTA);
    /* To be sure we fetch the potential pendig data */
    /* We disable the DMA request, Read the dummy data, endable back the DMA request */
    LL_USART_DisableDMAReq_RX (USARTA);
    (void)LL_USART_ReceiveData8(USARTA);
    LL_USART_EnableDMAReq_RX (USARTA);
    ASPEP_HWDMAReset (&aspepOverUartA);
  }

  /* USER CODE BEGIN USART2_IRQHandler 1 */

  /* USER CODE END USART2_IRQHandler 1 */
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  */
void HardFault_Handler(void)
{
 /* USER CODE BEGIN HardFault_IRQn 0 */

 /* USER CODE END HardFault_IRQn 0 */
  TSK_HardwareFaultTask();

  /* Go to infinite loop when Hard Fault exception occurs */
  while (true)
  {

  }
 /* USER CODE BEGIN HardFault_IRQn 1 */

 /* USER CODE END HardFault_IRQn 1 */

}

void SysTick_Handler(void)
{

#ifdef MC_HAL_IS_USED
static uint8_t SystickDividerCounter = SYSTICK_DIVIDER;
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  if (SystickDividerCounter == SYSTICK_DIVIDER)
  {
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
    SystickDividerCounter = 0;
  }
  SystickDividerCounter ++;
#endif /* MC_HAL_IS_USED */

  /* USER CODE BEGIN SysTick_IRQn 1 */
  /* USER CODE END SysTick_IRQn 1 */
    MC_RunMotorControlTasks();

  /* USER CODE BEGIN SysTick_IRQn 2 */
  /* USER CODE END SysTick_IRQn 2 */
}

/**
  * @brief  This function handles Button IRQ on PIN PC10.
  */
void EXTI15_10_IRQHandler (void)
{
  /* USER CODE BEGIN START_STOP_BTN */
  if (0U == LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_10))
  {
    /* Nothing to do */
  }
  else
  {
    LL_EXTI_ClearFlag_0_31 (LL_EXTI_LINE_10);
    (void)UI_HandleStartStopButton_cb ();
  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */
/******************* (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
