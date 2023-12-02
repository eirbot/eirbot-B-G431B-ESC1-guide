
/**
  ******************************************************************************
  * @file    usart_aspep_driver.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the uart driver for the aspep protocol
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
  */

#include <stdint.h>
#include "mc_stm_types.h"
#include "usart_aspep_driver.h"

void UASPEP_DAMCONFIG_TX(UASPEP_Handle_t *pHandle);
void UASPEP_DAMCONFIG_RX(UASPEP_Handle_t *pHandle);

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MCP
  * @{
  */

/**
  * @brief  Initialization of the DMAs used for data transmission and reception.
  *
  * @param  pHandle Handler of the current instance of the UASPEP component
  */
void UASPEP_INIT(void *pHWHandle)
{
  UASPEP_Handle_t *pHandle = (UASPEP_Handle_t *)pHWHandle; //cstat !MISRAC2012-Rule-11.5
  UASPEP_DAMCONFIG_TX(pHandle);
  UASPEP_DAMCONFIG_RX(pHandle);
}

/**
  * @brief  Configures the DMA used for data transmission to controller.
  *
  * @param  pHandle Handler of the current instance of the UASPEP component
  */
void UASPEP_DAMCONFIG_TX(UASPEP_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_USA_ASP_DRV
  if (NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    /* Enable DMA UART */
    LL_USART_ClearFlag_TC(pHandle->USARTx);
    LL_USART_EnableIT_TC(pHandle->USARTx);

    /* Enable DMA UART to start the TX request */
    LL_USART_EnableDMAReq_TX(pHandle->USARTx);

    /* Write the USART_TDR register address in the DMA control register to configure it as
     * the destination of the transfer */
    //cstat !MISRAC2012-Rule-11.4
    LL_DMA_SetPeriphAddress(pHandle->txDMA, pHandle->txChannel, (uint32_t)&pHandle->USARTx->TDR);
    /* Clear UART ISR */
    LL_USART_ClearFlag_TC(pHandle->USARTx);

    /* DMA end of transfer on UART TX channel completion is not activated */
    /* We prefer to activate UART TC itself to avoid to trig IT while queued data are still to be transmitted */
#ifdef NULL_PTR_CHECK_USA_ASP_DRV
  }
#endif
}

/**
  * @brief  Configures the DMA used for data reception from controller.
  *
  * @param  pHandle Handler of the current instance of the UASPEP component
  */
void UASPEP_DAMCONFIG_RX(UASPEP_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_USA_ASP_DRV
  if (NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    /* Enable DMA end of transfer on UART RX channel completion */
    LL_DMA_EnableIT_TC(pHandle->rxDMA, pHandle->rxChannel);
    /* Enable Error interrupt (EIE) to unmask Overrun interrupt */
    LL_USART_EnableIT_ERROR(pHandle->USARTx);

    /* Write the USART_RDR register address in the DMA control register to configure it as
     * the source of the transfer */
    //cstat !MISRAC2012-Rule-11.4
    LL_DMA_SetPeriphAddress(pHandle->rxDMA, pHandle->rxChannel, (uint32_t)&pHandle->USARTx->RDR);

    /* Clear UART ISR */
    LL_USART_ClearFlag_TC(pHandle->USARTx);

    LL_USART_EnableDMAReq_RX(pHandle->USARTx);
#ifdef NULL_PTR_CHECK_USA_ASP_DRV
  }
#endif
}

/**
  * @brief  Enables the configured DMA to send packet.
  *
  * @param  pHWHandle Hardware components chosen for communication
  * @param  data Data to be transmitted to controller
  * @param  length Length of the data to be transmitted
  */
bool UASPEP_SEND_PACKET(void *pHWHandle, void *data, uint16_t length)
{
  UASPEP_Handle_t *pHandle = (UASPEP_Handle_t *)pHWHandle; //cstat !MISRAC2012-Rule-11.5
  bool result;
  if (0U == LL_DMA_IsEnabledChannel(pHandle->txDMA, pHandle->txChannel))
  {
    //cstat !MISRAC2012-Rule-11.4 !MISRAC2012-Rule-11.6
    LL_DMA_SetMemoryAddress(pHandle->txDMA, pHandle->txChannel, (uint32_t)data);
    LL_DMA_SetDataLength(pHandle->txDMA, pHandle->txChannel, length);
    LL_DMA_EnableChannel(pHandle->txDMA, pHandle->txChannel);
    result = true;
  }
  else
  {
    result = false;
  }
  return (result);
}

/**
  * @brief  Enables the configured DMA to receive packet.
  *
  * @param  pHWHandle Hardware components chosen for communication
  * @param  buffer Buffer which will receive the communicated data
  * @param  length Length of the received data
  */
void UASPEP_RECEIVE_BUFFER(void *pHWHandle, void* buffer, uint16_t length)
{
  UASPEP_Handle_t *pHandle = (UASPEP_Handle_t *)pHWHandle; //cstat !MISRAC2012-Rule-11.5
  LL_DMA_DisableChannel(pHandle->rxDMA, pHandle->rxChannel);
  //cstat !MISRAC2012-Rule-11.4 !MISRAC2012-Rule-11.6
  LL_DMA_SetMemoryAddress(pHandle->rxDMA, pHandle->rxChannel, (uint32_t)buffer);
  LL_DMA_SetDataLength(pHandle->rxDMA, pHandle->rxChannel, length);

  LL_DMA_EnableChannel(pHandle->rxDMA, pHandle->rxChannel);
}

/**
  * @brief  Sets IDLE state : no transmission on going.
  *
  * @param  pHandle Handler of the current instance of the UASPEP component
  */
void UASPEP_IDLE_ENABLE(void *pHWHandle)
{
  UASPEP_Handle_t *pHandle = (UASPEP_Handle_t *)pHWHandle; //cstat !MISRAC2012-Rule-11.5
  LL_USART_ClearFlag_IDLE(pHandle->USARTx);
  LL_USART_EnableIT_IDLE(pHandle->USARTx);
}

/**
  * @}
  */

/**
  * @}
  */
/************************ (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/

