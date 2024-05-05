
/**
  ******************************************************************************
  * @file    mcp.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the MCP protocol
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

#include "mc_type.h"
#include "mcp.h"
#include "register_interface.h"
#include "mc_config.h"
#include "mcp_config.h"
#include "mc_api.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup MCP Motor Control Protocol
  *
  * @brief Motor Control Protocol components of the Motor Control SDK.
  *
  * These components implement the features needed to drive and monitor motor control applications embedded in STM32 MCUs.
  * They mainly focus on the communication with the controller, both on the receiving and the transmitting end.
  *
  * @{
  */

/**
  * @brief  Parses the payload in the received packet and call the required function in order to modify a value.
  *
  * The function called depends on the targeted motor and/or targeted register : RI_SetRegisterGlobal or RI_SetRegisterMotorX.
  *
  * @param  pHandle Handler of the current instance of the MCP component
  * @param  txSyncFreeSpace Space available for synchronous transmission
  *
  * @retval Returns #MCP_CMD_OK if the command is acknowledged and #MCP_CMD_NOK if not.
  */
uint8_t RI_SetRegCommandParser (MCP_Handle_t * pHandle, uint16_t txSyncFreeSpace)
{
  uint8_t retVal = MCP_CMD_OK;
#ifdef NULL_PTR_CHECK_REG_INT
  if (MC_NULL == pHandle)
  {
    retVal = MCP_CMD_NOK;
  }
  else
  {
#endif
    uint16_t * dataElementID;
    uint8_t * rxData = pHandle->rxBuffer;
    uint8_t * txData = pHandle->txBuffer;
    int16_t rxLength = pHandle->rxLength;
    uint16_t size = 0U;
    uint8_t accessResult;

    uint16_t regID;
    uint8_t typeID;
    uint8_t motorID;
    uint8_t (*SetRegFcts[NBR_OF_MOTORS+1])(uint16_t, uint8_t, uint8_t*, uint16_t*, int16_t) = {&RI_SetRegisterGlobal, &RI_SetRegisterMotor1};
    uint8_t number_of_item =0;
    pHandle->txLength = 0;

    while (rxLength > 0)
    {
      number_of_item ++;
      dataElementID = (uint16_t *) rxData;
      rxLength = rxLength-MCP_ID_SIZE; // We consume 2 byte in the DataID
      rxData = rxData+MCP_ID_SIZE; // Shift buffer to the next data

      regID = *dataElementID & REG_MASK;
      typeID = (uint8_t)*dataElementID & TYPE_MASK;

      motorID = (uint8_t)((*dataElementID & MOTOR_MASK));

      if (motorID > NBR_OF_MOTORS)
      {
        retVal = MCP_CMD_NOK;
        rxLength = 0;
      }
      else
      {
        accessResult = SetRegFcts[motorID](regID, typeID, rxData, &size, rxLength);
        /* Prepare next data*/
        rxLength = (int16_t) (rxLength - size);
        rxData = rxData+size;
        /* If there is only one CMD in the buffer, we do not store the result */
        if ((1U == number_of_item) && (0 == rxLength))
        {
          retVal = accessResult;
        }
        else
        {/* Store the result for each access to be able to report failing access */
          if (txSyncFreeSpace !=0 )
          {
            *txData = accessResult;
            txData = txData+1;
            pHandle->txLength++;
            txSyncFreeSpace--; /* decrement one by one no wraparound possible */
            retVal = (accessResult != MCP_CMD_OK) ? MCP_CMD_NOK : retVal;
            if ((accessResult == MCP_ERROR_BAD_DATA_TYPE) || (accessResult == MCP_ERROR_BAD_RAW_FORMAT))
            { /* From this point we are not able to continue to decode CMD buffer*/
              /* We stop the parsing */
              rxLength = 0;
            }
          }
          else
          {
            /* Stop parsing the cmd buffer as no space to answer */
            /* If we reach this state, chances are high the command was badly formated or received */
            rxLength = 0;
            retVal = MCP_ERROR_NO_TXSYNC_SPACE;
          }
        }
      }
    }
    /* If all accesses are fine, just one global MCP_CMD_OK is required*/
    if (MCP_CMD_OK == retVal)
    {
      pHandle->txLength = 0;
    }
    else
    {
      /* Nothing to do */
    }
  #ifdef NULL_PTR_CHECK_REG_INT
  }
#endif
  return (retVal);
}

/**
  * @brief  Parses the payload in the received packet and call the required function in order to return a value.
  *
  * The function called depends on the targeted motor and/or targeted register : RI_GetRegisterGlobal or RI_GetRegisterMotorX.
  *
  * @param  pHandle Handler of the current instance of the MCP component
  * @param  txSyncFreeSpace Space available for synchronous transmission
  *
  * @retval Returns #MCP_CMD_OK if the command is acknowledged and #MCP_CMD_NOK if not.
  */
uint8_t RI_GetRegCommandParser (MCP_Handle_t * pHandle, uint16_t txSyncFreeSpace)
{
  uint8_t retVal = MCP_CMD_NOK;
#ifdef NULL_PTR_CHECK_REG_INT
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    uint16_t * dataElementID;
    uint8_t * rxData = pHandle->rxBuffer;
    uint8_t * txData = pHandle->txBuffer;
    uint16_t size = 0U;
    uint16_t rxLength = pHandle->rxLength;
    int16_t freeSpaceS16 = (int16_t) txSyncFreeSpace;

    uint16_t regID;
    uint8_t typeID;
    uint8_t motorID;
    uint8_t (*GetRegFcts[NBR_OF_MOTORS+1])(uint16_t, uint8_t, uint8_t*, uint16_t*, int16_t) = {&RI_GetRegisterGlobal, &RI_GetRegisterMotor1};
    pHandle->txLength = 0;
    while (rxLength > 0U)
    {
      dataElementID = (uint16_t *) rxData;
      rxLength = rxLength - MCP_ID_SIZE;
      rxData = rxData + MCP_ID_SIZE; // Shift buffer to the next MCP_ID

      regID = *dataElementID & REG_MASK;
      typeID = (uint8_t)*dataElementID & TYPE_MASK;

      motorID = (uint8_t)((*dataElementID & MOTOR_MASK));

      if (motorID > NBR_OF_MOTORS)
      {
        retVal = MCP_CMD_NOK;
        rxLength = 0;
      }
      else
      {
        retVal = GetRegFcts[motorID](regID, typeID, txData, &size, freeSpaceS16);
        if (retVal == MCP_CMD_OK )
        {
          /* Prepare next data */
          txData = txData+size;
          pHandle->txLength += size;
          freeSpaceS16 = freeSpaceS16-size;
        }
        else
        {
          rxLength = 0;
        }
      }
    }
#ifdef NULL_PTR_CHECK_REG_INT
  }
#endif
  return (retVal);
}

/**
  * @brief  Parses the header from the received packet and call the required function depending on the command sent by the controller device.
  *
  * @param  pHandle Handler of the current instance of the MCP component
  */
void MCP_ReceivedPacket(MCP_Handle_t *pHandle)
{
  const uint16_t *packetHeader;
  uint16_t command;
  int16_t txSyncFreeSpace;
  uint8_t motorID;
  uint8_t MCPResponse;
  uint8_t userCommand=0;

#ifdef NULL_PTR_CHECK_MCP
  if ((MC_NULL == pHandle) || (0U == pHandle->rxLength))
  {
    /* Nothing to do, txBuffer and txLength have not been modified */
  }
  else /* Length is 0, this is a request to send back the last packet */
  {
#endif
    packetHeader = (uint16_t *)pHandle->rxBuffer; //cstat !MISRAC2012-Rule-11.3
    command = (uint16_t)(*packetHeader & CMD_MASK);

    if ((command & MCP_USER_CMD_MASK) == MCP_USER_CMD)
    {
      userCommand = ((uint8_t)(command & 0xF8U) >> 3U);
      command = MCP_USER_CMD;
    }
    else
    {
      /* Nothing to do */
    }

    motorID = (uint8_t)((*packetHeader - 1U) & MOTOR_MASK);
    MCI_Handle_t *pMCI = &Mci[motorID];

    /* Removing MCP Header from RxBuffer */
    pHandle->rxLength = pHandle->rxLength - MCP_HEADER_SIZE;
    pHandle->rxBuffer = pHandle->rxBuffer + MCP_HEADER_SIZE;

    /* Commands requiering payload response must be aware of space available for the payload */
    /* Last byte is reserved for MCP response*/
    txSyncFreeSpace = (int16_t)pHandle->pTransportLayer->txSyncMaxPayload - 1;

    /* Initialization of the tx length, command which send back data has to increment the txLength
     * (case of Read register) */
    pHandle->txLength = 0U;

    switch (command)
    {
      case GET_MCP_VERSION:
      {
        pHandle->txLength = 4U;
        *pHandle->txBuffer = MCP_VERSION;
        MCPResponse = MCP_CMD_OK;
        break;
      }

      case SET_DATA_ELEMENT:
      {
        MCPResponse = RI_SetRegCommandParser(pHandle, (uint16_t)txSyncFreeSpace);
        break;
      }

      case GET_DATA_ELEMENT:
      {
        MCPResponse = RI_GetRegCommandParser(pHandle, (uint16_t)txSyncFreeSpace);
        break;
      }

      case START_MOTOR:
      {
        MCPResponse = (MCI_StartWithPolarizationMotor(pMCI) == false) ? MCP_CMD_OK : MCP_CMD_NOK;

        break;
      }

      case STOP_MOTOR: /* Todo: Check the pertinance of return value */
      {
        (void)MCI_StopMotor(pMCI);
        MCPResponse = MCP_CMD_OK;
        break;
      }

      case STOP_RAMP:
      {
        if (RUN == MCI_GetSTMState(pMCI))
        {
          MCI_StopRamp(pMCI);
        }
        else
        {
          /* Nothing to do */
        }
        MCPResponse = MCP_CMD_OK;
        break;
      }

      case START_STOP:
      {
        /* Queries the STM and a command start or stop depending on the state */
        if (IDLE == MCI_GetSTMState(pMCI))
        {

          MCPResponse = (MCI_StartWithPolarizationMotor(pMCI) == true) ? MCP_CMD_OK : MCP_CMD_NOK;

        }
        else
        {
          (void)MCI_StopMotor(pMCI);
          MCPResponse = MCP_CMD_OK;
        }
        break;
      }

      case FAULT_ACK:
      {
        (void)MCI_FaultAcknowledged(pMCI);
        MCPResponse = MCP_CMD_OK;
        break;
      }

      case IQDREF_CLEAR:
      {
        MCI_Clear_Iqdref(pMCI);
        MCPResponse = MCP_CMD_OK;
        break;
      }

      case PFC_ENABLE:
      case PFC_DISABLE:
      case PFC_FAULT_ACK:
      {
        MCPResponse = MCP_CMD_UNKNOWN;
        break;
      }

      case PROFILER_CMD:
      {
        MCPResponse = MC_ProfilerCommand(pHandle->rxLength, pHandle->rxBuffer, txSyncFreeSpace, &pHandle->txLength,
                                         pHandle->txBuffer);
        break;
      }

      case MCP_USER_CMD:
      {
        if ((userCommand < MCP_USER_CALLBACK_MAX) && (MCP_UserCallBack[userCommand] != NULL))
        {
          MCPResponse = MCP_UserCallBack[userCommand](pHandle->rxLength, pHandle->rxBuffer, txSyncFreeSpace,
                                                      &pHandle->txLength, pHandle->txBuffer);
        }
        else
        {
          MCPResponse = MCP_ERROR_CALLBACK_NOT_REGISTRED;
        }
        break;
      }

      default :
      {
        MCPResponse = MCP_CMD_UNKNOWN;
        break;
      }
    }
    pHandle->txBuffer[pHandle->txLength] = MCPResponse;
    pHandle->txLength++;
#ifdef NULL_PTR_CHECK_MCP
  }
#endif
}

/**
  * @brief  Stores user's MCP function to be later called as MCP function.
  *
  * @param  callBackID: ID used to get to the stored @p fctCB function
  * @param  fctCB: User call back function structure
  *
  * @retval Returns #MCP_CMD_OK if the command is acknowledged and #MCP_CMD_NOK if not
  */
uint8_t MCP_RegisterCallBack (uint8_t callBackID, MCP_user_cb_t fctCB)
{
  uint8_t result;

  if (callBackID < MCP_USER_CALLBACK_MAX)
  {
    MCP_UserCallBack[callBackID] = fctCB;
    result = MCP_CMD_OK;
  }
  else
  {
    result = MCP_CMD_NOK;
  }
  return (result);
}

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
