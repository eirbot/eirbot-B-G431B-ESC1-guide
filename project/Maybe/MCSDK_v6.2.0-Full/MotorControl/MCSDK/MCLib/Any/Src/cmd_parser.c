/**
  ******************************************************************************
  * @file    register_interface.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the register access for the MCP protocol
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
#include "stdint.h"
#include "string.h"
#include "register_interface.h"
#include "mc_config.h"
#include "mc_parameters.h"
#include "mcp.h"
#include "mcp_config.h"
#include "mcpa.h"
#include "mc_configuration_registers.h"

//#include "dac_ui.h"

__weak uint8_t RI_SetRegCommandParser (MCP_Handle_t * pHandle, uint16_t txSyncFreeSpace)
{
  uint16_t * dataElementID;
  uint8_t * rxData = pHandle->rxBuffer;
  uint8_t * txData = pHandle->txBuffer;
  int16_t rxLength = pHandle->rxLength;
  uint16_t size;
  uint8_t retVal=MCP_CMD_OK;
  uint8_t accessResult;
  uint8_t number_of_item =0;
  pHandle->txLength = 0;
  while (rxLength > 0)
  {
     number_of_item ++;
     dataElementID = (uint16_t *) rxData;
     rxLength = rxLength-MCP_ID_SIZE; // We consume 2 byte in the DataID
     rxData = rxData+MCP_ID_SIZE; // Shift buffer to the next data
     accessResult = RI_SetReg (*dataElementID,rxData,&size,rxLength);

     /* Prepare next data*/
     rxLength = (int16_t) (rxLength - size);
     rxData = rxData+size;
     /* If there is only one CMD in the buffer, we do not store the result */
     if (number_of_item == 1 && rxLength == 0)
     {
       retVal = accessResult;
     }
     else
     {/* Store the result for each access to be able to report failling access */
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
  /* If all accesses are fine, just one global MCP_CMD_OK is required*/
  if (retVal == MCP_CMD_OK)
  {
    pHandle->txLength = 0;
  }
  return retVal;
}

__weak uint8_t RI_GetRegCommandParser (MCP_Handle_t * pHandle, uint16_t txSyncFreeSpace)
{
  uint16_t * dataElementID;
  uint8_t * rxData = pHandle->rxBuffer;
  uint8_t * txData = pHandle->txBuffer;
  uint16_t size = 0;
  uint16_t rxLength = pHandle->rxLength;
  int16_t freeSpaceS16 = (int16_t) txSyncFreeSpace;
  uint8_t retVal = MCP_CMD_NOK;
  pHandle->txLength = 0;
  while (rxLength > 0)
  {
     dataElementID = (uint16_t *) rxData;
     rxLength = rxLength-MCP_ID_SIZE;
     rxData = rxData+MCP_ID_SIZE; // Shift buffer to the next MCP_ID
     retVal = RI_GetReg (*dataElementID,txData, &size, freeSpaceS16);
     if (retVal == MCP_CMD_OK )
     {
       txData = txData+size;
       pHandle->txLength += size;
       freeSpaceS16 = freeSpaceS16-size;
     }
     else
     {
       rxLength = 0;
     }
  }
  return retVal;
}

