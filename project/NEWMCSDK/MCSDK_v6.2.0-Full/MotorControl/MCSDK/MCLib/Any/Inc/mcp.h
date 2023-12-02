/**
  ******************************************************************************
  * @file    mcp.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the Motor control protocol
  *          of the Motor Control SDK.
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
  *
  */
/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef MOTOR_CONTROL_PROTOCOL_H
#define MOTOR_CONTROL_PROTOCOL_H

#include "mcptl.h"

#define MCP_VERSION 0x1U
/* Action suppoted by the Motor control protocol*/
#define CMD_MASK                         0xFFF8U
#define MCP_USER_CMD_MASK                0xFF00U

#define GET_MCP_VERSION                  0x0
#define SET_DATA_ELEMENT                 0x8
#define GET_DATA_ELEMENT                 0x10
#define START_MOTOR                      0x18
#define STOP_MOTOR                       0x20
#define STOP_RAMP                        0x28
#define START_STOP                       0x30
#define FAULT_ACK                        0x38
#define CPULOAD_CLEAR                    0x40
#define IQDREF_CLEAR                     0x48
#define PFC_ENABLE                       0x50
#define PFC_DISABLE                      0x58
#define PFC_FAULT_ACK                    0x60
#define PROFILER_CMD                     0x68
#define SW_RESET                         0x78
#define MCP_USER_CMD                     0x100U

/* MCP ERROR CODE */
#define MCP_CMD_OK                       0x00U
#define MCP_CMD_NOK                      0x01U
#define MCP_CMD_UNKNOWN                  0x02U
#define MCP_DATAID_UNKNOWN               0x03U
#define MCP_ERROR_RO_REG                 0x04U
#define MCP_ERROR_UNKNOWN_REG            0x05U
#define MCP_ERROR_STRING_FORMAT          0x06U
#define MCP_ERROR_BAD_DATA_TYPE          0x07U
#define MCP_ERROR_NO_TXSYNC_SPACE        0x08U
#define MCP_ERROR_NO_TXASYNC_SPACE       0x09U
#define MCP_ERROR_BAD_RAW_FORMAT         0x0AU
#define MCP_ERROR_WO_REG                 0x0BU
#define MCP_ERROR_REGISTER_ACCESS        0x0CU
#define MCP_ERROR_CALLBACK_NOT_REGISTRED 0x0DU

#define MCP_HEADER_SIZE                  2U

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MCP
  * @{
  */

/**
  * @brief  MCP User call back function pointer structure
  *
  * @param  rxLength : Length of the transmitted buffer
  * @param  *rxBuffer : Buffer of data transmitted by the MCP controller device
  * @param  txSyncFreeSpace : space available in txBuffer to send data back to the MCP controller
  * @param  *txLength : Actual size of data that will be transmitted to the MCP controller must be < txSyncFreeSpace
  * @param  *txBuffer : Data that will be transmitted to the MCP controller in response to the user command
  *
  * @retval MCP status
  */

typedef uint8_t (*MCP_user_cb_t)(uint16_t rxLength, uint8_t *rxBuffer, int16_t txSyncFreeSpace, uint16_t *txLength,
                                 uint8_t *txBuffer);

/**
  * @brief  Handle structure for MCP related components
  */
typedef struct
{
  MCTL_Handle_t *pTransportLayer;     /** Pointer to the MCTL structure */
  uint8_t *rxBuffer;                  /** Buffer of data transmitted by the MCP controller device */
  uint8_t *txBuffer;                  /** Data that will be transmitted to the MCP controller in response to the user command */
  uint16_t rxLength;                  /** Length of the transmitted buffer */
  uint16_t txLength;                  /** Actual size of data that will be transmitted to the MCP controller ; must be < txSyncFreeSpace*/

} MCP_Handle_t;

/* Parses the received packet and call the required function depending on the command sent by the controller device. */
void MCP_ReceivedPacket(MCP_Handle_t *pHandle);

/* Stores user's MCP functions to be acknowledged as MCP functions. */
uint8_t MCP_RegisterCallBack (uint8_t callBackID, MCP_user_cb_t fctCB);

#endif /* MOTOR_CONTROL_PROTOCOL_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
