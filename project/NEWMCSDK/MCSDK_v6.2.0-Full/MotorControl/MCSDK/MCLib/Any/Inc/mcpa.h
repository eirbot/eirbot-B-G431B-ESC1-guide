/**
  ******************************************************************************
  * @file    mcpa.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the Datalog
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
#ifndef MCPA_H
#define MCPA_H

#include "mcptl.h"

extern uint32_t GLOBAL_TIMESTAMP;

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MCP
  * @{
  */


/**
  * @brief  MCP asynchronous parameters handle.
  *
  * Defined differently depending on the UART used.
  *
  */
typedef struct
{
  MCTL_Handle_t *pTransportLayer;     /** Pointer to the transport layer structure, containing Performer capabilities. */
  void ** dataPtrTable;               /** Table of pointers to the value to be returned. */
  void ** dataPtrTableBuff;           /** Buffered version of dataPtrTable. */
  uint8_t *dataSizeTable;             /** Table containing the sizes of the values to be returned.*/
  uint8_t *dataSizeTableBuff;         /** Buffered version of dataSizeTable. */
  uint8_t *currentBuffer;             /** Current buffer allocated. */
  uint16_t bufferIndex;               /** Index of the position inside the bufer, a new buffer is allocated when bufferIndex = 0. */
  uint16_t bufferTxTrigger;           /** Threshold upon which data is dumped. */
  uint16_t bufferTxTriggerBuff;       /** Buffered version of bufferTxTrigger. */
#ifdef MCP_DEBUG_METRICS
  uint16_t bufferMissed;              /** Incremented each time a buffer is missed. Debug only. */
#endif
  uint8_t nbrOfDataLog;               /** Total number of values the performer is able to send at once. */
  uint8_t HFIndex;                    /** Incremental value going from 0 to HFRateBuff, data is dump when HFRateBuff is reached. Incremented every HFT. */
  uint8_t MFIndex;                    /** Incremental value going from 0 to MFRateBuff, data is dump when MFRateBuff is reached with an exception made for MFRateBuff == 254. Incremented every MFT.*/
  uint8_t HFRate;                     /** Rate at which HF data is dumped. 0 means every HFT, 1 means 1 in 2. */
  uint8_t HFRateBuff;                 /** Buffered version of HFRate. */
  uint8_t HFNum;                      /** Number of HF values to be returned. */
  uint8_t HFNumBuff;                  /** Buffered version of HFNum. */
  uint8_t MFRate;                     /** Rate at which MF data is dumped. 254 means once per buffer, 255 means MF data is not dumped. */
  uint8_t MFRateBuff;                 /** Buffered version of MFRate. */
  uint8_t MFNum;                      /** Number of MF values to be returned. */
  uint8_t MFNumBuff;                  /** Buffered version of MFNum. */
  uint8_t Mark;                       /** Configuration of the ASYNC communication. */
  uint8_t MarkBuff;                   /** Buffered version of Mark. */
} MCPA_Handle_t; /* MCP Async handle type */


void MCPA_dataLog(MCPA_Handle_t *pHandle);
uint8_t MCPA_cfgLog(MCPA_Handle_t *pHandle, uint8_t *cfgdata);
void MCPA_flushDataLog (MCPA_Handle_t *pHandle);

#endif /* MCPA_H */

/**
  * @}
  */

/**
  * @}
  */
/************************ (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
