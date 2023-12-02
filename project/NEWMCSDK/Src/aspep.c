
/**
  ******************************************************************************
  * @file   aspep.c
  * @author Motor Control SDK Team, ST Microelectronics
  * @brief  This file provides firmware functions that implement the aspep protocol
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
#include "aspep.h"

/* Local definition */
#define MIN(a,b) ( ((a) < (b)) ? (a) : (b) )

/* Local functions */
static bool ASPEP_CheckBeacon (ASPEP_Handle_t *pHandle);
static uint8_t ASPEP_TXframeProcess(ASPEP_Handle_t *pHandle, uint8_t packetType, void *txBuffer, uint16_t bufferLength);
void ASPEP_sendBeacon(ASPEP_Handle_t *pHandle, ASPEP_Capabilities_def *capabilities);
void ASPEP_sendPing(ASPEP_Handle_t *pHandle, uint8_t state, uint16_t PacketNumber);

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MCP
  * @{
  */

#ifndef CRC4_PER_NIBBLE
/**
  * @brief CRC-4 lookup table with 256 entries
  *
  * Computes and checks the CRC on the header with the BYTE granularity.
  *
  * When computing the CRC, the 7th nibble must to be processed with the CRC_Lookup4
  * table. Indeed, the header contains 7 nibbles (28 bits) of data on which the CRC is
  * to be computed. Hence, the CRC4_Lookup8 table cannot be used for the last nibble.
  *
  * Using only this lookup table requires that the amount of input data is a multiple of
  * 8 bits.
  */
static uint8_t const CRC4_Lookup8[] =
{
  /*          0,    1,    2,    3,    4,    5,    6,    7,    8,    9,   A,     B,    C,    D,    E,    F */
  /* 0 */  0x00, 0x02, 0x04, 0x06, 0x08, 0x0a, 0x0c, 0x0e, 0x07, 0x05, 0x03, 0x01, 0x0f, 0x0d, 0x0b, 0x09,
  /* 1 */  0x07, 0x05, 0x03, 0x01, 0x0f, 0x0d, 0x0b, 0x09, 0x00, 0x02, 0x04, 0x06, 0x08, 0x0a, 0x0c, 0x0e,
  /* 2 */  0x0e, 0x0c, 0x0a, 0x08, 0x06, 0x04, 0x02, 0x00, 0x09, 0x0b, 0x0d, 0x0f, 0x01, 0x03, 0x05, 0x07,
  /* 3 */  0x09, 0x0b, 0x0d, 0x0f, 0x01, 0x03, 0x05, 0x07, 0x0e, 0x0c, 0x0a, 0x08, 0x06, 0x04, 0x02, 0x00,
  /* 4 */  0x0b, 0x09, 0x0f, 0x0d, 0x03, 0x01, 0x07, 0x05, 0x0c, 0x0e, 0x08, 0x0a, 0x04, 0x06, 0x00, 0x02,
  /* 5 */  0x0c, 0x0e, 0x08, 0x0a, 0x04, 0x06, 0x00, 0x02, 0x0b, 0x09, 0x0f, 0x0d, 0x03, 0x01, 0x07, 0x05,
  /* 6 */  0x05, 0x07, 0x01, 0x03, 0x0d, 0x0f, 0x09, 0x0b, 0x02, 0x00, 0x06, 0x04, 0x0a, 0x08, 0x0e, 0x0c,
  /* 7 */  0x02, 0x00, 0x06, 0x04, 0x0a, 0x08, 0x0e, 0x0c, 0x05, 0x07, 0x01, 0x03, 0x0d, 0x0f, 0x09, 0x0b,
  /* 8 */  0x01, 0x03, 0x05, 0x07, 0x09, 0x0b, 0x0d, 0x0f, 0x06, 0x04, 0x02, 0x00, 0x0e, 0x0c, 0x0a, 0x08,
  /* 9 */  0x06, 0x04, 0x02, 0x00, 0x0e, 0x0c, 0x0a, 0x08, 0x01, 0x03, 0x05, 0x07, 0x09, 0x0b, 0x0d, 0x0f,
  /* A */  0x0f, 0x0d, 0x0b, 0x09, 0x07, 0x05, 0x03, 0x01, 0x08, 0x0a, 0x0c, 0x0e, 0x00, 0x02, 0x04, 0x06,
  /* B */  0x08, 0x0a, 0x0c, 0x0e, 0x00, 0x02, 0x04, 0x06, 0x0f, 0x0d, 0x0b, 0x09, 0x07, 0x05, 0x03, 0x01,
  /* C */  0x0a, 0x08, 0x0e, 0x0c, 0x02, 0x00, 0x06, 0x04, 0x0d, 0x0f, 0x09, 0x0b, 0x05, 0x07, 0x01, 0x03,
  /* D */  0x0d, 0x0f, 0x09, 0x0b, 0x05, 0x07, 0x01, 0x03, 0x0a, 0x08, 0x0e, 0x0c, 0x02, 0x00, 0x06, 0x04,
  /* E */  0x04, 0x06, 0x00, 0x02, 0x0c, 0x0e, 0x08, 0x0a, 0x03, 0x01, 0x07, 0x05, 0x0b, 0x09, 0x0f, 0x0d,
  /* F */  0x03, 0x01, 0x07, 0x05, 0x0b, 0x09, 0x0f, 0x0d, 0x04, 0x06, 0x00, 0x02, 0x0c, 0x0e, 0x08, 0x0a
};

/**
  * @brief CRC-4 lookup table with 16 entries
  *
  *  Used to compute and check the CRC on the header with either the BYTE or NIBBLE granularity
  * In case the BYTE granularity is used, the table is used only with the 7th nibble of the header
  * (bits 24 to 27) when computing the CRC.
  */
  //cstat !MISRAC2012-Rule-8.9_a
static uint8_t const CRC4_Lookup4[] =
{
  /*   0,    1,    2,    3,    4,    5,    6,    7,    8,    9,   A,     B,    C,    D,    E,    F */
  0x00, 0x07, 0x0e, 0x09, 0x0b, 0x0c, 0x05, 0x02, 0x01, 0x06, 0x0f, 0x08, 0x0a, 0x0d, 0x04, 0x03
};
#endif /* CRC4_PER_NIBBLE */

/**
  * @brief Computes a 4-bit CRC on the 28 LSBs of @p header and returns it in the 4 MSB of the header
  *
  *  The generator polynomial used for the CRC is x^4+x+1 (ref. CCITT-G704).
  *
  *  The 28 input bits are split into 7 nibbles that are processed from the least significant to the
  * most significant one as follows:
  *
  *  - the least significant (4-bit) nibble is processed first as if it were the most significant part
  *    of the divident;
  *  - the order of bits in each nibble is unchanged for processing which would leads to the following
  *    bit processing sequence: 3, 2, 1, 0, 7, 6, 5, 4, 11, 10, 9, 8, 15, 14, 13, 12, 19, 18, 17, 16,
  *    23, 22, 21, 20, 27, 26, 25, 24.
  *
  *  Two lookup tables based implementations are proposed: one solely relying on a 16 entries lookup
  * table and another one that also uses a 256 entries lookup table. This last one is the default. It
  * requires 256 more bytes than the first but is also roughly two times faster.
  *
  *  The 16 entries lookup table based implementation is enabled when the CRC4_PER_NIBBLE preprocessor
  * flag is defined.
  *
  *  Note that the default, 256 entries lookup table based implementation also uses the 16 entries lookup
  * table because the amount of input data is not a multiple of 8 bits.
  *
  * The resulting CRC is written in bits 28 to 31 of @p header and the whole header is returned.
  */
static void ASPEP_ComputeHeaderCRC(uint32_t *headerPtr)
{
  uint8_t crc = 0;
  uint32_t header = *headerPtr;

#ifndef CRC4_PER_NIBBLE
  header &= 0x0fffffffU;

  crc = CRC4_Lookup8[crc ^ (uint8_t)(header         & 0xffU)];
  crc = CRC4_Lookup8[crc ^ (uint8_t)((header >> 8U) & 0xffU)];
  crc = CRC4_Lookup8[crc ^ (uint8_t)((header >> 16U) & 0xffU)];
  crc = CRC4_Lookup4[crc ^ (uint8_t)((header >> 24U) & 0x0fU)];
#else /* CRC4_PER_NIBBLE */
  crc = CRC4_Lookup4[crc ^ (uint8_t)(header         & 0xfU)];
  crc = CRC4_Lookup4[crc ^ (uint8_t)((header >>  4U) & 0xfU)];
  crc = CRC4_Lookup4[crc ^ (uint8_t)((header >>  8U) & 0xfU)];
  crc = CRC4_Lookup4[crc ^ (uint8_t)((header >> 12U) & 0xfU)];
  crc = CRC4_Lookup4[crc ^ (uint8_t)((header >> 16U) & 0xfU)];
  crc = CRC4_Lookup4[crc ^ (uint8_t)((header >> 20U) & 0xfU)];
  crc = CRC4_Lookup4[crc ^ (uint8_t)((header >> 24U) & 0xfU)];
#endif /* CRC4_PER_NIBBLE */

  *headerPtr |= (uint32_t)crc << 28;

}

/**
  * @brief Checks if @p header contains a valid 4-bit CRC and return true if this is the case and false otherwise
  *
  *  The CRC is computed over the 32 bits of the @p header. If the result is 0, the CRC is considered valid
  * and true is returned. If the result is not 0, false is returned.
  *
  * @sa ASPEP_ComputeHeaderCRC
  */
static bool ASPEP_CheckHeaderCRC(uint32_t header)
{
  uint8_t crc = 0;

#ifndef CRC4_PER_NIBBLE
  crc = CRC4_Lookup8[crc ^ (uint8_t)(header         & 0xffU)];
  crc = CRC4_Lookup8[crc ^ (uint8_t)((header >> 8U) & 0xffU)];
  crc = CRC4_Lookup8[crc ^ (uint8_t)((header >> 16U) & 0xffU)];
  crc = CRC4_Lookup8[crc ^ (uint8_t)((header >> 24U) & 0xffU)];
#else /* CRC4_PER_NIBBLE */
  crc = CRC4_Lookup4[crc ^ (uint8_t)(header         & 0xfU)];
  crc = CRC4_Lookup4[crc ^ (uint8_t)((header >>  4U) & 0xfU)];
  crc = CRC4_Lookup4[crc ^ (uint8_t)((header >>  8U) & 0xfU)];
  crc = CRC4_Lookup4[crc ^ (uint8_t)((header >> 12U) & 0xfU)];
  crc = CRC4_Lookup4[crc ^ (uint8_t)((header >> 16U) & 0xfU)];
  crc = CRC4_Lookup4[crc ^ (uint8_t)((header >> 20U) & 0xfU)];
  crc = CRC4_Lookup4[crc ^ (uint8_t)((header >> 24U) & 0xfU)];
  crc = crc ^ (uint8_t)((header >> 28) & 0xf);
#endif /* CRC4_PER_NIBBLE */

  return (crc == 0U);
}

/**
  * @brief  Starts ASPEP communication by configuring UART.
  *
  * @param  *pHandle Handler of the current instance of the ASPEP component
  */
void ASPEP_start(ASPEP_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_ASP
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->fASPEP_HWInit(pHandle->HWIp);
    pHandle->ASPEP_State = ASPEP_IDLE;
    pHandle->ASPEP_TL_State = WAITING_PACKET;
    pHandle->syncPacketCount = 0; /* Sync packet counter is reset only at startup*/

    /* Configure UART to receive first packet*/
    pHandle->fASPEP_receive(pHandle->HWIp, pHandle->rxHeader, ASPEP_HEADER_SIZE);
#ifdef NULL_PTR_CHECK_ASP
  }
#endif
}

/**
  * @brief  Sends BEACON to controller containing matched capabilities between controller and performer.
  *
  * @param  *pHandle Handler of the current instance of the ASPEP component
  * @param  *capabilities Matched capabilities between controller and performer
  */
void ASPEP_sendBeacon(ASPEP_Handle_t *pHandle, ASPEP_Capabilities_def *capabilities)
{
#ifdef NULL_PTR_CHECK_ASP
  if ((MC_NULL == pHandle) || (NULL == capabilities))
  {
    /* Nothing to do */
  }
  else
  {
#endif
    uint32_t *packet = (uint32_t *)pHandle->ctrlBuffer.buffer; //cstat !MISRAC2012-Rule-11.3
    *packet = (BEACON
             | (((uint32_t)capabilities->version) << 4U)
             | (((uint32_t)capabilities->DATA_CRC) << 7U)
             | (((uint32_t)capabilities->RX_maxSize) << 8U)
             | (((uint32_t)capabilities->TXS_maxSize) << 14U)
             | (((uint32_t)capabilities->TXA_maxSize) << 21U));

    (void)ASPEP_TXframeProcess(pHandle, ASPEP_CTRL, &pHandle->ctrlBuffer, ASPEP_CTRL_SIZE);
#ifdef NULL_PTR_CHECK_ASP
  }
#endif
}

/**
  * @brief  Sends back to controller a NOT_ACKNOWLEDGED message.
  *
  * @param  *pHandle Handler of the current instance of the ASPEP component
  * @param  errorInfo Information on the encountered error
  */
static void ASPEP_sendNack(ASPEP_Handle_t *pHandle, uint8_t errorInfo)
{
  uint32_t *packet = (uint32_t *)pHandle->ctrlBuffer.buffer; //cstat !MISRAC2012-Rule-11.3
  *packet = NACK | (uint32_t)(((uint32_t) errorInfo) << 8U) | (uint32_t)(((uint32_t) errorInfo) << 16U);
  (void)ASPEP_TXframeProcess(pHandle, ASPEP_CTRL, &pHandle->ctrlBuffer, ASPEP_CTRL_SIZE);
}

/**
  * @brief  Sends back ping to controller once connection has been established.
  *
  * @param  *pHandle Handler of the current instance of the ASPEP component
  * @param  cBit Set to 1 if Performer has not been reset during communication
  * @param  packetNumber Number of requested packet incremented throughout the entire communication process, reset when communication is aborted or lost
  */
void ASPEP_sendPing(ASPEP_Handle_t *pHandle, uint8_t cBit, uint16_t packetNumber)
{
#ifdef NULL_PTR_CHECK_ASP
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    uint32_t *packet = (uint32_t *)pHandle->ctrlBuffer.buffer; //cstat !MISRAC2012-Rule-11.3
    uint8_t Nbit = pHandle->syncPacketCount & 0x1U; /* Keep only LSB */
    uint8_t ipID = pHandle->liid & 0xFU;
    *packet = PING | (uint32_t)((uint32_t)cBit << 4U)
                   | (uint32_t)((uint32_t)cBit << 5U)
                   | (uint32_t)((uint32_t)Nbit << 6U)
                   | (uint32_t)((uint32_t)Nbit << 7U)
                   | (uint32_t)((uint32_t)ipID << 8U)
                   | (uint32_t)(((uint32_t) packetNumber) << 12U);
    (void)ASPEP_TXframeProcess(pHandle, ASPEP_CTRL, &pHandle->ctrlBuffer, ASPEP_CTRL_SIZE);
#ifdef NULL_PTR_CHECK_ASP
  }
#endif
}

/**
  * @brief  Assigns new buffer depending on buffer type request and prevents overwriting.
  *
  * @param  *pSupHandle Handler of the current instance of the MCTL component
  * @param  **buffer Pointer to the buffer to be assigned
  * @param  syncAsync Type of requested buffer, can be synchronous or asynchronous
  *
  * @return Returns true if there is a writable buffer. False otherwise.
  */
bool ASPEP_getBuffer(MCTL_Handle_t *pSupHandle, void **buffer,  uint8_t syncAsync)
{
  bool result = true;
#ifdef NULL_PTR_CHECK_ASP
  if (MC_NULL == buffer)
  {
    result = false;
  }
  else
  {
#endif
    ASPEP_Handle_t *pHandle = (ASPEP_Handle_t *)pSupHandle; //cstat !MISRAC2012-Rule-11.3

    if (MCTL_SYNC == syncAsync)
    {
      if (pHandle->syncBuffer.state <= writeLock) /* Possible values are free or writeLock*/
      {
        *buffer = &pHandle->syncBuffer.buffer[ASPEP_HEADER_SIZE];
        pHandle->syncBuffer.state = writeLock;
      }
      else
      {
        result = false;
      }
    }
    else /* Asynchronous buffer request */
    {
      if ((pHandle->asyncBufferA.state > writeLock) && (pHandle->asyncBufferB.state > writeLock))
      {
        result = false;
      }
      else
      {
        if (pHandle->asyncBufferA.state <= writeLock)
        {
          pHandle->asyncBufferA.state = writeLock;
          pHandle->lastRequestedAsyncBuff = &pHandle->asyncBufferA;
          *buffer = &pHandle->asyncBufferA.buffer[ASPEP_HEADER_SIZE];
#ifdef MCP_DEBUG_METRICS
          pHandle->asyncBufferA.RequestedNumber++;
#endif
        }
        else if (pHandle->asyncBufferB.state <= writeLock)
        {
          pHandle->asyncBufferB.state = writeLock;
          pHandle->lastRequestedAsyncBuff = &pHandle->asyncBufferB;
          *buffer = &pHandle->asyncBufferB.buffer[ASPEP_HEADER_SIZE];
#ifdef MCP_DEBUG_METRICS
          pHandle->asyncBufferB.RequestedNumber++;
#endif
        }
        else
        {
          /* Nothing to do */
        }
      }
    }
#ifdef NULL_PTR_CHECK_ASP
  }
#endif
  return (result);
}

/**
  * @brief  Checks controller's capabilities and compare them to performer's capabilities.
  *
  * @param  *pHandle Handler of the current instance of the ASPEP component
  *
  * @return Returns true if controller's capabilities match performer's one. False otherwise.
  */
bool ASPEP_CheckBeacon (ASPEP_Handle_t *pHandle)
{
  bool result = true;

  uint32_t packetHeader = *((uint32_t *)pHandle->rxHeader); //cstat !MISRAC2012-Rule-11.3
  ASPEP_Capabilities_def MasterCapabilities;
  MasterCapabilities.version = (uint8_t)((packetHeader &0x70U)>> 4U);           /*Bits 4 to 6*/
  MasterCapabilities.DATA_CRC = pHandle->rxHeader[0] >> 7U ;                     /*Bit 7 */
  MasterCapabilities.RX_maxSize = pHandle->rxHeader[1] &0x3FU;                  /*Bits 8 to  13*/
  MasterCapabilities.TXS_maxSize = (uint8_t)((packetHeader&0x01FC000U)  >> 14); /*Bits 14 to 20 */
  MasterCapabilities.TXA_maxSize = (uint8_t)((packetHeader&0xFE00000U) >> 21);  /*Bits 21 to 27  */

  pHandle->Capabilities.DATA_CRC = MIN(pHandle->Capabilities.DATA_CRC ,MasterCapabilities.DATA_CRC);
  pHandle->Capabilities.RX_maxSize = MIN(pHandle->Capabilities.RX_maxSize, MasterCapabilities.RX_maxSize);
  pHandle->Capabilities.TXS_maxSize = MIN(pHandle->Capabilities.TXS_maxSize, MasterCapabilities.TXS_maxSize);
  pHandle->Capabilities.TXA_maxSize = MIN(pHandle->Capabilities.TXA_maxSize, MasterCapabilities.TXA_maxSize);

  if ((MasterCapabilities.DATA_CRC != pHandle->Capabilities.DATA_CRC)
   /* Data packet the controller can send is bigger than performer can receive */
   || (MasterCapabilities.RX_maxSize > pHandle->Capabilities.RX_maxSize)
   /* Sync packet size alignement is required in order for the controller to be able to store it, and to not request a
    * response bigger than performer capability */
   || (pHandle->Capabilities.TXS_maxSize != MasterCapabilities.TXS_maxSize )
   /* Async packet the performer can send is bigger than the controller can receive (but controller must not request a bigger
    * packet than performer can transmit) */
   || (pHandle->Capabilities.TXA_maxSize != MasterCapabilities.TXA_maxSize )
   /* For the time being, controller version has to match performer version */
   || (MasterCapabilities.version != pHandle->Capabilities.version) )
  {
    result = false;
  }
  else
  {
    /* Nothing to do */
  }

  return (result);
}

/**
  * @brief  Checks if any error has occurred and calls the ASPEP_TXframeProcess function.
  *
  * @param  *pSupHandle Handler of the current instance of the MCTL component
  * @param  *txBuffer can be 8, 16 or 32 bits, but must be 32 bits aligned
  * @param  txDataLength Size of useful data in byte
  * @param  syncAsync nature of the communication : synchronous or asynchronous
  *
  * @return Returns an ASPEP response defined in aspep.h
  */
uint8_t ASPEP_sendPacket(MCTL_Handle_t *pSupHandle, void *txBuffer, uint16_t txDataLength, uint8_t syncAsync)
{
  uint8_t result = ASPEP_OK;
#ifdef NULL_PTR_CHECK_ASP
  if ((MC_NULL == pSupHandle) || (MC_NULL == txBuffer))
  {
    result = ASPEP_BUFFER_ERROR;
  }
  else
  {
#endif
    uint32_t *header;
    uint32_t tmpHeader;
    uint16_t txDataLengthTemp;
    uint8_t *packet;

    txDataLengthTemp = txDataLength;
    ASPEP_Handle_t *pHandle = (ASPEP_Handle_t *)pSupHandle; //cstat !MISRAC2012-Rule-11.3

    if (ASPEP_CONNECTED == pHandle-> ASPEP_State)
    {
      /*We must add packet header on  */
      /* | [0101|0011] | Length 13b | Reserved |CRCH 4b| */
      packet = (uint8_t *)txBuffer; //cstat !MISRAC2012-Rule-11.5
      header = (uint32_t *)txBuffer; //cstat !MISRAC2012-Rule-11.5
      header--; /* Header ues 4*8 bits on top of txBuffer*/
      tmpHeader = ((uint32_t)((uint32_t)txDataLengthTemp << (uint32_t)4) | (uint32_t)syncAsync);
      *header = tmpHeader;
      if (1U == pHandle->Capabilities.DATA_CRC)
      {
        /* TODO : Compute real CRC*/
        packet[txDataLengthTemp] = (uint8_t)0xCA; /* Dummy CRC */
        packet[txDataLengthTemp + 1U] = (uint8_t)0xFE; /* Dummy CRC */
        txDataLengthTemp += (uint16_t)ASPEP_DATACRC_SIZE;
      }
      if (MCTL_SYNC == syncAsync)
      {
        if (pSupHandle->MCP_PacketAvailable)
        {
          pSupHandle->MCP_PacketAvailable = false; /* CMD from controller is processed*/
        }
        else
        {
          result = MCTL_SYNC_NOT_EXPECTED;
        }
      }
      if (ASPEP_OK == result) /* Send packet only if no error detected so far*/
      {
        result = ASPEP_TXframeProcess(pHandle, syncAsync, header, txDataLengthTemp + (uint16_t)ASPEP_HEADER_SIZE);
      }
    }
    else
    {
      result = ASPEP_NOT_CONNECTED;
    }
#ifdef NULL_PTR_CHECK_ASP
  }
#endif
  return (result);
}

/**
  * @brief  Checks if HW resource is busy before sending packet. Otherwise saves packet for next opportunity to send.
  *
  * This function contains a critical section.
  * It can be accessed concurently under High frequency task (by MCPA_datalog)
  * and under Medium frequency task (MC_Scheduler -> ASPEP_RxFrameProcess ).
  *
  * @param  *pHandle Handler of the current instance of the ASPEP component
  * @param  dataType Nature of the communication : synchronous, asynchronous or a CTL packet
  * @param  *txBuffer CRC Header to be computed in sent packet
  * @param  bufferLength Size of the packet to be sent : Header + Data
  *
  * @return Returns an ASPEP response defined in aspep.h
  */
uint8_t ASPEP_TXframeProcess(ASPEP_Handle_t *pHandle, uint8_t dataType, void *txBuffer, uint16_t bufferLength)
{
  uint8_t result = ASPEP_OK;
#ifdef NULL_PTR_CHECK_ASP
  if (MC_NULL == txBuffer)
  {
    result = ASPEP_BUFFER_ERROR;
  }
  else
  {
#endif
    /* Insert CRC header in the packet to send */
    ASPEP_ComputeHeaderCRC((uint32_t *)txBuffer); //cstat !MISRAC2012-Rule-11.5
    __disable_irq(); /*TODO: Disable High frequency task is enough */
    if (NULL == pHandle->lockBuffer) /* Communication Ip free to send data*/
    {
      if (MCTL_ASYNC == dataType)
      {
        /* In ASYNC, two flipflop buffers are used, the txBuffer points always to lastRequestedAsyncBuff->buffer */
        pHandle->lastRequestedAsyncBuff->state = readLock;
        pHandle->lockBuffer = (void *)pHandle->lastRequestedAsyncBuff;
#ifdef MCP_DEBUG_METRICS
        pHandle->lastRequestedAsyncBuff->SentNumber++;
#endif
      }
      else if (MCTL_SYNC == dataType)
      {
        pHandle->syncBuffer.state = readLock;
        pHandle->lockBuffer = (void *)&pHandle->syncBuffer;
      }
      else
      {
        pHandle->ctrlBuffer.state = readLock;
        pHandle->lockBuffer = (void *)&pHandle->ctrlBuffer;
      }
      /* Enable HF task It */
      __enable_irq(); /*TODO: Enable High frequency task is enough */
      pHandle->fASPEP_send(pHandle->HWIp, txBuffer, bufferLength);
    }
    else /* HW resource busy, saving packet to sent it once resource will be freed*/
    {
      __enable_irq(); /*TODO: Enable High frequency task is enough */
      /* Lock buffer can be freed here */
      if (MCTL_ASYNC == dataType)
      {
        /* Check that the buffer received is the one expected - probably useless */
        if (txBuffer != (uint8_t *)pHandle->lastRequestedAsyncBuff->buffer)
        {
          result = ASPEP_BUFFER_ERROR;
        }
        else
        {
          /* Nothing to do */
        }
        if (NULL == pHandle->asyncNextBuffer)
        {
          /* Required to keep the right sending order */
          pHandle->asyncNextBuffer = pHandle->lastRequestedAsyncBuff;
        }
        else
        {
          /* Nothing to do */
        }
        pHandle->lastRequestedAsyncBuff->state = pending;
        pHandle->lastRequestedAsyncBuff->length = bufferLength;
#ifdef MCP_DEBUG_METRICS
        pHandle->lastRequestedAsyncBuff->PendingNumber++;
#endif
      }
      else if (MCTL_SYNC == dataType)
      {
        if (pHandle -> syncBuffer.state != writeLock)
        {
          result = ASPEP_BUFFER_ERROR;
        }
        else
        {
          pHandle->syncBuffer.state = pending;
          pHandle->syncBuffer.length = bufferLength;
        }
      }
      else if(ASPEP_CTRL == dataType)
      {
        if (pHandle->ctrlBuffer.state != available)
        {
          result = ASPEP_BUFFER_ERROR;
        }
        else
        {
          pHandle->ctrlBuffer.state = pending;
        }
      }
      else
      {
        /* Nothing to do */
      }
    }
#ifdef NULL_PTR_CHECK_ASP
  }
#endif
  return (result);
}

/**
  * @brief  Frees previously locked buffer and/or locks the next pending buffer. Once locked, sends the buffer.
  *
  * Called as soon as previous packet transfer is completed, pHandle->lockBuffer is set before packet transmission and ensures packet won't be read anymore.
  * Therefore, there is no need to protect this ISR against another higher priority ISR (HF Task).
  *
  * @param  *pHandle Handler of the current instance of the ASPEP component
  */
void ASPEP_HWDataTransmittedIT(ASPEP_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_ASP
  if (NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    /* First free previous readLock buffer */
    if (pHandle->ctrlBuffer.state == readLock)
    {
      pHandle->ctrlBuffer.state = available;
    }
    else /* if previous buffer was not ASPEP_CTRL, then the buffer locked is a MCTL_Buff_t */
    {
      MCTL_Buff_t *tempBuff = (MCTL_Buff_t *)pHandle->lockBuffer; //cstat !MISRAC2012-Rule-11.5
      tempBuff->state = available;
    }
    if (pHandle->syncBuffer.state == pending)
    {
      pHandle->lockBuffer = (void *)&pHandle->syncBuffer;
      pHandle->fASPEP_send(pHandle->HWIp, pHandle->syncBuffer.buffer, pHandle->syncBuffer.length);
      pHandle->syncBuffer.state = readLock;
    }
    /* Second prepare transfer of pending buffer */
    else if (pHandle->ctrlBuffer.state == pending)
    {
      pHandle->lockBuffer = (void *)(&pHandle ->ctrlBuffer);
      pHandle->fASPEP_send(pHandle ->HWIp, pHandle->ctrlBuffer.buffer, ASPEP_CTRL_SIZE);
      pHandle->ctrlBuffer.state = readLock;
    }
    else
    {
      __disable_irq();
      if (pHandle->asyncNextBuffer != NULL)
      {
        pHandle->lockBuffer = (void *)pHandle->asyncNextBuffer;
        pHandle->asyncNextBuffer->state = readLock;
#ifdef MCP_DEBUG_METRICS
        pHandle->asyncNextBuffer->SentNumber++;
#endif
        pHandle->fASPEP_send(pHandle ->HWIp, pHandle->asyncNextBuffer->buffer, pHandle->asyncNextBuffer->length);
        /* If one Async buffer is still pending, assign it to the asyncNextBuffer pointer*/
        if ((pHandle->asyncBufferA.state == pending) || (pHandle->asyncBufferB.state == pending))
        {
          //cstat !MISRAC2012-Rule-11.4
          uint32_t temp = (uint32_t)&pHandle->asyncBufferA + (uint32_t)&pHandle->asyncBufferB
                          - (uint32_t) pHandle->asyncNextBuffer; //cstat !MISRAC2012-Rule-11.4
          pHandle->asyncNextBuffer = (MCTL_Buff_t *) temp; //cstat !MISRAC2012-Rule-11.4
        }
        else
        {
          pHandle->asyncNextBuffer = NULL;
        }
      }
      else /* No TX packet are pending, HW resource is free*/
      {
        pHandle->lockBuffer = NULL;
      }
      __enable_irq();
    }
#ifdef NULL_PTR_CHECK_ASP
  }
#endif
}

/**
  * @brief  Updates ASPEP state depending on received packet from Controller and sends a response according to ASPEP protocol
  *
  * ASPEP protocol defined in section 4.4.1 Connection Procedure of Motor Control Protocol Suite of User Manual
  *
  * @param  *pSupHandle Handler of the current instance of the MCTL component
  * @param  *packetLength Length of the packet to be processed
  *
  * @return Returns the buffer of data transmitted by the MCP controller device
  */
uint8_t *ASPEP_RXframeProcess(MCTL_Handle_t *pSupHandle, uint16_t *packetLength)
{
  uint8_t *result = NULL; //cstat !MISRAC2012-Rule-8.13
#ifdef NULL_PTR_CHECK_ASP
  if ((NULL == pSupHandle) || (NULL == packetLength))
  {
    /* Nothing to do */
  }
  else
  {
#endif
    ASPEP_Handle_t *pHandle = (ASPEP_Handle_t *)pSupHandle; //cstat !MISRAC2012-Rule-11.3
    uint32_t packetHeader = *((uint32_t *)pHandle->rxHeader); //cstat !MISRAC2012-Rule-11.3
    uint16_t packetNumber;
#if VALID_CRC_DATA
    bool validCRCData = true;
#endif
    *packetLength = 0;
    if (pHandle->NewPacketAvailable)
    {
      pHandle->NewPacketAvailable = false; /* Consumes new packet*/
      switch (pHandle->ASPEP_State)
      {
        case ASPEP_IDLE:
        {
          if (BEACON == pHandle->rxPacketType)
          {
            if (ASPEP_CheckBeacon(pHandle) == true)
            {
              /* Controller capabilities match performer capabilities.*/
              pSupHandle->txSyncMaxPayload = (pHandle->Capabilities.TXS_maxSize + (uint16_t)1U) * (uint16_t)32U;
              pSupHandle->txAsyncMaxPayload = (pHandle->Capabilities.TXA_maxSize) * (uint16_t)64U;
              pHandle->maxRXPayload = (pHandle->Capabilities.RX_maxSize + (uint16_t)1U) * (uint16_t)32U;
              pHandle->ASPEP_State = ASPEP_CONFIGURED;
            }
            else
            {
              /* Nothing to do, controller has to send back new beacon with matching capabilities */
            }
            /* Beacon Packet must be answered*/
            ASPEP_sendBeacon(pHandle, &pHandle->Capabilities);
          }
          else if (PING == pHandle->rxPacketType)
          {
            /* In Listening for controller performer, */
            packetNumber = (uint16_t)((packetHeader & 0x0FFFF000U) >> (uint16_t)12U);
            ASPEP_sendPing(pHandle, ASPEP_PING_RESET, packetNumber);
          }
          else
          {
            /* Nothing to do */
          }
          break;
        }

        case ASPEP_CONFIGURED:
        {
          if (BEACON == pHandle->rxPacketType)
          {
            if (ASPEP_CheckBeacon(pHandle) == false)
            {
              pHandle->ASPEP_State = ASPEP_IDLE;
            }
            else
            {
              /* Nothing to do */
            }

            ASPEP_sendBeacon (pHandle, &pHandle->Capabilities);
          }
          else if (PING == pHandle->rxPacketType)
          {
            /* In Listening for controller performer, */
            packetNumber = (uint16_t)((packetHeader & 0x0FFFF000U) >> (uint16_t)12U);
            ASPEP_sendPing(pHandle, ASPEP_PING_CFG, packetNumber);
            pHandle->ASPEP_State = ASPEP_CONNECTED;
          }
          else
          {
            /* Nothing to do */
          }
          break;
        }

        case ASPEP_CONNECTED:
        {
          if (BEACON == pHandle->rxPacketType)
          {
            if (ASPEP_CheckBeacon(pHandle) == false)
            {
              pHandle->ASPEP_State = ASPEP_IDLE;
            }
            else
            {
              pHandle->ASPEP_State = ASPEP_CONFIGURED;
            }
            ASPEP_sendBeacon(pHandle, &pHandle->Capabilities);
          }
          else if (PING == pHandle->rxPacketType)
          {
            packetNumber = pHandle->rxHeader[1];
            ASPEP_sendPing(pHandle, ASPEP_PING_CFG, packetNumber);
          }
          else if (DATA_PACKET == pHandle->rxPacketType)
          {
#if VALID_CRC_DATA
            if (validCRCData)
            {
#endif
              pHandle->syncPacketCount++; /* this counter is incremented at each valid data packet received from controller */
              pSupHandle->MCP_PacketAvailable = true; /* Will be consumed in ASPEP_sendPacket */
              *packetLength = pHandle->rxLength;
              result = pHandle->rxBuffer;
#if VALID_CRC_DATA
            }
            else
            {
              ASPEP_sendNack (pHandle, ASPEP_BAD_CRC_DATA);
            }
#endif
          }
          else
          {
            /* This condition is not reachable because already filtred by NewPacketAvailable */
            /* ASPEP_sendNack (pHandle, ASPEP_BAD_PACKET_TYPE) */
          }
          break;
        }

        default:
          break;
      }
      /* The valid received packet is now safely consumes, we are ready to receive a new packet */
      pHandle->fASPEP_receive(pHandle->HWIp, pHandle->rxHeader, ASPEP_HEADER_SIZE);
    }
    else if (pHandle->badPacketFlag > ASPEP_OK)
    {
      ASPEP_sendNack(pHandle, pHandle->badPacketFlag);
      /* ASPEP_RXframeProcess can be called before reception of another packet */
      pHandle->badPacketFlag = ASPEP_OK;
      /* As we received a packet with a bad header, we need to be sure that the HW IP is well Synchronised
        * DMA will be configured to receive next packet as soon as HW IP RX line is free to receive new packet
        * It is important to note that we will detect only the NEXT free line transition, it means the next packet will
        * be lost but the end of this lost packet will generate the IDLE interrupt
        * the IDLE interrupt will call ASPEP_HWDMAReset (in charge of the IP_aspep driver to call it at the appropriate
        * time)*/
      pHandle->fASPEP_HWSync(pHandle->HWIp);
    }
    else
    {
      /* Nothing to do, no response is due to the controller */
    }
#ifdef NULL_PTR_CHECK_ASP
  }
#endif
  return (result);
}

/**
  * @brief  Processes the received data packet.
  *
  * This function is called once DMA has transfered the configure number of byte.
  * Upon reception of a new packet the DMA will be re-configured only once the answer has been sent.
  * This is mandatory to avoid a race condition in case of a new packet is received while executing ASPEP_RXframeProcess.
  * If the packet received contains an error in the header, the HW IP will be re-synchronised first, and DMA will be configured after.
  *
  * @param  *pHandle Handler of the current instance of the ASPEP component
  */
void ASPEP_HWDataReceivedIT(ASPEP_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_ASP
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    switch (pHandle->ASPEP_TL_State)
    {
      case WAITING_PACKET:
      {
        if (ASPEP_CheckHeaderCRC(*(uint32_t *)pHandle->rxHeader) == true) //cstat !MISRAC2012-Rule-11.3
        {
          pHandle->rxPacketType = (ASPEP_packetType)(((uint32_t)pHandle->rxHeader[0]) & ID_MASK);
          switch (pHandle->rxPacketType)
          {
            case DATA_PACKET:
            {
              //cstat !MISRAC2012-Rule-11.3
              pHandle->rxLength = (uint16_t)((*((uint16_t *)pHandle->rxHeader) & 0x1FFF0U) >> (uint16_t)4);
              if (0U == pHandle->rxLength) /* data packet with length 0 is a valid packet */
              {
                pHandle->NewPacketAvailable = true;
                /* The receiver is not reconfigure right now on purpose to avoid race condition when the packet will be
                  *  processed in ASPEP_RXframeProcess */
              }
              else if (pHandle->rxLength <= pHandle->maxRXPayload)
              {
                pHandle->fASPEP_receive(pHandle->HWIp, pHandle->rxBuffer,  /* need to read + 2 bytes CRC*/
                                        (pHandle->rxLength + ((uint16_t)ASPEP_DATACRC_SIZE * (uint16_t)pHandle->Capabilities.DATA_CRC)));
                pHandle->ASPEP_TL_State = WAITING_PAYLOAD;
              }
              else
              {
                pHandle->badPacketFlag = ASPEP_BAD_PACKET_SIZE;
              }
              break;
            }

            case BEACON:
            case PING:
            {
              pHandle->NewPacketAvailable = true;
              /* The receiver is not reconfigure right now on purpose to avoid race condition when the packet will be
                * processed in ASPEP_RXframeProcess */
              break;
            }

            default:
            {
              pHandle->badPacketFlag = ASPEP_BAD_PACKET_TYPE;
              break;
            }
          }
        }
        else
        {
          pHandle->badPacketFlag = ASPEP_BAD_CRC_HEADER;
        }
        break;
      }

      case WAITING_PAYLOAD:
      {
        pHandle->ASPEP_TL_State = WAITING_PACKET;
        /* Payload received, */
        pHandle->NewPacketAvailable = true;
        /* The receiver is not reconfigure right now on purpose to avoid race condition when the packet will be
          * processed in ASPEP_RXframeProcess */
        break;
      }

      default:
        break;
    }
#ifdef NULL_PTR_CHECK_ASP
  }
#endif
}

/**
  * @brief  Resets DMA after debugger has stopped the MCU.
  *
  * @param  *pHandle Handler of the current instance of the ASPEP component
  */
void ASPEP_HWDMAReset(ASPEP_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_ASP
  if (NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    /* We must reset the RX state machine to be sure to not be in Waiting packet state */
    /* Otherwise the arrival of a new packet will trigger a NewPacketAvailable despite */
    /* the fact that bytes have been lost because of overrun (debugger paused for instance) */
    pHandle->ASPEP_TL_State = WAITING_PACKET;
    pHandle->fASPEP_receive(pHandle->HWIp, pHandle->rxHeader, ASPEP_HEADER_SIZE);
#ifdef NULL_PTR_CHECK_ASP
  }
#endif
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
