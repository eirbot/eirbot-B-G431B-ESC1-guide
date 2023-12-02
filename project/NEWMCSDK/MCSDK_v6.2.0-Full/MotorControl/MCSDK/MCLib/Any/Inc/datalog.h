/**
  ******************************************************************************
  * @file    datalog.h
  * @author  Piak Electronic Design B.V.
  * @brief   This file is part of the Motor Control SDK.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 Piak Electronic Design B.V.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* datalog.h */

#ifndef __DATALOG_H
#define __DATALOG_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#define DATALOG_NUM_CHANNELS	(7)         /* Number of allocated channels */
#define DATALOG_SAMPLE_COUNT	(60)		/* Number of samples */ /* Max payload = 128 bytes */
#define DATALOG_SAMPLE_TYPE		int16_t		/* Type of the samples */


typedef enum _DATALOG_TRIGGER_STATE_e_
{
	DATALOG_TRIGGER_STATE_Idle,
	DATALOG_TRIGGER_STATE_Armed,
	DATALOG_TRIGGER_STATE_Running,
	DATALOG_TRIGGER_STATE_Holding,
} DATALOG_TRIGGER_STATE_e;

typedef enum _DATALOG_TRIGGER_EDGE_DIRECTION_e_
{
	DATALOG_TRIGGER_EDGE_DIRECTION_Rising,
	DATALOG_TRIGGER_EDGE_DIRECTION_Falling,
	DATALOG_TRIGGER_EDGE_DIRECTION_Both,
} DATALOG_TRIGGER_EDGE_DIRECTION_e;


typedef enum _DATALOG_TRIGGER_MODE_e_
{
	DATALOG_TRIGGER_MODE_Normal,		/* Log starts when triggered, hold when done */
	DATALOG_TRIGGER_MODE_Rolling,		/* Keep sampling to memory, do not hold */
} DATALOG_TRIGGER_MODE_e;


typedef struct _DATALOG_TRIGGER_s_
{
    // Trigger configuration and state
    int channel;
    DATALOG_SAMPLE_TYPE level;
    DATALOG_TRIGGER_MODE_e mode;
    DATALOG_TRIGGER_STATE_e state;
    DATALOG_TRIGGER_EDGE_DIRECTION_e edge;
    uint16_t preTrigger;
    bool justArmed;
} DATALOG_TRIGGER_s;


typedef struct _DATALOG_CHANNEL_s_
{
    bool bEnabled;
    void* pSource;
    DATALOG_SAMPLE_TYPE samples[DATALOG_SAMPLE_COUNT];
    DATALOG_SAMPLE_TYPE currentSample;
    DATALOG_SAMPLE_TYPE lastSample;
} DATALOG_CHANNEL_s;


typedef enum _DATALOG_STATE_e_
{
	DATALOG_STATE_Idle,				/* Not armed, will not start logging */
	DATALOG_STATE_Armed,			/* Armed, will start logging when triggered */
	DATALOG_STATE_Running,			/* Actively logging */
	DATALOG_STATE_Holding,			/* Holding, waiting for client to download data, then release */
} DATALOG_STATE_e;


typedef enum _DATALOG_COMMAND_e_
{
	DATALOG_COMMAND_None,			/* No command at present */
	DATALOG_COMMAND_Arm,			/* Release from Holding if holding, Arm trigger for next cycle */
	DATALOG_COMMAND_Stop,			/* stop datalog */
} DATALOG_COMMAND_e;


typedef struct _DATALOG_Obj_
{
    uint16_t 			nextSampleIdx;
    uint16_t 			endSampleIdx;
    uint16_t 			dataStartIdx;
    uint16_t 			triggerIdx;

    DATALOG_STATE_e 	state;
    DATALOG_COMMAND_e	command;
    DATALOG_TRIGGER_s 	trigger;
    DATALOG_CHANNEL_s 	channel[DATALOG_NUM_CHANNELS];

    uint16_t downsample;
    uint16_t downsamplecounter;

    /* Debug control */
    bool bClear;
    bool bArmTrigger;
    bool bClearHold;
} DATALOG_Obj;

typedef DATALOG_Obj* DATALOG_Handle;


DATALOG_Handle DATALOG_init(void* pMemory, const size_t numBytes);

void DATALOG_setup(DATALOG_Handle handle);

void DATALOG_run(DATALOG_Handle handle);

void DATALOG_setChannel(DATALOG_Handle handle, uint16_t channel, DATALOG_SAMPLE_TYPE* pSample);

void DATALOG_setCommand(DATALOG_Handle handle, const DATALOG_COMMAND_e command);

void DATALOG_getChannelBuffer(DATALOG_Handle handle, const int channel, DATALOG_SAMPLE_TYPE** ppBuffer, uint16_t* pSizeBytes);


#endif /* __DATALOG_H */

/* end of datalog.h */

/************************ (C) COPYRIGHT 2023 Piak Electronic Design B.V. *****END OF FILE****/
