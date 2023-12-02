/**
  ******************************************************************************
  * @file    datalog.c
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
/* datalog.c */

#include "datalog.h"
#include <string.h> // memset

bool DATALOG_checkTrigger(DATALOG_Handle handle);
void DATALOG_clearChannelData(DATALOG_Handle handle, int channel);
void DATALOG_setSampleWindow(DATALOG_Handle handle);



DATALOG_Handle DATALOG_init(void* pMemory, const size_t numBytes)
{
	DATALOG_Handle handle = (DATALOG_Handle) NULL;

    if (numBytes >= sizeof(DATALOG_Obj)) /* Size was mismatched once - Are dependancies to .h files being checked correctly? */
    {
        handle = (DATALOG_Handle) pMemory;

        memset(pMemory, 0, numBytes);
    }

    return (handle);
} /* end of DATALOG_init() function */


void DATALOG_setup(DATALOG_Handle handle)
{
	DATALOG_Obj* obj = handle;

    int channel;
    for (channel = 0; channel < DATALOG_NUM_CHANNELS; channel++)
    {
    	DATALOG_CHANNEL_s* pChannel = &handle->channel[channel];

        pChannel->bEnabled = false;
        pChannel->pSource = (void*) NULL;
        DATALOG_clearChannelData(handle, channel);
    }

    obj->nextSampleIdx = 0;
    obj->endSampleIdx = 0;
    obj->dataStartIdx = 0;
    obj->triggerIdx = 0;

    obj->bClear = false;
    obj->bArmTrigger = false;
    obj->bClearHold = false;

    obj->trigger.channel = 0;
    obj->trigger.level = 16384;
    obj->trigger.mode = DATALOG_TRIGGER_MODE_Normal;
    obj->trigger.state = DATALOG_TRIGGER_STATE_Idle;
    obj->trigger.edge = DATALOG_TRIGGER_EDGE_DIRECTION_Rising;
    obj->trigger.preTrigger = 1;

    obj->state = DATALOG_STATE_Idle;
    obj->command = DATALOG_COMMAND_None;

    obj->downsample = 3;
    obj->downsamplecounter = 0;
} /* end of DATALOG_setup() function */


void DATALOG_run(DATALOG_Handle handle)
{
    DATALOG_Obj* obj = handle;

    /* Debug control of scope */

    if (obj->bClearHold)
    {
        if (obj->trigger.state == DATALOG_TRIGGER_STATE_Holding)
        {
            obj->trigger.state = DATALOG_TRIGGER_STATE_Idle;
        }
        obj->bClearHold = false;
    }

    if (obj->bArmTrigger)
    {
        obj->trigger.state = DATALOG_TRIGGER_STATE_Armed;
        obj->channel[obj->trigger.channel].lastSample = 0;
        obj->channel[obj->trigger.channel].currentSample = 0;
        obj->bArmTrigger = false;
        obj->trigger.justArmed = true;
    }

    if (obj->bClear)
    {
        int channel;
        for (channel = 0; channel < DATALOG_NUM_CHANNELS; channel++)
        {
            DATALOG_clearChannelData(handle, channel);
        }
        obj->bClear = false;
    }

	switch (obj->command)
	{
	case DATALOG_COMMAND_None:
		break;

	case DATALOG_COMMAND_Arm:
		// Only when holding or idle /* Whatever the current state, reset and re-arm the trigger */

		if (obj->trigger.state == DATALOG_TRIGGER_STATE_Idle || obj->trigger.state == DATALOG_TRIGGER_STATE_Holding)
		{
			obj->trigger.state = DATALOG_TRIGGER_STATE_Armed;
			obj->channel[obj->trigger.channel].lastSample = 0;
			obj->channel[obj->trigger.channel].currentSample = 0;
			obj->state = DATALOG_STATE_Armed;
			obj->trigger.justArmed = true;
		}
		break;

	case DATALOG_COMMAND_Stop:
		break;
	}

	/* Clear the command */
	obj->command = DATALOG_COMMAND_None;

    /* Scope functionality */

    if (obj->trigger.mode == DATALOG_TRIGGER_MODE_Normal)
    {
        uint16_t nextSampleIdx = obj->nextSampleIdx;

        DATALOG_TRIGGER_STATE_e triggerState = obj->trigger.state;

        if (triggerState == DATALOG_TRIGGER_STATE_Holding)
        {
            // No capture during holding
            return;
        }

        if (triggerState == DATALOG_TRIGGER_STATE_Armed)
        {
        	/* While armed, we need to update the triggering channel values, or we won't see the trigger */
        	DATALOG_CHANNEL_s* pChannel = &obj->channel[obj->trigger.channel];
			if (pChannel->bEnabled)
			{
				DATALOG_SAMPLE_TYPE sample = *((DATALOG_SAMPLE_TYPE*) pChannel->pSource);
				pChannel->lastSample = pChannel->currentSample;
				pChannel->currentSample = sample;
			}

			// Check for trigger
            if (DATALOG_checkTrigger(handle))
            {
#ifdef SUPPORT_PRETRIGGER_AND_WRAPPED_DATA
            	/* Can only be used if the client supports unwrapping the data */
                // Trigger fired, determine end of log
                DATALOG_setSampleWindow(handle);
                obj->triggerIdx = nextSampleIdx;
                obj->trigger.state = DATALOG_TRIGGER_STATE_Running;
                obj->state = DATALOG_STATE_Running;
#else /* SUPPORT_PRETRIGGER_AND_WRAPPED_DATA */
                /* Simple capture, from start to end, no wrapping */
                obj->nextSampleIdx = 0;
                obj->endSampleIdx = 0; // DATALOG_SAMPLE_COUNT;
                obj->dataStartIdx = 0;
                obj->triggerIdx = 0;
                obj->trigger.state = DATALOG_TRIGGER_STATE_Running;
                obj->state = DATALOG_STATE_Running;
#endif /* SUPPORT_PRETRIGGER_AND_WRAPPED_DATA */
            }
        }

        if (obj->trigger.state == DATALOG_TRIGGER_STATE_Running)
        {
			if (obj->downsamplecounter >= obj->downsample)
			{
				obj->downsamplecounter = 0;

				/* This is the actual sampling */
				{
					int channel;
					for (channel = 0; channel < DATALOG_NUM_CHANNELS; channel++)
					{
						DATALOG_CHANNEL_s* pChannel = &obj->channel[channel];

						if (pChannel->bEnabled)
						{
							DATALOG_SAMPLE_TYPE sample = *((DATALOG_SAMPLE_TYPE*) pChannel->pSource);
							pChannel->lastSample = pChannel->currentSample;
							pChannel->currentSample = sample;
							pChannel->samples[nextSampleIdx] = sample;
						}
					}

					nextSampleIdx++;
					if (nextSampleIdx >= DATALOG_SAMPLE_COUNT) nextSampleIdx = 0;
					obj->nextSampleIdx = nextSampleIdx;
				}

				/* Detect end of sampling */
	            if (obj->nextSampleIdx == obj->endSampleIdx)
	            {
	                obj->trigger.state = DATALOG_TRIGGER_STATE_Holding;
	                obj->state = DATALOG_STATE_Holding;
	            }

			}
			else
			{
				obj->downsamplecounter++;
			}
        }

    }
    else if (obj->trigger.mode == DATALOG_TRIGGER_MODE_Rolling)
    {
        /* Basic continous rolling mode, for debugging use */
        uint16_t nextSampleIdx = obj->nextSampleIdx;
        int channel;
        for (channel = 0; channel < DATALOG_NUM_CHANNELS; channel++)
        {
            DATALOG_CHANNEL_s* pChannel = &obj->channel[channel];

            if (pChannel->bEnabled)
            {
            	DATALOG_SAMPLE_TYPE sample = *((DATALOG_SAMPLE_TYPE*) pChannel->pSource);
                pChannel->lastSample = pChannel->currentSample;
                pChannel->currentSample = sample;
                pChannel->samples[nextSampleIdx] = sample;
            }
        }

        nextSampleIdx++;

        if (nextSampleIdx >= DATALOG_SAMPLE_COUNT)
        {
            nextSampleIdx = 0;
        }

        obj->nextSampleIdx = nextSampleIdx;
    }
} /* end of DATALOG_run() function */





bool DATALOG_checkTrigger(DATALOG_Handle handle)
{
    DATALOG_Obj* obj = handle;
    bool bTriggered = false;

    if (obj->trigger.justArmed)
    {
    	/* Don't trigger directly after arming */
    	obj->trigger.justArmed = false;
    	return (false);
    }

    if (obj->trigger.state == DATALOG_TRIGGER_STATE_Armed)
    {
        DATALOG_TRIGGER_s* pTrigger = &obj->trigger;
        DATALOG_SAMPLE_TYPE currentSample = obj->channel[pTrigger->channel].currentSample;
        DATALOG_SAMPLE_TYPE lastSample = obj->channel[pTrigger->channel].lastSample;
        DATALOG_TRIGGER_EDGE_DIRECTION_e edge = pTrigger->edge;
        DATALOG_SAMPLE_TYPE level = pTrigger->level;

        switch (edge)
        {
        case DATALOG_TRIGGER_EDGE_DIRECTION_Rising:
            if (lastSample < level && currentSample >= level)
            {
                bTriggered = true;
            }
            break;

        case DATALOG_TRIGGER_EDGE_DIRECTION_Falling:
            if (lastSample > level && currentSample <= level)
            {
                bTriggered = true;
            }
            break;

        case DATALOG_TRIGGER_EDGE_DIRECTION_Both:
            if ((lastSample < level && currentSample >= level) ||
                (lastSample > level && currentSample <= level))
            {
                bTriggered = true;
            }
            break;
        }
    }

    return (bTriggered);
} /* end of DATALOG_checkTrigger() function */


void DATALOG_clearChannelData(DATALOG_Handle handle, int channel)
{
	DATALOG_Obj* obj = handle;
	DATALOG_CHANNEL_s* pChannel = &obj->channel[channel];

    int sample;
    for (sample = 0; sample < DATALOG_SAMPLE_COUNT; sample++)
    {
        pChannel->samples[sample] = 0;
    }
} /* end of DATALOG_clearChannelData() function */


void DATALOG_setChannel(DATALOG_Handle handle, uint16_t channel, DATALOG_SAMPLE_TYPE* pSample)
{
    DATALOG_Obj* obj = handle;

    DATALOG_CHANNEL_s* pChannel = &obj->channel[channel];

    pChannel->bEnabled = true;
    pChannel->pSource = pSample;
} /* end of DATALOG_setChannel() function */


void DATALOG_setCommand(DATALOG_Handle handle, const DATALOG_COMMAND_e command)
{
	DATALOG_Obj* obj = handle;

	obj->command = command;

	return;
} /* end of DATALOG_setCommand() function */


void DATALOG_getChannelBuffer(DATALOG_Handle handle, const int channel, DATALOG_SAMPLE_TYPE** ppBuffer, uint16_t* pSizeBytes)
{
	/* Does not support wrapped buffers */

	DATALOG_Obj* obj = handle;

	*ppBuffer = &obj->channel[channel].samples[0];
	*pSizeBytes = obj->endSampleIdx << 1; /* Samples are 16-bit, so we send twice the number of bytes */
	if (obj->dataStartIdx == 0 && obj->endSampleIdx == 0)
	{
		// ToDo: Figure out way to support a full buffer, 60 samples in all
		*pSizeBytes = DATALOG_SAMPLE_COUNT * 2;
	}

	return;
}


void DATALOG_setSampleWindow(DATALOG_Handle handle)
{
    DATALOG_Obj* obj = handle;

    uint16_t preTrigger = obj->trigger.preTrigger;
    uint16_t postTrigger = DATALOG_SAMPLE_COUNT - preTrigger;

    uint16_t preTriggerSamples = preTrigger;
    uint16_t postTriggerSamples = postTrigger;

    uint16_t dataStartIdx;
    if (preTriggerSamples > obj->nextSampleIdx)
    {
        // wrapped
        dataStartIdx = (obj->nextSampleIdx + DATALOG_SAMPLE_COUNT - preTriggerSamples);
    } else {
        dataStartIdx = obj->nextSampleIdx - preTriggerSamples;
    }

    obj->dataStartIdx = dataStartIdx;
    uint16_t endSampleIdx = obj->nextSampleIdx + postTriggerSamples;
    // Wrap
    if (endSampleIdx >= DATALOG_SAMPLE_COUNT)
    {
        endSampleIdx -= DATALOG_SAMPLE_COUNT;
    }
    obj->endSampleIdx = endSampleIdx;
} /* end of DATALOG_setSampleWindow() function */



/* end of datalog.c */

/************************ (C) COPYRIGHT 2023 Piak Electronic Design B.V. *****END OF FILE****/
