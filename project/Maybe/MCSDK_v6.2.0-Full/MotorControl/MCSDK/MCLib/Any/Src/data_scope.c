/**
  ******************************************************************************
  * @file    data_scope.c
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
/* data_scope.c */

#include "data_scope.h"

#include <string.h> // memset

/* ToDo:
 *      Conversion and scaling;
 *          Source is IQ24, IQ15, IQ30 etc.
 *          Current scaling
 *              IQ_PU_A to 1V/A, 0.1V/A, 0.01V/A etc.
 *      User sets preference in some scale that makes sense
 *          1V/Nm, 1V/VpHz
 *
 */

/* Responsibilities:
 *      Offset, scale. Compensate for DAC board deviations
 *      	(This module was first developed for DAC output, and this functionality is left in)
 *      Signal selection
 *          Whole group select, 4 channels at once
 *          Individual channel selection
 *      Scale selection
 *          1V/Nm, 0.1V/Nm etc for torques
 *          1V/VpHz for flux
 *          Scale-up, scale-down impulse, like encoder knob on oscilloscope
 *          Natural scale, not per unit
 *          Per unit to A, 1V/A etc.
 *          	(1V/<unit> terminology is a legacy of the DAC output)
 * Excluded:
 * 		Meta-data
 * 			Is defined in another module, which is application-specific. DATA_SCOPE is a more general module.
 */

/* Implementation:
 *      _run function
 *          performs scaling using current offset, scale factors
 *          uses current input values, pointer to struct parameter input
 *          does not do floats (?) (maybe FPU on only?)
 * choice:
 *      Should this lookup data in memory? (internal selection?
 *          Must know data then?
 *          Or location, type etc of data
 *      Should this be given data from outside (outside selection)
 *          Channel select is external then [So No]
 *
 * Option:
 * Channel has
 *      Source location (void*)
 *      Numerical type (uint16, int32 etc)
 *      Data type (A_PU, FLUX_PU, V_PU etc)
 *          Scaling from internal Per Unit to usable units)
 *      Data scale (iqFmt)
 *      Scope scale (1V/U, 10V/U, 0.1V/U etc)
 *          1x, 2x, 5x, 10x, 20x, 50x, 100x, 0.5x, 0.2x, 0.1x etc
 *          Ranges?
 *
 */


/* Internal function declarations */



/* Initialization functions */

DATA_SCOPE_Handle DATA_SCOPE_init(void* pMemory, const size_t numWords)
{
    DATA_SCOPE_Handle handle = (DATA_SCOPE_Handle) 0;

    if (numWords >= sizeof(DATA_SCOPE_Obj))
    {
        handle = (DATA_SCOPE_Handle) pMemory;

        memset(pMemory, 0, numWords);
    }

    return handle;
} /* end of DATA_SCOPE_init() function */


void DATA_SCOPE_setup(DATA_SCOPE_Handle handle)
{
    DATA_SCOPE_Obj* obj = (DATA_SCOPE_Obj*) handle;
    DATA_SCOPE_CHANNEL_s* pChannel = 0;

    int i;
    for (i = 0; i < DATA_SCOPE_NUMCHANNELS; i++)
    {
        pChannel = &obj->channel[i];

        /* Disable channel */
        pChannel->enabled = false;

        /* Set scale and offset to defaults */
        pChannel->dac_scale = FIXP16(0.1);
        pChannel->dac_offset = FIXP16(0.5);
        pChannel->data_scale_f = 0.0f;
        DATA_SCOPE_setChannelDataScale(handle, i, 1.0f);
    }

    return;
} /* end of DATA_SCOPE_setup() function */



/* Functional */

void DATA_SCOPE_run(DATA_SCOPE_Handle handle)
{
    DATA_SCOPE_Obj* obj = (DATA_SCOPE_Obj*) handle;
    DATA_SCOPE_CHANNEL_s* pChannel = 0;

    int i;

    for (i = 0; i < DATA_SCOPE_NUMCHANNELS; i++)
    {
        pChannel = &obj->channel[i];
        if (pChannel->enabled)
        {
            // Read data from RAM
            uint16_t dac_value = 0;
            switch (pChannel->datatype)
            {
            case DATA_DATATYPE_Int32:
                {
                    /* Read value, in native scale unit */
                    fixp_t value = *((fixp_t*)(pChannel->pData));

                    /* Multiply by data_scale, which is in FIXP(V/unit) */
                    value = FIXP_mpyFIXPscaled(value, &pChannel->data_scale_fixps);
                        // data_qFmt is taken into account in setChannelDataScale

                    pChannel->data_fixp24_volt = value;

                    /* Saturate to +/- 5V */
                    value = FIXP_sat(value, FIXP(5.0), FIXP(-5.0));

                    /* Scale to DAC scaling */
                    value = FIXP_mpy(value, pChannel->dac_scale);

                    /* Adjust for offset */
                    value -= pChannel->dac_offset;

                    /* Limit to full scale */
                    // value = FIXP_sat(value, 0xFFFF, 0);
					#warning Issue here is likely that value is signed, and 0xFFFF is unsigned?

                    dac_value = (uint16_t) value;
                }
                break;

            }

            // Store
            pChannel->dac_value = dac_value;
        }
    }

    return;
} /* end of DATA_SCOPE_run() function */


uint16_t DATA_SCOPE_getSingleChannelValue(DATA_SCOPE_Handle handle, uint16_t channel)
{
    DATA_SCOPE_Obj* obj = (DATA_SCOPE_Obj*) handle;
    DATA_SCOPE_CHANNEL_s* pChannel = &obj->channel[channel];

    uint16_t value = pChannel->dac_offset;

    if (pChannel->enabled) value = pChannel->dac_value;

    return (value);
} /* end of DATA_SCOPE_getSingleChannelValue() function */


void DATA_SCOPE_setChannelDacScaleAndOffset(DATA_SCOPE_Handle handle, const uint16_t channel, const fixp15_t scale, const fixp15_t offset)
{
    DATA_SCOPE_Obj* obj = (DATA_SCOPE_Obj*) handle;

    DATA_SCOPE_CHANNEL_s* pChannel = &obj->channel[channel];

    pChannel->dac_scale = scale;
    pChannel->dac_offset = offset;

    return;
} /* end of DATA_SCOPE_setChannelDacScaleAndOffset() function */


void DATA_SCOPE_setChannelDataScale(DATA_SCOPE_Handle handle, const uint16_t channel, const float dataScale)
{
    DATA_SCOPE_Obj* obj = (DATA_SCOPE_Obj*) handle;

    DATA_SCOPE_CHANNEL_s* pChannel = &obj->channel[channel];

    /* Only update if the data scale has actually changed */
    if (pChannel->data_scale_f == dataScale) return;

    /* Remember the new data scale */
    pChannel->data_scale_f = dataScale;

    float ds = 1.0f / dataScale;

    FIXPSCALED_calculateScaleFactor(pChannel->data_qFmt, 24, pChannel->data_fullscale, 1.0f, ds, &pChannel->data_scale_fixps);

    return;
}


/* Internal functions */

/* end of data_scope.c */

/************************ (C) COPYRIGHT 2023 Piak Electronic Design B.V. *****END OF FILE****/
