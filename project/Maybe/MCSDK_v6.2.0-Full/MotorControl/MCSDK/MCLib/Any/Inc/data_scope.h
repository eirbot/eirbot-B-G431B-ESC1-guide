/**
  ******************************************************************************
  * @file    data_scope.h
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
/* data_scope.h */

#ifndef _DATA_SCOPE_H_
#define _DATA_SCOPE_H_

/* Functionality:
 * 		Read data in RAM in one format, convert to another format, write to another location in RAM
 * 		Used to convert internal motor control signals into the correct format to communicate to the PC, or DAC output
 */

#include "fixpmath.h"
#include <stdbool.h>
#include <stddef.h>

#define DATA_SCOPE_NUMCHANNELS   (4)

/* ToDo:
 *      Remote control support
 *      - Select pre-defined set of measurements
 *      - Change single channel;
 *          - Select variable (implicitly sets data type, qFmt, full scale, address, maybe default display scale)
 *          - Change display scale (unit/V, e.g. 10A/V, 0.001VpHz/V etc)
 *          - Optional; Change internal parameters, like data qFmt, full scale -- never address, that's protected
 */

typedef enum _DATA_Datatype_e_
{
    DATA_DATATYPE_Int32,
    DATA_DATATYPE_Int16,
} DATA_Datatype_e;

typedef struct _DATA_SCOPE_CHANNEL_s_
{
    bool            enabled;        /* Disable channel while adjusting settings, or not yet configured */

    /* data description */
    void*           pData;          /* Location of data in memory */
    DATA_Datatype_e	datatype;       /* Data type in memory */
    fixpFmt_t		data_qFmt;      /* Data qFmt in memory */
    float           data_fullscale; /* Full scale data range for per unit values, 1.0f default */

    /* conversion to volts - calculated from data_scale_f and data description above */
    float           data_scale_f;   /* scale factor for data (V/unit), remembered to prevent unnecessary re-calculations */
    FIXP_scaled_t    data_scale_fixps; /* scale factor for data (V/unit), derived from data_scale_f */
    fixp24_t      	data_fixp24_volt; /* debug, data in volts */

    /* DAC calibration */
    fixp15_t           dac_scale;      /* Number of DAC counts for 1V range */
    fixp15_t           dac_offset;     /* DAC value for 0V output */

    /* output */
    uint16_t        dac_value;      /* Value written to the DAC */
} DATA_SCOPE_CHANNEL_s;

typedef struct _DATA_SCOPE_Command_s_
{
    uint32_t        command:8;          // 7:0
    uint32_t        channel:8;          // 15:8
    uint32_t        variable:8;         // 23:16
    uint32_t        parameter:8;        // 31:24
} DATA_SCOPE_Command_s;

typedef union _DATA_SCOPE_Command_u_
{
    DATA_SCOPE_Command_s     bits;
    uint32_t                all;
} DATA_SCOPE_Command_u;

typedef struct _DATA_SCOPE_Obj_
{
    DATA_SCOPE_Command_u command;
    DATA_SCOPE_CHANNEL_s channel[DATA_SCOPE_NUMCHANNELS];
} DATA_SCOPE_Obj;

typedef struct _DATA_SCOPE_Obj_* DATA_SCOPE_Handle;


extern DATA_SCOPE_Handle DATA_SCOPE_init(void* pMemory, const size_t numWords);

extern void DATA_SCOPE_setup(DATA_SCOPE_Handle handle);

/* Calculate new values for all active channels */
extern void DATA_SCOPE_run(DATA_SCOPE_Handle handle);

extern uint16_t DATA_SCOPE_getSingleChannelValue(DATA_SCOPE_Handle handle, uint16_t channel);

extern void DATA_SCOPE_setChannelDacScaleAndOffset(DATA_SCOPE_Handle handle, const uint16_t channel, const fixp15_t scale, const fixp15_t offset);

/* Oscilloscope divisions are 1V per division. DataScale is set to x/1V; E.g. dataScale 10.0 would mean 10A/V output. */
extern void DATA_SCOPE_setChannelDataScale(DATA_SCOPE_Handle handle, const uint16_t channel, const float dataScale);


#endif /* _DATA_SCOPE_H_ */

/* end of data_scope.h */

/************************ (C) COPYRIGHT 2023 Piak Electronic Design B.V. *****END OF FILE****/
