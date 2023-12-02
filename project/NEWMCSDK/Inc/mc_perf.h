/**
  ******************************************************************************
  * @file    mc_perf.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Execution time measurement definitions
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
#ifndef MC_PERF_H
#define MC_PERF_H

typedef enum
{
  MEASURE_TSK_HighFrequencyTaskM1,
  MEASURE_TSK_MediumFrequencyTaskM1,
/*  Others functions to measure to be added here. */
} MC_PERF_FUNCTIONS_LIST_t;

/* Define max number of traces according to the list defined in MC_PERF_FUNCTIONS_LIST_t */
#define  MC_PERF_NB_TRACES  2U

/* DWT (Data Watchpoint and Trace) registers, only exists on ARM Cortex with a DWT unit */
/* The DWT is usually implemented in Cortex-M3 or higher, but not on Cortex-M0(+) (ie not present on G0) */

typedef struct
{
    uint32_t  StartMeasure;
    uint32_t  DeltaTimeInCycle;
    uint32_t  min;
    uint32_t  max;
} Perf_Handle_t;

typedef struct
{
    bool   BG_Task_OnGoing;
    uint32_t  AccHighFreqTasksCnt;
    Perf_Handle_t MC_Perf_TraceLog[MC_PERF_NB_TRACES];
} MC_Perf_Handle_t;

void MC_Perf_Measure_Init(MC_Perf_Handle_t *pHandle);
void MC_Perf_Clear(MC_Perf_Handle_t *pHandle,uint8_t bMotor);
void MC_Perf_Measure_Start(MC_Perf_Handle_t *pHandle, uint8_t i);
void MC_BG_Perf_Measure_Start(MC_Perf_Handle_t *pHandle, uint8_t i);
void MC_Perf_Measure_Stop(MC_Perf_Handle_t *pHandle, uint8_t i);
void MC_BG_Perf_Measure_Stop(MC_Perf_Handle_t *pHandle, uint8_t i);

float_t MC_Perf_GetCPU_Load(const MC_Perf_Handle_t *pHandle);
float_t MC_Perf_GetMaxCPU_Load(const MC_Perf_Handle_t *pHandle);
float_t MC_Perf_GetMinCPU_Load(const MC_Perf_Handle_t *pHandle);

#endif /* MC_PERF_H */
/************************ (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
