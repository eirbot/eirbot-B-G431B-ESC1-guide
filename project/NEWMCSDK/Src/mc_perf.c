/**
  ******************************************************************************
  * @file    mc_perf.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Execution time measurement
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

#include "parameters_conversion.h"
#include "mc_perf.h"

void MC_Perf_Measure_Init(MC_Perf_Handle_t *pHandle)
{
#ifdef NULL_PTR_CHECK_MC_PERF
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
  uint8_t  i;
  Perf_Handle_t  *pHdl;

  /* Set Debug mod for DWT IP Enabling */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  if (DWT->CTRL != 0U)
  {                                        /* Check if DWT is present. */
    DWT->CYCCNT  = 0;
    DWT->CTRL   |= DWT_CTRL_CYCCNTENA_Msk; /* Enable Cycle Counter. */
  }
  else
  {
    /* Nothing to do */
  }

    for (i = 0U; i < MC_PERF_NB_TRACES; i++)
    {
      pHdl = &pHandle->MC_Perf_TraceLog[i];
      pHdl->StartMeasure = 0;
      pHdl->DeltaTimeInCycle = 0;
      pHdl->min = UINT32_MAX;
      pHdl->max = 0;
    }
    pHandle->BG_Task_OnGoing = false;
    pHandle->AccHighFreqTasksCnt = 0;
#ifdef NULL_PTR_CHECK_MC_PERF
  }
#endif
}

void MC_Perf_Clear(MC_Perf_Handle_t *pHandle,uint8_t bMotor)
{
#ifdef NULL_PTR_CHECK_MC_PERF
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    uint8_t  i;
    Perf_Handle_t  *pHdl;

    for (i = 0U; i < 2; i++)
    {
      pHdl = &pHandle->MC_Perf_TraceLog[2*bMotor+i];
      pHdl->DeltaTimeInCycle = 0;
      pHdl->min = UINT32_MAX;
      pHdl->max = 0;
    }
#ifdef NULL_PTR_CHECK_MC_PERF
  }
#endif
}

/**
 * @brief  Start the measure of a code section called in foreground.
 * @param  pHandle: handler of the performance measurement component.
 * @param  CodeSection: code section to measure.
 */
void MC_Perf_Measure_Start(MC_Perf_Handle_t *pHandle, uint8_t CodeSection)
{
#ifdef NULL_PTR_CHECK_MC_PERF
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    uint32_t StartMeasure = DWT->CYCCNT;
    pHandle->MC_Perf_TraceLog[CodeSection].StartMeasure = StartMeasure;
#ifdef NULL_PTR_CHECK_MC_PERF
  }
#endif
}

/**
 * @brief  Start the measure of a code section called in background.
 * @param  pHandle: handler of the performance measurement component.
 * @param  CodeSection: code section to measure.
 */
void MC_BG_Perf_Measure_Start(MC_Perf_Handle_t *pHandle, uint8_t  CodeSection)
{
#ifdef NULL_PTR_CHECK_MC_PERF
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    pHandle->BG_Task_OnGoing = true;
    pHandle->AccHighFreqTasksCnt = 0;
    uint32_t StartMeasure = DWT->CYCCNT;
    pHandle->MC_Perf_TraceLog[CodeSection].StartMeasure = StartMeasure;
#ifdef NULL_PTR_CHECK_MC_PERF
  }
#endif
}

/**
 * @brief  Stop the measurement of a code section and compute elapse time.
 * @param  pHandle: handler of the performance measurement component.
 * @param  CodeSection: code section to measure.
 */
void MC_Perf_Measure_Stop(MC_Perf_Handle_t *pHandle, uint8_t  CodeSection)
{
#ifdef NULL_PTR_CHECK_MC_PERF
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    uint32_t StopMeasure;
    Perf_Handle_t *pHdl;

    StopMeasure = DWT->CYCCNT;
    pHdl = &pHandle->MC_Perf_TraceLog[CodeSection];

    /* Check Overflow cases */
    if (StopMeasure < pHdl->StartMeasure)
    {
      pHdl->DeltaTimeInCycle = (UINT32_MAX - pHdl->StartMeasure) + StopMeasure;
    }
    else
    {
      pHdl->DeltaTimeInCycle = StopMeasure - pHdl->StartMeasure;
    }

    if(pHandle->BG_Task_OnGoing)
    {
      pHandle->AccHighFreqTasksCnt += pHdl->DeltaTimeInCycle;
    }
    else
    {
      /* Nothing to do */
    }

    if (pHdl->max < pHdl->DeltaTimeInCycle)
    {
      pHdl->max = pHdl->DeltaTimeInCycle;
    }
    else
    {
      /* Nothing to do */
    }

    if (pHdl->min > pHdl->DeltaTimeInCycle)
    {
      pHdl->min = pHdl->DeltaTimeInCycle;
    }
    else
    {
      /* Nothing to do */
    }
#ifdef NULL_PTR_CHECK_MC_PERF
  }
#endif
}

/**
 * @brief  Stop the measurement of a code section in BG and compute elapse time.
 * @param  pHandle: handler of the performance measurement component.
 * @param  CodeSection: code section to measure.
 */
void MC_BG_Perf_Measure_Stop(MC_Perf_Handle_t *pHandle, uint8_t CodeSection)
{
#ifdef NULL_PTR_CHECK_MC_PERF
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    Perf_Handle_t *pHdl;
    uint32_t StopMeasure = DWT->CYCCNT;
    pHandle->BG_Task_OnGoing = false;

    pHdl  = &pHandle->MC_Perf_TraceLog[CodeSection];

    /* Check Overflow cases */
    if (StopMeasure < pHdl->StartMeasure)
    {
      pHdl->DeltaTimeInCycle = (UINT32_MAX - pHdl->StartMeasure) + StopMeasure;
    }
    else
    {
      pHdl->DeltaTimeInCycle = StopMeasure - pHdl->StartMeasure;
    }

    if (pHdl->DeltaTimeInCycle > pHandle->AccHighFreqTasksCnt)
    {
      pHdl->DeltaTimeInCycle -= pHandle->AccHighFreqTasksCnt;
    }
    else
    {
      /* Nothing to do */
    }
    if (pHdl->max < pHdl->DeltaTimeInCycle)
    {
      pHdl->max = pHdl->DeltaTimeInCycle;
    }
    else
    {
      /* Nothing to do */
    }

    if (pHdl->min > pHdl->DeltaTimeInCycle)
    {
      pHdl->min = pHdl->DeltaTimeInCycle;
    }
    else
    {
      /* Nothing to do */
    }
#ifdef NULL_PTR_CHECK_MC_PERF
  }
#endif
}

/**
 * @brief  It returns the current CPU load of both High and Medium frequency tasks.
 * @param  pHandle: handler of the performance measurement component.
 * @retval CPU load.
 */
float_t MC_Perf_GetCPU_Load(const MC_Perf_Handle_t *pHandle)
{
  float_t cpuLoad = 0.0f;
#ifdef NULL_PTR_CHECK_MC_PERF
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    float_t MFT_cpu_loadM1;
    float_t HFT_cpu_loadM1;

    MFT_cpu_loadM1 = (((float_t)pHandle->MC_Perf_TraceLog[MEASURE_TSK_MediumFrequencyTaskM1].DeltaTimeInCycle\
                   / (float_t)SYSCLK_FREQ ) * (float_t)MEDIUM_FREQUENCY_TASK_RATE);
    HFT_cpu_loadM1 = (((float_t)pHandle->MC_Perf_TraceLog[MEASURE_TSK_HighFrequencyTaskM1].DeltaTimeInCycle\
                   / (float_t)SYSCLK_FREQ ) * (float_t)(PWM_FREQUENCY/REGULATION_EXECUTION_RATE));

    cpuLoad = MFT_cpu_loadM1 + HFT_cpu_loadM1;

    cpuLoad = (cpuLoad > 1.0f) ? 1.0f : cpuLoad;
    cpuLoad *= 100.0f;
#ifdef NULL_PTR_CHECK_MC_PERF
  }
#endif
  return (cpuLoad);
}

/**
 * @brief  It returns the maximum CPU load of both High and Medium frequency tasks.
 * @param  pHandle: handler of the performance measurement component.
 * @retval Max CPU load measured.
 */
float_t MC_Perf_GetMaxCPU_Load(const MC_Perf_Handle_t *pHandle)
{
  float_t cpuLoad = 0.0f;
#ifdef NULL_PTR_CHECK_MC_PERF
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    cpuLoad += (((float_t)pHandle->MC_Perf_TraceLog[MEASURE_TSK_MediumFrequencyTaskM1].max / (float_t)SYSCLK_FREQ )\
                   * (float_t)MEDIUM_FREQUENCY_TASK_RATE);
    cpuLoad += (((float_t)pHandle->MC_Perf_TraceLog[MEASURE_TSK_HighFrequencyTaskM1].max / (float_t)SYSCLK_FREQ )\
                   * (float_t)(PWM_FREQUENCY/REGULATION_EXECUTION_RATE));

    cpuLoad = (cpuLoad > 1.0f) ? 1.0f : cpuLoad;
    cpuLoad *= 100.0f;
#ifdef NULL_PTR_CHECK_MC_PERF
  }
#endif
  return (cpuLoad);
}

/**
 * @brief  It returns the minimum CPU load of both High and Medium frequency tasks.
 * @param  pHandle: handler of the performance measurement component.
 * @retval Min CPU load measured.
 */
float_t MC_Perf_GetMinCPU_Load(const MC_Perf_Handle_t *pHandle)
{
  float_t cpu_load_acc = 0.0f;
#ifdef NULL_PTR_CHECK_MC_PERF
  if (MC_NULL == pHandle)
  {
    /* Nothing to do */
  }
  else
  {
#endif
    if (pHandle->MC_Perf_TraceLog[MEASURE_TSK_MediumFrequencyTaskM1].min != UINT32_MAX)
    {
      cpu_load_acc += (((float_t)pHandle->MC_Perf_TraceLog[MEASURE_TSK_MediumFrequencyTaskM1].min / (float_t)SYSCLK_FREQ )\
                     * (float_t)MEDIUM_FREQUENCY_TASK_RATE);
    }
    if (pHandle->MC_Perf_TraceLog[MEASURE_TSK_HighFrequencyTaskM1].min != UINT32_MAX)
    {
      cpu_load_acc += (((float_t)pHandle->MC_Perf_TraceLog[MEASURE_TSK_HighFrequencyTaskM1].min / (float_t)SYSCLK_FREQ )\
                   * (float_t)(PWM_FREQUENCY/REGULATION_EXECUTION_RATE));
    }
    cpu_load_acc = (cpu_load_acc > 1.0f) ? 1.0f : cpu_load_acc;
    cpu_load_acc *= 100.0f;
#ifdef NULL_PTR_CHECK_MC_PERF
  }
#endif
  return (cpu_load_acc);
}

/************************ (C) COPYRIGHT 2023 STMicroelectronics *****END OF FILE****/
