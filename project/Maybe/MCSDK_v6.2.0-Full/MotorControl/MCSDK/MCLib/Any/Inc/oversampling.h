/**
  ******************************************************************************
  * @file    oversampling.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          oversampling component of the Motor Control SDK.
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
  * @ingroup oversampling
  */

#ifndef _OVERSAMPLING_H_
#define _OVERSAMPLING_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "fixpmath.h"
#include "drive_parameters.h" 
#include "parameters_conversion.h"
/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup oversampling
  * @{
  */
  
#define ADC_FIXPFMT (16) /*!< @brief ADC fixp format */
#define DMA_BUFFERS (2)  /*!< @brief double buffering constant */

/**
  * @brief  Sample data structure definition 
  */
typedef struct _Samples_t_
{
	fixp30_t I_R; /*!< @brief Phase R current sample */
	fixp30_t I_S; /*!< @brief Phase S current sample */
	fixp30_t I_T; /*!< @brief Phase T current sample */
	fixp30_t U_R; /*!< @brief Phase R voltage sample */
	fixp30_t U_S; /*!< @brief Phase S voltage sample */
	fixp30_t U_T; /*!< @brief Phase T voltage sample */
	fixp30_t U_DC; /*!< @brief DC bus voltage sample */
} Samples_t;

/**
  * @brief  Oversampling channel enum definition 
  */
typedef enum _Oversampling_ChannelName_e_
{
	OVS_I_R,          /*!< @brief index of R current sample */
	OVS_I_S,          /*!< @brief index of S current sample */
	OVS_I_T,          /*!< @brief index of T current sample */
	OVS_U_R,          /*!< @brief index of R voltage sample */
	OVS_U_S,          /*!< @brief index of S voltage sample */
	OVS_U_T,          /*!< @brief index of T voltage sample */
	OVS_U_DC,         /*!< @brief index of DC bus voltage sample */
	OVS_THROTTLE,     /*!< @brief index of throttle sample */
	OVS_TEMP,         /*!< @brief index of temperature voltage sample */
	OVS_numChannels
} Oversampling_ChannelName_e;

/**
  * @brief  Oversampling channel data structure definition 
  */
typedef struct _Oversampling_Channel_t_
{
	uint16_t	adc;		/*!< @brief ADC for this channel, zero-based */
	uint16_t	rank;		/*!< @brief Rank determines the starting point */
	uint16_t	tasklen;	/*!< @brief Number of ADC Channels this ADC has to sample */
	bool		single;		/*!< @brief Take a single sample at singleIdx, used for low 
	                                    side current shunt measurements */
} Oversampling_Channel_t;

/**
  * @brief  DMA double buffering defintion
  *
  * The instanciated buffer will be used to store the values transfered from ADCs by the DMA
  * with double buffering mechanism.
  *
  */
typedef uint16_t DMA_ADC_Buffer_t[OVS_LONGEST_TASK * OVS_COUNT * REGULATION_EXECUTION_RATE];

/**
  * @brief  oversampling data structure definition
  */
typedef struct _Oversampling_t_
{
	uint16_t	count;              /*!< @brief oversampling level */
	uint16_t	divider;			/*!< @brief Number of PWM periods per FOC cycle */
	int8_t		singleIdx;			/*!< @brief Oversampling index of single sample */
	int8_t		readBuffer;         /*!< @brief index of the read buffer (double buffering mechanism) */
	int8_t		writeBuffer;        /*!< @brief index of the write buffer (double buffering mechanism) */
	fixp30_t	current_factor;     /*!< @brief phase current scaling factor that will be applied to the value read from ADC */
	fixp30_t	voltage_factor;     /*!< @brief phase voltage scaling factor that will be applied to the value read from ADC */
	fixp30_t	busvoltage_factor;  /*!< @brief DC bus scaling factor that will be applied to the value read from ADC */
	DMA_ADC_Buffer_t DMABuffer[DMA_BUFFERS][OVS_NUM_ADCS]; /*!< @brief One DMA buffer per ADC.     
	                                                          These are all the same size as the largest required buffer. */
	Oversampling_Channel_t Channel[OVS_numChannels]; /*!< @brief Each measurement has an oversampling channel*/
} Oversampling_t;

extern Oversampling_t oversampling;

void OVERSAMPLING_getMeasurements(Currents_Irst_t* pCurrents_Irst, Voltages_Urst_t* pVoltages_Urst);
void OVERSAMPLING_getPotentiometerMeasurements( fixp30_t *value);
void OVERSAMPLING_getVbusMeasurements( fixp30_t *value);
void OVERSAMPLING_getTempMeasurements( fixp30_t *value);
/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* _OVERSAMPLING_H_ */

