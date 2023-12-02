/**
  ******************************************************************************
  * @file    stspin32g4.c
  * @author  P. Lombardi, IPC Application Team
  * @brief   Implementation of STSPIN32G4 driver library
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
  **/
#if defined(USE_FULL_LL_DRIVER)

#include <stm32g4xx_ll_i2c.h>
#include <stm32g4xx_ll_gpio.h>
#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_cortex.h>
#include <stm32g4xx_ll_rcc.h>
#include <stm32g4xx_ll_utils.h>

#else

#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_def.h>
#include <stm32g4xx_hal_i2c.h>

#endif

#include <stdint.h>
#include <stdbool.h>

#include <stspin32g4.h>

#define STSPIN32G4_I2C_TIMEOUT      (100u)
#define STSPIN32G4_I2C_LOCKCODE     (0xDu)

#if defined(USE_FULL_LL_DRIVER)

#define GD_READY_GPIO_Port GPIOE
#define GD_READY_Pin LL_GPIO_PIN_14
#define GD_NFAULT_GPIO_Port GPIOE
#define GD_NFAULT_Pin LL_GPIO_PIN_15
#define GD_WAKE_GPIO_Port GPIOE
#define GD_WAKE_Pin LL_GPIO_PIN_7

#else

#define GD_READY_GPIO_Port GPIOE
#define GD_READY_Pin GPIO_PIN_14
#define GD_NFAULT_GPIO_Port GPIOE
#define GD_NFAULT_Pin GPIO_PIN_15
#define GD_WAKE_GPIO_Port GPIOE
#define GD_WAKE_Pin GPIO_PIN_7

#endif

static uint8_t STSPIN32G4_bkupREADY;

#if !defined(USE_FULL_LL_DRIVER)

extern I2C_HandleTypeDef hi2c3;

#endif

void SystemClock_Config(void);

STSPIN32G4_StatusTypeDef STSPIN32G4_init(STSPIN32G4_HandleTypeDef *hdl)
{
  STSPIN32G4_StatusTypeDef status = STSPIN32G4_OK;

  if (hdl == NULL)
  {
    return STSPIN32G4_ERROR;
  }

#if defined(USE_FULL_LL_DRIVER)

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  hdl->i2cHdl = I2C3;

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOE);
  LL_GPIO_SetOutputPin(GD_WAKE_GPIO_Port, GD_WAKE_Pin);

  GPIO_InitStruct.Pin = GD_WAKE_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_LOW;
  LL_GPIO_Init(GD_WAKE_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GD_NFAULT_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GD_NFAULT_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GD_READY_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GD_READY_GPIO_Port, &GPIO_InitStruct);

#else

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  hdl->i2cHdl = &hi2c3;

  __HAL_RCC_GPIOE_CLK_ENABLE();
  HAL_GPIO_WritePin(GD_WAKE_GPIO_Port, GD_WAKE_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = GD_WAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GD_WAKE_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GD_NFAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GD_NFAULT_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GD_READY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GD_READY_GPIO_Port, &GPIO_InitStruct);

#endif

  if (status != STSPIN32G4_OK)
  {
    return status;
  }

  return status;
}

STSPIN32G4_StatusTypeDef STSPIN32G4_deInit(STSPIN32G4_HandleTypeDef *hdl)
{
  STSPIN32G4_StatusTypeDef status = STSPIN32G4_OK;

#if defined(USE_FULL_LL_DRIVER)

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  status = (STSPIN32G4_StatusTypeDef) LL_I2C_DeInit(hdl->i2cHdl);

  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;

  GPIO_InitStruct.Pin = GD_WAKE_Pin;
  LL_GPIO_Init(GD_WAKE_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GD_NFAULT_Pin;
  LL_GPIO_Init(GD_NFAULT_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GD_READY_Pin;
  LL_GPIO_Init(GD_READY_GPIO_Port, &GPIO_InitStruct);

#else

  status = (STSPIN32G4_StatusTypeDef) HAL_I2C_DeInit(hdl->i2cHdl);
  HAL_GPIO_DeInit(GD_WAKE_GPIO_Port, GD_WAKE_Pin);
  HAL_GPIO_DeInit(GD_NFAULT_GPIO_Port, GD_NFAULT_Pin);
  HAL_GPIO_DeInit(GD_READY_GPIO_Port, GD_READY_Pin);

#endif

  hdl->i2cHdl = NULL;

  return status;
}

STSPIN32G4_StatusTypeDef STSPIN32G4_lockReg(STSPIN32G4_HandleTypeDef *hdl)
{
  STSPIN32G4_StatusTypeDef status;
  STSPIN32G4_statusRegTypeDef statusReg;
  uint8_t i2cReg = 0;

  if (hdl == NULL)
  {
    return STSPIN32G4_ERROR;
  }

  i2cReg = ((STSPIN32G4_I2C_LOCKCODE << 4) & 0xf0) | (STSPIN32G4_I2C_LOCKCODE & 0x0f);
  status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_LOCK, i2cReg);

#ifdef STSPIN32G4_I2C_LOCKUSEPARANOID

  if (status == STSPIN32G4_OK)
  {
    status = STSPIN32G4_getStatus(hdl, &statusReg);
  }

  if (status == STSPIN32G4_OK)
  {
    if (statusReg.lock != 1)
    {
      status = STSPIN32G4_ERROR;
    }
  }

#endif

  return status;
}

STSPIN32G4_StatusTypeDef STSPIN32G4_unlockReg(STSPIN32G4_HandleTypeDef *hdl)
{
  STSPIN32G4_StatusTypeDef status;
  STSPIN32G4_statusRegTypeDef statusReg;
  uint8_t i2cReg = 0;

  if (hdl == NULL)
  {
    return STSPIN32G4_ERROR;
  }

  i2cReg = (((~STSPIN32G4_I2C_LOCKCODE) << 4) & 0xf0) | (STSPIN32G4_I2C_LOCKCODE & 0x0f);
  status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_LOCK, i2cReg);

#ifdef STSPIN32G4_I2C_LOCKUSEPARANOID

  if (status == STSPIN32G4_OK)
  {
    status = STSPIN32G4_getStatus(hdl, &statusReg);
  }

  if (status == STSPIN32G4_OK)
  {
    if (statusReg.lock == 1)
    {
      status = STSPIN32G4_ERROR;
    }
  }

#endif

  return status;
}

STSPIN32G4_StatusTypeDef STSPIN32G4_readReg(STSPIN32G4_HandleTypeDef *hdl, uint8_t regAddr, uint8_t *value)
{
  STSPIN32G4_StatusTypeDef status = STSPIN32G4_OK;
  if (hdl == NULL)
  {
		status = STSPIN32G4_ERROR;
    return status;
  }

  if (hdl->i2cHdl == NULL)
  {
		status = STSPIN32G4_ERROR;
    return status;
  }

  if (value == NULL)
  {
		status = STSPIN32G4_ERROR;
    return status;
  }

#if defined(USE_FULL_LL_DRIVER)

  LL_RCC_ClocksTypeDef RCC_Clocks;
  LL_RCC_GetSystemClocksFreq(&RCC_Clocks);

  uint32_t tickFreq = RCC_Clocks.HCLK_Frequency /  (SysTick->LOAD + 1);

  if (LL_SYSTICK_GetClkSource() == LL_SYSTICK_CLKSOURCE_HCLK_DIV8)
  {
    tickFreq /= 8;
  }

  uint8_t stat = 0;
  uint32_t ticks = (STSPIN32G4_I2C_TIMEOUT * tickFreq) / 1000 + 1;

  while (true)
  {
    switch (stat)
    {
      case 0:
      stat += !LL_I2C_IsActiveFlag_BUSY(hdl->i2cHdl);
      break;

      case 1:
      LL_I2C_HandleTransfer(hdl->i2cHdl, STSPIN32G4_I2C_ADDR, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);
      stat++;
      break;

      case 2:
      stat += LL_I2C_IsActiveFlag_TXIS(hdl->i2cHdl);
      break;

      case 3:
      LL_I2C_TransmitData8(hdl->i2cHdl, regAddr);
      stat++;
      break;

      case 4:
      stat += LL_I2C_IsActiveFlag_TC(hdl->i2cHdl);
      break;

      case 5:
      LL_I2C_HandleTransfer(hdl->i2cHdl, STSPIN32G4_I2C_ADDR, LL_I2C_ADDRSLAVE_7BIT, 1, I2C_CR2_AUTOEND, LL_I2C_GENERATE_START_READ);
      stat++;
      break;

      case 6:
      stat += LL_I2C_IsActiveFlag_RXNE(hdl->i2cHdl);
      break;

      case 7:
      *value = LL_I2C_ReceiveData8(hdl->i2cHdl);
      stat++;
      break;

      case 8:
      stat += LL_I2C_IsActiveFlag_STOP(hdl->i2cHdl);
      break;

      case 9:
      LL_I2C_ClearFlag_STOP(hdl->i2cHdl);
      status = STSPIN32G4_OK;
      return status;

      default:
      status = STSPIN32G4_ERROR;
      return status;

    }

    if (LL_SYSTICK_IsActiveCounterFlag())
    {
      ticks--;
    }

    if(ticks == 0)
    {
      status = STSPIN32G4_TIMEOUT;
      return status;
    }
  }

#else

  uint32_t ticks = (STSPIN32G4_I2C_TIMEOUT * HAL_GetTickFreq()) / 1000 + 1;
  status = (STSPIN32G4_StatusTypeDef) HAL_I2C_Mem_Read(hdl->i2cHdl, STSPIN32G4_I2C_ADDR, (uint16_t) regAddr, 1, value, 1, ticks);
  return status;


#endif

}

STSPIN32G4_StatusTypeDef STSPIN32G4_writeReg(STSPIN32G4_HandleTypeDef *hdl, uint8_t regAddr, uint8_t value)
{
  STSPIN32G4_StatusTypeDef status = STSPIN32G4_OK;
  if (hdl == NULL)
  {
		status = STSPIN32G4_ERROR;
    return status;
  }

  if (hdl->i2cHdl == NULL)
  {
		status = STSPIN32G4_ERROR;
    return status;
  }

#if defined(USE_FULL_LL_DRIVER)

  LL_RCC_ClocksTypeDef RCC_Clocks;
  LL_RCC_GetSystemClocksFreq(&RCC_Clocks);

  uint32_t tickFreq = RCC_Clocks.HCLK_Frequency /  (SysTick->LOAD + 1);

  if (LL_SYSTICK_GetClkSource() == LL_SYSTICK_CLKSOURCE_HCLK_DIV8)
  {
    tickFreq /= 8;
  }

  uint8_t stat = 0;
  uint32_t ticks = (STSPIN32G4_I2C_TIMEOUT * tickFreq) / 1000 + 1;

  while (true)
  {
    switch (stat)
    {
    case 0:
      stat += !LL_I2C_IsActiveFlag_BUSY(hdl->i2cHdl);
      break;

    case 1:
      LL_I2C_HandleTransfer(hdl->i2cHdl, STSPIN32G4_I2C_ADDR, LL_I2C_ADDRSLAVE_7BIT, 1, I2C_CR2_RELOAD, LL_I2C_GENERATE_START_WRITE);
      stat++;
      break;

    case 2:
      stat += LL_I2C_IsActiveFlag_TXIS(hdl->i2cHdl);
      break;

    case 3:
      LL_I2C_TransmitData8(hdl->i2cHdl, regAddr);
      stat++;
      break;

    case 4:
      stat += LL_I2C_IsActiveFlag_TCR(hdl->i2cHdl);
      break;

    case 5:
      LL_I2C_HandleTransfer(hdl->i2cHdl, STSPIN32G4_I2C_ADDR, LL_I2C_ADDRSLAVE_7BIT, 1, I2C_CR2_AUTOEND, LL_I2C_GENERATE_NOSTARTSTOP);
      stat++;
      break;

    case 6:
      stat += LL_I2C_IsActiveFlag_TXIS(hdl->i2cHdl);
      break;

    case 7:
      LL_I2C_TransmitData8(hdl->i2cHdl, value);
      stat++;
      break;

    case 8:
      stat += LL_I2C_IsActiveFlag_STOP(hdl->i2cHdl);
      break;

    case 9:
      LL_I2C_ClearFlag_STOP(hdl->i2cHdl);
	    status = STSPIN32G4_OK;
      return status;

    default:
			status = STSPIN32G4_ERROR;
      return status;

    }

    if (LL_SYSTICK_IsActiveCounterFlag())
    {
      ticks--;
    }

    if(ticks == 0)
    {
			status = STSPIN32G4_TIMEOUT;
      return status;
    }
  }

#else

  uint32_t ticks = (STSPIN32G4_I2C_TIMEOUT * HAL_GetTickFreq()) / 1000 + 1;
  status = (STSPIN32G4_StatusTypeDef) HAL_I2C_Mem_Write(hdl->i2cHdl, STSPIN32G4_I2C_ADDR, (uint16_t) regAddr, 1, &value, 1, ticks);
  return status;

#endif

}

STSPIN32G4_StatusTypeDef STSPIN32G4_writeVerifyReg(STSPIN32G4_HandleTypeDef *hdl, uint8_t regAddr, uint8_t value)
{
  STSPIN32G4_StatusTypeDef status;
  uint8_t i2cReg = 0;

  status = STSPIN32G4_writeReg(hdl, regAddr, value);

  if (status == STSPIN32G4_OK)
  {
    status = STSPIN32G4_readReg(hdl, regAddr, &i2cReg);
  }

  if (status == STSPIN32G4_OK)
  {
    if (value != i2cReg)
    {
      status = STSPIN32G4_ERROR;
    }
  }

  return status;
}

STSPIN32G4_StatusTypeDef STSPIN32G4_set3V3(STSPIN32G4_HandleTypeDef *hdl, bool enabled)
{
  STSPIN32G4_StatusTypeDef status;
  uint8_t i2cReg = 0;

  if (hdl == NULL)
  {
    return STSPIN32G4_ERROR;
  }

  status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_POWMNG, &i2cReg);

  if (status == STSPIN32G4_OK)
  {
    status = STSPIN32G4_unlockReg(hdl);
  }

  if (status == STSPIN32G4_OK)
  {
    if (enabled)
    {
      i2cReg &= ~STSPIN32G4_I2C_REG3V3_DIS;
    }
    else
    {
      i2cReg |= STSPIN32G4_I2C_REG3V3_DIS;
    }

    status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_POWMNG, i2cReg);
  }

  if (status == STSPIN32G4_OK)
  {
    STSPIN32G4_lockReg(hdl);
    return status;
  }
  else
  {
    return STSPIN32G4_lockReg(hdl);
  }
}

STSPIN32G4_StatusTypeDef STSPIN32G4_get3V3(STSPIN32G4_HandleTypeDef *hdl, bool *enabled)
{
  STSPIN32G4_StatusTypeDef status;
  uint8_t i2cReg = 0;

  if (hdl == NULL)
  {
    return STSPIN32G4_ERROR;
  }

  if (enabled == NULL)
  {
    return STSPIN32G4_ERROR;
  }

  status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_POWMNG, &i2cReg);
  *enabled = (i2cReg & STSPIN32G4_I2C_REG3V3_DIS) ? false : true;

  return status;
}

STSPIN32G4_StatusTypeDef STSPIN32G4_setVCC(STSPIN32G4_HandleTypeDef *hdl, STSPIN32G4_confVCC vcc)
{
  STSPIN32G4_StatusTypeDef status;
  uint8_t i2cReg = 0;

  if (hdl == NULL)
  {
    return STSPIN32G4_ERROR;
  }

  status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_POWMNG, &i2cReg);

  if (status == STSPIN32G4_OK)
  {
    status = STSPIN32G4_unlockReg(hdl);
  }

  if (status == STSPIN32G4_OK)
  {
    switch (vcc.voltage)
    {
      case _EXT:
        i2cReg |= STSPIN32G4_I2C_VCC_DIS;
        break;

      case _8V:
        i2cReg &= ~(STSPIN32G4_I2C_VCC_DIS | STSPIN32G4_I2C_VCC_VAL_3);
        i2cReg |= STSPIN32G4_I2C_VCC_VAL_0;
        break;

      case _10V:
        i2cReg &= ~(STSPIN32G4_I2C_VCC_DIS | STSPIN32G4_I2C_VCC_VAL_3);
        i2cReg |= STSPIN32G4_I2C_VCC_VAL_1;
        break;

      case _12V:
        i2cReg &= ~(STSPIN32G4_I2C_VCC_DIS | STSPIN32G4_I2C_VCC_VAL_3);
        i2cReg |= STSPIN32G4_I2C_VCC_VAL_2;
        break;

      case _15V:
        i2cReg &= ~STSPIN32G4_I2C_VCC_DIS;
        i2cReg |= STSPIN32G4_I2C_VCC_VAL_3;
        break;
      default:
        status = STSPIN32G4_ERROR;
        break;
    }
  }

  if (status == STSPIN32G4_OK)
  {
    status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_POWMNG, i2cReg);
  }

  if (status == STSPIN32G4_OK)
  {
    status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_NFAULT, &i2cReg);
  }

  if (status == STSPIN32G4_OK) // configuration of nFAULT pin
  {
    if (vcc.useNFAULT)
    {
      i2cReg |= STSPIN32G4_I2C_VCC_UVLO_FLT;
    }
    else
    {
      i2cReg &= ~STSPIN32G4_I2C_VCC_UVLO_FLT;
    }

    status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_NFAULT, i2cReg);
  }

  if (status == STSPIN32G4_OK)
  {
    status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_READY, &i2cReg);
  }

  if (status == STSPIN32G4_OK) // configuration of READY pin
  {
    if (vcc.useREADY)
    {
      i2cReg |= STSPIN32G4_I2C_VCC_UVLO_RDY;
    }
    else
    {
      i2cReg &= ~STSPIN32G4_I2C_VCC_UVLO_RDY;
    }

    status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_READY, i2cReg);
  }

  if (status == STSPIN32G4_OK)
  {
    STSPIN32G4_lockReg(hdl);
    return status;
  }
  else
  {
    return STSPIN32G4_lockReg(hdl);
  }
}

STSPIN32G4_StatusTypeDef STSPIN32G4_getVCC(STSPIN32G4_HandleTypeDef *hdl, STSPIN32G4_confVCC *vcc)
{
  STSPIN32G4_StatusTypeDef status;
  uint8_t i2cReg = 0;

  if (hdl == NULL)
  {
    return STSPIN32G4_ERROR;
  }

  if (vcc == NULL)
  {
    return STSPIN32G4_ERROR;
  }

  status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_POWMNG, &i2cReg);

  if (status == STSPIN32G4_OK)
  {
    if (i2cReg & STSPIN32G4_I2C_VCC_DIS)
    {
      vcc->voltage = _EXT;
    }
    else
    {
      switch (i2cReg & STSPIN32G4_I2C_VCC_VAL_3)
      {
        case STSPIN32G4_I2C_VCC_VAL_0:
          vcc->voltage = _8V;
          break;

        case STSPIN32G4_I2C_VCC_VAL_1:
          vcc->voltage = _10V;
          break;

        case STSPIN32G4_I2C_VCC_VAL_2:
          vcc->voltage = _12V;
          break;

        case STSPIN32G4_I2C_VCC_VAL_3:
          vcc->voltage = _15V;
          break;

        default:
          status = STSPIN32G4_ERROR;
          break;
      }
    }
  }

  if (status == STSPIN32G4_OK)
  {
    status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_NFAULT, &i2cReg);
    vcc->useNFAULT = (i2cReg & STSPIN32G4_I2C_VCC_UVLO_FLT) ? true : false;
  }

  if (status == STSPIN32G4_OK)
  {
    status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_READY, &i2cReg);
    vcc->useREADY = (i2cReg & STSPIN32G4_I2C_VCC_UVLO_RDY) ? true : false;
  }

  return status;
}

STSPIN32G4_StatusTypeDef STSPIN32G4_setInterlocking(STSPIN32G4_HandleTypeDef *hdl, bool enabled)
{
  STSPIN32G4_StatusTypeDef status;
  uint8_t i2cReg = 0;

  if (hdl == NULL)
  {
    return STSPIN32G4_ERROR;
  }

  status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_LOGIC, &i2cReg);

  if (status == STSPIN32G4_OK)
  {
    status = STSPIN32G4_unlockReg(hdl);
  }

  if (status == STSPIN32G4_OK)
  {
    if (enabled)
    {
      i2cReg |= STSPIN32G4_I2C_ILOCK;
    }
    else
    {
      i2cReg &= ~STSPIN32G4_I2C_ILOCK;
    }

    status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_LOGIC, i2cReg);
  }

  if (status == STSPIN32G4_OK)
  {
    STSPIN32G4_lockReg(hdl);
    return status;
  }
  else
  {
    return STSPIN32G4_lockReg(hdl);
  }
}

STSPIN32G4_StatusTypeDef STSPIN32G4_getInterlocking(STSPIN32G4_HandleTypeDef *hdl, bool *enabled)
{
  STSPIN32G4_StatusTypeDef status;
  uint8_t i2cReg = 0;

  if (hdl == NULL)
  {
    return STSPIN32G4_ERROR;
  }

  if (enabled == NULL)
  {
    return STSPIN32G4_ERROR;
  }

  status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_LOGIC, &i2cReg);
  *enabled = (i2cReg & STSPIN32G4_I2C_ILOCK) ? true : false;

  return status;
}

STSPIN32G4_StatusTypeDef STSPIN32G4_setMinimumDeadTime(STSPIN32G4_HandleTypeDef *hdl, bool enabled)
{
  STSPIN32G4_StatusTypeDef status;
  uint8_t i2cReg = 0;

  if (hdl == NULL)
  {
    return STSPIN32G4_ERROR;
  }

  status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_LOGIC, &i2cReg);

  if (status == STSPIN32G4_OK)
  {
    status = STSPIN32G4_unlockReg(hdl);
  }

  if (status == STSPIN32G4_OK)
  {
    if (enabled)
    {
      i2cReg |= STSPIN32G4_I2C_DTMIN;
    }
    else
    {
      i2cReg &= ~STSPIN32G4_I2C_DTMIN;
    }

    status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_LOGIC, i2cReg);
  }

  if (status == STSPIN32G4_OK)
  {
    STSPIN32G4_lockReg(hdl);
    return status;
  }
  else
  {
    return STSPIN32G4_lockReg(hdl);
  }
}

STSPIN32G4_StatusTypeDef STSPIN32G4_getMinimumDeadTime(STSPIN32G4_HandleTypeDef *hdl, bool *enabled)
{
  STSPIN32G4_StatusTypeDef status;
  uint8_t i2cReg = 0;

  if (hdl == NULL)
  {
    return STSPIN32G4_ERROR;
  }

  if (enabled == NULL)
  {
    return STSPIN32G4_ERROR;
  }

  status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_LOGIC, &i2cReg);
  *enabled = (i2cReg & STSPIN32G4_I2C_DTMIN) ? true : false;

  return status;
}

STSPIN32G4_StatusTypeDef STSPIN32G4_setVDSP(STSPIN32G4_HandleTypeDef *hdl, STSPIN32G4_confVDSP vdsp)
{
  STSPIN32G4_StatusTypeDef status;
  uint8_t i2cReg = 0;

  if (hdl == NULL)
  {
    return STSPIN32G4_ERROR;
  }

  status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_LOGIC, &i2cReg);

  if (status == STSPIN32G4_OK)
  {
    status = STSPIN32G4_unlockReg(hdl);
  }

  if (status == STSPIN32G4_OK)
  {
    i2cReg &= ~STSPIN32G4_I2C_VDS_P_DEG_3;
    switch (vdsp.deglitchTime)
    {
      case _6us:
        i2cReg |= STSPIN32G4_I2C_VDS_P_DEG_0;
        break;

      case _4us:
        i2cReg |= STSPIN32G4_I2C_VDS_P_DEG_1;
        break;

      case _3us:
        i2cReg |= STSPIN32G4_I2C_VDS_P_DEG_2;
        break;

      case _2us:
        i2cReg |= STSPIN32G4_I2C_VDS_P_DEG_3;
        break;

      default:
        status = STSPIN32G4_ERROR;
        break;
    }
  }
  if (status == STSPIN32G4_OK)
  {
    status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_LOGIC, i2cReg);
  }

  if (status == STSPIN32G4_OK)
  {
    status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_NFAULT, &i2cReg);
  }

  if (status == STSPIN32G4_OK) // configure nFault signaling
  {
    if (vdsp.useNFAULT)
    {
      i2cReg |= STSPIN32G4_I2C_VDS_P_FLT;
    }
    else
    {
      i2cReg &= ~STSPIN32G4_I2C_VDS_P_FLT;
    }

    status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_NFAULT, i2cReg);
  }

  if (status == STSPIN32G4_OK)
  {
    STSPIN32G4_lockReg(hdl);
    return status;
  }
  else
  {
    return STSPIN32G4_lockReg(hdl);
  }
}

STSPIN32G4_StatusTypeDef STSPIN32G4_getVDSP(STSPIN32G4_HandleTypeDef *hdl, STSPIN32G4_confVDSP *vdsp)
{
  STSPIN32G4_StatusTypeDef status;
  uint8_t i2cReg = 0;

  if (hdl == NULL)
  {
    return STSPIN32G4_ERROR;
  }

  if (vdsp == NULL)
  {
    return STSPIN32G4_ERROR;
  }

  status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_LOGIC, &i2cReg);

  if (status == STSPIN32G4_OK)
  {
    switch (i2cReg & STSPIN32G4_I2C_VDS_P_DEG_3)
    {
      case STSPIN32G4_I2C_VDS_P_DEG_0:
        vdsp->deglitchTime = _6us;
        break;

      case STSPIN32G4_I2C_VDS_P_DEG_1:
        vdsp->deglitchTime = _4us;
        break;

      case STSPIN32G4_I2C_VDS_P_DEG_2:
        vdsp->deglitchTime = _3us;
        break;

      case STSPIN32G4_I2C_VDS_P_DEG_3:
        vdsp->deglitchTime = _2us;
        break;

      default:
        status = STSPIN32G4_ERROR;
        break;
    }
  }

  if (status == STSPIN32G4_OK)
  {
    status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_NFAULT, &i2cReg);
    vdsp->useNFAULT = (i2cReg & STSPIN32G4_I2C_VDS_P_FLT) ? true : false;
  }

  return status;
}

STSPIN32G4_StatusTypeDef STSPIN32G4_setTHSD(STSPIN32G4_HandleTypeDef *hdl, STSPIN32G4_confTHSD thsd)
{
  STSPIN32G4_StatusTypeDef status;
  uint8_t i2cReg = 0;

  if (hdl == NULL)
  {
    return STSPIN32G4_ERROR;
  }

  status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_NFAULT, &i2cReg);

  if (status == STSPIN32G4_OK)
  {
    status = STSPIN32G4_unlockReg(hdl);
  }

  if (status == STSPIN32G4_OK)
  {
    if (thsd.useNFAULT)
    {
      i2cReg |= STSPIN32G4_I2C_THSD_FLT;
    }
    else
    {
      i2cReg &= ~STSPIN32G4_I2C_THSD_FLT;
    }

    status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_NFAULT, i2cReg);
  }

  if (status == STSPIN32G4_OK)
  {
    status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_READY, &i2cReg);
  }

  if (status == STSPIN32G4_OK)
  {
    if (thsd.useREADY)
    {
      i2cReg |= STSPIN32G4_I2C_THSD_RDY;
    }
    else
    {
      i2cReg &= ~STSPIN32G4_I2C_THSD_RDY;
    }

    status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_READY, i2cReg);
  }

  if (status == STSPIN32G4_OK)
  {
    STSPIN32G4_lockReg(hdl);
    return status;
  }
  else
  {
    return STSPIN32G4_lockReg(hdl);
  }
}

STSPIN32G4_StatusTypeDef STSPIN32G4_getTHSD(STSPIN32G4_HandleTypeDef *hdl, STSPIN32G4_confTHSD *thsd)
{
  STSPIN32G4_StatusTypeDef status;
  uint8_t i2cReg = 0;

  if (hdl == NULL)
  {
    return STSPIN32G4_ERROR;
  }

  if (thsd == NULL)
  {
    return STSPIN32G4_ERROR;
  }

  status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_NFAULT, &i2cReg);
  thsd->useNFAULT = (i2cReg & STSPIN32G4_I2C_THSD_FLT) ? true : false;

  if (status == STSPIN32G4_OK)
  {
    status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_READY, &i2cReg);
    thsd->useREADY = (i2cReg & STSPIN32G4_I2C_THSD_RDY) ? true : false;
  }

  return status;

}

STSPIN32G4_StatusTypeDef STSPIN32G4_clearFaults(STSPIN32G4_HandleTypeDef *hdl)
{
  uint8_t i2cReg = 0xff;

  if (hdl == NULL)
  {
    return STSPIN32G4_ERROR;
  }

  return STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_CLEAR, i2cReg);
}

STSPIN32G4_StatusTypeDef STSPIN32G4_reset(STSPIN32G4_HandleTypeDef *hdl)
{
  STSPIN32G4_StatusTypeDef status;
  uint8_t i2cReg = 0xff;

  if (hdl == NULL)
  {
    return STSPIN32G4_ERROR;
  }

  status = STSPIN32G4_unlockReg(hdl);

  if (status == STSPIN32G4_OK)
  {
    status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_RESET, i2cReg);
  }

  return status;
}

STSPIN32G4_StatusTypeDef STSPIN32G4_standby(STSPIN32G4_HandleTypeDef *hdl, bool enableStbyReg)
{
  STSPIN32G4_StatusTypeDef status;
  uint8_t i2cReg = 0;
  uint32_t tickFreq;
  uint32_t ticks;

#if defined(USE_FULL_LL_DRIVER)

  LL_RCC_ClocksTypeDef RCC_Clocks;
  LL_RCC_GetSystemClocksFreq(&RCC_Clocks);

  tickFreq = RCC_Clocks.HCLK_Frequency /  (SysTick->LOAD + 1);

  if (LL_SYSTICK_GetClkSource() == LL_SYSTICK_CLKSOURCE_HCLK_DIV8)
  {
    tickFreq /= 8;
  }

#else

  uint32_t tickStart;
  tickFreq = HAL_GetTickFreq() / 1000; // in kHz to have ms base

#endif

  if (hdl == NULL)
  {
    return STSPIN32G4_ERROR;
  }

  status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_POWMNG, &i2cReg);

  if (status == STSPIN32G4_OK)
  {
    status = STSPIN32G4_unlockReg(hdl);
  }

  if (status == STSPIN32G4_OK) // configure the Standby regulator
  {
    if (enableStbyReg)
    {
      i2cReg |= STSPIN32G4_I2C_STBY_REG_EN;
    }
    else
    {
      i2cReg &= ~STSPIN32G4_I2C_STBY_REG_EN;
    }

    status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_POWMNG, i2cReg);
  }

  if (status == STSPIN32G4_OK) // create backup of the READY register
  {
    status = STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_READY, &STSPIN32G4_bkupREADY);
  }

  if (status == STSPIN32G4_OK) // set only STBY_RDY to signal the exit from standby
  {
    i2cReg = STSPIN32G4_I2C_STBY_RDY;
    status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_READY, i2cReg);
  }

  if (status == STSPIN32G4_OK) // WAKE line low
  {
#if defined(USE_FULL_LL_DRIVER)

    LL_GPIO_ResetOutputPin(GD_WAKE_GPIO_Port, GD_WAKE_Pin);

#else

    HAL_GPIO_WritePin(GD_WAKE_GPIO_Port, GD_WAKE_Pin, GPIO_PIN_RESET);

#endif
  }

#ifdef STSPIN32G4_HSI16

  if (status == STSPIN32G4_OK)
  {
    if (enableStbyReg)
    {
      // Set HSI16 clock to reduce current consumption
#if defined(USE_FULL_LL_DRIVER)

      status = (STSPIN32G4_StatusTypeDef) LL_RCC_DeInit();

#else

      status = (STSPIN32G4_StatusTypeDef) HAL_RCC_DeInit();

#endif
    }
  }

#endif

  // request to enter standby
  if (status == STSPIN32G4_OK)
  {
    i2cReg = 0x01;
    status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_STBY, i2cReg);
  }

  ticks = (1 * tickFreq) / 1000 + 1;

#if defined(USE_FULL_LL_DRIVER)

  if (LL_GPIO_IsInputPinSet(GD_READY_GPIO_Port, GD_READY_Pin))
  {
    LL_GPIO_SetOutputPin(GD_WAKE_GPIO_Port, GD_WAKE_Pin);
    status =  STSPIN32G4_ERROR;
  }
  else
  {
    LL_SYSTICK_IsActiveCounterFlag();

    while (!LL_GPIO_IsInputPinSet(GD_READY_GPIO_Port, GD_READY_Pin))
    {
        if (LL_SYSTICK_IsActiveCounterFlag())
        {
          ticks--;
        }

        if (ticks == 0)
        {
          status = STSPIN32G4_TIMEOUT;
          break;
        }
    }
  }

#else

  if (HAL_GPIO_ReadPin(GD_READY_GPIO_Port, GD_READY_Pin) == GPIO_PIN_SET)
  {
    HAL_GPIO_WritePin(GD_WAKE_GPIO_Port, GD_WAKE_Pin, GPIO_PIN_SET);
    status =  STSPIN32G4_ERROR;
  }
  else
  {
    tickStart = HAL_GetTick();
    while (HAL_GPIO_ReadPin(GD_READY_GPIO_Port, GD_READY_Pin) == GPIO_PIN_RESET)
    {
      if ((HAL_GetTick() - tickStart) > ticks) // expected time is 100us
      {
        status = STSPIN32G4_TIMEOUT;
        break;
      }
    }
  }

#endif

  // wait 1ms and check possible exit from standby
#if defined(USE_FULL_LL_DRIVER)

  LL_mDelay(ticks);
  if (!LL_GPIO_IsInputPinSet(GD_READY_GPIO_Port, GD_READY_Pin) ||
       !LL_GPIO_IsInputPinSet(GD_NFAULT_GPIO_Port, GD_NFAULT_Pin))
  {
    status = STSPIN32G4_ERROR;
  }

#else

  HAL_Delay(ticks);
  if ((HAL_GPIO_ReadPin(GD_READY_GPIO_Port, GD_READY_Pin) != GPIO_PIN_SET) ||
      (HAL_GPIO_ReadPin(GD_NFAULT_GPIO_Port, GD_NFAULT_Pin) != GPIO_PIN_SET))
  {
    status = STSPIN32G4_ERROR;
  }

#endif

#ifdef STSPIN32G4_HSI16

  // if the driver failed to enter standby clock is reconfigured
  if (status != STSPIN32G4_OK)
  {
    SystemClock_Config();  // restore clock configuration
  }

#endif

  return status;
}

STSPIN32G4_StatusTypeDef STSPIN32G4_wakeup(STSPIN32G4_HandleTypeDef *hdl, uint8_t timeout_ms)
{
  STSPIN32G4_StatusTypeDef status;
  STSPIN32G4_statusRegTypeDef statReg;
  uint32_t tickFreq;
  uint32_t ticks;

#if defined(USE_FULL_LL_DRIVER)

  LL_RCC_ClocksTypeDef RCC_Clocks;
  LL_RCC_GetSystemClocksFreq(&RCC_Clocks);

  tickFreq = RCC_Clocks.HCLK_Frequency /  (SysTick->LOAD + 1);

  if (LL_SYSTICK_GetClkSource() == LL_SYSTICK_CLKSOURCE_HCLK_DIV8)
  {
    tickFreq /= 8;
  }

#else

  uint32_t tickStart;
  tickFreq = HAL_GetTickFreq() / 1000; // in kHz to have ms base

#endif


  if (hdl == NULL)
  {
    return STSPIN32G4_ERROR;
  }

#if defined(USE_FULL_LL_DRIVER)

  LL_GPIO_SetOutputPin(GD_WAKE_GPIO_Port, GD_WAKE_Pin);

#else

  HAL_GPIO_WritePin(GD_WAKE_GPIO_Port, GD_WAKE_Pin, GPIO_PIN_SET);

#endif

  if (timeout_ms < 4)
  {
    timeout_ms = 4;  // The soft start is expected to take 4ms
  }

  ticks = (timeout_ms * tickFreq) / 1000 + 1;

#if defined(USE_FULL_LL_DRIVER)

  LL_SYSTICK_IsActiveCounterFlag();

  while (!LL_GPIO_IsInputPinSet(GD_READY_GPIO_Port, GD_READY_Pin))
  {
      if (LL_SYSTICK_IsActiveCounterFlag())
      {
        ticks--;
      }

      if (ticks == 0)
      {
        status = STSPIN32G4_TIMEOUT;
        break;
      }
  }

#else

  tickStart = HAL_GetTick();

  while (HAL_GPIO_ReadPin(GD_READY_GPIO_Port, GD_READY_Pin) == GPIO_PIN_RESET)
  {
    if ((HAL_GetTick() - tickStart) > ticks)
    {
      status = STSPIN32G4_TIMEOUT;
      break;
    }
  }

#endif

#ifdef STSPIN32G4_HSI16

  if (status == STSPIN32G4_OK)
  {
    SystemClock_Config();
  }

#endif

  // Restore READY register
  if (status == STSPIN32G4_OK)
  {
    status = STSPIN32G4_writeReg(hdl, STSPIN32G4_I2C_READY, STSPIN32G4_bkupREADY);
  }

  if (status == STSPIN32G4_OK)
  {
    status = STSPIN32G4_lockReg(hdl);
  }

  if (status == STSPIN32G4_OK)
  {
    status = STSPIN32G4_getStatus(hdl, &statReg);
  }

  if (status == STSPIN32G4_OK)
  {
    if (statReg.reset == 1)
    {
      status = STSPIN32G4_ERROR;
    }
  }

  return status;
}

STSPIN32G4_StatusTypeDef STSPIN32G4_getStatus(STSPIN32G4_HandleTypeDef *hdl, STSPIN32G4_statusRegTypeDef *status)
{
  if (hdl == NULL)
  {
    return STSPIN32G4_ERROR;
  }

  if (status == NULL)
  {
    return STSPIN32G4_ERROR;
  }

  return STSPIN32G4_readReg(hdl, STSPIN32G4_I2C_STATUS, (uint8_t *)status);
}

