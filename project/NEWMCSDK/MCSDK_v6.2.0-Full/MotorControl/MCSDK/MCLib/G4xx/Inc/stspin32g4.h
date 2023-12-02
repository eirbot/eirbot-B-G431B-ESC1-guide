/**
  ******************************************************************************
  * @file    stspin32g4.h
  * @author  IPC Application Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          ICSSTSPIN32G4 driver library.
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
  * @ingroup stspin32g4_STSPIN32G4_DriverLib
  */
/**
  *
  * @defgroup stspin32g4_STSPIN32G4_DriverLib STSPIN32G4 Driver Library
  * @{
  * 
  * <h2>Operation</h2>
  * The STSPIN32G4 is a system in package integrating a triple high-performance
  * STSPIN32G4f-bridge gate driver with a rich set of programmable features
  * and one mixed signal STM32G431 microcontroller.
  * This library provides functions to configure STSPIN32G4 gate driver
  * but does not include functions to generate PWM signals for MOSFETs driving
  * 
  * @pre This library requires at least HAL drivers 1.3.0 or Low-layer (LL) drivers 1.5.0
  * 
  * The library should be included in an application as follows:
  * @code
  * #include <stspin32g4.h>
  * @endcode
  * 
  * <h3>Opening the driver</h3>
  * @code
  * STSPIN32G4_HandleTypeDef hdlG4;
  * 
  * STSPIN32G4_StatusTypeDef res = STSPIN32G4_init(&hdlG4);
  * 
  * if(res != STSPIN32G4_OK)
  *     STSPIN32G4_deInit(&hdlG4);
  * @endcode
  * 
  * <h3>Example</h3>
  * The following code shows a minimal device configuration to operate the gate driver
  * @code
  * // Purge registers to default values
  * STSPIN32G4_reset(&hdlG4);
  * 
  * // Configure the Buck regulator at 12V and map VCC UVLO on NFAULT pin
  * STSPIN32G4_confVCC vcc = {.voltage = _12V, .useNFAULT=true, .useREADY=false};
  * STSPIN32G4_setVCC(&hdlG4, vcc);
  * 
  * // Configure VDSP protection with 4us deglitch time and map triggering on NFAULT pin
  * STSPIN32G4_confVDSP vdsp = {.deglitchTime=_4us, .useNFAULT=true};
  * STSPIN32G4_setVDSP(&hdlG4, vdsp);
  * 
  * // After reset a clearing of fault is needed to enable GHSx/GLSx outputs
  * STSPIN32G4_clearFaults(&hdlG4);
  * @endcode
  * <h3>Closing the driver</h3>
  * @code
  * // Reset the device to put GHSx/GLSx outputs in safe state
  * STSPIN32G4_reset(&hdlG4);
  * 
  * // Close the driver
  * STSPIN32G4_deInit(&hdlG4);
  * @endcode
  * 
  * <h3>Standby</h3>
  * 
  * @code
  * // Enter Standby enabling low quiescent regulator
  * if(STSPIN32G4_OK == STSPIN32G4_standby(&hdlG4, true))
  * {
  *   // Stay 1s in standby
  *   HAL_Delay(1000);  // or LL_mDelay(1000);
  * 
  *   // Wakeup the driver waiting 4ms for completion of VCC soft-start
  *   STSPIN32G4_wakeup(&hdlG4, 4);
  * }
  * else
  * {
  *   // Consumption from 3.3V supply is likely too high
  * }
  * @endcode
  */

/**
  * @defgroup stspin32g4_Basic Basic
  * @brief Basic function and definitions to work with STSPIN32G4
  * @{
  * @}
  */

/**
  * @defgroup stspin32g4_PowerManagement Power Management
  * @brief Function and definitions to manage regulators embedded in STSPIN32G4 and standby
  * @{
  * @}
  */

/**
  * @defgroup stspin32g4_Protections Protections
  * @brief Function and definitions to configure protection features of STSPIN32G4
  * @{
  * @}
  */

/**
  @defgroup stspin32g4_I2c Driver Registers
  @brief Function and definitions to directly access I2C registers of STSPIN32G4
  @{
  @}
  */

/** @}  */

#ifndef STSPIN32G4
#define STSPIN32G4

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

/**
@ingroup stspin32g4_I2c
@{
  */

#define STSPIN32G4_I2C_LOCKUSEPARANOID  /**< Flag to check status register lock bit in functions STSPIN32G4_lockReg() and STSPIN32G4_unlockReg() */

#define STSPIN32G4_I2C_ADDR          (0x8E)  /**< I2C address of STSPIN32G4 */

#define STSPIN32G4_I2C_POWMNG        (0x01)  /**< Address of register POWMNG */
#define STSPIN32G4_I2C_REG3V3_DIS    (1<<6)  /**< Bit REG3V3_DIS */
#define STSPIN32G4_I2C_VCC_DIS       (1<<5)  /**< Bit VCC_DIS */
#define STSPIN32G4_I2C_STBY_REG_EN   (1<<4)  /**< Bit STBY_REG_EN */
#define STSPIN32G4_I2C_VCC_VAL_0     (0)     /**< Bits VCC_VAL as 00, (VCC=8V) */
#define STSPIN32G4_I2C_VCC_VAL_1     (1)     /**< Bits VCC_VAL as 01, (VCC=10V) */
#define STSPIN32G4_I2C_VCC_VAL_2     (2)     /**< Bits VCC_VAL as 10, (VCC=12V) */
#define STSPIN32G4_I2C_VCC_VAL_3     (3)     /**< Bits VCC_VAL as 11, (VCC=15V) */

#define STSPIN32G4_I2C_LOGIC         (0x02)  /**< Address of register LOGIC */
#define STSPIN32G4_I2C_VDS_P_DEG_0   (0<<2)  /**< Bits VDS_P_DEG as 00, (6us deglitch) */
#define STSPIN32G4_I2C_VDS_P_DEG_1   (1<<2)  /**< Bits VDS_P_DEG as 01, (4us deglitch) */
#define STSPIN32G4_I2C_VDS_P_DEG_2   (2<<2)  /**< Bits VDS_P_DEG as 10, (3us deglitch) */
#define STSPIN32G4_I2C_VDS_P_DEG_3   (3<<2)  /**< Bits VDS_P_DEG as 11, (2us deglitch) */
#define STSPIN32G4_I2C_DTMIN         (1<<1)  /**< Bit DTMIN */
#define STSPIN32G4_I2C_ILOCK         (1<<0)  /**< Bit ILOCK */

#define STSPIN32G4_I2C_READY         (0x07)  /**< Address of register READY */
#define STSPIN32G4_I2C_STBY_RDY      (1<<3)  /**< Bit STBY_RDY */
#define STSPIN32G4_I2C_THSD_RDY      (1<<1)  /**< Bit THSD_RDY */
#define STSPIN32G4_I2C_VCC_UVLO_RDY  (1<<0)  /**< Bit VCC_UVLO_RDY */

#define STSPIN32G4_I2C_NFAULT        (0x08)  /**< Address of register NFAULT */
#define STSPIN32G4_I2C_RESET_FLT     (1<<3)  /**< Bit RESET_FLT */
#define STSPIN32G4_I2C_VDS_P_FLT     (1<<2)  /**< Bit VDS_P_FLT */
#define STSPIN32G4_I2C_THSD_FLT      (1<<1)  /**< Bit THSD_FLT */
#define STSPIN32G4_I2C_VCC_UVLO_FLT  (1<<0)  /**< Bit VCC_UVLO_FLT */

#define STSPIN32G4_I2C_CLEAR         (0x09)  /**< Address of register CLEAR */

#define STSPIN32G4_I2C_STBY          (0x0A)  /**< Address of register STBY */

#define STSPIN32G4_I2C_LOCK          (0x0B)  /**< Address of register LOCK */

#define STSPIN32G4_I2C_RESET         (0x0C)  /**< Address of register RESET */

#define STSPIN32G4_I2C_STATUS        (0x80)  /**< Address of register STATUS */

/**
@}
  */

/**
@ingroup stspin32g4_PowerManagement
@{
  */

#define STSPIN32G4_HSI16  /**< Flag to automatically reduce system clock frequency before entering standby in function STSPIN32G4_standby() */

/**
@brief Possible results of library functions.
  */
typedef enum
{
  STSPIN32G4_OK       = 0,    /**< Operation successfully completed. */
  STSPIN32G4_ERROR    = 1,    /**< Operation failed. */
  STSPIN32G4_BUSY     = 2,    /**< The operation cannot be performed since a needed resource is not ready or available. */
  STSPIN32G4_TIMEOUT  = 3     /**< A timeout occurred before operation ended. */
} STSPIN32G4_StatusTypeDef;

/**
@brief Configuration for VCC voltage to be used with functions STSPIN32G4_setVCC() and STSPIN32G4_getVCC().
@details The STSPIN32G4 embeds a Buck converter to generate the supply voltage for the gate drivers, VCC,
starting from the motor supply voltage.
\n Four different VCC output values can be selected.
\n The Under Voltage Lock Out (UVLO) condition of VCC can be signaled via pins nFAULT and READY.
  */
typedef struct
{
  /**
  @brief VCC voltage selection.
  @details When VCC set point is changed the Buck converter performs a soft-start ramp.
    */
  enum valVCC
  {
    _EXT = 0,       /**< Buck converter is disabled and VCC should be supplied externally. */
    _8V,            /**< VCC = 8V generetad by Buck converter. Deafult value. */
    _10V,           /**< VCC = 10V generetad by Buck converter. */
    _12V,           /**< VCC = 12V generetad by Buck converter. */
    _15V            /**< VCC = 15V generetad by Buck converter. */
  } voltage;    /**< Value assigned to VCC voltage */
  bool useNFAULT; /**< If true the nNFAULT pin goes low when device is in VCC UVLO condition. */
  bool useREADY;  /**< If true the READY pin goes low when device is in VCC UVLO condition. */

} STSPIN32G4_confVCC;
/**
@}
  */

/**
@brief Configuration for VDS monitoring protection to be used with functions STSPIN32G4_setVDSP() and STSPIN32G4_getVDSP().
@details The STSPIN32G4 embeds a circuitry which measures the voltage between the drain and the source of each MOSFET (VDS)
and compares it with a specified threshold set by voltage of SCREF pin.
In case an overvoltage of one MOSFET is detected, all gate driver outputs GHSx and GLSx go low whatever the driver inputs INHx and INLx.
\n The protection provides a deglitch filtering with configurable value.
\n Triggering of the protection can be signaled via nFAULT pin.
\n Function STSPIN32G4_clearFaults() can be used to clear the fault condition and make the device operative again.
@ingroup stspin32g4_Protections
  */
typedef struct
{
  /**
  @brief Deglitch filtering time for VDS protection.
  @ingroup stspin32g4_Protections
    */
  enum degVDSP
  {
    _6us = 0,           /**< 6 micro seconds. Deafult value. */
    _4us,               /**< 4 micro seconds */
    _3us,               /**< 3 micro seconds */
    _2us                /**< 2 micro seconds */
  } deglitchTime;   /**< Value assigned to deglitch filtering time of VDS protection */
  bool useNFAULT;   /**< If true the nNFAULT pin goes low in case of VDS protection triggering. */
} STSPIN32G4_confVDSP;

/**
@brief Configuration for Thermal Shutdown signaling to be used with functions STSPIN32G4_setTHSD() and STSPIN32G4_getTHSD().
@details The STSPIN32G4 embeds one Buck converter and one LDO linear regulator.
Both voltage regulators are protected in case of overheating by thermal shutdown.
\n The protection is always active and its triggering can be signaled via nFAULT and READY pins.
@ingroup stspin32g4_Protections
  */
typedef struct
{
  bool useNFAULT;       /**< If true the nFAULT pin goes low in case of Thermal Shutdown. */
  bool useREADY;        /**< If true the READY pin goes low in case of Thermal Shutdown. */
} STSPIN32G4_confTHSD;

/**
@brief Status register fileds to be used with function STSPIN32G4_getStatus()
@ingroup stspin32g4_Basic
  */
typedef struct
{
  uint8_t vccUvlo:1;    /**< If 1 the device is in VCC UVLO condition. While in VCC UVLO condition the device cannot drive MOSFETs*/
  uint8_t thsd:1;       /**< If 1 one voltage regulator is in Thermal Shutdown. While in Thermal Shutdown the device cannot drive MOSFETs*/
  uint8_t vdsp:1;       /**< If 1 the VDS protection triggered. Use STSPIN32G4_clearFaults() to make the device operative again. */
  uint8_t reset:1;      /**< If 1 the device performed a reset or power up. Use STSPIN32G4_clearFaults() to make the device operative. */
  uint8_t r1:1;         /**< Reserved. */
  uint8_t r2:1;         /**< Reserved. */
  uint8_t r3:1;         /**< Reserved. */
  uint8_t lock:1;       /**< If 1 the protected registers are locked and cannot be modified. */
} STSPIN32G4_statusRegTypeDef;

/**
@brief Handler of STSPIN32G4 driver to be used with all driver functions
@see STSPIN32G4_init() for example code
@ingroup stspin32g4_Basic
 */
typedef struct
{
#if defined(USE_FULL_LL_DRIVER)
  I2C_TypeDef *i2cHdl;          /**< Handler to i2c3 */
#else
  I2C_HandleTypeDef *i2cHdl;    /**< Handler to i2c3 */
#endif
} STSPIN32G4_HandleTypeDef;


/**
@brief Initialize the STSPIN32G4 driver
@pre The STSPIN32G4_HandleTypeDef structure must exist
and be persistent before this function can be called.
\n This function must also be called before any other driver APIs.
\n The driver uses I2C3 interface which must be already initialized.
\n Initialization example:
@code
STSPIN32G4_HandleTypeDef hdlG4;
STSPIN32G4_StatusTypeDef ret = STSPIN32G4_init(&hdlG4);
@endcode
@param [out] hdl Driver handler
@return STSPIN32G4_ERROR in case of failure otherwise STSPIN32G4_OK.
@ingroup stspin32g4_Basic
  */
STSPIN32G4_StatusTypeDef STSPIN32G4_init(STSPIN32G4_HandleTypeDef *hdl);

/**
@brief De-initialize the STSPIN32G4 driver
@param [in] hdl Driver handler
@return STSPIN32G4_ERROR in case of failure otherwise STSPIN32G4_OK.
@ingroup stspin32g4_Basic
  */
STSPIN32G4_StatusTypeDef STSPIN32G4_deInit(STSPIN32G4_HandleTypeDef *hdl);

/**
@brief Perform a reset. All registers will be set to their default values.
@param [in] hdl Driver handler
@return STSPIN32G4_ERROR in case of failure otherwise STSPIN32G4_OK.
@ingroup stspin32g4_Basic
  */
STSPIN32G4_StatusTypeDef STSPIN32G4_reset(STSPIN32G4_HandleTypeDef *hdl);

/**
@brief Get the status register
@see STSPIN32G4_statusRegTypeDef
@param [in] hdl Driver handler
@param [out] status Current value of the status register
@return STSPIN32G4_ERROR in case of failure otherwise STSPIN32G4_OK.
@ingroup stspin32g4_Basic
  */
STSPIN32G4_StatusTypeDef STSPIN32G4_getStatus(STSPIN32G4_HandleTypeDef *hdl, STSPIN32G4_statusRegTypeDef *status);

/**
@brief Enable or Disable the LDO linear regulator
@param [in] hdl Driver handler
@param [in] enabled If 1 the 3.3V supply is internally generated via the LDO linear regulator.
\n If 0 the LDO regulator is disabled and the 3.3V supply have to be provided externally.
@warning The device will perform a reset if the LDO is disabled without providing externally the 3.3V supply.
@return STSPIN32G4_ERROR in case of failure otherwise STSPIN32G4_OK.
@ingroup stspin32g4_PowerManagement
  */
STSPIN32G4_StatusTypeDef STSPIN32G4_set3V3(STSPIN32G4_HandleTypeDef *hdl, bool enabled);

/**
@brief Get current configuration of LDO linear regulator
@param [in] hdl Driver handler
@param [out] enabled Set to 1 if the internal LDO is active, 0 otherwise.
@return STSPIN32G4_ERROR in case of failure otherwise STSPIN32G4_OK.
@ingroup stspin32g4_PowerManagement
  */
STSPIN32G4_StatusTypeDef STSPIN32G4_get3V3(STSPIN32G4_HandleTypeDef *hdl, bool *enabled);

/**
@brief Configures the Buck converter and VCC UVLO signaling
@see STSPIN32G4_confVCC
@param [in] hdl Driver handler
@param [in] vcc VCC configuration to be applied
\n Example to set VCC at 12V and enable VCC UVLO singaling on NFAULT pin:
@code
STSPIN32G4_confVCC vcc = {.voltage = _12V, .useNFAULT=true, .useREADY=false};
STSPIN32G4_StatusTypeDef ret = STSPIN32G4_setVCC(&hdlG4, vcc);
@endcode
@return STSPIN32G4_ERROR in case of failure otherwise STSPIN32G4_OK.
@ingroup stspin32g4_PowerManagement
  **/
STSPIN32G4_StatusTypeDef STSPIN32G4_setVCC(STSPIN32G4_HandleTypeDef *hdl, STSPIN32G4_confVCC vcc);

/**
@brief Get current configuration of Buck converter and VCC UVLO signaling
@see STSPIN32G4_confVCC
@param [in] hdl Driver handler
@param [out] vcc Current VCC configuration
@return STSPIN32G4_ERROR in case of failure otherwise STSPIN32G4_OK.
@ingroup stspin32g4_PowerManagement
  */
STSPIN32G4_StatusTypeDef STSPIN32G4_getVCC(STSPIN32G4_HandleTypeDef *hdl, STSPIN32G4_confVCC *vcc);

/**
@brief Configures the Thermal Shutdown signaling
@see STSPIN32G4_confTHSD
@param [in] hdl Driver handler
@param [in] thsd Configuration of Thermal Shutdown signaling to be applied
@return STSPIN32G4_ERROR in case of failure otherwise STSPIN32G4_OK.
@ingroup stspin32g4_Protections
  */
STSPIN32G4_StatusTypeDef STSPIN32G4_setTHSD(STSPIN32G4_HandleTypeDef *hdl, STSPIN32G4_confTHSD thsd);

/**
@brief Get current configuration of the Thermal Shutdown signaling
@see STSPIN32G4_confTHSD
@param [in] hdl Driver handler
@param [out] thsd Current configuration of Thermal Shutdown signaling
@return STSPIN32G4_ERROR in case of failure otherwise STSPIN32G4_OK.
@ingroup stspin32g4_Protections
  */
STSPIN32G4_StatusTypeDef STSPIN32G4_getTHSD(STSPIN32G4_HandleTypeDef *hdl, STSPIN32G4_confTHSD *thsd);

/**
@brief Requests the device to enter standby mode
@details In standby mode the device turns off most of its internal circuitry
with inclusion of Buck converter, LDO linear regulator and gate drivers.
\n In standby the overall current consumption is reduced to ~15uA and the MCU can be supplied
via dedicated low quiescent linear regulator from main supply voltage.
@param [in] hdl Driver handler
@param [in] enableStbyReg If 1 the internal low quiescent linear regulator will be activated when entering standby mode
@warning Overall consumption from 3.3V supply should be reduced as much as possible before entering standby.
\n In case the consumption exceeds 5mA the device will automatically exit from standby performing a reset.
\n Disable all MCU peripherals and reduce system clock frequency before calling function STSPIN32G4_standby().
\n If flag STSPIN32G4_HSI16 is used, the driver will automatically reduce system clock frequency.
@return STSPIN32G4_ERROR in case of failure otherwise STSPIN32G4_OK.
@ingroup stspin32g4_PowerManagement
  */
STSPIN32G4_StatusTypeDef STSPIN32G4_standby(STSPIN32G4_HandleTypeDef *hdl, bool enableStbyReg);

/**
@brief Wake up from standby
@param [in] hdl Driver handler
@param [in] timeout_ms Timeout in milliseconds allowed for device wake up. The minimum value is 4.
@return STSPIN32G4_ERROR in case of failure
\n STSPIN32G4_TIMEOUT in case the timeout time elapsed before device returned operative
\n STSPIN32G4_OK in case of proper wake up
@ingroup stspin32g4_PowerManagement
  */
STSPIN32G4_StatusTypeDef STSPIN32G4_wakeup(STSPIN32G4_HandleTypeDef *hdl, uint8_t timeout_ms);

/**
@brief Configures the interlocking
@details The device integrates interlocking features that prevents the power stage from accidental cross conduction.
\n HS and LS MOSFET of same channel cannot be turned on simultaneously when interlocking is active.
@param [in] hdl Driver handler
@param [in] enabled If 1 the interlocking is enabled.
\n If 0 the interlocking is disabled.
@warning Disable the interlocking is potentially desruptive for power stage.
@return STSPIN32G4_ERROR in case of failure otherwise STSPIN32G4_OK.
@ingroup stspin32g4_Protections
  */
STSPIN32G4_StatusTypeDef STSPIN32G4_setInterlocking(STSPIN32G4_HandleTypeDef *hdl, bool enabled);

/**
@brief Get current configuration of the interlocking
@param [in] hdl Driver handler
@param [out] enabled If 1 the interlocking is enabled.
\n If 0 the interlocking is disabled.
@return STSPIN32G4_ERROR in case of failure otherwise STSPIN32G4_OK.
@ingroup stspin32g4_Protections
  */
STSPIN32G4_StatusTypeDef STSPIN32G4_getInterlocking(STSPIN32G4_HandleTypeDef *hdl, bool *enabled);

/**
@brief Configures the minimum deadtime
@details The device integrates a minimum deadtime features. When enabled a minimum delay is imposed
between the turn-off of one MOSFET and the turn-on of the complementary one.
@param [in] hdl Driver handler
@param [in] enabled If 1 the minimum deadtime is enabled
\n If 0 the the minimum deadtime is disabled.
\n The minimum deadtime is not added to the one imposed by the MCU
@return STSPIN32G4_ERROR in case of failure otherwise STSPIN32G4_OK.
@ingroup stspin32g4_Protections
  */
STSPIN32G4_StatusTypeDef STSPIN32G4_setMinimumDeadTime(STSPIN32G4_HandleTypeDef *hdl, bool enabled);

/**
@brief Get current configuration of the minimum deadtime
@param [in] hdl Driver handler
@param [out] enabled If 1 the minimum deadtime is enabled
\n If 0 the the minimum deadtime is disabled.
@return STSPIN32G4_ERROR in case of failure otherwise STSPIN32G4_OK.
@ingroup stspin32g4_Protections
  */
STSPIN32G4_StatusTypeDef STSPIN32G4_getMinimumDeadTime(STSPIN32G4_HandleTypeDef *hdl, bool *enabled);

/**
@brief Configures the VDS monitoring protection
@see STSPIN32G4_confVDSP
@param [in] hdl Driver handler.
@param [in] vdsp Settings of the VDS protection.
@note Example to set deglitch time to 4 microsenconds and enable signaling on NFAULT pin:
@code
STSPIN32G4_confVDSP vdsp = {.deglitchTime=_4us, .useNFAULT=true};
STSPIN32G4_StatusTypeDef ret = STSPIN32G4_setVDSP(&hdlG4, vdsp);
@endcode
@return STSPIN32G4_ERROR in case of failure otherwise STSPIN32G4_OK.
@ingroup stspin32g4_Protections
  */
STSPIN32G4_StatusTypeDef STSPIN32G4_setVDSP(STSPIN32G4_HandleTypeDef *hdl, STSPIN32G4_confVDSP vdsp);

/**
@brief Get current configuration for the VDS monitoring protection
@see STSPIN32G4_confVDSP
@param [in] hdl Driver handler.
@param [out] vdsp Current configuration for the VDS protection.
@return STSPIN32G4_ERROR in case of failure otherwise STSPIN32G4_OK.
@ingroup stspin32g4_Protections
  */
STSPIN32G4_StatusTypeDef STSPIN32G4_getVDSP(STSPIN32G4_HandleTypeDef *hdl, STSPIN32G4_confVDSP *vdsp);

/**
@brief Clear a fault condition
@param [in] hdl Driver handler.
@return STSPIN32G4_ERROR in case of failure otherwise STSPIN32G4_OK.
@ingroup stspin32g4_Basic
  */
STSPIN32G4_StatusTypeDef STSPIN32G4_clearFaults(STSPIN32G4_HandleTypeDef *hdl);

/**
@brief Lock protected registers
@param [in] hdl Driver handler.
@note With flag STSPIN32G4_I2C_LOCKUSEPARANOID the lock bit of status register will be tested to 1 at the end of locking.
@return STSPIN32G4_ERROR in case of failure otherwise STSPIN32G4_OK.
@ingroup stspin32g4_I2c
  */
STSPIN32G4_StatusTypeDef STSPIN32G4_lockReg(STSPIN32G4_HandleTypeDef *hdl);

/**
@brief Un-Lock protected registers
@param [in] hdl Driver handler.
@note With flag STSPIN32G4_I2C_LOCKUSEPARANOID the lock bit of status register will be tested to 0 at the end of unlocking.
@return STSPIN32G4_ERROR in case of failure otherwise STSPIN32G4_OK.
@ingroup stspin32g4_I2c
  */
STSPIN32G4_StatusTypeDef STSPIN32G4_unlockReg(STSPIN32G4_HandleTypeDef *hdl);

/**
@brief Read register at address @p regAddr
@param [in] hdl Driver handler.
@param [in] regAddr Address of the register to read.
@param [out] value Contains the register value.
@return STSPIN32G4_ERROR in case of failure otherwise STSPIN32G4_OK.
@ingroup stspin32g4_I2c
  */
STSPIN32G4_StatusTypeDef STSPIN32G4_readReg(STSPIN32G4_HandleTypeDef *hdl, uint8_t regAddr, uint8_t *value);

/**
@brief Write register at address @p regAddr
@param [in] hdl Driver handler.
@param [in] regAddr Address of the register to write.
@param [in] value Content to write in the register.
@warning Before writing a protected register use function STSPIN32G4_unlockReg()
@return STSPIN32G4_ERROR in case of failure otherwise STSPIN32G4_OK.
@ingroup stspin32g4_I2c
  */
STSPIN32G4_StatusTypeDef STSPIN32G4_writeReg(STSPIN32G4_HandleTypeDef *hdl, uint8_t regAddr, uint8_t value);

/**
@brief Write register at address @p regAddr then read back register and verify value matching
@param [in] hdl Driver handler.
@param [in] regAddr Address of the register to write.
@param [in] value Content to write in the register.
@warning Before writing a protected register use function STSPIN32G4_unlockReg()
@return STSPIN32G4_ERROR in case of failure otherwise STSPIN32G4_OK.
@ingroup stspin32g4_I2c
  */
STSPIN32G4_StatusTypeDef STSPIN32G4_writeVerifyReg(STSPIN32G4_HandleTypeDef *hdl, uint8_t regAddr, uint8_t value);


#endif //#define STSPIN32G4

