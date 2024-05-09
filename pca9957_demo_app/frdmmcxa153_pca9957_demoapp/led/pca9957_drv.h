/*
 * Copyright 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file pca9957_drv.h
 * @brief The pca9957_drv.h file describes the PCA9957 driver interface and structures.
 */

#ifndef PCA9957_DRV_H_
#define PCA9957_DRV_H_

/* Standard C Includes */
#include <stdint.h>

/* ISSDK Includes */
#include "sensor_io_i2c.h"
#include "sensor_io_spi.h"
#include "register_io_i2c.h"
#include "register_io_spi.h"
#include "pca9957.h"

/*--------------------------------
** Enum: LEDERRORTYPE
** @brief Led channel output error
** ------------------------------*/
typedef enum LEDERRORTYPE
{
	noError = 0x00,                  /* In normal operation and no errorn */
	shortCircuit = 0x01,             /* Detected LED short-circuit condition */
	openCircuit = 0x02,              /* Detected LED open-circuit condition */
}LedErrorType;

/*--------------------------------
** Enum: TEMPCONDITION
** @brief Operating temperature condition
** ------------------------------*/
typedef enum TEMPCONDITION
{
	underTemp = 0x00,                  /* under temperature condition */
	overTemp = 0x01,                   /* over temperature condition */
}TempCondition;

/*--------------------------------
** Enum: ERRORSTATE
** @brief Led channel output error state
** ------------------------------*/
typedef enum ERRORSTATE
{
	errorNotDetected = 0x00,                /* Error Not detected */
	errorDetected = 0x01,                   /* Error detected */
}ErrorState;

/*--------------------------------
** Enum: AUTOSWITCHOFF
** @brief Auto Led channel output off on error
** ------------------------------*/
typedef enum AUTO_SWITCHOFF
{
	autoDisableOn = 0x00,                  /* Turned off when open/short detected */
	autoDisableOff = 0x01,                 /* won’t be turned off when open/short detected */
}AutoSwitchOff;

/*--------------------------------
** Enum: SLEEPCONTROL
** @brief Normal mode / Low-power mode
** ------------------------------*/
typedef enum SLEEPCONTROL
{
	standByMode  = 0x00,                  /* Normal mode */
	lowPowerMode = 0x01,                  /* Low-power mode (Oscillator off)*/
}SleepControl;

/*--------------------------------
** Enum: GRADADJUSTMENT
** @brief Gradation (Linear/Exponential) adjustment
** ------------------------------*/
typedef enum GRADADJUSTMENT
{
	linearAdjustment = 0x00,                   /* Linear Adjustment */
	exponentialAdjustment = 0x01,              /* Exponential Adjustment */
}GrandAdjustment;

/*--------------------------------
** Enum: GRADDIMMBLINK
** @brief Gradation Dimming Blinking Operation
** ------------------------------*/
typedef enum GRADDIMMBLINK
{
	groupDimming = 0x00,                        /* Gradation Dimming */
	groupBlinking = 0x01,                       /* Gradation Blinking */
}GradDimmBlink;

/*--------------------------------
** Enum: GRADCONTROL
** @brief Gradation Control
** ------------------------------*/
typedef enum GRADCONTROL
{
	gradStop = 0x00,                        /* Gradation stop  */
	gradStart = 0x01,                       /* Gradation start */
}GradControl;

/*--------------------------------
** Enum: GRADOPERATION
** @brief Gradation Operation (Single / Continuous) shot
** ------------------------------*/
typedef enum GRADOPERATION
{
	singleShot = 0x00,                       /* Single shot operation */
	continuous = 0x01,                       /* Continuous operation  */
}GradOperation;

/*--------------------------------
** Enum: MODECONTROL
** @brief Mode (Normal / Gradation) Control
** ------------------------------*/
typedef enum MODECONTROL
{
	normalMode = 0x00,                     /* Normal Mode */
    gradMode = 0x01,                       /* Gradation mode Enable*/
}ModeControl;

/*--------------------------------
** Enum: HOLDTIMESELECT
** @brief Hold Time Select
** ------------------------------*/
typedef enum HOLDTIMESELECT
{
	zeroSecond = 0x00,                /* 0 Second */
	quarterSecond = 0x01,             /* 0.25 Second */
	halfSecond = 0x02,                /* 0.5 Second */
	triquarterSecond = 0x03,          /* 0.75 Second */
	oneSecond = 0x04,                 /* 1 Second */
	twoSecond = 0x05,                 /* 2 Second */
	fourSecond = 0x06,                /* 4 Second */
	SixSecond = 0x07,                 /* 6 Second */
}HoldTimeSelect;

/*--------------------------------
** Enum: HOLDCONTROL
** @brief Hold Enable/Disable
** ------------------------------*/
typedef enum HOLDCONTROL
{
	holdDisable = 0x00,                      /* Hold Disable */
    holdEnable = 0x01,                       /* Hold Enable*/
}HoldControl;

/*--------------------------------
** Enum: CYCLETIME
** @brief Cycle time
** ------------------------------*/
typedef enum CYCLETIME
{
	halfms = 0x00,                      /* Cycle Time 0.5 ms */
    eightms = 0x01,                     /* Cycle Time 8 ms*/
}CycleTime;

/*--------------------------------
** Enum: RAMPRATE
** @brief Ramp Enable/Disable
** ------------------------------*/
typedef enum RAMPRATE
{
	rampDisable = 0x00,                    /* Ramp Disable */
    rampEnable = 0x01,                     /* Ramp Enable */
}RampRate;

/*--------------------------------
** Enum: LedState
** @brief: Led channel output State
** ------------------------------*/
typedef enum LEDSTATE
{
	sledOff = 0x00,                        /* LED driver Off */
	sledOn = 0x01,                         /* LED driver On */
	sLedBrightness = 0x02,                 /* LED brightness */
	sLedBrightnessgDimmBlink  = 0x03,      /* LED brightness and group dimming/blinking */
}LedState;

/*--------------------------------
** Enum: GRADATIONGRP
** @brief: Gradation group ( 0 to 5)
** ------------------------------*/
typedef enum GRADATIONGRP
{
	GradationGroup0 = 0x00,                /* Gradation Group 0 */
	GradationGroup1 = 0x01,                /* Gradation Group 1 */
	GradationGroup2 = 0x02,                /* Gradation Group 2 */
	GradationGroup3 = 0x03,                /* Gradation Group 3 */
	GradationGroup4 = 0x04,                /* Gradation Group 4 */
	GradationGroup5 = 0x05,                /* Gradation Group 5 */
}GradationGroup;

/*--------------------------------
** Enum: CLOCKDELAY
** @brief: Delay clock cycle  ( 0 to 5)
** ------------------------------*/
typedef enum CLOCKDELAY
{
	 noDelay = 0x00,                  /* No Delay */
	 oneClock = 0x01,                 /* 1 clock cycle */
	 twoClock = 0x02,                 /* 2 clock cycle */
	 threeClock = 0x03,               /* 3 clock cycle */
	 fourClock = 0x04,                /* 4 clock cycle */
	 fiveClock = 0x05,                /* 5 clock cycle */
	 sixClock = 0x06,                 /* 6 clock cycle */
	 sevenClock = 0x07,               /* 7 clock cycle */
	 eightClock = 0x08,               /* 8 clock cycle */
	 nineClock = 0x09,                /* 9 clock cycle */
	 tenClock = 0x10,                 /* 10 clock cycle */
	 elevenClock = 0x11,              /* 11 clock cycle */
}ClockDelay;

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*!
 * @brief This defines the sensor specific information for SPI.
 */
typedef struct
{
    registerDeviceInfo_t deviceInfo;      /*!< SPI device context. */
    ARM_DRIVER_SPI *pCommDrv;             /*!< Pointer to the spi driver. */
    bool isInitialized;                   /*!< Whether sensor is intialized or not.*/
    spiSlaveSpecificParams_t slaveParams; /*!< Slave Specific Params.*/
} pca9957_spi_sensorhandle_t;


/*! @def    PCA9957_SPI_MAX_MSG_SIZE
 *  @brief  The MAX size of SPI message. */
#define PCA9957_SPI_MAX_MSG_SIZE               (64)

/*! @def    PCA9957_SPI_CMD_LEN
 *  @brief  The size of the Sensor specific SPI Header. */
#define PCA9957_SPI_CMD_LEN                    (1)

/*! @def    PCA9957_SS_ACTIVE_VALUE
 *  @brief  Is the Slave Select Pin Active Low or High. */
#define PCA9957_SS_ACTIVE_VALUE                SPI_SS_ACTIVE_LOW

/*! @def    PCA9957_SPI_WR_CMD
 *  @brief  write command of PCA9957 LED Driver */
#define PCA9957_SPI_WR_CMD                     (0xFE)

/*! @def    PCA9957_SPI_RD_CMD
 *  @brief  Read command of PCA9957 LED Driver */
#define PCA9957_SPI_RD_CMD                     (0x01)

/*! @def    PCA9957_MAX_LED_PORT
 *  @brief  Maximum LED Driver port */
#define PCA9957_MAX_LED_PORT                   (23)

/*! @def    PCA9957_MAX_LED_BRIGHTNESS
 *  @brief  Maximum LED Brightness */
#define PCA9957_MAX_LED_BRIGHTNESS             (0xFF)

/*! @def    PCA9957_MAX_GRP_FREQUENCY
 *  @brief  Maximum Group Frequency */
#define PCA9957_MAX_GRP_FREQUENCY             (0xFF)

/*! @def    PCA9957_DEFAULT_LED_BRIGHTNESS
 *  @brief  Default LED Brightness */
#define PCA9957_DEFAULT_LED_BRIGHTNESS         (0x80)

/*! @def    PCA9957_LED_STATE_MASK
 *  @brief  LED STATE MASK */
#define PCA9957_LED_STATE_MASK                 (0x03)

/*! @def    PCA9957_MAX_GROUP_RAMP
 *  @brief  Maximum Group Ramp value */
#define PCA9957_MAX_GROUP_RAMP                 (0x3F)

/*! @def    PCA9957_MAX_GROUP_MF
 *  @brief  Maximum Group Multiple Factor */
#define PCA9957_MAX_GROUP_MF                   (0x3F)

/*! @def    PCA9957_MAX_CURRENT_GAIN
 *  @brief  Maximum Current Gain  */
#define PCA9957_MAX_CURRENT_GAIN               (0xFF)

/*! @def    PCA9957_ONE_BIT_MASK
 *  @brief  One Bit Mask  */
#define PCA9957_ONE_BIT_MASK                   (0x01)

/*! @def    PCA9957_MAX_CLOCK_DELAY
 *  @brief  Maximum Clock Delay */
#define PCA9957_MAX_CLOCK_DELAY                (0x0B)

/*! @def    PCA9957_ALL_LED_MASK
 *  @brief  All LED (8 Bit) Mask */
#define PCA9957_ALL_LED_MASK                   (0xFF)

/*! @def    PCA9957_ALL_LED_OFF
 *  @brief  LED off (8 Bit) State */
#define PCA9957_ALL_LED_OFF                    (0x00)

/*! @def    PCA9957_ALL_LED_ON
 *  @brief  LED on (8 Bit) State */
#define PCA9957_ALL_LED_ON                     (0x55)

/*! @def    PCA9957_ALL_LED_BRIGHTNESS
 *  @brief  PCA9957 all LED Brightness */
#define PCA9957_ALL_LED_BRIGHTNESS             (0xAA)

/*! @def    PCA9957_ALL_LED_BRIGHTNESS_DIMMBLINK
 *  @brief   PCA9957 all LED Dimmimg Blinking */
#define PCA9957_ALL_LED_BRIGHTNESS_DIMMBLINK   (0xFF)

/*! @def    PCA9957_DEFAULT_RAMP_VALUE
 *  @brief  Default Ramp value */
#define PCA9957_DEFAULT_RAMP_VALUE             (0x01)

/*! @def    PCA9957_DEFAULT_STEP_TIME
 *  @brief  Default step time value */
#define PCA9957_DEFAULT_STEP_TIME              (0x3F)

/*! @def    PCA9957_DEFAULT_HOLD_ON_TIME
 *  @brief  Default hold on time value*/
#define PCA9957_DEFAULT_HOLD_ON_TIME           (0x02)

/*! @def    PCA9957_DEFAULT_HOLD_OFF_TIME
 *  @brief  Default hold off time value */
#define PCA9957_DEFAULT_HOLD_OFF_TIME          (0x02)

/*! @def    PCA9957_RESET_DELAY_MS
 *  @brief  Reset Delay MS */
#define PCA9957_RESET_DELAY_MS                 (0x0A)

/*! @def    PCA9957_ALL_LED_GRAD_EN
 *  @brief  All LED gradation Enable */
#define PCA9957_ALL_LED_GRAD_EN                (0xFF)

/*! @def    PCA9957_ALL_LED_GRAD_DIS
 *  @brief  All LED gradation Disable */
#define PCA9957_ALL_LED_GRAD_DIS               (0x00)

/*! @def    PCA9957_CLRERR
 *  @brief  Clear Error */
#define PCA9957_CLRERR                         (0x01)

/*! @def    PCA9957_REG_SIZE_BYTE
 *  @brief  Register Size  */
#define PCA9957_REG_SIZE_BYTE                  (0x01)

/*! @def    PCA9957_LED_ERROR_TYPE_MASK
 *  @brief  LED error clear Mask */
#define PCA9957_LED_ERROR_TYPE_MASK             (0x03)

/*! @def    PCA9957_GROUP0_CONT_START
 *  @brief  Group0 continuous gradation and start */
#define PCA9957_GROUP0_CONT_START              (0x03)

/*! @def    PCA9957_GROUP0_CONT_START
 *  @brief  Group0 Single shot gradation and start */
#define PCA9957_GROUP0_SINGLE_START            (0x02)

/*! @def    PCA9957_GROUP1_CONT_START
 *  @brief  Group1 continuous gradation and start */
#define PCA9957_GROUP1_CONT_START              (0x0C)

/*! @def    PCA9957_GROUP1_CONT_START
 *  @brief  Group1 Single shot gradation and start */
#define PCA9957_GROUP1_SINGLE_START            (0x08)

/*! @def    PCA9957_GROUP2_CONT_START
 *  @brief  Group2 continuous gradation and start */
#define PCA9957_GROUP2_CONT_START              (0x30)

/*! @def    PCA9957_GROUP2_CONT_START
 *  @brief  Group2 Single shot gradation and start */
#define PCA9957_GROUP2_SINGLE_START            (0x20)

/*! @def    PCA9957_GROUP3_CONT_START
 *  @brief  Group3 continuous gradation and start */
#define PCA9957_GROUP3_CONT_START              (0xC0)

/*! @def    PCA9957_GROUP3_CONT_START
 *  @brief  Group3 Single shot gradation and start */
#define PCA9957_GROUP3_SINGLE_START            (0x80)

/*! @def    PCA9957_GROUP4_CONT_START
 *  @brief  Group4 continuous gradation and start */
#define PCA9957_GROUP4_CONT_START              (0x30)

/*! @def    PCA9957_GROUP4_CONT_START
 *  @brief  Group4 Single shot gradation and start */
#define PCA9957_GROUP4_SINGLE_START            (0x20)

/*! @def    PCA9957_GROUP5_CONT_START
 *  @brief  Group5 continuous gradation and start */
#define PCA9957_GROUP5_CONT_START              (0xC0)

/*! @def    PCA9957_GROUP5_CONT_START
 *  @brief  Group5 Single shot gradation and start */
#define PCA9957_GROUP5_SINGLE_START            (0x80)

/*! @def    PCA9957_TWO_BIT_MASK
 *  @brief  TWO Bit Mask  */
#define PCA9957_TWO_BIT_MASK                   (0x03)

/*! @def    PCA9957_THREE_BIT_MASK
*  @brief  TWO Bit Mask  */
#define PCA9957_THREE_BIT_MASK                   (0x07)

/*! @def    PCA9957_ALL_BIT_ONE
 *  @brief  All Bit One  */
#define PCA9957_ALL_BIT_ONE                    (0xFF)

/* Led NUmber */
typedef uint8_t LedNum;

/* Led Brightness */
typedef uint8_t LedBrightness;

/* Group Ramp Rate */
typedef uint8_t GroupRampRate;

/* Group Multiple Factor */
typedef uint8_t GroupMultipleFactor;

/* Current Gain */
typedef uint8_t CurrentGain;

/* Group Frequency */
typedef uint8_t GroupFrequency;

/*******************************************************************************
 * APIs
 ******************************************************************************/

/*! @brief       Preprocesses a read command for the PCA9957 LED Driver.
 *  @details     Prepares a read command to be sent to the sensor.
 *  @param[in]   pCmdOut  		Pointer to the command output buffer.
 *  @param[in]   offset   		Offset for the read command.
 *  @param[in]   size     		Size of the read command.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant  No
 */
void PCA9957_SPI_ReadPreprocess(void *pCmdOut, uint32_t offset, uint32_t size);

/*! @brief       Preprocesses a write command for the PCA9957 LED Driver.
 *  @details     Prepares a write command to be sent to the sensor.
 *  @param[in]   pCmdOut  		Pointer to the command output buffer.
 *  @param[in]   offset  		Offset for the write command.
 *  @param[in]   size     		Size of the write command.
 *  @param[in] 	 pWritebuffer 	Pointer to the buffer containing data to be written.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant  No
 */
void PCA9957_SPI_WritePreprocess(void *pCmdOut, uint32_t offset, uint32_t size, void *pWritebuffer);

/*! @brief       Initializes the PCA9957 LED Driver.
 *  @details     Initializes the PCA9957 LED Driver and its handle.
 *  @param[in]   pSensorHandle  Pointer to sensor handle structure.
 *  @param[in]   pBus  			Pointer to CMSIS API compatible SPI bus object.
 *  @param[in]   index     		Index of the sensor.
 *  @param[in] 	 pSlaveSelect 	Pointer to the slave select pin.
 *  @constraints This should be the first API to be called.
 *				 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::PCA9957_SPI_Initialize() returns the status
 */
int32_t PCA9957_SPI_Initialize(pca9957_spi_sensorhandle_t *pSensorHandle, ARM_DRIVER_SPI *pBus, uint8_t index, void *pSlaveSelect);

/*! @brief       Sets an idle task for the PCA9957 LED Driver.
 *  @details     Sets a function to be called when the sensor is in idle state.
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @param[in]   idleTask  			Function Pointer to the idle task.
 *  @param[in]   userParam     		Pointer to user defined parameter for the idle task.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 */
void PCA9957_SPI_SetIdleTask(pca9957_spi_sensorhandle_t *pSensorHandle, registeridlefunction_t idleTask, void *userParam);

/*! @brief       Configures the PCA9957 LED Driver.
 *  @details     Initializes the PCA9957 LED Driver and its handle.
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @param[in]   pRegWriteList      Pointer to the list of register write operations.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return      ::PCF2131_SPI_Configue() returns the status.
 */
int32_t PCA9957_SPI_Configure(pca9957_spi_sensorhandle_t *pSensorHandle, const registerwritelist_t *pRegWriteList);

/*! @brief       De-initializes the PCA9957 LED Driver.
 *  @details     De-initializes the PCA9957 LED Driver and its handle.
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_Deinit() returns the status.
 */
int32_t PCA9957_SPI_Deinit(pca9957_spi_sensorhandle_t *pSensorHandle, void *pResetPin);

/*! @brief       Turn all LED OFF.
 *  @details     Disable all LED Channel Output.
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_All_LED_Off() returns the status.
 */
int32_t PCA9957_SPI_All_LED_Off(pca9957_spi_sensorhandle_t *pSensorHandle);

/*! @brief       Turn all LED On.
 *  @details     Enable all LED Channel Output.
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_All_LED_On() returns the status.
 */
int32_t PCA9957_SPI_All_LED_On(pca9957_spi_sensorhandle_t *pSensorHandle);

/*! @brief       Set All LED Brightness.
 *  @details     Set Brightness for all LED channel output.
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @param[in]   brightness         Brightness for LED channel.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_Set_All_LED_Brightness() returns the status.
 */
int32_t PCA9957_SPI_Set_All_LED_Brightness(pca9957_spi_sensorhandle_t *pSensorHandle, LedBrightness brightness);

/*! @brief       Turn Individual LED OFF.
 *  @details     Disable Individual LED Channel Output.
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @param[in]   lednum             LED channel.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_All_LED_Off() returns the status.
 */
int32_t PCA9957_SPI_LED_Off(pca9957_spi_sensorhandle_t *pSensorHandle, LedNum lednum);

/*! @brief       Turn Individual LED On.
 *  @details     Enable Individual LED Channel Output.
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @param[in]   lednum             LED channel.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_All_LED_On() returns the status.
 */
int32_t PCA9957_SPI_LED_On(pca9957_spi_sensorhandle_t *pSensorHandle, LedNum lednum);

/*! @brief       Set Individual LED Brightness.
 *  @details     Set Brightness for Individual LED channel.
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @param[in]   brightness         Brightness for LED channel.
 *  @param[in]   lednum             LED channel.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_Set_LED_Brightness() returns the status.
 */
int32_t PCA9957_SPI_Set_LED_Brightness(pca9957_spi_sensorhandle_t *pSensorHandle, LedNum lednum, LedBrightness brightness);

/*! @brief       Set Individual LED State.
 *  @details     Set State for Individual LED channel.
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @param[in]   lednum             LED channel.
 *  @param[in]   ledstate           LED State.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_Set_LED_State() returns the status.
 */
int32_t PCA9957_SPI_Set_LED_State(pca9957_spi_sensorhandle_t *pSensorHandle, LedNum lednum, LedState ledstate);

/*! @brief       Set all LED State.
 *  @details     Set State for all LED channel.
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @param[in]   ledstate           State for LED channel.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_Set_All_LED_State() returns the status.
 */
int32_t PCA9957_SPI_Set_All_LED_State(pca9957_spi_sensorhandle_t *pSensorHandle, LedState ledstate);

/*! @brief       Set Individual LED output Gain.
 *  @details     Set current gain for Individual LED output channel.
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @param[in]   CurrentGain        Current Gain.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_Set_LED_OP_Current_Gain() returns the status.
 */
int32_t PCA9957_SPI_Set_LED_OP_Current_Gain(pca9957_spi_sensorhandle_t *pSensorHandle, LedNum lednum, CurrentGain currentgain);

/*! @brief       Set all LED output Gain.
 *  @details     Set current gain for all LED output channel.
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @param[in]   CurrentGain        Current Gain.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_Set_All_LED_OP_Current_Gain() returns the status.
 */
int32_t PCA9957_SPI_Set_All_LED_OP_Current_Gain(pca9957_spi_sensorhandle_t *pSensorHandle, CurrentGain currentgain);

/*! @brief       Start Gradation.
 *  @details     Start Gradation for group (0 to 5).
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @param[in]   gradationgroup     Gradation group.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_Gradation_Group_Start() returns the status.
 */
int32_t PCA9957_SPI_Gradation_Group_Start(pca9957_spi_sensorhandle_t *pSensorHandle,  GradationGroup gradationgroup, GradOperation grandoperation);

/*! @brief       Stop Gradation.
 *  @details     Stop Gradation for group (0 to 5).
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @param[in]   gradationgroup     Gradation group.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_Gradation_Group_Stop() returns the status.
 */
int32_t PCA9957_SPI_Gradation_Group_Stop(pca9957_spi_sensorhandle_t *pSensorHandle, GradationGroup gradationgroup);

/*! @brief       Enable Gradation.
 *  @details     Enable Gradation for LED channel 9 0to 24).
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @param[in]   lednum             LED channel.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_Enable_Gradation_Mode() returns the status.
 */
int32_t PCA9957_SPI_Enable_Gradation_Mode(pca9957_spi_sensorhandle_t *pSensorHandle, LedNum lednum);

/*! @brief       Disable Gradation.
 *  @details     Disable Gradation for LED channel (0 to 23).
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @param[in]   lednum             LED channel.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_Disable_Gradation_Mode() returns the status.
 */
int32_t PCA9957_SPI_Disable_Gradation_Mode(pca9957_spi_sensorhandle_t *pSensorHandle, LedNum lednum);

/*! @brief       Set Gradation group (0 to 5) ramp value.
 *  @details     Set ramp value for Gradation group (0 to 5).
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @param[in]   gradationgroup     Gradation group.
 *  @param[in]   groupramprate      Gradation group ramp value.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_Set_Group_Ramp_Value() returns the status.
 */
int32_t PCA9957_SPI_Set_Group_Ramp_Value(pca9957_spi_sensorhandle_t *pSensorHandle, GradationGroup gradationgroup, GroupRampRate groupramprate);

/*! @brief       Enable Gradation group (0 to 5) Ramp UP.
 *  @details     Enable Gradation group (0 to 5) Ramp UP.
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @param[in]   gradationgroup     Gradation group.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_Enable_Group_RampUp() returns the status.
 */
int32_t PCA9957_SPI_Enable_Group_RampUp(pca9957_spi_sensorhandle_t *pSensorHandle, GradationGroup gradationgroup);

/*! @brief       Disable Gradation group (0 to 5) Ramp UP.
 *  @details     Disable Gradation group (0 to 5) Ramp UP.
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @param[in]   gradationgroup     Gradation group.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_Disable_Group_RampUp() returns the status.
 */
int32_t PCA9957_SPI_Disable_Group_RampUp(pca9957_spi_sensorhandle_t *pSensorHandle, GradationGroup gradationgroup);

/*! @brief       Enable Gradation group (0 to 5) Ramp Down.
 *  @details     Enable Gradation group (0 to 5) Ramp Down.
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @param[in]   gradationgroup     Gradation group.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_Enable_Group_RampDown() returns the status.
 */
int32_t PCA9957_SPI_Enable_Group_RampDown(pca9957_spi_sensorhandle_t *pSensorHandle, GradationGroup gradationgroup);

/*! @brief       Disable Gradation group Ramp Down.
 *  @details     Disable Gradation group (0 to 5) Ramp Down.
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @param[in]   gradationgroup     Gradation group.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_Disable_Group_RampDown() returns the status.
 */
int32_t PCA9957_SPI_Disable_Group_RampDown(pca9957_spi_sensorhandle_t *pSensorHandle, GradationGroup gradationgroup);

/*! @brief       Set Gradation group cycle time.
 *  @details     Set Gradation group cycle time.
 *  @param[in]   pSensorHandle  	Pointer to sensor handle structure.
 *  @param[in]   gradationgroup     Gradation group.
 *  @param[in]   cycletime          Cycle time.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_Set_Cycle_Time() returns the status.
 */
int32_t PCA9957_SPI_Set_Cycle_Time(pca9957_spi_sensorhandle_t *pSensorHandle, GradationGroup gradationgroup, CycleTime cycletime);

/*! @brief       Set Multiplication Factor.
 *  @details     Set Multiplication Factor Gradation group.
 *  @param[in]   pSensorHandle  	   Pointer to sensor handle structure.
 *  @param[in]   gradationgroup        Gradation group.
 *  @param[in]   groupmultiplefactor   Multiplication Factor.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_Set_Group_MF() returns the status.
 */
int32_t PCA9957_SPI_Set_Group_MF(pca9957_spi_sensorhandle_t *pSensorHandle, GradationGroup gradationgroup, GroupMultipleFactor groupmultiplefactor);

/*! @brief       Enable Hold on for Gradation group.
 *  @details     Enable Hold on for Gradation group.
 *  @param[in]   pSensorHandle  	   Pointer to sensor handle structure.
 *  @param[in]   gradationgroup        Gradation group.
 *  @param[in]   holdtimeselect        Hold time.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_Enable_HoldOn() returns the status.
 */
int32_t PCA9957_SPI_Enable_HoldOn(pca9957_spi_sensorhandle_t *pSensorHandle, GradationGroup gradationgroup, HoldTimeSelect holdtimeselect);

/*! @brief       Disable Hold off for Gradation group.
 *  @details     Disable Hold off for Gradation group.
 *  @param[in]   pSensorHandle  	   Pointer to sensor handle structure.
 *  @param[in]   gradationgroup        Gradation group.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_Disable_HoldOn() returns the status.
 */
int32_t PCA9957_SPI_Disable_HoldOn(pca9957_spi_sensorhandle_t *pSensorHandle, GradationGroup gradationgroup);

/*! @brief       Enable Hold off for Gradation group.
 *  @details     Enable Hold off for Gradation group.
 *  @param[in]   pSensorHandle  	   Pointer to sensor handle structure.
 *  @param[in]   gradationgroup        Gradation group.
 *  @param[in]   holdtimeselect        Hold time.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_Enable_HoldOn() returns the status.
 */
int32_t PCA9957_SPI_Enable_HoldOff(pca9957_spi_sensorhandle_t *pSensorHandle, GradationGroup gradationgroup, HoldTimeSelect holdtimeselect);

/*! @brief       Disable Hold off for Gradation group.
 *  @details     Disable Hold off for Gradation group.
 *  @param[in]   pSensorHandle  	   Pointer to sensor handle structure.
 *  @param[in]   gradationgroup        Gradation group.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_Disable_HoldOff() returns the status.
 */
int32_t PCA9957_SPI_Disable_HoldOff(pca9957_spi_sensorhandle_t *pSensorHandle, GradationGroup gradationgroup);

/*! @brief       Set Gradation group(0 to 5) current gain.
 *  @details     Set Gradation group(0 to 5) current gain.
 *  @param[in]   pSensorHandle  	   Pointer to sensor handle structure.
 *  @param[in]   gradationgroup        Gradation group.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_Set_Gradation_Group_Current_Gain() returns the status.
 */
int32_t PCA9957_SPI_Set_Gradation_Group_Current_Gain(pca9957_spi_sensorhandle_t *pSensorHandle, GradationGroup gradationgroup, CurrentGain currentgain);

/*! @brief       Assign LED to  Gradation group(0 to 5).
 *  @details     Assign LED to  Gradation group(0 to 5).
 *  @param[in]   pSensorHandle  	   Pointer to sensor handle structure.
 *  @param[in]   gradationgroup        Gradation group.
 *  @param[in]   lednum                LED channel.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_Assign_LED_to_Gradation_Group() returns the status.
 */
int32_t PCA9957_SPI_Assign_LED_to_Gradation_Group(pca9957_spi_sensorhandle_t *pSensorHandle, LedNum lednum, GradationGroup gradationgroup);

/*! @brief       Set all LED to Dimming or Blinking.
 *  @details     Set all LED to Dimming or Blinking.
 *  @param[in]   pSensorHandle  	   Pointer to sensor handle structure.
 *  @param[in]   GradDimmBlink        Dimming/Blinking.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_Set_All_LED_Dimm_Blink() returns the status.
 */
int32_t PCA9957_SPI_Set_All_LED_Dimm_Blink(pca9957_spi_sensorhandle_t *pSensorHandle, GradDimmBlink graddimmblink);

/*! @brief       Set all LED Brightness to Dimming or Blinking.
 *  @details     Set all LED Brightness to Dimming or Blinking.
 *  @param[in]   pSensorHandle  	   Pointer to sensor handle structure.
 *  @param[in]   brightness        Brightness.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_Set_All_LED_Dimm_Blink_Brightness() returns the status.
 */
int32_t PCA9957_SPI_Set_All_LED_Dimm_Blink_Brightness(pca9957_spi_sensorhandle_t *pSensorHandle, LedBrightness brightness);

/*! @brief       Set all LED frequency for Dimming or Blinking.
 *  @details     Set all LED frequency for Dimming or Blinking.
 *  @param[in]   pSensorHandle  	   Pointer to sensor handle structure.
 *  @param[in]   GroupFrequency        group frequency.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_Set_All_LED_Dimm_Blink_Frequency() returns the status.
 */
int32_t PCA9957_SPI_Set_All_LED_Dimm_Blink_Frequency(pca9957_spi_sensorhandle_t *pSensorHandle, GroupFrequency groupfrequency);


/*! @brief       Set output delay cycle for LED output channel.
 *  @details     Set all LED frequency for Dimming or Blinking.
 *  @param[in]   pSensorHandle  	   Pointer to sensor handle structure.
 *  @param[in]   clockdelay            Clock Delay.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return     ::PCA9957_SPI_Set_LED_OP_Delay() returns the status.
 */
int32_t PCA9957_SPI_Set_LED_OP_Delay(pca9957_spi_sensorhandle_t *pSensorHandle, ClockDelay clockdelay);


/*! @brief       Set gradation adjustment.
 *  @details     Set adjustment(linear or exponential) for gradation group..
 *  @param[in]   pSensorHandle  	   Pointer to sensor handle structure.
 *  @param[in]   grandadjustment       adjustment(linear or exponential).
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return      ::PCA9957_SPI_Set_Gradation_Adjustment() returns the status.
 */
int32_t PCA9957_SPI_Set_Gradation_Adjustment(pca9957_spi_sensorhandle_t *pSensorHandle, GrandAdjustment grandadjustment);

/*! @brief       Enable all LED gradation mode.
 *  @details     Enable gradation mode for all LED.
 *  @param[in]   pSensorHandle  	   Pointer to sensor handle structure.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return      ::PCA9957_SPI_Set_Gradation_Adjustment() returns the status.
 */
int32_t PCA9957_SPI_Enable_All_LED_Gradation_Mode(pca9957_spi_sensorhandle_t *pSensorHandle);

/*! @brief       Disable all LED gradation mode.
 *  @details     Disable gradation mode for all LED.
 *  @param[in]   pSensorHandle  	   Pointer to sensor handle structure.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return      ::PCA9957_SPI_Set_Gradation_Adjustment() returns the status.
 */
int32_t PCA9957_SPI_Disable_All_LED_Gradation_Mode(pca9957_spi_sensorhandle_t *pSensorHandle);

/*! @brief       Disable Sleep.
 *  @details     Disable Sleep.
 *  @param[in]   pSensorHandle  	   Pointer to sensor handle structure.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return      ::PCA9957_SPI_Disable_Sleep() returns the status.
 */
int32_t PCA9957_SPI_Disable_Sleep(pca9957_spi_sensorhandle_t *pSensorHandle);

/*! @brief       Enable Sleep.
 *  @details     Enable Sleep.
 *  @param[in]   pSensorHandle  	   Pointer to sensor handle structure.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return      ::PCA9957_SPI_Enable_Sleep() returns the status.
 */
int32_t PCA9957_SPI_Enable_Sleep(pca9957_spi_sensorhandle_t *pSensorHandle);

/*! @brief       Enable Auto switch off.
 *  @details     Enable Auto switch off.
 *  @param[in]   pSensorHandle  	   Pointer to sensor handle structure.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return      ::PCA9957_SPI_Enable_Auto_SwitchOff() returns the status.
 */
int32_t PCA9957_SPI_Enable_Auto_SwitchOff(pca9957_spi_sensorhandle_t *pSensorHandle);

/*! @brief       Disable Auto switch off.
 *  @details     Disable Auto switch off.
 *  @param[in]   pSensorHandle  	   Pointer to sensor handle structure.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return      ::PCA9957_SPI_Disable_Auto_SwitchOff() returns the status.
 */
int32_t PCA9957_SPI_Disable_Auto_SwitchOff(pca9957_spi_sensorhandle_t *pSensorHandle);

/*! @brief       Clear Error.
 *  @details     clear error for LED channel.
 *  @param[in]   pSensorHandle  	   Pointer to sensor handle structure.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return      ::PCA9957_SPI_Clear_Error() returns the status.
 */
int32_t PCA9957_SPI_Clear_Error(pca9957_spi_sensorhandle_t *pSensorHandle);

/*! @brief       check Error.
 *  @details     clear error (open or short) for any LED channel.
 *  @param[in]   pSensorHandle  	   Pointer to sensor handle structure.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return      ::PCA9957_SPI_Clear_Error() returns the status.
 */
int32_t PCA9957_SPI_Check_Error(pca9957_spi_sensorhandle_t *pSensorHandle, ErrorState *errorstate);

/*! @brief       check Over Temperature.
 *  @details     clear Over Temperature for PCA9957.
 *  @param[in]   pSensorHandle  	   Pointer to sensor handle structure.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return      ::PCA9957_SPI_Check_OvTemp() returns the status.
 */
int32_t PCA9957_SPI_Check_OvTemp(pca9957_spi_sensorhandle_t *pSensorHandle, TempCondition *tempcondition);

/*! @brief       Get error type.
 *  @details      Get error type for LED channel.
 *  @param[in]   pSensorHandle  	   Pointer to sensor handle structure.
 *  @param[in]   lednum                LED channel.
 *  @param[out]  lederrortype          LED error type.
 *  @constraints This can be called any number of times only after PCA9957_SPI_Initialize().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation
 *  @reentrant   No
 *  @return      ::PCA9957_SPI_LED_Get_Error_Type() returns the status.
 */
int32_t PCA9957_SPI_LED_Get_Error_Type(pca9957_spi_sensorhandle_t *pSensorHandle, LedNum lednum, LedErrorType *lederrortype);

/*! @brief       Initialize PCA9957 OE pin.
 *  @details     make OE pin as output and set its default state to high .
 *  @param[in]   pOePin  	   Pointer to gpioHandleKSDK_t.
 *  @constraints No
 *  @reentrant   No
 *  @return      ::PCA9957_OE_Pin_Init() returns the status.
 */
int32_t PCA9957_OE_Pin_Init(void *pOePin);

/*! @brief       Initialize PCA9957 Reset pin.
 *  @details     make Reset pin as output and set its default state to high .
 *  @param[in]   pOePin  	   Pointer to gpioHandleKSDK_t.
 *  @constraints No
 *  @reentrant   No
 *  @return      ::PCA9957_Reset_Pin_Init() returns the status.
 */
int32_t PCA9957_Reset_Pin_Init(void *pResetPin);

/*! @brief       Initialize PCA9957 SW EN pin.
 *  @details     make SW EN pin as output and set its default state to clear .
 *  @param[in]   pOePin  	   Pointer to gpioHandleKSDK_t.
 *  @constraints No
 *  @reentrant   No
 *  @return      ::PCA9957_SW_EN_Pin_Init() returns the status.
 */
int32_t PCA9957_SW_EN_Pin_Init(void *pSwEnPin);

/*! @brief       Reset PCA9957.
 *  @details     Set Reset pin as low for PCA9957_RESET_DELAY_MS.
 *  @param[in]   pOePin  	   Pointer to gpioHandleKSDK_t.
 *  @constraints This can be called any number of times only after PCA9957_Reset_Pin_Init().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::PCA9957_Reset() returns the status.
 */
int32_t PCA9957_Reset(void *pResetPin);

/*! @brief       Enable OE pin.
 *  @details     Set OE pin as low.
 *  @param[in]   pOePin  	   Pointer to gpioHandleKSDK_t.
 *  @constraints This can be called any number of times only after PCA9957_OE_Pin_Init().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::PCA9957_Output_Enable() returns the status.
 */
int32_t PCA9957_Output_Enable(void *pOePin);

/*! @brief       Disable OE pin.
 *  @details     Set OE pin as High.
 *  @param[in]   pOePin  	   Pointer to gpioHandleKSDK_t.
 *  @constraints This can be called any number of times only after PCA9957_OE_Pin_Init().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::PCA9957_Output_Disable() returns the status.
 */
int32_t PCA9957_Output_Disable(void *pOePin);

/*! @brief       Set maximum current to 30 MA.
 *  @details     Set SwEn pin as High.
 *  @param[in]   pOePin  	   Pointer to gpioHandleKSDK_t.
 *  @constraints This can be called any number of times only after PCA9957_SW_EN_Pin_Init().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::PCA9957_Set_Imax_30MA() returns the status.
 */
int32_t PCA9957_Set_Imax_30MA(void *pImaxPin);

/*! @brief       Set maximum current to 20 MA.
 *  @details     Set SwEn pin as Low.
 *  @param[in]   pOePin  	   Pointer to gpioHandleKSDK_t.
 *  @constraints This can be called any number of times only after PCA9957_SW_EN_Pin_Init().
 *				 Application has to ensure that previous instances of these APIs have exited before invocation.
 *  @reentrant   No
 *  @return      ::PCA9957_Set_Imax_20MA() returns the status.
 */
int32_t PCA9957_Set_Imax_20MA(void *pImaxPin);
#endif /* PCA9957_DRV_H_ */
