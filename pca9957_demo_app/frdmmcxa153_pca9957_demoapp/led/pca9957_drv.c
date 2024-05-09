/*
 * Copyright 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file pca9957_drv.c
 * @brief The pca9957_drv.c file implements the PCA9957 LED driver interfaces.
 */

//-----------------------------------------------------------------------
// ISSDK Includes
//-----------------------------------------------------------------------
#include "pca9957_drv.h"
#include "gpio_driver.h"
#include "systick_utils.h"

//-----------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------

uint8_t pca9957_spiRead_CmdBuffer[PCA9957_SPI_MAX_MSG_SIZE] = {0};
uint8_t pca9957_spiRead_DataBuffer[PCA9957_SPI_MAX_MSG_SIZE] = {0};
uint8_t pca9957_spiWrite_CmdDataBuffer[PCA9957_SPI_MAX_MSG_SIZE] = {0};

//-----------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------

void PCA9957_SPI_ReadCmdPreprocess(void *pCmdOut, uint32_t offset, uint32_t size)
{
	spiCmdParams_t *pSlaveCmd = pCmdOut;

	uint8_t *pWBuff = pca9957_spiRead_CmdBuffer;
	uint8_t *pRBuff = pca9957_spiRead_DataBuffer;

	/* Formatting for Read command of PCA9957 LED driver. */
	*(pWBuff) = ( offset << 1) | PCA9957_SPI_RD_CMD;  /* offset is the internal register address of the sensor at which Read is performed. */
	*(pWBuff + 1) = PCA9957_ALL_BIT_ONE;

	/* Create the slave read command. */
	pSlaveCmd->size = size + PCA9957_SPI_CMD_LEN;
	pSlaveCmd->pWriteBuffer = pWBuff;
	pSlaveCmd->pReadBuffer = NULL;
}

void PCA9957_SPI_ReadPreprocess(void *pCmdOut, uint32_t offset, uint32_t size)
{
	spiCmdParams_t *pSlaveCmd = pCmdOut;

	uint8_t *pWBuff = pca9957_spiRead_CmdBuffer;
	uint8_t *pRBuff = pca9957_spiRead_DataBuffer;

	/* Formatting for Read command of PCA9957 LED driver. */
	*(pWBuff) =  PCA9957_ALL_BIT_ONE;

	/* Create the slave read command. */
	pSlaveCmd->size = size + PCA9957_SPI_CMD_LEN;
	pSlaveCmd->pWriteBuffer = pWBuff;
	pSlaveCmd->pReadBuffer = pRBuff;
}

void PCA9957_SPI_WritePreprocess(void *pCmdOut, uint32_t offset, uint32_t size, void *pWritebuffer)
{
    spiCmdParams_t *pSlaveCmd = pCmdOut;

    uint8_t *pWBuff = pca9957_spiWrite_CmdDataBuffer;

    /* Formatting for Write command of PCA9957 LED driver. */
    *(pWBuff) = ( offset << 1) & PCA9957_SPI_WR_CMD; /* offset is the internal register address of the sensor at which write is performed. */

    /* Copy the slave write command */
    memcpy(pWBuff + PCA9957_SPI_CMD_LEN, pWritebuffer, size);

    /* Create the slave command. */
    pSlaveCmd->size = size + PCA9957_SPI_CMD_LEN;
    pSlaveCmd->pWriteBuffer = pWBuff;
    pSlaveCmd->pReadBuffer = NULL;
}

int32_t PCA9957_SPI_Initialize(pca9957_spi_sensorhandle_t *pSensorHandle,
										ARM_DRIVER_SPI *pBus, uint8_t index,
										void *pSlaveSelect)
{
    int32_t status;
    uint8_t reg;
    GENERIC_DRIVER_GPIO *pGPIODriver = &Driver_GPIO_KSDK;

    /*! Check the input parameters. */
    if ((pSensorHandle == NULL) || (pBus == NULL) || (pSlaveSelect == NULL))
    {
        return SENSOR_ERROR_INVALID_PARAM;
    }

    /*! Initialize the sensor handle. */
    pSensorHandle->pCommDrv = pBus;
    pSensorHandle->slaveParams.pReadPreprocessFN = PCA9957_SPI_ReadPreprocess;
    pSensorHandle->slaveParams.pWritePreprocessFN = PCA9957_SPI_WritePreprocess;
    pSensorHandle->slaveParams.pReadCmdPreprocessFN = PCA9957_SPI_ReadCmdPreprocess;
    pSensorHandle->slaveParams.pTargetSlavePinID = pSlaveSelect;
    pSensorHandle->slaveParams.spiCmdLen = PCA9957_SPI_CMD_LEN;
    pSensorHandle->slaveParams.ssActiveValue = PCA9957_SS_ACTIVE_VALUE;
    pSensorHandle->deviceInfo.deviceInstance = index;
    pSensorHandle->deviceInfo.functionParam = NULL;
    pSensorHandle->deviceInfo.idleFunction = NULL;

    /* Initialize the Slave Select Pin. */
    pGPIODriver->pin_init(pSlaveSelect, GPIO_DIRECTION_OUT, NULL, NULL, NULL);
    if (pSensorHandle->slaveParams.ssActiveValue == SPI_SS_ACTIVE_LOW)
    {
        pGPIODriver->set_pin(pSlaveSelect);
    }
    else
    {
        pGPIODriver->clr_pin(pSlaveSelect);
    }

    pSensorHandle->isInitialized = true;
    return SENSOR_ERROR_NONE;
}

int32_t PCA9957_OE_Pin_Init(void *pOePin)
{
	 GENERIC_DRIVER_GPIO *pGPIODriver = &Driver_GPIO_KSDK;

	 /* Initialize OE pin and set its default state */
	 pGPIODriver->pin_init(pOePin, GPIO_DIRECTION_OUT, NULL, NULL, NULL);
	 pGPIODriver->set_pin(pOePin);

	 return SENSOR_ERROR_NONE;
}

int32_t PCA9957_Reset_Pin_Init(void *pResetPin)
{
	 GENERIC_DRIVER_GPIO *pGPIODriver = &Driver_GPIO_KSDK;

	 /* Initialize Reset pin and set its default state */
	 pGPIODriver->pin_init(pResetPin, GPIO_DIRECTION_OUT, NULL, NULL, NULL);
	 pGPIODriver->set_pin(pResetPin);

	 return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SW_EN_Pin_Init(void *pSwEnPin)
{
	 GENERIC_DRIVER_GPIO *pGPIODriver = &Driver_GPIO_KSDK;

	 /* Initialize SW Enable pin and set its default state */
	 pGPIODriver->pin_init(pSwEnPin, GPIO_DIRECTION_OUT, NULL, NULL, NULL);
	 pGPIODriver->clr_pin(pSwEnPin);

	 return SENSOR_ERROR_NONE;
}


int32_t PCA9957_Reset(void *pResetPin)
{
	 GENERIC_DRIVER_GPIO *pGPIODriver = &Driver_GPIO_KSDK;

	 /* Set Reset pin Low for PCA9957_RESET_DELAY_MS */
	 pGPIODriver->clr_pin(pResetPin);
	 BOARD_DELAY_ms(PCA9957_RESET_DELAY_MS);
	 pGPIODriver->set_pin(pResetPin);

	 return SENSOR_ERROR_NONE;
}

int32_t PCA9957_Output_Enable(void *pOePin)
{
	 GENERIC_DRIVER_GPIO *pGPIODriver = &Driver_GPIO_KSDK;

	 /* Set OE pin Low */
	 pGPIODriver->clr_pin(pOePin);

	 return SENSOR_ERROR_NONE;
}

int32_t PCA9957_Output_Disable(void *pOePin)
{
	 GENERIC_DRIVER_GPIO *pGPIODriver = &Driver_GPIO_KSDK;

	 /* Set OE pin High */
	 pGPIODriver->set_pin(pOePin);

	 return SENSOR_ERROR_NONE;
}

int32_t PCA9957_Set_Imax_30MA(void *pImaxPin)
{
	 GENERIC_DRIVER_GPIO *pGPIODriver = &Driver_GPIO_KSDK;

	 /* Set SwEn pin High */
	 pGPIODriver->set_pin(pImaxPin);

	 return SENSOR_ERROR_NONE;
}

int32_t PCA9957_Set_Imax_20MA(void *pImaxPin)
{
	 GENERIC_DRIVER_GPIO *pGPIODriver = &Driver_GPIO_KSDK;

	 /* Set SwEn pin Low */
	 pGPIODriver->clr_pin(pImaxPin);

	 return SENSOR_ERROR_NONE;
}

void PCA9957_SPI_SetIdleTask(pca9957_spi_sensorhandle_t *pSensorHandle,
                              registeridlefunction_t idleTask,
                              void *userParam)
{
    pSensorHandle->deviceInfo.functionParam = userParam;
    pSensorHandle->deviceInfo.idleFunction = idleTask;
}


int32_t PCA9957_SPI_ReadData(pca9957_spi_sensorhandle_t *pSensorHandle,
                              const registerreadlist_t *pReadList,
                              uint8_t *pBuffer)
{
    int32_t status;

    /*! Validate for the correct handle and register read list.*/
    if ((pSensorHandle == NULL) || (pReadList == NULL) || (pBuffer == NULL))
    {
        return SENSOR_ERROR_INVALID_PARAM;
    }

    /*! Check whether sensor handle is initialized before reading sensor data.*/
    if (pSensorHandle->isInitialized != true)
    {
        return SENSOR_ERROR_INIT;
    }

    /*! Parse through the read list and read the data one by one. */
    status = Sensor_SPI_Read(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
                             pReadList, pBuffer);
    if (ARM_DRIVER_OK != status)
    {
        return SENSOR_ERROR_READ;
    }

    return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Deinit(pca9957_spi_sensorhandle_t *pSensorHandle, void *pResetPin)
{
    int32_t status;
    GENERIC_DRIVER_GPIO *pGPIODriver = &Driver_GPIO_KSDK;

    /*! Validate for the correct handle */
    if (pSensorHandle == NULL)
    {
        return SENSOR_ERROR_INVALID_PARAM;
    }

    /*! Check whether sensor handle is initialized before triggering sensor reset.*/
    if (pSensorHandle->isInitialized != true)
    {
        return SENSOR_ERROR_INIT;
    }

    /* Set Reset pin Low for PCA9957_RESET_DELAY_MS (Trigger device reset)*/
   	pGPIODriver->clr_pin(pResetPin);
   	BOARD_DELAY_ms(PCA9957_RESET_DELAY_MS);
   	pGPIODriver->set_pin(pResetPin);

	/*! De-initialize sensor handle. */
	pSensorHandle->isInitialized = false;

    return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_All_LED_Off(pca9957_spi_sensorhandle_t *pSensorHandle)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/* Set Led channel LED off */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			  PCA9957_IREFALL ,(uint8_t)(PCA9957_ALL_LED_OC_OFF), PCA9957_ALL_LED_OC_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	/* Set Default Brightness for all LED */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			PCA9957_PWMALL ,(uint8_t)(PCA9957_ALL_LED_LOW_PWM), PCA9957_ALL_LED_PWM_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_All_LED_On(pca9957_spi_sensorhandle_t *pSensorHandle)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/* Set All Led channel on */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			  PCA9957_IREFALL ,(uint8_t)(PCA9957_ALL_LED_DFT_OC), PCA9957_ALL_LED_OC_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	/* Set Default Brightness for all LED */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			PCA9957_PWMALL ,(uint8_t)(PCA9957_ALL_LED_DFT_PWM), PCA9957_ALL_LED_PWM_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Set_All_LED_Brightness(pca9957_spi_sensorhandle_t *pSensorHandle, LedBrightness brightness)
{
	int32_t status;

	/*! Validate for the correct handle */
	if ((pSensorHandle == NULL) && (brightness > PCA9957_MAX_LED_BRIGHTNESS))
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/* Set All Led channel Brightness */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			PCA9957_PWMALL ,(uint8_t)brightness, PCA9957_ALL_LED_PWM_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Set_All_LED_OP_Current_Gain(pca9957_spi_sensorhandle_t *pSensorHandle, CurrentGain currentgain)
{
	int32_t status;

	/*! Validate for the correct handle */
	if ((pSensorHandle == NULL) && (currentgain > PCA9957_MAX_CURRENT_GAIN))
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/* Set All Led channel Output Current */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			PCA9957_IREFALL ,(uint8_t)currentgain, PCA9957_ALL_LED_OC_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Set_LED_OP_Current_Gain(pca9957_spi_sensorhandle_t *pSensorHandle, LedNum lednum, CurrentGain currentgain)
{
	int32_t status;

	/*! Validate for the correct handle */
	if ((pSensorHandle == NULL) && (currentgain > PCA9957_MAX_CURRENT_GAIN))
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/* Set Individual Led channel Output Current */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			(PCA9957_IREF0 + lednum ) ,(uint8_t)currentgain, PCA9957_ALL_LED_OC_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_LED_On(pca9957_spi_sensorhandle_t *pSensorHandle, LedNum lednum)
{
	int32_t status;

	/*! Validate for the correct handle */
	if ((pSensorHandle == NULL) && (lednum > PCA9957_MAX_LED_PORT))
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/*  Individual LED channel On */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			( PCA9957_IREF0 + lednum ) ,(uint8_t)(PCA9957_LED_DFT_OC), PCA9957_LED_OC_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	/* Set Default Brightness for Individual LED channel */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			( PCA9957_PWM0 + lednum) ,(uint8_t)(PCA9957_ALL_LED_DFT_PWM), PCA9957_ALL_LED_PWM_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_LED_Off(pca9957_spi_sensorhandle_t *pSensorHandle, LedNum lednum)
{
	int32_t status;

	/*! Validate for the correct handle */
	if ((pSensorHandle == NULL) && (lednum > PCA9957_MAX_LED_PORT))
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

    /*  Individual LED channel Off */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			( PCA9957_IREF0 + lednum ) ,(uint8_t)(PCA9957_LED_OC_OFF), PCA9957_LED_OC_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	/* Set Low Brightness for Individual LED channel */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			( PCA9957_PWM0 + lednum) ,(uint8_t)(PCA9957_ALL_LED_LOW_PWM), PCA9957_ALL_LED_PWM_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Set_LED_Brightness(pca9957_spi_sensorhandle_t *pSensorHandle, LedNum lednum, LedBrightness brightness)
{
	int32_t status;

	/*! Validate for the correct handle */
	if ((pSensorHandle == NULL) && (brightness > PCA9957_MAX_LED_BRIGHTNESS))
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/* Set Brightness for Individual LED channel */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			(PCA9957_PWM0 + lednum) ,(uint8_t)brightness, PCA9957_ALL_LED_PWM_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Set_LED_State(pca9957_spi_sensorhandle_t *pSensorHandle, LedNum lednum, LedState ledstate)
{
    int32_t status;

    /*! Validate for the correct handle */
    if ((pSensorHandle == NULL) || (lednum > PCA9957_MAX_LED_PORT))
    {
        return SENSOR_ERROR_INVALID_PARAM;
    }

    /*! Check whether sensor handle is initialized before triggering sensor reset.*/
    if (pSensorHandle->isInitialized != true)
    {
        return SENSOR_ERROR_INIT;
    }

	/*! Set Individual LED channel State */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			(PCA9957_LEDOUT0 + (lednum / 4)),(uint8_t)(ledstate << ((lednum % 4) * 2)), (PCA9957_LED_STATE_MASK << ((lednum % 4) * 2)));
    if (ARM_DRIVER_OK != status)
    {
        return SENSOR_ERROR_WRITE;
    }

    return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Set_All_LED_State(pca9957_spi_sensorhandle_t *pSensorHandle, LedState ledstate)
{
	 int32_t status;
	 uint8_t temp;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	switch(ledstate)
	{
		case sledOff:  /* Set Individual LED channel State off */
		{
			temp = PCA9957_ALL_LED_OFF;
			break;
		}
		case sledOn:  /* Set Individual LED channel State on */
		{
			temp = PCA9957_ALL_LED_ON;
			break;
		}
		case sLedBrightness:   /* Set Individual LED channel State Brightness */
		{
			temp = PCA9957_ALL_LED_BRIGHTNESS;
			break;
		}
		case sLedBrightnessgDimmBlink: /* Set Individual LED channel State Brightness and Dimm Blink*/
		{
			temp = PCA9957_ALL_LED_BRIGHTNESS_DIMMBLINK;
			break;
		}
		default:
		{
			return SENSOR_ERROR_INVALID_PARAM;
			break;
		}
	}

	/* Set LED channel 0 to 3 State */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
				(PCA9957_LEDOUT0), temp, PCA9957_ALL_LED_MASK);
	if (ARM_DRIVER_OK != status)
	{
			return SENSOR_ERROR_WRITE;
	}

	/* Set LED channel 4 to 7 State */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
				(PCA9957_LEDOUT1), temp, PCA9957_ALL_LED_MASK);
	if (ARM_DRIVER_OK != status)
	{
			return SENSOR_ERROR_WRITE;
	}

	/* Set LED channel 8 to 11 State */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
				(PCA9957_LEDOUT2), temp, PCA9957_ALL_LED_MASK);
	if (ARM_DRIVER_OK != status)
	{
			return SENSOR_ERROR_WRITE;
	}

	/* Set LED channel 12 to 15 State */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
				(PCA9957_LEDOUT3), temp, PCA9957_ALL_LED_MASK);
	if (ARM_DRIVER_OK != status)
	{
			return SENSOR_ERROR_WRITE;
	}

	/* Set LED channel 16 to 19 State */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
				(PCA9957_LEDOUT4), temp, PCA9957_ALL_LED_MASK);
	if (ARM_DRIVER_OK != status)
	{
			return SENSOR_ERROR_WRITE;
	}

	/* Set LED channel 20 to 23 State */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
				(PCA9957_LEDOUT5), temp, PCA9957_ALL_LED_MASK);
	if (ARM_DRIVER_OK != status)
	{
			return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Set_Group_Ramp_Value(pca9957_spi_sensorhandle_t *pSensorHandle, GradationGroup gradationgroup, GroupRampRate groupramprate)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL || (groupramprate > PCA9957_MAX_GROUP_RAMP))
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

   /* Set Group Ramp Value for LED channel  */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			(PCA9957_RAMP_RATE_GRP0 + (gradationgroup * 4)),(uint8_t)(groupramprate << PCA9957_RAMP_RATE_VALUE_SHIFT), PCA9957_RAMP_RATE_VALUE_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}


int32_t PCA9957_SPI_Enable_Group_RampUp(pca9957_spi_sensorhandle_t *pSensorHandle, GradationGroup gradationgroup)
{
    int32_t status;

    /*! Validate for the correct handle */
    if (pSensorHandle == NULL)
    {
        return SENSOR_ERROR_INVALID_PARAM;
    }

    /*! Check whether sensor handle is initialized before triggering sensor reset.*/
    if (pSensorHandle->isInitialized != true)
    {
        return SENSOR_ERROR_INIT;
    }

    /* Enable Group Ramp Up LED channel */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			(PCA9957_RAMP_RATE_GRP0 + (gradationgroup * 4)),(uint8_t)(rampEnable << PCA9957_RAMP_UP_ENABLE_SHIFT), PCA9957_RAMP_UP_ENABLE_MASK);
    if (ARM_DRIVER_OK != status)
    {
        return SENSOR_ERROR_WRITE;
    }

    return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Disable_Group_RampUp(pca9957_spi_sensorhandle_t *pSensorHandle, GradationGroup gradationgroup)
{
    int32_t status;

    /*! Validate for the correct handle */
    if (pSensorHandle == NULL)
    {
        return SENSOR_ERROR_INVALID_PARAM;
    }

    /*! Check whether sensor handle is initialized before triggering sensor reset.*/
    if (pSensorHandle->isInitialized != true)
    {
        return SENSOR_ERROR_INIT;
    }

    /* Disable Group Ramp Up LED channel  */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			(PCA9957_RAMP_RATE_GRP0 + (gradationgroup * 4)),(uint8_t)(rampDisable << PCA9957_RAMP_UP_DISABLE_SHIFT), PCA9957_RAMP_UP_DISABLE_MASK);
    if (ARM_DRIVER_OK != status)
    {
        return SENSOR_ERROR_WRITE;
    }

    return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Enable_Group_RampDown(pca9957_spi_sensorhandle_t *pSensorHandle, GradationGroup gradationgroup)
{
    int32_t status;

    /*! Validate for the correct handle */
    if (pSensorHandle == NULL)
    {
        return SENSOR_ERROR_INVALID_PARAM;
    }

    /*! Check whether sensor handle is initialized before triggering sensor reset.*/
    if (pSensorHandle->isInitialized != true)
    {
        return SENSOR_ERROR_INIT;
    }

    /* Enable Group Ramp Down for LED channel */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			(PCA9957_RAMP_RATE_GRP0 + (gradationgroup * 4)),(uint8_t)(rampEnable << PCA9957_RAMP_DOWN_ENABLE_SHIFT), PCA9957_RAMP_DOWN_ENABLE_MASK);
    if (ARM_DRIVER_OK != status)
    {
        return SENSOR_ERROR_WRITE;
    }

    return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Disable_Group_RampDown(pca9957_spi_sensorhandle_t *pSensorHandle, GradationGroup gradationgroup)
{
    int32_t status;

    /*! Validate for the correct handle */
    if (pSensorHandle == NULL)
    {
        return SENSOR_ERROR_INVALID_PARAM;
    }

    /*! Check whether sensor handle is initialized before triggering sensor reset.*/
    if (pSensorHandle->isInitialized != true)
    {
        return SENSOR_ERROR_INIT;
    }

    /* Disable Group Ramp Down for LED channel */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			(PCA9957_RAMP_RATE_GRP0 + (gradationgroup * 4)),(uint8_t)(rampDisable << PCA9957_RAMP_DOWN_DISABLE_SHIFT), PCA9957_RAMP_DOWN_DISABLE_MASK);
    if (ARM_DRIVER_OK != status)
    {
        return SENSOR_ERROR_WRITE;
    }

    return SENSOR_ERROR_NONE;
}


int32_t PCA9957_SPI_Set_Cycle_Time(pca9957_spi_sensorhandle_t *pSensorHandle, GradationGroup gradationgroup, CycleTime cycletime)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/* Set cycle time for Gradation Group */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			(PCA9957_STEP_TIME_GRP0 + (gradationgroup * 4)),(uint8_t)(cycletime << PCA9957_STEP_CYCLE_TIME_SHIFT), PCA9957_STEP_CYCLE_TIME_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Set_Group_MF(pca9957_spi_sensorhandle_t *pSensorHandle, GradationGroup gradationgroup, GroupMultipleFactor groupmultiplefactor)
{
    int32_t status;

    /*! Validate for the correct handle */
    if ((pSensorHandle == NULL) || (groupmultiplefactor > PCA9957_MAX_GROUP_MF))
    {
        return SENSOR_ERROR_INVALID_PARAM;
    }

    /*! Check whether sensor handle is initialized before triggering sensor reset.*/
    if (pSensorHandle->isInitialized != true)
    {
        return SENSOR_ERROR_INIT;
    }

    /* Set Multiple Factor for Gradation Group */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			(PCA9957_STEP_TIME_GRP0 + (gradationgroup * 4)),(uint8_t)(groupmultiplefactor << PCA9957_STEP_TIME_MF_SHIFT), PCA9957_STEP_TIME_MF_MASK);
    if (ARM_DRIVER_OK != status)
    {
        return SENSOR_ERROR_WRITE;
    }

    return SENSOR_ERROR_NONE;
}


int32_t PCA9957_SPI_Enable_HoldOn(pca9957_spi_sensorhandle_t *pSensorHandle, GradationGroup gradationgroup, HoldTimeSelect holdtimeselect)
{
    int32_t status;

    /*! Validate for the correct handle */
    if (pSensorHandle == NULL)
    {
        return SENSOR_ERROR_INVALID_PARAM;
    }

    /*! Check whether sensor handle is initialized before triggering sensor reset.*/
    if (pSensorHandle->isInitialized != true)
    {
        return SENSOR_ERROR_INIT;
    }

    /* Enable Group Hold On for LED channel */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			(PCA9957_HOLD_CNTL_GRP0 + (gradationgroup * 4)),(uint8_t)(holdEnable << PCA9957_HOLD_ON_CNTL_SHIFT), PCA9957_HOLD_ON_CNTL_MASK);
    if (ARM_DRIVER_OK != status)
    {
        return SENSOR_ERROR_WRITE;
    }

    /* Set Group Hold On for LED channel */
    status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			(PCA9957_HOLD_CNTL_GRP0 + (gradationgroup * 4)),(uint8_t)(holdtimeselect << PCA9957_HOLD_ON_TIME_SHIFT), PCA9957_HOLD_ON_TIME_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

    return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Disable_HoldOn(pca9957_spi_sensorhandle_t *pSensorHandle, GradationGroup gradationgroup)
{
    int32_t status;

    /*! Validate for the correct handle */
    if (pSensorHandle == NULL)
    {
        return SENSOR_ERROR_INVALID_PARAM;
    }

    /*! Check whether sensor handle is initialized before triggering sensor reset.*/
    if (pSensorHandle->isInitialized != true)
    {
        return SENSOR_ERROR_INIT;
    }

    /* Disable Group Hold On for LED channel */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			(PCA9957_HOLD_CNTL_GRP0 + (gradationgroup * 4)),(uint8_t)(holdDisable << PCA9957_HOLD_ON_CNTL_SHIFT), PCA9957_HOLD_ON_CNTL_MASK);
    if (ARM_DRIVER_OK != status)
    {
        return SENSOR_ERROR_WRITE;
    }

    return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Enable_HoldOff(pca9957_spi_sensorhandle_t *pSensorHandle, GradationGroup gradationgroup, HoldTimeSelect holdtimeselect)
{
    int32_t status;

    /*! Validate for the correct handle */
    if (pSensorHandle == NULL)
    {
        return SENSOR_ERROR_INVALID_PARAM;
    }

    /*! Check whether sensor handle is initialized before triggering sensor reset.*/
    if (pSensorHandle->isInitialized != true)
    {
        return SENSOR_ERROR_INIT;
    }

    /* Enable Group Hold Off for LED channel */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			(PCA9957_HOLD_CNTL_GRP0 + (gradationgroup * 4)),(uint8_t)(holdEnable << PCA9957_HOLD_OFF_CNTL_SHIFT), PCA9957_HOLD_OFF_CNTL_MASK);
    if (ARM_DRIVER_OK != status)
    {
        return SENSOR_ERROR_WRITE;
    }

    /* Set Group Hold Off for LED channel */
    status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
    		(PCA9957_HOLD_CNTL_GRP0 + (gradationgroup * 4)),(uint8_t)(holdtimeselect << PCA9957_HOLD_OFF_TIME_SHIFT), PCA9957_HOLD_OFF_TIME_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

    return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Disable_HoldOff(pca9957_spi_sensorhandle_t *pSensorHandle, GradationGroup gradationgroup)
{
    int32_t status;

    /*! Validate for the correct handle */
    if (pSensorHandle == NULL)
    {
        return SENSOR_ERROR_INVALID_PARAM;
    }

    /*! Check whether sensor handle is initialized before triggering sensor reset.*/
    if (pSensorHandle->isInitialized != true)
    {
        return SENSOR_ERROR_INIT;
    }

    /* Disable Group Hold Off for LED channel */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			(PCA9957_HOLD_CNTL_GRP0 + (gradationgroup * 4)),(uint8_t)(holdDisable << PCA9957_HOLD_OFF_CNTL_SHIFT), PCA9957_HOLD_OFF_CNTL_MASK);
    if (ARM_DRIVER_OK != status)
    {
        return SENSOR_ERROR_WRITE;
    }

    return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Set_Gradation_Group_Current_Gain(pca9957_spi_sensorhandle_t *pSensorHandle, GradationGroup gradationgroup, CurrentGain currentgain)
{
    int32_t status;

    /*! Validate for the correct handle */
    if ((pSensorHandle == NULL) || (currentgain > PCA9957_MAX_CURRENT_GAIN))
    {
        return SENSOR_ERROR_INVALID_PARAM;
    }

    /*! Check whether sensor handle is initialized before triggering sensor reset.*/
    if (pSensorHandle->isInitialized != true)
    {
        return SENSOR_ERROR_INIT;
    }

    /* Set Group Current gain for LED channel */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			(PCA9957_IREF_GRP0 + (gradationgroup * 4)),(uint8_t)(currentgain << PCA9957_CURRENT_GAIN_SHIFT), PCA9957_CURRENT_GAIN_MASK);
    if (ARM_DRIVER_OK != status)
    {
        return SENSOR_ERROR_WRITE;
    }

    return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Enable_Gradation_Mode(pca9957_spi_sensorhandle_t *pSensorHandle, LedNum lednum)
{
    int32_t status;

    /*! Validate for the correct handle */
    if ((pSensorHandle == NULL) || (lednum > PCA9957_MAX_LED_PORT))
    {
        return SENSOR_ERROR_INVALID_PARAM;
    }

    /*! Check whether sensor handle is initialized before triggering sensor reset.*/
    if (pSensorHandle->isInitialized != true)
    {
        return SENSOR_ERROR_INIT;
    }

    /* Enable Gradation mode for LED channel */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			(PCA9957_GRAD_MODE_SEL0 + (lednum  / 8)),(uint8_t)(gradMode << (lednum % 8)), (uint8_t)(PCA9957_ONE_BIT_MASK << (lednum % 8)));
    if (ARM_DRIVER_OK != status)
    {
        return SENSOR_ERROR_WRITE;
    }

    status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			(PCA9957_PWM0 + lednum) , PCA9957_ALL_LED_DFT_PWM , PCA9957_ALL_LED_PWM_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

    return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Enable_All_LED_Gradation_Mode(pca9957_spi_sensorhandle_t *pSensorHandle)
{
    int32_t status;

    /*! Validate for the correct handle */
    if (pSensorHandle == NULL)
    {
        return SENSOR_ERROR_INVALID_PARAM;
    }

    /*! Check whether sensor handle is initialized before triggering sensor reset.*/
    if (pSensorHandle->isInitialized != true)
    {
        return SENSOR_ERROR_INIT;
    }

    /* Enable Gradation mode for LED channel 0 to 7 */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			(PCA9957_GRAD_MODE_SEL0), (uint8_t)(PCA9957_ALL_LED_GRAD_EN), (uint8_t)(PCA9957_ALL_LED_MASK));
    if (ARM_DRIVER_OK != status)
    {
        return SENSOR_ERROR_WRITE;
    }

    /* Enable Gradation mode for LED channel 8 to 15 */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			(PCA9957_GRAD_MODE_SEL1), (uint8_t)(PCA9957_ALL_LED_GRAD_EN), (uint8_t)(PCA9957_ALL_LED_MASK));
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	/* Enable Gradation mode for LED channel 15 to 23 */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			(PCA9957_GRAD_MODE_SEL2), (uint8_t)(PCA9957_ALL_LED_GRAD_EN), (uint8_t)(PCA9957_ALL_LED_MASK));
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

    ///check
    status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			(PCA9957_PWMALL) , PCA9957_ALL_LED_DFT_PWM , PCA9957_ALL_LED_PWM_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

    return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Disable_Gradation_Mode(pca9957_spi_sensorhandle_t *pSensorHandle, LedNum lednum)
{
    int32_t status;

    /*! Validate for the correct handle */
    if ((pSensorHandle == NULL) || (lednum > PCA9957_MAX_LED_PORT))
    {
        return SENSOR_ERROR_INVALID_PARAM;
    }

    /*! Check whether sensor handle is initialized before triggering sensor reset.*/
    if (pSensorHandle->isInitialized != true)
    {
        return SENSOR_ERROR_INIT;
    }

    /* Disable Gradation mode for LED channel */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			(PCA9957_GRAD_MODE_SEL0 + (lednum  / 8)),(uint8_t)(normalMode << (lednum % 8)), (uint8_t)(PCA9957_ONE_BIT_MASK << (lednum % 8)));
    if (ARM_DRIVER_OK != status)
    {
        return SENSOR_ERROR_WRITE;
    }

    return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Disable_All_LED_Gradation_Mode(pca9957_spi_sensorhandle_t *pSensorHandle)
{
    int32_t status;

    /*! Validate for the correct handle */
    if (pSensorHandle == NULL)
    {
        return SENSOR_ERROR_INVALID_PARAM;
    }

    /*! Check whether sensor handle is initialized before triggering sensor reset.*/
    if (pSensorHandle->isInitialized != true)
    {
        return SENSOR_ERROR_INIT;
    }

	/* Disable Gradation mode for LED channel 0 to 7 */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			(PCA9957_GRAD_MODE_SEL0), (uint8_t)(PCA9957_ALL_LED_GRAD_DIS), (uint8_t)(PCA9957_ALL_LED_MASK));
    if (ARM_DRIVER_OK != status)
    {
        return SENSOR_ERROR_WRITE;
    }

    /* Disable Gradation mode for LED channel 8 to 15 */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			(PCA9957_GRAD_MODE_SEL1), (uint8_t)(PCA9957_ALL_LED_GRAD_DIS), (uint8_t)(PCA9957_ALL_LED_MASK));
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	/* Disable Gradation mode for LED channel 16 to 23 */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			(PCA9957_GRAD_MODE_SEL2), (uint8_t)(PCA9957_ALL_LED_GRAD_DIS), (uint8_t)(PCA9957_ALL_LED_MASK));
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

    return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Assign_LED_to_Gradation_Group(pca9957_spi_sensorhandle_t *pSensorHandle, LedNum lednum, GradationGroup gradationgroup)
{
    int32_t status;

    /*! Validate for the correct handle */
    if ((pSensorHandle == NULL) || (lednum > PCA9957_MAX_LED_PORT))
    {
        return SENSOR_ERROR_INVALID_PARAM;
    }

    /*! Check whether sensor handle is initialized before triggering sensor reset.*/
    if (pSensorHandle->isInitialized != true)
    {
        return SENSOR_ERROR_INIT;
    }

    /* Assign LED channel to gradation group */
    	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
					(PCA9957_IREF_GRAD_GRP_SEL0 + (lednum  / 2)),
					(uint8_t)(gradationgroup << ((lednum % 2) ? PCA9957_GRAD_GRP_SECOND_LED_SHIFT : PCA9957_GRAD_GRP_FIRST_LED_SHIFT)),
					(uint8_t)((lednum % 2) ? PCA9957_GRAD_GRP_SECOND_LED_MASK : PCA9957_GRAD_GRP_FIRST_LED_MASK));
    if (ARM_DRIVER_OK != status)
    {
        return SENSOR_ERROR_WRITE;
    }

    return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Gradation_Group_Start(pca9957_spi_sensorhandle_t *pSensorHandle,  GradationGroup gradationgroup, GradOperation grandoperation)
{
    int32_t status;
    uint8_t goprsn;

    /*! Validate for the correct handle */
    if (pSensorHandle == NULL)
    {
        return SENSOR_ERROR_INVALID_PARAM;
    }

    /*! Check whether sensor handle is initialized before triggering sensor reset.*/
    if (pSensorHandle->isInitialized != true)
    {
        return SENSOR_ERROR_INIT;
    }

    /*! Set Gradation group Operation (Single / Continuous) */
    switch(gradationgroup)
    {
    	case GradationGroup0:
    	{
    		if(grandoperation == continuous)
    			goprsn = PCA9957_GROUP0_CONT_START;
    		else
    			goprsn = PCA9957_GROUP0_SINGLE_START;

    		break;
    	}
    	case GradationGroup1:
    	{
    		if(grandoperation == continuous)
				goprsn = PCA9957_GROUP1_CONT_START;
			else
				goprsn = PCA9957_GROUP1_SINGLE_START;

    		break;
    	}
    	case GradationGroup2:
    	{
    		if(grandoperation == continuous)
				goprsn = PCA9957_GROUP2_CONT_START;
			else
				goprsn = PCA9957_GROUP2_SINGLE_START;

    		break;
    	}
    	case GradationGroup3:
    	{
    		if(grandoperation == continuous)
				goprsn = PCA9957_GROUP3_CONT_START;
			else
				goprsn = PCA9957_GROUP3_SINGLE_START;

    		break;
    	}
    	case GradationGroup4:
    	{
    		if(grandoperation == continuous)
				goprsn = PCA9957_GROUP4_CONT_START;
			else
				goprsn = PCA9957_GROUP4_SINGLE_START;

    		break;
    	}
    	case GradationGroup5:
    	{
    		if(grandoperation == continuous)
				goprsn = PCA9957_GROUP5_CONT_START;
			else
				goprsn = PCA9957_GROUP5_SINGLE_START;

    		break;
    	}
    	default:
    		break;
    }

	/* Start Gradation for group (0 to 3)*/
    if(gradationgroup <= 3)
    {
    	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
					 PCA9957_CNTL0, goprsn, (uint8_t)(PCA9957_TWO_BIT_MASK << (gradationgroup * 2)));
    }
    else  /* Start Gradation for group (4 to 5) */
    {
    	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
    				PCA9957_CNTL1, goprsn, (uint8_t)(PCA9957_TWO_BIT_MASK << ((gradationgroup - 2) * 2)));
    }
    if (ARM_DRIVER_OK != status)
    {
        return SENSOR_ERROR_WRITE;
    }

    return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Gradation_Group_Stop(pca9957_spi_sensorhandle_t *pSensorHandle,  GradationGroup gradationgroup)
{
    int32_t status;

    /*! Validate for the correct handle */
    if (pSensorHandle == NULL)
    {
        return SENSOR_ERROR_INVALID_PARAM;
    }

    /*! Check whether sensor handle is initialized before triggering sensor reset.*/
    if (pSensorHandle->isInitialized != true)
    {
        return SENSOR_ERROR_INIT;
    }

    /* Stop Gradation for group */
    if(gradationgroup <= 3)
    {
    	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
    				(PCA9957_CNTL0 + (gradationgroup  / 4)),(uint8_t)(gradStop << ((gradationgroup * 2) + 1)),
					(uint8_t)(PCA9957_ONE_BIT_MASK << ((gradationgroup * 2) + 1)));
    }
    else  /* Stop Gradation for group */
    {
    	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
    				(PCA9957_CNTL0 + (gradationgroup  / 4)),(uint8_t)(gradStop << (((gradationgroup - 2) * 2) + 1)),
					(uint8_t)(PCA9957_ONE_BIT_MASK << (((gradationgroup - 2) * 2) + 1)));
    }
    if (ARM_DRIVER_OK != status)
    {
        return SENSOR_ERROR_WRITE;
    }

    return SENSOR_ERROR_NONE;
}


int32_t PCA9957_SPI_Set_All_LED_Dimm_Blink_Brightness(pca9957_spi_sensorhandle_t *pSensorHandle, LedBrightness brightness)
{
	int32_t status;

	/*! Validate for the correct handle */
	if ((pSensorHandle == NULL) && (brightness > PCA9957_MAX_LED_BRIGHTNESS))
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/* Set Dimm Blink Brightness */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			PCA9957_GRPPWM ,(uint8_t)brightness, PCA9957_GRPPWM_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Set_All_LED_Dimm_Blink_Frequency(pca9957_spi_sensorhandle_t *pSensorHandle, GroupFrequency groupfrequency)
{
	int32_t status;

	/*! Validate for the correct handle */
	if ((pSensorHandle == NULL) && (groupfrequency > PCA9957_MAX_GRP_FREQUENCY))
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/* Set Dimm Blink Brightness Frequency */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			PCA9957_GRPFREQ ,(uint8_t)groupfrequency, PCA9957_GRPFREQ_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Set_LED_OP_Delay(pca9957_spi_sensorhandle_t *pSensorHandle, ClockDelay clockdelay)
{
	int32_t status;

	/*! Validate for the correct handle */
	if ((pSensorHandle == NULL) && (clockdelay > PCA9957_MAX_CLOCK_DELAY))
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/* Set LED channel output delay cycle */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			PCA9957_OFFSET ,(uint8_t)(clockdelay << PCA9957_CLOCK_DELAY_SHIFT) , PCA9957_CLOCK_DELAY_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Set_All_LED_Dimm_Blink(pca9957_spi_sensorhandle_t *pSensorHandle, GradDimmBlink graddimmblink)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/* Set All LED Channel Dimm Blink */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			PCA9957_MODE2 ,(uint8_t)(graddimmblink << PCA9957_DMBLNK_SHIFT) , PCA9957_DMBLNK_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Set_Gradation_Adjustment(pca9957_spi_sensorhandle_t *pSensorHandle, GrandAdjustment grandadjustment)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/* Set Gradation group Adjustment */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
				PCA9957_MODE2 ,(uint8_t)(grandadjustment << PCA9957_EXP_EN_SHIFT) , PCA9957_EXP_EN_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Enable_Sleep(pca9957_spi_sensorhandle_t *pSensorHandle)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/* Enable Sleep (Oscillator on) */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
				PCA9957_MODE1 ,(uint8_t)(lowPowerMode << PCA9957_SLEEP_SHIFT) , PCA9957_SLEEP_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Disable_Sleep(pca9957_spi_sensorhandle_t *pSensorHandle)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/* Disable Sleep (Oscillator off) */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			PCA9957_MODE1 ,(uint8_t)(standByMode << PCA9957_SLEEP_SHIFT) , PCA9957_SLEEP_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Enable_Auto_SwitchOff(pca9957_spi_sensorhandle_t *pSensorHandle)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/* Enable auto switch off on error */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			PCA9957_MODE2 ,(uint8_t)(autoDisableOn << PCA9957_AUTO_SWITCHOFF_DIS_SHIFT) , PCA9957_AUTO_SWITCHOFF_DIS_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Disable_Auto_SwitchOff(pca9957_spi_sensorhandle_t *pSensorHandle)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/* Disable auto switch off on error */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			PCA9957_MODE2 ,(uint8_t)(autoDisableOff << PCA9957_AUTO_SWITCHOFF_DIS_SHIFT) , PCA9957_AUTO_SWITCHOFF_DIS_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Clear_Error(pca9957_spi_sensorhandle_t *pSensorHandle)
{
	int32_t status;

	/*! Validate for the correct handle */
	if (pSensorHandle == NULL)
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/*  clear Error */
	status = Register_SPI_Write(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			PCA9957_MODE2 ,(uint8_t)( PCA9957_CLRERR << PCA9957_CLRERR_SHIFT) , PCA9957_CLRERR_MASK);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Check_Error(pca9957_spi_sensorhandle_t *pSensorHandle, ErrorState *errorstate)
{
	int32_t status;
	PCA9957_MODE_2 Mode2_Reg;

	/*! Validate for the correct handle */
	if ((pSensorHandle == NULL) || (errorstate == NULL))
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/*  check Error */
	status = Register_SPI_Read(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			PCA9957_MODE2 , PCA9957_REG_SIZE_BYTE, (uint8_t *)&Mode2_Reg);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	*errorstate = Mode2_Reg.b.error;

	return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_Check_OvTemp(pca9957_spi_sensorhandle_t *pSensorHandle, TempCondition *tempcondition)
{
	int32_t status;
	PCA9957_MODE_2 Mode2_Reg;

	/*! Validate for the correct handle */
	if ((pSensorHandle == NULL) || (tempcondition == NULL))
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/* check over temperature */
	status = Register_SPI_Read(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			PCA9957_MODE2 , PCA9957_REG_SIZE_BYTE, (uint8_t *)&Mode2_Reg);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	*tempcondition = Mode2_Reg.b.overtemp;

	return SENSOR_ERROR_NONE;
}

int32_t PCA9957_SPI_LED_Get_Error_Type(pca9957_spi_sensorhandle_t *pSensorHandle, LedNum lednum, LedErrorType *lederrortype)
{
	int32_t status;
	uint8_t lederr;

	/*! Validate for the correct handle */
	if ((pSensorHandle == NULL) || (lederrortype == NULL))
	{
		return SENSOR_ERROR_INVALID_PARAM;
	}

	/*! Check whether sensor handle is initialized before triggering sensor reset.*/
	if (pSensorHandle->isInitialized != true)
	{
		return SENSOR_ERROR_INIT;
	}

	/* Get LED Channel error typr */
	status = Register_SPI_Read(pSensorHandle->pCommDrv, &pSensorHandle->deviceInfo, &pSensorHandle->slaveParams,
			 (PCA9957_EFLAG0 + (lednum / 4)) , PCA9957_REG_SIZE_BYTE, &lederr);
	if (ARM_DRIVER_OK != status)
	{
		return SENSOR_ERROR_WRITE;
	}

	*lederrortype = ((lederr >> ((lednum % 4) * 2)) & PCA9957_LED_ERROR_TYPE_MASK);

	return SENSOR_ERROR_NONE;
}

