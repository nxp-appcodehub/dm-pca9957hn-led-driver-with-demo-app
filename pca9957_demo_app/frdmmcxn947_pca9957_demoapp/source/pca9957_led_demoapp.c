/*
 * Copyright 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file  pca9957_led_demoapp.c
 *  @brief The pca9957_led_demoapp.c file implements the ISSDK PCA9957 SPI LED driver
 *         example demonstration for SPI Mode with interrupts.
 */

//-----------------------------------------------------------------------
// SDK Includes
//-----------------------------------------------------------------------
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"

//-----------------------------------------------------------------------
// ISSDK Includes
//-----------------------------------------------------------------------
#include "issdk_hal.h"
#include "gpio_driver.h"
#include "systick_utils.h"
#include "Driver_GPIO.h"
#include "pca9957_drv.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#if (RTE_SPI1_DMA_EN)
#define EXAMPLE_DMA_BASEADDR (DMA0)
#define EXAMPLE_DMA_CLOCK    kCLOCK_Dma0
#endif

/*******************************************************************************
 * Macros
 ******************************************************************************/

#define ERROR_NONE       0
#define ERROR            1

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
uint8_t blinkFrequncy;
uint8_t blinkPwm;
uint8_t gradGroupI;
uint8_t gradStepTime;
uint8_t gradRampValue;
uint8_t glednum;
uint8_t gbrightness;
uint8_t goutputgain;

/*******************************************************************************
 * Constants
 ******************************************************************************/

/*!@brief        set Gradation Adjustment.
 *  @details     set Gradation Adjustment (Single / Continuous) shot.
 *  @param[in]   Pointer to spi sensor handle structure (pca9957Driver).
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No.
 */
void setGradAdjustment(pca9957_spi_sensorhandle_t *pca9957Driver)
{
	uint32_t status;
	uint8_t gradAdjustment;

	PRINTF("\r\n 1. Linear adjustment for gradation control \r\n");
	PRINTF("\r\n 2. Exponential adjustment for gradation control \r\n");

	PRINTF("\r\n Enter your choice :- ");
	gradAdjustment = GETCHAR();
	gradAdjustment -= 48;
	PRINTF("%d\r\n",gradAdjustment);
	GETCHAR();

	if( gradAdjustment >= 1 && gradAdjustment <= 2)
	{
		status = PCA9957_SPI_Set_Gradation_Adjustment(pca9957Driver, (gradAdjustment - 1));
		if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n Gradation Adjustment Set Failed\r\n");
		}
		else
			PRINTF("\r\n Gradation Adjustment Done \r\n");
	}
	else
	{
		PRINTF("\r\nInvalid Option \r\n");
	}
}

/*!@brief        set Led Output Delay .
 *  @details     Set Led Output Delay (No of Cycle) between LED Channel.
 *  @param[in]   Pointer to spi sensor handle structure (pca9957Driver).
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No.
 */
void setLedOpDelay(pca9957_spi_sensorhandle_t *pca9957Driver)
{
	uint32_t status;
	uint8_t ledopdelay;

	PRINTF("\r\n Enter the LED o/p Delay [1 to 12] :- ");
	SCANF("%d",&ledopdelay);
	PRINTF("%d\r\n",ledopdelay);

	if( ledopdelay >= 1 && ledopdelay <= 12)
	{
		status = PCA9957_SPI_Set_LED_OP_Delay(pca9957Driver,(ledopdelay -1));
		if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n LED Delay Set Failed\r\n");
		}
		else
			PRINTF("\r\n LED Delay Set Done \r\n");

	}
	else
	{
		PRINTF("\r\nInvalid Input \r\n");
	}
}

/*!@brief        Get Blink Frequency .
 *  @details     Get the Blink Frequency from user.
 *  @param[in]   NO
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      Blink Frequency.
 */
uint8_t getBlinkFrequncy()
{

	PRINTF("\r\n Enter the Blinking frequency [1 to 255] :- ");
	SCANF("%d",&blinkFrequncy);
	PRINTF("%d\r\n",blinkFrequncy);

	if( blinkFrequncy >= 1 && blinkFrequncy <= 255)
		return blinkFrequncy ;
	else
	{
		PRINTF("\r\nInvalid Input \r\n");
		return 0;
	}
}

/*!@brief        Get Blink PWM.
 *  @details     Get the Blink PWM from user.
 *  @param[in]   NO
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      Blink PWM.
 */
uint8_t getBlinkPwm()
{

	PRINTF("\r\n Enter the Blinking PWM [1 to 255] :- ");
	SCANF("%d",&blinkPwm);
	PRINTF("%d\r\n",blinkPwm);

	if( blinkPwm >= 1 && blinkPwm <= 255)
		return blinkPwm ;
	else
	{
		PRINTF("\r\nInvalid Input \r\n");
		return 0;
	}
}

/*!@brief        Get gradation group current.
 *  @details     Get gradation group current from user.
 *  @param[in]   NO
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      gradation group current.
 */
uint8_t getgradGroupI()
{

	PRINTF("\r\n Enter the Gradation Group Current [1 to 255] :- ");
	SCANF("%d",&gradGroupI);
	PRINTF("%d\r\n",gradGroupI);

	if( gradGroupI >= 1 && gradGroupI <= 255)
		return gradGroupI ;
	else
	{
		PRINTF("\r\nInvalid Input \r\n");
		return 0;
	}

}

/*!@brief        Get gradation Hold Time.
 *  @details     Get gradation Hold Time from user.
 *  @param[in]   NO
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      Hold Time.
 */
uint8_t getGradHoldTime()
{
	uint8_t gradHoldTime;

	PRINTF("\r\n Enter Hold Time in Second (0, 0.25, 0.5, 0.75, 1, 2, 4, 6) [1 to 8] :- ");
	gradHoldTime = GETCHAR();
	gradHoldTime -= 48;
	PRINTF("%d\r\n",gradHoldTime);
	GETCHAR();

	if( gradHoldTime >= 1 && gradHoldTime <= 8)
		return gradHoldTime ;
	else
	{
		PRINTF("\r\nInvalid Input \r\n");
		return 0;
	}

}

/*!@brief        Get gradation Step Time MF.
 *  @details     Get gradation Step Time MF from user.
 *  @param[in]   NO
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      Step Time MF.
 */
uint8_t getGradStepTimeMF()
{

	PRINTF("\r\n Enter Step Time [1 to 64] :- ");
	SCANF("%d",&gradStepTime);
	PRINTF("%d\r\n",gradStepTime);

	if( gradStepTime >= 1 && gradStepTime <= 64)
		return gradStepTime ;
	else
	{
		PRINTF("\r\nInvalid Input \r\n");
		return 0;
	}

}

/*!@brief        Get Ramp value.
 *  @details     Get Ramp value from user.
 *  @param[in]   NO
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      Ramp value.
 */
uint8_t getGradRampValue()
{

	PRINTF("\r\n Enter Ramp Value [1 to 64] :- ");
	SCANF("%d",&gradRampValue);
	PRINTF("%d\r\n",gradRampValue);

	if( gradRampValue >= 1 && gradRampValue <= 64)
		return gradRampValue ;
	else
	{
		PRINTF("\r\nInvalid Input \r\n");
		return 0;
	}

}

/*!@brief        Get Gradation group operation.
 *  @details     Get Gradation group operation (Single / Continuous) from user.
 *  @param[in]   Pointer to spi sensor handle structure (pca9957Driver).
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No.
 */
uint8_t  getGradGroupOprsn()
{
	uint32_t status;
	uint8_t goprsn;

	PRINTF("\r\n  1. Single Shot Gradation Operation \r\n");
	PRINTF("\r\n  2. Continuous Gradation Operation \r\n");

	PRINTF("\r\n Enter your choice :- ");
	goprsn = GETCHAR();
	goprsn -= 48;
	PRINTF("%d\r\n",goprsn);
	GETCHAR();

	if( goprsn >= 1 && goprsn <= 2)
		return goprsn;
	else
		PRINTF("\r\nInvalid Option \r\n");
}

/*!@brief        Get Led Number.
 *  @details     Get Led Number from user.
 *  @param[in]   NO
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      Led Number.
 */

uint8_t getLednum()
{
	PRINTF("\r\n Enter LED Number from 1 to 24 :- ");
	SCANF("%d",&glednum);
	PRINTF("%d\r\n",glednum);

	if( glednum >= 1 && glednum <= 24)
		return glednum ;
	else
	{
		PRINTF("\r\nInvalid Input\r\n");
		return 0;
	}
}

/*!@brief        Set Final Ramp Up current .
 *  @details     get Final Ramp Up current from user and set it..
 *  @param[in]   Pointer to spi sensor handle structure (pca9957Driver).
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No.
 */
uint8_t setFinalRampUpCurrent(pca9957_spi_sensorhandle_t *pca9957Driver, GradationGroup gradgroup)
{
	uint8_t gradFinalRampUp;
	int32_t status;

	PRINTF("\r\n Enter the RampUp Current Gain [1 to 255] :- ");
	SCANF("%d",&gradFinalRampUp);
	PRINTF("%d\r\n",gradFinalRampUp);

	if( gradFinalRampUp >= 1 && gradFinalRampUp <= 255)
	{
		status = PCA9957_SPI_Set_Gradation_Group_Current_Gain(pca9957Driver, gradgroup, (gradFinalRampUp -1));
		if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n Final Ramp UP current set Failed\r\n");
			return ERROR;
		}
		else
			PRINTF("\r\n Final Ramp UP current set Done \r\n");

		return gradFinalRampUp ;
	}
	else
	{
		PRINTF("\r\nInvalid Input \r\n");
		return 0;
	}

}

/*!@brief        Set Hold Control .
 *  @details     Configure hold control setting related to Hold on and off.
 *  @param[in]   Pointer to spi sensor handle structure (pca9957Driver).
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      status.
 */
uint8_t setHoldControl(pca9957_spi_sensorhandle_t *pca9957Driver, GradationGroup gradgroup)
{
	uint8_t holdOnOffControl, HoldTime, lhold = 1;
	int32_t status;

	do
	{
		PRINTF("\r\n **** Hold Configuration **** \r\n");
		PRINTF("\r\n 1: Enable Hold On\r\n");
		PRINTF("\r\n 2: Disable Hold On\r\n");
		PRINTF("\r\n 3: Enable Hold Off\r\n");
		PRINTF("\r\n 4: Disable Hold Off\r\n");
		PRINTF("\r\n 5. Exit from Hold Configuration \r\n");

		PRINTF("\r\n Enter your choice :- ");
		holdOnOffControl = GETCHAR();
		holdOnOffControl -= 48;
		PRINTF("%d\r\n",holdOnOffControl);
		GETCHAR();

		switch(holdOnOffControl)
		{
		case 1:
			HoldTime = getGradHoldTime();
			if(HoldTime)
			{
				status = PCA9957_SPI_Enable_HoldOn(pca9957Driver, gradgroup, (HoldTime - 1));
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n Hold On Enable Failed\r\n");
					return ERROR;
				}
				else
					PRINTF("\r\n Hold On Enabled \r\n");
			}
			break;
		case 2:
			status = PCA9957_SPI_Disable_HoldOn(pca9957Driver, gradgroup);
			if (SENSOR_ERROR_NONE != status)
			{
				PRINTF("\r\n Hold On Disable Failed\r\n");
				return ERROR;
			}
			else
				PRINTF("\r\n Hold On Disabled \r\n");
			break;
		case 3:
			HoldTime = getGradHoldTime();
			if(HoldTime)
			{
				status = PCA9957_SPI_Enable_HoldOff(pca9957Driver, gradgroup, (HoldTime - 1));
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n Hold Off Enable Failed\r\n");
					return ERROR;
				}
				else
					PRINTF("\r\n Hold Off Enabled \r\n");
			}
			break;
		case 4:
			status = PCA9957_SPI_Disable_HoldOff(pca9957Driver, gradgroup);
			if (SENSOR_ERROR_NONE != status)
			{
				PRINTF("\r\n Hold Off Disable Failed\r\n");
				return ERROR;
			}
			break;
		case 5:
			lhold = 0;
			break;
		default:
			PRINTF("\r\n Invalid Input \r\n");
			break;
		}
	}while(lhold);
}

/*!@brief        Set Ramp Rate .
 *  @details     Configure Ramp control setting related to RAmp Up and Down.
 *  @param[in]   Pointer to spi sensor handle structure (pca9957Driver).
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      status.
 */
uint8_t setRampRate(pca9957_spi_sensorhandle_t *pca9957Driver, GradationGroup gradgroup)
{
	uint8_t RampRate, rampControl, lramp = 1;
	int32_t status;

	do
	{
		PRINTF("\r\n **** Ramp Configuration **** \r\n");
		PRINTF("\r\n 1: Enable Ramp UP \r\n");
		PRINTF("\r\n 2: Disable Ramp UP \r\n");
		PRINTF("\r\n 3: Enable Ramp Down \r\n");
		PRINTF("\r\n 4: Disable Ramp Down \r\n");
		PRINTF("\r\n 5: Set Ramp Value \r\n");
		PRINTF("\r\n 6. Exit from Ramp Configuration \r\n");

		PRINTF("\r\n Enter your choice :- ");
		rampControl = GETCHAR();
		rampControl -= 48;
		PRINTF("%d\r\n",rampControl);
		GETCHAR();

		switch(rampControl)
		{
		case 1:
			status = PCA9957_SPI_Enable_Group_RampUp(pca9957Driver, gradgroup);
			if (SENSOR_ERROR_NONE != status)
			{
				PRINTF("\r\n Ramp UP Enable Failed\r\n");
				return ERROR;
			}
			else
				PRINTF("\r\n Ramp UP Enabled \r\n");

			break;
		case 2:
			status = PCA9957_SPI_Disable_Group_RampUp(pca9957Driver, gradgroup);
			if (SENSOR_ERROR_NONE != status)
			{
				PRINTF("\r\n Ramp UP Disable Failed\r\n");
				return ERROR;
			}
			else
				PRINTF("\r\n Ramp UP Disabled \r\n");
			break;
		case 3:
			status = PCA9957_SPI_Enable_Group_RampDown(pca9957Driver, gradgroup);
			if (SENSOR_ERROR_NONE != status)
			{
				PRINTF("\r\n Ramp Down Enable Failed \r\n");
				return ERROR;
			}
			else
				PRINTF("\r\n Ramp Down Enabled \r\n");
			break;
		case 4:
			status = PCA9957_SPI_Disable_Group_RampDown(pca9957Driver, gradgroup);
			if (SENSOR_ERROR_NONE != status)
			{
				PRINTF("\r\n Ramp Down Disabled Failed \r\n");
				return ERROR;
			}
			else
				PRINTF("\r\n Ramp Down Disabled \r\n");
			break;
		case 5:
			RampRate = getGradRampValue();
			if(RampRate)
			{
				status = PCA9957_SPI_Set_Group_Ramp_Value(pca9957Driver, gradgroup, (RampRate -1));
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n Ramp Value Set Failed \r\n");
					return ERROR;
				}
				else
					PRINTF("\r\n Ramp Value Set Done \r\n");
			}
			break;
		case 6:
			lramp = 0;
			break;
		default:
			PRINTF("\r\n Invalid Input \r\n");
			break;
		}
	}while(lramp);
}

/*!@brief        Set Step Time .
 *  @details     Configure Step Time setting related to MS Cycle time and Step Time .
 *  @param[in]   Pointer to spi sensor handle structure (pca9957Driver).
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      status.
 */
uint8_t setStepTime(pca9957_spi_sensorhandle_t *pca9957Driver, GradationGroup gradgroup)
{
	uint8_t ipstep, StepTime, lsteptime = 1;
	int32_t status;

	do
	{
		PRINTF("\r\n **** Step Time Configuration **** \r\n");
		PRINTF("\r\n 1: 0.5 MS Cycle time\r\n");
		PRINTF("\r\n 2: 8 MS Cycle time\r\n");
		PRINTF("\r\n 3: Set Step Time MF\r\n");
		PRINTF("\r\n 4. Exit from Step Time Configuration \r\n");

		PRINTF("\r\n Enter your choice :- ");
		ipstep = GETCHAR();
		ipstep -= 48;
		PRINTF("%d\r\n",ipstep);
		GETCHAR();

		switch(ipstep)
		{
		case 1:
			status = PCA9957_SPI_Set_Cycle_Time(pca9957Driver, gradgroup, halfms);
			if (SENSOR_ERROR_NONE != status)
			{
				PRINTF("\r\n 0.5 ms Cycle time Set Failed\r\n");
				return ERROR;
			}
			else
				PRINTF("\r\n 0.5 ms Cycle time Set Done\r\n");
			break;
		case 2:
			status = PCA9957_SPI_Set_Cycle_Time(pca9957Driver, gradgroup, eightms);
			if (SENSOR_ERROR_NONE != status)
			{
				PRINTF("\r\n 8 ms Cycle time Set Failed\r\n");
				return ERROR;
			}
			else
				PRINTF("\r\n 8 ms Cycle time Set Done\r\n");
			break;
		case 3:
			StepTime = getGradStepTimeMF();
			if(StepTime)
			{
				status = PCA9957_SPI_Set_Group_MF(pca9957Driver, gradgroup, (StepTime - 1));
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n Multiple factor Set Failed\r\n");
					return ERROR;
				}
				else
					PRINTF("\r\n Multiple factor Set Done\r\n");
			}
			break;
		case 4:
			lsteptime = 0;
			break;
		default:
			PRINTF("\r\n Invalid Input \r\n");
			break;
		}
	}
	while(lsteptime);
}

/*!@brief        Get Gradation Group.
 *  @details     Get Gradation Group from user.
 *  @param[in]   NO
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      Gradation Group.
 */
uint8_t getGradationGroup()
{
	uint8_t gradgroup;

	PRINTF("\r\n Enter Gradation Group [1 to 6] :- ");
	gradgroup = GETCHAR();
	gradgroup -= 48;
	PRINTF("%d\r\n",gradgroup);
	GETCHAR();

	if( gradgroup >= 1 && gradgroup <= 6)
		return gradgroup ;
	else
	{
		PRINTF("\r\nGradation Group \r\n");
		return 0;
	}
}

/*!@brief        Dimm Blink Control.
 *  @details     Configure Dimm/Blink setting .
 *  @param[in]   Pointer to spi sensor handle structure (pca9957Driver).
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      Gradation Group.
 */
uint8_t dimmBlinkControl(pca9957_spi_sensorhandle_t *pca9957Driver)
{
	int32_t status, ldimmBlink = 1;
	uint8_t dimmblink, bPwm, bfrequency;

	do
	{
		PRINTF("\r\n ********  Dimming/Blinking Control ********\r\n");
		PRINTF("\r\n 1. Set Dimming/Blinking LED Brightness \r\n");
		PRINTF("\r\n 2. Set Dimming/Blinking LED Frequency \r\n");
		PRINTF("\r\n 3. Enable Dimming/Blinking \r\n");
		PRINTF("\r\n 4. Disable Dimming/Blinking \r\n");
		PRINTF("\r\n 5. Exit Dimming/Blinking Control \r\n");

		PRINTF("\r\n Enter your choice :- ");
		dimmblink = GETCHAR();
		dimmblink -= 48;
		PRINTF("%d\r\n",dimmblink);
		GETCHAR();

		switch(dimmblink)
		{
		case 1:
			bPwm = getBlinkPwm();
			if(bPwm)
			{
				status = PCA9957_SPI_Set_All_LED_Dimm_Blink_Brightness(pca9957Driver, bPwm);
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n Blinking Brightness Set Failed\r\n");
					return ERROR;
				}
				else
					PRINTF("\r\n Blinking Brightness Set Done \r\n");
			}
			break;
		case 2:
			bfrequency = getBlinkFrequncy();
			if(bfrequency)
			{
				status = PCA9957_SPI_Set_All_LED_Dimm_Blink_Frequency(pca9957Driver, bfrequency);
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n Blinking frequency Set Failed\r\n");
					return ERROR;
				}
				else
					PRINTF("\r\n Blinking frequency Set Done \r\n");
			}
			break;
		case 3:
			status = PCA9957_SPI_Set_All_LED_Dimm_Blink(pca9957Driver, groupBlinking);
			if (SENSOR_ERROR_NONE != status)
			{
				PRINTF("\r\n Blinking Enabled Failed\r\n");
				return ERROR;
			}
			else
				PRINTF("\r\n Blinking Enabled \r\n");
			break;
		case 4:
			status = PCA9957_SPI_Set_All_LED_Dimm_Blink(pca9957Driver, groupDimming);
			if (SENSOR_ERROR_NONE != status)
			{
				PRINTF("\r\n Blinking Disabled Failed\r\n");
				return ERROR;
			}
			else
				PRINTF("\r\n Blinking Disabled \r\n");
			break;

		case 5:
			ldimmBlink = 0;
			break;
		default:
			PRINTF("\r\n Invalid Input \r\n");
			break;
		}
	}
	while(ldimmBlink);
}

/*!@brief        gradation Configuration.
 *  @details     Configure gradation setting related to Ramp, Step, Hold,
 *               Ramp Current, Linear/Exponential adjustment, Gradation Group Operation etc .
 *  @param[in]   Pointer to spi sensor handle structure (pca9957Driver).
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      status.
 */
void gradationConfiguration(pca9957_spi_sensorhandle_t *pca9957Driver, GradationGroup gradgp)
{
	volatile uint8_t  loopconfg = 1;
	uint8_t ipgradconfg;
	int32_t status;

	do
	{
		PRINTF("\r\n 1. Ramp Configuration \r\n");
		PRINTF("\r\n 2. Step Time Configuration \r\n");
		PRINTF("\r\n 3. Hold Configuration \r\n");
		PRINTF("\r\n 4. Ramp Current Configuration \r\n");
		PRINTF("\r\n 5. Set Linear/Exponential adjustment \r\n");
		PRINTF("\r\n 6. Exit Gradation Configuration \r\n");

		PRINTF("\r\n Enter your choice :- ");
		ipgradconfg = GETCHAR();
		ipgradconfg -= 48;
		PRINTF("%d\r\n",ipgradconfg);
		GETCHAR();

		switch(ipgradconfg)
		{
		case 1:
			setRampRate(pca9957Driver, gradgp );
			break;
		case 2:
			setStepTime(pca9957Driver, gradgp);
			break;
		case 3:
			setHoldControl(pca9957Driver, gradgp);
			break;
		case 4:
			setFinalRampUpCurrent(pca9957Driver, gradgp);
			break;
		case 5:
			setGradAdjustment(pca9957Driver);
			break;
		case 6:
			loopconfg = 0;
			break;
		default:
			PRINTF("\r\n Invalid Input \r\n");
			break;
		}

	}
	while(loopconfg);
}

/*!@brief        Enable Gradation.
 *  @details     Enable Gradation.
 *  @param[in]   Pointer to spi sensor handle structure (pca9957Driver).
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      status.
 */
void enableGradation(pca9957_spi_sensorhandle_t *pca9957Driver)
{
	uint32_t status;
	uint8_t ipgrad, lnum;

	PRINTF("\r\n 1. Individual LED Gradation Enable \r\n");
	PRINTF("\r\n 2. All LED Gradation Enable \r\n");

	PRINTF("\r\n Enter your choice :- ");
	ipgrad = GETCHAR();
	ipgrad -= 48;
	PRINTF("%d\r\n",ipgrad);
	GETCHAR();

	switch(ipgrad)
	{
	case 1:
		lnum = getLednum();
		if(lnum)
		{
			status = PCA9957_SPI_Enable_Gradation_Mode(pca9957Driver, (lnum - 1));
			if (SENSOR_ERROR_NONE != status)
			{
				PRINTF("\r\n Gradation for LED %d Enable Failed \r\n",lnum );
			}
			else
				PRINTF("\r\n Gradation for LED %d Enabled \r\n",lnum );
		}
		break;
	case 2:
		status = PCA9957_SPI_Enable_All_LED_Gradation_Mode(pca9957Driver);
		if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n Gradation for All LED Enable Failed \r\n");
		}
		else
			PRINTF("\r\n Gradation for All LED Enabled \r\n");
		break;
	default:
		break;
	}
}

/*!@brief        Disable Gradation.
 *  @details     Disable Gradation.
 *  @param[in]   Pointer to spi sensor handle structure (pca9957Driver).
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      status.
 */
void disableGradation(pca9957_spi_sensorhandle_t *pca9957Driver)
{
	uint32_t status;
	uint8_t ipgrad, lnum;

	PRINTF("\r\n 1. Individual LED Gradation Disable \r\n");
	PRINTF("\r\n 2. All LED Gradation Disable \r\n");

	PRINTF("\r\n Enter your choice :- ");
	ipgrad = GETCHAR();
	ipgrad -= 48;
	PRINTF("%d\r\n",ipgrad);
	GETCHAR();

	switch(ipgrad)
	{
	case 1:
		lnum = getLednum();
		if(lnum)
		{
			status = PCA9957_SPI_Disable_Gradation_Mode(pca9957Driver, (lnum - 1));
			if (SENSOR_ERROR_NONE != status)
			{
				PRINTF("\r\n Gradation for LED %d Enable Failed \r\n",lnum );
			}
			else
				PRINTF("\r\n Gradation for LED %d Enabled \r\n",lnum );
		}
		break;
	case 2:
		status = PCA9957_SPI_Disable_All_LED_Gradation_Mode(pca9957Driver);
		if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n Gradation for All LED Enable Failed \r\n");
		}
		else
			PRINTF("\r\n Gradation for All LED Enabled \r\n");
		break;
	default:
		break;
	}
}

/*!@brief        Gradation Control.
 *  @details     Configure gradation Control setting related Gradation Configuration,
 *               Assign LED to Gradation Group, Enable/Disable LED channel for Gradation ,
 *               Start/Stop Gradation etc.
 *  @param[in]   Pointer to spi sensor handle structure (pca9957Driver).
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No.
 */
void gradationControl(pca9957_spi_sensorhandle_t *pca9957Driver)
{
	uint8_t gcontrol, gradgroup, lnum, gradoprsn, lgain = 1;
	int32_t status;

	do
	{
		PRINTF("\r\n ********* Gradation Main Menu *********\r\n");
		PRINTF("\r\n 1. Gradation Configuration \r\n");
		PRINTF("\r\n 2. Assign LED to Gradation Group \r\n");
		PRINTF("\r\n 3. Enable LED channel for Gradation \r\n");
		PRINTF("\r\n 4. Disable LED channel for Gradation \r\n");
		PRINTF("\r\n 5. Start Gradation \r\n");
		PRINTF("\r\n 6. Stop Gradation \r\n");
		PRINTF("\r\n 7. Exit Gradation Main Menu \r\n");

		PRINTF("\r\n Enter your choice :- ");
		gcontrol = GETCHAR();
		gcontrol -= 48;
		PRINTF("%d\r\n",gcontrol);
		GETCHAR();

		switch(gcontrol)
		{
		case 1:
			gradgroup = getGradationGroup();
			if(gradgroup)
			{
				gradationConfiguration(pca9957Driver, (gradgroup - 1));
			}
			break;
		case 2:
			gradgroup = getGradationGroup();
			lnum = getLednum();
			if( gradgroup && lnum )
			{
				status = PCA9957_SPI_Assign_LED_to_Gradation_Group(pca9957Driver, (lnum - 1), (gradgroup -1));
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n LED Assignment Failed\r\n");
				}
				else
					PRINTF("\r\n LED Assignment Done \r\n");
			}
			break;
		case 3:
			enableGradation(pca9957Driver);
			break;
		case 4:
			disableGradation(pca9957Driver);
			break;
		case 5:
			gradgroup = getGradationGroup();
			gradoprsn = getGradGroupOprsn();
			if(gradgroup && gradoprsn)
			{
				status = PCA9957_SPI_Gradation_Group_Start(pca9957Driver, (gradgroup -1), (gradoprsn - 1));
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n Gradation Start Failed\r\n");
				}
				else
					PRINTF("\r\n Gradation Start \r\n");
			}
			break;
		case 6:
			gradgroup = getGradationGroup();
			if(gradgroup)
			{
				status = PCA9957_SPI_Gradation_Group_Stop(pca9957Driver,(gradgroup -1));
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n Gradation Stop Failed\r\n");
				}
				else
					PRINTF("\r\n Gradation Stop \r\n");
			}
			break;
		case 7:
			lgain = 0;
			break;
		default:
			break;
		}
	}while(lgain);
}

/*!@brief        Get Led state .
 *  @details     Get Led state.
 *  @param[in]   None.
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      LED state.
 */
uint8_t getLedstate()
{
	uint8_t gledstate;

	PRINTF("\r\n 1. LED Off\r\n");
	PRINTF("\r\n 2. LED On\r\n");
	PRINTF("\r\n 3. Individual LED brightness Control\r\n");
	PRINTF("\r\n 4. Individual LED brightness and group dimming/blinking\r\n");

	PRINTF("\r\n Enter your choice :- ");
	gledstate = GETCHAR();
	gledstate -= 48;
	PRINTF("%d\r\n",gledstate);
	GETCHAR();

	if( gledstate >= 1 && gledstate <= 4)
		return gledstate ;
	else
	{
		PRINTF("\r\nInvalid LED State\r\n");
		return 0;
	}
}

/*!@brief        Get Brightness.
 *  @details     Get Brightness from user.
 *  @param[in]   None.
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      Brightness.
 */
uint8_t getBrightness()
{
	PRINTF("\r\n Enter Brightness from 1 to 255 :- ");
	SCANF("%d",&gbrightness);
	PRINTF("%d\r\n",gbrightness);

	if( gbrightness >= 1 && gbrightness <= 255)
		return gbrightness;
	else
	{
		PRINTF("\r\nInvalid Brightness\r\n");
		return 0;
	}
}

/*!@brief        Get output gain.
 *  @details     Get output gain from user.
 *  @param[in]   None.
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      output gain.
 */
uint8_t getOutputGain()
{
	PRINTF("\r\n Enter Output Gain from 1 to 255 :- ");
	SCANF("%d",&goutputgain);
	PRINTF("%d\r\n",goutputgain);

	if( goutputgain >= 1 && goutputgain <= 255)
		return goutputgain;
	else
	{
		PRINTF("\r\nInvalid Output Gain\r\n");
		return 0;
	}
}

/*!@brief        all Led Control.
 *  @details     Configure all Led Control setting related to  all LED on, off, brightness
 *               led state etc.
 *
 *  @param[in]   Pointer to spi sensor handle structure (pca9957Driver).
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No.
 */
void allLedControl(pca9957_spi_sensorhandle_t *pca9957Driver)
{
	int32_t status;
	uint8_t ipLedControl, brightness, ledstate, lled = 1, outputgain;

	do
	{
		PRINTF("\r\n");
		PRINTF("\r\n ******** ALL LED Control ********\r\n");
		PRINTF("\r\n 1. ALL LED On \r\n");
		PRINTF("\r\n 2. ALL LED Off \r\n");
		PRINTF("\r\n 3. Set ALL LED Brightness \r\n");
		PRINTF("\r\n 4. Set ALL LED Output Current Gain \r\n");
		PRINTF("\r\n 5. Set ALL LED State \r\n");
		PRINTF("\r\n 6. Exit From ALL LED Control \r\n");

		PRINTF("\r\n Enter your choice :- ");
		ipLedControl = GETCHAR();
		ipLedControl -= 48;
		PRINTF("%d\r\n",ipLedControl);
		GETCHAR();

		switch (ipLedControl)
		{
		case 1:
			status = PCA9957_SPI_All_LED_On(pca9957Driver);
			if (SENSOR_ERROR_NONE != status)
			{
				PRINTF("\r\n ALL LED ON Failed\r\n");
			}
			else
				PRINTF("\r\n ALL LED Switched ON\r\n");
			break;
		case 2:
			status = PCA9957_SPI_All_LED_Off(pca9957Driver);
			if (SENSOR_ERROR_NONE != status)
			{
				PRINTF("\r\n ALL LED OFF Failed\r\n");
			}
			else
				PRINTF("\r\n ALL LED Switched OFF\r\n");
			break;
		case 3:
			brightness = getBrightness();
			if(brightness)
			{
				status = PCA9957_SPI_Set_All_LED_Brightness(pca9957Driver, brightness);
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n ALL LED Brightness Set Failed\r\n");
				}
				else
					PRINTF("\r\n ALL LED Brightness Set Done \r\n");
			}
			break;
		case 4:
			 outputgain = getOutputGain();
			 if(outputgain)
			 {
				 status = PCA9957_SPI_Set_All_LED_OP_Current_Gain(pca9957Driver, outputgain);
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n ALL LED Output Gain Set Failed\r\n");
				}
				else
					PRINTF("\r\n ALL LED Output Gain Set Done \r\n");
			 }
			 break;
		case 5:
			ledstate = getLedstate();
			if(ledstate)
			{
				status = PCA9957_SPI_Set_All_LED_State(pca9957Driver, (ledstate -1));
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n ALL LED State Set Failed\r\n");
				}
				else
					PRINTF("\r\n ALL LED State Set Done\r\n");
			}
			break;
		case 6:
			lled = 0;
			break;
		default:
			break;
		}
	}
	while(lled);
}

/*!@brief        Individual  Led Control.
 *  @details     Configure Individual Led Control setting related to  Individual LED on, off, brightness
 *               led state etc.
 *
 *  @param[in]   Pointer to spi sensor handle structure (pca9957Driver).
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No.
 */
void ledControl(pca9957_spi_sensorhandle_t *pca9957Driver)
{
	int32_t status;
	uint8_t ipLedControl, brightness, ledstate, loopcontrol = 1, lednum, outputgain;

	do
	{
		PRINTF("\r\n");
		PRINTF("\r\n ******** Individual LED Control ********\r\n");
		PRINTF("\r\n 1. Individual LED On \r\n");
		PRINTF("\r\n 2. Individual LED Off \r\n");
		PRINTF("\r\n 3. Set Individual LED Brightness \r\n");
		PRINTF("\r\n 4. Set Individual LED Output Current Gain \r\n");
		PRINTF("\r\n 5. Set Individual LED State \r\n");
		PRINTF("\r\n 6. Exit From Individual LED Control \r\n");

		PRINTF("\r\n Enter your choice :- ");
		ipLedControl = GETCHAR();
		ipLedControl -= 48;
		PRINTF("%d\r\n",ipLedControl);
		GETCHAR();

		switch (ipLedControl)
		{
		case 1:
			lednum = getLednum();
			if(lednum)
			{
				status = PCA9957_SPI_LED_On(pca9957Driver, (lednum -1));
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n LED-%d On Failed\r\n",lednum);
				}
				else
					PRINTF("\r\n LED-%d On \r\n",lednum);
			}
			break;
		case 2:
			lednum = getLednum();
			if(lednum)
			{
				status = PCA9957_SPI_LED_Off(pca9957Driver, (lednum -1));
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n LED-%d Off Failed\r\n",lednum);
				}
				else
					PRINTF("\r\n LED-%d Off Done\r\n",lednum);
			}
			break;
		case 3:
			lednum = getLednum();
			brightness = getBrightness();
			if((brightness) && (lednum))
			{
				status = PCA9957_SPI_Set_LED_Brightness(pca9957Driver, (lednum - 1), brightness );
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n LED-%d Brightness Set Failed\r\n",lednum);
				}
				else
					PRINTF("\r\n LED-%d Brightness Set Done\r\n",lednum);
			}
			break;
		case 4:
			 lednum = getLednum();
			 outputgain = getOutputGain();
			 if(outputgain && lednum)
			 {
				status = PCA9957_SPI_Set_LED_OP_Current_Gain(pca9957Driver, (lednum -1), outputgain);
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n LED-%d Output Gain Set Failed\r\n",lednum);
				}
				else
					PRINTF("\r\n LED-%d Output Gain Set Done\r\n",lednum);
			 }
			 break;
		case 5:
			lednum = getLednum();
			ledstate = getLedstate();
			if((lednum) && (ledstate))
			{
				status = PCA9957_SPI_Set_LED_State(pca9957Driver, (lednum - 1), (ledstate -1));
				if (SENSOR_ERROR_NONE != status)
				{
					PRINTF("\r\n LED-%d Individual LED State Set Failed\r\n",lednum);
				}
				else
					PRINTF("\r\n LED-%d Individual LED State Set Done\r\n",lednum);
			}
			break;
		case 6:
			loopcontrol = 0;
			break;
		default:
			break;
		}
	}
	while(loopcontrol);
}

/*!@brief        Auto Switch off Control.
 *  @details     Enable/Disable auto Switch off LED channel on error.
 *
 *  @param[in]   Pointer to spi sensor handle structure (pca9957Driver).
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No.
 */
void autoSwitchoffControl(pca9957_spi_sensorhandle_t *pca9957Driver)
{
	int32_t status;
	uint8_t ipswitch;

	PRINTF("\r\n 1. Auto Switch Off Enable \r\n");
	PRINTF("\r\n 2. Auto Switch Off Disable \r\n");

	PRINTF("\r\n Enter your choice :- ");
	ipswitch = GETCHAR();
	ipswitch -= 48;
	PRINTF("%d\r\n",ipswitch);
	GETCHAR();

	if(ipswitch == 1)
	{
		status = PCA9957_SPI_Enable_Auto_SwitchOff(pca9957Driver);
		if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n Auto Switch Off Enable Failed \r\n");
		}
		else
			PRINTF("\r\n Auto Switch Off Enable \r\n");
	}
	else
	{
		status = PCA9957_SPI_Disable_Auto_SwitchOff(pca9957Driver);
		if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n Auto Switch Off Disable Failed \r\n");
		}
		else
			PRINTF("\r\n Auto Switch Off Disable \r\n");

	}
}

/*!@brief        Set maximum current.
 *  @details     set Imax to 30MA or 20 MA.
 *
 *  @param[in]   Pointer to spi sensor handle structure (pca9957Driver).
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No.
 */
void setMAxCurrent()
{
	int32_t status;
	uint8_t ipImax;

	PRINTF("\r\n 1. Set Max Current 30 MA \r\n");
	PRINTF("\r\n 2. Set Max Current 20 MA \r\n");

	PRINTF("\r\n Enter your choice :- ");
	ipImax = GETCHAR();
	ipImax -= 48;
	PRINTF("%d\r\n",ipImax);
	GETCHAR();

	if(ipImax == 1)
	{
		status = PCA9957_Set_Imax_30MA(&PCA9957_SW_EN);
		if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n 30 MA maximum Current Set Failed \r\n");
		}
		else
			PRINTF("\r\n 30 MA maximum Current Set \r\n");
	}
	else
	{
		status = PCA9957_Set_Imax_20MA(&PCA9957_SW_EN);
		if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n 20 MA maximum Current Set Failed \r\n");
		}
		else
			PRINTF("\r\n 20 MA maximum Current Set \r\n");

	}
}

/*!@brief        Set Sleep Control.
 *  @details     Enable/DIsable Sleep.
 *
 *  @param[in]   Pointer to spi sensor handle structure (pca9957Driver).
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No.
 */
void setSleepControl(pca9957_spi_sensorhandle_t *pca9957Driver)
{
	int32_t status;
	uint8_t ipSleep;

	PRINTF("\r\n 1. Enable Sleep \r\n");
	PRINTF("\r\n 2. Disable Sleep \r\n");

	PRINTF("\r\n Enter your choice :- ");
	ipSleep = GETCHAR();
	ipSleep -= 48;
	PRINTF("%d\r\n",ipSleep);
	GETCHAR();

	if(ipSleep == 1)
	{
		status = PCA9957_SPI_Enable_Sleep(pca9957Driver);
		if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n Enable Sleep Failed \r\n");
		}
		else
			PRINTF("\r\n Enabled Sleep \r\n");
	}
	else
	{
		status = PCA9957_SPI_Disable_Sleep(pca9957Driver);
		if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n Disable Sleep Failed \r\n");
		}
		else
			PRINTF("\r\n Disable Sleep\r\n");

	}
}

/*!@brief        LED channel error.
 *  @details     check LED channel error.
 *
 *  @param[in]   Pointer to spi sensor handle structure (pca9957Driver).
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No.
 */
void errorControl(pca9957_spi_sensorhandle_t *pca9957Driver)
{
	int32_t status;
	uint8_t ipError, ledn, i;
	ErrorState errorstate;
	LedErrorType lederror;

	PRINTF("\r\n 1. Check Error \r\n");
	PRINTF("\r\n 2. Clear Error \r\n");
	PRINTF("\r\n 3. Check Individual LED Error \r\n");

	PRINTF("\r\n Enter your choice :- ");
	ipError = GETCHAR();
	ipError -= 48;
	PRINTF("%d\r\n",ipError);
	GETCHAR();

	switch(ipError)
	{
	case 1:
		status = PCA9957_SPI_Check_Error(pca9957Driver, &errorstate);
		if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n Check Error Failed \r\n");
		}
		else
		{
			if(errorstate == errorDetected)
			{
				for( i =0; i <= PCA9957_MAX_LED_PORT; i++)
				{
					status = PCA9957_SPI_LED_Get_Error_Type(pca9957Driver, i, &lederror);
					if (SENSOR_ERROR_NONE != status)
					{
						PRINTF("\r\n Check Error Failed for Led :- %d\r\n", (i+1));
					}
					else
					{
						if(lederror == shortCircuit)
							PRINTF("\r\n\033[31m Short Circuit Occurred for Led :- %d \033[37m \r\n", (i+1));
						else if(lederror == openCircuit)
							PRINTF("\r\n\033[31m Open Circuit Occurred for Led :- %d \033[37m \r\n", (i +1));
					}
				}
			}
			else
				PRINTF("\r\n Error Not Occurred \r\n");
		}
		break;
	case 2:
		status = PCA9957_SPI_Clear_Error(pca9957Driver);
		if (SENSOR_ERROR_NONE != status)
		{
			PRINTF("\r\n Clear Error Failed \r\n");
		}
		else
			PRINTF("\r\n Error Cleared \r\n");
		break;
	case 3:
		ledn = getLednum();
		if(ledn)
		{
			status = PCA9957_SPI_LED_Get_Error_Type(pca9957Driver, (ledn -1), &lederror);
			if (SENSOR_ERROR_NONE != status)
			{
				PRINTF("\r\n Check Led :- %d Error Failed \r\n", ledn);
			}
			else
			{
				if(lederror == noError)
					PRINTF("\r\n No Error Occurred \r\n");
				else if(lederror == shortCircuit)
					PRINTF("\r\n\033[31m Short Circuit Occurred \033[37m\r\n");
				else if(lederror == openCircuit)
					PRINTF("\r\n\033[31m Open Circuit Occurred \033[37m\r\n");
				else
					PRINTF("\r\n Check Led :- %d Error Failed \r\n", ledn);
			}
		}
		break;
	}
}

/*!@brief        check over temperature check.
 *  @details     check over temperature check.
 *
 *  @param[in]   Pointer to spi sensor handle structure (pca9957Driver).
 *  @constraints None
 *
 *  @reentrant   No
 *  @return      No.
 */
void overTempCheck(pca9957_spi_sensorhandle_t *pca9957Driver)
{
	int32_t status;
	TempCondition tempCondsn;

	status = PCA9957_SPI_Check_OvTemp(pca9957Driver, &tempCondsn);
	if (SENSOR_ERROR_NONE != status)
	{
		PRINTF("\r\n Check Over Temperature Failed \r\n");
	}
	else
	{
		if(tempCondsn == underTemp)
			PRINTF("\r\n Operating in Under Temperature \r\n");
		else
			PRINTF("\r\n Operating in Normal Temperature \r\n");
	}
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the The main function implementation.
 *  @details     This function invokes board initializes routines, then then brings up the sensor and
 *               finally enters an endless loop to continuously read available samples.
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints None
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
int main(void)
{
	int32_t status, i;
	uint8_t character;
	ErrorState errorstate;

    ARM_DRIVER_SPI *pSPIdriver = &SPI_S_DRIVER;
	GENERIC_DRIVER_GPIO *pGPIODriver = &Driver_GPIO_KSDK;

	pca9957_spi_sensorhandle_t  pca9957Driver;

	/* Enable EDMA for SPI */
#if (RTE_SPI1_DMA_EN)
	/* Enable DMA clock. */
	CLOCK_EnableClock(EXAMPLE_DMA_CLOCK);
	edma_config_t edmaConfig = {0};
	EDMA_GetDefaultConfig(&edmaConfig);
	EDMA_Init(EXAMPLE_DMA_BASEADDR, &edmaConfig);
#endif

	/*! Initialize the MCU hardware. */
	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_SystickEnable();
	BOARD_InitDebugConsole();

	/* Initialize OE Pin and set its default state to high */ 
	PCA9957_OE_Pin_Init(&PCA9957_OE);
	/* By default enable output */
	PCA9957_Output_Enable(&PCA9957_OE);

	/* Initialize Reset Pin and set its default state to high */ 
	PCA9957_Reset_Pin_Init(&PCA9957_RESET);

	/* Initialize SW Enable Pin and set its default state to low */
	PCA9957_SW_EN_Pin_Init(&PCA9957_SW_EN);

	PRINTF("\r\n ISSDK PCA9957 LED driver example demonstration for SPI\r\n");

	/*! Initialize the SPI driver. */
	status = pSPIdriver->Initialize(SPI_S_SIGNAL_EVENT);
	if (ARM_DRIVER_OK != status)
	{
		PRINTF("\r\n SPI Initialization Failed\r\n");
		return -1;
	}

	/*! Set the SPI Power mode. */
	status = pSPIdriver->PowerControl(ARM_POWER_FULL);
	if (ARM_DRIVER_OK != status)
	{
		PRINTF("\r\n SPI Power Mode setting Failed\r\n");
		return -1;
	}

	/*! Set the SPI Slave speed. */
	status = pSPIdriver->Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL0_CPHA0, SPI_S_BAUDRATE);
	if (ARM_DRIVER_OK != status)
	{
		PRINTF("\r\n SPI Control Mode setting Failed\r\n");
		return -1;
	}

	/* Initialize the PCA9957 driver. */
	status = PCA9957_SPI_Initialize(&pca9957Driver, &SPI_S_DRIVER, SPI_S_DEVICE_INDEX, &PCA9957_CS);
	if (SENSOR_ERROR_NONE != status)
	{
		PRINTF("\r\n PCA9957 Sensor Initialization Failed\r\n");
		return -1;
	}

	/* Set all LED off */
	PCA9957_SPI_All_LED_Off(&pca9957Driver);

	PRINTF("\r\n Successfully Applied PCA9957 Configuration\r\n");
	PRINTF("\r\n \033[32m It is recommended to use external supply (3V - 3.3V) on J1 \033[37m \r\n");
     PRINTF("\r\n \033[32m To use external supply Please connect pin number 2 and 3 of J2 \033[37m \r\n");
	do
	{
		PRINTF("\r\n *********** Main Menu ***************\r\n");

		PRINTF("\r\n 1.  All LED Control \r\n");
		PRINTF("\r\n 2.  Individual LED Control \r\n");
		PRINTF("\r\n 3.  Dimming/Blinking Control \r\n");
		PRINTF("\r\n 4.  Gradation Group Control \r\n");
		PRINTF("\r\n 5.  Reset \r\n");
		PRINTF("\r\n 6.  LED Output Delay (No of Cycle) \r\n");
		PRINTF("\r\n 7.  Sleep Control \r\n");
		PRINTF("\r\n 8.  Auto Switch off on Error Control \r\n");
		PRINTF("\r\n 9.  Maximum Current Control \r\n");
		PRINTF("\r\n 10. LED Error \r\n");
		PRINTF("\r\n 11. Over Temperature Control \r\n");

		PRINTF("\r\n Enter your choice :- ");
		SCANF("%d",&character);
		PRINTF("%d\r\n",character);

		switch (character)
		{
		case 1: /* All LED control */
			allLedControl(&pca9957Driver);
			break;
		case 2: /* Individual LED control */
			ledControl(&pca9957Driver);
			break;
		case 3:  /* Dim Blink Control */
			dimmBlinkControl(&pca9957Driver);
			break;
		case 4: /* Gradation Control */
			gradationControl(&pca9957Driver);
			break;
		case 5: /* Reset PCA9957 device */
			status = PCA9957_Reset(&PCA9957_RESET);
			if (SENSOR_ERROR_NONE != status)
			{
				PRINTF("\r\n Reset Failed \r\n");
			}
			else
				PRINTF("\r\n Reset Done \r\n");
			break;
		case 6: /* Set Output delay cycle per LED channel  */
			setLedOpDelay(&pca9957Driver);
			break;
		case 7: /* Sleep Control */
			setSleepControl(&pca9957Driver);
			break;
		case 8: /* Auto switch off control */
			autoSwitchoffControl(&pca9957Driver);
			break;
		case 9: /* Set maximum current */
			setMAxCurrent();
			break;
		case 10: /* error control */
			errorControl(&pca9957Driver);
			break;
		case 11:  /* check over temperature */
			overTempCheck(&pca9957Driver);
			break;
		default:
			PRINTF("\r\n Invalid option...chose correct one from Main Menu\r\n");
			break;
		}

		if(character != 10)
		{
			status = PCA9957_SPI_Check_Error(&pca9957Driver, &errorstate);
			if (SENSOR_ERROR_NONE != status)
			{
				PRINTF("\r\n Check Error Failed \r\n");
			}
			else
			{
				if(errorstate == errorDetected)
				{
					PRINTF("\r\n \033[31m Error Occurred on one or more LEDs...For more details goto \"LED Error \" option in main menu \033[37m \r\n");
					PRINTF("\r\n Press Enter for main menu \r\n");
					GETCHAR();
				}
			}
		}
	}
	while(1);

	return 0;
}
