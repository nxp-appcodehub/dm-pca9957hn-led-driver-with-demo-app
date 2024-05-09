/*
 * Copyright 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file pca9957.h
 * @brief This file contains the PCA9957 LED driver register definitions, access macros, and
 * device access functions.
 */
#ifndef PCA9957_H_
#define PCA9957_H_
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief The PCA9957 types
 */

/**
 **
 **  @brief The PCA9957 LED driver Internal Register Map.
 */
enum
{
	PCA9957_MODE1 = 0x00,
	PCA9957_MODE2 = 0x01,
	PCA9957_EFLAG0 = 0x02,
   	PCA9957_EFLAG1 = 0x03,
	PCA9957_EFLAG2 = 0x04,
	PCA9957_EFLAG3 = 0x05,
	PCA9957_EFLAG4 = 0x06,
	PCA9957_EFLAG5 = 0x07,
	PCA9957_LEDOUT0 = 0x08,
	PCA9957_LEDOUT1 = 0x09,
	PCA9957_LEDOUT2 = 0x0A,
	PCA9957_LEDOUT3 = 0x0B,
	PCA9957_LEDOUT4 = 0x0C,
	PCA9957_LEDOUT5 = 0x0D,
	PCA9957_GRPPWM = 0x0E,
	PCA9957_GRPFREQ = 0x0F,
	PCA9957_PWM0 = 0x10,
	PCA9957_PWM1 = 0x11,
	PCA9957_PWM2 = 0x12,
	PCA9957_PWM3 = 0x13,
	PCA9957_PWM4 = 0x14,
	PCA9957_PWM5 = 0x15,
	PCA9957_PWM6 = 0x16,
	PCA9957_PWM7 = 0x17,
	PCA9957_PWM8 = 0x18,
	PCA9957_PWM9 = 0x19,
	PCA9957_PWM10 = 0x1A,
	PCA9957_PWM11 = 0x1B,
	PCA9957_PWM12 = 0x1C,
	PCA9957_PWM13 = 0x1D,
	PCA9957_PWM14 = 0x1E,
	PCA9957_PWM15 = 0x1F,
	PCA9957_PWM16 = 0x20,
	PCA9957_PWM17 = 0x21,
	PCA9957_PWM18 = 0x22,
	PCA9957_PWM19 = 0x23,
	PCA9957_PWM20 = 0x24,
	PCA9957_PWM21 = 0x25,
	PCA9957_PWM22 = 0x26,
	PCA9957_PWM23 = 0x27,
	PCA9957_IREF0 = 0x28,
	PCA9957_IREF1 = 0x29,
	PCA9957_IREF2 = 0x2A,
	PCA9957_IREF3 = 0x2B,
	PCA9957_IREF4 = 0x2C,
	PCA9957_IREF5 = 0x2D,
	PCA9957_IREF6 = 0x2E,
	PCA9957_IREF7 = 0x2F,
	PCA9957_IREF8 = 0x30,
	PCA9957_IREF9 = 0x31,
	PCA9957_IREF10 = 0x32,
	PCA9957_IREF11 = 0x33,
	PCA9957_IREF12 = 0x34,
	PCA9957_IREF13 = 0x35,
	PCA9957_IREF14 = 0x36,
	PCA9957_IREF15 = 0x37,
	PCA9957_IREF16 = 0x38,
	PCA9957_IREF17 = 0x39,
	PCA9957_IREF18 = 0x3A,
	PCA9957_IREF19 = 0x3B,
	PCA9957_IREF20 = 0x3C,
	PCA9957_IREF21 = 0x3D,
	PCA9957_IREF22 = 0x3E,
	PCA9957_IREF23 = 0x3F,
	PCA9957_RAMP_RATE_GRP0 = 0x40,
	PCA9957_STEP_TIME_GRP0 = 0x41,
	PCA9957_HOLD_CNTL_GRP0 = 0x42,
	PCA9957_IREF_GRP0 = 0x43,
	PCA9957_RAMP_RATE_GRP1 = 0x44,
	PCA9957_STEP_TIME_GRP1 = 0x45,
	PCA9957_HOLD_CNTL_GRP1 = 0x46,
	PCA9957_IREF_GRP1 = 0x47,
	PCA9957_RAMP_RATE_GRP2 = 0x48,
	PCA9957_STEP_TIME_GRP2 = 0x49,
	PCA9957_HOLD_CNTL_GRP2 = 0x4A,
	PCA9957_IREF_GRP2 = 0x4B,
	PCA9957_RAMP_RATE_GRP3 = 0x4C,
	PCA9957_STEP_TIME_GRP3 = 0x4D,
	PCA9957_HOLD_CNTL_GRP3 = 0x4E,
	PCA9957_IREF_GRP3 = 0x4F,
	PCA9957_RAMP_RATE_GRP4 = 0x50,
	PCA9957_STEP_TIME_GRP4 = 0x51,
	PCA9957_HOLD_CNTL_GRP4 = 0x52,
	PCA9957_IREF_GRP4 = 0x53,
	PCA9957_RAMP_RATE_GRP5 = 0x54,
	PCA9957_STEP_TIME_GRP5 = 0x55,
	PCA9957_HOLD_CNTL_GRP5 = 0x56,
	PCA9957_IREF_GRP5 = 0x57,
	PCA9957_GRAD_MODE_SEL0 = 0x58,
	PCA9957_GRAD_MODE_SEL1 = 0x59,
	PCA9957_GRAD_MODE_SEL2 = 0x5A,
	PCA9957_IREF_GRAD_GRP_SEL0 = 0x5B,
	PCA9957_IREF_GRAD_GRP_SEL1 = 0x5C,
	PCA9957_IREF_GRAD_GRP_SEL2 = 0x5D,
	PCA9957_IREF_GRAD_GRP_SEL3 = 0x5E,
	PCA9957_IREF_GRAD_GRP_SEL4 = 0x5F,
	PCA9957_IREF_GRAD_GRP_SEL5 = 0x60,
	PCA9957_IREF_GRAD_GRP_SEL6 = 0x61,
	PCA9957_IREF_GRAD_GRP_SEL7 = 0x62,
	PCA9957_IREF_GRAD_GRP_SEL8 = 0x63,
	PCA9957_IREF_GRAD_GRP_SEL9 = 0x64,
	PCA9957_IREF_GRAD_GRP_SEL10 = 0x65,
	PCA9957_IREF_GRAD_GRP_SEL11 = 0x66,
	PCA9957_CNTL0 = 0x67,
	PCA9957_CNTL1 = 0x68,
	PCA9957_OFFSET = 0x69,
	PCA9957_PWMALL = 0x6A,
	PCA9957_IREFALL = 0x6B,
};

/*--------------------------------
 ** Register: MODE1
 ** Enum: PCA9957_MODE1
 ** --
 ** Offset : 0x00 Mode register 1.
 ** ------------------------------*/
typedef union
{
	struct
	{
		uint8_t _reserved_0 : 1;     /*  Reserved Bit (Unused)  */

		uint8_t _reserved_1 : 1;     /*  Reserved Bit (Unused)  */

		uint8_t _reserved_2 : 1;     /*  Reserved Bit (Unused)  */

		uint8_t _reserved_3 : 1;     /*  Reserved Bit (Unused)  */

		uint8_t sleep : 1;           /*  Low-powe/Normal mode.  */

		uint8_t _reserved_4 : 1;     /*  Reserved Bit (Unused)  */

		uint8_t _reserved_5 : 1;     /*  Reserved Bit (Unused)  */

		uint8_t _reserved_6 : 1;     /*  Reserved Bit (Unused)  */
	} b;
	uint8_t w;
} PCA9957_Mode1;

/*--------------------------------
 ** Register: MODE2
 ** Enum: PCA9957_MODE2
 ** --
 ** Offset : 0x01 Mode register 2.
 ** ------------------------------*/
typedef union
{
	struct
	{
		uint8_t _reserved_0 : 1;     /*  Reserved Bit (Unused)  */

		uint8_t _reserved_1 : 1;     /*  Reserved Bit (Unused)  */

		uint8_t exp_en : 1;          /*  linear/exponential adjustment for gradation control */

		uint8_t auro_switchoff : 1;  /*  auto switch off disable */

		uint8_t clrerr : 1;          /*  clear all error status bits  */

		uint8_t dmblink : 1;         /*  dimming/blinking control */

		uint8_t error : 1;           /*  open or short-circuit detection */

		uint8_t overtemp : 1;        /* over temperature condition. */
	} b;
	uint8_t w;
} PCA9957_MODE_2;


/*
 * MODE2 - Bit field mask definitions
 */

#define PCA9957_OVERTEMP_SHIFT                   ((uint8_t)0x07)
#define PCA9957_OVERTEMP_MASK                    ((uint8_t)0x80)

#define PCA9957_ERROR_SHIFT                      ((uint8_t)0x06)
#define PCA9957_ERROR_MASK                       ((uint8_t)0x40)

#define PCA9957_DMBLNK_SHIFT                     ((uint8_t)0x05)
#define PCA9957_DMBLNK_MASK                      ((uint8_t)0x20)

#define PCA9957_CLRERR_SHIFT                     ((uint8_t)0x04)
#define PCA9957_CLRERR_MASK                      ((uint8_t)0x10)

#define PCA9957_AUTO_SWITCHOFF_DIS_SHIFT         ((uint8_t)0x03)
#define PCA9957_AUTO_SWITCHOFF_DIS_MASK          ((uint8_t)0x08)

#define PCA9957_EXP_EN_SHIFT                     ((uint8_t)0x02)
#define PCA9957_EXP_EN_MASK                      ((uint8_t)0x04)


/*
 * OFFSET - Bit field mask definitions
 */
#define PCA9957_CLOCK_DELAY_SHIFT                ((uint8_t)0x00)
#define PCA9957_CLOCK_DELAY_MASK                 ((uint8_t)0x0F)


/*
 * GRPFREQ - Bit field mask definitions
 */
#define PCA9957_GRPFREQ_SHIFT                     ((uint8_t)0x00)
#define PCA9957_GRPFREQ_MASK                      ((uint8_t)0xFF)

/*
 * GRPPWM - Bit field mask definitions
 */
#define PCA9957_GRPPWM_SHIFT                     ((uint8_t)0x00)
#define PCA9957_GRPPWM_MASK                      ((uint8_t)0xFF)

/*
 * RAMP_RATE_GRP0-5 - Bit field mask definitions
 */
#define PCA9957_RAMP_UP_DISABLE_SHIFT            ((uint8_t)0x07)
#define PCA9957_RAMP_UP_DISABLE_MASK             ((uint8_t)0x80)

#define PCA9957_RAMP_UP_ENABLE_SHIFT             ((uint8_t)0x07)
#define PCA9957_RAMP_UP_ENABLE_MASK              ((uint8_t)0x80)

#define PCA9957_RAMP_DOWN_DISABLE_SHIFT          ((uint8_t)0x06)
#define PCA9957_RAMP_DOWN_DISABLE_MASK           ((uint8_t)0x40)

#define PCA9957_RAMP_DOWN_ENABLE_SHIFT           ((uint8_t)0x06)
#define PCA9957_RAMP_DOWN_ENABLE_MASK            ((uint8_t)0x40)

#define PCA9957_RAMP_RATE_VALUE_SHIFT            ((uint8_t)0x00)
#define PCA9957_RAMP_RATE_VALUE_MASK             ((uint8_t)0x3F)

/*
 * STEP_TIME_GRP0-5 - Bit field mask definitions
 */

#define PCA9957_STEP_CYCLE_TIME_SHIFT            ((uint8_t)0x06)
#define PCA9957_STEP_CYCLE_TIME_MASK             ((uint8_t)0x40)

#define PCA9957_STEP_TIME_MF_SHIFT               ((uint8_t)0x00)
#define PCA9957_STEP_TIME_MF_MASK                ((uint8_t)0x3F)


/*
 * HOLD_CNTL_GRP0-5 - Bit field mask definitions
 */

#define PCA9957_HOLD_ON_CNTL_SHIFT               ((uint8_t)0x07)
#define PCA9957_HOLD_ON_CNTL_MASK                ((uint8_t)0x80)

#define PCA9957_HOLD_OFF_CNTL_SHIFT              ((uint8_t)0x06)
#define PCA9957_HOLD_OFF_CNTL_MASK               ((uint8_t)0x40)

#define PCA9957_HOLD_ON_TIME_SHIFT               ((uint8_t)0x03)
#define PCA9957_HOLD_ON_TIME_MASK                ((uint8_t)0x38)

#define PCA9957_HOLD_OFF_TIME_SHIFT              ((uint8_t)0x00)
#define PCA9957_HOLD_OFF_TIME_MASK               ((uint8_t)0x07)


/*
 * REF_GRP00-5 - Bit field mask definitions
 */
#define PCA9957_CURRENT_GAIN_SHIFT               ((uint8_t)0x00)
#define PCA9957_CURRENT_GAIN_MASK                ((uint8_t)0xFF)

/*
 * GRAD_GRP0-11 - Bit field mask definitions
 */
#define PCA9957_GRAD_GRP_FIRST_LED_SHIFT         ((uint8_t)0x00)
#define PCA9957_GRAD_GRP_FIRST_LED_MASK          ((uint8_t)0x07)

#define PCA9957_GRAD_GRP_SECOND_LED_SHIFT        ((uint8_t)0x04)
#define PCA9957_GRAD_GRP_SECOND_LED_MASK         ((uint8_t)0x70)

/*
 * IREF0-23 - Bit field mask definitions
 */

#define PCA9957_LED_OC_OFF                       ((uint8_t)0x00)
#define PCA9957_LED_DFT_OC                       ((uint8_t)0x80)  
#define PCA9957_LED_OC_ON                        ((uint8_t)0xFF)
#define PCA9957_LED_OC_MASK                      ((uint8_t)0xFF)

/*
 * PWMALL - Bit field mask definitions
 */
#define PCA9957_ALL_LED_MAX_PWM                  ((uint8_t)0xFF)
#define PCA9957_ALL_LED_DFT_PWM                  ((uint8_t)0x80)
#define PCA9957_ALL_LED_LOW_PWM                  ((uint8_t)0x00)
#define PCA9957_ALL_LED_PWM_MASK                 ((uint8_t)0xFF)

/*
 * IREFALL - Bit field mask definitions
 */

#define PCA9957_ALL_LED_OC_OFF                   ((uint8_t)0x00)
#define PCA9957_ALL_LED_DFT_OC                   ((uint8_t)0x80)       
#define PCA9957_ALL_LED_OC_ON                    ((uint8_t)0xFF)
#define PCA9957_ALL_LED_OC_MASK                  ((uint8_t)0xFF)

/*
 * MODE1 - Bit field mask definitions
 *
 */
#define PCA9957_SLEEP_SHIFT                      ((uint8_t)0x04)
#define PCA9957_SLEEP_MASK                       ((uint8_t)0x10)

#endif /* PCA9957_H_ */
