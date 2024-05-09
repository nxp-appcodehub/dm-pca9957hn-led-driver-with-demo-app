/*
 * Copyright 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! File: frdm_stbi_pca9957_shield.h
* @brief The frdm_stbi_pca9957_shield.h file declares arduino pin mapping for frdm_stbi_pca9957_shield expansion board.
*/

#ifndef _FRDM_STBI_PCA9957_SHIELD_H_
#define _FRDM_STBI_PCA9957_SHIELD_H_

/* The shield name */
#define SHIELD_NAME "FRDM-STBI-PCA9957"

/* Enable PCA9957 SPI Read */
#define PCA9957_ARD    1

#define PCA9957_CS        D10
#define PCA9957_MOSI      D11
#define PCA9957_MISO      D12
#define PCA9957_SCLK      D13

#define PCA9957_OE        D9
#define PCA9957_RESET     D8
#define PCA9957_SW_EN     D2

#endif /* _FRDM_STBI_PCA9957_SHIELD_H_ */
