/*
 * TDC7200_driver.c
 *
 *  Created on: Jul 4, 2023
 *      Author: Halha
 */


#include "TDC7200_driver.h"
#include <stdint.h>



void TDC7200_WriteRegister(uint8_t reg, uint8_t configRegData)
{
	HAL_GPIO_WritePin(GPIO_TDC_PORT, GPIO_TDC_PIN, 0);
	HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIO_TDC_PORT, GPIO_TDC_PIN, 1);
}



void TDC7200_ReadRegister(uint8_t reg, uint8_t rxData);

double TDC7200_WriteRegister(uint8_t reg, uint8_t txData);
