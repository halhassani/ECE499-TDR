/*
 * TDC7200_driver.c
 *
 *  Created on: Jul 4, 2023
 *      Author: Halha
 */


#include "TDC7200_driver.h"
#include <stdint.h>

extern SPI_HandleTypeDef hspi1; //declared in OG SPI source/header file generated by CubeMX IDE

void TDC7200_WriteRegister(uint8_t reg, uint8_t dataToWrite)
{
	//select this slave device, CS = 0
	HAL_GPIO_WritePin(TDC7200_GPIO_Port, TDC7200_Pin, GPIO_PIN_RESET);

	uint8_t combinedJuicer[2];
	combinedJuicer[0] = reg;
	combinedJuicer[1] = dataToWrite;

	//Tx 1 byte to TDC, includes R/W cmd and desired register
	HAL_SPI_Transmit(&hspi1, combinedJuicer, 2, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(TDC7200_GPIO_Port, TDC7200_Pin, GPIO_PIN_SET);
}



void TDC7200_ReadRegister(uint8_t reg, uint8_t rxData);

double TDC7200_ReadBytes(uint8_t reg, uint8_t txData);
