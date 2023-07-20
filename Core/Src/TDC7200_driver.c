/*
 * TDC7200_driver.c
 *
 *  Created on: Jul 4, 2023
 *      Author: Halha
 */


#include "TDC7200_driver.h"
#include <stdint.h>

extern SPI_HandleTypeDef hspi1; //declared in OG SPI source/header file generated by CubeMX IDE

uint8_t TDC7200_WriteRegister(uint8_t reg, uint8_t* dataToWrite)
{
	//select this slave device, CS = 0
	HAL_GPIO_WritePin(TDC7200_CS_GPIO_Port, TDC7200_CS_Pin, GPIO_PIN_RESET);

	uint8_t combinedJuicer[2];
	combinedJuicer[0] = reg;
	combinedJuicer[1] = *dataToWrite;

	/*	Tx 2 bytes to TDC:
			Byte 1: Contains the read/write/auto-increment command bits and desired register to access
			Byte 2: Contains the data in which we want to write into desired register
	*/
	//Tx data via SPI API, if SPI txn fails (ie: != HAL_OK), return -1
	if((HAL_SPI_Transmit(&hspi1, combinedJuicer, 2, HAL_MAX_DELAY)) != HAL_OK)
		return -1;

	//de-select this slave device, set CS line HIGH/1
	HAL_GPIO_WritePin(TDC7200_CS_GPIO_Port, TDC7200_CS_Pin, GPIO_PIN_SET);

	return 0;
}



double TDC7200_Read_N_Registers(uint8_t regToRead, uint8_t n)
{
	//Note: the TDC chip reads from 1 register (1byte) or 3 registers if AutoIncr bit cmd is used (ie: 3bytes)
	//User chooses whether to send n=1 or n=3 into this function depending on what register(s) they want to read from


	uint8_t	rxSpiData[3]; //this array will hold either 1 or 3 bytes of data sent from TDC register(s)
	uint32_t processedData = 0;
	double finalResult = 0;
	uint8_t regAndOpcode = 0;


	if (n == 3) //ie: if reading more than 1 byte..(ex: 3), enable auto_incr cmd bit and read cmd bit
		regAndOpcode = regToRead | TDC_READ_CMD | TDC_AUTO_INCR;

	else //if reading 1 byte, simply attach read cmd bit, no auto incr
		regAndOpcode = regToRead | TDC_READ_CMD;

	//select this slave device, CS = 0
	HAL_GPIO_WritePin(TDC7200_CS_GPIO_Port, TDC7200_CS_Pin, GPIO_PIN_RESET);

	//Tx data via SPI API, if SPI txn fails (ie: != HAL_OK), return -1
	if((HAL_SPI_Transmit(&hspi1, &regAndOpcode, 1, HAL_MAX_DELAY)) != HAL_OK)
		return -1;

	//Rx data via SPI API, if SPI rxn fails (ie: != HAL_OK), return -1
	if((HAL_SPI_Receive(&hspi1, rxSpiData, n, 2000)) != HAL_OK)
		return -1;


	if(n == 1)
		processedData = rxSpiData[0];
	if(n==3)
		processedData = (rxSpiData[0] << 16) | (rxSpiData[1] << 8) | (rxSpiData[0] << 0);


	finalResult = processedData;

	HAL_GPIO_WritePin(TDC7200_CS_GPIO_Port, TDC7200_CS_Pin, GPIO_PIN_SET); //release SPI CS line, un-select this slave device
	return finalResult;
}

double TDC7200_ReadBytes(uint8_t reg, uint8_t txData);
