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
	combinedJuicer[0] = reg | TDC_WRITE_CMD;
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



uint32_t TDC7200_Read_N_Registers(uint8_t regToRead, uint8_t n)
{
	//Note: the TDC chip reads from 1 register (1byte) or 3 registers if AutoIncr bit cmd is used (ie: 3bytes)
	//User chooses whether to send n=1 or n=3 into this function depending on what register(s) they want to read from


	uint8_t	rxSpiData[3]; //this array will hold either 1 or 3 bytes of data sent from TDC register(s)
	uint32_t processedData = 0;
	uint32_t finalResult = 0;
	uint8_t regAndOpcode = 0;


	if (n == 3) //ie: if reading more than 1 byte..(ex: 3), enable auto_incr cmd bit and read cmd bit
		regAndOpcode= regToRead | TDC_READ_CMD | TDC_AUTO_INCR;

	else //if reading 1 byte, simply attach read cmd bit, no auto incr
		regAndOpcode = regToRead | TDC_READ_CMD;

	//select this slave device, CS = 0
	HAL_GPIO_WritePin(TDC7200_CS_GPIO_Port, TDC7200_CS_Pin, GPIO_PIN_RESET);

	//Tx data via SPI API, if SPI txn fails (ie: != HAL_OK), return -1
	HAL_SPI_Transmit(&hspi1, &regAndOpcode, 1, HAL_MAX_DELAY);


	//Rx data via SPI API, if SPI rxn fails (ie: != HAL_OK), return -1
	HAL_SPI_Receive(&hspi1, rxSpiData, n, HAL_MAX_DELAY);




	if(n == 1)
		processedData = rxSpiData[0];
	if(n==3)
		processedData = (rxSpiData[0] << 16) | (rxSpiData[1] << 8) | (rxSpiData[2] << 0);


	finalResult = processedData;

	HAL_GPIO_WritePin(TDC7200_CS_GPIO_Port, TDC7200_CS_Pin, GPIO_PIN_SET); //release SPI CS line, un-select this slave device
	return finalResult;
}

uint8_t myTDC_ReadInterruptRegister(void)
{
	uint8_t INT_STATUS_REGISTER = 0;
	INT_STATUS_REGISTER = TDC_READ_CMD | TDC_INT_STATUS;
	uint8_t retVal = 0;

	//select this slave device, CS = 0
	HAL_GPIO_WritePin(TDC7200_CS_GPIO_Port, TDC7200_CS_Pin, GPIO_PIN_RESET);

	//Tx data via SPI API, if SPI txn fails (ie: != HAL_OK), return -1
	HAL_SPI_Transmit(&hspi1, &INT_STATUS_REGISTER, 1, HAL_MAX_DELAY);


	//Rx data via SPI API, if SPI rxn fails (ie: != HAL_OK), return -1
	HAL_SPI_Receive(&hspi1, &retVal, 1, HAL_MAX_DELAY);

	//de-select this slave device, CS = 1
	HAL_GPIO_WritePin(TDC7200_CS_GPIO_Port, TDC7200_CS_Pin, GPIO_PIN_SET);

	return retVal;
}



//This fxn toggles TDC_EN pin from 0 to 1 to ensure TDC powers up properly
//(TDC must see 1 low-to-high edge on EN pin)
void myTDC_EnablePowerOn(void)
{
	HAL_GPIO_WritePin(TDC7200_EN_GPIO_Port, TDC7200_EN_Pin, 0);
	HAL_Delay(500);
	HAL_GPIO_WritePin(TDC7200_EN_GPIO_Port, TDC7200_EN_Pin, 1);
	HAL_Delay(500); //Delay cuz TDC requires some time until its internal Vregulator becomes stable
					//(TDC7200 datasheet pg.20: 8.4.7 Wait Times for TDC7200 Startup)
}


//THIS FXN INITIALIZES TDC REGISTERS TO SETTINGS THAT WE WANT, NO CAP
void myTDC_Init(void)
{
	uint8_t regConfigurations = 0;

	/***************************** TDC CONFIG_1 REG ******************************/
	regConfigurations = MEASURE_MODE_1 | START_EDGE_RISING | STOP_EDGE_RISING
			|	TRIGG_EDGE_RISING | PARITY_DISABLED | FORCE_CALIBRATION_ON; //ie: setting Config_1 reg to 0x80
	TDC7200_WriteRegister(TDC_CONFIG1, &regConfigurations);
	regConfigurations = 0; //reset variable to get ready for new register configs
	/****************************************************************************/
	/****************************************************************************/


	/***************************** TDC CONFIG_2 REG ******************************/
	regConfigurations = NUM_STOP_SINGLE | AVG_CYCLES_1 | CALIBRATION2_PERIOD_2;
	TDC7200_WriteRegister(TDC_CONFIG2, &regConfigurations); //ie: setting Config_2 reg to 0x00
	regConfigurations = 0;
	/****************************************************************************/
	/****************************************************************************/


	/***************************** TDC INT_MASK REG ******************************/
	regConfigurations = CLOCK_CNTR_OVF_MASK_DISABLED | COARSE_CNTR_OVF_MASK_ENABLED
			| NEW_MEAS_MASK_ENABLED; 	//ie: disable CLOCK OVF INT flag since this used only
																//in Measurement Mode 2 (and we using Mode 1)
			//ie: setting INT_MASK reg to 0x03

	TDC7200_WriteRegister(TDC_INT_MASK, &regConfigurations);
	regConfigurations = 0;
	/****************************************************************************/
	/****************************************************************************/
}

//an array of addresses to make life easier when using the myTDC_CalculateTime fxn shown below
uint8_t TDC_Time_Registers[6] =
{
		TDC_TIME + TDC_TIME1,
		TDC_TIME + TDC_TIME2,
		TDC_TIME + TDC_TIME3,
		TDC_TIME + TDC_TIME4,
		TDC_TIME + TDC_TIME5,
		TDC_TIME + TDC_TIME6,
};

void myTDC_CalculateTime(double* pData)
{
	*pData = 0;
	for(uint8_t idx = 0; idx < 6; idx++)
	{
		*pData  += TDC7200_Read_N_Registers((TDC_Time_Registers[idx]),3);
	}

	uint32_t calibrationRegData[2] = {0,0};
	calibrationRegData[CALIBRATION_REG_1_DATA] = TDC7200_Read_N_Registers(TDC_CALIBRATION1, 3);
	//right shift by 1 since bit 23 is a parity bit, nothing to do with the calibration value
	calibrationRegData[CALIBRATION_REG_1_DATA] = (calibrationRegData[CALIBRATION_REG_1_DATA] >> 1);
	calibrationRegData[CALIBRATION_REG_2_DATA] = TDC7200_Read_N_Registers(TDC_CALIBRATION2, 3);
	//right shift by 1 since bit 23 is a parity bit, nothing to do with the calibration value
	calibrationRegData[CALIBRATION_REG_2_DATA] = (calibrationRegData[CALIBRATION_REG_2_DATA] >> 1);

	uint32_t calibration2Period = 0;
	calibration2Period = TDC7200_Read_N_Registers(TDC_CONFIG2, 1); //read entire Config2 register into this var
	calibration2Period &= CALIBRATION_PERIOD_pos; //isolate for bits 6 and 7, responsible for calibr.Period value

	//now we check which calibr.period settings were used and assign our variable accordingly
	if(calibration2Period == CALIBRATION2_PERIOD_2) calibration2Period = 2;
	else if(calibration2Period == CALIBRATION2_PERIOD_10) calibration2Period = 10;
	else if(calibration2Period == CALIBRATION2_PERIOD_20) calibration2Period = 20;
	else calibration2Period = 40;

	//formula used from TDC7200 datasheet pg.16
	double calCount = ((calibrationRegData[CALIBRATION_REG_2_DATA] - calibrationRegData[CALIBRATION_REG_1_DATA] )
			/ ((double)calibration2Period - 1) );

	//formula used from TDC7200 datasheet pg.16
	double normLSB = CLOCK_PERIOD / calCount;

	//this is our TimeOfFlight ToF value:
	*pData = (	(*pData) * normLSB	);
}

void myTDC_CableLength(double* pLength)
{
	double cablePropogationVel = (0.71 * 299792458.0L);
	*pLength = cablePropogationVel * (*pLength);
}


void myTDC_StartMeasurement(void)
{
	uint8_t regConfigurations = 0;
	regConfigurations = START_MEASUREMENT;
	TDC7200_WriteRegister(TDC_CONFIG1, &regConfigurations);
}

