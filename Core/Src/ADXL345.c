/*
 * adxl345.c
 *
 *  Created on: Jul 3, 2023
 *      Author: Halha
 */

#include "ADXL345.h"

//Initialization fxn:
uint8_t ADXL345_Init(ADXL345* device, I2C_HandleTypeDef* i2cHandle)
{

	//setup ADXL struct parameters:
	device->i2cHandle 		= i2cHandle;

	device->acc_mps2[0] 	= 0.0f;
	device->acc_mps2[1] 	= 0.0f;
	device->acc_mps2[2] 	= 0.0f;

//	device->temp_C 				= 0.0f; //no temperature sensor on this device (ADXL345)

	uint8_t errNum 				= 0; //store number of communication errors that occur (returned @endOfFxn)
	HAL_StatusTypeDef 		status;

	//Check deviceAddr, mems and part IDs
	uint8_t regData;

	status = ADXL345_ReadRegister(device, ADXL_DEV_ID, &regData); //reads the reg and stores 1byte into regData var
	errNum += (status != HAL_OK);
	if( regData != ADXL_REG_DEV_ID)
	{
		return 255;
	}

	//setup BW_RATE (bandwidth rate)
	regData = 0X0C; //upper nibble (0) = normal operation mode (a 1 would be low pwr mode)
									//lower nibble (C) = 1100 -> 400Hz OutputDataRate (default [A] = 100Hz)

	status = ADXL345_WriteRegister(device, ADXL_BW_RATE, &regData);
	errNum += (status != HAL_OK);

	//put sensor into measurement mode:
	regData = 0X28; //upper nibble (2) = Link bit = 1, default
									//lower nibble (8) = set device to measure mode
	status = ADXL345_WriteRegister(device, ADXL_PWR_CTRL, &regData);
	errNum += (status != HAL_OK);

	//DATA_FORMAT register config juicer:
	regData = 0x03; //set to 10bit res mode, range up to 16g
	status = ADXL345_WriteRegister(device, ADXL_DATA_FORMAT, &regData);
	errNum += (status != HAL_OK);

	return errNum; //ie: if errNum != 0, we will know that the ADXL init sequence failed, ong fr fr
}





//Data Acquisition fxns:
HAL_StatusTypeDef ADXL345_ReadAccel(ADXL345* device) //passing the device struct handle ptr juicer
{


	uint8_t regData[6]; //read all 6 registers (x0,x1 y0,y1 z0,z1) //24 bits each reg

	//combine the raw register vals to give the raw (unsigned) accel. readings
	int32_t accelRawSigned[3];

	HAL_StatusTypeDef status = ADXL345_ReadRegisters(device, ADXL_DATA_X0, regData, 6);
	accelRawSigned[0] = ( (regData[0]<< 8) | (regData[1]<< 0) ); //X
	accelRawSigned[1] = ( (regData[2]<< 8) | (regData[3]<< 0) ); //Y
	accelRawSigned[2] = ( (regData[4]<< 8) | (regData[5]<< 0) ); //Z

  //now convert to m/s^2 (given range setting of +/-16g)
	device->acc_mps2[0] = 9.81f * 0.000488f * accelRawSigned[0];
  device->acc_mps2[1] = 9.81f * 0.000488f * accelRawSigned[1];
  device->acc_mps2[2] = 9.81f * 0.000488f * accelRawSigned[2];

	return status;
}

//Low-Level Register Fxns:

//reads 1 byte of data from reg
HAL_StatusTypeDef ADXL345_ReadRegister(ADXL345* device, uint8_t reg, uint8_t* pdata)
{
	return HAL_I2C_Mem_Read(device->i2cHandle, ADXL_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, pdata, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef ADXL345_ReadRegisters(ADXL345* device, uint8_t reg, uint8_t* pdata, uint8_t len)
{
	return HAL_I2C_Mem_Read(device->i2cHandle, ADXL_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, pdata, len, HAL_MAX_DELAY);
}


//writes 1byte of data to reg
HAL_StatusTypeDef ADXL345_WriteRegister(ADXL345* device, uint8_t reg, uint8_t* pdata)
{
	return HAL_I2C_Mem_Write(device->i2cHandle, ADXL_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, pdata, 1, HAL_MAX_DELAY);

}











