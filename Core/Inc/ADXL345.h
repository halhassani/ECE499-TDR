/*
 * ADXL345.h - Accelerometer I2C driver header file, ong fr fr
 *
 *  Created on: Jul 3, 2023
 *      Author: Halha
 */

#ifndef INC_ADXL345_H_
#define INC_ADXL345_H_

#include "stm32g4xx_hal.h" //needed for i2c



#define ADXL_I2C_ADDR					(0x53<<1) //as long as pin ASEL = 0 (grounded)
#define ADXL_DEV_ID						0X00
#define ADXL_THRESH_TAP				0X1D
#define ADXL_OFSX							0X1E
#define ADXL_OFSY							0X1F
#define ADXL_OFSZ							0X20
#define ADXL_DUR							0X21
#define ADXL_LATENT						0X22
#define ADXL_WINDOW						0X23


#define ADXL_BW_RATE					0X2C
#define ADXL_PWR_CTRL					0X2D

#define ADXL_DATA_FORMAT			0X31
#define ADXL_DATA_X0					0X32
#define ADXL_DATA_X1					0X33
#define ADXL_DATA_Y0					0X34
#define ADXL_DATA_Y1					0X35
#define ADXL_DATA_Z0					0X36
#define ADXL_DATA_Z1					0X37
#define ADXL_FIFO_CTRL				0X38
#define ADXL_FIFO_STATUS			0X39


typedef struct
{

	I2C_HandleTypeDef* i2cHandle;
	float acc_mps2[3]; //(X,Y,Z) acceleration data in m/s^2


	//no temperature sensor on this device (ADXL345)
//	float temp_C; //temperature data in deg C

} ADXL345;


//Initialization fxn:
uint8_t ADXL345_Init(ADXL345* device, I2C_HandleTypeDef* i2cHandle);

//Data Acquisition fxns:
HAL_StatusTypeDef ADXL345_ReadAccel(ADXL345* device); //passing the device struct handle ptr juicer

//Low-Level Register Fxns:
HAL_StatusTypeDef ADXL345_ReadRegister(ADXL345* device, uint8_t reg, uint8_t* pdata); //reads 1 byte of data from reg
HAL_StatusTypeDef ADXL345_ReadRegisters(ADXL345* device, uint8_t reg, uint8_t* pdata, uint8_t len);

HAL_StatusTypeDef ADXL345_WriteRegister(ADXL345* device, uint8_t reg, uint8_t* pdata); //writes 1byte of data to reg


#endif /* INC_ADXL345_H_ */
