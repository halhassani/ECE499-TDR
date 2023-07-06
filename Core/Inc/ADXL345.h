/*
 * ADXL345.h - Accelerometer I2C driver header file, ong fr fr
 *
 *  Created on: Jul 3, 2023
 *      Author: Halha
 */

#ifndef INC_ADXL345_H_
#define INC_ADXL345_H_

#include "stm32g4xx_hal.h" //needed for i2c, spi, gpio..all HAL API fxn calls
#include "main.h"

extern uint8_t spiData[2];

#define ADXL_I2C_ADDR					(0x53<<1) //as long as pin ASEL = 0 (grounded)
#define ADXL_DEV_ID						0XE5
#define ADXL_SPI_READ_CMD			(1<<7)
#define ADXL_SPI_WRITE_CMD		(0<<7)
#define ADXL_SPI_MB						(1<<6)

#define ADXL_REG_DEV_ID				0X00
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
	SPI_HandleTypeDef* spiHandle;
	float acc_mps2[3]; //(X,Y,Z) acceleration data in m/s^2


	//no temperature sensor on this device (ADXL345)
//	float temp_C; //temperature data in deg C

} ADXL345;

/************************************** I2C API Functions *********************************************/
//Initialization fxn:
uint8_t ADXL345_Init_I2C(ADXL345* device, I2C_HandleTypeDef* i2cHandle);
//Data Acquisition fxns:
HAL_StatusTypeDef ADXL345_ReadAccel_I2C(ADXL345* device); //passing the device struct handle ptr juicer
//Low-Level Register Fxns:
HAL_StatusTypeDef ADXL345_ReadRegister_I2C(ADXL345* device, uint8_t reg, uint8_t* pdata); //reads 1 byte of data from reg
HAL_StatusTypeDef ADXL345_ReadRegisters_I2C(ADXL345* device, uint8_t reg, uint8_t* pdata, uint8_t len);
HAL_StatusTypeDef ADXL345_WriteRegister_I2C(ADXL345* device, uint8_t reg, uint8_t* pdata); //writes 1byte of data to reg


/************************************** SPI API Functions *********************************************/
uint8_t ADXL345_Init_SPI(ADXL345* device, SPI_HandleTypeDef* spiHandle);
void ADXL345_ReadRegister_SPI(ADXL345* device, uint8_t reg, uint8_t* pdata);
void ADXL345_WriteRegister_SPI(ADXL345* device, uint8_t reg, uint8_t* pdata);
void ADXL345_ReadAccel_SPI(ADXL345* device);
void ADXL345_ReadRegisters_SPI(ADXL345* device, uint8_t reg, uint8_t* pdata, uint8_t len);

#endif /* INC_ADXL345_H_ */
