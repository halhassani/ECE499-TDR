/*
 * TDC7200_driver.h
 *
 *  Created on: Jul 4, 2023
 *      Author: Halha
 */

#ifndef INC_TDC7200_DRIVER_H_
#define INC_TDC7200_DRIVER_H_

#include "main.h"
#include "stm32g4xx.h"
#include "stm32g4xx_hal_spi.h"


//Command bits (READ, WRITE, AUTO-INCREMENT)
#define TDC_READ_CMD							(0 << 6)
#define TDC_WRITE_CMD							(1 << 6)
#define TDC_AUTO_INCR							(1 << 7)

//Register Defines for TDC7200 Memory Locations
#define TDC_CONFIG1								0X00
#define TDC_CONFIG2								0X01
#define TDC_INT_STATUS						0x02
#define TDC_INT_MASK							0X03
#define TDC_COARSE_CNTR_OVF_H			0X04
#define TDC_COARSE_CNTR_OVF_L			0X05
#define TDC_CLK_CNTR_OVF_H				0X06
#define TDC_CLK_CNTR_OVF_L				0X07
#define TDC_CLK_CNTR_STOP_MASK_H	0X08
#define TDC_CLK_CNTR_STOP_MASK_L	0X09
#define TDC_TIME1									0X10
#define TDC_CLK_COUNT1						0X11
#define TDC_TIME2									0X12
#define TDC_CLK_COUNT2						0X13
#define TDC_TIME3									0X14
#define TDC_CLK_COUNT3						0X15
#define TDC_TIME4									0X16
#define TDC_CLK_COUNT4						0X17
#define TDC_TIME5									0X18
#define TDC_CLK_COUNT5						0X19
#define TDC_TIME6									0X1A
#define TDC_CALIBRATION1					0X1B
#define TDC_CALIBRATION2					0X1C

//Register Structures to make low-level TDC configurations easier/faster ~~ABSTRACTION~~
typedef struct
{
	uint8_t start_measurement : 1;	//bit index 0
	uint8_t measurement_mode 	: 2;
	uint8_t start_edge				: 1;
	uint8_t stop_edge					: 1;
	uint8_t trigger_edge			: 1;
	uint8_t parity_enable			: 1;
	uint8_t force_calibration :	1;	//bit index 7
} TDC7200_ConfigReg1_t;

TDC7200_ConfigReg1_t* const TDC7200_CONFIG_REG1 = (TDC7200_ConfigReg1_t*)0x00;


//Functions
void TDC7200_StartMeasure(void);
uint8_t TDC7200_WriteRegister(uint8_t reg, uint8_t txData);
void TDC7200_ReadRegister(uint8_t reg, uint8_t rxData);
double TDC7200_ReadBytes(uint8_t numOfBytes, uint8_t readOpCode);


#endif /* INC_TDC7200_DRIVER_H_ */
