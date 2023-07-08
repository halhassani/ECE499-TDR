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

//Bitfield definitions/macros
/***************** ConfigReg_1 ******************/
#define START_MEASUREMENT					(1<<0) //Start New Measurement. Writing a 1 will clear all bits in the Interrupt Status
																				 //Register and Start the measurement (by generating an TRIGG signal) and will
																				 //reset the content of all Measurement Results registers (TIME1 to TIME6,
																				 //CLOCK_COUNT1 to CLOCK_COUNT5, CALIBRATION1, CALIBRATION2) to 0.
																				 //Writing 0 to this bit has no effect.


#define MEASURE_MODE_1						((0<<1) | (0<<2)) //we using Measurement Mode 1 (recommended for < 500ns timings)
#define MEASURE_MODE_2						((1<<1) | (0<<2))

#define START_EDGE_RISING					(0<<3) //Measurement is started on Rising edge of START signal
#define START_EDGE_FALLING				(1<<3) //Measurement is started on Falling edge of START signal

#define STOP_EDGE_RISING					(0<<4) //Measurement is stopped on Rising edge of STOP signal
#define STOP_EDGE_FALLING					(1<<4) //Measurement is stopped on Falling edge of STOP signal

#define TRIGG_EDGE_RISING					(0<<5) //TRIGG is output as a Rising edge signal
#define TRIGG_EDGE_FALLING				(1<<5) //TRIGG is output as a Falling edge signal

#define PARITY_DISABLED						(0<<6) //Parity bit for Measurement Result Registers* disabled (Parity Bit always 0)
#define PARITY_ENABLED						(1<<6) //Parity bit for Measurement Result Registers enabled (Even Parity)

#define FORCE_CALIBRATION_OFF			(0<<7) //Calibration is not performed after interrupted measurement
																				//(Ex: due to counter overflow or missing STOP signal)

#define FORCE_CALIBRATION_ON			(1<<7) //Calibration is always performed at the end
																				//(Ex: after a counter overflow)

/************************************************/
/************************************************/
/************************************************/
/************************************************/

/***************** ConfigReg_2 ******************/
#define NUM_STOP_SINGLE				((0<<2) | (0<<1) | (0<<0)) //Single Stop
#define NUM_STOP_TWO					((0<<2) | (0<<1) | (1<<0)) //Two Stops
#define NUM_STOP_THREE				((0<<2) | (1<<1) | (0<<0)) //Three Stops .. etc
#define NUM_STOP_FOUR					((0<<2) | (1<<1) | (1<<0))
#define NUM_STOP_FIVE					((1<<2) | (0<<1) | (0<<0))
//bit combinations 101, 110, 111 have no effect, defaults to Single Stop mode

#define AVG_CYCLES_1					((0<<5) | (0<<4) | (0<<3)) // 1 Measurement cycle only (no Multi-Cycle Averaging Mode)
#define AVG_CYCLES_2					((0<<5) | (0<<4) | (1<<3)) // 2 Measurement cycles
#define AVG_CYCLES_4					((0<<5) | (1<<4) | (0<<3))
#define AVG_CYCLES_8					((0<<5) | (1<<4) | (1<<3))
#define AVG_CYCLES_16					((1<<5) | (0<<4) | (0<<3)) // etc..
#define AVG_CYCLES_32					((1<<5) | (0<<4) | (1<<3))
#define AVG_CYCLES_64					((1<<5) | (1<<4) | (0<<3))
#define AVG_CYCLES_128				((1<<5) | (1<<4) | (1<<3)) // 128 Measurement cycles

#define CALIBRATION2_PERIOD_2					((0<<7) | (0<<6)) //Calibration 2 - measuring 2 CLOCK periods
#define CALIBRATION2_PERIOD_10				((0<<7) | (1<<6)) //Calibration 2 - measuring 10 CLOCK periods
#define CALIBRATION2_PERIOD_20				((1<<7) | (0<<6)) //Calibration 2 - measuring 20 CLOCK periods
#define CALIBRATION2_PERIOD_40				((1<<7) | (1<<6)) //Calibration 2 - measuring 40 CLOCK periods
/************************************************/
/************************************************/

/***************** Interrupt Status Register ******************/
//If READing bits in these positions = 1: it means int detected from that particular bit/source (0 = no int detected)
//Writing a 1 in a certain bit position will clear that bits' int status
#define NEW_MEAS_INT_CLEAR					(1<<0)
#define COARSE_CNTR_OVF_INT_CLEAR		(1<<1)
#define CLOCK_CNTR_OVF_INT_CLEAR		(1<<2)
#define MEAS_STARTED_FLAG_CLEAR			(1<<3)
#define MEAS_COMPLETE_FLAG_CLEAR		(1<<4) //same int info as NEW_MEAS_INT bit flag ^
/************************************************/
/************************************************/

/***************** Interrupt Mask Register ******************/
#define NEW_MEAS_MASK_DISABLED						(0<<0) //New measurement int disabled
#define NEW_MEAS_MASK_ENABLED							(1<<0) //New measurement int enabled

#define COARSE_CNTR_OVF_MASK_DISABLED			(0<<1) //course counter ovf int disabled
#define COARSE_CNTR_OVF_MASK_ENABLED			(1<<1) //course counter ovf int enabled

#define CLOCK_CNTR_OVF_MASK_DISABLED			(0<<2) //CLOCK counter ovf int disabled
#define CLOCK_CNTR_OVF_MASK_ENABLED				(1<<2) //CLOCK counter ovf int disabled
/************************************************/
/************************************************/
/************************************************/
/************************************************/

//Functions
void TDC7200_StartMeasure(void);
uint8_t TDC7200_WriteRegister(uint8_t reg, uint8_t txData);
void TDC7200_ReadRegister(uint8_t reg, uint8_t rxData);
double TDC7200_ReadBytes(uint8_t numOfBytes, uint8_t readOpCode);


#endif /* INC_TDC7200_DRIVER_H_ */
