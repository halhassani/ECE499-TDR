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


//Bitfield defines for individual bits in various registers
#define BIT_SET										(1<<0)
#define BIT_RESET									(0<<0)

/***************** ConfigReg_1 ******************/
#define MEASURE_MODE_1						BIT_RESET
#define START_MEASUREMENT					BIT_SET

#define START_EDGE_RISING					BIT_RESET
#define START_EDGE_FALLING				BIT_SET

#define STOP_EDGE_RISING					BIT_RESET
#define STOP_EDGE_FALLING					BIT_SET

#define TRIGG_EDGE_RISING					BIT_RESET
#define TRIGG_EDGE_FALLING				BIT_SET
/************************************************/
/************************************************/


/***************** ConfigReg_2 ******************/
#define NUM_STOP_SINGLE				0
#define NUM_STOP_TWO					1
#define NUM_STOP_THREE				2
#define NUM_STOP_FOUR					3
#define NUM_STOP_FIVE					4

#define AVG_CYCLES_1					0
#define AVG_CYCLES_2					1
#define AVG_CYCLES_4					2
#define AVG_CYCLES_8					3
#define AVG_CYCLES_16					4
#define AVG_CYCLES_32					5
#define AVG_CYCLES_64					6
#define AVG_CYCLES_128				7

#define CALIBRATION2_PERIOD_2				0
#define CALIBRATION2_PERIOD_10			1
#define CALIBRATION2_PERIOD_20			2
#define CALIBRATION2_PERIOD_40			3
/************************************************/
/************************************************/



//Register Structures to make low-level TDC configurations easier/faster ~~ABSTRACTION~~
typedef struct
{
	uint8_t start_measurement : 1;	//0 = no effect, 1 = Start new measurement
	uint8_t measurement_mode 	: 2;	//00 = Measurement_Mode_1 (which we want), 01 = Mode 2,  10/11 = Reserved
	uint8_t start_edge				: 1;	//0 = measure starts on RISING edge of START signal, 1 = FALLing edge
	uint8_t stop_edge					: 1;	//0 = Rising edge, 		1 = Falling edge
	uint8_t trigger_edge			: 1;	//0 = Rising edge, 		1 = Falling edge

			//0 = Parity bit disabled for MeasurementResultRegisters (TIME1-6, CLK_CNT1-5...etc) ie: Parity bit always 0
	uint8_t parity_enable			: 1;		//1 = Parity bit enabled (Even parity)

	//0 = Calibration not performed after int. measurement,  1 = calibration always performed at end
	uint8_t force_calibration :	1;
} TDC7200_ConfigReg1_t;

typedef struct
{
	uint8_t number_of_stops 			: 3;	//datasheet pg.26
	uint8_t average_cycles				: 3;
	uint8_t calibration2_periods	: 2;
}TDC7200_ConfigReg2_t;

typedef struct
{
	uint8_t new_measure_int					: 1;	//datasheet pg.27
	uint8_t course_counter_ovf_int	: 1;
	uint8_t clock_counter_ovf_int		: 1;
	uint8_t measure_started_flag		: 1;
	uint8_t measure_complete_flag		: 1;
	uint8_t reserved								: 3;

} TDC7200_IntStatusReg_t;

typedef struct
{
	uint8_t new_measure_mask				: 1;	//datasheet pg.28
	uint8_t coarse_counter_ovf_mask	: 1;
	uint8_t clock_counter_ovf_mask	: 1;
	uint8_t reserved								: 5;
} TDC7200_IntMaskReg_t;

typedef struct
{
	uint8_t coarse_counter_ovf_H 		: 8;	//Coarse Counter Overflow Value, upper 8 Bit ("High" byte)
} TDC7200_CoarseCounterOVF_H_Reg_t;

typedef struct
{
	uint8_t coarse_counter_ovf_L 		: 8;	//Coarse Counter Overflow Value, lower 8 Bit ("Low" byte)
} TDC7200_CoarseCounterOVF_L_Reg_t;

typedef struct
{
	uint8_t clock_counter_ovf_H 		: 8;	//CLOCK Counter Overflow_Value, UPPER 8 Bit ("High" byte)
} TDC7200_ClockCounterOVF_H_Reg_t;

typedef struct
{
	uint8_t clock_counter_ovf_L 		: 8;	//CLOCK Counter Overflow_Value, LOWER 8 Bit ("Low" byte)
} TDC7200_ClockCounterOVF_L_Reg_t;

typedef struct
{
	uint8_t clock_counter_stop_mask_H	: 8;	//CLOCK Counter STOP_Mask, UPPER 8 Bit ("High" byte)
} TDC7200_ClockCounterStopMask_H_Reg_t;

typedef struct
{
	uint8_t clock_counter_stop_mask_L	: 8;	//CLOCK Counter STOP_Mask, LOWER 8 Bit ("Low" byte)
} TDC7200_ClockCounterStopMask_L_Reg_t;

// ^^^^^^ These 10 registers are read/write capable. ^^^^^^
//The remaining 10 registers in the datasheet are read-only, so no abstraction code written for those.




volatile TDC7200_ConfigReg1_t* const TDC7200_CONFIG_REG1 				= (TDC7200_ConfigReg1_t*)TDC_CONFIG1;
volatile TDC7200_ConfigReg2_t* const TDC7200_CONFIG_REG2 				= (TDC7200_ConfigReg2_t*)TDC_CONFIG2;
volatile TDC7200_IntStatusReg_t* const TDC7200_INT_STATUS_REG 	= (TDC7200_IntStatusReg_t*)TDC_INT_STATUS;
volatile TDC7200_IntMaskReg_t* const TDC7200_INT_MASK_REG				= (TDC7200_IntMaskReg_t*)TDC_INT_MASK;

volatile TDC7200_CoarseCounterOVF_H_Reg_t* const TDC7200_COARSE_CNTR_OVF_H_REG =
													(TDC7200_CoarseCounterOVF_H_Reg_t*)TDC_COARSE_CNTR_OVF_H;

volatile TDC7200_CoarseCounterOVF_L_Reg_t* const TDC7200_COARSE_CNTR_OVF_L_REG =
													(TDC7200_CoarseCounterOVF_L_Reg_t*)TDC_COARSE_CNTR_OVF_L;

volatile TDC7200_ClockCounterOVF_H_Reg_t* const TDC7200_CLOCK_CNTR_OVF_H_REG =
													(TDC7200_ClockCounterOVF_H_Reg_t*)TDC_CLK_CNTR_OVF_H;

volatile TDC7200_ClockCounterOVF_L_Reg_t* const TDC7200_CLOCK_CNTR_OVF_L_REG =
													(TDC7200_ClockCounterOVF_L_Reg_t*)TDC_CLK_CNTR_OVF_L;

volatile TDC7200_ClockCounterStopMask_H_Reg_t* const TDC7200_CLOCK_CNTR_STOP_MASK_H_REG =
													(TDC7200_ClockCounterStopMask_H_Reg_t*)TDC_CLK_CNTR_STOP_MASK_H;

volatile TDC7200_ClockCounterStopMask_L_Reg_t* const TDC7200_CLOCK_CNTR_STOP_MASK_L_REG =
													(TDC7200_ClockCounterStopMask_L_Reg_t*)TDC_CLK_CNTR_STOP_MASK_L;



//Functions
void TDC7200_StartMeasure(void);
uint8_t TDC7200_WriteRegister(uint8_t reg, uint8_t txData);
void TDC7200_ReadRegister(uint8_t reg, uint8_t rxData);
double TDC7200_ReadBytes(uint8_t numOfBytes, uint8_t readOpCode);


#endif /* INC_TDC7200_DRIVER_H_ */
