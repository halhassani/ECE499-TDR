/*
 * TDC7200_driver.h
 *
 *  Created on: Jul 4, 2023
 *      Author: Halha
 */

#ifndef INC_TDC7200_DRIVER_H_
#define INC_TDC7200_DRIVER_H_

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

#endif /* INC_TDC7200_DRIVER_H_ */
