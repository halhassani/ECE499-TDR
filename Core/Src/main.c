/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dac.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include "ADXL345.h"
#include "TDC7200_driver.h"
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

char buff[100];
ADXL345 accelDevice;
uint8_t gTDC_TrigFlag= 0;
uint8_t gTDC_IntFlag= 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void myOLED_Startup(void);
void myOLED_PrintTDC_Data(double* tdcTime, double* tdcLength);
void myDAC_init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//int _write(int file, char *ptr, int len)
//{
//	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, 1000);
//	return len;
//}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  //Note: this GPIO pin PB4 is assigned to the system JTAG RESET pin for the onboard JTAG debugger.
  	//Since we arent using JTAG debugging, we can release PB4 pin from this alternate functionality
  		//and it can now be set to any GPIO functionality
  			//(in the MX_GPIO_Init fxn, it assigns PB4 as the SPI MISO line)
    GPIOB->MODER &= ~GPIO_MODER_MODE4_Msk;
    GPIOB->MODER |= GPIO_MODER_MODE4_0;

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim2);

	SSD1306_Init();		//initialize I2C communication b/w MCU and OLED screen
	myOLED_Startup(); //prints startup message to OLED screen
	HAL_Delay(2000);


	myTDC_EnablePowerOn();	//This fxn toggles TDC_EN pin from 0 to 1 to ensure TDC powers up properly
  myTDC_Init(); //this fxn configures various TDC registers to our desired settings

	myDAC_init(); 	//CONFIGURING DAC1 PERIPHERAL MANUALLY
				//(Cuz STM32CubeMX IDE doesn't let you configure DAC to output a simple DC voltage, which we want)

	//HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

	double totalTime = 0;
	double TDC_singleConvertedTime = 0;
	double TDC_convertedTimeAvg = 0;

	double totalLength = 0;
	double TDC_singleCableLength = 0;
	double TDC_cableLengthAvg = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while(1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


		if(HAL_GPIO_ReadPin(BUTTON2_GPIO_Port, BUTTON2_Pin) == 1)
		{
			//button debouncing and anti-button-holding-juicer
			HAL_Delay(100);
			while(HAL_GPIO_ReadPin(BUTTON2_GPIO_Port, BUTTON2_Pin) == 1); //do nothing, wait here until user releases button

			for(uint8_t itr = 0; itr <= 100; itr++)
			{
				myTDC_StartMeasurement(); //MCU will write to TDC configReg to START_MEASUREMENT
				while(gTDC_TrigFlag == 0); //DO NOTHING, WAIT UNTIL TDC_TRIG PIN TRIGGERS MCU ISR, ie: w8 until TDC rdy

				//SET PULSE SIGNAL TO HIGH (TDC WILL START MEASUREMENT AS SOON AS MCU SETS THIS PULSE SIGNAL HIGH)
				HAL_GPIO_WritePin(PULSE_SIG_GPIO_Port, PULSE_SIG_Pin, 1);
				HAL_GPIO_WritePin(PULSE_SIG_GPIO_Port, PULSE_SIG_Pin, 0);

				while(gTDC_IntFlag == 0); //wait here until TDC raises interrupt to MCU
				// (ie: wait for TDC to say to MCU: "MEASUREMENT DONE, COME COLLECT JUICER MEASUREMENTS")

				//yoink the measurements from TDC TIMEx registers and do some black magic math to convert to seconds
				myTDC_CalculateTime(&TDC_singleConvertedTime);
				totalTime += TDC_singleConvertedTime;

				TDC_singleCableLength = TDC_singleConvertedTime;
				myTDC_CableLength(&TDC_singleCableLength);
				totalLength += TDC_singleCableLength;
			}
			TDC_convertedTimeAvg = totalTime / 100.0;
			TDC_cableLengthAvg = totalLength / 100.0;


			totalTime = 0;
			totalLength = 0;
			//reset the TDC-related Trigger and Interrupt flags
			gTDC_TrigFlag = 0;
			gTDC_IntFlag 	= 0;

		}

		SSD1306_Clear();
		myOLED_PrintTDC_Data(&TDC_convertedTimeAvg, &TDC_cableLengthAvg); //print to OLED juicer
		HAL_Delay(250);
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void myOLED_Startup(void)
{
	SSD1306_Clear();
	SSD1306_GotoXY(38, 0);
	SSD1306_Puts(" TDR ", &Font_11x18, 0);
	SSD1306_GotoXY(24, 22);
	SSD1306_Puts("Time-Domain", &Font_7x10, 1);
	SSD1306_GotoXY(20, 34);
	SSD1306_Puts("Reflectometer", &Font_7x10, 1);

	SSD1306_GotoXY(24, 52);
	SSD1306_Puts("UVic ECE499", &Font_7x10, 1);
	SSD1306_UpdateScreen();
}

void myOLED_PrintTDC_Data(double* tdcTime, double* tdcLength)
{
	SSD1306_GotoXY(38, 0);
	SSD1306_Puts(" TDR ", &Font_11x18, 0);
	SSD1306_GotoXY(0, 24);
	sprintf(buff, "ToF: %0.4e s", *tdcTime);
	SSD1306_Puts(buff, &Font_7x10, 1);

	SSD1306_GotoXY(0, 40);
	SSD1306_Puts("Cable ", &Font_7x10, 1);
	SSD1306_GotoXY(50, 46);
	sprintf(buff, "%0.2f m", *tdcLength);
	SSD1306_Puts(buff, &Font_7x10, 1);
	SSD1306_GotoXY(0, 52);
	SSD1306_Puts("Length ", &Font_7x10, 1);
	SSD1306_UpdateScreen();
}


//THIS FXN WILL CONFIGURE DAC TO OUTPUT A STEADY VOLTAGE WHICH WE CAN ADJUST AS DESIRED
void myDAC_init(void)
{
  DAC->CR |= DAC_CR_EN1; 	// Enable channel 1 (connected to pin PA4)

  // Configure DAC trigger source (software trigger)
  DAC->CR &= ~DAC_CR_TEN1;
  DAC->CR &= ~DAC_CR_TSEL1;

  DAC->CR |= DAC_CR_EN1;	// Enable DAC

	// Set DAC output voltage to "on" state (e.g., VREF)
	DAC->DHR12R1 = 2047; 	//ie: DAC will output VREF/2 on its' output
												//(ex: if VREF = 3.3V, DAC outputs a constant 1.65V)

	// Trigger a DAC conversion - ie: DAC will now output 1.65V dc
	DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//confirm that the EXTI was triggered by TDC7200_TRIG_Pin (PIN PB6) and not some rando pin
																					// (since this ISR is shared between pins 5:9 of each GPIO port)
	if(GPIO_Pin == TDC7200_TRIG_Pin)
	{
		//TDC HAS RAISED INTERRUPT, SIGNALING TO MCU THAT NEW MEASUREMENT HAS BEGUN
		gTDC_TrigFlag = 1;  //set the triggerFlag variable to 1, then main while loop code juicer will continue
	}
	if (GPIO_Pin == TDC7200_INT_Pin)
	{
		gTDC_IntFlag = 1;
	}
}




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while(1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
