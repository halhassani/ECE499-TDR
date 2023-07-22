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

char buff[25];
ADXL345 accelDevice;
uint8_t signalBit = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ADC_ChannelTemp(void);
void ADC_Channel4(void);
void ADC_Channel15(void);
void OLED_Startup(void);


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

	SSD1306_Init();
	OLED_Startup();
	HAL_Delay(2000);
	SSD1306_Clear();


  HAL_Delay(1000);
  HAL_GPIO_WritePin(TDC7200_EN_GPIO_Port, TDC7200_EN_Pin, 0);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(TDC7200_EN_GPIO_Port, TDC7200_EN_Pin, 1);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(TDC7200_EN_GPIO_Port, TDC7200_EN_Pin, 0);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(TDC7200_EN_GPIO_Port, TDC7200_EN_Pin, 1);


	uint8_t juice = TDC_WRITE_CMD | MEASURE_MODE_1 | START_EDGE_RISING | STOP_EDGE_FALLING;

  //HAL_Delay(500);
	//TDC7200_startMeasurement();
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	//TDC7200_WriteRegister(TDC_CONFIG1, &juice);
//
	//juice  |= START_MEASUREMENT;
	//TDC7200_WriteRegister(TDC_CONFIG1, &juice);

  HAL_Delay(500);
  double idk = 99;
  //idk = TDC7200_Read_N_Registers(TDC_CONFIG2, 1);
 //HAL_Delay(500);


	//ADXL345_Init_SPI(&accelDevice, &hspi1);

  //testing juicer for MISO pin -- MISO PIN PB4 WORKS CORRECTLY CONFIRMED
//  GPIOB->MODER &= ~GPIO_MODER_MODE4_Msk;
//  GPIOB->MODER |= GPIO_MODER_MODE4_0;
//  GPIOB->BSRR = 1 << 4; 	//set to 1
//  GPIOB->BSRR = 1 << 20; 	//set to 0
//  GPIOB->BSRR = 1 << 4; 	//set to 1

  uint8_t incr = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while(1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		//ADXL345_ReadAccel_SPI(&accelDevice);
		//TDC7200_startMeasurement();
		//TDC7200_WriteRegister(TDC_CONFIG1, &juice);
	  idk = TDC7200_Read_N_Registers((TDC_CONFIG1), 1);
	  HAL_GPIO_TogglePin(PULSE_SIG_GPIO_Port, PULSE_SIG_Pin);
	  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

		SSD1306_Clear();

		SSD1306_GotoXY(38, 0);
		SSD1306_Puts(" TDR ", &Font_11x18, 0);

		SSD1306_GotoXY(0, 36);
		sprintf(buff, "rxData: %0.2f", idk);
		SSD1306_Puts(buff, &Font_7x10, 1);

		SSD1306_GotoXY(0, 48);
		sprintf(buff, "PulseSig: %d", signalBit);
		SSD1306_Puts(buff, &Font_7x10, 1);

		SSD1306_UpdateScreen();

		HAL_Delay(1000);
		//TDC7200_startMeasurement();
		incr++;
		if(incr > 22) incr = 0;

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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//HAL_GPIO_TogglePin(TDC7200_EN_GPIO_Port, TDC7200_EN_Pin);
	//signalBit = HAL_GPIO_ReadPin(PULSE_SIG_IN_GPIO_Port, PULSE_SIG_IN_Pin);
}

void OLED_Startup(void)
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
