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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile float gData = 0;
char buff[50];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ADC_ChannelTemp(void);
void ADC_Channel4(void);
void ADC_Channel15(void);
void OLED_Startup(void);

uint16_t ADCval[3];
uint16_t var = 0;

float val = 0.0;


uint32_t adcVal[3];
uint32_t gVariable = 0;

#define ADC_BUF_SIZE 		(20)

volatile uint32_t adcFlag = RESET;
volatile uint32_t i;
uint32_t adcBuff[ADC_BUF_SIZE];

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, 1000);
	return len;
}

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

	SSD1306_Init();
	OLED_Startup();
	HAL_Delay(2000);
	SSD1306_Clear();

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED); //Ex means this fxn is specific to this MCU family and therefore found in the extension file drivers
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

	HAL_ADCEx_MultiModeStart_DMA(&hadc1, adcBuff, ADC_BUF_SIZE);
	LL_ADC_SetMultiDMATransfer(ADC12_COMMON, LL_ADC_MULTI_REG_DMA_LIMIT_RES12_10B); //ONEShot DMA mode of dual ADC
//	HAL_ADC_Start_DMA(&hadc1, adcBuff, ADC_BUF_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT); //disable DMA interrupt cuz not really needed, saves time/performance

//	printf("\n\rApplication started. \n\r");

	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	//start timer2 (which triggers ADC)
	HAL_TIM_Base_Start(&htim2);
	//now start ADC as DMA

	HAL_TIM_Base_Start(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while(1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		if(TIM1->CNT != 0)
		{
			if(adcFlag)
			{
//			printf("------------\n\r");
				for(i = 0; i < ADC_BUF_SIZE; i++)
				{
				printf("%u\n\r", (uint16_t)(adcBuff[i] & 0X0000FFFF));	//LSB: Master ADC
				printf("%u\n\r", (uint16_t)(adcBuff[i] >> 16)); 		//MSB: Slave ADC
				}

				adcFlag = RESET;
				HAL_ADCEx_MultiModeStart_DMA(&hadc1, adcBuff, ADC_BUF_SIZE);
				LL_ADC_SetMultiDMATransfer(ADC12_COMMON, LL_ADC_MULTI_REG_DMA_LIMIT_RES12_10B);
//					HAL_ADC_Start_DMA(&hadc1, adcBuff, ADC_BUF_SIZE);
				__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT);
			}
		}

//		SSD1306_Clear();
//
//
//		SSD1306_GotoXY(38, 0);
//		SSD1306_Puts(" TDR ", &Font_11x18, 0);
//
//		SSD1306_GotoXY(0, 24);
////		sprintf(buffer1, "Temperature: %0.2f", temperature);
//		SSD1306_Puts(buff, &Font_7x10, 1);
//
//		SSD1306_UpdateScreen();
//
//		HAL_Delay(1000);

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
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
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

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	adcFlag = SET;
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
