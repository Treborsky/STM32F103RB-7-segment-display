/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
uint8_t number[10] = {
		  0b11111100,
		  0b01100000,
		  0b11011010,
		  0b11110010,
		  0b01100110,
		  0b10110110,
		  0b10111110,
		  0b11100000,
		  0b11111110,
		  0b11110110
  };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void Disp_WritePin(uint16_t pin, GPIO_PinState PinState);
void Disp_WriteBits(uint8_t bits);
void Disp_WriteReset(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Disp_WritePin(uint16_t pin, GPIO_PinState PinState)
{
	HAL_GPIO_WritePin(GPIOB, pin, PinState);
}

void Disp_WriteBits(uint8_t bits)
{
	HAL_GPIO_WritePin(GPIOB, Disp_A_Pin, (bits&0b01111111) == bits);
	HAL_GPIO_WritePin(GPIOB, Disp_B_Pin, (bits&0b10111111) == bits);
	HAL_GPIO_WritePin(GPIOB, Disp_C_Pin, (bits&0b11011111) == bits);
	HAL_GPIO_WritePin(GPIOB, Disp_D_Pin, (bits&0b11101111) == bits);
	HAL_GPIO_WritePin(GPIOB, Disp_E_Pin, (bits&0b11110111) == bits);
	HAL_GPIO_WritePin(GPIOB, Disp_F_Pin, (bits&0b11111011) == bits);
	HAL_GPIO_WritePin(GPIOB, Disp_G_Pin, (bits&0b11111101) == bits);
	HAL_GPIO_WritePin(GPIOB, Disp_DP_Pin, (bits&0b11111110) == bits);
}

void Disp_WriteReset(void)
{
	HAL_GPIO_WritePin(GPIOB, Disp_A_Pin, 0);
	HAL_GPIO_WritePin(GPIOB, Disp_B_Pin, 0);
	HAL_GPIO_WritePin(GPIOB, Disp_C_Pin, 0);
	HAL_GPIO_WritePin(GPIOB, Disp_D_Pin, 0);
	HAL_GPIO_WritePin(GPIOB, Disp_E_Pin, 0);
	HAL_GPIO_WritePin(GPIOB, Disp_F_Pin, 0);
	HAL_GPIO_WritePin(GPIOB, Disp_G_Pin, 0);
	HAL_GPIO_WritePin(GPIOB, Disp_DP_Pin, 0);
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_GPIO_WritePin(GPIOB, Disp_COM1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, Disp_COM2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, Disp_COM3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, Disp_COM4_Pin, GPIO_PIN_RESET);
  Disp_WriteBits(0b11111111);

  int i = 0;
  int j = i;

  while (1)
  {
	  /* Handling the counter */
	  i %= 10000;
	  j = i;
	  i++;

	  /* Displaying stuff */
	  Disp_WritePin(Disp_COM4_Pin, 1);
	  Disp_WriteBits(number[j%10]);
	  HAL_Delay(10);
	  Disp_WriteReset();
	  Disp_WritePin(Disp_COM4_Pin, 0);

	  j /= 10;

	  Disp_WritePin(Disp_COM3_Pin, 1);
	  Disp_WriteBits(number[j%10]);
	  HAL_Delay(10);
	  Disp_WriteReset();
	  Disp_WritePin(Disp_COM3_Pin, 0);

	  j /= 10;

	  Disp_WritePin(Disp_COM2_Pin, 1);
	  Disp_WriteBits(number[j%10]);
	  HAL_Delay(10);
	  Disp_WriteReset();
	  Disp_WritePin(Disp_COM2_Pin, 0);

	  j /= 10;

	  Disp_WritePin(Disp_COM1_Pin, 1);
	  Disp_WriteBits(number[j%10]);
	  HAL_Delay(10);
	  Disp_WriteReset();
	  Disp_WritePin(Disp_COM1_Pin, 0);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Disp_COM1_Pin|Disp_DP_Pin|Disp_COM4_Pin|Disp_COM3_Pin
                          |Disp_COM2_Pin|Disp_C_Pin|Disp_D_Pin|Disp_B_Pin
                          |Disp_E_Pin|Disp_A_Pin|Disp_F_Pin|Disp_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Disp_COM1_Pin Disp_DP_Pin Disp_COM4_Pin Disp_COM3_Pin
                           Disp_COM2_Pin Disp_C_Pin Disp_D_Pin Disp_B_Pin
                           Disp_E_Pin Disp_A_Pin Disp_F_Pin Disp_G_Pin */
  GPIO_InitStruct.Pin = Disp_COM1_Pin|Disp_DP_Pin|Disp_COM4_Pin|Disp_COM3_Pin
                          |Disp_COM2_Pin|Disp_C_Pin|Disp_D_Pin|Disp_B_Pin
                          |Disp_E_Pin|Disp_A_Pin|Disp_F_Pin|Disp_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
  while (1)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
