/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Disp_COM1_Pin GPIO_PIN_1
#define Disp_COM1_GPIO_Port GPIOB
#define Disp_DP_Pin GPIO_PIN_10
#define Disp_DP_GPIO_Port GPIOB
#define Disp_COM4_Pin GPIO_PIN_13
#define Disp_COM4_GPIO_Port GPIOB
#define Disp_COM3_Pin GPIO_PIN_14
#define Disp_COM3_GPIO_Port GPIOB
#define Disp_COM2_Pin GPIO_PIN_15
#define Disp_COM2_GPIO_Port GPIOB
#define Disp_C_Pin GPIO_PIN_3
#define Disp_C_GPIO_Port GPIOB
#define Disp_D_Pin GPIO_PIN_4
#define Disp_D_GPIO_Port GPIOB
#define Disp_B_Pin GPIO_PIN_5
#define Disp_B_GPIO_Port GPIOB
#define Disp_E_Pin GPIO_PIN_6
#define Disp_E_GPIO_Port GPIOB
#define Disp_A_Pin GPIO_PIN_7
#define Disp_A_GPIO_Port GPIOB
#define Disp_F_Pin GPIO_PIN_8
#define Disp_F_GPIO_Port GPIOB
#define Disp_G_Pin GPIO_PIN_9
#define Disp_G_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
