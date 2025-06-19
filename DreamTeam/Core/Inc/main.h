/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CS_I2C_SPI_Pin GPIO_PIN_3
#define CS_I2C_SPI_GPIO_Port GPIOE
#define sensor_derecho_Pin GPIO_PIN_0
#define sensor_derecho_GPIO_Port GPIOB
#define sensor_izquierdo_Pin GPIO_PIN_1
#define sensor_izquierdo_GPIO_Port GPIOB
#define m0_izquierda_Pin GPIO_PIN_11
#define m0_izquierda_GPIO_Port GPIOB
#define m1_izquierda_Pin GPIO_PIN_12
#define m1_izquierda_GPIO_Port GPIOB
#define m0_derecha_Pin GPIO_PIN_13
#define m0_derecha_GPIO_Port GPIOB
#define m1_derecha_Pin GPIO_PIN_14
#define m1_derecha_GPIO_Port GPIOB
#define sensor_frontal_Pin GPIO_PIN_6
#define sensor_frontal_GPIO_Port GPIOC
#define sensor_linea_Pin GPIO_PIN_7
#define sensor_linea_GPIO_Port GPIOC
#define V_izquierda_Pin GPIO_PIN_8
#define V_izquierda_GPIO_Port GPIOC
#define V_derecha_Pin GPIO_PIN_9
#define V_derecha_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
