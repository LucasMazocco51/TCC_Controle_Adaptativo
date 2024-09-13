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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

struct campo_Sw {
	unsigned Sw1 : 1;
	unsigned Sw2 : 1;
	unsigned Sw3 : 1;
	unsigned Sw4 : 1;
	unsigned Sw5 : 1;
	unsigned Sw6 : 1;
	unsigned rsd  : 2;
};

typedef union Switches{
	uint8_t all;
	struct campo_Sw bit;
}Switches;

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
#define Sw3_Pin GPIO_PIN_2
#define Sw3_GPIO_Port GPIOE
#define Sw4_Pin GPIO_PIN_5
#define Sw4_GPIO_Port GPIOE
#define Sw5_Pin GPIO_PIN_6
#define Sw5_GPIO_Port GPIOE
#define Sw6_Pin GPIO_PIN_13
#define Sw6_GPIO_Port GPIOC
#define Time_Rotina_Pin GPIO_PIN_8
#define Time_Rotina_GPIO_Port GPIOE
#define trigger_inj1_Pin GPIO_PIN_5
#define trigger_inj1_GPIO_Port GPIOB
#define LED_ONBOARD_Pin GPIO_PIN_9
#define LED_ONBOARD_GPIO_Port GPIOB
#define Sw2_Pin GPIO_PIN_0
#define Sw2_GPIO_Port GPIOE
#define Sw1_Pin GPIO_PIN_1
#define Sw1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
