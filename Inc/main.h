/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#define LCD_LED_Pin GPIO_PIN_6
#define LCD_LED_GPIO_Port GPIOC
#define LCD_DC_RS_Pin GPIO_PIN_7
#define LCD_DC_RS_GPIO_Port GPIOC
#define LCD_RESET_Pin GPIO_PIN_8
#define LCD_RESET_GPIO_Port GPIOC
#define LCD_CS_Pin GPIO_PIN_9
#define LCD_CS_GPIO_Port GPIOC
#define RELAY_Pin GPIO_PIN_10
#define RELAY_GPIO_Port GPIOC
#define UV_Pin GPIO_PIN_11
#define UV_GPIO_Port GPIOC
#define lowspeed 70
#define medspeed 40
#define hightspeed 0
#define low 1
#define med 2
#define hight 3
#define Power_On 1
#define Power_Off 0
#define Nightmode_On 1
#define Nightmode_Off 0
#define Fresh_Air 1
#define Indoor 0
#define Manual 1
#define Auto 0
#define UVon 1
#define UVoff 0
#define IONon 1
#define IONoff 0
enum Interface{
	OFFALL,
	POWER_ON,
	POWER_OFF,
	SPEED_HIGHT,
	SPEED_LOW,
	SPEED_MEDIUM,
	NIGHT_ON,
	NIGHT_OFF,
	FRESH_AIR,
	INDOOR,
	MANUAL,
	AUTO,
	IONON,
	IONOFF,
	UVON,
	UVOFF

};
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
