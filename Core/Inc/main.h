/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "rf.h"
#include "hx711.h"
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
#define Ignition_Pin_Pin GPIO_PIN_0
#define Ignition_Pin_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_2
#define SPI2_CS_GPIO_Port GPIOB
#define RF_TX_Pin GPIO_PIN_9
#define RF_TX_GPIO_Port GPIOA
#define RF_RX_Pin GPIO_PIN_10
#define RF_RX_GPIO_Port GPIOA
#define HX711_SCK_Pin GPIO_PIN_12
#define HX711_SCK_GPIO_Port GPIOA
#define HX711_DOUT_Pin GPIO_PIN_13
#define HX711_DOUT_GPIO_Port GPIOA
#define RF_M1_Pin GPIO_PIN_2
#define RF_M1_GPIO_Port GPIOD
#define RF_M0_Pin GPIO_PIN_3
#define RF_M0_GPIO_Port GPIOB
#define RF_AUX_Pin GPIO_PIN_4
#define RF_AUX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define RF_Channel FREQ_433
#define RF_Add_HIGH 0x03
#define RF_Add_LOW 0x03

#define RF_Gonderilecek_Channel FREQ_433
#define RF_Gonderilecek_Add_HIGH 0x06
#define RF_Gonderilecek_Add_LOW 0x07

#define SD_SPI_HANDLE hspi2
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
