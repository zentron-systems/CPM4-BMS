/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
#define BQ79616_NFAULT_Pin GPIO_PIN_11
#define BQ79616_NFAULT_GPIO_Port GPIOC
#define CONFIG_ADDRESS_J2_Pin GPIO_PIN_12
#define CONFIG_ADDRESS_J2_GPIO_Port GPIOC
#define PC13_LED_Pin GPIO_PIN_13
#define PC13_LED_GPIO_Port GPIOC
#define USART6_TX_TO_BQ79616_RX_Pin GPIO_PIN_0
#define USART6_TX_TO_BQ79616_RX_GPIO_Port GPIOC
#define USART6_RX_FROM_BQ79616_TX_Pin GPIO_PIN_1
#define USART6_RX_FROM_BQ79616_TX_GPIO_Port GPIOC
#define CONFIG_ADDRESS_J3_Pin GPIO_PIN_2
#define CONFIG_ADDRESS_J3_GPIO_Port GPIOC
#define NUM_SLAVES_J2_Pin GPIO_PIN_3
#define NUM_SLAVES_J2_GPIO_Port GPIOC
#define NUM_SLAVES_J1_Pin GPIO_PIN_0
#define NUM_SLAVES_J1_GPIO_Port GPIOA
#define NUM_SLAVES_J0_Pin GPIO_PIN_1
#define NUM_SLAVES_J0_GPIO_Port GPIOA
#define MCU_HVIL_RETURN_Pin GPIO_PIN_2
#define MCU_HVIL_RETURN_GPIO_Port GPIOA
#define MCU_HVIL_OUT_Pin GPIO_PIN_3
#define MCU_HVIL_OUT_GPIO_Port GPIOA
#define MCU_BALANCING_Pin GPIO_PIN_4
#define MCU_BALANCING_GPIO_Port GPIOA
#define MCU_OVER_VOLTAGE_Pin GPIO_PIN_5
#define MCU_OVER_VOLTAGE_GPIO_Port GPIOA
#define MCU_UNDER_VOLTAGE_Pin GPIO_PIN_6
#define MCU_UNDER_VOLTAGE_GPIO_Port GPIOA
#define MCU_OVER_TEMP_Pin GPIO_PIN_7
#define MCU_OVER_TEMP_GPIO_Port GPIOA
#define MCU_UNDER_TEMP_Pin GPIO_PIN_4
#define MCU_UNDER_TEMP_GPIO_Port GPIOC
#define MCU_BATTERY_HEAT_Pin GPIO_PIN_5
#define MCU_BATTERY_HEAT_GPIO_Port GPIOC
#define MCU_BATTERY_COOL_Pin GPIO_PIN_0
#define MCU_BATTERY_COOL_GPIO_Port GPIOB
#define MCU_OVER_CURRENT_Pin GPIO_PIN_1
#define MCU_OVER_CURRENT_GPIO_Port GPIOB
#define MCU_CHARGE_ALLOWED_Pin GPIO_PIN_2
#define MCU_CHARGE_ALLOWED_GPIO_Port GPIOB
#define MCU_CURRENT_SENSOR_HIGH_Pin GPIO_PIN_10
#define MCU_CURRENT_SENSOR_HIGH_GPIO_Port GPIOB
#define MCU_CURRENT_SENSOR_LOW_Pin GPIO_PIN_11
#define MCU_CURRENT_SENSOR_LOW_GPIO_Port GPIOB
#define MCU_HV_PRESENT_Pin GPIO_PIN_14
#define MCU_HV_PRESENT_GPIO_Port GPIOB
#define MCU_CHARGE_REQUEST_Pin GPIO_PIN_15
#define MCU_CHARGE_REQUEST_GPIO_Port GPIOB
#define MCU_POWER_HOLD_Pin GPIO_PIN_8
#define MCU_POWER_HOLD_GPIO_Port GPIOA
#define MCU_D_AUX_IN_1_Pin GPIO_PIN_6
#define MCU_D_AUX_IN_1_GPIO_Port GPIOC
#define MCU_CRASH_Pin GPIO_PIN_7
#define MCU_CRASH_GPIO_Port GPIOC
#define MCU_KEY_START_Pin GPIO_PIN_8
#define MCU_KEY_START_GPIO_Port GPIOD
#define MCU_KEY_ON_Pin GPIO_PIN_9
#define MCU_KEY_ON_GPIO_Port GPIOD
#define DIAG_EN_CONTACTORS_Pin GPIO_PIN_15
#define DIAG_EN_CONTACTORS_GPIO_Port GPIOA
#define MCU_CONTACTOR_FAULT_Pin GPIO_PIN_8
#define MCU_CONTACTOR_FAULT_GPIO_Port GPIOC
#define MCU_CONTACTOR__Pin GPIO_PIN_9
#define MCU_CONTACTOR__GPIO_Port GPIOC
#define MCU_CONTACTOR_FAULTD2_Pin GPIO_PIN_2
#define MCU_CONTACTOR_FAULTD2_GPIO_Port GPIOD
#define MCU_PRECHARGE_Pin GPIO_PIN_3
#define MCU_PRECHARGE_GPIO_Port GPIOD
#define MCU_PRECHARGE_FAULT_Pin GPIO_PIN_4
#define MCU_PRECHARGE_FAULT_GPIO_Port GPIOD
#define MCU_CONTACTOR_D5_Pin GPIO_PIN_5
#define MCU_CONTACTOR_D5_GPIO_Port GPIOD
#define MCU_PYROFUSE_Pin GPIO_PIN_6
#define MCU_PYROFUSE_GPIO_Port GPIOD
#define CONFIG_CELLS_J1_Pin GPIO_PIN_3
#define CONFIG_CELLS_J1_GPIO_Port GPIOB
#define CONFIG_CELLS_J2_Pin GPIO_PIN_4
#define CONFIG_CELLS_J2_GPIO_Port GPIOB
#define CONFIG_CELLS_J3_Pin GPIO_PIN_5
#define CONFIG_CELLS_J3_GPIO_Port GPIOB
#define CONFIG_CELLS_J4_Pin GPIO_PIN_6
#define CONFIG_CELLS_J4_GPIO_Port GPIOB
#define CONFIG_HICAP_LONGLIFE_Pin GPIO_PIN_7
#define CONFIG_HICAP_LONGLIFE_GPIO_Port GPIOB
#define CONFIG_ION_LFP_Pin GPIO_PIN_8
#define CONFIG_ION_LFP_GPIO_Port GPIOB
#define CONFIG_ADDRESS_J1_Pin GPIO_PIN_9
#define CONFIG_ADDRESS_J1_GPIO_Port GPIOB
#define CONFIG_NO_APP_Pin GPIO_PIN_10
#define CONFIG_NO_APP_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
