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

#include "stdbool.h"

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

//bool get_configs_form_OD();
void control_valve(uint16_t ctrl_index, uint16_t status_index, GPIO_TypeDef* port, uint16_t pin);
float get_temperature(uint16_t adc_value);
void show_err_LED();

void set_pid_config();



/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Heat_Enable_Pin GPIO_PIN_4
#define Heat_Enable_GPIO_Port GPIOE
#define LED_Default_Pin GPIO_PIN_13
#define LED_Default_GPIO_Port GPIOC
#define CAN_Mode_Pin GPIO_PIN_0
#define CAN_Mode_GPIO_Port GPIOC
#define Pkg_Len_In_1_Pin GPIO_PIN_4
#define Pkg_Len_In_1_GPIO_Port GPIOA
#define Pkg_Len_In_2_Pin GPIO_PIN_5
#define Pkg_Len_In_2_GPIO_Port GPIOA
#define Roller_In_1_Pin GPIO_PIN_4
#define Roller_In_1_GPIO_Port GPIOC
#define Roller_In_2_Pin GPIO_PIN_5
#define Roller_In_2_GPIO_Port GPIOC
#define PH_X1_Pin GPIO_PIN_7
#define PH_X1_GPIO_Port GPIOE
#define PH_X1_EXTI_IRQn EXTI9_5_IRQn
#define PH_X2_Pin GPIO_PIN_8
#define PH_X2_GPIO_Port GPIOE
#define PH_X2_EXTI_IRQn EXTI9_5_IRQn
#define PH_X3_Pin GPIO_PIN_9
#define PH_X3_GPIO_Port GPIOE
#define PH_X3_EXTI_IRQn EXTI9_5_IRQn
#define PH_X4_Pin GPIO_PIN_10
#define PH_X4_GPIO_Port GPIOE
#define PH_X4_EXTI_IRQn EXTI15_10_IRQn
#define PH_X5_Pin GPIO_PIN_11
#define PH_X5_GPIO_Port GPIOE
#define PH_X5_EXTI_IRQn EXTI15_10_IRQn
#define PH_X6_Pin GPIO_PIN_12
#define PH_X6_GPIO_Port GPIOE
#define PH_X6_EXTI_IRQn EXTI15_10_IRQn
#define PH_X7_Pin GPIO_PIN_13
#define PH_X7_GPIO_Port GPIOE
#define PH_X7_EXTI_IRQn EXTI15_10_IRQn
#define PH_X8_Pin GPIO_PIN_14
#define PH_X8_GPIO_Port GPIOE
#define PH_X8_EXTI_IRQn EXTI15_10_IRQn
#define Pill_Pkg_Rev_Pin GPIO_PIN_12
#define Pill_Pkg_Rev_GPIO_Port GPIOB
#define Pill_Pkg_Brk_Pin GPIO_PIN_13
#define Pill_Pkg_Brk_GPIO_Port GPIOB
#define Pill_Pkg_Fwd_Pin GPIO_PIN_15
#define Pill_Pkg_Fwd_GPIO_Port GPIOB
#define Pill_Pkg_Jog_Pin GPIO_PIN_8
#define Pill_Pkg_Jog_GPIO_Port GPIOD
#define Pill_Pkg_Clr_Pin GPIO_PIN_9
#define Pill_Pkg_Clr_GPIO_Port GPIOD
#define RS_X2_Pin GPIO_PIN_10
#define RS_X2_GPIO_Port GPIOD
#define RS_X1_Pin GPIO_PIN_11
#define RS_X1_GPIO_Port GPIOD
#define RS_X4_Pin GPIO_PIN_12
#define RS_X4_GPIO_Port GPIOD
#define RS_X3_Pin GPIO_PIN_13
#define RS_X3_GPIO_Port GPIOD
#define RS_X6_Pin GPIO_PIN_14
#define RS_X6_GPIO_Port GPIOD
#define RS_X5_Pin GPIO_PIN_15
#define RS_X5_GPIO_Port GPIOD
#define RS_X8_Pin GPIO_PIN_6
#define RS_X8_GPIO_Port GPIOC
#define RS_X7_Pin GPIO_PIN_7
#define RS_X7_GPIO_Port GPIOC
#define SV_X2_Pin GPIO_PIN_3
#define SV_X2_GPIO_Port GPIOD
#define SV_X1_Pin GPIO_PIN_4
#define SV_X1_GPIO_Port GPIOD
#define SV_X4_Pin GPIO_PIN_5
#define SV_X4_GPIO_Port GPIOD
#define SV_X3_Pin GPIO_PIN_6
#define SV_X3_GPIO_Port GPIOD
#define LED_2_Pin GPIO_PIN_7
#define LED_2_GPIO_Port GPIOD
#define LED_1_Pin GPIO_PIN_3
#define LED_1_GPIO_Port GPIOB
#define LED_4_Pin GPIO_PIN_4
#define LED_4_GPIO_Port GPIOB
#define LED_3_Pin GPIO_PIN_5
#define LED_3_GPIO_Port GPIOB
#define Pkg_Dis_Enable_Pin GPIO_PIN_6
#define Pkg_Dis_Enable_GPIO_Port GPIOB
#define Pkg_Dis_Dir_Pin GPIO_PIN_7
#define Pkg_Dis_Dir_GPIO_Port GPIOB
#define Pill_Gate_Enable_Pin GPIO_PIN_0
#define Pill_Gate_Enable_GPIO_Port GPIOE
#define Pill_Gate_Dir_Pin GPIO_PIN_1
#define Pill_Gate_Dir_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
