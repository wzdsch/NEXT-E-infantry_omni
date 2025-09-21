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
#define RSTN_IST8310_Pin GPIO_PIN_6
#define RSTN_IST8310_GPIO_Port GPIOG
#define DRDY_IST8310_Pin GPIO_PIN_3
#define DRDY_IST8310_GPIO_Port GPIOG
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOA
#define INT1_ACCEL_Pin GPIO_PIN_4
#define INT1_ACCEL_GPIO_Port GPIOC
#define INT1_GYRO_Pin GPIO_PIN_5
#define INT1_GYRO_GPIO_Port GPIOC
#define CS1_GYRO_Pin GPIO_PIN_0
#define CS1_GYRO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define NEW_ROBOT 1

#define MG6020ControlMode                                                                                                                   \
  1 /*6020有两种控制模式，电压控制(标识符为1FF/2FF),以及电流控制(1FE/2FE),目前 \ \ \ \ \ \                                                                                                                                          \
只有舵轮底盘舵向6020使用电流控制(为的是去做底盘功率限�??)，这个宏定义指在切换HAL_CAN_AddTxMessage()函数 \
\                                                                                                                                           \
\ \                                                                                                                                          \
\ \ \ 发�?�结构体的stdid,是该套代码最为底层的部分�?? \ \                                                                                                                                          \
因此此宏定义旨在起一个提示作�??,1为使用电流控�??(舵轮默认)(2024.10.25,by \ \ Nan)*/

#if NEW_ROBOT == 0

#define YAW_FLG_ECD 6977
#define IF_WITH_SUPERCAP 0 // 0无超电，1有超电
#define WITH_RM_POWER_MANAGER 1 // 0无电管，1有电管
#define motorYaw_ID 0x207

#endif

#if NEW_ROBOT == 1

#define YAW_FLG_ECD 6992
#define IF_WITH_SUPERCAP 0 // 0无超电，1有超电
#define WITH_RM_POWER_MANAGER 1 // 0无电管，1有电管
#define motorYaw_ID 0x207

#endif

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
