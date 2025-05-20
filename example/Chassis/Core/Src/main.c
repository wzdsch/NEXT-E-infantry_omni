/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DJI_Motor.h"
#include "MCUConnect.h"
#include "MCUConnectStructs.h"
#include "RC_task.h"
#include "Tools.h"
#include "addOns.h"
#include "bsp_wbus.h"
#include "chassis.h"
#include "pid.h"
#include "pidData.h"
#include "referee.h"
#include "refereeData_v1.6.h"
#include "ui.h"
#include "SuperCap.h"

// #include "remote_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// 底盘电机结构�?
// 底盘电机结构�???;
DJI_MotorGroup group1;
DJI_Motor motor1;
DJI_Motor motor2;
DJI_Motor motor3;
DJI_Motor motor4;

// 云台电机结构�???;
DJI_MotorGroup group2;
DJI_Motor motorYaw;

chassis chassis1;
MCUConnection connect;

// referee referee1;
Referee_data RefereeData;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_USART6_UART_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART3_UART_Init();
  MX_TIM11_Init();
  MX_TIM10_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /***********************************************************************************************/
  /**************************************电机初始�???************************************************/
  /***********************************************************************************************/
  //  底盘电机

  DJI_MotorGroupInit(&group1, &hcan2, CAN_RX_FIFO0);

  DJI_MotorInit(&motor1, 0x204, 0, pid0, NULL);  // 起火步兵地盘ID�?3412，另外一�?1234
  DJI_MotorPidSet(&motor1, &(motor1.motorPid0), PID_POSITION, M3508_Speed_PID, &(motor1.realSpeedF),
                  &(motor1.target));

  DJI_MotorInit(&motor2, 0x201, 0, pid0, NULL);
  DJI_MotorPidSet(&motor2, &(motor2.motorPid0), PID_POSITION, M3508_Speed_PID, &(motor2.realSpeedF),
                  &(motor2.target));

  DJI_MotorInit(&motor3, 0x202, 0, pid0, NULL);
  DJI_MotorPidSet(&motor3, &(motor3.motorPid0), PID_POSITION, M3508_Speed_PID, &(motor3.realSpeedF),
                  &(motor3.target));

  DJI_MotorInit(&motor4, 0x203, 0, pid0, NULL);
  DJI_MotorPidSet(&motor4, &(motor4.motorPid0), PID_POSITION, M3508_Speed_PID, &(motor4.realSpeedF),
                  &(motor4.target));
  #ifdef REFEREE_H
    #ifdef SUPERCAP_H
      DJI_MotorPostProcessHandlerSet(&motor1, &Supercap_powerlimit);
      DJI_MotorPostProcessHandlerSet(&motor2, &Supercap_powerlimit);
      DJI_MotorPostProcessHandlerSet(&motor3, &Supercap_powerlimit);
      DJI_MotorPostProcessHandlerSet(&motor4, &Supercap_powerlimit);
    #else
      DJI_MotorPostProcessHandlerSet(&motor1, &powerlimit_pro);
      DJI_MotorPostProcessHandlerSet(&motor2, &powerlimit_pro);
      DJI_MotorPostProcessHandlerSet(&motor3, &powerlimit_pro);
      DJI_MotorPostProcessHandlerSet(&motor4, &powerlimit_pro);
    #endif
    DJI_MotorCalculateResultSet(&motor1, &(motor1.postProcessResult));
    DJI_MotorCalculateResultSet(&motor2, &(motor2.postProcessResult));
    DJI_MotorCalculateResultSet(&motor3, &(motor3.postProcessResult));
    DJI_MotorCalculateResultSet(&motor4, &(motor4.postProcessResult));
  #else
    DJI_MotorCalculateResultSet(&motor1, &(motor1.pidOutput0));
    DJI_MotorCalculateResultSet(&motor2, &(motor2.pidOutput0));
    DJI_MotorCalculateResultSet(&motor3, &(motor3.pidOutput0));
    DJI_MotorCalculateResultSet(&motor4, &(motor4.pidOutput0));
  #endif
    DJI_MotorListAdd(&group1, &motor1);
    DJI_MotorListAdd(&group1, &motor2);
    DJI_MotorListAdd(&group1, &motor3);
    DJI_MotorListAdd(&group1, &motor4);

  DJI_MotorEnable(&motor1);
  DJI_MotorEnable(&motor2);
  DJI_MotorEnable(&motor3);
  DJI_MotorEnable(&motor4);

  // 云台yaw轴电机，这里可以把相关计算放在底盘，这里是为了接收电机数据才创建�??
  DJI_MotorGroupInit(&group2, &hcan1, CAN_RX_FIFO0);  // 电机组初始化
  // 这里flagEcd只能�??0
  DJI_MotorInit(&motorYaw, motorYaw_ID, 0, NULL, NULL);  // 电机初始�? 起火步兵的ID
  // DJI_MotorInit(&motorYaw, 0x205, 0, NULL, NULL);  // 电机初始�? 另一台步兵的ID
  DJI_MotorListAdd(&group2, &motorYaw);

  /***********************************************************************************************/
  /**************************************底盘初始化************************************************/
  /***********************************************************************************************/
  // 这里�??要依赖yaw轴电机或者底盘imu来底盘跟随，�??以上面创建了yaw轴电机的结构体来接收电机数据
  chassisINIT(&chassis1, &group1, &motor1, &motor2, &motor3, &motor4, &motorYaw);
  chasisFollowINIT(&chassis1, 2468, PID_POSITION, Chassis_Angle_PID, PID_POSITION,
                   Chassis_Speed_PID);  // 底盘跟随初始�?(起火步兵)
  followResultSet(&chassis1, &(chassis1.followPidout));  // 底盘跟随结果设置
  connectionINIT(&connect, &hcan1, 0x400, CAN_RX_FIFO1);

  /***********************************************************************************************/
  /*************************************任务启用管理***********************************************/
  /***********************************************************************************************/
  HAL_TIM_Base_Start_IT(&htim12);
  refereeINIT(&htim12);
  HAL_TIM_Base_Start_IT(&htim14);  // 电机pid计算
  HAL_TIM_Base_Start_IT(&htim13);  // 双机通信
  HAL_TIM_Base_Start_IT(&htim11);  // UI
  HAL_TIM_Base_Start_IT(&htim10);
  
#ifdef SUPERCAP_H
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, supercap_rxD.rx_buf, sizeof(supercap_rxD.rx_buf));
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    UI_Init();  // UI初始�?
    HAL_Delay(1);
    ui_self_id = robot_state.robot_id;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  while (1) {
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
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
