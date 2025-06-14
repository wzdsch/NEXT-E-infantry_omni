/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// #include "IMU.h"
#include "BMI088.h"
#include "DJI_Motor.h"
#include "MCUConnect.h"
#include "MCUConnectStructs.h"
#include "RC_task.h"
#include "RobotModManage.h"
#include "VisionConnect.h"
#include "addOns.h"
#include "bsp_wbus.h"
#include "gimbal.h"
#include "math.h"
#include "pid.h"
#include "pidData.h"
#include "referee.h"
#include "remote_control.h"
#include "shooter.h"
#include "stm32f4xx_it.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
DJI_MotorGroup group1;
DJI_Motor motorYaw;

DJI_Motor motorPitch;

DJI_MotorGroup group2;
DJI_Motor motorfriR;
DJI_Motor motorfriL;
DJI_Motor motor2006;

shooter shooter1;
VisionConnect vision1;
ChassisControl ChassisControlData;
GimbalControl GimbalControlData;
// RC_ctrl_t rc_ctrl;

MCUConnection connect;
gimbal gimbal1;

BMI088_IMU BMI088_gimbal;
Referee_data RefereeData;
uint8_t tuneTime = 0;

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
extern DMA_HandleTypeDef hdma2;
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM12_Init();
  MX_USB_DEVICE_Init();
  MX_TIM11_Init();
  MX_SPI1_Init();
  MX_TIM10_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  /***********************************************************************************************/
  /**************************************电机初始�???************************************************/
  /***********************************************************************************************/
  // 云台电机
  DJI_MotorGroupInit(&group1, &hcan1, CAN_RX_FIFO0);
  DJI_MotorInit(&motorYaw, 0x207, 0, pidBoth, 0);
  //  处理�???螺仪的数据，以及过零
  DJI_MotorPreProcessHandlerSet(&motorYaw, &IMUecdZeroCrossing);

  DJI_MotorPidSet(&motorYaw, &(motorYaw.motorPid0), PID_POSITION, YAW_Angle_PID,
				  &(motorYaw.preProcessResult), &(motorYaw.target));
  DJI_MotorPidSet(&motorYaw, &(motorYaw.motorPid1), PID_POSITION, YAW_Speed_PID,
                  (fp32 *)&(BMI088_gimbal.yawSpeed), &(motorYaw.pidOutput0));
  DJI_MotorPostProcessHandlerSet(&motorYaw, &negative);
  DJI_MotorCalculateResultSet(&motorYaw, &(motorYaw.postProcessResult));

  DJI_MotorListAdd(&group1, &motorYaw);

  DJI_MotorInit(&motorPitch, 0x206, 0, pidBoth, 4981);  // 这是起火步兵pitchID

#if IMU_PITCH == 0
  DJI_MotorPreProcessHandlerSet(&motorPitch, &ecdZeroCrossing);
  DJI_MotorPidSet(&motorPitch, &(motorPitch.motorPid0), PID_POSITION, PITCH_Angle_PID,
                  &(motorPitch.preProcessResult), &(motorPitch.target));
  DJI_MotorPidSet(&motorPitch, &(motorPitch.motorPid1), PID_POSITION, PITCH_Speed_PID,
                  &(motorPitch.realSpeedF), &(motorPitch.pidOutput0));
  DJI_MotorCalculateResultSet(&motorPitch, &(motorPitch.pidOutput1));
  DJI_MotorListAdd(&group1, &motorPitch);

#else

  DJI_MotorPreProcessHandlerSet(&motorPitch, &ecdZeroCrossing);
  DJI_MotorPidSet(&motorPitch, &(motorPitch.motorPid0), PID_POSITION, PITCH_Angle_IMU_PID,
                  &(BMI088_gimbal.pitchAngle), &(motorPitch.target));
  DJI_MotorPidSet(&motorPitch, &(motorPitch.motorPid1), PID_POSITION, PITCH_Speed_IMU_PID,
                  &(BMI088_gimbal.pitchSpeed), &(motorPitch.pidOutput0));
  DJI_MotorCalculateResultSet(&motorPitch, &(motorPitch.pidOutput1));
  DJI_MotorListAdd(&group1, &motorPitch);

#endif

  // 发射机构
  DJI_MotorGroupInit(&group2, &hcan2, CAN_RX_FIFO0);
  DJI_MotorInit(&motorfriR, 0x201, 0, pid0, NULL);
  DJI_MotorPidSet(&motorfriR, &(motorfriR.motorPid0), PID_POSITION, FRI_Speed_PID,
                  &(motorfriR.realSpeedF), &(motorfriR.target));
  DJI_MotorCalculateResultSet(&motorfriR, &(motorfriR.pidOutput0));
  DJI_MotorListAdd(&group2, &motorfriR);

  DJI_MotorInit(&motorfriL, 0x202, 1, pid0, NULL);
  DJI_MotorPidSet(&motorfriL, &(motorfriL.motorPid0), PID_POSITION, FRI_Speed_PID,
                  &(motorfriL.realSpeedF), &(motorfriL.target));
  DJI_MotorCalculateResultSet(&motorfriL, &(motorfriL.pidOutput0));
  DJI_MotorListAdd(&group2, &motorfriL);

#if SUPPLIER_ECD == 0
  DJI_MotorInit(&motor2006, 0x203, 0, pid0, NULL);
  DJI_MotorPidSet(&motor2006, &(motor2006.motorPid0), PID_POSITION, M2006_Speed_PID,
                  &(motor2006.realSpeedF), &(motor2006.target));
  DJI_MotorCalculateResultSet(&motor2006, &(motor2006.pidOutput0));
  DJI_MotorListAdd(&group1, &motor2006); // 拨弹在底盘，用can1
#else
  DJI_MotorInit(&motor2006, 0x203, 0, pidBoth, NULL);
  DJI_MotorPidSet(&motor2006, &(motor2006.motorPid0), PID_POSITION, M2006_Angle_PID, &(shooter1.supplier_total_ecd), &(motor2006.target));
  DJI_MotorPidSet(&motor2006, &(motor2006.motorPid1), PID_POSITION, M2006_Speed_PID,
                  &(motor2006.realSpeedF), &(motor2006.pidOutput0));
  DJI_MotorCalculateResultSet(&motor2006, &(motor2006.pidOutput1));
  DJI_MotorListAdd(&group1, &motor2006);
#endif

  /***********************************************************************************************/
  /**************************************云台初始�???************************************************/
  /***********************************************************************************************/
  gimbalINIT(&gimbal1, &group1, &motorYaw, &motorPitch);
#if SUPPLIER_ECD == 1
  shooterINIT(&shooter1, &group2, &motorfriL, &motorfriR, &motor2006, shooterSpeed, SHOOTER_FREQ_DFLT,
              10, TOTAL_ECD_PER_SHOOT);
#elif SUPPLIER_ECD == 0
  shooterINIT(&shooter1, &group2, &motorfriL, &motorfriR, &motor2006, shooterSpeed,
              SHOOTER_FREQ_DFLT, 10, TOTAL_ECD_PER_SHOOT);
#endif

  connectionINIT(&connect, &hcan1, 0x400, CAN_RX_FIFO1);
  VisionConnectINIT(&vision1, BLUE);
  /***********************************************************************************************/
  /*************************************任务启用管理***********************************************/
  /***********************************************************************************************/
  BMI088_INIT(&BMI088_gimbal);
  tuneINIT();
  tune(2);  // 初始化完成则响两�???

  W_BUS_Init(W_BUSRxBuffer[0], W_BUSRxBuffer[1], RC_FRAME_NUM);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim9);   // 视觉发�??
  refereeINIT(&htim9);             // 裁判系统数据接收
  HAL_TIM_Base_Start_IT(&htim6);   // bmi088
  HAL_TIM_Base_Start_IT(&htim8);   // vofa
  HAL_TIM_Base_Start_IT(&htim11);  //
  HAL_TIM_Base_Start_IT(&htim12);  // 双机通信
  HAL_TIM_Base_Start_IT(&htim13);  // 云台pid
  HAL_TIM_Base_Start_IT(&htim14);  // 摩擦轮pid

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    HAL_Delay(1);
    gimbalRun(&gimbal1, GimbalControlData.yawAngle, GimbalControlData.pitchAngle);
    GimbalChangeMode(&gimbal1, GimbalControlData.mode);
    shooterRun(&shooter1);
    tuneRun();
  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  /* User can add his own implementation to report the HAL error return state
   */
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
     number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line)
   */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
