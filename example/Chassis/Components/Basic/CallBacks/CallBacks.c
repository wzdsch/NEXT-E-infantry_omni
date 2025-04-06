#include "CallBacks.h"

#include "MCUConnect.h"
#include "MCUConnectStructs.h"
#include "Tools.h"
#include "addOns.h"
#include "chassis.h"
#include "main.h"
#include "referee.h"
#include "refereeData_v1.6.h"
#include "struct_typedef.h"
#include "ui.h"
#include "vofa.h"

// #include "remote_control.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim10) {  // UI发送100ms@10Hz
    UI_Refresh();
    uint16_t powerlimit = robot_state.chassis_power_limit;
//    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&powerlimit, sizeof(powerlimit));
	  JustFloat(chassis1.chassisMotor1->realSpeedF, chassis1.chassisMotor2->realSpeedF, chassis1.chassisMotor3->realSpeedF, chassis1.chassisMotor4->realSpeedF, &huart1);
  }
  if (htim == &htim11) {  // 底盘pid计算与发送1ms@1000Hz
//    DJI_MotorPidRUN(chassis1.group);
//    DJI_MotorSendData(chassis1.group);
  }
  if (htim == &htim12) {  // 双机通信发送,裁判系统解包10ms@100Hz
    // referee_unpack(&referee1);
    referee_unpack_fifo_data();
    RefereeDataUpdate(&RefereeData);
    connectionSendData(&connect, (uint8_t *)&RefereeData, sizeof(RefereeData), RefereeData_ID);
  }
  if (htim == &htim13) {  // 双机通讯解包0.67ms@1500Hz
    connectionUnpackData(&connect);
  }
  if (htim == &htim14) {  // 底盘解算2ms@500Hz
    chassisRun(&chassis1, ChassisControlData.speedx, ChassisControlData.speedy,
               ChassisControlData.speedz, DJI_MotorGetSoftEcd(&motorYaw));
    chassisChangeMode(&chassis1, ChassisControlData.mode);
    DJI_MotorPidRUN(chassis1.group);
    DJI_MotorSendData(chassis1.group);
  }
}

uint16_t receive_count = 0;
uint16_t error_count = 0;

// 串口接收完成中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  //  RC_ReceiveCpltCallback(&rc_ctrl,
  //                         huart); // de16遥控串口接收完成,解包准备再次接收
  // refereeRxCpltCallBack(&referee1, huart);
  if (huart == &huart1) {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)&super_cup_energy, sizeof(super_cup_energy));
    receive_count++;
  }
}

// 串口错误中断
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  // RC_ErrorCallback(&rc_ctrl,huart);  // dr16遥控错误中断，清除中断标志位，再次接收
  if (huart == &huart1) {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)&super_cup_energy, sizeof(super_cup_energy));
    error_count++;
  }
}

// 串口事件中断
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  // RC_EventCallback(&rc_ctrl, huart);  //
  // dr16遥控事件中断，因为是接收定长数据，
  //  正常不会进这个中断。
  //  直接再次接收
  // refereeRxEventCallBack(&referee1, huart);
  if (huart == &huart1) {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)&super_cup_energy, sizeof(super_cup_energy));
    receive_count++;
  }
  // super_cup_energy = 0;
}

// can中断0
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (hcan == &hcan1) {
    DJI_MotorReceiveMotordat(&group2);  // 接收yaw轴电机的数据
  }
  if (hcan == &hcan2) {
    DJI_MotorReceiveMotordat(&group1);  // 接收底盘电机的数据
  }
}

// can中断1
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (hcan == &hcan1) {
    connectionRcceiveData(&connect);
  }
  if (hcan == &hcan2) {
  }
}
