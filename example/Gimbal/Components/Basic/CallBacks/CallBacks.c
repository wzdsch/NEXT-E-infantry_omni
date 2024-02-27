#include "CallBacks.h"

#include "MCUConnect.h"
#include "MCUConnectStructs.h"
#include "gimbal.h"
#include "main.h"
#include "remote_control.h"
#include "shooter.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim11) {  // 双击通信解包
    connectionRcceiveData(&connect);
  }
  if (htim == &htim12) {  // 双机通信发送2ms@500HZ
    connectionSendData(&connect, (uint8_t *)&ChassisControlData, sizeof(ChassisControlData),
                       ChassisControlData_ID);
  }
  if (htim == &htim13) {  // 云台任务2ms@500Hz
    DJI_MotorPidRUN(gimbal1.gimbalGroup);
    DJI_MotorSendData(gimbal1.gimbalGroup);
  }
  if (htim == &htim14) {  // 发射机构任务2ms@500Hz
    DJI_MotorPidRUN(shooter1.shooterGroup);
    DJI_MotorSendData(shooter1.shooterGroup);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  RC_ReceiveCpltCallback(&rc_ctrl, huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  RC_ErrorCallback(&rc_ctrl, huart);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  RC_EventCallback(&rc_ctrl, huart);
}

// can中断0
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (hcan == &hcan1) {
    DJI_MotorReceiveMotordat(&group1);
  }
  if (hcan == &hcan2) {
    DJI_MotorReceiveMotordat(&group2);
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
