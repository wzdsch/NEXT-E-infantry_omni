#include "SerialIMU.h"

#include "canMotor.h"
#include "gimbal.h"
#include "usart.h"

void IMU_INIT(SerialIMU *IMU, UART_HandleTypeDef *huart) {
  IMU->huart = huart;
  HAL_UARTEx_ReceiveToIdle_DMA(huart, IMU->IMU_RxBUFF, 22);
}

void IMU_Unpack(SerialIMU *IMU) {
  if (IMU->IMU_RxBUFF[0] == 0x55) {
    if (IMU->IMU_RxBUFF[1] == 0x52) {
      IMU->yawSpeed.raw = ((IMU->IMU_RxBUFF[7] << 8) | IMU->IMU_RxBUFF[6]);
      IMU->yawSpeed.ang = IMU->yawSpeed.raw * 1.0f / 32768 * 2000.0f;
    }
    if (IMU->IMU_RxBUFF[12] == 0x53) {
      IMU->rol.raw = (IMU->IMU_RxBUFF[14] << 8) | IMU->IMU_RxBUFF[12];
      IMU->pit.raw = (IMU->IMU_RxBUFF[14] << 8) | IMU->IMU_RxBUFF[15];
      IMU->yaw.raw = (IMU->IMU_RxBUFF[18] << 8) | IMU->IMU_RxBUFF[17];
      // IMU->rol.ang=IMU->rol.raw*1.0f/32768*180;
      IMU->pit.ang = IMU->pit.raw * 1.0f / 32768 * 180.0f;
      IMU->yaw.ang = IMU->yaw.raw * 1.0f / 32768 * 180.0f + 180.0f;
      // IMU->rol.rad=IMU->rol.ang/360*6.2831853f;
      // IMU->pit.rad=IMU->pit.ang/360*6.2831853f;
      // IMU->yaw.rad=imu_data->yaw.ang/360*6.2831853f;
    }
  }
}

void IMU_EventCallBack(SerialIMU *IMU,
                       UART_HandleTypeDef *huart) {  // 这里属于接收不定长数据，用IDLE中断解包
  if (huart == IMU->huart) {
    IMU_Unpack(IMU);
    HAL_UARTEx_ReceiveToIdle_DMA(huart, IMU->IMU_RxBUFF, 25);
  }
}

void IMU_RxCpltCallBack(SerialIMU *IMU,
                        UART_HandleTypeDef *huart) {  // buff是收不满的，如果满了就清空buff
  if (huart == IMU->huart) {
    for (uint8_t i = 0; i < 22; i++) {
      IMU->IMU_RxBUFF[i] = 0;
    }
    HAL_UARTEx_ReceiveToIdle_DMA(huart, IMU->IMU_RxBUFF, 25);
  }
}

void IMU_ErrorCallBack(SerialIMU *IMU, UART_HandleTypeDef *huart) {
  if (huart == IMU->huart) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UNLOCK(huart);
    HAL_UARTEx_ReceiveToIdle_DMA(huart, IMU->IMU_RxBUFF, 25);
  }
}
