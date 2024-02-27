#include "RadarConnect.h"

#include "BMI088.h"
#include "IMU.h"
#include "usart.h"

void RadarConnectINIT(RadarConnect *connect, UART_HandleTypeDef *huart) {
  connect->huart = huart;
  connect->RX.data.head = 0;
  connect->RX.data.speedYAW = 0;
  connect->RX.data.speedX = 0;
  connect->RX.data.speedY = 0;
  connect->gimbalYaw = 0;
  HAL_UARTEx_ReceiveToIdle_DMA(huart, connect->RXBuf, sizeof(connect->RXBuf));
}

void RadarConnect_EventCallBack(RadarConnect *connect, UART_HandleTypeDef *huart) {
  if (huart == connect->huart) {
    if (connect->RXBuf[0] == 0xA5) {
      connect->gimbalYaw = BMI088_gimbal.yawAngle;
      for (uint8_t i = 0; i < sizeof(connect->RXBuf); i++) {
        connect->RX.rawRX[i] = connect->RXBuf[i];
      }
    }
  }
  HAL_UARTEx_ReceiveToIdle_DMA(huart, connect->RXBuf, sizeof(connect->RXBuf));
}

void RadarConnect_RxCpltCallBack(RadarConnect *connect, UART_HandleTypeDef *huart) {

  if (huart == connect->huart) {
    HAL_UARTEx_ReceiveToIdle_DMA(huart, connect->RXBuf, sizeof(connect->RXBuf));
  }
}

void RadarConnect_ErrorCallBack(RadarConnect *connect, UART_HandleTypeDef *huart) {
  if (huart == connect->huart) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UNLOCK(huart);
    HAL_UARTEx_ReceiveToIdle_DMA(huart, connect->RXBuf, sizeof(connect->RXBuf));
  }
}
