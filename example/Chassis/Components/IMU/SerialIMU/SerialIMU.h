#ifndef SERIALIMU_H
#define SERIALIMU_H

#include "main.h"
#include "struct_typedef.h"

typedef struct imudata {
  struct {
    int16_t raw;
    float rad, ang;
  } rol, yaw, pit, yawSpeed;

  UART_HandleTypeDef *huart;
  uint8_t IMU_RxBUFF[25];
} SerialIMU;

// extern SerialIMU imu_gimbal;
extern void IMU_INIT(SerialIMU *IMU, UART_HandleTypeDef *huart);
extern void IMU_EventCallBack(SerialIMU *IMU, UART_HandleTypeDef *huart);
extern void IMU_RxCpltCallBack(SerialIMU *IMU, UART_HandleTypeDef *huart);
extern void IMU_ErrorCallBack(SerialIMU *IMU, UART_HandleTypeDef *huart);

#endif
