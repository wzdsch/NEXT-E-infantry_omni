#include "BMI088.h"

#include "BMI088Middleware.h"
#include "BMI088driver.h"
#include "BMI_IMU.h"
#include "bsp_dwt.h"
#include "main.h"
#include "spi.h"

void BMI088_INIT(BMI088_IMU *IMU) {
  DWT_Init(168);
  while (BMI088_init(&hspi1, 1) != BMI088_NO_ERROR)
    ;
  INS_Init();
  INS.IMU = IMU;
}

void BMI088_RUN(BMI088_IMU *IMU) {
  INS_Task();
}
