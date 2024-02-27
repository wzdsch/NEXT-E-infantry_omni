#ifndef BMI088_H
#define BMI088_H
#include "struct_typedef.h"

typedef struct {
  fp32 yawAngle;
  fp32 yawTotalAngle;
  fp32 pitchAngle;
  fp32 rollAngle;
  fp32 yawSpeed;
  fp32 pitchSpeed;
  fp32 rollSpeed;
} BMI088_IMU;

extern BMI088_IMU BMI088_gimbal;

extern void BMI088_INIT(BMI088_IMU *IMU);
extern void BMI088_RUN(BMI088_IMU *IMU);
#endif


