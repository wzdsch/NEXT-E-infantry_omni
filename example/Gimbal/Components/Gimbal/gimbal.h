#ifndef __GIMBAL_H
#define __GIMBAL_H
#include "DJI_Motor.h"

enum GIMBAL_MODE {
  GIMBAL_STOP = 0,  // 云台无力
  GIMBAL_PITCH_EN,  // 仅pitch有力
  GIMBAL_YAW_EN,    // 仅yaw有力
  GIMBAL_ALL_EN,    // 云台全有力
};

typedef struct gimbalData {

  DJI_MotorGroup *gimbalGroup;
  DJI_Motor *yawMotor;
  DJI_Motor *pitchMotor;
  uint8_t mode;
} gimbal;

extern void gimbalINIT(gimbal *gimbal, DJI_MotorGroup *group, DJI_Motor *yawMotor,
                       DJI_Motor *pitchMotor);
extern void gimbalRun(gimbal *gimbal, fp32 yaw, fp32 pitch);
extern void GimbalChangeMode(gimbal *gimbal, uint8_t mode);

extern gimbal gimbal1;

#endif
