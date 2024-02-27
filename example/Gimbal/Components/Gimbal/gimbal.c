/**
 * @File Name: gimbal.c
 * @brief
 *
 *
 * @Version : 1.0
 *
 * Date:2023.2023-04-04        Author:Almost_Noob email:luodanwei03@outlook.com
 * 修改计划(完成末尾写1):
 */
#include "gimbal.h"

#include "DJI_Motor.h"
#include "MCUConnect.h"
#include "addOns.h"

/**
 * @brief  云台结构体初始化
 * @param  gimbal:
 * @param  gimbalGroup:
 */
void gimbalINIT(gimbal *gimbal, DJI_MotorGroup *group, DJI_Motor *yawMotor, DJI_Motor *pitchMotor) {
  gimbal->gimbalGroup = group;
  gimbal->yawMotor = yawMotor;
  gimbal->pitchMotor = pitchMotor;
  gimbal->mode = GIMBAL_STOP;
}

void gimbalRun(gimbal *gimbal, fp32 yaw, fp32 pitch) {
  switch (gimbal->mode) {
    case GIMBAL_STOP:
      DJI_MotorDisable(gimbal->yawMotor);
      DJI_MotorDisable(gimbal->pitchMotor);
      break;
    case GIMBAL_PITCH_EN:
      DJI_MotorDisable(gimbal->yawMotor);
      DJI_MotorEnable(gimbal->yawMotor);
      DJI_MotorSetTarget(gimbal->pitchMotor, pitch);
      break;
    case GIMBAL_YAW_EN:
      DJI_MotorDisable(gimbal->pitchMotor);
      DJI_MotorEnable(gimbal->yawMotor);
      DJI_MotorSetTarget(gimbal->yawMotor, yaw);
      break;
    case GIMBAL_ALL_EN:
      DJI_MotorEnable(gimbal->yawMotor);
      DJI_MotorEnable(gimbal->pitchMotor);
      DJI_MotorSetTarget(gimbal->pitchMotor, pitch);
      DJI_MotorSetTarget(gimbal->yawMotor, yaw);
      break;
  }
}

void GimbalChangeMode(gimbal *gimbal, uint8_t mode) {
  gimbal->mode = mode;
}
