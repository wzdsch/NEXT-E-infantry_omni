/**
 * @File Name: addOns.c
 * @brief  此文件存放电机计算的预处理和后处理函数，函数格式统一
 *
 *
 * @Version : 1.0
 *
 * Date:2023.2023-05-18        Author:Almost_Noob email:luodanwei03@outlook.com
 * 修改计划(完成末尾写1):
 */
#include "addOns.h"

#include "DJI_Motor.h"
#include "struct_typedef.h"
// #include "referee.h"
#include "BMI088.h"
#include "BPnet.h"

fp32 MIUecdZeroCrossing(DJI_Motor *motor) {
  fp32 temp = 0;
  if (BMI088_gimbal.yawAngle >= motor->flagEcd) {
    temp = (BMI088_gimbal.yawAngle) - (motor->flagEcd);
  }
  else {
    temp = (BMI088_gimbal.yawAngle) - (motor->flagEcd) + 360.0f;
  }

  fp32 offset = 180.0f;  // 半圈机械角度
  if ((motor->target) - temp > offset) {
    temp = temp + (offset * 2);
    return temp;
  }
  else if ((motor->target) - temp <= -offset) {
    temp = temp - (offset * 2);
    return temp;
  }
  return temp;
}

fp32 yawMotorpostProcess(DJI_Motor *motor) {
  fp32 temp = 0;
  if (motor->pidOutput1 > 0) {
    temp = (motor->pidOutput1) + 2700.0f;
  }
  else if (motor->pidOutput1 < 0) {
    temp = (motor->pidOutput1) - 2700.0f;
  }
  if (temp > 30000.0f) {
    temp = 30000.0f;
  }
  if (temp < -30000.0f) {
    temp = -30000.0f;
  }
  return temp;
}

/**
 * @brief  电机编码器过零函数
 * @param  motor: 电机结构体
 * @return fp32: 过零处理后的编码
 */
fp32 ecdZeroCrossing(DJI_Motor *motor) {
  fp32 offset = 4096;  // 半圈机械角度
  fp32 temp;
  fp32 softEcd = DJI_MotorGetSoftEcd(motor);

  if ((motor->target) - softEcd > offset) {
    temp = softEcd + (offset * 2);
  }
  else if ((motor->target) - softEcd <= -offset) {
    temp = softEcd - (offset * 2);
  }
  else {
    temp = softEcd;
  }
  return temp;
  // return softEcd;
}

/**
 * @brief  底盘跟随云台过零函数
 * @param  motor:
 * @return fp32:
 */
fp32 chassisFollowZeroCrossing(DJI_Motor *motor) {
  fp32 temp = 0;
  if (motor->realEcd > 4096) {
    fp32 temp = motor->realEcd - 8192;
    return temp;
  }
  else {
    temp = motor->realEcd;
    return temp;
  }
}

/*
fp32 powerlimit(DJI_Motor *motor){
  fp32 temp=(power_heat_data_t.chassis_power_buffer)/60.0f;
  return (motor->pidOutput0*temp);
}
*/

fp32 example(DJI_Motor *motor) {
  return motor->pidOutput0 / 2;
}

fp32 BPpidSpeed(DJI_Motor *motor) {
  BPforward(&BPnet1);
  BPbackward(&BPnet1);
  return BPnet1.pidout;
}
