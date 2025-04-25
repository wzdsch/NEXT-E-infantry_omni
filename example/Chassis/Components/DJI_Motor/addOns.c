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
#include "math.h"
#include "referee.h"
#include "refereeData_v1.6.h"

//  #include "BMI088.h"
//  #include "LADRC.h"
#include "struct_typedef.h"

#ifdef BMI088_H

/**
 * @brief  陀螺仪过零
 * @param  motor:
 * @return fp32:
 */
fp32 IMUecdZeroCrossing(DJI_Motor *motor) {
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
#endif

#ifdef LADRC_H

fp32 LADRC_YawControl(DJI_Motor *motor) {
  return LADRC_O2_Loop(&LADRC_Yaw, &(motor->target), &(motor->preProcessResult));
}

#endif
/**
 * @brief  电机编码器过零函数
 * @param  motor: 电机结构体
 * @return fp32: 过零处理后的编码
 */
fp32 ecdZeroCrossing(DJI_Motor *motor) {
  fp32 offset = 4096;  // 半圈机械角度
  fp32 temp;
  fp32 softEcd = DJI_MotorGetSoftEcd(motor);  // 返回0-8191

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

#ifdef REFEREEDATA_H

#define WARNING_POWER_BUFF 50.0f

/**
 * @brief  官方功率限制逻辑
 * @param  motor: 电机结构体
 * @return fp32: 功率限制后的电流值
 */
fp32 powerlimit(DJI_Motor *motor) {
  fp32 chassis_power = power_heat_data.chassis_power;
  fp32 chassis_power_buffer = power_heat_data.buffer_energy;
  fp32 total_current_limit = 0.0f;
  static fp32 total_current = 0.0f;
  uint16_t WARNING_POWER = robot_state.chassis_power_limit / 2;
  uint16_t POWER_LIMIT = robot_state.chassis_power_limit;
  static uint8_t count = 0;
  static fp32 current_scale = 0.0f;
  fp32 BUFFER_TOTAL_CURRENT_LIMIT = 16000;
  fp32 POWER_TOTAL_CURRENT_LIMIT = 20000;

  // power > 80w and buffer < 60j, because buffer < 60 means power has been more
  // than 80w
  // 功率超过80w 和缓冲能量小于60j,因为缓冲能量小于60意味着功率超过80w
  if (chassis_power_buffer < WARNING_POWER_BUFF) {
    fp32 power_scale;
    if (chassis_power_buffer > 5.0f) {
      // scale down WARNING_POWER_BUFF
      // 缩小WARNING_POWER_BUFF
      power_scale = chassis_power_buffer / WARNING_POWER_BUFF;
    }
    else {
      // only left 10% of WARNING_POWER_BUFF
      power_scale = 5.0f / WARNING_POWER_BUFF;
    }
    // scale down
    // 缩小
    total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;
  }

  else {
    // power > WARNING_POWER
    // 功率大于WARNING_POWER
    if (chassis_power > WARNING_POWER) {
      fp32 power_scale;
      // power < 80w
      // 功率小于80w
      if (chassis_power < POWER_LIMIT) {
        // scale down
        // 缩小
        power_scale = (POWER_LIMIT - chassis_power) / (POWER_LIMIT - WARNING_POWER);
      }
      // power > 80w
      // 功率大于80w
      else {
        power_scale = 0.0f;
      }
      total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
    }
    // power < WARNING_POWER
    // 功率小于WARNING_POWER
    else {
      total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
    }
  }

  // calculate the original motor current set
  // 计算原本电机电流设定

  total_current += fabs(motor->pidOutput0);
  count++;

  if (count == 4) {
    count = 0;
    if (total_current > total_current_limit) {
      current_scale = total_current_limit / total_current;
    }
    else {
      current_scale = 1.0f;
    }
    total_current = 0;
  }

  return (motor->pidOutput0) * current_scale;
}



/**
 * @brief  西交利物浦大学功率限制移植
 * @param  motor: 电机结构体
 * @return fp32: 功率限制后的电流值
 */  fp32 constant = 4.081f;
float chassis_total_power = 0;
fp32 powerlimit_pro(DJI_Motor *motor) {
  static uint8_t pidsetCount = 0;
  static pids buffer_pid;
  if (pidsetCount == 0) {
    pidINIT(&buffer_pid, PID_POSITION, 1.5, 0, 0, 30, 3);
    pidsetCount++;
  }
  // fp32 buffer_pid_out = 0;
  uint16_t max_power_limit = robot_state.chassis_power_limit;
  fp32 chassis_max_power = 0;
  float input_power = 0;         // input power from battery (referee system)
  float initial_give_power = 0;  // initial power from PID calculation
  static float initial_total_power = 0;
  static float initial_total_power_last = 0;
  static fp32 scaled_give_power = 0;

  fp32 toque_coefficient = 1.99688994e-6f;  // (20/16384)*(0.3)*(187/3591)/9.55
  fp32 a = 1.23e-07;                        // k1
  fp32 k2 = 1.453e-07;                      // k2


  // buffer_pid_out = PID_calc(&buffer_pid, power_heat_data.buffer_energy,
  //                           40);                      // 适配代码(自己写的)
  max_power_limit = robot_state.chassis_power_limit;  // 适配代码(自己写的)
  input_power = max_power_limit;
  chassis_max_power = input_power;
  static uint8_t count = 0;
  initial_give_power = motor->pidOutput0 * toque_coefficient * motor->realSpeed
                       + k2 * motor->realSpeed * motor->realSpeed
                       + a * motor->pidOutput0 * motor->pidOutput0 + constant;
  if (initial_give_power >= 0) {  // negative power not included (transitory)
    initial_total_power += initial_give_power;
  }
  count++;

  if (count == 4) {
    count = 0;
    initial_total_power_last = initial_total_power;
    chassis_total_power = initial_total_power; //
    initial_total_power = 0;
  }

  if (initial_total_power_last > chassis_max_power)  // determine if larger than max power
  {
    fp32 power_scale = chassis_max_power / initial_total_power_last;
    scaled_give_power = initial_give_power * power_scale;  // get scaled power
    if (scaled_give_power < 0) {
      return motor->pidOutput0;
    }
	
    // chassis_total_power = scaled_give_power; //
	
    fp32 b = toque_coefficient * motor->realSpeed;
    fp32 c = k2 * motor->realSpeed * motor->realSpeed - scaled_give_power + constant;

    if (motor->pidOutput0 > 0)  // Selection of the calculation formula according
                                // to the direction of the original moment
    {
      fp32 temp = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);////////////////////////////////////////////////////
      if (temp > 16000) {
        return 16000;
      }
      else
        return temp;
    }
    else {
      fp32 temp = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);////////////////////////////////////////////////////
      if (temp < -16000) {
        return -16000;
      }
      else
        return temp;
    }
  }
  else {
    return motor->pidOutput0;
  }
}

#endif

fp32 example(DJI_Motor *motor) {
  return motor->pidOutput0 / 2;
}
