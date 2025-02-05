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

fp32 helmTotalPower = 0;
fp32 buffer_pid_k = 4;

fp32 super_cup_energy = 0;

/**
 * @brief  西交利物浦大学功率限制移植
 * @param  motor: 电机结构体
 * @return fp32: 功率限制后的电流值
 */
fp32 buffer_pid_out = 0;

fp32 powerlimit_pro(DJI_Motor *motor) {
  int16_t energy = super_cup_energy;
  static uint8_t pidsetCount = 0;
  static pids buffer_pid;
  if (pidsetCount == 0) {  // 初始化缓存控制的pid
    pidINIT(&buffer_pid, PID_POSITION, buffer_pid_k, 0, 0, robot_state.chassis_power_limit, 3);
    pidsetCount++;
  }

  static fp32 initial_total_power = 0;       // 四个电机的总功率
  static fp32 initial_total_power_last = 0;  // 上次的总功率
  static fp32 scaled_give_power = 0;         // 放缩后的功率
  // uint16_t max_power_limit = robot_state.chassis_power_limit - helmTotalPower;  // 标称最大功率
  //  buffer_pid.max_out = robot_state.chassis_power_limit;  // 限制缓存pid的输出
  //  fp32 buffer_pid_out =
  //    PID_calc(&buffer_pid, power_heat_data.buffer_energy, 50);  // pid计算缓存机制可提供的功率

  /**
   * @brief  超电测试
   */
  uint16_t max_power_limit = robot_state.chassis_power_limit - helmTotalPower;  // 标称最大功率
  buffer_pid.max_out = 60;                             // 限制缓存pid的输出
                                                       // fp32
  buffer_pid_out = PID_calc(&buffer_pid, energy, 50);  // pid计算缓存机制可提供的功率

  /**
   * @brief  超电测试结束
   */

  fp32 input_power = max_power_limit - buffer_pid_out;  // 标称+缓存机制功率

  fp32 chassis_max_power = input_power;  // 实际可输入功率作为底盘最大功率

  // 计算电机模型的理论功率
  fp32 toque_coefficient = 1.99688994e-6f;  // (20/16384)*(0.3)*(187/3591)/9.55
  fp32 a = 1.23e-07;                        // k1
  fp32 k2 = 1.453e-07;                      // k2
  fp32 constant = 4.081f;

  static uint8_t count = 0;
  fp32 initial_give_power = motor->pidOutput0 * toque_coefficient * motor->realSpeed
                            + k2 * motor->realSpeed * motor->realSpeed
                            + a * motor->pidOutput0 * motor->pidOutput0 + constant;

  if (initial_give_power >= 0) {  // 累加四个电机的功率
    initial_total_power += initial_give_power;
  }
  count++;
  if (count == 4) {  // 得到四个电机的总功率后，更新值
    count = 0;
    initial_total_power_last = initial_total_power;
    initial_total_power = 0;
  }

  // 如果模型功率>底盘最大功率
  if (initial_total_power_last > chassis_max_power) {
    fp32 power_scale = chassis_max_power / initial_total_power_last;  // 计算超功率的比例
    scaled_give_power =
      initial_give_power * power_scale;  // 比例*单个电机的理论功率，将电机的输出功率减小
    if (scaled_give_power < 0) {
      return motor->pidOutput0;
    }
    // 用电机模型反解出在scaled_give_power下，应该给电机的电流值
    fp32 b = toque_coefficient * motor->realSpeed;
    fp32 c = k2 * motor->realSpeed * motor->realSpeed - scaled_give_power + constant;
    if (motor->pidOutput0 > 0) {
      fp32 temp = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
      if (temp > 16000) {
        return 16000;
      }
      else
        return temp;
    }
    else {
      fp32 temp = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
      if (temp < -16000) {
        return -16000;
      }
      else
        return temp;
    }
  }
  // 如果没超功率，就不做限制
  else {
    return motor->pidOutput0;
  }
}

/**
 * @brief  西交利物浦大学功率限制移植
 * @param  motor: 电机结构体
 * @return fp32: 功率限制后的电流值
 */

fp32 powerlimit_pro_helm(DJI_Motor *motor) {
  // uint16_t max_power_limit = 60;
  uint16_t max_power_limit = robot_state.chassis_power_limit;
  fp32 Current_Initial_Out = motor->pidOutput1;
  static fp32 initial_total_power = 0;
  static fp32 initial_total_power_last = 0;
  static fp32 scaled_give_power = 0;

  fp32 toque_coefficient = 1.42074505e-5f;  // 力矩电流系数，详见开源文档
  fp32 constant = 0;
  fp32 a = 1.23e-07;    // k1
  fp32 k2 = 1.453e-07;  // k2

  fp32 input_power = max_power_limit;
  fp32 chassis_max_power = input_power;
  static uint8_t count = 0;
  fp32 initial_give_power = Current_Initial_Out * toque_coefficient * motor->realSpeed
                            + k2 * motor->realSpeed * motor->realSpeed
                            + a * Current_Initial_Out * Current_Initial_Out + constant;

  if (initial_give_power >= 0) {  // negative power not included (transitory)
    initial_total_power += initial_give_power;
  }
  count++;

  if (count == 2) {  // 电机数量
    count = 0;
    initial_total_power_last = initial_total_power;
    helmTotalPower = initial_total_power;
    if (helmTotalPower > max_power_limit) {
      helmTotalPower = max_power_limit;
    }
    initial_total_power = 0;
  }

  if (initial_total_power_last > chassis_max_power) {
    fp32 power_scale = chassis_max_power / initial_total_power_last;
    scaled_give_power = initial_give_power * power_scale;
    if (scaled_give_power < 0) {
      return Current_Initial_Out;
    }

    fp32 b = toque_coefficient * motor->realSpeed;
    fp32 c = k2 * motor->realSpeed * motor->realSpeed - scaled_give_power + constant;
    if (Current_Initial_Out > 0) {
      fp32 temp = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
      if (temp > 16000) {
        return 16000;
      }
      else {
        return temp;
      }
    }
    else {
      fp32 temp = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
      if (temp < -16000) {
        return -16000;
      }
      else {
        return temp;
      }
    }
  }
  else {
    return Current_Initial_Out;
  }
}

#endif

fp32 example(DJI_Motor *motor) {
  return motor->pidOutput0 / 2;
}
