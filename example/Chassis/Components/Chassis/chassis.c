
/**
 * @File Name: chassis.c
 * @brief  底盘解算代码
 *
 *
 * @Version : 1.0
 *
 * Date:2023.2023-03-20        Author:Almost_Noob email:luodanwei03@outlook.com
 * 修改计划(完成末尾写1):
 */
#include "chassis.h"

#include <stdlib.h>

#include "DJI_Motor.h"
#include "MCUConnect.h"
#include "Tools.h"
#include "math.h"
#include "pid.h"
#include "pidData.h"
#include "remote_control.h"
#include "BMI088.h"
#include "addOns.h"

extern BMI088_IMU BMI088_chassis;

#if IF_WITH_SUPERCAP == 1
#include "SuperCap.h"
#endif

extern uint8_t referee_rx_flg;

#if IF_WITH_SUPERCAP == 1
extern uint8_t supercap_rx_flg;
#endif

fp32 ave_spd_z = 0;
int top_spd_z = 3000;
fp32 correction_rate = -2.0f;
fp32 correction_val = 150.0f;

int top_time_cnt = 0; // 变速小陀螺计数
#define TOP_T 1000 // 变速小陀螺计数周期


/**
 * @brief  底盘结构体初始化
 * @param  chassis:
 * @param  group:
 */
void chassisINIT(chassis* chassis, DJI_MotorGroup* group, DJI_Motor* chassisMotor1,
                 DJI_Motor* chassisMotor2, DJI_Motor* chassisMotor3, DJI_Motor* chassisMotor4,
                 DJI_Motor* gimbalMotor) {
  chassis->mode = CHASSIS_DISABLE;
  chassis->group = group;
  chassis->chassisMotor1 = chassisMotor1;
  chassis->chassisMotor2 = chassisMotor2;
  chassis->chassisMotor3 = chassisMotor3;
  chassis->chassisMotor4 = chassisMotor4;
  chassis->gimbalMotor = gimbalMotor;
  chassis->followEN = 1;
}

/**
 * @brief  底盘跟随初始化
 * @param  chassis: 底盘结构体
 * @param  flagEcd: 底盘跟随软零点
 * @param  pid0Mode: 角度外环的模式
 * @param  pid0Datas: 角度外环的参数
 * @param  pid1Mode: 速递内环的模式
 * @param  pid1Datas: 速度内环的参数
 */
void chasisFollowINIT(chassis* chassis, uint16_t flagEcd, uint8_t pid0Mode, fp32 pid0Datas[5],
                      uint8_t pid1Mode, fp32 pid1Datas[5]) {
  chassis->followFlagEcd = flagEcd;
  if (pid0Datas != NULL) {
    pidINIT(&(chassis->followPid0), pid0Mode, pid0Datas[0], pid0Datas[1], pid0Datas[2],
            pid0Datas[3], pid0Datas[4]);
  }
  if (pid1Datas != NULL) {
    pidINIT(&(chassis->followPid1), pid1Mode, pid1Datas[0], pid1Datas[1], pid1Datas[2],
            pid1Datas[3], pid1Datas[4]);
  }
}

/**
 * @brief  设置底盘跟随的结果
 *
 * @param chassis 底盘结构体地址
 * @param result 结果的地址
 */
void followResultSet(chassis* chassis, fp32* result) {
  chassis->follwoResult = result;
}

/**
 * @brief  使能底盘跟随
 * @param  chassis:
 */
void chassisFollowEnable(chassis* chassis) {
  if (chassis->followEN == 0) {
    PID_clear(&(chassis->followPid0));  // 清除pid
    PID_clear(&(chassis->followPid1));
  }
  chassis->followEN = 1;
}

/**
 * @brief  失能底盘跟随
 * @param  chassis:
 */
void chassisFollowDisable(chassis* chassis) {
  chassis->followEN = 0;
}

/**
 * @brief  改变底盘模式
 * @param  chassis:
 * @param  mode:
 */
void chassisChangeMode(chassis* chassis, uint8_t mode) {
  chassis->mode = mode;
}

/// @brief 从底盘陀螺仪解算出底盘欧拉角
/// @param chassis 
/// @param imu 
void get_chassis_euler(chassis* chassis, BMI088_IMU* imu) {
  chassis->chassis_pitch = atan(((fp32)tan(imu->pitchAngle) + (fp32)tan(imu->rollAngle)) / 1.414f);
  chassis->chassis_roll = atan(((fp32)tan(imu->pitchAngle) - (fp32)tan(imu->rollAngle)) / 1.414f);
  chassis->chassis_yaw = imu->yawAngle;
}

void chassisFollowRun(chassis* chassis) {
  if (chassis->followEN == 1) {  // 如果底盘跟随使能
    fp32 temp;	// 相对角度
    if (chassis->gimbalMotor->realEcd >= chassis->followFlagEcd) {  // 得到编码值
      temp = (chassis->gimbalMotor->realEcd) - (chassis->followFlagEcd);
    }
    else {
      temp = (chassis->gimbalMotor->realEcd) - (chassis->followFlagEcd) + 8192;
    }
    // 过零处理
    if (temp < -4096) {
      temp = temp + (4096 * 2);
    }
    else if (temp >= 4096) {
      temp = temp - (4096 * 2);
    }
    // 计算pid
    chassis->followPidout = PID_calc(&(chassis->followPid1), chassis->gimbalMotor->realSpeedF, PID_calc(&(chassis->followPid0), -temp, 0));
  }
}

fp32 real_chassis_xy_spd = 0;
fp32 real_chassis_xy_spd_err = 0;
fp32 real_gim_x = 0;
fp32 real_gim_y = 0;

fp32 real_chassis_x = 0;
fp32 real_chassis_y = 0;
fp32 real_chassis_z = 0;

/**
 * @brief  底盘解算
 *          电机id要求与运动方向示意：
 *                                   ^chassis_y
 *                   \       ^gim_y  /
 *                    \      |      /   <-z
 *                    m2           m1     |旋转正方向
 *                -----     yaw     ----->gim_x
 *                    m3           m4
 *                   /       |       \
 *                  /        |         \chassis_x
 *                                      >
 * @param  chassis:底盘结构体
 * @param  gim_x: x轴速度
 * @param  gim_y: y轴速度
 * @param  z: z轴速度
 */
 
 int16_t yaw_ecd = 0;
void chassisRun(chassis* chassis, fp32 gim_x, fp32 gim_y, fp32 z, int16_t yaw_err_ecd) {
  // 从yaw电机数据得到实际角度
  if (yaw_err_ecd >= chassis->followFlagEcd) {
    yaw_err_ecd = yaw_err_ecd - chassis->followFlagEcd;
  }
  else {
    yaw_err_ecd = yaw_err_ecd - chassis->followFlagEcd + 8192;
  }
  yaw_err_ecd = 8192 - yaw_err_ecd; // yaw轴电机反装，对角度做处理

  // 根据chassis_y与云台指向的夹角计算temp，从云台坐标系(gim_y-gim_x)转换为底盘坐标系(chassis_y-chassis_x)
  // 然后根据轮子位置合成各轮子的转速
	
  fp32 xAngle = 0;
  fp32 yAngle = 0;
  fp32 speed_x = 0;
  fp32 speed_y = 0;

  fp32 max_spd = 0;
  fp32 spd_adj = 1.0f;

  static fp32 current_x = 0;
  static fp32 current_y = 0;
  static fp32 current_z = 0;

  // 对云台发来的速度进行限幅
  if (my_fabs(gim_x) > 1.414f * SPEED_LIMIT || my_fabs(gim_y) > 1.414f * SPEED_LIMIT) {
    fp32 gim_spd_adj = SPEED_LIMIT * 1.414f / (my_fabs(gim_x) > my_fabs(gim_y) ? my_fabs(gim_x) : my_fabs(gim_y));
    gim_x *= gim_spd_adj;
    gim_y *= gim_spd_adj;
  }

  // 底盘坐标系真实速度计算
  real_chassis_x = (chassis->chassisMotor1->realSpeedF - chassis->chassisMotor3->realSpeedF) / 2.0f;
  real_chassis_y = (chassis->chassisMotor2->realSpeedF - chassis->chassisMotor4->realSpeedF) / 2.0f;
  real_chassis_z = (chassis->chassisMotor1->realSpeedF + chassis->chassisMotor2->realSpeedF\
    + chassis->chassisMotor3->realSpeedF + chassis->chassisMotor4->realSpeedF) / 4.0f;

    // 云台坐标系真实速度计算
  real_gim_x = real_chassis_x * cos(PI / 4.0f - yaw_err_ecd * PI / 4096.0f) + \
    real_chassis_y * cos(PI / 4.0f + yaw_err_ecd * PI / 4096.0f);
  
  real_gim_y = real_chassis_y * cos(PI / 4.0f - yaw_err_ecd * PI / 4096.0f) - \
    real_chassis_x * cos(PI / 4.0f + yaw_err_ecd * PI / 4096.0f);


//   if ((my_fabs(chassis->chassisMotor1->realSpeedF) > my_fabs(chassis->chassisMotor1->target) * UPDATE_SPD_RATE - UPDATE_SPD_ERR \
//     && my_fabs(chassis->chassisMotor2->realSpeedF) > my_fabs(chassis->chassisMotor2->target) * UPDATE_SPD_RATE - UPDATE_SPD_ERR \
//     && my_fabs(chassis->chassisMotor3->realSpeedF) > my_fabs(chassis->chassisMotor3->target) * UPDATE_SPD_RATE - UPDATE_SPD_ERR \
//     && my_fabs(chassis->chassisMotor4->realSpeedF) > my_fabs(chassis->chassisMotor4->target) * UPDATE_SPD_RATE - UPDATE_SPD_ERR)
//     || my_fabs(gim_x) < my_fabs(current_x)) {
     current_x = rampPlanner(real_gim_x, gim_x, MOTOR_SPD_UP_RATE, MOTOR_SPD_DOWN_RATE);
    // }
//   if ((my_fabs(chassis->chassisMotor1->realSpeedF) > my_fabs(chassis->chassisMotor1->target) * UPDATE_SPD_RATE - UPDATE_SPD_ERR \
//     && my_fabs(chassis->chassisMotor2->realSpeedF) > my_fabs(chassis->chassisMotor2->target) * UPDATE_SPD_RATE - UPDATE_SPD_ERR \
//     && my_fabs(chassis->chassisMotor3->realSpeedF) > my_fabs(chassis->chassisMotor3->target) * UPDATE_SPD_RATE - UPDATE_SPD_ERR \
//     && my_fabs(chassis->chassisMotor4->realSpeedF) > my_fabs(chassis->chassisMotor4->target) * UPDATE_SPD_RATE - UPDATE_SPD_ERR)
//     || my_fabs(gim_y) < my_fabs(current_y)) {
     current_y = rampPlanner(real_gim_y, gim_y, MOTOR_SPD_UP_RATE, MOTOR_SPD_DOWN_RATE);
    // }

  // x, y 速度解算
  yAngle = ((yaw_err_ecd + 1024) / 8192.0f) * 2 * PI;  // 取得y轴与行进方向的夹角，(从yaw电机的编码得出)
  xAngle = -yAngle;                          // 进而得到x与chassis_x的夹角
  // 根据夹角对速速度向量做坐标系转换
  speed_x = cos(yAngle + PI / 2) * current_y + cos(xAngle) * current_x;
  speed_y = cos(xAngle + PI / 2) * current_x + cos(yAngle) * current_y;

  fp32 spd_1 = speed_x;
  fp32 spd_2 = speed_y;
  fp32 spd_3 = -speed_x;
  fp32 spd_4 = -speed_y;
  

  // real_chassis_xy_spd = sqrt(real_chassis_x * real_chassis_x + real_chassis_y * real_chassis_y); // 实际底盘xy速度
  // static fp32 real_chassis_xy_spd_buf[2] = {0};
  // real_chassis_xy_spd_buf[1] = real_chassis_xy_spd_buf[0];
  // real_chassis_xy_spd_buf[0] = real_chassis_xy_spd;
  // real_chassis_xy_spd_err = real_chassis_xy_spd_buf[0] - real_chassis_xy_spd_buf[1];

  switch (chassis->mode) {
    case CHASSIS_DISABLE:  // 底盘失能
      DJI_MotorDisable(chassis->chassisMotor1);
      DJI_MotorDisable(chassis->chassisMotor2);
      DJI_MotorDisable(chassis->chassisMotor3);
      DJI_MotorDisable(chassis->chassisMotor4);
      chassisFollowDisable(chassis);
      break;

    case CHASSIS_STOP:  // 底盘有力停止，不咋用
      DJI_MotorEnable(chassis->chassisMotor1);
      DJI_MotorEnable(chassis->chassisMotor2);
      DJI_MotorEnable(chassis->chassisMotor3);
      DJI_MotorEnable(chassis->chassisMotor4);
      chassisFollowDisable(chassis);

      DJI_MotorSetTarget((chassis->chassisMotor1), 0);
      DJI_MotorSetTarget((chassis->chassisMotor2), 0);
      DJI_MotorSetTarget((chassis->chassisMotor3), 0);
      DJI_MotorSetTarget((chassis->chassisMotor4), 0);

      break;

    case CHASSIS_FREE:  // 底盘不跟随
      DJI_MotorEnable(chassis->chassisMotor1);
      DJI_MotorEnable(chassis->chassisMotor2);
      DJI_MotorEnable(chassis->chassisMotor3);
      DJI_MotorEnable(chassis->chassisMotor4);
      chassisFollowDisable(chassis);

      z = 0;

      current_z = rampPlanner(current_z, z, MOTOR_SPD_UP_RATE, MOTOR_SPD_DOWN_RATE);

      max_spd = my_fabs(my_fabs(spd_1) > my_fabs(spd_2) ? spd_1 : spd_2);
      max_spd = my_fabs(max_spd > my_fabs(spd_3) ? max_spd : spd_3);
      max_spd = my_fabs(max_spd > my_fabs(spd_4) ? max_spd : spd_4);

      if (max_spd > SPEED_LIMIT) {
        spd_adj = SPEED_LIMIT / max_spd; // 这里max_spd一定大于0
        spd_1 *= spd_adj;
        spd_2 *= spd_adj;
        spd_3 *= spd_adj;
        spd_4 *= spd_adj;
      }

      DJI_MotorSetTarget((chassis->chassisMotor1), spd_1);
      DJI_MotorSetTarget((chassis->chassisMotor2), spd_2);
      DJI_MotorSetTarget((chassis->chassisMotor3), spd_3);
      DJI_MotorSetTarget((chassis->chassisMotor4), spd_4);
      break;

    case CHASSIS_FOLLOW:  // 底盘跟随云台
      DJI_MotorEnable(chassis->chassisMotor1);
      DJI_MotorEnable(chassis->chassisMotor2);
      DJI_MotorEnable(chassis->chassisMotor3);
      DJI_MotorEnable(chassis->chassisMotor4);
      chassisFollowEnable(chassis);

      chassisFollowRun(chassis);  // 底盘跟随pid计算

      z = *(chassis->follwoResult);

      current_z = rampPlanner(current_z, z, MOTOR_SPD_UP_RATE, MOTOR_SPD_DOWN_RATE);

      spd_1 += current_z;
      spd_2 += current_z;
      spd_3 += current_z;
      spd_4 += current_z;

      if (max_spd > SPEED_LIMIT) {
        spd_adj = SPEED_LIMIT / max_spd; // 这里max_spd一定大于0
        spd_1 *= spd_adj;
        spd_2 *= spd_adj;
        spd_3 *= spd_adj;
        spd_4 *= spd_adj;
      }

      DJI_MotorSetTarget((chassis->chassisMotor1), spd_1);
      DJI_MotorSetTarget((chassis->chassisMotor2), spd_2);
      DJI_MotorSetTarget((chassis->chassisMotor3), spd_3);
      DJI_MotorSetTarget((chassis->chassisMotor4), spd_4);
      break;

    case CHASSIS_TOP:  // 小陀螺（也就是以云台指向为行进y轴正方向，然后加z）
      DJI_MotorEnable(chassis->chassisMotor1);
      DJI_MotorEnable(chassis->chassisMotor2);
      DJI_MotorEnable(chassis->chassisMotor3);
      DJI_MotorEnable(chassis->chassisMotor4);
      DJI_MotorDisable(chassis->gimbalMotor);

      yAngle = ((yaw_err_ecd + 1024.0f - 3.0f * chassis->gimbalMotor->realSpeedF + 200.0f) / 8192.0f) * 2 * PI;  // 取得y轴与行进方向的夹角，(从yaw电机的编码得出)
      xAngle = -yAngle;                          // 进而得到x与chassis_x的夹角
      // 根据夹角对速速度向量做坐标系转换
      speed_x = cos(yAngle + PI / 2) * current_y + cos(xAngle) * current_x;
      speed_y = cos(xAngle + PI / 2) * current_x + cos(yAngle) * current_y;

      spd_1 = speed_x;
      spd_2 = speed_y;
      spd_3 = -speed_x;
      spd_4 = -speed_y;

      top_time_cnt++;
      top_time_cnt %= TOP_T;

      // fp32 min_top_spd = 3000;  // TOP_SPD_SCALE * sqrtf(gim_x * gim_x + gim_y * gim_y);

      // top_spd_z = 1.9996f * SPEED_LIMIT - sqrtf(gim_x * gim_x + gim_y * gim_y);
      // top_spd_z = top_spd_z > min_top_spd ? top_spd_z : min_top_spd;

      top_spd_z = 3000;

      current_z = rampPlanner(current_z, top_spd_z, MOTOR_SPD_UP_RATE, MOTOR_SPD_DOWN_RATE);

      spd_1 += current_z;
      spd_2 += current_z;
      spd_3 += current_z;
      spd_4 += current_z;

      if (max_spd > SPEED_LIMIT) {
        spd_adj = SPEED_LIMIT / max_spd; // 这里max_spd一定大于0
        spd_1 *= spd_adj;
        spd_2 *= spd_adj;
        spd_3 *= spd_adj;
        spd_4 *= spd_adj;
      }

      DJI_MotorSetTarget((chassis->chassisMotor1), spd_1);
      DJI_MotorSetTarget((chassis->chassisMotor2), spd_2);
      DJI_MotorSetTarget((chassis->chassisMotor3), spd_3);
      DJI_MotorSetTarget((chassis->chassisMotor4), spd_4);
      break;
  }
}

fp32 G = 0; // 重力
/// @brief 底盘电机在功率限制下的前馈(放在后处理函数)
/// @param motor 
/// @return 
fp32 chassis_motor_feedforward_in_power_limit(DJI_Motor* motor) {
  if (motor->ID == PITCH_POSITIVE_WHELL_ID) {
    motor->pidOutput0 += G * sin(chassis1.chassis_pitch);
  }
  else if (motor->ID == PITCH_NEGATIVE_WHELL_ID) {
    motor->pidOutput0 -= G * sin(chassis1.chassis_pitch);
  }
  else if (motor->ID == ROLL_POSITIVE_WHELL_ID) {
    motor->pidOutput0 += G * sin(chassis1.chassis_roll);
  }
  else if (motor->ID == ROLL_NEGATIVE_WHELL_ID) {
    motor->pidOutput0 -= G * sin(chassis1.chassis_roll);
  }

  if (motor->pidOutput0 > motor->motorPid0.max_out) {
    motor->pidOutput0 = motor->motorPid0.max_out;
  }
  else if (motor->pidOutput0 < -motor->motorPid0.max_out) {
    motor->pidOutput0 = -motor->motorPid0.max_out;
  }

  return powerlimit_pro(motor);
}
