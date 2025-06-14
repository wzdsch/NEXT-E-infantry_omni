
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

void chassisMotorSpdProcess(chassis* chassis, fp32 set_spd1, fp32 set_spd2, fp32 set_spd3, fp32 set_spd4) {
  chassis->motor_spd.real_spd1 = chassis->chassisMotor1->realSpeedF;
  chassis->motor_spd.real_spd2 = chassis->chassisMotor2->realSpeedF;
  chassis->motor_spd.real_spd3 = chassis->chassisMotor3->realSpeedF;
  chassis->motor_spd.real_spd4 = chassis->chassisMotor4->realSpeedF;

  chassis->motor_spd.set_spd1 = set_spd1;
  chassis->motor_spd.set_spd2 = set_spd2;
  chassis->motor_spd.set_spd3 = set_spd3;
  chassis->motor_spd.set_spd4 = set_spd4;

  fp32 max_set_spd = my_fabs(my_fabs(chassis->motor_spd.set_spd1) > my_fabs(chassis->motor_spd.set_spd2) ? \
    chassis->motor_spd.set_spd1 : chassis->motor_spd.set_spd2);
  max_set_spd = my_fabs(max_set_spd > my_fabs(chassis->motor_spd.set_spd3) ? max_set_spd : chassis->motor_spd.set_spd3);
  max_set_spd = my_fabs(max_set_spd > my_fabs(chassis->motor_spd.set_spd4) ? max_set_spd : chassis->motor_spd.set_spd4);

  if (max_set_spd > SPEED_LIMIT) {
    fp32 spd_adj = SPEED_LIMIT / max_set_spd; // 这里max_spd一定大于0
    chassis->motor_spd.set_spd1 *= spd_adj;
    chassis->motor_spd.set_spd2 *= spd_adj;
    chassis->motor_spd.set_spd3 *= spd_adj;
    chassis->motor_spd.set_spd4 *= spd_adj;
  }

  // 这个判断是为了保证在加速的过程中，等待所有电机的速度都跟上来，再继续加速
  if (my_fabs(chassis->motor_spd.real_spd1) >= (my_fabs(chassis->motor_spd.set_spd1) * UPDATE_SPD_RATE - UPDATE_SPD_ERR)
      && my_fabs(chassis->motor_spd.real_spd2) >= (my_fabs(chassis->motor_spd.set_spd2) * UPDATE_SPD_RATE - UPDATE_SPD_ERR)
      && my_fabs(chassis->motor_spd.real_spd3) >= (my_fabs(chassis->motor_spd.set_spd3) * UPDATE_SPD_RATE - UPDATE_SPD_ERR)
      && my_fabs(chassis->motor_spd.real_spd4) >= (my_fabs(chassis->motor_spd.set_spd4) * UPDATE_SPD_RATE - UPDATE_SPD_ERR)) {
    // 电机速度达到设定范围，开始进一步加速
    chassis->motor_spd.processed_set_spd1 = \
      rampPlanner(chassis->motor_spd.processed_set_spd1, chassis->motor_spd.set_spd1, MOTOR_SPD_UP_RATE, MOTOR_SPD_DOWN_RATE);
    chassis->motor_spd.processed_set_spd2 = \
      rampPlanner(chassis->motor_spd.processed_set_spd2, chassis->motor_spd.set_spd2, MOTOR_SPD_UP_RATE, MOTOR_SPD_DOWN_RATE);
    chassis->motor_spd.processed_set_spd3 = \
      rampPlanner(chassis->motor_spd.processed_set_spd3, chassis->motor_spd.set_spd3, MOTOR_SPD_UP_RATE, MOTOR_SPD_DOWN_RATE);
    chassis->motor_spd.processed_set_spd4 = \
      rampPlanner(chassis->motor_spd.processed_set_spd4, chassis->motor_spd.set_spd4, MOTOR_SPD_UP_RATE, MOTOR_SPD_DOWN_RATE);
  }
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
    if (0 - temp > 4096) {
      temp = temp + (4096 * 2);
    }
    else if (0 - temp <= -4096) {
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
void chassisRun(chassis* chassis, fp32 gim_x, fp32 gim_y, fp32 z, int16_t yaw_err_ecd) {
  // 根据chassis_y与云台指向的夹角计算temp，从云台坐标系(gim_y-gim_x)转换为底盘坐标系(chassis_y-chassis_x)
  // 然后根据轮子位置合成各轮子的转速

  fp32 xAngle = 0;
  fp32 yAngle = 0;
  fp32 speed_x = 0;
  fp32 speed_y = 0;

  // 对云台发来的速度进行限幅
  if (my_fabs(gim_x) > 1.414f * SPEED_LIMIT || my_fabs(gim_y) > 1.414f * SPEED_LIMIT) {
    fp32 gim_spd_adj = SPEED_LIMIT * 1.414f / (my_fabs(gim_x) > my_fabs(gim_y) ? my_fabs(gim_x) : my_fabs(gim_y));
    gim_x *= gim_spd_adj;
    gim_y *= gim_spd_adj;
  }

  // 从yaw电机数据得到实际角度
  if (yaw_err_ecd >= chassis->followFlagEcd) {
    yaw_err_ecd = yaw_err_ecd - chassis->followFlagEcd;
  }
  else {
    yaw_err_ecd = yaw_err_ecd - chassis->followFlagEcd + 8192;
  }
  yaw_err_ecd = 8192 - yaw_err_ecd; // yaw轴电机反装，对角度做处理

  yAngle = ((yaw_err_ecd + 1024) / 8192.0f) * 2 * PI;  // 取得y轴与行进方向的夹角，(从yaw电机的编码得出)
  xAngle = -yAngle;                          // 进而得到x与chassis_x的夹角
  // 根据夹角对速速度向量做坐标系转换
  speed_x = cos(yAngle + PI / 2) * gim_y + cos(xAngle) * gim_x;
  speed_y = cos(xAngle + PI / 2) * gim_x + cos(yAngle) * gim_y;
  // 把转换好的速度发给各电机
  fp32 spd_1 = speed_x;
  fp32 spd_2 = speed_y;
  fp32 spd_3 = -speed_x;
  fp32 spd_4 = -speed_y;
  
  // 这些是通过电机速度，反解得到底盘或云台坐标系下的速度，可能不太对
  // real_chassis_x = (chassis->chassisMotor1->realSpeedF - chassis->chassisMotor3->realSpeedF) / 2.0f;
  // real_chassis_y = (chassis->chassisMotor2->realSpeedF - chassis->chassisMotor4->realSpeedF) / 2.0f;
  // real_chassis_z = (chassis->chassisMotor1->realSpeedF + chassis->chassisMotor2->realSpeedF\
  //   + chassis->chassisMotor3->realSpeedF + chassis->chassisMotor4->realSpeedF) / 4.0f;

  // real_gim_x = real_chassis_x * cos(PI / 4.0f - yaw_err_ecd * PI / 8192.0f) + \
  //   real_chassis_y * cos(PI / 4.0f + yaw_err_ecd * PI / 8192.0f);
  
  // real_gim_y = real_chassis_y * cos(PI / 4.0f - yaw_err_ecd * PI / 8192.0f) - \
  //   real_chassis_x * cos(PI / 4.0f + yaw_err_ecd * PI / 8192.0f);

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

      chassisMotorSpdProcess(chassis, spd_1, spd_2, spd_3, spd_4);

      DJI_MotorSetTarget((chassis->chassisMotor1), chassis->motor_spd.processed_set_spd1);
      DJI_MotorSetTarget((chassis->chassisMotor2), chassis->motor_spd.processed_set_spd2);
      DJI_MotorSetTarget((chassis->chassisMotor3), chassis->motor_spd.processed_set_spd3);
      DJI_MotorSetTarget((chassis->chassisMotor4), chassis->motor_spd.processed_set_spd4);
      break;

    case CHASSIS_FOLLOW:  // 底盘跟随云台
      DJI_MotorEnable(chassis->chassisMotor1);
      DJI_MotorEnable(chassis->chassisMotor2);
      DJI_MotorEnable(chassis->chassisMotor3);
      DJI_MotorEnable(chassis->chassisMotor4);
      chassisFollowEnable(chassis);

      spd_1 += *(chassis->follwoResult);
      spd_2 += *(chassis->follwoResult);
      spd_3 += *(chassis->follwoResult);
      spd_4 += *(chassis->follwoResult);
      chassisMotorSpdProcess(chassis, spd_1, spd_2, spd_3, spd_4);

      DJI_MotorSetTarget((chassis->chassisMotor1), chassis->motor_spd.processed_set_spd1);
      DJI_MotorSetTarget((chassis->chassisMotor2), chassis->motor_spd.processed_set_spd2);
      DJI_MotorSetTarget((chassis->chassisMotor3), chassis->motor_spd.processed_set_spd3);
      DJI_MotorSetTarget((chassis->chassisMotor4), chassis->motor_spd.processed_set_spd4);
      break;

    case CHASSIS_TOP:  // 小陀螺（也就是以云台指向为行进y轴正方向，然后加z）
      DJI_MotorEnable(chassis->chassisMotor1);
      DJI_MotorEnable(chassis->chassisMotor2);
      DJI_MotorEnable(chassis->chassisMotor3);
      DJI_MotorEnable(chassis->chassisMotor4);
      DJI_MotorDisable(chassis->gimbalMotor);

      top_time_cnt++;
      top_time_cnt %= TOP_T;

      fp32 min_top_spd = TOP_SPD_SCALE * sqrtf(gim_x * gim_x + gim_y * gim_y);

      top_spd_z = 1.9996f * SPEED_LIMIT - sqrtf(gim_x * gim_x + gim_y * gim_y);
      top_spd_z = top_spd_z > min_top_spd ? top_spd_z : min_top_spd;

      spd_1 += top_spd_z;
      spd_2 += top_spd_z;
      spd_3 += top_spd_z;
      spd_4 += top_spd_z;

      chassisMotorSpdProcess(chassis, spd_1, spd_2, spd_3, spd_4);

      DJI_MotorSetTarget((chassis->chassisMotor1), chassis->motor_spd.processed_set_spd1);
      DJI_MotorSetTarget((chassis->chassisMotor2), chassis->motor_spd.processed_set_spd2);
      DJI_MotorSetTarget((chassis->chassisMotor3), chassis->motor_spd.processed_set_spd3);
      DJI_MotorSetTarget((chassis->chassisMotor4), chassis->motor_spd.processed_set_spd4);
      break;
  }
  chassisFollowRun(chassis);  // 底盘跟随pid计算
}

