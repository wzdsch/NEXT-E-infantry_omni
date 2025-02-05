
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

#include "DJI_Motor.h"
#include "MCUConnect.h"
#include "Tools.h"
#include "math.h"
#include "pid.h"
#include "pidData.h"
#include "remote_control.h"

fp32 freeSpeed_z = 5000.0f;

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

void chassisFollowRun(chassis* chassis) {
  if (chassis->followEN == 1) {  // 如果底盘跟随使能
    fp32 temp;	// 相对角度
    if (chassis->gimbalMotor->realEcd >= chassis->followFlagEcd) {  // 得到编码值
      temp = (chassis->gimbalMotor->realEcd) - (chassis->followFlagEcd);
    }
    else {
      temp = (chassis->gimbalMotor->realEcd) - (chassis->followFlagEcd) + 8191;
    }
    // 过零处理
    if (0 - temp > 4096) {
      temp = temp + (4096 * 2);
    }
    else if (0 - temp <= -4096) {
      temp = temp - (4096 * 2);
    }
    // 计算pid
    chassis->followPidout = 0;//PID_calc(&(chassis->followPid1), chassis->gimbalMotor->realSpeedF, PID_calc(&(chassis->followPid0), -temp, 0));
  }
}

/**
 * @brief  底盘解算
 *          电机id要求与运动方向示意：
 *                                   ^chassis_y
 *                   \       ^y      /
 *                    \      |      /   <-z
 *                    m2           m1     |旋转正方向
 *                -----     yaw     ----->x
 *                    m3           m4
 *                   /       |       \
 *                  /        |         \chassis_x
 *                                      >
 * @param  chassis:底盘结构体
 * @param  x: x轴速度
 * @param  y: y轴速度
 * @param  z: z轴速度
 */
void chassisRun(chassis* chassis, fp32 x, fp32 y, fp32 z, int16_t angle) {
  // 根据chassis_y与云台指向的夹角计算temp，从云台坐标系(y-x)转换为底盘坐标系(chassis_y-chassis_x)
  // 然后根据轮子位置合成各轮子的转速
  fp32 speed0 = 0;
  fp32 speed1 = 0;
  fp32 speed2 = 0;
  fp32 speed3 = 0;
  fp32 xAngle = 0;
  fp32 yAngle = 0;
  fp32 speed_x = 0;
  fp32 speed_y = 0;
  // 从电机数据得到实际角度
  if (angle >= chassis->followFlagEcd) {
    angle = angle - chassis->followFlagEcd;
  }
  else {
    angle = angle - chassis->followFlagEcd + 8191;
  }

  switch (chassis->mode) {
    case CHASSIS_DISABLE:  // 底盘失能
      DJI_MotorDisable(chassis->chassisMotor1);
      DJI_MotorDisable(chassis->chassisMotor2);
      DJI_MotorDisable(chassis->chassisMotor3);
      DJI_MotorDisable(chassis->chassisMotor4);
      chassisFollowDisable(chassis);
      break;

    case CHASSIS_STOP:  // 底盘有力停止
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
      yAngle = ((angle + 1024) / 8192.0f) * 2 * PI;  // 取得y轴与行进方向的夹角，(从yaw电机的编码得出)
      xAngle = -yAngle;                          // 进而得到x与chassis_x的夹角
      // 根据夹角对速速度向量做坐标系转换
      speed_x = cos(yAngle + PI / 2) * y + cos(xAngle) * x;
      speed_y = cos(xAngle + PI / 2) * x + cos(yAngle) * y;
      // 把转换好的速度发给各电机
      speed0 = speed_x;
      speed1 = speed_y;
      speed2 = -speed_x;
      speed3 = -speed_y;
      DJI_MotorSetTarget((chassis->chassisMotor1), speed0);
      DJI_MotorSetTarget((chassis->chassisMotor2), speed1);
      DJI_MotorSetTarget((chassis->chassisMotor3), speed2);
      DJI_MotorSetTarget((chassis->chassisMotor4), speed3);
      break;

    case CHASSIS_FOLLOW:  // 底盘跟随云台
      DJI_MotorEnable(chassis->chassisMotor1);
      DJI_MotorEnable(chassis->chassisMotor2);
      DJI_MotorEnable(chassis->chassisMotor3);
      DJI_MotorEnable(chassis->chassisMotor4);
      chassisFollowEnable(chassis);
      yAngle =
        ((angle + 1024) / 8192.0f) * 2 * PI;  // 取得y轴与行进方向的夹角，(从yaw电机的编码得出)
      xAngle = -yAngle;                       // 进而得到x与chassis_x的夹角
      // 根据夹角对速速度向量做坐标系转换
      speed_x = cos(yAngle + PI / 2) * y + cos(xAngle) * x;
      speed_y = cos(xAngle + PI / 2) * x + cos(yAngle) * y;
      // 把转换好的速度发给各电机
      speed0 = speed_x;
      speed1 = speed_y;
      speed2 = -speed_x;
      speed3 = -speed_y;
      DJI_MotorSetTarget((chassis->chassisMotor1), speed0 + *(chassis->follwoResult));
      DJI_MotorSetTarget((chassis->chassisMotor2), speed1 + *(chassis->follwoResult));
      DJI_MotorSetTarget((chassis->chassisMotor3), speed2 + *(chassis->follwoResult));
      DJI_MotorSetTarget((chassis->chassisMotor4), speed3 + *(chassis->follwoResult));
      break;

    case CHASSIS_TOP:  // 小陀螺（也就是以云台指向为行进y轴正方向，然后加z）
      DJI_MotorEnable(chassis->chassisMotor1);
      DJI_MotorEnable(chassis->chassisMotor2);
      DJI_MotorEnable(chassis->chassisMotor3);
      DJI_MotorEnable(chassis->chassisMotor4);
      DJI_MotorDisable(chassis->gimbalMotor);
      yAngle =
        ((angle + 1024) / 8192.0f) * 2 * PI;  // 取得y轴与行进方向的夹角，(从yaw电机的编码得出)
      xAngle = -yAngle;                       // 进而得到x与chassis_x的夹角
      // 根据夹角对速速度向量做坐标系转换
      speed_x = cos(yAngle + PI / 2) * y + cos(xAngle) * x;
      speed_y = cos(xAngle + PI / 2) * x + cos(yAngle) * y;
      // 把转换好的速度发给各电机
      speed0 = speed_x;
      speed1 = speed_y;
      speed2 = -speed_x;
      speed3 = -speed_y;
      DJI_MotorSetTarget((chassis->chassisMotor1), speed0 + z);
      DJI_MotorSetTarget((chassis->chassisMotor2), speed1 + z);
      DJI_MotorSetTarget((chassis->chassisMotor3), speed2 + z);
      DJI_MotorSetTarget((chassis->chassisMotor4), speed3 + z);
      break;
  }
  chassisFollowRun(chassis);  // 底盘跟随pid计算
}
