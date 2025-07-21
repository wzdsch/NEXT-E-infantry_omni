#ifndef CHASSIS_H
#define CHASSIS_H

#include "DJI_Motor.h"
#include "can.h"
#include "struct_typedef.h"
#include "MCUConnectStructs.h"

#define SPEED_LIMIT 8000.0f
#define SPEED_LIMIT_TOP 5000.0f

// 电机速度达到 set * UPDATE_SPD_RATE - UPDATE_SPD_ERR 时，认为已经达到设定速度
#define UPDATE_SPD_RATE 0.8f
#define UPDATE_SPD_ERR 500.0f

#define MOTOR_SPD_UP_RATE 500.0f // 电机加速度
#define MOTOR_SPD_DOWN_RATE 1000.0f // 电机减速度

#define TOP_SPD_SCALE 1.0f // 小陀螺平移时，对小陀螺速度和平移速度的分配比例，此值越大，小陀螺速度越快，平移速度越慢

// 底盘速度结构体
typedef struct chassis_motor_spd
{
    fp32 set_spd1; // 原始设定速度
    fp32 set_spd2; // 原始设定速度
    fp32 set_spd3; // 原始设定速度
    fp32 set_spd4; // 原始设定速度

    fp32 processed_set_spd1; // 处理过后的设定速度
    fp32 processed_set_spd2; // 处理过后的设定速度
    fp32 processed_set_spd3; // 处理过后的设定速度
    fp32 processed_set_spd4; // 处理过后的设定速度

    fp32 real_spd1; // 实际速度
    fp32 real_spd2; // 实际速度
    fp32 real_spd3; // 实际速度
    fp32 real_spd4; // 实际速度
} chassis_motor_spd_t;

// 底盘模式列表
enum chassisMode {
  CHASSIS_DISABLE = 0,  // 底盘无力
  CHASSIS_STOP,         // 底盘有力停止
  CHASSIS_FREE,         // 底盘不跟随
  CHASSIS_FOLLOW,       // 底盘跟随
  CHASSIS_TOP,          // 底盘小陀螺
};

/**
 * @brief  底盘结构体，重要的参数是底盘4个电机的结构体，以及云台电机结构体用于底盘跟随
 */
typedef struct chassisData {

  DJI_MotorGroup *group;  // 底盘所用的电机组，其实底盘电机不一定要在同一个组里，所以可以不填。
                          // 但填了可以直接通过chassis结构体查到电机组
  DJI_Motor *chassisMotor1;  // 底盘的四个电机，编号对应了位置关系的。
  DJI_Motor *chassisMotor2;
  DJI_Motor *chassisMotor3;
  DJI_Motor *chassisMotor4;
  DJI_Motor *gimbalMotor;  // yaw轴电机的结构体，目的是通过这个结构体接收电机数据，
                           // 但不对这个电机进行控制。
  uint16_t followFlagEcd;  // 底盘跟随的零点
  pids followPid0;         // 底盘跟随pid0,角度外环
  pids followPid1;         // 底盘跟随pid1，速度内环
  fp32 followPidout;       // 底盘跟随pid输出
  uint8_t followEN;        // 底盘跟随使能
  fp32 *follwoResult;      // 底盘跟随结果
  uint8_t mode;            // 底盘模式

  chassis_motor_spd_t motor_spd;  // 底盘电机速度
} chassis;


extern void chassisINIT(chassis *chassis, DJI_MotorGroup *group, DJI_Motor *chassisMotor1,
                        DJI_Motor *chassisMotor2, DJI_Motor *chassisMotor3,
                        DJI_Motor *chassisMotor4, DJI_Motor *gimbalMotor);

extern void chasisFollowINIT(chassis *chassis, uint16_t flagEcd, uint8_t pid0Mode,
                             fp32 pid0Datas[5], uint8_t pid1Mode, fp32 pid1Datas[5]);

extern void chassisChangeMode(chassis *chassis, uint8_t mode);

extern void chassisFollowEnable(chassis *chassis);
extern void chassisFollowDisable(chassis *chassis);
extern void followResultSet(chassis *chassis, fp32 *result);

extern void chassisRun(chassis *chassis, fp32 x, fp32 y, fp32 z, int16_t angle);

// extern void chassisMotorSpdUpdate(chassis *chassis);

extern fp32 filterF(fp32 new_data, fp32* buf, int num);

extern chassis chassis1;

#endif
