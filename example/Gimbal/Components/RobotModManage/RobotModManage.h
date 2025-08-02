#ifndef ROBOT_MOD_MANAGE_H
#define ROBOT_MOD_MANAGE_H
#include "struct_typedef.h"
#include "usart.h"
#if IMU_PITCH == 0

#define pitchAngleMax 600
#define pitchAngleMin -600
#define OneStep       2.0f
#define MotorRate     22.775f
#define GetAngle      MotorToAngle
#else

#define pitchAngleMax 30
#define pitchAngleMin -20
#define OneStep       0.16f
#define MotorRate     1.0f  // 主要是pitch编码器闭环下需要用这个角度到编码器值的换算倍数
#define GetAngle      IMUToAngle

#endif

extern uint8_t tuneTime;

typedef struct flagdata {  // 一些标志位，PC控制使用
  uint8_t FreeFlag;         // 小陀螺
  uint8_t LeftTurn;         // 左转
  uint8_t RightTurn;        // 右转
  uint8_t single_shoot_flg; // 单发
} PC_Flags;

extern void remote_controller(void);
extern void tuneRun(void);
extern void tuneINIT(void);
extern void tune(uint8_t time);

#endif
