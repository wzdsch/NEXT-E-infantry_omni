#ifndef PIDDATA_H
#define PIDDATA_H

#include "struct_typedef.h"

// 3508速度环
#define M3508_Speed_PID_KP       5.0f
#define M3508_Speed_PID_KI       0.05f
#define M3508_Speed_PID_KD       0.001f
#define M3508_Speed_PID_MAX_OUT  16384.0f
#define M3508_Speed_PID_MAX_IOUT 1000.0f

// 底盘跟随速度环
#define Chassis_Speed_PID_KP       0.8f
#define Chassis_Speed_PID_KI       0.001f
#define Chassis_Speed_PID_KD       5.00f
#define Chassis_Speed_PID_MAX_OUT  16384.0f
#define Chassis_Speed_PID_MAX_IOUT 1000.0f

// 底盘跟随角度环
#define Chassis_Angle_PID_KP       10.0f
#define Chassis_Angle_PID_KI       0.00f
#define Chassis_Angle_PID_KD       10.0f
#define Chassis_Angle_PID_MAX_OUT  16384.0f
#define Chassis_Angle_PID_MAX_IOUT 1000.0f
#endif
