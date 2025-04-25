#ifndef PIDDATA_H
#define PIDDATA_H

#include "struct_typedef.h"

// 3508速度环
#define M3508_Speed_PID_KP       10.0f
#define M3508_Speed_PID_KI       0.5f
#define M3508_Speed_PID_KD       0.1f
#define M3508_Speed_PID_MAX_OUT  10000.0f
#define M3508_Speed_PID_MAX_IOUT 3000.0f

// 底盘跟随速度环
#define Chassis_Speed_PID_KP       5.0f
#define Chassis_Speed_PID_KI       0.1f
#define Chassis_Speed_PID_KD       0.00f
#define Chassis_Speed_PID_MAX_OUT  3000.0f
#define Chassis_Speed_PID_MAX_IOUT 500.0f

// 底盘跟随角度环
#define Chassis_Angle_PID_KP       0.14f
#define Chassis_Angle_PID_KI       0.00f
#define Chassis_Angle_PID_KD       20.0f
#define Chassis_Angle_PID_MAX_OUT  2000.0f
#define Chassis_Angle_PID_MAX_IOUT 500.0f
#endif
