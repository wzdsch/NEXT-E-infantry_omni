#ifndef PIDDATA_H
#define PIDDATA_H

// 3508速度环
#define M3508_Speed_PID_KP       5.0f
#define M3508_Speed_PID_KI       0.05f
#define M3508_Speed_PID_KD       -0.01f
#define M3508_Speed_PID_MAX_OUT  16384.0f
#define M3508_Speed_PID_MAX_IOUT 1000.0f

/*
//6020速度环
#define M6020_Speed_PID_KP 50.0f
#define M6020_Speed_PID_KI 2.5f
#define M6020_Speed_PID_KD 0.003f
#define M6020_Speed_PID_MAX_OUT 15000.0f
#define M6020_Speed_PID_MAX_IOUT 1000.0f
//6020角度环
#define M6020_Angle_PID_KP 0.1f
#define M6020_Angle_PID_KI 0.0002f
#define M6020_Angle_PID_KD 0.0f
#define M6020_Angle_PID_MAX_OUT 15000.0f
#define M6020_Angle_PID_MAX_IOUT 1000.0f
*/

// PITCH速度环
#define PITCH_Speed_PID_KP       5000.0f
#define PITCH_Speed_PID_KI       0.01f
#define PITCH_Speed_PID_KD       -0.1f
#define PITCH_Speed_PID_MAX_OUT  30000.0f
#define PITCH_Speed_PID_MAX_IOUT 500.0f
// PITCH角度环
#define PITCH_Angle_PID_KP       2.0f
#define PITCH_Angle_PID_KI       0.0001f
#define PITCH_Angle_PID_KD       -1.0f
#define PITCH_Angle_PID_MAX_OUT  600.0f
#define PITCH_Angle_PID_MAX_IOUT 0.7f

// PITCH速度环(陀螺仪闭环)
#define PITCH_Speed_IMU_PID_KP       7000.0f
#define PITCH_Speed_IMU_PID_KI       10.0f
#define PITCH_Speed_IMU_PID_KD       0.0f
#define PITCH_Speed_IMU_PID_MAX_OUT  15000.0f
#define PITCH_Speed_IMU_PID_MAX_IOUT 5000.0f
// PITCH角度环(陀螺仪闭环)
#define PITCH_Angle_IMU_PID_KP       1.0f
#define PITCH_Angle_IMU_PID_KI       0.01f
#define PITCH_Angle_IMU_PID_KD       1.0f
#define PITCH_Angle_IMU_PID_MAX_OUT  8.0f
#define PITCH_Angle_IMU_PID_MAX_IOUT 3.0f

/*
//串口陀螺仪参数
// yaw速度环
#define YAW_Speed_PID_KP 200.0f
#define YAW_Speed_PID_KI 0.5f
#define YAW_Speed_PID_KD 0.0f
#define YAW_Speed_PID_MAX_OUT 30000.0f
#define YAW_Speed_PID_MAX_IOUT 30000.0f
// yaw角度环
#define YAW_Angle_PID_KP 30.0f
#define YAW_Angle_PID_KI 0.0f
#define YAW_Angle_PID_KD 0.0f
#define YAW_Angle_PID_MAX_OUT 600.0f
#define YAW_Angle_PID_MAX_IOUT 100.0f
*/
// yaw速度环
#define YAW_Speed_PID_KP 7000.0f
#define YAW_Speed_PID_KI 10.0f
#define YAW_Speed_PID_KD 0.0f
#define YAW_Speed_PID_MAX_OUT 15000.0f
#define YAW_Speed_PID_MAX_IOUT 5000.0f
// yaw角度环
#define YAW_Angle_PID_KP 0.5f
#define YAW_Angle_PID_KI 0.00f
#define YAW_Angle_PID_KD 0.5f
#define YAW_Angle_PID_MAX_OUT 20.0f
#define YAW_Angle_PID_MAX_IOUT 2.0f

// 摩擦轮速度环
#define FRI_Speed_PID_KP       20.0f
#define FRI_Speed_PID_KI       0.0f
#define FRI_Speed_PID_KD       40.0f
#define FRI_Speed_PID_MAX_OUT  16384.0f
#define FRI_Speed_PID_MAX_IOUT 1000.0f

// 2006速度环
#define M2006_Speed_PID_KP       10.0f
#define M2006_Speed_PID_KI       0.000f
#define M2006_Speed_PID_KD       0.0f
#define M2006_Speed_PID_MAX_OUT  15000.0f
#define M2006_Speed_PID_MAX_IOUT 1000.0f

// 2006角度环
#define M2006_Angle_PID_KP       0.3f
#define M2006_Angle_PID_KI       0.00f
#define M2006_Angle_PID_KD       0.0f
#define M2006_Angle_PID_MAX_OUT  15000.0f
#define M2006_Angle_PID_MAX_IOUT 2000.0f

// 视觉pid
#define Vision_Yaw_PID_KP       100.0f
#define Vision_Yaw_PID_KI       2.0f
#define Vision_Yaw_PID_KD       0.0f
#define Vision_Yaw_PID_MAX_OUT  16384.0f
#define Vision_Yaw_PID_MAX_IOUT 16384.0f

#define Vision_Pitch_PID_KP       0.5f
#define Vision_Pitch_PID_KI       0.0f
#define Vision_Pitch_PID_KD       0.0f
#define Vision_Pitch_PID_MAX_OUT  1000.0f
#define Vision_Pitch_PID_MAX_IOUT 1000.0f

#endif
