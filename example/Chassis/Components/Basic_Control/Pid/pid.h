#include "main.h"
#include "struct_typedef.h"
#ifndef PID_H
#define PID_H

enum PID_MODE {
  PID_POSITION = 0,
  PID_DELTA
};

typedef struct pidData {
  uint8_t mode;
  // PID数据
  fp32 Kp;
  fp32 Ki;
  fp32 Kd;

  fp32 max_out;   // 输出限幅
  fp32 max_iout;  // 积分限幅

  fp32 set;
  fp32 fdb;

  fp32 out;
  fp32 Pout;
  fp32 Iout;
  fp32 Dout;
  fp32 Dbuf[3];
  fp32 error[3];
} pids;

extern void PID_init(pids *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
extern fp32 PID_calc(pids *pid, fp32 ref, fp32 set);
extern void PID_clear(pids *pid);
void pidINIT(pids *pid, uint8_t mode, fp32 KP, fp32 KI, fp32 KD, fp32 maxOut, fp32 maxIout);

extern fp32 M3508_Speed_PID[5];
extern fp32 Chassis_Speed_PID[5];
extern fp32 Chassis_Angle_PID[5];
extern fp32 PITCH_Speed_PID[5];
extern fp32 PITCH_Angle_PID[5];
extern fp32 Helm0_Angle_PID[5];
extern fp32 Helm0_Speed_PID[5];
extern fp32 Helm1_Angle_PID[5];
extern fp32 Helm1_Speed_PID[5];
extern pids Helm0_Speed;
extern pids Helm0_Position;
extern pids Helm0_Current;
extern pids Helm1_Speed;
extern pids Helm1_Position;
extern pids Helm1_Current;
#endif
