#include "pid.h"

#include "main.h"
#include "pidData.h"
#include "stdio.h"

#define LimitMax(input, max) \
  {                          \
    if (input > max) {       \
      input = max;           \
    }                        \
    else if (input < -max) { \
      input = -max;          \
    }                        \
  }

void PID_init(pids *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout) {
  if (pid == NULL || PID == NULL) {
    return;
  }
  pid->mode = mode;
  pid->Kp = PID[0];
  pid->Ki = PID[1];
  pid->Kd = PID[2];
  pid->max_out = max_out;
  pid->max_iout = max_iout;
  pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
  pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out =
    0.0f;
}

fp32 PID_calc(pids *pid, fp32 ref, fp32 set) {
  if (pid == NULL) {
    return 0.0f;
  }
  pid->error[2] = pid->error[1];
  pid->error[1] = pid->error[0];
  pid->set = set;
  pid->fdb = ref;
  pid->error[0] = set - ref;
  switch (pid->mode) {
    case PID_POSITION:
      pid->Pout = pid->Kp * pid->error[0];
      pid->Iout += pid->Ki * pid->error[0];
      pid->Dbuf[2] = pid->Dbuf[1];
      pid->Dbuf[1] = pid->Dbuf[0];
      pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
      pid->Dout = pid->Kd * pid->Dbuf[0];
      LimitMax(pid->Iout, pid->max_iout);
      pid->out = pid->Pout + pid->Iout + pid->Dout;
      LimitMax(pid->out, pid->max_out);
      break;
    case PID_DELTA:
      pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
      pid->Iout = pid->Ki * pid->error[0];
      pid->Dbuf[2] = pid->Dbuf[1];
      pid->Dbuf[1] = pid->Dbuf[0];
      pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
      pid->Dout = pid->Kd * pid->Dbuf[0];
      pid->out += pid->Pout + pid->Iout + pid->Dout;
      LimitMax(pid->out, pid->max_out);
      break;
  }
  return pid->out;
}

void PID_clear(pids *pid) {
  if (pid == NULL) {
    return;
  }
  pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
  pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
  pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
  pid->fdb = pid->set = 0.0f;
}

void pidINIT(pids *pid, uint8_t mode, fp32 KP, fp32 KI, fp32 KD, fp32 maxOut, fp32 maxIout) {
  fp32 val_pid[3] = {KP, KI, KD};
  PID_init(pid, mode, val_pid, maxOut, maxIout);
  PID_clear(pid);
}

fp32 M3508_Speed_PID[5] = {M3508_Speed_PID_KP, M3508_Speed_PID_KI, M3508_Speed_PID_KD,
                           M3508_Speed_PID_MAX_OUT, M3508_Speed_PID_MAX_IOUT};
fp32 YAW_Speed_PID[5] = {YAW_Speed_PID_KP, YAW_Speed_PID_KI, YAW_Speed_PID_KD,
                         YAW_Speed_PID_MAX_OUT, YAW_Speed_PID_MAX_IOUT};
fp32 YAW_Angle_PID[5] = {YAW_Angle_PID_KP, YAW_Angle_PID_KI, YAW_Angle_PID_KD,
                         YAW_Angle_PID_MAX_OUT, YAW_Angle_PID_MAX_IOUT};
fp32 PITCH_Speed_PID[5] = {PITCH_Speed_PID_KP, PITCH_Speed_PID_KI, PITCH_Speed_PID_KD,
                           PITCH_Speed_PID_MAX_OUT, PITCH_Speed_PID_MAX_IOUT};
fp32 PITCH_Angle_PID[5] = {PITCH_Angle_PID_KP, PITCH_Angle_PID_KI, PITCH_Angle_PID_KD,
                           PITCH_Angle_PID_MAX_OUT, PITCH_Angle_PID_MAX_IOUT};
fp32 PITCH_Speed_IMU_PID[5] = {PITCH_Speed_IMU_PID_KP, PITCH_Speed_IMU_PID_KI,
                               PITCH_Speed_IMU_PID_KD, PITCH_Speed_IMU_PID_MAX_OUT,
                               PITCH_Speed_IMU_PID_MAX_IOUT};
fp32 PITCH_Angle_IMU_PID[5] = {PITCH_Angle_IMU_PID_KP, PITCH_Angle_IMU_PID_KI,
                               PITCH_Angle_IMU_PID_KD, PITCH_Angle_IMU_PID_MAX_OUT,
                               PITCH_Angle_IMU_PID_MAX_IOUT};
fp32 FRI_Speed_PID[5] = {FRI_Speed_PID_KP, FRI_Speed_PID_KI, FRI_Speed_PID_KD,
                         FRI_Speed_PID_MAX_OUT, FRI_Speed_PID_MAX_IOUT};
fp32 M2006_Speed_PID[5] = {M2006_Speed_PID_KP, M2006_Speed_PID_KI, M2006_Speed_PID_KD,
                           M2006_Speed_PID_MAX_OUT, M2006_Speed_PID_MAX_IOUT};
fp32 M2006_Angle_PID[5] = {M2006_Angle_PID_KP, M2006_Angle_PID_KI, M2006_Angle_PID_KD,
                            M2006_Angle_PID_MAX_OUT, M2006_Angle_PID_MAX_IOUT};
fp32 Vision_Yaw_PID[5] = {Vision_Yaw_PID_KP, Vision_Yaw_PID_KI, Vision_Yaw_PID_KD,
                          Vision_Yaw_PID_MAX_OUT, Vision_Yaw_PID_MAX_IOUT};
fp32 Vision_Pitch_PID[5] = {Vision_Pitch_PID_KP, Vision_Pitch_PID_KI, Vision_Pitch_PID_KD,
                            Vision_Pitch_PID_MAX_OUT, Vision_Pitch_PID_MAX_IOUT};
