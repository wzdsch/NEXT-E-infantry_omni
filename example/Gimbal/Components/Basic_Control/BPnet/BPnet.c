#include "BPnet.h"

#include "arm_math.h"
#include "main.h"
#include "pid.h"

int8_t sign(BPnet* net) {
  fp32 temp = ((net->yout) - (net->lastyout)) / ((net->pidout) - (net->lastpidout));
  if (temp > 0) {
    return 1;
  }
  else if (temp < 0) {
    return -1;
  }
  else {
    return 0;
  }
}

fp64 sigmod(fp64 x) {
  return 1.0f / (1 + exp(0 - x));
}

fp64 sigmod_dx(fp64 x) {
  return x * (1.0f - x);
}

void BPnetINIT(BPnet* net, pids* pid, fp32* target, fp32* feedback, fp32 step) {
  net->bpPID = pid;
  net->target = target;
  net->feedback = feedback;
  net->step = step;

  for (uint8_t i = 0; i < 3; i++) {
    net->input[i] = 0;
  }

  for (uint8_t i = 0; i < 3; i++) {
    for (uint8_t j = 0; j < 6; j++) {
      net->w[i][j] = 1;
    }
  }

  for (uint8_t i = 0; i < 6; i++) {
    net->alpha[i] = 0;
  }

  for (uint8_t i = 0; i < 6; i++) {
    net->h[i] = 0;
  }

  for (uint8_t i = 0; i < 6; i++) {
    for (uint8_t j = 0; j < 3; j++) {
      net->u[i][j] = 1;
    }
  }

  for (uint8_t i = 0; i < 3; i++) {
    net->beta[i] = 0;
  }

  for (uint8_t i = 0; i < 3; i++) {
    net->output[i] = 0;
  }

  net->yout = 0;
  net->lastyout = 0;
  net->pidout = 0;
  net->lastpidout = 0;
  net->Ei = 0;
  pidINIT(net->bpPID, PID_POSITION, 0, 0, 0, 16384, 1000);
}

void BPforward(BPnet* net) {
  net->lastyout = net->yout;
  net->input[0] = *(net->target) / 10000.0f;
  net->input[1] = *(net->feedback) / 10000.0f;
  net->input[2] = (net->input[0]) - (net->input[1]);
  net->Ei += net->input[2];
  net->yout = *(net->feedback);
  for (uint8_t i = 0; i < 6; i++) {
    // 计算隐藏层输入
    net->alpha[i] = (net->input[0]) * (net->w[0][i]) + (net->input[1]) * (net->w[1][i])
                    + (net->input[2]) * (net->w[2][i]);
  }
  for (uint8_t i = 0; i < 6; i++) {
    // 计算隐藏层输出
    net->h[i] = sigmod(net->alpha[i]);
  }
  for (uint8_t i = 0; i < 3; i++) {
    // 计算输出层输入
    net->beta[i] = (net->h[0]) * (net->u[0][i]) + (net->h[1]) * (net->u[1][i])
                   + (net->h[2]) * (net->u[2][i]) + (net->h[3]) * (net->u[3][i])
                   + (net->h[4]) * (net->u[4][i]) + (net->h[5]) * (net->u[5][i]);
  }
  for (uint8_t i = 0; i < 3; i++) {
    // 计算输出层输出
    net->output[i] = sigmod(net->beta[i]);
  }
  // 更新pid参数
  net->bpPID->Kp = net->output[0] * 50000;
  net->bpPID->Ki = net->output[1] * 50000;
  net->bpPID->Kd = net->output[2] * 50000;
  net->lastpidout = net->pidout;
  net->pidout = PID_calc(net->bpPID, net->input[1], net->input[0]);
}

void BPbackward(BPnet* net) {
  fp64 dE_dKp = (net->input[2] * net->input[2]) * (-1.0f) * sign(net);
  fp64 dE_dKi = (net->input[2]) * (-1.0f) * net->Ei * sign(net);
  fp64 dE_dKd = (net->input[2]) * (-1.0f) * net->bpPID->Dbuf[0] * sign(net);

  for (uint8_t i = 0; i < 6; i++) {
    net->u[i][0] += net->step * (dE_dKp * sigmod_dx(net->output[0]) * net->h[i]);
    net->u[i][1] += net->step * (dE_dKi * sigmod_dx(net->output[1]) * net->h[i]);
    net->u[i][2] += net->step * (dE_dKd * sigmod_dx(net->output[2]) * net->h[i]);
  }
  for (uint8_t j = 0; j < 6; j++) {
    net->w[0][j] +=
      net->step
      * ((dE_dKp * sigmod_dx(net->output[0]) * net->u[j][0] * sigmod_dx(net->h[j]) * net->input[0])
         + (dE_dKi * sigmod_dx(net->output[1]) * net->u[j][1] * sigmod_dx(net->h[j])
            * net->input[0])
         + (dE_dKd * sigmod_dx(net->output[2]) * net->u[j][2] * sigmod_dx(net->h[j])
            * net->input[0]));

    net->w[1][j] +=
      net->step
      * ((dE_dKp * sigmod_dx(net->output[0]) * net->u[j][0] * sigmod_dx(net->h[j]) * net->input[1])
         + (dE_dKi * sigmod_dx(net->output[1]) * net->u[j][1] * sigmod_dx(net->h[j])
            * net->input[1])
         + (dE_dKd * sigmod_dx(net->output[2]) * net->u[j][2] * sigmod_dx(net->h[j])
            * net->input[1]));

    net->w[2][j] +=
      net->step
      * ((dE_dKp * sigmod_dx(net->output[0]) * net->u[j][0] * sigmod_dx(net->h[j]) * net->input[2])
         + (dE_dKi * sigmod_dx(net->output[1]) * net->u[j][1] * sigmod_dx(net->h[j])
            * net->input[2])
         + (dE_dKd * sigmod_dx(net->output[2]) * net->u[j][2] * sigmod_dx(net->h[j])
            * net->input[2]));
  }
}
