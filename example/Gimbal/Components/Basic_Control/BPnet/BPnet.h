#include "arm_math.h"
#include "main.h"
#include "pid.h"

typedef struct BPnet_t {
  fp32 step;
  fp32* target;
  fp32* feedback;
  fp64 input[3];
  fp64 w[3][6];
  fp64 alpha[6];
  fp64 h[6];
  fp64 u[6][3];
  fp64 beta[3];
  fp64 output[3];
  pids* bpPID;

  fp64 Ei;
  fp32 yout;
  fp32 lastyout;
  fp32 pidout;
  fp32 lastpidout;
} BPnet;

extern BPnet BPnet1;
extern void BPnetINIT(BPnet* net, pids* pid, fp32* target, fp32* feedback, fp32 step);
extern void BPforward(BPnet* net);
extern void BPbackward(BPnet* net);
