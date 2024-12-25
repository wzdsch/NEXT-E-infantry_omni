#ifndef LADRC_H
#define LADRC_H
#include "struct_typedef.h"
/**
 *@Brief  以下为LADRC系统参数
 *@WangShun  2022-07-03  注释
 */
typedef struct LADRC {
  float v1, v2;         // 最速输出值
  float r;              // 速度因子
  float h;              // 积分步长
  float z1, z2, z3;     // 观测器输出
  float w0, wc, b0, u;  // 观测器带宽 控制器带宽 系统参数 控制器输出
} LADRC_O2;

/**
 *@Brief  以下为LADRC相关函数
 *@WangShun  2022-07-03  注释
 */
void LADRC_O2_Init(LADRC_O2 *LADRC_TYPE1, fp32 h, fp32 r, fp32 Wc, fp32 W0, fp32 B0);
void LADRC_O2_REST(LADRC_O2 *LADRC_TYPE1);
void LADRC_O2_TD(LADRC_O2 *LADRC_TYPE1, float Expect);
void LADRC_O2_ESO(LADRC_O2 *LADRC_TYPE1, float FeedBack);
void LADRC_O2_LF(LADRC_O2 *LADRC_TYPE1);
float LADRC_O2_Loop(LADRC_O2 *LADRC_TYPE1, float *Expect, float *RealTimeOut);

extern LADRC_O2 LADRC_Yaw;
#endif
