#ifndef VOFA_H
#define VOFA_H
#include "usart.h"
#include "struct_typedef.h"
// 使用联合体，避免对数据的重复搬运
typedef struct VofaDatas {
  fp32 set;
  fp32 feedback;
  fp32 TargetSpeed1;
  fp32 Speed1;
  uint8_t tail1;
  uint8_t tail2;
  uint8_t tail3;
  uint8_t tail4;
} VofaData;

union VofaDATA {
  VofaData VofaD;
  uint8_t ch[20];  // 4*4+4=20
};
extern union VofaDATA Vofa;
extern void JustFloat(fp32 set, fp32 feedback, fp32 target, fp32 speed, UART_HandleTypeDef *huart);
#endif
