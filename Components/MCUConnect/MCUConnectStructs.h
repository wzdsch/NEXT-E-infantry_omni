#ifndef MCUCONNECTSTRUCTS_H
#define MCUCONNECTSTRUCTS_H

#include "struct_typedef.h"

#define ChassisControlData_ID 0x8001

typedef struct ChassisControlData {
  uint8_t mode;
  fp32 speedx;
  fp32 speedy;
  fp32 speedz;
} ChassisControl;

// 双机通信数据
extern ChassisControl ChassisControlData;

#endif
