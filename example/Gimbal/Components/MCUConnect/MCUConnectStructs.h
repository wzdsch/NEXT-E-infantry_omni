#ifndef MCUCONNECTSTRUCTS_H
#define MCUCONNECTSTRUCTS_H

#include "struct_typedef.h"

#define ChassisControlData_ID 0x8001

typedef struct ChassisControlData {
  uint8_t mode;
  uint8_t AutoAim;
  uint8_t cover;
  fp32 speedx;
  fp32 speedy;
  fp32 speedz;
} ChassisControl;

typedef struct GimbalControlData {
  uint8_t mode;
  fp32 yawAngle;
  fp32 pitchAngle;
} GimbalControl;

typedef struct RefereeData {
  // 云台接收
  uint16_t maxHeat;   // 枪管最大热量
  uint16_t gunHeat1;  // 枪管1热量
  uint8_t target;     // 目标
  fp32 gunSpeed1;     // 枪管1射速

} Referee_data;

// 双机通信数据
extern ChassisControl ChassisControlData;
extern GimbalControl GimbalControlData;
extern Referee_data RefereeData;
#endif
