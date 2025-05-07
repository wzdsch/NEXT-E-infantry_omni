
#ifndef MCUCONNECTSTRUCTS_H
#define MCUCONNECTSTRUCTS_H

#include "struct_typedef.h"

#define ChassisControlData_ID 0x8001
#define RefereeData_ID        0x4001

typedef struct ChassisControlData {
  uint8_t mode;
  uint8_t AutoAim;
  uint8_t cover;
  fp32 speedx;
  fp32 speedy;
  fp32 speedz;
} ChassisControl;

typedef struct RefereeData {
  // 云台接收
  uint16_t maxHeat;   // 枪管最大热量
  uint16_t gunHeat1;  // 枪管1热量
  uint16_t heat_cooling; // 冷却速率
  uint8_t our_color;     // 己方颜色 'B' = "blue"  'R' = "red"
  fp32 gunSpeed1;     // 枪管1射速
  
} Referee_data;

// 双机通信数据
extern ChassisControl ChassisControlData;
extern Referee_data RefereeData;
#endif
