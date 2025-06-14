#ifndef MCUCONNECTSTRUCTS_H
#define MCUCONNECTSTRUCTS_H

#include "struct_typedef.h"

#define ChassisControlData_ID 0x8001

typedef struct ChassisControlData {
  uint8_t mode;
  uint8_t with_supercap; // 是否使用超级电容w
  fp32 speedx;
  fp32 speedy;
  fp32 speedz;
  uint8_t ui_refresh; // UI刷新标志位 1是刷新
  uint8_t vision_tracking; // 视觉跟踪标志位 1是跟踪
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
  uint16_t heat_cooling; // 冷却速率
  uint8_t OurColor;     // 目标
  fp32 gunSpeed1;     // 枪管1射速

} Referee_data;

// 双机通信数据
extern ChassisControl ChassisControlData;
extern GimbalControl GimbalControlData;
extern Referee_data RefereeData;
#endif
