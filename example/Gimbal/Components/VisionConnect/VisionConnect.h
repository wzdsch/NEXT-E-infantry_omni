#ifndef VISIONCONNECT_H
#define VISIONCONNECT_H
#include "pid.h"
#include "struct_typedef.h"

enum FireFlag {
  HOLD_FIRE = 0,
  OPEN_FIRE = 1,
};

enum TargetColor {
  BLUE = 0,
  RED = 1,
};

#pragma pack(2)

typedef struct VisionDataRX {
  uint8_t RXBUF[10];

  union {
    uint8_t RxData[10];

    struct {
      uint8_t head;
      uint8_t fireControl;
      fp32 YawAngleTarget;
      fp32 PitchAngleTarget;
    } data;
  } rx;
} visionRX;

typedef struct VisionDataTX {
  uint16_t packHead;
  uint8_t targetType;   // 目标类型
  uint8_t TargetColor;  // 敌方颜色
  fp32 YawAngle;        // yaw轴当前值（角度）
  fp32 PitchAngle;      // pitch轴当前值
  fp32 ShootSpeed;      // 弹速
} visionTX;

typedef struct VisionDataAll {
  visionRX RXData;
  visionTX TXData;
} VisionConnect;

#pragma pack()
void VisionConnectINIT(VisionConnect *connect, uint8_t TargetColor);
void VisionConnectUpdateTX(VisionConnect *connect, uint8_t TargetColor, fp32 yawAngle,
                           fp32 pitchAngle, fp32 shootSpeed);
void VisionConnectSend(VisionConnect *connnect);
void VisionConnectUnpack(VisionConnect *connect);

extern VisionConnect vision1;

#endif
