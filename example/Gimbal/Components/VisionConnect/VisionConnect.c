#include "VisionConnect.h"

#include "pid.h"
#include "stdio.h"
#include "usbd_cdc_if.h"

/**
 * @brief
 * @param  connect:
 * @param  TargetColor:
 */
void VisionConnectINIT(VisionConnect *connect, uint8_t TargetColor) {
  connect->RXData.rx.data.PitchAngleTarget = 0.0f;
  connect->RXData.rx.data.YawAngleTarget = 0.0f;
  connect->RXData.rx.data.fireControl = HOLD_FIRE;
  connect->RXData.rx.data.head = 0;
  connect->TXData.targetType = 0;
  connect->TXData.PitchAngle = 0.0f;
  connect->TXData.YawAngle = 0.0f;
  connect->TXData.TargetColor = TargetColor;
  connect->TXData.ShootSpeed = 0.0f;
}

/**
 * @brief  更新视觉发送缓存
 * @param  connect:
 * @param  TargetColor:
 * @param  yawAngle:
 * @param  pitchAngle:
 * @param  shootSpeed:
 */
void VisionConnectUpdateTX(VisionConnect *connect, uint8_t TargetColor, fp32 yawAngle,
                           fp32 pitchAngle, fp32 shootSpeed) {
  connect->TXData.targetType = 0;
  connect->TXData.TargetColor = TargetColor;
  connect->TXData.PitchAngle = pitchAngle;
  connect->TXData.YawAngle = yawAngle;
  connect->TXData.ShootSpeed = shootSpeed;
}

/**
 * @brief  向视觉上位机发送数据
 * @param  connnect:
 */
void VisionConnectSend(VisionConnect *connnect) {

  connnect->TXData.packHead = 0x00a5;  // 包头
  connnect->TXData.targetType = 0x00;
  // 敌方颜色
  if (connnect->TXData.TargetColor == RED) {
    connnect->TXData.TargetColor = 0x01;
  }
  else if (connnect->TXData.TargetColor == BLUE) {
    connnect->TXData.TargetColor = 0x00;
  }
  // 发送
  CDC_Transmit_FS((uint8_t *)&(connnect->TXData), 16);
}

/**
 * @brief  解包视觉接收数据
 * @param  connect:
 */
void VisionConnectUnpack(VisionConnect *connect) {
  if (connect->RXData.RXBUF[0] == 0xa5) {
    for (uint8_t i = 0; i < 10; i++) {
      connect->RXData.rx.RxData[i] = connect->RXData.RXBUF[i];
    }
  }
}
