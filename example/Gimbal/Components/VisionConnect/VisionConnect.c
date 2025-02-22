#include "VisionConnect.h"

#include "pid.h"
#include "stdio.h"
#include "usbd_cdc_if.h"

VisionState visionState;
union nav_rxd_u nav_rxd;

/**
 * @brief
 * @param  connect:
 * @param  TargetColor:
 */
void VisionConnectINIT(VisionConnect *connect, uint8_t OurColor) {
  connect->RXData.VisionRxData.PitchAngleTarget = 0.0f;
  connect->RXData.VisionRxData.YawAngleTarget = 0.0f;
  connect->RXData.VisionRxData.fireControl = HOLD_FIRE;
  connect->RXData.VisionRxData.header = 0;
  // connect->TXData.targetType = 0;
  connect->TXData.PitchAngle = 0.0f;
  connect->TXData.YawAngle = 0.0f;
  connect->TXData.OurColor = OurColor;
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
void VisionConnectUpdateTX(VisionConnect *connect, uint8_t OurColor, fp32 yawAngle,
                           fp32 pitchAngle, fp32 shootSpeed) {
  // connect->TXData.targetType = 0;
  connect->TXData.OurColor = OurColor;

  connect->TXData.PitchAngle = -pitchAngle;	// 反转pitch轴
  connect->TXData.YawAngle = yawAngle + 180.0f;	// 将yawAngle的范围由-180~180改为0~360
  connect->TXData.ShootSpeed = shootSpeed;
}

/**
 * @brief  向视觉上位机发送数据
 * @param  connnect:
 */
void VisionConnectSend(VisionConnect *connnect) {

  connnect->TXData.packHead = 0x00a5;  // 包头
  // connnect->TXData.targetType = 0x00;
  // 我方颜色
  if (connnect->TXData.OurColor == RED) {
    connnect->TXData.OurColor = RED;
  }
  else if (connnect->TXData.OurColor == BLUE) {
    connnect->TXData.OurColor = BLUE;
  }
	
  // 发送
  CDC_Transmit_FS((uint8_t *)&(connnect->TXData), sizeof(connnect->TXData));
}

// 解包state
void VisionGetState(VisionConnect* connect, VisionState* visionState)
{
	uint8_t state = connect->RXData.VisionRxData.state;
	visionState->aoto_aim_state = state & 0x01;
	visionState->our_color = (state & 0x02) >> 1;
	visionState->tracking = (state & 0x04) >> 2;
	visionState->aim_type = (state & 0x18) >> 3;
}
