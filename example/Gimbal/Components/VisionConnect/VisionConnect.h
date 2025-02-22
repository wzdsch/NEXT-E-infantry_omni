#ifndef VISIONCONNECT_H
#define VISIONCONNECT_H
#include "pid.h"
#include "struct_typedef.h"

enum FireFlag {
  HOLD_FIRE = 0,
  OPEN_FIRE = 1,
};

enum OurColor {
  BLUE = 'B',
  RED = 'R'
};

#pragma pack(1) // 按1字节对齐

typedef struct VisionDataRX {
  uint8_t header; // 包头

	uint8_t state; // 状态位
	  /*
     * ## 状态位，指示自瞄运行状态 ##
     *  ______________________________________________________
     * | BIT | DESCRIBE      | SET 1      | SET 0   | DEFAULT |
     * |  0  | auto aim      | enable     | disable | disable | 自瞄是否可用 | 自瞄可用时，每帧此项都为1，方便画UI
     * |  1  | our color     | red        | blue    | blue    | 当前己方颜色，用于电控核验数据
     * |  2  | tracking      | yes        | no      | no      | 是否正在跟踪（建议在操作手按下右键但tracking为no的情况下，在UI标明）
     * | 3&4 | aim type      | 0 0 = car (default)  |         | 当前跟踪目标
     * | 3&4 | aim type      | 1 0 = outpost        |         | 非吊射模式锁定基地时，改标志位为：0 0
     * | 3&4 | aim type      | 0 1 = lob            |         | 吊射和能量机关功能需操作手启动。启动后无论是否跟踪到目标，该项均为启动
     * | 3&4 | aim type      | 1 1 = rune           |         | 的模式。
     * |  5  | reserve       |            |         |         |
     * |  6  | reserve       |            |         |         |
     * |  7  | reserve       |            |         |         |
     */

  fp32 PitchAngleTarget;
  fp32 YawAngleTarget;

  uint8_t fireControl; // 开火位
	  /*
     * ## 开火位 ##
     * 注意：只有当开火位为 0xff 时才表示开火
     * 注意：开火位不会只发送一次，当自瞄认为当前处于适宜状态下，该开位均为 0xff
     *
     * Fire     | 0xff
     * Not Fire | Other Num
     * Default  | Not Fire
     *
     */
} VisionDatas;

union VisionData {
  VisionDatas VisionRxData;
  uint8_t Rxdata[10];
};

typedef struct VisionDataTX {
  uint8_t packHead;	// 包头
  uint8_t OurColor;	// 我方颜色 == 'R'(red) | 'B'(blue)
  uint8_t Command;	// 命令位，对自瞄进行控制
    /*
     * ## 命令位，对自瞄进行控制 ##
     *  __________________________________________
     * | BIT | DESCRIBE  | SET 1 | SET 0 | DEFAULT |
     * |  0  | reserve   |       |       |         |
     * |  1  | reserve   |       |       |         |
     * |  2  | reserve   |       |       |         |
     * |  3  | reserve   |       |       |         |
     * |  4  | reserve   |       |       |         |
     * |  5  | reserve   |       |       |         |
     * |  6  | reserve   |       |       |         |
     * |  7  | reserve   |       |       |         |
     */

  fp32 PitchAngle;	// pitch轴当前值
  fp32 YawAngle;		// yaw轴当前值（角度）
  fp32 ShootSpeed;	// 弹速
} visionTX;

typedef struct VisionDataAll {
  union VisionData RXData;
  visionTX TXData;
} VisionConnect;

typedef struct { // 导航结构体
  uint8_t head; // 0xf5
  fp32 spd_y;
  fp32 spd_x;
  fp32 spd_z;
} nav_rxd_t;

union nav_rxd_u {
  nav_rxd_t nav_rxd;
  uint8_t rx_buf[13];
};

#pragma pack() // 恢复默认对齐

enum aim_type	// 从视觉接收的 正在自瞄的目标类型
{
	CAR = 0,			// 机器人
	OUT_POST = 1,	// 前哨站
	RUNE = 3,			// 能量机关
};

typedef struct
{
	uint8_t aoto_aim_state;	// 自瞄状态 不能跟踪(0) | 能跟踪(1)
	uint8_t our_color;			// 我方颜色 'R'(red) | 'B'(blue)
	uint8_t tracking;				// 是否锁到敌人 0(no) | 1(yes)
	uint8_t aim_type;				// 从视觉接收的 正在自瞄的目标类型 enum类型
} VisionState;


void VisionConnectINIT(VisionConnect *connect, uint8_t OurColor);
void VisionConnectUpdateTX(VisionConnect *connect, uint8_t OurColor, fp32 yawAngle,
                           fp32 pitchAngle, fp32 shootSpeed);
void VisionConnectSend(VisionConnect *connnect);
void VisionConnectUnpack(VisionConnect *connect);
void VisionGetState(VisionConnect* connect, VisionState* visionState);

extern VisionConnect vision1;
extern VisionState visionState;

#endif
