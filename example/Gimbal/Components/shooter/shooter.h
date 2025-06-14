#include "DJI_Motor.h"
#include "main.h"
#include "struct_typedef.h"

#define stuckProcessCountLimit 100    // 卡弹处理时间
#define startDelayLimit        50   // 电机启动时间
#define stuckCountLimit        100   // 卡弹处理触发时间
#define stuckPersent           0.01f  // 卡弹处理阈值

#define shooterSpeed 6200.0f

#define TOTAL_ECD_PER_SHOOT 73728

#define SHOOTER_FREQ_DFLT 20.0f // 默认频率

enum shooter_Mode {
  SHOOTER_STOP = 0,    // 所有电机停转5
  SHOOTER_HOLD = 1,    // 只开摩擦轮
  SHOOTER_FIRE = 2,    // 所有电机运转
  SHOOTER_DBUG = 3,    // 摩擦轮停转，拨弹轮旋转
  SHOOTER_DBUG1 = 4,   // 摩擦轮停转，拨弹轮反转
  SHTOOER_ONETAP = 5,  // 单发模式
};

enum supplier_Mode {
  SUPPLIER_STOP = 0,   // 停转
  SUPPLIER_ERROR = 1,  // 反转
  SUPPLIER_RUN = 2,    // 正转
};

typedef struct shooterDat {  // 发射机构结构体
  DJI_MotorGroup *shooterGroup;
  DJI_Motor *friLmotor;       // 左摩擦轮
  DJI_Motor *friRmotor;       // 右摩擦轮
  DJI_Motor *supplierMotor;   // 拨弹轮
  uint8_t shooterMode;        // 发射机构模式
  uint8_t supplierMode;       // 拨弹轮模式
  fp32 friSpeed;              // 摩擦轮速度
  fp32 shootFreq;             // 拨弹轮速度
  uint8_t stuckCount;         // 卡弹周期计数
  uint8_t startDelay;         // 卡弹计数开始前的延时
  uint8_t stuckProcessCount;  // 卡弹处理时间
  uint16_t maxHeat; // 最大热量
  uint16_t gunHeat; // 当前热量
  uint16_t heat_cooling; // 冷却速率
  uint8_t heatPerShoot; // 每发子弹的热量
#if SUPPLIER_ECD == 1
  int supplier_total_round;
  uint16_t supplier_ecd_buf[2];
  fp32 ecd_per_shoot;
#endif
} shooter;

extern shooter shooter1;

extern void shooterINIT(shooter *shooter, DJI_MotorGroup *group, DJI_Motor *friL, DJI_Motor *friR,
                        DJI_Motor *supplier, fp32 friSpeed, fp32 shootFreq, uint8_t heatPerShoot, fp32 ecd_per_shoot);
extern void shooterRun(shooter *shooter);
extern void getSupplierTotalEcd(shooter* shoot);
extern void shooterFreqControl(shooter *shooter);
extern uint8_t FriMotorSpdLvCtrl(void);
