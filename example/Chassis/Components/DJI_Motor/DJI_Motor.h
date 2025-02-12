/**
 * @File Name: DJI_Motor.h
 * @brief
 *
 *
 * @Version : 1.0
 *
 * Date:2024.2024-02-02        Author:Almost_Noob email:luodanwei03@outlook.com
 * 备注:
 */
#ifndef __DJI_MOTOR_H
#define __DJI_MOTOR_H
#include "can.h"
#include "main.h"
#include "pid.h"
#include "struct_typedef.h"
#include <stdlib.h>

#define Current_Control 1  // 判断6020为电流环还是电压环控制，0为电压环

#define motorYaw_ID 0x207

#define YAW_REVERSE

/**
 * 还是保留下来
 *
 */
enum cycleMode {
  pid0 = 0,  // 仅使用pid0
  pidBoth,   // 串级pid
  custom,    // 自定义控制函数
};

typedef struct dji_motor DJI_Motor;

/**
 * @brief  can总线电机结构体
 */
typedef struct dji_motor {
  uint32_t ID;        // 电机ID
  uint8_t EN;         // 使能标志
  uint8_t dir;        // 电机正方向
  uint8_t cycleMode;  // 电机闭环形式（pid0仅使用pid0;pidBoth串级pid）

  fp32 target;  // 电机闭环目标值

  int16_t realSpeed;  // 电机实际转速
  fp32 realSpeedF;
  uint16_t realEcd;  // 电机实际编码(0-8191)
  fp32 realEcdF;
  uint16_t flagEcd;  // 电机编码软参考点

  int16_t realCurrent;  // 电机实际转矩电流
  fp32 realCurrentF;
  uint8_t temperature;  // 电机温度

  ///////////////////////////////////////////////////////////////////////////////////////////
  /*********************************闭环控制接口*********************************************/
  ///////////////////////////////////////////////////////////////////////////////////////////
  fp32 (*prePrecessHandler)(DJI_Motor *motor);   // 预处理函数地址
  fp32 preProcessResult;                         // 预处理函数结果
  fp32 (*customControl)(DJI_Motor *motor);       // 自定义控制函数地址
  fp32 customControlResult;                      // 自定义控制函数结果
  fp32 (*postPrecessHandler)(DJI_Motor *motor);  // 后处理函数地址
  fp32 postProcessResult;                        // 后处理函数结果

  fp32 *calculateResult;  // 要发送的数据地址
  ///////////////////////////////////////////////////////////////////////////////////////////
  /********************************PID闭环设置参数*******************************************/
  ///////////////////////////////////////////////////////////////////////////////////////////
  pids motorPid0;   // 电机pid0
  pids motorPid1;   // 电机pid1
  fp32 *pid0Ref;    // pid0的ref地址
  fp32 *pid0Set;    // pid0的set地址
  fp32 pidOutput0;  // pid0输出值

  fp32 *pid1Ref;    // pid1的ref地址
  fp32 *pid1Set;    // pid1的set地址
  fp32 pidOutput1;  // pid1输出值

} DJI_Motor;

/**
 * @brief
 * can总线电机组（最多8个电机为一组）一个can口为一组，MG6020ID只能为0x209-0x20B（够用了，然后懒得写别的id）
 */
typedef struct groupData {
  CAN_HandleTypeDef *canHandler;  // can
  uint16_t txEnFlag;  // 发送端口使能标识，有3位(0x0-0x7)，高有效，每一位按200，1ff，2ff控制使能
  CAN_TxHeaderTypeDef txHandler200;  // id为0x200的发送端口
  CAN_TxHeaderTypeDef txHandler1fe;  // id为0x1fe的发送端口  6020电流环标识符
  CAN_TxHeaderTypeDef txHandler2fe;  // id为0x2fe的发送端口
  CAN_TxHeaderTypeDef txHandler1ff;  // id为0x1ff的发送端口
  CAN_TxHeaderTypeDef txHandler2ff;  // id为0x2ff的发送端口
  CAN_RxHeaderTypeDef rxHandler;     // 接收端口
  uint32_t FIFO;                     // 接收FIFO    ac
  uint32_t MSG_PENDING;              // 接收中断标志
  uint8_t motorRXdata[8];            // 接收缓存
  uint8_t motorTXdata1ff[8];         // 发送缓存0x1ff
  uint8_t motorTXdata200[8];         // 发送缓存0x200
  uint8_t motorTXdata2ff[8];         // 发送缓存0x2ff
  DJI_Motor *list[11];               // 电机映射表
  // 在这里并没有加入0x1fe和0x2fe的发送缓存，因为判断发送缓存是依据的电机ID
} DJI_MotorGroup;

// 初始化按上下顺序调用
extern void DJI_MotorGroupInit(DJI_MotorGroup *group, CAN_HandleTypeDef *_canHandler,
                               uint32_t _FIFO);

extern void DJI_MotorInit(DJI_Motor *motor, uint32_t ID, uint8_t dir, uint8_t cycleMode,
                          int16_t flagEcd);

extern void DJI_MotorListAdd(DJI_MotorGroup *group, DJI_Motor *motor);

// 用户接口
extern void DJI_MotorPidSet(DJI_Motor *motor, pids *pid, uint8_t mode, fp32 piddatas[5], fp32 *ref,
                            fp32 *set);
extern void DJI_MotorCalculateResultSet(DJI_Motor *motor, fp32 *Result);
extern void DJI_MotorDisable(DJI_Motor *motor);
extern void DJI_MotorEnable(DJI_Motor *motor);
extern uint16_t DJI_MotorGetSoftEcd(DJI_Motor *motor);
extern void DJI_MotorSetTarget(DJI_Motor *motor, fp32 target);
extern void DJI_MotorPreProcessHandlerSet(DJI_Motor *motor,
                                          fp32 (*prePrecessHandler)(DJI_Motor *motor));
void DJI_MotorCustonControlSet(DJI_Motor *motor, fp32 (*custonControl)(DJI_Motor *motor));
extern void DJI_MotorPostProcessHandlerSet(DJI_Motor *motor,
                                           fp32 (*postPrecessHandler)(DJI_Motor *motor));
extern void DJI_MotorUpdateTXD(DJI_MotorGroup *group, DJI_Motor *motor, int16_t data);
extern void DJI_MotorReceiveMotordat(DJI_MotorGroup *group);
extern void DJI_MotorPidRUN(DJI_MotorGroup *group);
extern void DJI_MotorSendData(DJI_MotorGroup *group);

extern DJI_MotorGroup group1;
extern DJI_MotorGroup group2;
extern DJI_Motor motor2;
extern DJI_Motor motor4;
extern DJI_Motor motorYaw;

#endif
