/**
 * @File Name: DJI_Motor.c
 * @brief 代码封装了大疆C620,C610电调、GM6020电机的can驱动程序
 *
 * @Version : 2.0
 *
 * Date:2024.2024-02-02        Author:Almost_Noob email:luodanwei03@outlook.com
 * 备注:
 * 硬件要求：使能can1或can1、can2同时使能，不能单独使用can2（can2是can1的从设备，单独用不了）
 *  两个can公用28个过滤器，can1用前14个，can2用后14个；
 *  本库占用多个can过滤器，注意。
 */

#include "DJI_Motor.h"

#include "Tools.h"
#include "addOns.h"
#include "can.h"
#include "pid.h"

/******************************************************************************************/
/***********************************初始化函数**********************************************/
/******************************************************************************************/

/**
 * @brief  电机组基本数据初始化
 * @param  group: 电机组地址
 * @param  _canHandler: 电机组can端口
 * @param  _FIFO: can端口的FIFO,决定了数据在哪个can中断里接收
 */
void DJI_MotorGroupInit(DJI_MotorGroup *group, CAN_HandleTypeDef *_canHandler, uint32_t _FIFO) {
  group->canHandler = _canHandler;
  group->FIFO = _FIFO;
  // can过滤器配置
  CAN_FilterTypeDef sFilterConfig;

  for (uint8_t i = 0; i < 3; i++) {  // 循环配置过滤器0(14)-2(16),写入大疆电机所有的canID
    sFilterConfig.FilterActivation = ENABLE;                  // 过滤器开启
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;         // 列表模式
    sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;        // 16位
    sFilterConfig.FilterIdHigh = (0x201 + (i * 4)) << 5;      // 0x201,205,209
    sFilterConfig.FilterIdLow = (0x202 + (i * 4)) << 5;       // 0x202,206,20A
    sFilterConfig.FilterMaskIdHigh = (0x203 + (i * 4)) << 5;  // 0x203,207,20B
    if ((0x204 + (i * 4)) == 0x20C) {
      sFilterConfig.FilterMaskIdLow =
        0x201;  // 这里多写了一个0x201，是防止0x000通过过滤器，同时根据过滤器优先级
                // 0x201会从编号小的过滤器通过，所以这里仅仅是占位用
    }
    else {
      sFilterConfig.FilterMaskIdLow = (0x204 + (i * 4)) << 5;  // 0x204,208
    }

    if (_canHandler == &hcan1) {  // 使用can1,过滤器使用0-13
      sFilterConfig.FilterBank = i;
      sFilterConfig.SlaveStartFilterBank = 14;  // 顾名思义，从can使用的过滤器第一个id号,
                                                // can1过滤器不够用可以改，但相应的can2的会减少
    }
    else if (_canHandler == &hcan2) {  // 使用can2，过滤器使用14-27
      sFilterConfig.FilterBank = 14 + i;
      sFilterConfig.SlaveStartFilterBank = 14;
    }

    if (_FIFO == CAN_RX_FIFO0) {  // 如果用fifo0,那么在cube里也就是rx0中断
      sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
      group->MSG_PENDING = CAN_IT_RX_FIFO0_MSG_PENDING;
    }
    else if (_FIFO == CAN_RX_FIFO1) {  // 如果用fifo1,那么在cube里也就是rx1中断
      sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
      group->MSG_PENDING = CAN_IT_RX_FIFO1_MSG_PENDING;
    }

    HAL_CAN_ConfigFilter(_canHandler, &sFilterConfig);
  }

  sFilterConfig.FilterActivation = ENABLE;            // 过滤器开启
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;   // 列表模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;  // 16位
  sFilterConfig.FilterIdHigh = 0x1ff << 5;
  sFilterConfig.FilterIdLow = 0x200 << 5;
  sFilterConfig.FilterMaskIdHigh = 0x2ff << 5;
  sFilterConfig.FilterMaskIdLow = 0x200 << 5;
  if (_canHandler == &hcan1) {  // 使用can1,过滤器使用0-13
    sFilterConfig.FilterBank = 4;
    sFilterConfig.SlaveStartFilterBank = 14;  // 顾名思义，从can使用的过滤器第一个id号,
                                              // can1过滤器不够用可以改，但相应的can2的会减少
  }
  else if (_canHandler == &hcan2) {  // 使用can2，过滤器使用14-27
    sFilterConfig.FilterBank = 17;
    sFilterConfig.SlaveStartFilterBank = 14;
  }

  if (_FIFO == CAN_RX_FIFO0) {  // 如果用fifo0,那么在cube里也就是rx0中断
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    group->MSG_PENDING = CAN_IT_RX_FIFO0_MSG_PENDING;
  }
  else if (_FIFO == CAN_RX_FIFO1) {  // 如果用fifo1,那么在cube里也就是rx1中断
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    group->MSG_PENDING = CAN_IT_RX_FIFO1_MSG_PENDING;
  }

  HAL_CAN_ConfigFilter(_canHandler, &sFilterConfig);
  // 开启can中断
  HAL_CAN_ActivateNotification(_canHandler, group->MSG_PENDING);
  HAL_CAN_Start(_canHandler);

  // can发送，接收初始化
  group->txHandler200.IDE = CAN_ID_STD;
  group->txHandler200.RTR = CAN_RTR_DATA;
  group->txHandler200.DLC = 0x08;
  group->txHandler200.StdId = 0x200;

  group->txHandler1ff.IDE = CAN_ID_STD;
  group->txHandler1ff.RTR = CAN_RTR_DATA;
  group->txHandler1ff.DLC = 0x08;
  group->txHandler1ff.StdId = 0x1ff;

  group->txHandler2ff.IDE = CAN_ID_STD;
  group->txHandler2ff.RTR = CAN_RTR_DATA;
  group->txHandler2ff.DLC = 0x08;
  group->txHandler2ff.StdId = 0x2ff;

  // 电机注册表初始化
  for (uint8_t i = 0; i < 11; i++) {
    group->list[i] = NULL;
  }
}

/**
 * @brief  电机初始化,针对单个电机
 * @param  motor: 电机结构体地址
 * @param  ID: 电机ID
 * @param  dir: 电机正方向（不是0就是1）
 * @param  cycleMode: 电机闭环形式（pid0仅使用pid0;pidBoth串级pid）
 * @param  flagEcd: 电机编码值的软零点
 */
void DJI_MotorInit(DJI_Motor *motor, uint32_t ID, uint8_t dir, uint8_t cycleMode, int16_t flagEcd) {
  motor->ID = ID;
  motor->dir = dir;
  motor->cycleMode = cycleMode;
  motor->flagEcd = flagEcd;
  motor->postPrecessHandler = NULL;
  motor->prePrecessHandler = NULL;
  motor->pid0Ref = NULL;
  motor->pid0Set = NULL;
  motor->pid1Ref = NULL;
  motor->pid1Set = NULL;
}

/**
 * @brief  整理电机所需要的txHandler
 * @param  group: 电机组结构体
 */
void DJI_MotorListClassicify(DJI_MotorGroup *group) {
  group->txEnFlag = 0;
  uint8_t i = 0;
  uint8_t tx200 = 0;
  uint8_t tx1ff = 0;
  uint8_t tx2ff = 0;
  for (i = 0; i < 4; i++) {
    if (group->list[i] != NULL) {
      tx200 += (group->list[i]->ID - 0x200);
    }
  }
  if (tx200 != 0) {
    group->txEnFlag += 0x100;
  }

  for (i = 4; i < 8; i++) {
    if (group->list[i] != NULL) {
      tx1ff += (group->list[i]->ID - 0x200);
    }
  }
  if (tx1ff != 0) {
    group->txEnFlag += 0x010;
  }

  for (i = 8; i < 11; i++) {
    if (group->list[i] != NULL) {
      tx2ff += (group->list[i]->ID - 0x200);
    }
  }
  if (tx2ff != 0) {
    group->txEnFlag += 0x001;
  }
}

/**
 * @brief  将电机的地址写入电机组的电机注册表
 * @param  group: 电机组地址
 * @param  motor: 电机结构体地址
 */
void DJI_MotorListAdd(DJI_MotorGroup *group, DJI_Motor *motor) {
  // 依据电机id算出电机编号，并把对应的电机结构体地址存入数组
  //  3508，2006电机的ID是0x201-0x208,处理对应到数组的0-7
  //  6020电机ID是0x205-0x20B,这里只使用0x209-0x20B,对应数组的8-10
  if (motor != NULL) {
    group->list[(motor->ID) - 0x200 - 0x1] = motor;
  }
  DJI_MotorListClassicify(group);
}

/******************************************************************************************/
/************************************内部处理***********************************************/
/******************************************************************************************/

/**
 * @brief  更新电机组中的发送缓存
 * @param  group: 电机组结构体地址
 * @param  motor: 电机结构体地址
 * @param  data: 发送的数据
 */
void DJI_MotorUpdateTXD(DJI_MotorGroup *group, DJI_Motor *motor, int16_t data) {
  uint8_t temp = motor->ID - 0x200;
  // 电机id为0x200+1,2,3,4
  //-1*2后得到对应的0，2，4，6就是数组中每个电机数据的起始位
  //  1,2,3,4对应0x200
  //  5,6,7,8对应0x1ff
  //  9，10，11对应0x2ff
  switch (temp) {
    case 1:
    case 2:
    case 3:
    case 4:  // 标识为0x200
      temp = (temp - 1) * 2;
      group->motorTXdata200[temp] = (data >> 8);
      group->motorTXdata200[temp + 1] = data;
      break;

    case 5:
    case 6:
    case 7:
    case 8:  // 标识为0x1ff
      temp = (temp - 5) * 2;
      group->motorTXdata1ff[temp] = (data >> 8);
      group->motorTXdata1ff[temp + 1] = data;
      break;

    case 9:
    case 10:
    case 11:  // 标识为0x2ff,针对MG6020
      temp = (temp - 9) * 2;
      group->motorTXdata2ff[temp] = (data >> 8);
      group->motorTXdata2ff[temp + 1] = data;
    default:
      break;
  }
}

/**
 * @brief 向电机发送数据，这里的做法可能会存在发送失败的问题，需要改进，
 *        改完之后，DJI_MotorListClassicify()函数应弃用
 * @param  group:
 */
void DJI_MotorSendData(DJI_MotorGroup *group) {
  // 写入缓存后发送
  switch (group->txEnFlag) {
    case 0x001:
      // HAL_CAN_AddTxMessage(group->canHandler,&(group->txHandler200),group->motorTXdata200,(uint32_t
      // *)CAN_TX_MAILBOX0);
      // HAL_CAN_AddTxMessage(group->canHandler,&(group->txHandler1ff),group->motorTXdata1ff,(uint32_t
      // *)CAN_TX_MAILBOX1);
      HAL_CAN_AddTxMessage(group->canHandler, &(group->txHandler2ff), group->motorTXdata2ff,
                           (uint32_t *)CAN_TX_MAILBOX2);
      break;

    case 0x010:
      // HAL_CAN_AddTxMessage(group->canHandler,&(group->txHandler200),group->motorTXdata200,(uint32_t
      // *)CAN_TX_MAILBOX0);
      HAL_CAN_AddTxMessage(group->canHandler, &(group->txHandler1ff), group->motorTXdata1ff,
                           (uint32_t *)CAN_TX_MAILBOX1);
      // HAL_CAN_AddTxMessage(group->canHandler,&(group->txHandler2ff),group->motorTXdata2ff,(uint32_t
      // *)CAN_TX_MAILBOX2);
      break;

    case 0x011:
      // HAL_CAN_AddTxMessage(group->canHandler,&(group->txHandler200),group->motorTXdata200,(uint32_t
      // *)CAN_TX_MAILBOX0);
      HAL_CAN_AddTxMessage(group->canHandler, &(group->txHandler1ff), group->motorTXdata1ff,
                           (uint32_t *)CAN_TX_MAILBOX1);
      HAL_CAN_AddTxMessage(group->canHandler, &(group->txHandler2ff), group->motorTXdata2ff,
                           (uint32_t *)CAN_TX_MAILBOX2);
      break;

    case 0x100:
      HAL_CAN_AddTxMessage(group->canHandler, &(group->txHandler200), group->motorTXdata200,
                           (uint32_t *)CAN_TX_MAILBOX0);
      // HAL_CAN_AddTxMessage(group->canHandler,&(group->txHandler1ff),group->motorTXdata1ff,(uint32_t
      // *)CAN_TX_MAILBOX1);
      // HAL_CAN_AddTxMessage(group->canHandler,&(group->txHandler2ff),group->motorTXdata2ff,(uint32_t
      // *)CAN_TX_MAILBOX2);
      break;

    case 0x101:
      HAL_CAN_AddTxMessage(group->canHandler, &(group->txHandler200), group->motorTXdata200,
                           (uint32_t *)CAN_TX_MAILBOX0);
      // HAL_CAN_AddTxMessage(group->canHandler,&(group->txHandler1ff),group->motorTXdata1ff,(uint32_t
      // *)CAN_TX_MAILBOX1);
      HAL_CAN_AddTxMessage(group->canHandler, &(group->txHandler2ff), group->motorTXdata2ff,
                           (uint32_t *)CAN_TX_MAILBOX2);
      break;

    case 0x110:
      HAL_CAN_AddTxMessage(group->canHandler, &(group->txHandler200), group->motorTXdata200,
                           (uint32_t *)CAN_TX_MAILBOX0);
      HAL_CAN_AddTxMessage(group->canHandler, &(group->txHandler1ff), group->motorTXdata1ff,
                           (uint32_t *)CAN_TX_MAILBOX1);
      // HAL_CAN_AddTxMessage(group->canHandler,&(group->txHandler2ff),group->motorTXdata2ff,(uint32_t
      // *)CAN_TX_MAILBOX2);
      break;

    case 0x111:
      HAL_CAN_AddTxMessage(group->canHandler, &(group->txHandler200), group->motorTXdata200,
                           (uint32_t *)CAN_TX_MAILBOX0);
      HAL_CAN_AddTxMessage(group->canHandler, &(group->txHandler1ff), group->motorTXdata1ff,
                           (uint32_t *)CAN_TX_MAILBOX1);
      HAL_CAN_AddTxMessage(group->canHandler, &(group->txHandler2ff), group->motorTXdata2ff,
                           (uint32_t *)CAN_TX_MAILBOX2);
      break;
  }
}

/**
 * @brief  计算pid(速度环和角度环)
 * @param  group: 电机组结构体地址
 */
void DJI_MotorPidRUN(DJI_MotorGroup *group) {
  uint8_t i = 0;
  for (i = 0; i < 11; i++) {  // 对列表中的所有电机计算pid,写入对应的发送缓存
    if (group->list[i] == NULL) {
      continue;
    }
    if (group->list[i]->EN == 0) {  // 电机失能，直接写0
      DJI_MotorUpdateTXD(group, group->list[i], 0);
      continue;
    }

    // 如果有预处理函数，就执行
    if (*(group->list[i]->prePrecessHandler) != NULL) {
      group->list[i]->preProcessResult = (group->list[i]->prePrecessHandler)(group->list[i]);
    }
    // pid计算
    switch (group->list[i]->cycleMode) {
      case pid0:  // 只计算一个pid
        group->list[i]->pidOutput0 = PID_calc(
          &(group->list[i]->motorPid0), *(group->list[i]->pid0Ref), *(group->list[i]->pid0Set));
        break;

      case pidBoth:  // 算两个pid
        group->list[i]->pidOutput0 = PID_calc(
          &(group->list[i]->motorPid0), *(group->list[i]->pid0Ref), *(group->list[i]->pid0Set));
        group->list[i]->pidOutput1 = PID_calc(
          &(group->list[i]->motorPid1), *(group->list[i]->pid1Ref), *(group->list[i]->pid1Set));
        break;

      default:
        break;
    }
    ////如果有后处理函数，就执行
    if (*(group->list[i]->postPrecessHandler) != NULL) {
      group->list[i]->postProcessResult = (group->list[i]->postPrecessHandler)(group->list[i]);
    }
    DJI_MotorUpdateTXD(group, group->list[i], floatTOint(*(group->list[i]->calculateResult)));
  }
}

/**
 * @brief  对电机的反馈数据经行分类，写入结构体
 * @param  group: 电机组地址
 * @param  motor: 电机地址
 */
void DJI_MotorClassifyMotorData(DJI_MotorGroup *group, DJI_Motor *motor) {
  motor->realEcd = (group->motorRXdata[0] << 8);
  motor->realEcd += group->motorRXdata[1];
  motor->realEcdF = motor->realEcd;

  motor->realSpeed = (group->motorRXdata[2] << 8);
  motor->realSpeed += group->motorRXdata[3];
  motor->realSpeedF = motor->realSpeed;

  motor->realCurrent = (group->motorRXdata[4] << 8);
  motor->realCurrent += group->motorRXdata[5];
  motor->realCurrentF = motor->realCurrent;

  motor->temperature = group->motorRXdata[6];
}

/**
 * @brief  获取、处理CAN电机数据，在对应的CAN接收中断中调用
 * @param  group: 电机组结构体地址
 */
void DJI_MotorReceiveMotordat(DJI_MotorGroup *group) {
  HAL_CAN_GetRxMessage(group->canHandler, group->FIFO, &(group->rxHandler), group->motorRXdata);
  HAL_CAN_ActivateNotification(group->canHandler, group->MSG_PENDING);
  // 根据电机ID对电机结构体经行赋值，建立了一个映射表list,直接由id可查找到对应的电机结构体地址
  uint8_t temp = (group->rxHandler.StdId) - 0x200 - 0x1;  // 由电机ID得到映射表的序号
  DJI_MotorClassifyMotorData(group, group->list[temp]);   // 传入对应的电机结构体
}

/******************************************************************************************/
/************************************用户接口***********************************************/
/******************************************************************************************/

/**
 * @brief  设置电机结构体的pid参数以及pid数据源
 * @param  motor: 电机结构体
 * @param  pid: pid结构体地址
 * @param  mode: pid模式
 * @param  piddatas: pid参数数组
 * @param  ref: pid参考量地址
 * @param  set: pid目标量地址
 */
void DJI_MotorPidSet(DJI_Motor *motor, pids *pid, uint8_t mode, fp32 piddatas[5], fp32 *ref,
                     fp32 *set) {
  if (ref != NULL) {  // 设置pid的数据源
    if (pid == &(motor->motorPid0)) {
      motor->pid0Ref = ref;
    }
    if (pid == &(motor->motorPid1)) {
      motor->pid1Ref = ref;
    }
  }
  if (set != NULL) {
    if (pid == &(motor->motorPid0)) {
      motor->pid0Set = set;
    }
    if (pid == &(motor->motorPid1)) {
      motor->pid1Set = set;
    }
  }
  if (piddatas != NULL) {
    pidINIT(pid, mode, piddatas[0], piddatas[1], piddatas[2], piddatas[3], piddatas[4]);
  }
}

void DJI_MotorCalculateResultSet(DJI_Motor *motor, fp32 *Result) {
  if (Result != NULL) {
    motor->calculateResult = Result;
  }
}

/**
 * @brief  失能电机
 * @param  motor: 电机结构体地址
 */
void DJI_MotorDisable(DJI_Motor *motor) {
  motor->EN = 0;
}

/**
 * @brief  使能电机
 * @param  motor: 电机结构体地址
 */
void DJI_MotorEnable(DJI_Motor *motor) {
  if (motor->EN == 0) {
    PID_clear(&(motor->motorPid0));  // 清除pid
    PID_clear(&(motor->motorPid1));
  }
  motor->EN = 1;
}

/**
 * @brief  获得电机的当前软编码值
 * @param  motor: 电机结构体地址
 * @return uint16_t: 电机软编码(0-8191)
 */
uint16_t DJI_MotorGetSoftEcd(DJI_Motor *motor) {
  if (motor->realEcd >= motor->flagEcd) {
    return (motor->realEcd - motor->flagEcd);
  }
  else {
    return (motor->realEcd - motor->flagEcd + 8191);
  }
}

/**
 * @brief  设置电机闭环目标值
 * @param  motor:
 * @param  target:
 */
void DJI_MotorSetTarget(DJI_Motor *motor, fp32 target) {
  // 根据dir改变电机转动方向
  if (motor->dir) {
    target = -target;
  }
  motor->target = target;
}

void DJI_MotorPreProcessHandlerSet(DJI_Motor *motor, fp32 (*prePrecessHandler)(DJI_Motor *motor)) {
  motor->prePrecessHandler = prePrecessHandler;
}

void DJI_MotorPostProcessHandlerSet(DJI_Motor *motor,
                                    fp32 (*postPrecessHandler)(DJI_Motor *motor)) {
  motor->postPrecessHandler = postPrecessHandler;
}
