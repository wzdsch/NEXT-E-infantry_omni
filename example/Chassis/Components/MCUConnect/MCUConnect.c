/**
 * @File Name: MCUConnect.c
 * @brief
 *        双机通信将直接利用结构体的地址发送数据，会把结构体拆成若干个can报文发送，
 *        头报文stdid就是TxID，并且报文的[0],[1]是结构体的id,[2]是结构体大小,后续的报文stdid依次增加。
 *        报文收完后会存进fifo中，总共有三个fifo，每个fifo只存一个结构体的信息，在另外一个中断中对fifo解包
 * @Version : 2.0
 *
 * Date:2024-02-024       Author:Almost_Noob email:luodanwei03@outlook.com
 * 修改计划(完成末尾写1):
 */
#include "MCUConnect.h"

#include "DJI_Motor.h"
#include "MCUConnectStructs.h"
#include "Tools.h"
#include "math.h"
#include "referee.h"
#include "refereeData_v1.6.h"
#include "stdio.h"
#include "string.h"
#include "struct_typedef.h"
ChassisControl ChassisControlData;

/**
 * @brief  初始化双机通信结构体
 * @param  connection: 双机通信结构体地址
 * @param  canHandler: 双机通信can
 * @param  FIFO: can的FIFO
 * @param  MSG_PENDING:
 */
void connectionINIT(MCUConnection *connection, CAN_HandleTypeDef *canHandler, uint16_t TxID,
                    uint8_t FIFO) {
  connection->canHandler = canHandler;
  connection->FIFO = FIFO;
  connection->TxID = TxID;
  for (uint8_t i = 0; i < 3; i++) {
    connection->RxFIFO[i].fifoState = CONNECT_FIFO_RESET;
  }

  connection->txHandler.IDE = CAN_ID_STD;
  connection->txHandler.RTR = CAN_RTR_DATA;
  connection->txHandler.DLC = 0x08;
  connection->txHandler.StdId = TxID;

  CAN_FilterTypeDef connectFilterConfig;

  connectFilterConfig.FilterActivation = ENABLE;            // 使能过滤器
  connectFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;   // 掩码模式
  connectFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;  // 过滤器为16位
  connectFilterConfig.FilterIdHigh = 0x0000;  // 掩码全为0，则不过滤同时过滤器的id应最大
                                              // 以通过优先级机制防止接收电机数据
  connectFilterConfig.FilterIdLow = 0x0000;
  connectFilterConfig.FilterMaskIdHigh = 0x0000;
  connectFilterConfig.FilterMaskIdLow = 0x0000;

  if (canHandler == &hcan1) {                       // can1过滤器配置
    connectFilterConfig.FilterBank = 13;            // 使用过滤器13
    connectFilterConfig.SlaveStartFilterBank = 14;  // 从过can过滤器从14开始
  }
  else if (canHandler == &hcan2) {
    connectFilterConfig.FilterBank = 27;            // 使用过滤器27
    connectFilterConfig.SlaveStartFilterBank = 14;  // 从过can过滤器从14开始
  }

  if (FIFO == CAN_RX_FIFO0) {  // 如果用fifo0,那么在cube里也就是rx0中断
    connectFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    connection->MSG_PENDING = CAN_IT_RX_FIFO0_MSG_PENDING;
  }
  else if (FIFO == CAN_RX_FIFO1) {  // 如果用fifo1,那么在cube里也就是rx1中断
    connectFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    connection->MSG_PENDING = CAN_IT_RX_FIFO1_MSG_PENDING;
  }

  HAL_CAN_ConfigFilter(canHandler, &connectFilterConfig);             // 配置过滤器
  HAL_CAN_ActivateNotification(canHandler, connection->MSG_PENDING);  // 开始中断
  HAL_CAN_Start(canHandler);
}

/**
 * @brief  解包fifo写入
 * @param  connection:
 * @param  addr: 写入fifo的位置
 * @param  data: 写入的数据
 * @param  size: 写入的大小
 * @return uint8_t: 返回0为写入失败，fifo无空闲
 */
uint8_t connectionFifoWrite(MCUConnection *connection, uint16_t packID, uint8_t packSize,
                            uint16_t addr, uint8_t *data, uint8_t size) {

  if (addr == 0) {  // 写入地址为0说明是新的结构体接收，使用新的FIFO
    if (connection->RxFIFO[0].fifoState == CONNECT_FIFO_RESET) {
      connection->usingFIFO = 0;
      connection->RxFIFO[0].fifoState = CONNECT_FIFO_WRITING;
      connection->RxFIFO[0].packID = packID;
      connection->RxFIFO[0].pcakSize = packSize;
      memcpy(&(connection->RxFIFO[connection->usingFIFO].buffer[addr]), data, size);
      return 1;
    }
    else if (connection->RxFIFO[1].fifoState == CONNECT_FIFO_RESET) {
      connection->usingFIFO = 1;
      connection->RxFIFO[1].fifoState = CONNECT_FIFO_WRITING;
      connection->RxFIFO[1].packID = packID;
      connection->RxFIFO[1].pcakSize = packSize;
      memcpy(&(connection->RxFIFO[connection->usingFIFO].buffer[addr]), data, size);
      return 1;
    }
    else if (connection->RxFIFO[2].fifoState == CONNECT_FIFO_RESET) {
      connection->usingFIFO = 2;
      connection->RxFIFO[2].fifoState = CONNECT_FIFO_WRITING;
      connection->RxFIFO[2].packID = packID;
      connection->RxFIFO[2].pcakSize = packSize;
      memcpy(&(connection->RxFIFO[connection->usingFIFO].buffer[addr]), data, size);
      return 1;
    }
    else {  // 如果fifo用完

      return 0;
    }
  }
  else {  // 地址不为0，则向当前使用的fifo写入
    memcpy(&(connection->RxFIFO[connection->usingFIFO].buffer[addr]), data, size);
    if (connection->RxFIFO[connection->usingFIFO].pcakSize
        == addr + size) {  // 如果写入地址加上写入大小
                           // 等于结构体大小说明写完了
      connection->RxFIFO[connection->usingFIFO].fifoState = CONNECT_FIFO_DONE;
    }
    return 1;
  }
}

uint32_t packLostCount = 0;

/**
 * @brief  接收数据处理函数，在can中断中调用
 * @param  connection: 双机通信结构体地址
 * @param  group: 云台电机组结构体的地址，底盘使用写NULL
 */
void connectionRcceiveData(MCUConnection *connection) {
  HAL_CAN_GetRxMessage(connection->canHandler, connection->FIFO, &(connection->rxHandler),
                       connection->RXdata);
  HAL_CAN_ActivateNotification(connection->canHandler, connection->MSG_PENDING);
  static uint8_t dataSize = 0;
  static uint16_t packCount = 0;
  static uint16_t dataID = 0;
  static uint16_t lastStdID = 0;
  if (connection->rxHandler.StdId >= connection->TxID
      && connection->rxHandler.StdId <= connection->TxID + 100) {  // 如果是包id
    if (connection->rxHandler.StdId == connection->TxID) {  // 如果是新结构体数据，获得结构体id
      dataID = (connection->RXdata[0] << 8) + connection->RXdata[1];
      lastStdID = connection->rxHandler.StdId;
      dataSize = connection->RXdata[2];
      packCount = ceil((dataSize + 3) / 8.0f);
      connectionFifoWrite(connection, dataID, dataSize, 0, &(connection->RXdata[3]),
                          5);  // 将数据暂存
    }
    else if (connection->rxHandler.StdId - 1 == lastStdID) {  // 如果是连续的包
      lastStdID = connection->rxHandler.StdId;
      if (connection->rxHandler.StdId == connection->TxID + packCount - 1) {  // 如果是最后一个包
        connectionFifoWrite(connection, dataID, dataSize, 5 + ((packCount - 2) * 8),
                            connection->RXdata, connection->rxHandler.DLC);
      }
      else {  // 如果是中间的包
        connectionFifoWrite(connection, dataID, dataSize,
                            (5 + ((connection->rxHandler.StdId) - (connection->TxID) - 1) * 8),
                            connection->RXdata, 8);
      }
    }
    else {  // 如果丢包，初始化
      packLostCount++;
      dataSize = 0;
      packCount = 0;
      dataID = 0;
      lastStdID = 0;
      connection->RxFIFO[connection->usingFIFO].fifoState = CONNECT_FIFO_RESET;  // 复位fifo状态
    }
  }
}

/**
 * @brief  双机通信发送函数，调用则把结构体拆包发送
 * @param  connection:双机通信结构体
 * @param  dataAddr:要发送的数据的地址，对于结构体，可用(uint8_t *)(&structName)来完成地址转换
 * @param  size:要发送的数据大小，可用sizeof(structName)来获得
 * @param  dataID:数据的编号，每个结构体都应有一个专有的通用代号来区分
 */
void connectionSendData(MCUConnection *connection, uint8_t *dataAddr, uint8_t size,
                        uint16_t dataID) {
  uint8_t packCount = ceil((size + 3) / 8.0f);  // 计算发送所有的数据需要的包数量
  uint8_t txBuf[size + 2];                      // 创建发送缓存，防止数据变动
  txBuf[0] = dataID >> 8;
  txBuf[1] = dataID;
  txBuf[2] = size;
  memcpy(&txBuf[3], dataAddr, size);  // 复制数据，可以替换为dma
  for (uint8_t count = 0; count < packCount; count++) {
    connection->txHandler.StdId = connection->TxID + count;
    if ((size + 3) - (count * 8) >= 8) {  // 计算还没发送的数据
      connection->txHandler.DLC = 8;
    }
    else {
      connection->txHandler.DLC = (size + 3) - (count * 8);  // 如果剩下的不满8位，就改变包大小
    }
    // 这个阻塞视情况注释
    while (HAL_CAN_GetTxMailboxesFreeLevel(connection->canHandler)
           == 0) {  // 如果没有空闲邮箱，就阻塞
    }
    // 查找空闲邮箱，发送
    if ((connection->canHandler->Instance->TSR & CAN_TSR_TME0) != RESET) {
      HAL_CAN_AddTxMessage(connection->canHandler, &(connection->txHandler), &txBuf[count * 8],
                           (uint32_t *)CAN_TX_MAILBOX0);
    }
    else if ((connection->canHandler->Instance->TSR & CAN_TSR_TME1) != RESET) {
      HAL_CAN_AddTxMessage(connection->canHandler, &(connection->txHandler), &txBuf[count * 8],
                           (uint32_t *)CAN_TX_MAILBOX1);
    }
    else if ((connection->canHandler->Instance->TSR & CAN_TSR_TME2) != RESET) {
      HAL_CAN_AddTxMessage(connection->canHandler, &(connection->txHandler), &txBuf[count * 8],
                           (uint32_t *)CAN_TX_MAILBOX2);
    }
  }
}

/**
 * @brief  双机通信解包
 * @param  connection:
 */
void connectionUnpackData(MCUConnection *connection) {
  static uint8_t lastCheck = 0;
  static uint32_t time = 0;
  time++;
  if (connection->RxFIFO[lastCheck].fifoState == CONNECT_FIFO_DONE) {
    switch (connection->RxFIFO[lastCheck].packID) {
      case 0x8001:
        memcpy(&ChassisControlData, &(connection->RxFIFO[lastCheck].buffer),
               connection->RxFIFO[lastCheck].pcakSize);
        connection->RxFIFO[lastCheck].fifoState = CONNECT_FIFO_RESET;

        break;

      default:
        break;
    }
  }
  lastCheck++;
  if (lastCheck == 3) {
    lastCheck = 0;
  }
}

/**
 * @brief 发给云台的裁判系统数据结构体更新函数
 *
 * @param data
 */
void RefereeDataUpdate(Referee_data *data) {
  if (shoot_data.shooter_number == 1) {
    data->gunHeat1 = power_heat_data.shooter_17mm_1_barrel_heat;
  }
  else {
    data->gunHeat1 =
      power_heat_data.shooter_17mm_2_barrel_heat;  // 如果是42mm枪管，后面再加一个判断
  }
  if (shoot_data.initial_speed < 10.0f) {  // 如过初速度小于10m/s，则发送23m/s
    data->gunSpeed1 = 23.0f;
  }
  else {
    data->gunSpeed1 = shoot_data.initial_speed;
  }

  data->maxHeat = robot_state.shooter_barrel_heat_limit;
  data->heat_cooling = robot_state.shooter_barrel_cooling_value;

  if (robot_state.robot_id == 3 || robot_state.robot_id == 4 || robot_state.robot_id == 5
      || robot_state.robot_id == 7) {
    data->our_color = 'R';  // 己方为红色
  }
  else {
    data->our_color = 'B';  // 己方为蓝色
  }
}
