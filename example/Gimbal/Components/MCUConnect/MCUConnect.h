#ifndef __MCU_CONNECT_H
#define __MCU_CONNECT_H

#include "DJI_Motor.h"
#include "can.h"
#include "main.h"

enum CONNECT_FIFO_STATE {
  CONNECT_FIFO_RESET = 0,  // fifo空
  CONNECT_FIFO_WRITING,    // fifo写入中
  CONNECT_FIFO_DONE,       // fifo待解包
};

typedef struct ConnectFifo {
  uint8_t fifoState;  // fifo状态

  uint8_t pcakSize;  // 包大小
  uint16_t packID;   // 包id
  uint8_t buffer[256];
} connectFifo;


enum car_MODE {
  STOP = 0,
  FREE = 1,
  FOLLOW = 2,
  VISION = 3,
};




typedef struct connectData {

  CAN_HandleTypeDef *canHandler;  // can
  uint16_t TxID;  // 发送起始ID，会占用该id后的若干id号，注意id分配
  CAN_TxHeaderTypeDef txHandler;  // 发送端口
  CAN_RxHeaderTypeDef rxHandler;  // 接收端口
  uint32_t FIFO;                  // 接收FIFO
  uint32_t MSG_PENDING;           // 接收中断标志
  uint8_t RXdata[8];              // 包接收缓存
  connectFifo RxFIFO[3];          // 解包缓存
  uint8_t usingFIFO;              // 正在使用的fifo编号
} MCUConnection;

extern void connectionINIT(MCUConnection *connection, CAN_HandleTypeDef *canHandler, uint16_t TxID,
                           uint8_t FIFO);
extern void connectionRcceiveData(MCUConnection *connection);
extern void connectionSendData(MCUConnection *connection, uint8_t *dataAddr, uint8_t size,
                               uint16_t dataID);
extern void connectionUnpackData(MCUConnection *connection);

extern MCUConnection connect;

#endif
