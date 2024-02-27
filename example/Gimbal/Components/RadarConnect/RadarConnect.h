#ifndef RADARCONNECT_H
#define RADARCONNECT_H
#include "struct_typedef.h"
#include "usart.h"
#pragma pack(1)

typedef struct radarUnpack {
  UART_HandleTypeDef *huart;
  uint8_t RXBuf[13];
  fp32 gimbalYaw;

  union {
    uint8_t rawRX[13];

    struct {
      uint8_t head;
      fp32 speedX;
      fp32 speedY;
      fp32 speedYAW;
    } data;
  } RX;
} RadarConnect;

#pragma pack()
extern RadarConnect radar1;

extern void RadarConnectINIT(RadarConnect *connect, UART_HandleTypeDef *huart);
extern void RadarConnect_EventCallBack(RadarConnect *connect, UART_HandleTypeDef *huart);
extern void RadarConnect_RxCpltCallBack(RadarConnect *connect, UART_HandleTypeDef *huart);
extern void RadarConnect_ErrorCallBack(RadarConnect *connect, UART_HandleTypeDef *huart);

#endif
