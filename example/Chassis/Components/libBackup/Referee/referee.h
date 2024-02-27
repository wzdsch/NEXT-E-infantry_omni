#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"

typedef struct referee_data {
  UART_HandleTypeDef *huart;
  uint8_t REF_rxd[130];
} referee;

extern referee referee1;

extern void refereeINIT(referee *referee, UART_HandleTypeDef *huart);
extern void refereeRxCpltCallBack(referee *referee, UART_HandleTypeDef *huart);
extern void refereeRxEventCallBack(referee *referee, UART_HandleTypeDef *huart);
void referee_unpack(referee *referee);

#endif
