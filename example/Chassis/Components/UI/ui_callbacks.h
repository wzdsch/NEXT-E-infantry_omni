#ifndef UI_CALLBACKS_H
#define UI_CALLBACKS_H
#include "struct_typedef.h"
#include "main.h"
volatile extern uint8_t ui_tx_cplt_flag;

extern void ui_Transmit(const uint8_t *message, int length);

extern void ui_TxCpltCallback(void);
extern void ui_DrawAtStartUP(TIM_HandleTypeDef* htim);

#endif
