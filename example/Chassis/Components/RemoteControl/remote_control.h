/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.c/h
  * @brief 遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  * 资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board
  tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include "main.h"
#include "stdint.h"
#define RC_FRAME_LENGTH 24u

#define RC_CH_VALUE_MIN    ((uint16_t)364)   // 最小值
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)  // 中间值
#define RC_CH_VALUE_MAX    ((uint16_t)1684)  // 最大值

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_NOT_DETECTED ((uint16_t)0)
#define RC_SW_UP           ((uint16_t)353)  // 起火步兵的控
// #define RC_SW_UP                  ((uint16_t)352)  // 另外一台步兵的控
#define RC_SW_MID                 ((uint16_t)1024)
#define RC_SW_DOWN                ((uint16_t)1695)
#define PC_Control                ((uint16_t)1)
#define T7_Control                ((uint16_t)0)
#define Pi                        ((fp32)3.14f)
#define switch_is_down(s)         (s == RC_SW_DOWN)
#define switch_is_mid(s)          (s == RC_SW_MID)
#define switch_is_up(s)           (s == RC_SW_UP)
#define switch_is_not_detected(s) (s == RC_SW_NOT_DETECTED)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W     ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S     ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A     ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D     ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL  ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q     ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E     ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R     ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F     ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G     ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z     ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X     ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C     ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V     ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B     ((uint16_t)1 << 15)

/* ----------------------- Data Struct ------------------------------------- */
/**********************************
ch[0] 右边遥杆左右方向
ch[1] 右边遥杆上下方向
ch[2] 左边遥杆左右方向
ch[3] 左边遥杆上下方向
ch[4] 左边滑轮
s[0]  右边开关
s[1]  左边开关
***********************************/
// typedef __packed struct {
//   UART_HandleTypeDef *huart;
//   uint8_t RC_rxd[18];
//   __packed struct {
//     int16_t ch[5];
//     char s[2];
//   } rc;
//   __packed struct {
//     int16_t x;
//     int16_t y;
//     int16_t z;
//     uint8_t press_l;
//     uint8_t press_r;
//   } mouse;
//   __packed struct { uint16_t v; } key;

//} RC_ctrl_t;

typedef __packed struct {
  UART_HandleTypeDef *huart;
  uint8_t sbus_buf[24];

  __packed struct {
    int16_t SBUS_channels[10];
  } sbus;
} RC_SBUS_t;

// extern RC_ctrl_t rc_ctrl;
extern RC_SBUS_t RC_sbus;

// void RemoteControlINIT(RC_ctrl_t *rc, UART_HandleTypeDef *huart);

// void RC_EventCallback(RC_ctrl_t *rc, UART_HandleTypeDef *huart);
// void RC_ErrorCallback(RC_ctrl_t *rc, UART_HandleTypeDef *huart);
// void RC_ReceiveCpltCallback(RC_ctrl_t *rc, UART_HandleTypeDef *huart);

void RemoteControlINIT(RC_SBUS_t *sbus, UART_HandleTypeDef *huart);
void RC_EventCallback(RC_SBUS_t *sbus, UART_HandleTypeDef *huart);
void RC_ErrorCallback(RC_SBUS_t *sbus, UART_HandleTypeDef *huart);
void RC_ReceiveCpltCallback(RC_SBUS_t *sbus, UART_HandleTypeDef *huart);

void SBUS_Unpack(RC_SBUS_t *sbus);

#endif
