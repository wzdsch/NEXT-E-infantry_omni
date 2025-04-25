#ifndef SUPERCAP_H
#define SUPERCAP_H

#include "main.h"
#include "struct_typedef.h"
#include "DJI_Motor.h"
#include "referee.h"
#include "refereeData_v1.6.h"
#include "math.h"
#include "addons.h"

#define SUPERCAP_RX_HEAD 0xF0F0
#define SUPERCAP_TX_HEAD 0xF1F2

// 超电接收结构体
typedef struct {
	uint16_t head;			// 包头 0xF0F0
    uint16_t raw_cap_power; // 原始值，单位0.01w
    fp32 max_cap_power; // 电容最大提供功率，单位w
    uint16_t cap_percent;   // 电容剩余容量
    fp32 input_power;   // 总输入功率
    uint16_t total_energy;  // 总消耗能量
    uint16_t err_code; // 0无错误
	uint8_t rx_buf[10];
}supercap_rx_t;

#pragma pack(1)

// 超电发送结构体
typedef struct {
  uint16_t head;                  // 包头 0xF1F1
  uint8_t chassis_power_state;    // 底盘供电状态
  uint16_t chassis_power_limit;   // 底盘功率限制
} supercap_tx_t;

#pragma pack()

extern supercap_rx_t supercap_rxD;
extern supercap_tx_t supercap_txD;

fp32 Supercap_powerlimit(DJI_Motor *motor);

void Supercap_unpack(supercap_rx_t *supercap);

void Supercap_transmit(UART_HandleTypeDef *huart, supercap_tx_t *supercap_tx);

void Supercap_update_txd(supercap_tx_t *supercap_tx, robot_state_t *const robot_state);

#endif
