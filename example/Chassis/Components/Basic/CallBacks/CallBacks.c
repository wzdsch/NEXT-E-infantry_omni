#include "CallBacks.h"

#include "MCUConnect.h"
#include "MCUConnectStructs.h"
#include "Tools.h"
#include "addOns.h"
#include "chassis.h"
#include "main.h"
#include "referee.h"
#include "refereeData_v1.7.h"
#include "struct_typedef.h"
#include "ui.h"
#include "vofa.h"
#if IF_WITH_SUPERCAP == 1
#include "SuperCap.h"
#endif
#include "ui_app.h"

uint8_t vofa_mode = 0;
#if IF_WITH_SUPERCAP == 1
extern uint8_t supercap_rx_flg;
#endif

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim10) {  // UI发送100ms@10Hz 优先级低！！！！！！！
    ui_init_g();
#if IF_WITH_SUPERCAP == 1
    ui_g_dynamic_graph_super_cap_persent->number = supercap_rxD.cap_percent;
#else
    ui_g_dynamic_graph_super_cap_persent->number = 0;
#endif
    if (ChassisControlData.vision_tracking == 1) {
      ui_g_aim_aim_line_l->color = 2;
      ui_g_aim_aim_line_r->color = 2;
    }
    else {
      ui_g_aim_aim_line_l->color = 3;
      ui_g_aim_aim_line_r->color = 3;
    }
    ui_update_g();
  }
  if (htim == &htim11) {  // 底盘pid计算与发送1ms@1000Hz
    
  }
  if (htim == &htim12) {  // 双机通信发送,裁判系统解包10ms@100Hz
    referee_unpack_fifo_data();
    RefereeDataUpdate(&RefereeData);
    connectionSendData(&connect, (uint8_t *)&RefereeData, sizeof(RefereeData), RefereeData_ID);
#if IF_WITH_SUPERCAP == 1
    Supercap_unpack(&supercap_rxD);
    Supercap_update_txd(&supercap_txD, &robot_state);
    Supercap_transmit(&huart1, &supercap_txD);
#endif
  }
  if (htim == &htim13) {  // 双机通讯解包0.67ms@1500Hz
    connectionUnpackData(&connect);
  }
  if (htim == &htim14) {  // 底盘解算2ms@500Hz
    chassisRun(&chassis1, ChassisControlData.speedx, ChassisControlData.speedy,
               ChassisControlData.speedz, DJI_MotorGetSoftEcd(&motorYaw));
    chassisChangeMode(&chassis1, ChassisControlData.mode);
    DJI_MotorPidRUN(chassis1.group);
    DJI_MotorSendData(chassis1.group);
  }
}

uint16_t receive_count = 0;
uint16_t error_count = 0;

// 串口接收完成中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart1) {
#if IF_WITH_SUPERCAP == 1
    supercap_rx_flg = 1;
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, supercap_rxD.rx_buf, sizeof(supercap_rxD.rx_buf)); // 超电接收
#endif
    receive_count++;
  }
}

// 串口错误中断
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart1) {
#if IF_WITH_SUPERCAP == 1
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, supercap_rxD.rx_buf, sizeof(supercap_rxD.rx_buf));
#endif
    error_count++;
  }
}

// 串口事件中断
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart == &huart1) {
#if IF_WITH_SUPERCAP == 1
    supercap_rx_flg = 1;
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, supercap_rxD.rx_buf, sizeof(supercap_rxD.rx_buf));
#endif
    receive_count++;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart == &huart6) {
    ui_TxCpltCallback();
  }
}

// can中断0
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (hcan == &hcan1) {
    DJI_MotorReceiveMotordat(&group2);  // 接收yaw轴电机的数据
  }
  if (hcan == &hcan2) {
    DJI_MotorReceiveMotordat(&group1);  // 接收底盘电机的数据
  }
}

// can中断1
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (hcan == &hcan1) {
    connectionRcceiveData(&connect);
  }
  if (hcan == &hcan2) {
  }
}
