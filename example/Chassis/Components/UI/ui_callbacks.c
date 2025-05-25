#include "ui_callbacks.h"

#include "main.h"
#include "refereeData_v1.7.h"
#include "tim.h"
#include "ui.h"
#include "usart.h"
volatile uint8_t ui_tx_cplt_flag = 0;

/**
 * @brief  用于UI库的发送接口，写到库的宏定义中
 * @param  message:
 * @param  length:
 */
void ui_Transmit(const uint8_t *message, int length) {
  if (__get_IPSR() != 0) {
    // 当前处于中断中，执行相应的中断处理逻辑
    while (ui_tx_cplt_flag == 1) {
    }
    ui_tx_cplt_flag = 1;
    HAL_UART_Transmit_DMA(&huart6, message, length);
  }
  else {
    // 当前处于主程序中，执行主程序逻辑
    HAL_UART_Transmit(&huart6, message, length, 10);
  }
}

/**
 * @brief  发送完成的回调，放在对应串口发送完成中断中
 */
void ui_TxCpltCallback() {
  ui_tx_cplt_flag = 0;
}

/**
 * @brief  用于第一次绘制ui,如果ui通过中断一直刷不出来，就要调这个函数
 * @param  htim:用于刷新ui的定时器
 */
void ui_DrawAtStartUP(TIM_HandleTypeDef *htim) {
  HAL_TIM_Base_Stop_IT(htim);
  for (uint8_t i = 0; i < 100; i++) {
    ui_self_id = robot_state.robot_id;
    ui_init_g();
    ui_update_g();
  }
  HAL_TIM_Base_Start_IT(htim);
}
