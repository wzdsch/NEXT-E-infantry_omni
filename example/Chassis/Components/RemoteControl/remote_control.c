#include "remote_control.h"

#include "main.h"

void RemoteControlINIT(RC_ctrl_t *rc, UART_HandleTypeDef *huart) {
  rc->huart = huart;
  HAL_UART_Receive_DMA(huart, (uint8_t *)rc->RC_rxd, RC_FRAME_LENGTH);
}

void RC_EventCallback(RC_ctrl_t *rc, UART_HandleTypeDef *huart) {
  if (huart == rc->huart) {
    HAL_UART_Receive_DMA(rc->huart, (uint8_t *)rc->RC_rxd, RC_FRAME_LENGTH);
  }
}

void RC_ErrorCallback(RC_ctrl_t *rc, UART_HandleTypeDef *huart) {
  if (huart == rc->huart) {
    __HAL_UART_CLEAR_FEFLAG(rc->huart);
    HAL_UART_Receive_DMA(rc->huart, (uint8_t *)rc->RC_rxd, RC_FRAME_LENGTH);
  }
}

int16_t RC_LimitVal(int16_t val) {
  if (val >= 660) {
    val = 660;
  }
  else if (val <= -660) {
    val = -660;
  }
  return val;
}

void RC_Unpack(RC_ctrl_t *rc) {
  rc->rc.ch[0] = (rc->RC_rxd[0] | (rc->RC_rxd[1] << 8)) & 0x07ff;         //!< Channel 0
  rc->rc.ch[1] = ((rc->RC_rxd[1] >> 3) | (rc->RC_rxd[2] << 5)) & 0x07ff;  //!< Channel 1
  rc->rc.ch[2] = ((rc->RC_rxd[2] >> 6) | (rc->RC_rxd[3] << 2) |           //!< Channel 2
                  (rc->RC_rxd[4] << 10))
                 & 0x07ff;
  rc->rc.ch[3] = ((rc->RC_rxd[4] >> 1) | (rc->RC_rxd[5] << 7)) & 0x07ff;  //!< Channel 3
  rc->rc.s[0] = ((rc->RC_rxd[5] >> 4) & 0x0003);                          //!< Switch left
  rc->rc.s[1] = ((rc->RC_rxd[5] >> 4) & 0x000C) >> 2;                     //!< Switch right
  rc->mouse.x = rc->RC_rxd[6] | (rc->RC_rxd[7] << 8);                     //!< Mouse X axis
  rc->mouse.y = rc->RC_rxd[8] | (rc->RC_rxd[9] << 8);                     //!< Mouse Y axis
  rc->mouse.z = rc->RC_rxd[10] | (rc->RC_rxd[11] << 8);                   //!< Mouse Z axis
  rc->mouse.press_l = rc->RC_rxd[12];                                     //!< Mouse Left Is Press ?
  rc->mouse.press_r = rc->RC_rxd[13];                     //!< Mouse Right Is Press ?
  rc->key.v = rc->RC_rxd[14] | (rc->RC_rxd[15] << 8);     //!< KeyBoard value
  rc->rc.ch[4] = rc->RC_rxd[16] | (rc->RC_rxd[17] << 8);  // NULL

  rc->rc.ch[0] -= RC_CH_VALUE_OFFSET;
  rc->rc.ch[1] -= RC_CH_VALUE_OFFSET;
  rc->rc.ch[2] -= RC_CH_VALUE_OFFSET;
  rc->rc.ch[3] -= RC_CH_VALUE_OFFSET;
  rc->rc.ch[4] -= RC_CH_VALUE_OFFSET;
  rc->rc.ch[0] = RC_LimitVal(rc->rc.ch[0]);
  rc->rc.ch[1] = RC_LimitVal(rc->rc.ch[1]);
  rc->rc.ch[2] = RC_LimitVal(rc->rc.ch[2]);
  rc->rc.ch[3] = RC_LimitVal(rc->rc.ch[3]);
  rc->rc.ch[4] = RC_LimitVal(rc->rc.ch[4]);
}

void RC_ReceiveCpltCallback(RC_ctrl_t *rc, UART_HandleTypeDef *huart) {
  if (huart == rc->huart) {
    HAL_UART_Receive_DMA(rc->huart, (uint8_t *)rc->RC_rxd, RC_FRAME_LENGTH);
    RC_Unpack(rc);
  }
}
