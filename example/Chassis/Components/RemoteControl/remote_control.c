#include "remote_control.h"

#include "main.h"
uint64_t tempcount = 0;

void RemoteControlINIT(RC_SBUS_t *sbus, UART_HandleTypeDef *huart) {
  sbus->huart = huart;
  HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t *)sbus->sbus_buf, RC_FRAME_LENGTH);
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

void SBUS_Unpack(RC_SBUS_t *sbus) {
  //	if(sbus_buf[0] == 0x0f&&sbus_buf==0x00)//sbus
  if (sbus->sbus_buf[0] == 0x0f)  // wbus 校验
  {

    tempcount++;
    sbus->sbus.SBUS_channels[0] = ((sbus->sbus_buf[1] | sbus->sbus_buf[2] << 8) & 0x07FF);
    sbus->sbus.SBUS_channels[1] = ((sbus->sbus_buf[2] >> 3 | sbus->sbus_buf[3] << 5) & 0x07FF);
    sbus->sbus.SBUS_channels[2] =
      ((sbus->sbus_buf[3] >> 6 | sbus->sbus_buf[4] << 2 | sbus->sbus_buf[5] << 10) & 0x07FF);
    sbus->sbus.SBUS_channels[3] = ((sbus->sbus_buf[5] >> 1 | sbus->sbus_buf[6] << 7) & 0x07FF);
    sbus->sbus.SBUS_channels[4] = ((sbus->sbus_buf[6] >> 4 | sbus->sbus_buf[7] << 4) & 0x07FF);
    sbus->sbus.SBUS_channels[5] =
      ((sbus->sbus_buf[7] >> 7 | sbus->sbus_buf[8] << 1 | sbus->sbus_buf[9] << 9) & 0x07FF);
    sbus->sbus.SBUS_channels[6] = ((sbus->sbus_buf[9] >> 2 | sbus->sbus_buf[10] << 6) & 0x07FF);
    sbus->sbus.SBUS_channels[7] = ((sbus->sbus_buf[10] >> 5 | sbus->sbus_buf[11] << 3) & 0x07FF);
    sbus->sbus.SBUS_channels[8] = ((sbus->sbus_buf[12] | sbus->sbus_buf[13] << 8) & 0x07FF);
    sbus->sbus.SBUS_channels[9] = ((sbus->sbus_buf[13] >> 3 | sbus->sbus_buf[14] << 5) & 0x07FF);
    //		sbus->sbus.SBUS_channels[10] =
    //((sbus->sbus_buf[14]>>6|sbus->sbus_buf[15]<<2|sbus->sbus_buf[16]<<10) & 0x07FF);
    //		sbus->sbus.SBUS_channels[11] = ((sbus->sbus_buf[16]>>1|sbus->sbus_buf[17]<<7) & 0x07FF);
    //		sbus->sbus.SBUS_channels[12] = ((sbus->sbus_buf[17]>>4|sbus->sbus_buf[18]<<4) & 0x07FF);
    //		sbus->sbus.SBUS_channels[13] =
    //((sbus->sbus_buf[18]>>7|sbus->sbus_buf[19]<<1|sbus->sbus_buf[20]<<9)  & 0x07FF);
    //		sbus->sbus.SBUS_channels[14] = ((sbus->sbus_buf[20]>>2|sbus->sbus_buf[21]<<6) & 0x07FF);
    //		sbus->sbus.SBUS_channels[15] = ((sbus->sbus_buf[21]>>5|sbus->sbus_buf[22]<<3) & 0x07FF);
  }
}

void RC_ReceiveCpltCallback(RC_SBUS_t *sbus, UART_HandleTypeDef *huart) {
  if (huart == sbus->huart) {
    HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t *)sbus->sbus_buf, RC_FRAME_LENGTH);
    // SBUS_Unpack(sbus);
  }
}

void RC_EventCallback(RC_SBUS_t *sbus, UART_HandleTypeDef *huart) {
  if (huart == sbus->huart) {
    HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t *)sbus->sbus_buf, RC_FRAME_LENGTH);
    SBUS_Unpack(sbus);
  }
}

void RC_ErrorCallback(RC_SBUS_t *sbus, UART_HandleTypeDef *huart) {
  if (huart == sbus->huart) {
    __HAL_UART_CLEAR_FEFLAG(sbus->huart);
    HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t *)sbus->sbus_buf, RC_FRAME_LENGTH);
  }
}
