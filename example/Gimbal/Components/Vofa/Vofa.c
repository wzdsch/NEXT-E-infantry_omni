#include "Vofa.h"

#include "struct_typedef.h"
#include "usart.h"

/**
 * @brief Vofa+函数justfloat协议发送函数，使用DMA发送(cubemx内开启对应串口的DMA
 *
 * @param set 下列四个fp32(float)格式变量名称和个数可以自行更改，对应更改ch数组大小即可
 * @param feedback
 * @param target
 * @param speed
 * @param huart 所使用的串口号地址
 */
void JustFloat(fp32 set, fp32 feedback, fp32 target, fp32 speed, UART_HandleTypeDef *huart) {
  // uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};  协议规定的帧尾
  Vofa.VofaD.set = set;
  Vofa.VofaD.feedback = feedback;
  Vofa.VofaD.TargetSpeed1 = target;
  Vofa.VofaD.Speed1 = speed;
  Vofa.VofaD.tail1 = 0x00;
  Vofa.VofaD.tail2 = 0x00;
  Vofa.VofaD.tail3 = 0x80;
  Vofa.VofaD.tail4 = 0x7f;
//  HAL_UART_Transmit(huart,Vofa.ch,20,0xffff);
	
  HAL_UART_Transmit_DMA(huart, Vofa.ch, 20);
}
