#include "Tools.h"

#include "struct_typedef.h"

/**
 * @brief  4舍5入函数，将小数变为整形
 * @param  flNum: 要转换的fp32
 * @return int16_t:
 */
int16_t floatTOint(fp32 flNum) {
  return (int16_t)(flNum + 0.5f);
}

/**
 * @brief  linear(m/s)->rpm
 * @param  linear: 线速度(m/s)
 * @param  dia: 直径(m)
 * @return int16_t:
 */
fp32 linearTOrpm(fp32 linear, fp32 dia) {
  return (linear * 60.0f) / (PI * dia);
}
