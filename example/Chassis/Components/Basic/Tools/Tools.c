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

// 初始化窗口滤波
void window_filter_init(winter_variance_t* variable, char width) {
  // 设置窗口宽度
  variable->winter_width = width;
  // 初始化窗口中的数据为0
  for (char i = 0; i < width; i++) {
    variable->x[i] = 0;
  }
  // 初始化窗口数据的和为0
  variable->x_sum = 0;
  // 初始化窗口数据的平均值
  variable->x_avr = 0;
}

// 计算窗口滤波
int32_t window_filter(winter_variance_t* variable, int32_t input) {
  // 将输入数据存入窗口
  variable->x[variable->ptr] = input;
  // 初始化窗口数据的和为0
  variable->x_sum = 0;
  // 计算窗口数据的和
  for (char i = 0; i < variable->winter_width; i++) {
    variable->x_sum += variable->x[i];
  }
  // 计算窗口数据的平均值
  variable->x_avr = variable->x_sum / variable->winter_width;
  // 更新指针
  variable->ptr++;
  // 如果指针到达窗口末尾，则重置为0
  if (variable->ptr == variable->winter_width)
    variable->ptr = 0;
  return variable->x_avr;
}

/**
 * @brief  斜坡规划器函数，用于实现从当前值到目标值的平滑过渡。
 * @param  currentValue: 当前值（float）。
 * @param  targetValue: 目标值（float）。
 * @param  increaseRate: 增加时的变化率（float）。
 * @param  decreaseRate: 减小时的变化率（float）。
 * @return float: 返回当前过渡后的值。
 */
float rampPlanner(float current_value, float target_value, float increase_rate, float decrease_rate) {
  increase_rate = fabs(increase_rate);
  decrease_rate = fabs(decrease_rate);
  // 判断目标值与当前值的关系，确定过渡方向
  fp32 fabs_current_value = fabs(current_value);
  fp32 fabs_target_value = fabs(target_value);
  fp32 tar_mul_cur = target_value * current_value;
  
  // 如果目标值等于当前值，直接返回
  if (target_value <= current_value + 0.0001f && target_value >= current_value - 0.0001f)
  {
    return current_value;
  }

  // 目标值绝对值大于当前绝对值，并且目标值和当前值都为正，则为正向加速 
  else if ((fabs_target_value > fabs_current_value && current_value >= 0 && target_value > 0))
  {
      current_value += increase_rate;  // 每次增加变化率
      if (current_value > target_value) {
          current_value = target_value;  // 防止超过目标值
      }
      return current_value;
  }

  // 目标值绝对值大于当前绝对值，并且目标值和当前值都为负，则为负向加速
  else if (fabs_target_value > fabs_current_value && current_value <= 0 && target_value < 0)
  {
      current_value -= increase_rate;
      if (current_value < target_value) {
          current_value = target_value;  // 防止低于目标值
      }
      return current_value;
  }

  /*
    正向减速：
      1.当前值为正，且目标值为负
      2.目标值绝对值小于当前值绝对值，且目标值与当前值都为正
  */
  else if ((current_value > 0 && target_value < 0)
    || (fabs_target_value < fabs_current_value && target_value >= 0 && current_value > 0)) 
  {
    current_value -= decrease_rate;
    if (current_value < 0) // 防止减至反向
    {
      current_value = 0;
    }
    else if (current_value < target_value) // 防止低于目标值
    {
      current_value = target_value;
    }
    return current_value;
  }

  /*
    负向减速：
      1.当前值为负，且目标值为正
      2.目标值绝对值小于当前值绝对值，且目标值与当前值都为负
  */
  else if ((current_value < 0 && target_value > 0) || \
  (fabs_target_value < fabs_current_value && target_value <= 0 && current_value < 0))
  {
    current_value += decrease_rate;
    if (current_value > 0) // 防止减至反向
    {
      current_value = 0;
    }
    else if (current_value > target_value) // 防止高于目标值
    {
      current_value = target_value;
    }
    return current_value;
  }
  return current_value;
}


/// @brief 初始化分段函数
/// @param config 分段函数参数结构体
/// @param threshold 阈值
/// @param constant_rate 恒定的输出值比例
/// @param slope 一次函数部分的斜率
/// @param max_out 最大输出
/// @param min_out 最小输出
void initializeRampConfig(RampConfig_t* config, float threshold, float constant_rate, float slope, float max_out, float min_out) {
  config->out = 0;
  config->input_value = 0;
  config->target_value = 0;
  config->threshold = threshold;
  config->constant_rate = constant_rate;;
  config->slope = slope;
  config->max_out = max_out;
  config->min_out = min_out;
}

/**
* @brief  将输入映射成由一条水平的直线和一条斜线组成的输出。
* @param  inputValue: 输入值（float类型）。
* @param  config: 指向RampConfig结构体的指针，包含所有配置参数。
* @param  target_value: 需要映射变化的值
* @return double: 返回映射后的输出值。
*/
float mapInputToOutput(float inputValue, RampConfig_t* config) {
  float outputValue;

  // 判断输入值是否小于等于阈值
  if (fabs(inputValue) <= config->threshold) {
    outputValue = config->target_value;  // 水平部分，输出保持恒定
  }
  else {
    if (config->target_value >= 0) { // 目标值大于0时，斜率极性不变
      outputValue = config->slope * fabs(inputValue - config->threshold) + config->target_value;  // 斜线部分，输出按照斜率变化
      outputValue = (outputValue > 0 ) ? outputValue : 0; //保持输出与目标值极性一致
    }
    else { // 目标值小于0时，斜率极性取反
      outputValue = -config->slope * fabs(inputValue - config->threshold) + config->target_value;  // 斜线部分，输出按照斜率变化w
      outputValue = (outputValue < 0 ) ? outputValue : 0; //保持输出与目标值极性一致
    }
  }

  outputValue *= config->constant_rate;

  outputValue = (fabs(config->max_out) > fabs(outputValue)) ? outputValue : config->max_out; //限制输出值不超过最大输出值
  outputValue = (fabs(config->min_out) < fabs(outputValue)) ? outputValue : config->min_out; //限制输出值不超过最小输出值

  config->out = outputValue;

  return outputValue;
}


/// @brief 三角波函数
/// @param x 自变量
/// @param T 周期
/// @param min_value 最小值
/// @param max_value 最大值
/// @return 
fp32 tri_puls(int x, int T, fp32 min_value, fp32 max_value) {
  x %= T;
  return min_value + (max_value - min_value) * ((x < T / 2) ? x / (T / 2.0f) : (max_value - min_value) * (T - x) / (T / 2.0f));
}

fp32 my_fabs(fp32 val) {
  return val >= 0 ? val : -val;
}
