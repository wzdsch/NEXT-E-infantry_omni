#ifndef TOOLS_H
#define TOOLS_H
#include "struct_typedef.h"
#include <math.h>

typedef struct {
  int32_t x[20];
  int32_t x_sum;
  int32_t x_avr;
  int32_t variance;
  char winter_width;
  char ptr;
}winter_variance_t;

typedef struct {
  float input_value;     // 输入值
  float target_value;    // 目标值(常函数部分的值)
  float threshold;       // 阈值
  float constant_rate;   // 恒定输出比率
  float slope;           // 斜线部分的斜率(大于0表示绝对值递增，小于0表示绝对值递减)
  float max_out;         // 输出最大值
  float min_out;         // 输出最小值
  float out;              // 输出值
  // float intercept;        // 斜线部分的截距
} RampConfig_t;

extern RampConfig_t RampConfig_CHASSISnfig;

extern float rampPlanner(float currentValue, float targetValue, float increaseRate, float decreaseRate);
extern float mapInputToOutput(float inputValue, RampConfig_t* config);
extern void initializeRampConfig(RampConfig_t* config, float threshold, float constant_rate, float slope, float max_out, float min_out);

extern winter_variance_t chassis_control_filter_x;
extern winter_variance_t chassis_control_filter_y;
extern winter_variance_t chassis_control_filter_z;

extern void window_filter_init(winter_variance_t* variable, char width);
extern int32_t window_filter(winter_variance_t* variable, int32_t data);

extern fp32 linearTOrpm(fp32 linear, fp32 dia);
extern int16_t floatTOint(fp32 flNum);
extern fp32 tri_puls(int x,  int T, fp32 min_value, fp32 max_value);
extern fp32 my_fabs(fp32 val);
#endif
