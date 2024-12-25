#include "LADRC.h"

#include "ladrcData.h"
#include "struct_typedef.h"

/**
   *@Brief default 参数表
   *@Brief 根据经验 ts在0.2~0.3之间 Wo与Wc选定后从小到大整定b0
   *@Date@WangShun 2022-05-28 2022-07-03补充
---------------------------------------------------
          LADRC default参数表
---------------------------------------------------
---------------------------------------------------
  ts	|	h	|	r	|   wc   |   w0  |	b0
---------------------------------------------------
  0.1	|	h	|	r	|  100   |  400  |	b0
---------------------------------------------------
   0.157|	h	|	r	|   64   |  224~255  |	b0
---------------------------------------------------
   0.158|	h	|	r	|   63   |  253  |	b0
---------------------------------------------------
   0.159|	h	|	r	|   63   |  252  |	b0
---------------------------------------------------
  0.16|	h	|	r	|   63   |  250  |	b0
---------------------------------------------------
  0.17|	h	|	r	|   59   |  235  |	b0
---------------------------------------------------
  0.18|	h	|	r	|   56   |  222  |	b0
---------------------------------------------------
  0.2	|	h	|	r	|   50   |  200  |	b0
---------------------------------------------------
  0.21|	h	|	r	|   48   |  190  |	b0
---------------------------------------------------
  0.22|	h	|	r	|   45   |  182  |	b0
---------------------------------------------------
  0.23|	h	|	r	|   43   |  174  |	b0
---------------------------------------------------
  0.24|	h	|	r	|   42   |  167  |	b0
---------------------------------------------------
  0.25|	h	|	r	|   40   |  160  |	b0
---------------------------------------------------
  0.26|	h	|	r	|   38   |  154  |	b0
---------------------------------------------------
  0.27|	h	|	r	|   37   |  148 |	b0
---------------------------------------------------
  0.28|	h	|	r	|   36   |  144  |	b0
---------------------------------------------------
  0.29|	h	|	r	|   34   |  138  |	b0
---------------------------------------------------
  0.3	|	h	|	r	|   33   |  133  |	b0
---------------------------------------------------
  0.4	|	h	|	r	|   25   |  100  |	b0
---------------------------------------------------
  0.5	|	h	|	r	|   20   |   80  |	b0
---------------------------------------------------
---------------------------------------------------
*/

/**
 * @brief  LADRC初始化
 * @param  LADRC_TYPE1: ladrc结构体
 * @param  h: // 定时时间及时间步长
 * @param  r: // 跟踪速度参数
 * @param  Wc: // 观测器带宽
 * @param  W0: // 状态误差反馈率带宽
 * @param  B0: // 系统参数
 */
void LADRC_O2_Init(LADRC_O2 *LADRC_TYPE1, fp32 h, fp32 r, fp32 Wc, fp32 W0, fp32 B0) {
  LADRC_TYPE1->h = h;
  LADRC_TYPE1->r = r;
  LADRC_TYPE1->wc = Wc;
  LADRC_TYPE1->w0 = W0;
  LADRC_TYPE1->b0 = B0;
}

/**
 * 函数说明：LADRC缺省
 * 		WangShun于2022-07-03创建
 */
void LADRC_O2_REST(LADRC_O2 *LADRC_TYPE1) {
  LADRC_TYPE1->z1 = 0;  // 定时时间及时间步长
  LADRC_TYPE1->z2 = 0;  // 跟踪速度参数
  LADRC_TYPE1->z3 = 0;  // 观测器带宽
}

/**
 * 函数名：void ADRC_TD(LADRC_O2 *LADRC_TYPE1,fp32 Expect)
 * 函数说明：LADRC跟踪微分部分
 * @param[in]	入口参数，期望值Expect(v0)输出值v1,v2
 * @par 修改日志
 * 		WangShun于2022-05-28创建
 */
void LADRC_O2_TD(LADRC_O2 *LADRC_TYPE1, fp32 Expect) {
  fp32 fh = -LADRC_TYPE1->r * LADRC_TYPE1->r * (LADRC_TYPE1->v1 - Expect)
            - 2 * LADRC_TYPE1->r * LADRC_TYPE1->v2;
  LADRC_TYPE1->v1 += LADRC_TYPE1->v2 * LADRC_TYPE1->h;
  LADRC_TYPE1->v2 += fh * LADRC_TYPE1->h;
}

/**
 * 函数名：LADRC_ESO(LADRC_O2 *LADRC_TYPE1,fp32 FeedBack)
 * 函数说明：LADRC线性状态观测器
 * @param[in]
 * @par 修改日志
 * 		WangShun于2022-07-03创建
 */
void LADRC_O2_ESO(LADRC_O2 *LADRC_TYPE1, fp32 FeedBack) {
  fp32 Beita_01 = 3 * LADRC_TYPE1->w0;
  fp32 Beita_02 = 3 * LADRC_TYPE1->w0 * LADRC_TYPE1->w0;
  fp32 Beita_03 = LADRC_TYPE1->w0 * LADRC_TYPE1->w0 * LADRC_TYPE1->w0;

  fp32 e = LADRC_TYPE1->z1 - FeedBack;
  LADRC_TYPE1->z1 += (LADRC_TYPE1->z2 - Beita_01 * e) * LADRC_TYPE1->h;
  LADRC_TYPE1->z2 +=
    (LADRC_TYPE1->z3 - Beita_02 * e + LADRC_TYPE1->b0 * LADRC_TYPE1->u) * LADRC_TYPE1->h;
  LADRC_TYPE1->z3 += -Beita_03 * e * LADRC_TYPE1->h;
}

/**
   *@Brief  LADRC_LSEF
   *@Date   线性控制率
      WangShun于2022-07-03创建
   */
void LADRC_O2_LF(LADRC_O2 *LADRC_TYPE1) {
  fp32 Kp = LADRC_TYPE1->wc * LADRC_TYPE1->wc;
  fp32 Kd = 2 * LADRC_TYPE1->wc;
  /**
   *@Brief  按自抗扰入门书上kd = 2wc
   *@Before Kd=3*LADRC_TYPE1->wc;
   *@Now    Kd=2*LADRC_TYPE1->wc;
   *@WangShun  2022-04-27  注释
   */
  fp32 e1 = LADRC_TYPE1->v1 - LADRC_TYPE1->z1;
  fp32 e2 = LADRC_TYPE1->v2 - LADRC_TYPE1->z2;
  fp32 u0 = Kp * e1 + Kd * e2;
  LADRC_TYPE1->u = (u0 - LADRC_TYPE1->z3) / LADRC_TYPE1->b0;
  if (LADRC_TYPE1->u > 30000)
    LADRC_TYPE1->u = 30000;
  else if (LADRC_TYPE1->u < -30000)
    LADRC_TYPE1->u = -30000;
}

/**
 * LADRC控制函数 .
 * 将其置于任务循环中即可
 * @par 其它
 * @par 修改日志
 * @WangShun  2022-07-03  注释
 */
fp32 LADRC_O2_Loop(LADRC_O2 *LADRC_TYPE1, fp32 *Expect, fp32 *RealTimeOut) {
  fp32 Expect_Value = *Expect;
  fp32 Measure = *RealTimeOut;
  LADRC_O2_TD(LADRC_TYPE1, Expect_Value);
  LADRC_O2_ESO(LADRC_TYPE1, Measure);
  LADRC_O2_LF(LADRC_TYPE1);
  return LADRC_TYPE1->u;
}
