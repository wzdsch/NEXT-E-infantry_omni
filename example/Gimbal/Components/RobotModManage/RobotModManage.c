#include "RobotModManage.h"

#include "BMI088.h"
#include "DJI_Motor.h"
#include "MCUConnectStructs.h"
#include "RC_task.h"
#include "VisionConnect.h"
#include "Vofa.h"
#include "chassis.h"
#include "gimbal.h"
#include "referee.h"
#include "refereeData_v1.6.h"
#include "remote_control.h"
#include "shooter.h"
#include "tim.h"
#include "usart.h"
RC_SBUS_t RC_sbus;
PC_Flags PCflags = {0};
uint16_t PWMpulse = 1000;  // 弹仓盖
union VofaDATA Vofa;
#define zSpeed  2500
#define xySpeed 30000

#define Xfactor 100.0f
#define Yfactor 100.0f

#define DEADBAND 5  // 遥控死区

extern VisionState visionState;
extern nav_rxd_t nav_rxd;

/////////////////////////////////////////////////////////////////////////////////////////
void remote_controller() {
  //  givespeed = RAMP_float(setspeed, givespeed, 1.5);
  //	RC_Data.ET07_DATA.V1_CH9=1807;
  // if (RC_Data.ET07_DATA.LeftTransverse_CH4 > RC_SW_DOWN
  //     || RC_Data.ET07_DATA.LeftTransverse_CH4 < RC_SW_UP) {
  //   RC_Data.ET07_DATA.LeftTransverse_CH4 = 1024;
  // }
  // if (RC_Data.ET07_DATA.LeftDirection_CH3 > RC_SW_DOWN
  //     || RC_Data.ET07_DATA.LeftDirection_CH3 < RC_SW_UP) {
  //   RC_Data.ET07_DATA.LeftDirection_CH3 = 1024;
  // }
  // if (RC_Data.ET07_DATA.RightDirection_CH2 > RC_SW_DOWN
  //     || RC_Data.ET07_DATA.RightDirection_CH2 < RC_SW_UP) {
  //   RC_Data.ET07_DATA.RightDirection_CH2 = 1024;
  // }
  // if (RC_Data.ET07_DATA.RightTransverse_CH1 > RC_SW_DOWN
  //     || RC_Data.ET07_DATA.RightTransverse_CH1 < RC_SW_UP) {
  //   RC_Data.ET07_DATA.RightTransverse_CH1 = 1024;
  // }

  // 遥控限幅
  if (RC_Data.ET07_DATA.LeftTransverse_CH4 > RC_SW_DOWN) {
    RC_Data.ET07_DATA.LeftTransverse_CH4 = RC_SW_DOWN;
  }
  if (RC_Data.ET07_DATA.LeftTransverse_CH4 < RC_SW_UP) {
    RC_Data.ET07_DATA.LeftTransverse_CH4 = RC_SW_UP;
  }

  if (RC_Data.ET07_DATA.LeftDirection_CH3 > RC_SW_DOWN) {
    RC_Data.ET07_DATA.LeftDirection_CH3 = RC_SW_DOWN;
  }
  if (RC_Data.ET07_DATA.LeftDirection_CH3 < RC_SW_UP) {
    RC_Data.ET07_DATA.LeftDirection_CH3 = RC_SW_UP;
  }

  if (RC_Data.ET07_DATA.RightDirection_CH2 > RC_SW_DOWN) {
    RC_Data.ET07_DATA.RightDirection_CH2 = RC_SW_DOWN;
  }
  if (RC_Data.ET07_DATA.RightDirection_CH2 < RC_SW_UP) {
    RC_Data.ET07_DATA.RightDirection_CH2 = RC_SW_UP;
  }

  if (RC_Data.ET07_DATA.RightTransverse_CH1 > RC_SW_DOWN) {
    RC_Data.ET07_DATA.RightTransverse_CH1 = RC_SW_DOWN;
  }
  if (RC_Data.ET07_DATA.RightTransverse_CH1 < RC_SW_UP) {
    RC_Data.ET07_DATA.RightTransverse_CH1 = RC_SW_UP;
  }

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWMpulse);  // 舵机位置

  if (RC_Data.ET07_DATA.V1_CH9 < 1800
      && RC_Data.ET07_DATA.V1_CH9 >= 240) {  // 如果V1旋钮没有顺时针旋到底，则为遥控器控制

    if (RC_Data.ET07_DATA.LeftTransverse_CH4 <= (RC_SW_MID + DEADBAND)  // X轴速度
        && RC_Data.ET07_DATA.LeftTransverse_CH4 >= (RC_SW_MID - DEADBAND)) {
      ChassisControlData.speedx = 0;
    }
    else {
      ChassisControlData.speedx =
        15
        * ((RC_Data.ET07_DATA.LeftTransverse_CH4 > RC_SW_MID)
             ? RC_Data.ET07_DATA.LeftTransverse_CH4 - (RC_SW_MID + DEADBAND)
             : RC_Data.ET07_DATA.LeftTransverse_CH4 - (RC_SW_MID - DEADBAND));
      if (ChassisControlData.speedx > 15000) {
        ChassisControlData.speedx = 15000;
      }
      if (ChassisControlData.speedx < -15000) {
        ChassisControlData.speedx = -15000;
      }
    }

    if (RC_Data.ET07_DATA.LeftDirection_CH3 <= (RC_SW_MID + DEADBAND)  // y轴速度
        && RC_Data.ET07_DATA.LeftDirection_CH3 >= (RC_SW_MID - DEADBAND)) {
      ChassisControlData.speedy = 0;
    }
    else {
      ChassisControlData.speedy =
        15
        * ((RC_Data.ET07_DATA.LeftDirection_CH3 > RC_SW_MID)
             ? RC_Data.ET07_DATA.LeftDirection_CH3 - (RC_SW_MID + DEADBAND)
             : RC_Data.ET07_DATA.LeftDirection_CH3 - (RC_SW_MID - DEADBAND));
      if (ChassisControlData.speedy > 15000) {
        ChassisControlData.speedy = 15000;
      }
      if (ChassisControlData.speedy < -15000) {
        ChassisControlData.speedy = -15000;
      }
    }

    if (RC_Data.ET07_DATA.SD_CH8
        == RC_SW_UP) {
      ChassisControlData.speedz = zSpeed;
      // ChassisControlData.withSuperCap = 0;
    }
    else {
      ChassisControlData.speedz = -zSpeed;
      // ChassisControlData.withSuperCap = 1;
      // ChassisControlData.speedx = nav_rxd.spd_x;
      // ChassisControlData.speedy = nav_rxd.spd_y;
      // ChassisControlData.speedz = nav_rxd.spd_z;
    }

    if (GimbalControlData.mode == GIMBAL_YAW_EN || GimbalControlData.mode == GIMBAL_ALL_EN) {
      if (RC_Data.ET07_DATA.RightTransverse_CH1 >= (RC_SW_MID + DEADBAND)) {  // 遥控左右转
        GimbalControlData.yawAngle +=
          (-0.0006) * (RC_Data.ET07_DATA.RightTransverse_CH1 - (RC_SW_MID + DEADBAND));
      }
      if (RC_Data.ET07_DATA.RightTransverse_CH1 <= (RC_SW_MID - DEADBAND)) {
        GimbalControlData.yawAngle -=
          (-0.0006) * ((RC_SW_MID - DEADBAND) - RC_Data.ET07_DATA.RightTransverse_CH1);
      }
      if (RC_Data.ET07_DATA.RightDirection_CH2 >= (RC_SW_MID + DEADBAND)) {  // 遥控抬头低头

        GimbalControlData.pitchAngle +=
          OneStep
          * ((fp32)(RC_Data.ET07_DATA.RightDirection_CH2 - (RC_SW_MID + DEADBAND))
             / (fp32)(RC_SW_DOWN - (RC_SW_MID + DEADBAND)));
      }
      if (RC_Data.ET07_DATA.RightDirection_CH2 <= (RC_SW_MID - DEADBAND)) {
        GimbalControlData.pitchAngle +=
          OneStep
          * ((fp32)((RC_SW_MID - DEADBAND) - RC_Data.ET07_DATA.RightDirection_CH2)
             / (fp32)((RC_SW_MID - DEADBAND) - RC_SW_DOWN));
      }
    }

    if (RC_Data.ET07_DATA.SA_CH5 == RC_SW_DOWN
        && visionState.tracking == 0x01) {  // SA扳机向下开启自瞄

      GimbalControlData.yawAngle = vision1.RXData.VisionRxData.YawAngleTarget;
      GimbalControlData.pitchAngle = (vision1.RXData.VisionRxData.PitchAngleTarget) * MotorRate;
       // 由角度值换算为编码器值
    }

    //                        角度幅值限制
    if (GimbalControlData.yawAngle > 360.0f) {
      GimbalControlData.yawAngle -= 360.0f;
    }
    if (GimbalControlData.yawAngle < 0.0f) {
      GimbalControlData.yawAngle += 360.0f;
    }
    if (GimbalControlData.pitchAngle > pitchAngleMax) {
      GimbalControlData.pitchAngle = pitchAngleMax;
    }
    if (GimbalControlData.pitchAngle < pitchAngleMin) {
      GimbalControlData.pitchAngle = pitchAngleMin;
    }
    /***********************************************************************************************/
    /***********************************模式切换逻辑*************************************************/
    /***********************************************************************************************/

    if (RC_Data.ET07_DATA.V2_CH10 >= RC_SW_UP && RC_Data.ET07_DATA.V2_CH10 < 650
        && RC_Data.ET07_DATA.SC_CH7 == RC_SW_UP) {
      ChassisControlData.mode = CHASSIS_DISABLE;
      GimbalControlData.mode = GIMBAL_STOP;
    }
    else if (RC_Data.ET07_DATA.SA_CH5 == RC_SW_DOWN) {
    }
    else {
      switch (RC_Data.ET07_DATA.SC_CH7) {  // SC键逻辑
        case RC_SW_MID:
          GimbalControlData.mode = GIMBAL_ALL_EN;
          ChassisControlData.mode = CHASSIS_FOLLOW;
          break;
        case RC_SW_DOWN:
          ChassisControlData.mode = CHASSIS_TOP;
          GimbalControlData.mode = GIMBAL_ALL_EN;
          break;
        default:
          break;
      }
    }

    // PWMpulse = 2000;
    // switch (RC_Data.ET07_DATA.SD_CH8) {  // SD单独控制弹仓盖
    //   case RC_SW_DOWN:
    //     ChassisControlData.speedx = nav_rxd.spd_x;
    //     ChassisControlData.speedy = nav_rxd.spd_y;
    //     // ChassisControlData.speedz = nav_rxd.spd_z; // 暂时用不上z
    //     break;
    //   case RC_SW_UP:
    //     // PWMpulse = 1000;
    //     break;
    //   default:
    //     break;
    // }
    if (RC_Data.ET07_DATA.V2_CH10 < 650
        && RC_Data.ET07_DATA.V2_CH10 >= RC_SW_UP) {  // V2旋钮控制发射机构
      shooter1.shooterMode = SHOOTER_STOP;
    }
    else if (RC_Data.ET07_DATA.V2_CH10 <= RC_SW_DOWN) {
      GimbalControlData.mode = GIMBAL_ALL_EN;
      switch (RC_Data.ET07_DATA.SB_CH6) {
        case RC_SW_DOWN:
          if (vision1.RXData.VisionRxData.fireControl == 0xff
              || RC_Data.ET07_DATA.SA_CH5 == RC_SW_UP) {
            shooter1.shooterMode = SHOOTER_FIRE;
          }
          else {
            shooter1.shooterMode = SHOOTER_HOLD;
          }
          break;
        case RC_SW_UP:
          shooter1.shooterMode = SHOOTER_HOLD;
          break;
        default:
          break;
      }
    }
  }
  else if (RC_Data.ET07_DATA.V1_CH9 >= 1800
           && RC_Data.ET07_DATA.V1_CH9 <= 1810) {  // 如果V1旋钮选到底，则为PC控制，图传链路
                                                   // 左键开火控制
    shooter1.shooterMode = SHOOTER_HOLD;
    GimbalControlData.mode = GIMBAL_ALL_EN;
    switch (referee_remote_control.left_button_down) {
      case 1: {
        if (referee_remote_control.right_button_down
            == 0) {  // 未按下右键开启自瞄时，单独左键控制打弹
          shooter1.shooterMode = SHOOTER_FIRE;
        }
        else if (referee_remote_control.right_button_down == 1
                 && vision1.RXData.VisionRxData.fireControl
                      == 0xff) {  // 按下右键开启自瞄时，左键按下并且开火时机适宜，再打弹
          shooter1.shooterMode = SHOOTER_FIRE;
        }
        else {
          shooter1.shooterMode = SHOOTER_HOLD;
        }
      } break;
      case 0: {
        shooter1.shooterMode = SHOOTER_HOLD;
      break;
	  } 
      default:
        break;
    }
    // 右键开启自瞄
    if (referee_remote_control.right_button_down == 1 && visionState.tracking == 1) {
      GimbalControlData.yawAngle = vision1.RXData.VisionRxData.YawAngleTarget;
      GimbalControlData.pitchAngle = (vision1.RXData.VisionRxData.PitchAngleTarget) * MotorRate;  // 由角度值换算为编码器值
    }
    // 鼠标控制
    else {
      GimbalControlData.pitchAngle += (fp32)referee_remote_control.mouse_y / Yfactor;
      GimbalControlData.yawAngle -= (fp32)referee_remote_control.mouse_x / Xfactor;
    }
    //                        角度幅值限制
    if (GimbalControlData.yawAngle > 360) {
      GimbalControlData.yawAngle -= 360;
    }
    if (GimbalControlData.yawAngle < 0) {
      GimbalControlData.yawAngle += 360;
    }
    if (GimbalControlData.pitchAngle > pitchAngleMax) {
      GimbalControlData.pitchAngle = pitchAngleMax;
    }
    if (GimbalControlData.pitchAngle < pitchAngleMin) {
      GimbalControlData.pitchAngle = pitchAngleMin;
    }

    // 底盘模式切换逻辑
    if (referee_remote_control.KEY_Z == 1) {  // 按Z开启小陀螺
      PCflags.FreeFlag = 1;
    }
    if (referee_remote_control.KEY_X == 1) {  // 按X切换至底盘跟随模式
      PCflags.FreeFlag = 0;
    }
    if (PCflags.FreeFlag == 1) {
      ChassisControlData.mode = CHASSIS_TOP;
    }
    else {
      ChassisControlData.mode = CHASSIS_FOLLOW;
    }
    if (ChassisControlData.mode == CHASSIS_TOP) {
      ChassisControlData.speedz = zSpeed;
    }
    else {
      ChassisControlData.speedz = 0;
    }
    // 运动逻辑
    // WS键逻辑
    if (referee_remote_control.KEY_W == 1) {
      ChassisControlData.speedy = xySpeed;
    }
    else if (referee_remote_control.KEY_S == 1) {
      ChassisControlData.speedy = -xySpeed;
    }
    else {
      ChassisControlData.speedy = 0;
    }
    // AD键逻辑
    if (referee_remote_control.KEY_A == 1) {
      ChassisControlData.speedx = -xySpeed;
    }
    else if (referee_remote_control.KEY_D) {
      ChassisControlData.speedx = xySpeed;
    }
    else {
      ChassisControlData.speedx = 0;
    }
    // QE键逻辑
    if (referee_remote_control.KEY_Q == 1) {
      PCflags.LeftTurn = 1;
    }
    else if (PCflags.LeftTurn == 1) {
      GimbalControlData.yawAngle += 45.0f;
      PCflags.LeftTurn = 0;
    }

    if (referee_remote_control.KEY_E == 1) {
      PCflags.RightTurn = 1;
    }
    else if (PCflags.RightTurn == 1) {
      GimbalControlData.yawAngle -= 45.0f;
      PCflags.RightTurn = 0;
    }
    // 弹仓盖逻辑
    if (referee_remote_control.KEY_B == 1) {
      PWMpulse = 2000;
    }
    if (referee_remote_control.KEY_V == 1) {
      PWMpulse = 1000;
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWMpulse);  // 舵机位置
  }
  ChassisControlData.AutoAim = vision1.RXData.VisionRxData.fireControl == 1 ? 1 : 0;
  // ChassisControlData.cover =
  //   PWMpulse == 1000 ? 0 : 1;  // UI的两个变量赋值，通过chassiscontrol结构体发送
#ifdef refereedebug
  PCflags.FreeFlag = 1;
#endif
}

void tuneRun() {
  static uint16_t timeCount = 0;
  if (tuneTime > 0) {
    timeCount += 2;
  }
  if (timeCount > 350) {
    timeCount = 0;
    tuneTime--;
  }
  if (timeCount > 350 / 2) {
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  }
  else {
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
  }
}

void tuneINIT() {
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
}

void tune(uint8_t time) {
  tuneTime += time;
}

/** pitch闭环编码器值
 * @brief 给视觉反馈当前角度，6760是pitch轴设定零点
 *
 * @return fp32
 */
fp32 MotorToAngle(void) {
  return (6760.0f - motorPitch.realEcdF) * 0.04395f;
}

/** pitch闭环陀螺仪角度
 * @brief 还是给视觉反馈角度，直接使用陀螺仪角度
 *
 * @return fp32
 */
fp32 IMUToAngle(void) {
  return 0.0f - BMI088_gimbal.pitchAngle;
}

/**
 * @brief 斜坡函数
 *
 * @param now 当前值
 * @param ramp 单次变化最大幅度
 * @param final 期望值
 * @return fp32
 */
fp32 RAMP_float(fp32 final, fp32 now, fp32 ramp) {
  fp32 buffer = 0;

  buffer = final - now;

  if (buffer > 0) {
    if (buffer > ramp) {
      now += ramp;
    }
    else {
      now += buffer;
    }
  }
  else {
    if (buffer < -ramp) {
      now += -ramp;
    }
    else {
      now += buffer;
    }
  }

  return now;
}
