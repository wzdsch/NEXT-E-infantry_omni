#include "shooter.h"

#include "DJI_Motor.h"
#include "MCUConnect.h"
#include "math.h"
#include "MCUConnectStructs.h"

extern unsigned int time_ms;
extern Referee_data RefereeData;

void shooterINIT(shooter *shooter, DJI_MotorGroup *group, DJI_Motor *friL, DJI_Motor *friR,
                 DJI_Motor *supplier, fp32 friSpeed, fp32 shootFreq, uint8_t heatPerShoot, fp32 ecd_per_shoot) {
  shooter->shooterGroup = group;
  shooter->friLmotor = friL;
  shooter->friRmotor = friR;
  shooter->supplierMotor = supplier;
  shooter->shooterMode = SHOOTER_STOP;
  shooter->supplierMode = SUPPLIER_RUN;
  shooter->friSpeed = friSpeed;
  shooter->shootFreq = shootFreq;
  shooter->startDelay = 0;
  shooter->stuckCount = 0;
  shooter->heatPerShoot = heatPerShoot;
#if SUPPLIER_ECD == 1
  shooter->supplier_ecd_buf[0] = shooter->supplierMotor->realEcdF;
  shooter->supplier_ecd_buf[1] = shooter->supplierMotor->realEcdF;
  shooter->supplier_total_ecd = 0;
  shooter->ecd_per_shoot = ecd_per_shoot;
#endif
}

#if SUPPLIER_ECD == 1

void getSupplierTotalEcd(shooter* shoot) {
  int16_t delta_ecd = 0; // 编码值增量

  // 更新缓存
  shoot->supplier_ecd_buf[1] = shoot->supplier_ecd_buf[0];
  shoot->supplier_ecd_buf[0] = shoot->supplierMotor->realEcdF;

  // 过零
  delta_ecd = shoot->supplier_ecd_buf[0] - shoot->supplier_ecd_buf[1];
  if (delta_ecd >= 4096) {
    delta_ecd -= 8191;
  }
  if (delta_ecd < -4096) {
    delta_ecd += 8191;
  }

  // 计算
  shoot->supplier_total_ecd += delta_ecd;
  shoot->supplier_total_ecd_f = shoot->supplier_total_ecd;
}

void shooterStuckProcess(shooter *shooter) {
  static uint8_t supplier_reverse_flg = 0; // 卡弹反拨标志位
  static fp32 reverse_tar_ecd = 0; // 反拨目标编码值
  if (shooter->supplierMode != SUPPLIER_ERROR) {
    // 卡弹检测
    if (shooter->startDelay < startDelayLimit) {  // 如果电机还在启动阶段
      shooter->startDelay++;
    }
    // 如果过了启动阶段还在堵转
    else if (fabs(shooter->supplierMotor->realSpeedF) < (fabs(shooter->supplierMotor->pidOutput0) * stuckPersent)\
            && (shooter->supplierMotor->realCurrentF < -8000 || shooter->supplierMotor->realCurrentF > 8000)) {
      shooter->stuckCount++;
    }
    else {
      shooter->stuckCount = 0;
    }
    if (shooter->stuckCount >= stuckCountLimit) {  // 如果堵转时间超时
      shooter->supplierMode = SUPPLIER_ERROR;
    }
  }
  // 卡弹反拨
  if (shooter->supplierMode == SUPPLIER_ERROR) {
    if (shooter->stuckProcessCount < stuckProcessCountLimit) {  // 还在堵转处理中
      if (supplier_reverse_flg == 0) {
        // 保证累计的没打出去的弹丸 不会在停止打弹后才打出
        DJI_MotorSetTarget(shooter->supplierMotor, \
          shooter->supplierMotor->target - (long long)((shooter->supplierMotor->target - shooter->supplier_total_ecd) / (int)TOTAL_ECD_PER_SHOOT) * TOTAL_ECD_PER_SHOOT);
    
        // 若拨盘已经拨出一颗弹的四分之一的编码值，则将这颗弹丸打出
        DJI_MotorSetTarget(shooter->supplierMotor, \
          (shooter->supplierMotor->target - shooter->supplier_total_ecd) < (TOTAL_ECD_PER_SHOOT / 4 * 3) ? \
          shooter->supplierMotor->target : shooter->supplierMotor->target - TOTAL_ECD_PER_SHOOT);

        reverse_tar_ecd = shooter->supplierMotor->target - TOTAL_ECD_PER_SHOOT;

        supplier_reverse_flg = 1;
      }
        DJI_MotorSetTarget(shooter->supplierMotor, reverse_tar_ecd);

      shooter->stuckProcessCount++;
    }
    else {  // 堵转处理完成
      // 保证累计的没打出去的弹丸 不会在停止打弹后才打出
      DJI_MotorSetTarget(shooter->supplierMotor, \
        shooter->supplierMotor->target - (long long)((shooter->supplierMotor->target - shooter->supplier_total_ecd) / (int)TOTAL_ECD_PER_SHOOT) * TOTAL_ECD_PER_SHOOT);
  
      // 若拨盘已经拨出一颗弹的四分之一的编码值，则将这颗弹丸打出
      DJI_MotorSetTarget(shooter->supplierMotor, \
        (shooter->supplierMotor->target - shooter->supplier_total_ecd) < (TOTAL_ECD_PER_SHOOT / 4 * 3) ? \
        shooter->supplierMotor->target : shooter->supplierMotor->target - TOTAL_ECD_PER_SHOOT);
      shooter->supplierMode = SUPPLIER_RUN;
      shooter->startDelay = 0;
      shooter->stuckProcessCount = 0;
      shooter->stuckCount = 0;
      supplier_reverse_flg = 0;
    }
    time_ms = 0;
  }
}

void shooterRun(shooter *shooter) {
  static uint8_t last_mode = SHOOTER_STOP;
  uint8_t mode_change_flg = 0;
  static int change_cnt = 0;
  // static uint8_t error_stop_flg = 0; // 调试用，卡弹立即停止

  if (shooter->shooterMode != last_mode) {
    mode_change_flg = 1;
    change_cnt++;
  }
  // if (change_cnt != 0) {
  //   change_cnt++;
  //   if (change_cnt >= 5) {
  //     change_cnt = 0;
  //   }
  // }
  last_mode = shooter->shooterMode;

  // if (shooter->shooterMode == SHOOTER_STOP || shooter->shooterMode == SHOOTER_HOLD) {
  //   error_stop_flg = 0;
  // }

  // if (error_stop_flg == 1) {
  //   DJI_MotorDisable(shooter->supplierMotor);
  // }

  // else {
  
  // 弹速控制
  int fri_motor_spd = shooter->friSpeed + 100 * FriMotorSpdLvCtrl();
  if (fri_motor_spd > 8000) { // 
    fri_motor_spd = 8000;
  }
  else if (fri_motor_spd < 3000) {
    fri_motor_spd = 3000;
  }

  switch (shooter->shooterMode) {
  case SHOOTER_STOP: {
    DJI_MotorEnable(shooter->friLmotor);
    DJI_MotorEnable(shooter->friRmotor);
    DJI_MotorSetTarget(shooter->friLmotor, 0.0f);
    DJI_MotorSetTarget(shooter->friRmotor, 0.0f);
    DJI_MotorDisable(shooter->supplierMotor);

    if (mode_change_flg == 1) {
      // 保证累计的没打出去的弹丸 不会在停止打弹后才打出
      DJI_MotorSetTarget(shooter->supplierMotor, \
        shooter->supplierMotor->target - (long long)((shooter->supplierMotor->target - shooter->supplier_total_ecd) / (int)TOTAL_ECD_PER_SHOOT) * TOTAL_ECD_PER_SHOOT);
  
      // 若拨盘已经拨出一颗弹的四分之一的编码值，则将这颗弹丸打出
      DJI_MotorSetTarget(shooter->supplierMotor, \
        (shooter->supplierMotor->target - shooter->supplier_total_ecd) < (TOTAL_ECD_PER_SHOOT / 100 * 99) ? \
        shooter->supplierMotor->target : shooter->supplierMotor->target - TOTAL_ECD_PER_SHOOT);
    }

    shooter->startDelay = 0;
    shooter->stuckCount = 0;
    time_ms = 0;
    break;
  }

  case SHOOTER_HOLD: {
    DJI_MotorEnable(shooter->friLmotor);
    DJI_MotorEnable(shooter->friRmotor);
    DJI_MotorEnable(shooter->supplierMotor);
    DJI_MotorSetTarget(shooter->friLmotor, fri_motor_spd);
    DJI_MotorSetTarget(shooter->friRmotor, fri_motor_spd);

  if (mode_change_flg == 1 && shooter->supplierMode != SUPPLIER_ERROR) {
  // 保证累计的没打出去的弹丸 不会在停止打弹后才打出
  DJI_MotorSetTarget(shooter->supplierMotor, \
    shooter->supplierMotor->target - (long long)((shooter->supplierMotor->target - shooter->supplier_total_ecd) / (int)TOTAL_ECD_PER_SHOOT) * TOTAL_ECD_PER_SHOOT);

  // 若拨盘已经拨出一颗弹的百分之一的编码值，则将这颗弹丸打出
  DJI_MotorSetTarget(shooter->supplierMotor, \
    (shooter->supplierMotor->target - shooter->supplier_total_ecd) < (TOTAL_ECD_PER_SHOOT / 100 * 99) ? \
    shooter->supplierMotor->target : shooter->supplierMotor->target - TOTAL_ECD_PER_SHOOT);
  }
    
    shooter->startDelay = 0;
    shooter->stuckCount = 0;
    time_ms = 0;
    break;
  }

  case SHOOTER_FIRE: {
    DJI_MotorEnable(shooter->friLmotor);
    DJI_MotorEnable(shooter->friRmotor);
    DJI_MotorEnable(shooter->supplierMotor);
    DJI_MotorSetTarget(shooter->friLmotor, fri_motor_spd);
    DJI_MotorSetTarget(shooter->friRmotor, fri_motor_spd);

    if (shooter->supplierMode == SUPPLIER_RUN) {
      shooterFreqControl(shooter);
    }
    break;
  }
  case SHOOTER_ONETAP: {
    DJI_MotorEnable(shooter->friLmotor);
    DJI_MotorEnable(shooter->friRmotor);
    DJI_MotorEnable(shooter->supplierMotor);
    DJI_MotorSetTarget(shooter->friLmotor, fri_motor_spd);
    DJI_MotorSetTarget(shooter->friRmotor, fri_motor_spd);

    if (mode_change_flg == 1 && shooter->supplierMode != SUPPLIER_ERROR) {
      // 保证累计的没打出去的弹丸 不会在停止打弹后才打出
      DJI_MotorSetTarget(shooter->supplierMotor, \
        shooter->supplierMotor->target - (long long)((shooter->supplierMotor->target - shooter->supplier_total_ecd) / (int)TOTAL_ECD_PER_SHOOT) * TOTAL_ECD_PER_SHOOT);

      // 若拨盘已经拨出一颗弹的百分之一的编码值，则将这颗弹丸打出
      DJI_MotorSetTarget(shooter->supplierMotor, \
        (shooter->supplierMotor->target - shooter->supplier_total_ecd) < (TOTAL_ECD_PER_SHOOT / 100 * 99) ? \
        shooter->supplierMotor->target : shooter->supplierMotor->target - TOTAL_ECD_PER_SHOOT);

      if (shooter->maxHeat - shooter->gunHeat > 3 * (shooter->heatPerShoot)) {
        DJI_MotorSetTarget(shooter->supplierMotor, shooter->supplierMotor->target + TOTAL_ECD_PER_SHOOT);
      }
    }
    break;
  }
  default:
    break;
  }
  // }
  shooterStuckProcess(shooter);
  // if (shooter->supplierMode == SUPPLIER_ERROR) {
  //   error_stop_flg = 1;
  // }
}

#elif SUPPLIER_ECD == 0

void shooterStuckProcess(shooter *shooter) {
  if (shooter->supplierMode != SUPPLIER_ERROR) {
    // 卡弹检测
    if (shooter->startDelay < startDelayLimit) {  // 如果电机还在启动阶段
      shooter->startDelay++;
    }
    // 如果过了启动阶段还在堵转
    else if (fabs(shooter->supplierMotor->realSpeedF) < (fabs(shooter->supplierMotor->target) * stuckPersent)) {
      shooter->stuckCount++;
    }
    else {
      shooter->stuckCount = 0;
    }
    if (shooter->stuckCount >= stuckCountLimit) {  // 如果堵转时间超时
      shooter->supplierMode = SUPPLIER_ERROR;
    }
  }
  // 卡弹反拨
  if (shooter->supplierMode == SUPPLIER_ERROR) {
    if (shooter->stuckProcessCount < stuckProcessCountLimit) {  // 还在堵转处理中
      DJI_MotorSetTarget(shooter->supplierMotor, -8000.0f);
      shooter->stuckProcessCount++;
    }
    else {  // 堵转处理完成
      DJI_MotorSetTarget(shooter->supplierMotor, 0.0f);
      shooter->supplierMode = SUPPLIER_RUN;
      shooter->startDelay = 0;
      shooter->stuckProcessCount = 0;
      shooter->stuckCount = 0;
    }
    time_ms = 0;
  }
}

void shooterRun(shooter *shooter) {
#if SUPPLIER_ECD == 1
  static uint8_t last_mode = SHOOTER_STOP;
  uint8_t mode_change_flg = 0;
  static int change_cnt = 0;
  // static uint8_t error_stop_flg = 0; // 调试用，卡弹立即停止

  if (shooter->shooterMode != last_mode) {
    mode_change_flg = 1;
    change_cnt++;
  }
  // if (change_cnt != 0) {
  //   change_cnt++;
  //   if (change_cnt >= 5) {
  //     change_cnt = 0;
  //   }
  // }
  last_mode = shooter->shooterMode;


#endif
  // if (shooter->shooterMode == SHOOTER_STOP || shooter->shooterMode == SHOOTER_HOLD) {
  //   error_stop_flg = 0;
  // }

  // if (error_stop_flg == 1) {
  //   DJI_MotorDisable(shooter->supplierMotor);
  // }

  // else {
  
  // 弹速控制
  int fri_motor_spd = shooter->friSpeed + 100 * FriMotorSpdLvCtrl();
  if (fri_motor_spd > 8000) { // 
    fri_motor_spd = 8000;
  }
  else if (fri_motor_spd < 3000) {
    fri_motor_spd = 3000;
  }

  switch (shooter->shooterMode) {
  case SHOOTER_STOP: {
    DJI_MotorEnable(shooter->friLmotor);
    DJI_MotorEnable(shooter->friRmotor);
    DJI_MotorSetTarget(shooter->friLmotor, 0.0f);
    DJI_MotorSetTarget(shooter->friRmotor, 0.0f);
    DJI_MotorDisable(shooter->supplierMotor);

    DJI_MotorSetTarget(shooter->supplierMotor, 0.0f);

    shooter->startDelay = 0;
    shooter->stuckCount = 0;
    time_ms = 0;
    break;
  }

  case SHOOTER_HOLD: {
    DJI_MotorEnable(shooter->friLmotor);
    DJI_MotorEnable(shooter->friRmotor);
    DJI_MotorEnable(shooter->supplierMotor);
    DJI_MotorSetTarget(shooter->friLmotor, fri_motor_spd);
    DJI_MotorSetTarget(shooter->friRmotor, fri_motor_spd);

    DJI_MotorSetTarget(shooter->supplierMotor, 0.0f);
    
    shooter->startDelay = 0;
    shooter->stuckCount = 0;
    time_ms = 0;
    break;
  }

  case SHOOTER_FIRE: {
    DJI_MotorEnable(shooter->friLmotor);
    DJI_MotorEnable(shooter->friRmotor);
    DJI_MotorEnable(shooter->supplierMotor);
    DJI_MotorSetTarget(shooter->friLmotor, fri_motor_spd);
    DJI_MotorSetTarget(shooter->friRmotor, fri_motor_spd);

    if (shooter->supplierMode == SUPPLIER_RUN) {
      shooterFreqControl(shooter);
    }
    break;
  }
  default:
    break;
  }
  // }
  shooterStuckProcess(shooter);
}

#endif

void shooterFreqControl(shooter *shooter) {
  if (shooter->supplierMode == SUPPLIER_RUN) {
    uint8_t stop_flg = 0; // 停止标志位
    // 最大热量大于150
    if (shooter->maxHeat >= 400){
      // 热量较小时，打弹频率设为高
      if (shooter->gunHeat <= (shooter->maxHeat) * 0.85f - (5 * (shooter->heatPerShoot))) {
        shooter->shootFreq = SHOOTER_FREQ_DFLT;
      }
      // 枪管热量较大时，打弹频率率设为和冷却速率一样
      else if (shooter->gunHeat <= shooter->maxHeat - (5 * (shooter->heatPerShoot))) {
        shooter->shootFreq = shooter->heat_cooling / 10;
      }
      else {
        stop_flg = 1; // 其他情况停止打弹
      }
    }

    // 最大热量大于150
    else if (shooter->maxHeat >= 150 && shooter->maxHeat < 400) {
      // 热量较小时，打弹频率设为高
      if (shooter->gunHeat <= (shooter->maxHeat) * 0.7f - (5 * (shooter->heatPerShoot))) {
        shooter->shootFreq = SHOOTER_FREQ_DFLT;
      }
      // 枪管热量较大时，打弹频率率设为和冷却速率一样
      else if (shooter->gunHeat <= shooter->maxHeat - (5 * (shooter->heatPerShoot))) {
        shooter->shootFreq = shooter->heat_cooling / 10;
      }
      else {
        stop_flg = 1; // 其他情况停止打弹
      }
    }

    else if (shooter->maxHeat < 150 && shooter->maxHeat >= 60) {
      if (shooter->gunHeat <= (shooter->maxHeat) * 0.7f - (4 * (shooter->heatPerShoot))) {
        shooter->shootFreq = shooter->heat_cooling / 5;
      }
      else if (shooter->gunHeat <= shooter->maxHeat - (4 * (shooter->heatPerShoot))) {
        shooter->shootFreq = shooter->heat_cooling / 10;
      }
      else {
        stop_flg = 1; // 其他情况停止打弹
      }
    }
    else if (shooter->maxHeat < 60) {
      if (shooter->gunHeat <= shooter->maxHeat - (4 * (shooter->heatPerShoot))) {
        shooter->shootFreq = shooter->heat_cooling / 10;
      }
      else {
        stop_flg = 1; // 其他情况停止打弹
      }
    }

#if SUPPLIER_ECD == 1
    time_ms %= 1000;
    if (time_ms >= 1000 / shooter->shootFreq) {
      time_ms = 0;
    }
    if (time_ms == 2) {
      DJI_MotorSetTarget(shooter->supplierMotor, shooter->supplierMotor->target + TOTAL_ECD_PER_SHOOT);
    }
#elif SUPPLIER_ECD == 0
    DJI_MotorSetTarget(shooter->supplierMotor, shooter->shootFreq * 540);
#endif
    // 剩余热量少于40，停止打弹
    if (stop_flg) {
    #if SUPPLIER_ECD == 0
      DJI_MotorSetTarget(shooter->supplierMotor, 0.0f);
    #else
      // 保证累计的没打出去的弹丸 不会在停止打弹后才打出
      DJI_MotorSetTarget(shooter->supplierMotor, \
        shooter->supplierMotor->target - (long long)((shooter->supplierMotor->target - shooter->supplier_total_ecd) / (int)TOTAL_ECD_PER_SHOOT) * TOTAL_ECD_PER_SHOOT);

      // 若拨盘已经拨出一颗弹的四分之一的编码值，则将这颗弹丸打出
      DJI_MotorSetTarget(shooter->supplierMotor, \
        (shooter->supplierMotor->target - shooter->supplier_total_ecd) < (TOTAL_ECD_PER_SHOOT / 4 * 3) ? \
        shooter->supplierMotor->target : shooter->supplierMotor->target - TOTAL_ECD_PER_SHOOT);
    #endif
    }
  }
}

uint8_t FriMotorSpdLvCtrl() {
  static int fri_spd_lv = 0;
  static int delay_cnt = 0;
  static uint8_t last_spd = 20;
  uint8_t now_spd = RefereeData.gunSpeed1;
  delay_cnt++;
  if (delay_cnt >= 1000) {
    delay_cnt = 0;
    last_spd = now_spd;
    now_spd = RefereeData.gunSpeed1;
    if (now_spd == last_spd) {
      return fri_spd_lv;
    }
    else if (now_spd > last_spd && now_spd > 23.0f) {
      fri_spd_lv--;
    }
    else if (now_spd < last_spd && now_spd < 18.0f) {
      fri_spd_lv++;
    }
  }
  return fri_spd_lv;
}
