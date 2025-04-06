#include "shooter.h"

#include "DJI_Motor.h"
#include "MCUConnect.h"
#include "math.h"

void shooterINIT(shooter *shooter, DJI_MotorGroup *group, DJI_Motor *friL, DJI_Motor *friR,
                 DJI_Motor *supplier, fp32 friSpeed, fp32 shootFreq, uint8_t heatPerShoot) {
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
}

void shooterStuckProcess(shooter *shooter) {
  if (/*shooter->gunHeat <= (shooter->maxHeat) - (4 * (shooter->heatPerShoot))*/ 1) {
    if (shooter->supplierMode == SUPPLIER_RUN) {  // 正常模式下
      DJI_MotorSetTarget(shooter->supplierMotor, shooter->shootFreq);
      if (shooter->startDelay < startDelayLimit) {  // 如果电机还在启动阶段
        shooter->startDelay++;
      }
      // 如果过了启动阶段还在堵转
      else if (fabs(shooter->supplierMotor->realSpeedF)
               < (fabs(shooter->supplierMotor->target) * stuckPersent)) {
        shooter->stuckCount++;
      }
      else {
        shooter->stuckCount = 0;
      }
      if (shooter->stuckCount >= stuckCountLimit) {  // 如果堵转时间超时
        shooter->supplierMode = SUPPLIER_ERROR;
      }
    }
    else if (shooter->supplierMode == SUPPLIER_ERROR) {           // 如果堵转了
      if (shooter->stuckProcessCount < stuckProcessCountLimit) {  // 还在堵转处理中
        DJI_MotorSetTarget(shooter->supplierMotor, -5000/*(shooter->shootFreq)*/);
        shooter->stuckProcessCount++;
      }
      else {  // 堵转处理完成
        DJI_MotorSetTarget(shooter->supplierMotor, 0.0f);
        shooter->supplierMode = SUPPLIER_RUN;
        shooter->startDelay = 0;
        shooter->stuckProcessCount = 0;
        shooter->stuckCount = 0;
      }
    }
    else {
      shooter->stuckProcessCount = 0;
      shooter->stuckCount = 0;
    }
  }
  else {
    DJI_MotorSetTarget(shooter->supplierMotor, 0.0f);
  }
}

void shooterRun(shooter *shooter) {

  switch (shooter->shooterMode) {
    case SHOOTER_STOP:
      DJI_MotorEnable(shooter->friLmotor);
      DJI_MotorEnable(shooter->friRmotor);
      DJI_MotorSetTarget(shooter->friLmotor, 0.0f);
      DJI_MotorSetTarget(shooter->friRmotor, 0.0f);
      DJI_MotorDisable(shooter->supplierMotor);
      shooter->startDelay = 0;
      shooter->stuckCount = 0;
      break;

    case SHOOTER_HOLD:
      DJI_MotorEnable(shooter->friLmotor);
      DJI_MotorEnable(shooter->friRmotor);
      DJI_MotorEnable(shooter->supplierMotor);
      DJI_MotorSetTarget(shooter->friLmotor, shooter->friSpeed);
      DJI_MotorSetTarget(shooter->friRmotor, shooter->friSpeed);
      DJI_MotorSetTarget(shooter->supplierMotor, 0.0f);
      shooter->startDelay = 0;
      shooter->stuckCount = 0;
      break;

    case SHOOTER_FIRE:
      DJI_MotorEnable(shooter->friLmotor);
      DJI_MotorEnable(shooter->friRmotor);
      DJI_MotorEnable(shooter->supplierMotor);
      DJI_MotorSetTarget(shooter->friLmotor, shooter->friSpeed);
      DJI_MotorSetTarget(shooter->friRmotor, shooter->friSpeed);
      // DJI_MotorSetTarget(shooter->supplierMotor, shooter->shootFreq);
      shooterStuckProcess(shooter);
      break;

    case SHOOTER_DBUG:
      DJI_MotorEnable(shooter->friLmotor);
      DJI_MotorEnable(shooter->friRmotor);
      DJI_MotorEnable(shooter->supplierMotor);
      DJI_MotorSetTarget(shooter->friLmotor, 0.0f);
      DJI_MotorSetTarget(shooter->friRmotor, 0.0f);
      DJI_MotorSetTarget(shooter->supplierMotor, (shooter->shootFreq));
      // shooterStuckProcess(shooter);
      break;

    case SHOOTER_DBUG1:
      DJI_MotorEnable(shooter->friLmotor);
      DJI_MotorEnable(shooter->friRmotor);
      DJI_MotorEnable(shooter->supplierMotor);
      DJI_MotorSetTarget(shooter->friLmotor, 0.0f);
      DJI_MotorSetTarget(shooter->friRmotor, 0.0f);
      DJI_MotorSetTarget(shooter->supplierMotor, -(shooter->shootFreq));
      // shooterStuckProcess(shooter);
      break;
    default:
      break;
  }
}
