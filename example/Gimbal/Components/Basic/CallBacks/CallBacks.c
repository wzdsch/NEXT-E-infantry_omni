#include "CallBacks.h"

#include "MCUConnect.h"
#include "MCUConnectStructs.h"
#include "RobotModManage.h"
#include "VisionConnect.h"
#include "Vofa.h"
#include "gimbal.h"
#include "main.h"
#include "tim.h"
// #include "remote_control.h"
#include "BMI088.h"
#include "addOns.h"
#include "bsp_wbus.h"
#include "referee.h"
#include "shooter.h"

unsigned int time_ms = 0;

extern GimbalControl GimbalControlData;
extern DJI_Motor motor2006;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim6) {  // BMI088@1KHZ
    BMI088_RUN(&BMI088_gimbal);
#if SUPPLIER_ECD == 1
    getSupplierTotalEcd(&shooter1);
#endif
    time_ms++;
  }
  if (htim == &htim8) {  // VOFA+调试200HZ
	  // JustFloat(GimbalControlData.yawAngle, BMI088_gimbal.yawAngle < 0 ? BMI088_gimbal.yawAngle + 360.0f : BMI088_gimbal.yawAngle,\
	  // motorYaw.pidOutput0, motorYaw.realSpeedF, &huart1); // yaw
	  // JustFloat(GimbalControlData.yawAngle, BMI088_gimbal.yawAngle,\
		// vision1.RXData.VisionRxData.YawAngleTarget, motorYaw.realSpeedF, &huart1); // pitch
    // JustFloat((int)motor2006.target % 1000000, (int)shooter1.supplier_total_ecd % 1000000, motor2006.pidOutput0, motor2006.realSpeedF, &huart1);  // shooter
    // JustFloat(vision1.RXData.VisionRxData.YawAngleTarget, BMI088_gimbal.yawAngle, vision1.TXData.YawAngle, 0.0f, &huart1); // yaw
    //JustFloat(motor2006.realCurrentF, motor2006.realSpeedF, 0.0f, 0.0f, &huart1); // shooter
		JustFloat(motorPitch.realCurrentF, motorPitch.preProcessResult, BMI088_gimbal.pitchAngle, 0.0f, &huart1);
  }
  if (htim == &htim9) {
    VisionConnectSend(&vision1);  // 向视觉发送数据10ms@100hz
    referee_unpack_fifo_data();   // 裁判系统数据解包
  }
  if (htim == &htim11) {  // 双机通讯解包0.67ms@1500HZ
    connectionUnpackData(&connect);
    shooter1.maxHeat = 500; // RefereeData.maxHeat;  // 枪管热量赋值
    shooter1.gunHeat = RefereeData.gunHeat1;
    shooter1.heat_cooling = RefereeData.heat_cooling;
    VisionConnectUpdateTX(&vision1, RefereeData.OurColor,
                          BMI088_gimbal.yawAngle,  // 更新视觉发送缓存
                          BMI088_gimbal.pitchAngle, RefereeData.gunSpeed1);
  }
  if (htim == &htim12) {  // 双机通信发送2ms@500HZ
    remote_controller();
    ChassisControlData.vision_tracking = visionState.tracking;
    connectionSendData(&connect, (uint8_t *)&ChassisControlData, sizeof(ChassisControlData),
                       ChassisControlData_ID);
  }
  if (htim == &htim13) {  // 云台任务2ms@500Hz
    DJI_MotorPidRUN(gimbal1.gimbalGroup);
    DJI_MotorSendData(gimbal1.gimbalGroup);
  }
  if (htim == &htim14) {  // 发射机构任务2ms@500Hz
    DJI_MotorPidRUN(shooter1.shooterGroup);
    DJI_MotorSendData(shooter1.shooterGroup);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
}

 void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
 }

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
}

// can中断0
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (hcan == &hcan1) {
    DJI_MotorReceiveMotordat(&group1);
  }
  if (hcan == &hcan2) {
    DJI_MotorReceiveMotordat(&group2);
  }
}

// can中断1
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (hcan == &hcan1) {
    connectionRcceiveData(&connect);  // 双击通信解包
  }
  if (hcan == &hcan2) {
  }
}
