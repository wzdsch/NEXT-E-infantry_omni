#ifndef ADDONS_H
#define ADDONS_H

#include "DJI_Motor.h"
#include "struct_typedef.h"

fp32 example(DJI_Motor *motor);
fp32 ecdZeroCrossing(DJI_Motor *motor);
fp32 chassisFollowZeroCrossing(DJI_Motor *motor);
fp32 powerlimit(DJI_Motor *motor);
fp32 powerlimit_pro(DJI_Motor *motor);
fp32 IMUecdZeroCrossing(DJI_Motor *motor);
fp32 LADRCtest(DJI_Motor *motor);

fp32 LADRC_YawControl(DJI_Motor *motor);
fp32 negative(DJI_Motor *motor);
fp32 pitch_gravity_compensation(DJI_Motor *motor);

#endif
