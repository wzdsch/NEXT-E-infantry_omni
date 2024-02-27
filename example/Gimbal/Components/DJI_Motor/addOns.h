#ifndef ADDONS_H
#define ADDONS_H

#include "DJI_Motor.h"
#include "struct_typedef.h"

fp32 example(DJI_Motor *motor);
fp32 ecdZeroCrossing(DJI_Motor *motor);
fp32 chassisFollowZeroCrossing(DJI_Motor *motor);
fp32 powerlimit(DJI_Motor *motor);
fp32 MIUecdZeroCrossing(DJI_Motor *motor);
fp32 yawMotorpostProcess(DJI_Motor *motor);

#endif
