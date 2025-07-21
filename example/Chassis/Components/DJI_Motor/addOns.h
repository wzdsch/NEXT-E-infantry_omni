#ifndef ADDONS_H
#define ADDONS_H

#include "DJI_Motor.h"
#include "struct_typedef.h"

#define ERROR_POWER_DISTRIBUTION_SET 200.0f
#define PROP_POWER_DISTRIBUTION_SET 150.0f

fp32 example(DJI_Motor *motor);
fp32 ecdZeroCrossing(DJI_Motor *motor);
fp32 chassisFollowZeroCrossing(DJI_Motor *motor);
fp32 powerlimit(DJI_Motor *motor);
fp32 powerlimit_pro(DJI_Motor *motor);
fp32 IMUecdZeroCrossing(DJI_Motor *motor);
fp32 LADRCtest(DJI_Motor *motor);
fp32 powerlimit_LVP_HK_pro(DJI_Motor *motor);

fp32 LADRC_YawControl(DJI_Motor *motor);
#endif
