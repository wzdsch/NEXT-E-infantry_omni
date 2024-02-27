#ifndef ROBOT_MOD_MANAGE_H
#define ROBOT_MOD_MANAGE_H
#include "struct_typedef.h"

typedef struct {
  uint8_t mod;
} RobotMod;

extern uint8_t tuneTime;

extern RobotMod mod;

extern void ModINIT(RobotMod *mod);
extern void ModManage(RobotMod *mod);

#endif



