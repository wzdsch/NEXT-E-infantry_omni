#include "RobotModManage.h"

#include "MCUConnect.h"
#include "remote_control.h"

void ModINIT(RobotMod *mod) {
  mod->mod = ROBOT_STOP;
}

void ModManage(RobotMod *mod) {
  static uint8_t last_sw = RC_SW_MID;
  if (rc_ctrl.rc.s[0] == RC_SW_MID) {
    mod->mod = ROBOT_FOLLOW;
  }
  if (rc_ctrl.rc.s[0] == RC_SW_UP) {
    mod->mod = ROBOT_FREE;
  }

  if (rc_ctrl.rc.s[0] == RC_SW_DOWN && rc_ctrl.rc.s[1] == RC_SW_DOWN) {
    mod->mod = ROBOT_STOP;
  }
  else if (rc_ctrl.rc.s[0] == RC_SW_DOWN && rc_ctrl.rc.s[1] != RC_SW_DOWN) {
    if (rc_ctrl.rc.s[1] == RC_SW_MID) {
      if (last_sw != rc_ctrl.rc.s[1]) {
        last_sw = rc_ctrl.rc.s[1];
        if (mod->mod < 3) {
          mod->mod = 3;
          tuneTime = mod->mod;
        }
        else {
          mod->mod++;
          if (mod->mod > 6) {
            mod->mod = 3;
          }
          tuneTime = mod->mod;
        }
      }

      if (mod->mod > 6) {
        mod->mod = 3;
      }
    }
    if (rc_ctrl.rc.s[1] == RC_SW_UP) {
      if (last_sw != rc_ctrl.rc.s[1]) {
        last_sw = rc_ctrl.rc.s[1];
      }
    }
  }
}
