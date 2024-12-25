#include "ui_draw.h"

#include "MCUConnectStructs.h"
#include "main.h"
#include "ui.h"
#include "usart.h"
uint8_t first_flag = 0;

uint8_t Last_Vision = 0;
uint8_t Last_Cover = 0;
uint8_t Last_Mode = 0;

void my_send_message(const uint8_t* message, int length) {
  HAL_UART_Transmit(&huart6, message, length, 10);
}

extern char Vision_Handle(void);
extern char Cover_Handle(void);
extern char Chassis_Handle(void);

void UI_Init() {
  if (first_flag < 5) {
    Last_Mode = ChassisControlData.mode;
    for (int i = 0; i < 5; i++) {
      ui_init_default_Vision(0);
      HAL_Delay(50);
      ui_init_default_Chassis(0);
      HAL_Delay(50);
      ui_init_default_Cover(0);
      HAL_Delay(50);
    }
    first_flag++;
  }
  _ui_init_default_always_0();
  HAL_Delay(100);
  _ui_init_default_always_1();
  HAL_Delay(100);
  _ui_init_default_always_2();
  HAL_Delay(100);
  _ui_init_default_always_3();
  HAL_Delay(100);
}

void UI_Refresh() {
  static uint8_t Handle_Order = 0;
  switch (Handle_Order) {
    case 0:
      if (Vision_Handle())
        break;
    case 1:
      if (Cover_Handle())
        break;
    case 2:
      if (Chassis_Handle())
        break;
  }
  Handle_Order++;
  if (Handle_Order == 3) {
    Handle_Order = 0;
  }
}

char Vision_Handle() {
  if (ChassisControlData.AutoAim != Last_Vision) {
    Last_Vision = ChassisControlData.AutoAim;
    for (int i = 0; i < 5; i++) {
      _ui_init_default_Vision_0(Last_Vision);
    }
    for (int i = 0; i < 30; i++) {
      _ui_update_default_Vision_0();
    }
    return 1;
  }
  return 0;
}

char Cover_Handle() {
  if (ChassisControlData.cover != Last_Cover) {
    Last_Cover = ChassisControlData.cover;
    for (int i = 0; i < 5; i++) {
      _ui_init_default_Cover_0(Last_Cover);
    }
    for (int i = 0; i < 30; i++) {
      _ui_update_default_Cover_0();
    }
    return 1;
  }
  return 0;
}

char Chassis_Handle() {
  if (ChassisControlData.mode != Last_Mode) {
    Last_Mode = ChassisControlData.mode;
    for (int i = 0; i < 5; i++) {
      _ui_init_default_Chassis_0(Last_Mode);
    }
    for (int i = 0; i < 30; i++) {
      _ui_update_default_Chassis_0();
    }
    return 1;
  }
  return 0;
}
