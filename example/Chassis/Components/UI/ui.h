//
// Created by RM UI Designer
//

#ifndef UI_H
#define UI_H
#ifdef __cplusplus
extern "C" {
#endif

#include "ui_interface.h"

#include "ui_default_always_0.h"
#include "ui_default_always_1.h"
#include "ui_default_always_2.h"
#include "ui_default_always_3.h"

#define ui_init_default_always() \
_ui_init_default_always_0(); \
_ui_init_default_always_1(); \
_ui_init_default_always_2(); \
_ui_init_default_always_3()

#define ui_update_default_always() \
_ui_update_default_always_0(); \
_ui_update_default_always_1(); \
_ui_update_default_always_2(); \
_ui_update_default_always_3()

#define ui_remove_default_always() \
_ui_remove_default_always_0(); \
_ui_remove_default_always_1(); \
_ui_remove_default_always_2(); \
_ui_remove_default_always_3()
    

#include "ui_default_Vision_0.h"

#define ui_init_default_Vision(temp) \
_ui_init_default_Vision_0(temp)

#define ui_update_default_Vision() \
_ui_update_default_Vision_0()

#define ui_remove_default_Vision() \
_ui_remove_default_Vision_0()
    

#include "ui_default_Chassis_0.h"

#define ui_init_default_Chassis(num) \
_ui_init_default_Chassis_0(num)

#define ui_update_default_Chassis() \
_ui_update_default_Chassis_0()

#define ui_remove_default_Chassis() \
_ui_remove_default_Chassis_0()
    

#include "ui_default_Cover_0.h"

#define ui_init_default_Cover(num) \
_ui_init_default_Cover_0(num)

#define ui_update_default_Cover() \
_ui_update_default_Cover_0()

#define ui_remove_default_Cover() \
_ui_remove_default_Cover_0()
    


#ifdef __cplusplus
}
#endif

#endif //UI_H
