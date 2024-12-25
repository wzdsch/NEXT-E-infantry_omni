//
// Created by RM UI Designer
//

#include "ui_default_Vision_0.h"

#define FRAME_ID 0
#define GROUP_ID 1
#define START_ID 0
#define OBJ_NUM 1
#define FRAME_OBJ_NUM 1

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_Vision_0;
ui_interface_number_t *ui_default_Vision_Vision = (ui_interface_number_t *)&(ui_default_Vision_0.data[0]);

void _ui_init_default_Vision_0(char num) {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Vision_0.data[i].figure_name[0] = FRAME_ID;
        ui_default_Vision_0.data[i].figure_name[1] = GROUP_ID;
        ui_default_Vision_0.data[i].figure_name[2] = i + START_ID;
        ui_default_Vision_0.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_default_Vision_0.data[i].operate_tpyel = 0;
    }

    ui_default_Vision_Vision->figure_tpye = 6;
    ui_default_Vision_Vision->layer = 0;
    ui_default_Vision_Vision->font_size = 20;
    ui_default_Vision_Vision->start_x = 1121;
    ui_default_Vision_Vision->start_y = 563;
    ui_default_Vision_Vision->color = 2;
    ui_default_Vision_Vision->number = num;
    ui_default_Vision_Vision->width = 2;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Vision_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Vision_0, sizeof(ui_default_Vision_0));
}

void _ui_update_default_Vision_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Vision_0.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Vision_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Vision_0, sizeof(ui_default_Vision_0));
}

void _ui_remove_default_Vision_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Vision_0.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Vision_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Vision_0, sizeof(ui_default_Vision_0));
}
