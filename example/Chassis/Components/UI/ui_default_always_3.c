//
// Created by RM UI Designer
//

#include "ui_default_always_3.h"

#define FRAME_ID 0
#define GROUP_ID 0
#define START_ID 3
#define OBJ_NUM 7
#define FRAME_OBJ_NUM 7

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_always_3;
ui_interface_line_t *ui_default_always_NewLine1 = (ui_interface_line_t *)&(ui_default_always_3.data[0]);
ui_interface_line_t *ui_default_always_NewLine2 = (ui_interface_line_t *)&(ui_default_always_3.data[1]);
ui_interface_line_t *ui_default_always_NewLine3 = (ui_interface_line_t *)&(ui_default_always_3.data[2]);
ui_interface_line_t *ui_default_always_NewLine4 = (ui_interface_line_t *)&(ui_default_always_3.data[3]);
ui_interface_line_t *ui_default_always_NewLine5 = (ui_interface_line_t *)&(ui_default_always_3.data[4]);
ui_interface_line_t *ui_default_always_NewLine6 = (ui_interface_line_t *)&(ui_default_always_3.data[5]);
ui_interface_line_t *ui_default_always_NewLine7 = (ui_interface_line_t *)&(ui_default_always_3.data[6]);

void _ui_init_default_always_3() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_always_3.data[i].figure_name[0] = FRAME_ID;
        ui_default_always_3.data[i].figure_name[1] = GROUP_ID;
        ui_default_always_3.data[i].figure_name[2] = i + START_ID;
        ui_default_always_3.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_default_always_3.data[i].operate_tpyel = 0;
    }

    ui_default_always_NewLine1->figure_tpye = 0;
    ui_default_always_NewLine1->layer = 0;
    ui_default_always_NewLine1->start_x = 610;
    ui_default_always_NewLine1->start_y = 20;
    ui_default_always_NewLine1->end_x = 930;
    ui_default_always_NewLine1->end_y = 510;
    ui_default_always_NewLine1->color = 2;
    ui_default_always_NewLine1->width = 3;

    ui_default_always_NewLine2->figure_tpye = 0;
    ui_default_always_NewLine2->layer = 0;
    ui_default_always_NewLine2->start_x = 1350;
    ui_default_always_NewLine2->start_y = 20;
    ui_default_always_NewLine2->end_x = 990;
    ui_default_always_NewLine2->end_y = 510;
    ui_default_always_NewLine2->color = 2;
    ui_default_always_NewLine2->width = 3;

    ui_default_always_NewLine3->figure_tpye = 0;
    ui_default_always_NewLine3->layer = 0;
    ui_default_always_NewLine3->start_x = 934;
    ui_default_always_NewLine3->start_y = 499;
    ui_default_always_NewLine3->end_x = 987;
    ui_default_always_NewLine3->end_y = 499;
    ui_default_always_NewLine3->color = 0;
    ui_default_always_NewLine3->width = 3;

    ui_default_always_NewLine4->figure_tpye = 0;
    ui_default_always_NewLine4->layer = 0;
    ui_default_always_NewLine4->start_x = 915;
    ui_default_always_NewLine4->start_y = 539;
    ui_default_always_NewLine4->end_x = 1005;
    ui_default_always_NewLine4->end_y = 539;
    ui_default_always_NewLine4->color = 0;
    ui_default_always_NewLine4->width = 3;

    ui_default_always_NewLine5->figure_tpye = 0;
    ui_default_always_NewLine5->layer = 0;
    ui_default_always_NewLine5->start_x = 925;
    ui_default_always_NewLine5->start_y = 520;
    ui_default_always_NewLine5->end_x = 997;
    ui_default_always_NewLine5->end_y = 520;
    ui_default_always_NewLine5->color = 0;
    ui_default_always_NewLine5->width = 3;

    ui_default_always_NewLine6->figure_tpye = 0;
    ui_default_always_NewLine6->layer = 0;
    ui_default_always_NewLine6->start_x = 944;
    ui_default_always_NewLine6->start_y = 480;
    ui_default_always_NewLine6->end_x = 973;
    ui_default_always_NewLine6->end_y = 480;
    ui_default_always_NewLine6->color = 0;
    ui_default_always_NewLine6->width = 3;

    ui_default_always_NewLine7->figure_tpye = 0;
    ui_default_always_NewLine7->layer = 0;
    ui_default_always_NewLine7->start_x = 960;
    ui_default_always_NewLine7->start_y = 541;
    ui_default_always_NewLine7->end_x = 960;
    ui_default_always_NewLine7->end_y = 440;
    ui_default_always_NewLine7->color = 0;
    ui_default_always_NewLine7->width = 3;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_always_3);
    SEND_MESSAGE((uint8_t *) &ui_default_always_3, sizeof(ui_default_always_3));
}

void _ui_update_default_always_3() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_always_3.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_always_3);
    SEND_MESSAGE((uint8_t *) &ui_default_always_3, sizeof(ui_default_always_3));
}

void _ui_remove_default_always_3() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_always_3.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_always_3);
    SEND_MESSAGE((uint8_t *) &ui_default_always_3, sizeof(ui_default_always_3));
}
