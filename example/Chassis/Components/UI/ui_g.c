//
// Created by RM UI Designer
// Dynamic Edition
//

#include "string.h"
#include "ui_interface.h"
#include "ui_g.h"

#define TOTAL_FIGURE 5
#define TOTAL_STRING 0

ui_interface_figure_t ui_g_now_figures[TOTAL_FIGURE];
uint8_t ui_g_dirty_figure[TOTAL_FIGURE];

#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_g_last_figures[TOTAL_FIGURE];
#endif

#define SCAN_AND_SEND() ui_scan_and_send(ui_g_now_figures, ui_g_dirty_figure, NULL, NULL, TOTAL_FIGURE, TOTAL_STRING)

void ui_init_g() {
    ui_g_dynamic_graph_super_cap_persent->figure_type = 6;
    ui_g_dynamic_graph_super_cap_persent->operate_type = 1;
    ui_g_dynamic_graph_super_cap_persent->layer = 0;
    ui_g_dynamic_graph_super_cap_persent->color = 1;
    ui_g_dynamic_graph_super_cap_persent->start_x = 297;
    ui_g_dynamic_graph_super_cap_persent->start_y = 753;
    ui_g_dynamic_graph_super_cap_persent->width = 5;
    ui_g_dynamic_graph_super_cap_persent->font_size = 50;
    ui_g_dynamic_graph_super_cap_persent->number = 50;

    ui_g_aim_auto_aim_scale->figure_type = 1;
    ui_g_aim_auto_aim_scale->operate_type = 1;
    ui_g_aim_auto_aim_scale->layer = 0;
    ui_g_aim_auto_aim_scale->color = 1;
    ui_g_aim_auto_aim_scale->start_x = 408;
    ui_g_aim_auto_aim_scale->start_y = 327;
    ui_g_aim_auto_aim_scale->width = 5;
    ui_g_aim_auto_aim_scale->end_x = 1533;
    ui_g_aim_auto_aim_scale->end_y = 833;

    ui_g_aim_aim_line_l->figure_type = 0;
    ui_g_aim_aim_line_l->operate_type = 1;
    ui_g_aim_aim_line_l->layer = 0;
    ui_g_aim_aim_line_l->color = 3;
    ui_g_aim_aim_line_l->start_x = 748;
    ui_g_aim_aim_line_l->start_y = 705;
    ui_g_aim_aim_line_l->width = 5;
    ui_g_aim_aim_line_l->end_x = 950;
    ui_g_aim_aim_line_l->end_y = 502;

    ui_g_aim_aim_line_r->figure_type = 0;
    ui_g_aim_aim_line_r->operate_type = 1;
    ui_g_aim_aim_line_r->layer = 0;
    ui_g_aim_aim_line_r->color = 3;
    ui_g_aim_aim_line_r->start_x = 1151;
    ui_g_aim_aim_line_r->start_y = 705;
    ui_g_aim_aim_line_r->width = 5;
    ui_g_aim_aim_line_r->end_x = 949;
    ui_g_aim_aim_line_r->end_y = 502;

    ui_g_aim_aim_line_center->figure_type = 0;
    ui_g_aim_aim_line_center->operate_type = 1;
    ui_g_aim_aim_line_center->layer = 0;
    ui_g_aim_aim_line_center->color = 0;
    ui_g_aim_aim_line_center->start_x = 951;
    ui_g_aim_aim_line_center->start_y = 799;
    ui_g_aim_aim_line_center->width = 2;
    ui_g_aim_aim_line_center->end_x = 951;
    ui_g_aim_aim_line_center->end_y = 490;

    uint32_t idx = 0;
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].figure_name[2] = idx & 0xFF;
        ui_g_now_figures[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_figures[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_figures[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_g_last_figures[i] = ui_g_now_figures[i];
#endif
        ui_g_dirty_figure[i] = 1;
        idx++;
    }

    SCAN_AND_SEND();

    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].operate_type = 2;
    }
}

void ui_update_g() {
#ifndef MANUAL_DIRTY
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        if (memcmp(&ui_g_now_figures[i], &ui_g_last_figures[i], sizeof(ui_g_now_figures[i])) != 0) {
            ui_g_dirty_figure[i] = 1;
            ui_g_last_figures[i] = ui_g_now_figures[i];
        }
    }
#endif
    SCAN_AND_SEND();
}
