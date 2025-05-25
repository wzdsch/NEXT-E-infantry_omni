//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

extern ui_interface_figure_t ui_g_now_figures[5];
extern uint8_t ui_g_dirty_figure[5];

#define ui_g_dynamic_graph_super_cap_persent ((ui_interface_number_t*)&(ui_g_now_figures[0]))
#define ui_g_aim_auto_aim_scale ((ui_interface_rect_t*)&(ui_g_now_figures[1]))
#define ui_g_aim_aim_line_l ((ui_interface_line_t*)&(ui_g_now_figures[2]))
#define ui_g_aim_aim_line_r ((ui_interface_line_t*)&(ui_g_now_figures[3]))
#define ui_g_aim_aim_line_center ((ui_interface_line_t*)&(ui_g_now_figures[4]))


#ifdef MANUAL_DIRTY
#define ui_g_dynamic_graph_super_cap_persent_dirty (ui_g_dirty_figure[0])
#define ui_g_aim_auto_aim_scale_dirty (ui_g_dirty_figure[1])
#define ui_g_aim_aim_line_l_dirty (ui_g_dirty_figure[2])
#define ui_g_aim_aim_line_r_dirty (ui_g_dirty_figure[3])
#define ui_g_aim_aim_line_center_dirty (ui_g_dirty_figure[4])

#endif

void ui_init_g(void);
void ui_update_g(void);

#endif // UI_g_H
