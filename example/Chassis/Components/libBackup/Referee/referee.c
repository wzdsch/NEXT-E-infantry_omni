#include "main.h"
#include "referee.h"
#include "refereeData_v1.4.h"
//#include "refereeData_v1.5.h"
#include "string.h"
#include "main.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"
#include "fifo.h"
#include "struct_typedef.h"
#include "tim.h"

frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

game_state_t game_state;                             // 0x0001
game_result_t game_result;                           // 0x0002
game_robot_HP_t game_robot_HP;                       // 0x0003
event_data_t event_data;                             // 0x0101
supply_projectile_action_t supply_projectile_action; // 0x0102
referee_warning_t referee_warning;                   // 0x0104
dart_remaining_time_t dart_remaining_time;           // 0x0105
robot_state_t robot_state;                           // 0x0201
power_heat_data_t power_heat_data;                   // 0x0202
game_robot_pos_t game_robot_pos;                     // 0x0203
robot_buff_t robot_buff;                             // x0204
air_support_data_t air_support_data;                 // 0x0205
hurt_data_t hurt_data;                               // 0x0206
shoot_data_t shoot_data;                             // 0x0207
projectile_allowance_t projectile_allowance;         // 0x0208
rfid_status_t rfid_status;                           // 0x0209
dart_client_cmd_t dart_client_cmd;                   // 0x020A
ground_robot_position_t ground_robot_position;       // 0x020B
radar_mark_data_t radar_mark_data;                   // 0x020C
robot_interaction_data_t robot_interaction_data;     // 0x0301
referee_remote_control_t referee_remote_control;     // 0x0304

void init_referee_struct_data(void) {
  memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
  memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

  memset(&game_state, 0, sizeof(game_state_t));
  memset(&game_result, 0, sizeof(game_result_t));
  memset(&game_robot_HP, 0, sizeof(game_robot_HP_t));

  memset(&event_data, 0, sizeof(event_data_t));
  memset(&supply_projectile_action, 0, sizeof(supply_projectile_action_t));
  memset(&referee_warning, 0, sizeof(referee_warning_t));

  memset(&dart_remaining_time, 0, sizeof(dart_remaining_time_t));
  memset(&robot_state, 0, sizeof(robot_state_t));
  memset(&power_heat_data, 0, sizeof(power_heat_data_t));
  memset(&game_robot_pos, 0, sizeof(game_robot_pos_t));
  memset(&robot_buff, 0, sizeof(robot_buff_t));
  memset(&air_support_data, 0, sizeof(air_support_data_t));
  memset(&hurt_data, 0, sizeof(hurt_data_t));
  memset(&shoot_data, 0, sizeof(shoot_data_t));
  memset(&projectile_allowance, 0, sizeof(projectile_allowance_t));
  memset(&rfid_status, 0, sizeof(rfid_status_t));
  memset(&dart_client_cmd, 0, sizeof(dart_client_cmd_t));
  memset(&ground_robot_position, 0, sizeof(ground_robot_position_t));
  memset(&radar_mark_data, 0, sizeof(radar_mark_data_t));
  memset(&robot_interaction_data, 0, sizeof(robot_interaction_data_t));
  memset(&referee_remote_control, 0, sizeof(referee_remote_control_t));
}

void referee_data_solve(uint8_t *frame) {
  uint16_t cmd_id = 0;

  uint8_t index = 0;

  memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

  index += sizeof(frame_header_struct_t);

  memcpy(&cmd_id, frame + index, sizeof(uint16_t));
  index += sizeof(uint16_t);

  switch (cmd_id) {
  case GAME_STATE_CMD_ID: {
    memcpy(&game_state, frame + index, sizeof(game_state_t));
  } break;
  case GAME_RESULT_CMD_ID: {
    memcpy(&game_result, frame + index, sizeof(game_result_t));
  } break;
  case GAME_ROBOT_HP_CMD_ID: {
    memcpy(&game_robot_HP, frame + index, sizeof(game_robot_HP_t));
  } break;
  case EVENTS_CMD_ID: {
    memcpy(&event_data, frame + index, sizeof(event_data_t));
  } break;
  case SUPPLY_PROJECTILE_ACTION_CMD_ID: {
    memcpy(&supply_projectile_action, frame + index,
           sizeof(supply_projectile_action_t));
  } break;

  case REFEREE_WARNING_CMD_ID: {
    memcpy(&referee_warning, frame + index, sizeof(referee_warning_t));
  } break;
  case DART_REMAINING_TIME_CMD_ID: {
    memcpy(&dart_remaining_time, frame + index, sizeof(dart_remaining_time_t));
  } break;
  case ROBOT_STATE_CMD_ID: {
    memcpy(&robot_state, frame + index, sizeof(robot_state_t));
  } break;
  case POWER_HEAT_DATA_CMD_ID: {
    memcpy(&power_heat_data, frame + index, sizeof(power_heat_data_t));
  } break;
  case GAME_ROBOT_POS_CMD_ID: {
    memcpy(&game_robot_pos, frame + index, sizeof(game_robot_pos_t));
  } break;
  case ROBOT_BUFF_CMD_ID: {
    memcpy(&robot_buff, frame + index, sizeof(robot_buff_t));
  } break;
  case AIR_SUPPORT_CMD_ID: {
    memcpy(&air_support_data, frame + index, sizeof(air_support_data_t));
  } break;
  case HURT_DATA_CMD_ID: {
    memcpy(&hurt_data, frame + index, sizeof(hurt_data_t));
  } break;
  case SHOOT_DATA_CMD_ID: {
    memcpy(&shoot_data, frame + index, sizeof(shoot_data_t));
  } break;
  case PROJECTILE_ALLOWANCE_CMD_ID: {
    memcpy(&projectile_allowance, frame + index,
           sizeof(projectile_allowance_t));
  } break;
  case RFID_STATUS_CMD_ID: {
    memcpy(&rfid_status, frame + index, sizeof(rfid_status_t));
  } break;
  case DART_CLIENT_CMD_CMD_ID: {
    memcpy(&dart_client_cmd, frame + index, sizeof(dart_client_cmd_t));
  } break;
  case GROUND_ROBOT_POSITION_CMD_ID: {
    memcpy(&ground_robot_position, frame + index,
           sizeof(ground_robot_position_t));
  } break;
  case RADAR_MARK_DATA_CMD_ID: {
    memcpy(&radar_mark_data, frame + index, sizeof(radar_mark_data_t));
  } break;
  case ROBOT_INTERACTION_DATA_CMD_ID: {
    memcpy(&robot_interaction_data, frame + index,
           sizeof(robot_interaction_data_t));
  } break;
  case REFEREE_REMOTE_CONTROL_CMD_ID: {
    memcpy(&referee_remote_control, frame + index,
           sizeof(referee_remote_control_t));
  } break;
  default: { break; }
  }
}

void refereeINIT(referee *referee, UART_HandleTypeDef *huart) {
  referee->huart = huart;
  init_referee_struct_data();
  HAL_UARTEx_ReceiveToIdle_DMA(referee->huart, referee->REF_rxd,
                               sizeof(referee->REF_rxd));
}

void refereeRxCpltCallBack(referee *referee, UART_HandleTypeDef *huart) {
  if (huart == referee->huart) {
    for (uint8_t i = 0; i < sizeof(referee->REF_rxd); i++) {
      referee->REF_rxd[i] = 0;
    }
    HAL_UARTEx_ReceiveToIdle_DMA(referee->huart, referee->REF_rxd,
                                 sizeof(referee->REF_rxd));
  }
}

void referee_unpack(referee *referee) {

  if (referee->REF_rxd[0] == HEADER_SOF) {
    uint16_t length = referee->REF_rxd[1] + (referee->REF_rxd[2] << 8);
    if (length < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN)) {
      if (verify_CRC8_check_sum(referee->REF_rxd, REF_PROTOCOL_HEADER_SIZE)) {
        if (verify_CRC16_check_sum(referee->REF_rxd,
                                   REF_HEADER_CRC_CMDID_LEN + length)) {
          referee_data_solve(referee->REF_rxd); //这里是真正的对数据进行解包
        }
      }
    }
  }
}

void refereeRxEventCallBack(referee *referee, UART_HandleTypeDef *huart) {
  if (huart == referee->huart) {

    HAL_UARTEx_ReceiveToIdle_DMA(referee->huart, referee->REF_rxd,
                                 sizeof(referee->REF_rxd));
  }
}
