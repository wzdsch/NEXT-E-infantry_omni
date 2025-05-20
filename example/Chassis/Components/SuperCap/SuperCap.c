#include "Supercap.h"

supercap_rx_t supercap_rxD;
supercap_tx_t supercap_txD;
uint8_t referee_rx_flg = 0;
uint8_t supercap_rx_flg = 0;

fp32 Supercap_powerlimit(DJI_Motor *motor) {

  fp32 chassis_max_power = 0;       // input power from battery (referee system)
  float initial_give_power = 0;  // initial power from PID calculation
  static float initial_total_power = 0;
  static float initial_total_power_last = 0;
  static fp32 scaled_give_power = 0;

  fp32 toque_coefficient = 1.99688994e-6f;  // (20/16384)*(0.3)*(187/3591)/9.55
  fp32 a = 1.23e-07;                        // k1
  fp32 k2 = 1.453e-07;                      // k2
  fp32 constant = 4.081f;

  //supercap.max_cap_power = supercap.raw_cap_power/100;
  chassis_max_power = supercap_rxD.max_cap_power + robot_state.chassis_power_limit;

  static uint8_t count = 0;

  initial_give_power = motor->pidOutput0 * toque_coefficient * motor->realSpeed
                       + k2 * motor->realSpeed * motor->realSpeed
                       + a * motor->pidOutput0 * motor->pidOutput0 + constant;
  if (initial_give_power >= 0) {  // negative power not included (transitory)
    initial_total_power += initial_give_power;
  }
  count++;

  if (count == 4) {
    count = 0;
    initial_total_power_last = initial_total_power;
    initial_total_power = 0;
  }

  if (initial_total_power_last > chassis_max_power)  // determine if larger than max power
  {
    fp32 power_scale = chassis_max_power / initial_total_power_last;
    scaled_give_power = initial_give_power * power_scale;  // get scaled power
    if (scaled_give_power < 0) {
      return motor->pidOutput0;
    }

    fp32 b = toque_coefficient * motor->realSpeed;
    fp32 c = k2 * motor->realSpeed * motor->realSpeed - scaled_give_power + constant;
    if (motor->pidOutput0 > 0)  // Selection of the calculation formula according
                                // to the direction of the original moment
    {
      fp32 temp = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
      if (temp > 16000) {
        return 16000;
      }
      else
        return temp;
    }
    else {
      fp32 temp = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
      if (temp < -16000) {
        return -16000;
      }
      else
        return temp;
    }
  }
  else {
    return motor->pidOutput0;
  }
}

void Supercap_unpack(supercap_rx_t *supercap)
{
  supercap->head = supercap->rx_buf[0] << 8 | supercap->rx_buf[1];
  if (supercap->head == SUPERCAP_RX_HEAD && supercap_rx_flg == 1)
  {
    supercap->raw_cap_power = (supercap->rx_buf[2] | supercap->rx_buf[3] << 8);
    supercap->max_cap_power = (supercap->rx_buf[2] | supercap->rx_buf[3] << 8) * 0.01f;
    supercap->cap_percent = (supercap->rx_buf[4] | supercap->rx_buf[5] << 8);
    supercap->input_power = (supercap->rx_buf[6] | supercap->rx_buf[7] << 8) * 0.01f;
    supercap->err_code = (supercap->rx_buf[8] | supercap->rx_buf[9] << 8);
    if(supercap->max_cap_power >= 45) // 保证超电提供的功率不超过50W
    {
      supercap_rxD.max_cap_power = 45;
    }
    else if (supercap->max_cap_power < 0)
    {
      supercap_rxD.max_cap_power = 0;
    }

    if (supercap->err_code != 0) // 超电报错，改回原有功率限制
    {
      if (referee_rx_flg == 1)
      {
        DJI_MotorPostProcessHandlerSet(&motor1, &powerlimit_pro);
        DJI_MotorPostProcessHandlerSet(&motor2, &powerlimit_pro);
        DJI_MotorPostProcessHandlerSet(&motor3, &powerlimit_pro);
        DJI_MotorPostProcessHandlerSet(&motor4, &powerlimit_pro);
        
        DJI_MotorCalculateResultSet(&motor1, &(motor1.postProcessResult));
        DJI_MotorCalculateResultSet(&motor2, &(motor2.postProcessResult));
        DJI_MotorCalculateResultSet(&motor3, &(motor3.postProcessResult));
        DJI_MotorCalculateResultSet(&motor4, &(motor4.postProcessResult));
      }
      else
      {
        DJI_MotorCalculateResultSet(&motor1, &(motor1.pidOutput0));
        DJI_MotorCalculateResultSet(&motor2, &(motor2.pidOutput0));
        DJI_MotorCalculateResultSet(&motor3, &(motor3.pidOutput0));
        DJI_MotorCalculateResultSet(&motor4, &(motor4.pidOutput0));
      }
    }
    else //超电正常，改回超电功率限制
    {
      DJI_MotorPostProcessHandlerSet(&motor1, &Supercap_powerlimit);
      DJI_MotorPostProcessHandlerSet(&motor2, &Supercap_powerlimit);
      DJI_MotorPostProcessHandlerSet(&motor3, &Supercap_powerlimit);
      DJI_MotorPostProcessHandlerSet(&motor4, &Supercap_powerlimit);

      DJI_MotorCalculateResultSet(&motor1, &(motor1.postProcessResult));
      DJI_MotorCalculateResultSet(&motor2, &(motor2.postProcessResult));
      DJI_MotorCalculateResultSet(&motor3, &(motor3.postProcessResult));
      DJI_MotorCalculateResultSet(&motor4, &(motor4.postProcessResult));
    }
  
  }
  else // 收不到超电数据/收到的数据错误，改回原有功率限制
  {
    if (referee_rx_flg == 1)
    {
      DJI_MotorPostProcessHandlerSet(&motor1, &powerlimit_pro);
      DJI_MotorPostProcessHandlerSet(&motor2, &powerlimit_pro);
      DJI_MotorPostProcessHandlerSet(&motor3, &powerlimit_pro);
      DJI_MotorPostProcessHandlerSet(&motor4, &powerlimit_pro);
      
      DJI_MotorCalculateResultSet(&motor1, &(motor1.postProcessResult));
      DJI_MotorCalculateResultSet(&motor2, &(motor2.postProcessResult));
      DJI_MotorCalculateResultSet(&motor3, &(motor3.postProcessResult));
      DJI_MotorCalculateResultSet(&motor4, &(motor4.postProcessResult));
    }
    else
    {
      DJI_MotorCalculateResultSet(&motor1, &(motor1.pidOutput0));
      DJI_MotorCalculateResultSet(&motor2, &(motor2.pidOutput0));
      DJI_MotorCalculateResultSet(&motor3, &(motor3.pidOutput0));
      DJI_MotorCalculateResultSet(&motor4, &(motor4.pidOutput0));
    }
  }
}

void Supercap_transmit(UART_HandleTypeDef* huart, supercap_tx_t* supercap_tx) {
    HAL_UART_Transmit_DMA(huart, (uint8_t*)supercap_tx, sizeof(supercap_tx_t));
}

void Supercap_update_txd(supercap_tx_t* supercap_tx, robot_state_t* const robot_state) {
  supercap_tx->head = SUPERCAP_TX_HEAD;
  if (referee_rx_flg == 1) {
    supercap_tx->chassis_power_state = robot_state->power_management_chassis_output;
    supercap_tx->chassis_power_limit = robot_state->chassis_power_limit;
  }
  else {
    supercap_tx->chassis_power_limit = 60;
    supercap_tx->chassis_power_state = 1;
  }
}
