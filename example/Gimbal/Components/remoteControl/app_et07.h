/*
 * @File         : 
 * @Author       : JL HUANG
 * @Date         : 2023-12-24 16:14:20
 * @LastEditTime : 2023-12-30 11:17:49
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\balance\Components\Devices\app_et07.h
 * @Description  : 
 */

#ifndef __APP_ET07_H
#define __APP_ET07_H

#include "main.h"


typedef struct 
{
  uint8_t ET07_Start;

  int16_t RightTransverse_CH1;
  int16_t RightDirection_CH2;
  int16_t LeftDirection_CH3;
  int16_t LeftTransverse_CH4;
  int16_t SA_CH5;
  int16_t SB_CH6;
  int16_t SC_CH7;
  int16_t SD_CH8;
  int16_t V1_CH9;
  int16_t V2_CH10;
//  int16_t CH11;
//  int16_t CH12;
//  int16_t CH13;
//  int16_t CH14;
//  int16_t CH15;
//  int16_t CH16;

  uint8_t ET07_Flag;
  uint8_t ET07_End;
}ET07_Data_t;


void Get_ET07_DataDecode(ET07_Data_t* et07_data_t,uint8_t* rx_buf);

#endif
