/*
 * @File         : 
 * @Author       : JL HUANG
 * @Date         : 2023-12-24 16:16:31
 * @LastEditTime : 2023-12-30 11:16:48
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\balance\Application\RC_task.h
 * @Description  : 
 */

#ifndef __RC_TASK_H
#define __RC_TASK_H

#include "main.h"

#include "app_et07.h"

typedef struct
{
  ET07_Data_t ET07_DATA;
}RC_Data_t;

extern RC_Data_t RC_Data;

#endif
