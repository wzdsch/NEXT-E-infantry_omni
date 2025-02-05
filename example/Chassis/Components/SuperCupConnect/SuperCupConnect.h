#ifndef CUPERCUPCONNECT_H
#define CUPERCUPCONNECT_H
#include "main.h"
#include "struct_typedef.h"

typedef struct supcup_t {
  fp32 P_cup;
  fp32 P_chassis;
  fp32 P_referee;
  fp32 cupstate;
  fp32 P_referee_max;
} supercup;

#endif
