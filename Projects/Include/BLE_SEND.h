#ifndef BLE_SEND_H_
#define BLE_SEND_H_

#include "hal_types.h"

#define MAXNUM 16

typedef struct 
{
  uint8 state[2];
  uint8 data[MAXNUM];
}STRSEND;

typedef struct 
{
  uint8 len;
  STRSEND strsenddata;
}STRREV;

extern void Rev_data_analy(uint8* data,uint8 len);

#endif
