#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "BLE_SEND.h"
#include "OSAL_Memory.h"
#include "simplekeys.h"

#include "battservice.h"

#define ECG_DR_125      125
#define ECG_DR_250      250
#define ECG_DR_500      500
#define ECG_DR_1000     1000

#define SAMPLENUM       128


STRREV strrev;

//static uint8 analystate=0;



extern  uint8 skKeyPressed[SEND_DATA_NUM_LEN];



void RevdataInit(void)
{
  osal_memset((uint8*)&strrev,0,sizeof(STRREV));
}

///*************************************************************************************************

//uint16 number=0;
static uint8 lastdata;
static uint8 revdatanum = 0;
extern uint8 Batt_Measure( void );


extern void UpdataRev(STRSEND * strdata);
void Rev_data_analy(uint8* data,uint8 len)
{
  //uint8 LOFFP,LOFFN;
  uint8 LeadOff_Time;
  int16 buffer;
  uint8 LOFF,LEAD_II_H,LEAD_II_L;
  LOFF = 0x00;
  
  revdatanum++;
  
  if(revdatanum >= 128)
  {
      revdatanum = 0;
  }

  //提取电极脱落状态和I导联数据；
  LOFF    = ((data[1]&0xf0)|((data[2]&0xf0)>>4));
  LEAD_II_H  = data[5];
  LEAD_II_L  = data[6];
#if 1  
  buffer = (LEAD_II_H<<8) + LEAD_II_L  +128 - 0x80;

  LEAD_II_L =buffer;
#endif 
#if 0    
 
  if(LEAD_II_H &0x80)   //负的
  {
    if(LEAD_II_L&0x80)
    {
      LEAD_II_L = 0x80;
    }
    else
    {
      LEAD_II_L =  LEAD_II_L|0x10;
    }
   // LEAD_II_L = 128-255;
  }
  else
  {
    if(LEAD_II_L&0x80)
    {
      LEAD_II_L = 0x7f;
    }
    else
    {
    LEAD_II_L = LEAD_II_L&0x7f;
    }
  }

 
#endif
  
 // LEAD_I  = revdatanum;
  

  //如果电极脱落，将数据处理为基线数值；对于ADS1194采集3导联ECG信号，最多5个电极。
  if((LOFF&0x22))
  {      
      LeadOff_Time =0;
      LEAD_II_L = 0x20;         //0x30为人为设置默认基线位置；RL电极脱落，状态不稳定，数据可能不会每次都清为0x30。
  }
  else
  {
      LeadOff_Time++;
      if(LeadOff_Time>250)
      {
        LeadOff_Time=120;
      }
      if(LeadOff_Time >100)
      {
        LEAD_II_L = 0x20;  
      }
  }
#if 0
  //将125个采样点在1s中时间内插入3个点；
 
  if((revdatanum==30)||(revdatanum==70)||(revdatanum==110))
  {
    strrev.strsenddata.data[strrev.len++] = (lastdata + LEAD_II_L)/2; 
//    strrev.strsenddata.data[strrev.len++] = revdatanum;

    //如果插值后正好16 bit的长度，就直接发送出去；
    if(strrev.len >= MAXNUM)
    {   
        //每16 Bit的数据发送一次电量和电极状态；    
        strrev.strsenddata.state[0] = Batt_Measure();
        strrev.strsenddata.state[1] = LOFF;
        UpdataRev(&strrev.strsenddata);
        strrev.len = 0;
    }
  }
  
#endif
  //Simple Key Profile中只能发送16 Bit的数据，将16次中断数据中的Lead I提取、组合成一个发送包；
   strrev.strsenddata.data[strrev.len++] = LEAD_II_L;
//  strrev.strsenddata.data[strrev.len++] = revdatanum;
  if(strrev.len >= MAXNUM)
  {   
      //每16 Bit的数据发送一次电量和电极状态；
      strrev.strsenddata.state[0] = Batt_Measure();
      strrev.strsenddata.state[1] = LOFF;
      UpdataRev(&strrev.strsenddata);
      strrev.len=0;
      RevdataInit();
  }

  lastdata = LEAD_II_L;                 //保存上次的数据，以供插平均值；
}

//**************************************************************************************************/


