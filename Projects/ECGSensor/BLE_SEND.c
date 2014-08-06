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

  //��ȡ�缫����״̬��I�������ݣ�
  LOFF    = ((data[1]&0xf0)|((data[2]&0xf0)>>4));
  LEAD_II_H  = data[5];
  LEAD_II_L  = data[6];
#if 1  
  buffer = (LEAD_II_H<<8) + LEAD_II_L  +128 - 0x80;

  LEAD_II_L =buffer;
#endif 
#if 0    
 
  if(LEAD_II_H &0x80)   //����
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
  

  //����缫���䣬�����ݴ���Ϊ������ֵ������ADS1194�ɼ�3����ECG�źţ����5���缫��
  if((LOFF&0x22))
  {      
      LeadOff_Time =0;
      LEAD_II_L = 0x20;         //0x30Ϊ��Ϊ����Ĭ�ϻ���λ�ã�RL�缫���䣬״̬���ȶ������ݿ��ܲ���ÿ�ζ���Ϊ0x30��
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
  //��125����������1s��ʱ���ڲ���3���㣻
 
  if((revdatanum==30)||(revdatanum==70)||(revdatanum==110))
  {
    strrev.strsenddata.data[strrev.len++] = (lastdata + LEAD_II_L)/2; 
//    strrev.strsenddata.data[strrev.len++] = revdatanum;

    //�����ֵ������16 bit�ĳ��ȣ���ֱ�ӷ��ͳ�ȥ��
    if(strrev.len >= MAXNUM)
    {   
        //ÿ16 Bit�����ݷ���һ�ε����͵缫״̬��    
        strrev.strsenddata.state[0] = Batt_Measure();
        strrev.strsenddata.state[1] = LOFF;
        UpdataRev(&strrev.strsenddata);
        strrev.len = 0;
    }
  }
  
#endif
  //Simple Key Profile��ֻ�ܷ���16 Bit�����ݣ���16���ж������е�Lead I��ȡ����ϳ�һ�����Ͱ���
   strrev.strsenddata.data[strrev.len++] = LEAD_II_L;
//  strrev.strsenddata.data[strrev.len++] = revdatanum;
  if(strrev.len >= MAXNUM)
  {   
      //ÿ16 Bit�����ݷ���һ�ε����͵缫״̬��
      strrev.strsenddata.state[0] = Batt_Measure();
      strrev.strsenddata.state[1] = LOFF;
      UpdataRev(&strrev.strsenddata);
      strrev.len=0;
      RevdataInit();
  }

  lastdata = LEAD_II_L;                 //�����ϴε����ݣ��Թ���ƽ��ֵ��
}

//**************************************************************************************************/


