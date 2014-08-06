/******************************************* Copyright (c) **********************************************
**             Zhejiang Helowin Medical technology  Co.,LTD.
**                   http://www.hellowin.cn
**Filename:    ADS1194.c
**Revised:    $Date: 2012-10-18 $
**Revision:    $Revision: V1.0.0 $
**Description:  This file contains the ADS1194 application.
********************************************************************************************************/

/**************************************************************************************************
 *                      INCLUDES
 **************************************************************************************************/
#include "ads1194.h"
#include "BLE_SEND.h"




/**************************************************************************************************
 *                                       FUNCTIONS - API
 **************************************************************************************************/



/**************************************************************************************************
 *                                       LOCAL FUNCTIONS 
 **************************************************************************************************/
static void ADS1194_WriteReg(uint8, uint8);
static void ADS1194_WriteCommand(uint8);
static uint8 ADS1194_ReadReg(uint8);
static uint8 ADS1194_ReadSingleLeadData(uint8);
static void ADS1194_ChipEnable ( void );
static void ADS1194_ChipDisable ( void );
static void ADS1194_DataReadyPoll ( void );
static void ADS1194_Init_StartUp( void );
static void ADS1194_Init_TestSignal( void );
static void ADS1194_Init_ShortInput( void );
//static void ADS1194_Init_Normal( void );
static void ADS1194_Init_Normal( unsigned char Rate,unsigned char  gain);

/********************************************************************************************************
* Function Name : ADS1194_WriteReg
* Description :
* Input     :
* Output     : 
* Return     :
********************************************************************************************************/
static void ADS1194_WriteReg(uint8 address, uint8 data)
{
  HalEcg_CSEnable();                              // CS使能
  HalEcg_WriteByte(SDATAC);                       // 停止RDATAC模式
  HalEcg_WriteByte(WREG | address & 0x1F);        // 寄存器地址
  HalEcg_WriteByte(0x00);                         // 写1个字节
  HalEcg_WriteByte(data);                         // 写数据
  HalEcg_CSDisable();                             // 
}

/********************************************************************************************************
* Function Name : ADS1194_WriteCommand
* Description :
* Input     :
* Output     : 
* Return     :
********************************************************************************************************/
static void ADS1194_WriteCommand(uint8 command)
{
  HalEcg_CSEnable();                              // CS使能
  HalEcg_WriteByte(command);                      // 写数据
  //HalEcg_CSDisable();                           // 
}


/********************************************************************************************************
* Function Name : ADS1194_ReadReg
* Description :
* Input     :
* Output     : 
* Return     :
********************************************************************************************************/
static uint8 ADS1194_ReadReg(uint8 address)
{
  uint8 read;
  HalEcg_CSEnable();                             // CS使能
  HalEcg_WriteByte(SDATAC);                      // 停止RDATAC模式
  HalEcg_WriteByte(RREG | address & 0x1F);       // 寄存器地址
  HalEcg_WriteByte(0x00);                        // 读1个字节
  HalEcg_ReadByte(&read, 0xFF);                  // 读数据
  //HalEcg_CSDisable();                            //
  return read;
}

/********************************************************************************************************
* Function Name : ADS1194_ReadDataContinuous
* Description :
* Input     :
* Output     : 
* Return     :
********************************************************************************************************/

uint8 ADS1194_ReadDataContinuous(uint8 *ptr)
{
  ADS1194_ChipEnable();
  ADS1194_WriteCommand(SDATAC);
  ADS1194_WriteCommand(RDATAC);
  ADS1194_SoftStart();

  ADS1194_ChipDisable(); 
  return 1;    
}

/********************************************************************************************************
* Function Name : ADS1194_ReadSingleData
* Description :
* Input     :
* Output     : 
* Return     :
********************************************************************************************************/
void ADS1194_ReadSingleData(uint8 *pLoff, uint8 *pLeadI, uint8 *pLeadII, uint8 *pLeadC1)
{
  uint8 regdata[11];
  uint8 index;
  ADS1194_ChipEnable();
  ADS1194_WriteCommand(SDATAC);
  ADS1194_WriteCommand(RDATA);
  ADS1194_WriteCommand(START);
  ADS1194_DataReadyPoll();

  for(index=0; index<11; index++)                                 // 88Bit(24+4*16)连续11个Byte的数据
  {
    HalEcg_ReadByte(&regdata[index], 0xFF);
  }
  *pLoff       = ((regdata[1]&0xf0)|((regdata[2]&0xf0)>>4));
  *pLeadI      = regdata[4];                                      //16bit的数据只取低8位有效数据；
  *pLeadII     = regdata[6];                                      //16bit的数据只取低8位有效数据；
  *pLeadC1     = regdata[8];                                      //16bit的数据只取低8位有效数据；
}

/********************************************************************************************************
* Function Name : ADS1194_ReadSingleLeadData
* Description :
* Input     :
* Output     : 
* Return     :
********************************************************************************************************/
static uint8 ADS1194_ReadSingleLeadData(uint8 leadparam)
{
  uint8 regdata[11];
  uint8 index;
  uint8 leaddata;
  ADS1194_ChipEnable();
  ADS1194_WriteCommand(SDATAC);   //停止连续采集模式
  ADS1194_WriteCommand(START);
  ADS1194_WriteCommand(RDATA);
  ADS1194_DataReadyPoll();

  for(index=0; index<11; index++)                                 // 88Bit(24+4*16)连续11个Byte的数据
  {
    HalEcg_ReadByte(&regdata[index], 0xFF);
  }
  
  switch(leadparam)
  {
    case LOFF_DATA:
      leaddata = ((regdata[1]&0xf0)|((regdata[2]&0xf0)>>4));
      break;
      
    case LEADI_DATA:
      leaddata = regdata[4];           //16bit的数据只取低8位有效数据；
      break;  
      
    case LEADII_DATA:
      leaddata = regdata[6];           //16bit的数据只取低8位有效数据；
      break; 
      
    case LEADIII_DATA:
      leaddata = regdata[8];           //16bit的数据只取低8位有效数据；
      break; 

    default:  
      leaddata = 0xFF;                //默认数据为0xFF；
      break; 
  }
  
  return leaddata;
}


/********************************************************************************************************
* Function Name : ADS1194_ReadData
* Description :
* Input     :
* Output     : 
* Return     :
********************************************************************************************************/
void ADS1194_ReadData(uint8 *ptr)
{
  uint8 regdata[11];
  uint8 index;
  ADS1194_ChipEnable();
  ADS1194_WriteCommand(SDATAC);
  ADS1194_WriteCommand(START);
  ADS1194_WriteCommand(RDATA);
  ADS1194_DataReadyPoll();

  for(index=0; index<11; index++)             // 88Bit(24+4*16)连续11个Byte的数据
  {
    HalEcg_ReadByte(&regdata[index], 0xFF);
    *(ptr+index) = regdata[index];
  }
  //ADS1194_ChipDisable(); 
}

/********************************************************************************************************
* Function Name : ADS1194_WriteData
* Description :
* Input     :
* Output     : 
* Return     :
********************************************************************************************************/
void ADS1194_WriteData(uint8 address, uint8 data)
{
  
}

/**********************************************************************************************************
*ADS1194配置
**********************************************************************************************************/

//ADS1194唤醒
void ADS1194_WakeUp( void )
{
  ADS1194_WriteCommand(WAKEUP);
}

//ADS1194休眠
void ADS1194_Sleep( void )
{
  ADS1194_WriteCommand(STANDBY);
}

//ADS1194软件复位命令
void ADS1194_SoftReset( void )
{
  ADS1194_WriteCommand(RESET);
}

//ADS1194软件开始ADC转换
void ADS1194_SoftStart( void )
{ 
  ADS1194_WriteCommand(START);
}

//ADS1194软件停止ADC转换
void ADS1194_SoftStop( void )
{
  ADS1194_WriteCommand(STOP);
}

//ADS1194开始连续读数据模式
void ADS1194_StartReadDataContinuous( void )
{
  ADS1194_ChipEnable();
  ADS1194_WriteCommand(SDATAC);
  ADS1194_SoftStart();
  ADS1194_WriteCommand(RDATAC);
  // ADS1194_DataReadyPoll();     //通过中断读取ECG数据，不需要查询ECG_DRDY电平
}

//ADS1194停止连续读数据模式
void ADS1194_StopReadDataContinuous( void )
{
  ADS1194_WriteCommand(SDATAC);
  ADS1194_ChipDisable();
}

//读ADS1194设备ID
uint8 ADS1194_ID( void )
{   
  return(ADS1194_ReadReg(ID));
}

//写ADS1194 GPIO口
void ADS1194_WriteGPIO(uint8 num, uint8 val)
{
  uint8 i=0;

  i=ADS1194_ReadReg(GPIO);
  ADS1194_WriteReg(GPIO,i&(~(1<<num)));
  if ( val>0 ) 
  {
    val=i|(1<<(num+4));
  } 
  else 
  {
    val=i&(~(1<<(num+4)));
  }
  ADS1194_WriteReg(GPIO,val);
}

//ADS1194读GPIO寄存器值
uint8 ADS1194_ReadGPIO(uint8 num)
{
  uint8 i=0;

  i=ADS1194_ReadReg(GPIO);
  ADS1194_WriteReg(GPIO,i|(1<<num));
  i=ADS1194_ReadReg(GPIO);
  i&=(1<<num);
  i>>=num;
  return(i);
}

//


/**********************************************************************************************************
* Device SPI Chip Select Manipulation                                                                 *
**********************************************************************************************************/
static void ADS1194_ChipEnable ( void )    // ADS1194 module uses GPIO as the SPI CS
{
  HalEcg_CSEnable(); 
}

static void ADS1194_ChipDisable ( void )   // ADS1194 uses GPIO for SPI CS
{
  HalEcg_CSDisable(); 
}

/**********************************************************************************************************
* ADS1194 Functions                                                                        *
**********************************************************************************************************/
static void ADS1194_DataReadyPoll ( void )
{
  while(HalEcg_DrdyStatus());      // ADS1194等待DRDY信号变低，采集数据转换完成，可以读取数据
 // HalEcg_WaitMs(5);
}

/**********************************************************************************************************
                ADS1194 Functions                                                                        *
**********************************************************************************************************/
void ADS1194_PowerDown ( void )
{
  HalEcg_PowerDisable();
}

void ADS1194_PowerUp ( void )
{   
  HalEcg_PowerEnable();
}

/**********************************************************************************************************
                ADS1194 Initialization
**********************************************************************************************************/
// ADS1194初始化
void ADS1194_Init( unsigned char RateMode,unsigned  char  gain)
{
  HalEcgInit();
  ADS1194_Init_StartUp();
//  ADS1194_Init_Normal(0,0x80);
  Reset_ADS1194_Mode(RateMode,gain);
  
//  ADS1194_Init_Normal();
 // ADS1194_Init_TestSignal();
//  ADS1194_Init_ShortInput();

}

// ADS1194初始化启动
static void ADS1194_Init_StartUp( void )
{  
  ADS1194_PowerUp();                  // 上电启动：打开AVDD电源
  ADS1194_ChipEnable();               // CS使能
  ADS1194_SoftReset();                // 软件复位
}

// ADS1194默认初始化配置:2Hz方波测试信号
static void ADS1194_Init_TestSignal( void )
{
  ADS1194_ChipEnable();             // CS使能
  ADS1194_WriteReg(CONFIG2,0x30);   // CONFIG2
  ADS1194_WriteReg(CH1SET,0x05);    // CH1SET
  ADS1194_WriteReg(CH2SET,0x05);    // CH2SET
  ADS1194_WriteReg(CH3SET,0x05);    // CH3SET
  ADS1194_ChipDisable(); 
}

// ADS1194默认初始化配置:输入通道短接
static void ADS1194_Init_ShortInput( void )
{
  ADS1194_ChipEnable();             // CS使能
  ADS1194_WriteReg(CH1SET,0x01);    // CH1SET
  ADS1194_WriteReg(CH2SET,0x01);    // CH2SET
  ADS1194_WriteReg(CH3SET,0x01);    // CH3SET
  ADS1194_ChipDisable(); 
}


// ADS1194默认初始化配置:正常输入模式
static void ADS1194_Init_Normal( unsigned char Rate,unsigned char  gain)
{
  HalEcg_CSEnable();                // CS使能
  HalEcg_WriteByte(SDATAC);         // 停止RDATAC模式
  HalEcg_WriteByte(WREG | CONFIG1); // 起始寄存器地址：CONFIG1
  HalEcg_WriteByte(0x10);           // 连续写17个寄存器
  HalEcg_WriteByte(0x40|Rate);           // CONFIG1  
  
  HalEcg_WriteByte(0x30);           // CONFIG2      
  HalEcg_WriteByte(0xCC);           // CONFIG3      
  HalEcg_WriteByte(0xE3);           // LOFF//  
//  HalEcg_WriteByte(0x81);           // CH1SET      
//  HalEcg_WriteByte(0x81);           // CH2SET      
//  HalEcg_WriteByte(0x81);           // CH3SET    
  
  HalEcg_WriteByte(gain);           // CH1SET      
  HalEcg_WriteByte(gain);           // CH2SET      
  HalEcg_WriteByte(gain);           // CH3SET      
  HalEcg_WriteByte(0x81);           // CH4SET      
  HalEcg_WriteByte(0x81);           // CH5SET      
  HalEcg_WriteByte(0x81);           // CH6SET      
  HalEcg_WriteByte(0x81);           // CH7SET      
  HalEcg_WriteByte(0x81);           // CH8SET      
  HalEcg_WriteByte(0x03);           // RLD_SENSP    
  HalEcg_WriteByte(0x03);           // RLD_SENSN    
  HalEcg_WriteByte(0x07);           // LOFF_SENSP  
  HalEcg_WriteByte(0x07);           // LOFF_SENSN  
  HalEcg_WriteByte(0x00);           // LOFF_FLIP    

  HalEcg_CSDisable(); 
  HalEcg_WaitUs(5);
  HalEcg_CSEnable(); 
  HalEcg_WriteByte(WREG | GPIO);    // 起始寄存器地址:GPIO
  HalEcg_WriteByte(0x05);           // 连续写6个寄存器
  HalEcg_WriteByte(0x0F);           // GPIO        
  HalEcg_WriteByte(0x00);           // PACE        
  HalEcg_WriteByte(0x00);           // RESERVED        
  HalEcg_WriteByte(0x02);           // CONFIG4      
  HalEcg_WriteByte(0x09);           // WCT1        
  HalEcg_WriteByte(0xC2);           // WCT2
  HalEcg_CSDisable();  
}



void  Reset_ADS1194_Mode(unsigned char RateMode,unsigned  char  gain)
{
  ADS1194_StopReadDataContinuous();
  ADS1194_SoftReset();                // 软件复位
  
  if(gain == 5)
  {
    gain = 0x60;
  }
  else if(gain == 0x01)
  {
    gain = 0x50;
  }
  else if(gain ==  0x02)
  {
    gain = 0x0;
  }
  else if(gain == 0x04)
  {
    gain = 0x40;
  }
  
  if(RateMode == 1)
  {
  ADS1194_Init_Normal(0x06,gain);
  }
  else if (RateMode == 2)
  {
      ADS1194_Init_Normal(0x05,gain);
  }
  else if (RateMode == 0x04)
  {
      ADS1194_Init_Normal(0x04,gain);
  }
  else if (RateMode == 3)
  {
      ADS1194_Init_Normal(0x03,gain);
  }
  else if (RateMode == 5)
  {
      ADS1194_Init_Normal(0x02,gain);
  }
  ADS1194_StartReadDataContinuous();
}
/**********************************************************************************************************
                ADS1194 Calibration
**********************************************************************************************************/
void ADS1194_Calibration(uint8* caldatum)
{ 
//  uint8 cal_0mv[11],cal_1mv[11];
  uint8 LeadII[100]={0}; 
  uint16 LeadData = 0;
  uint8 i=0;
  //0mV校准
  ADS1194_Init_ShortInput();
  HalEcg_WaitMs(5);
  for(i=0; i<100; i++);
  {
    LeadII[i] = ADS1194_ReadSingleLeadData(LEADII_DATA);
  }

  for(i=0; i<100; i++);
  {
    LeadData += LeadII[i];
  }
  *caldatum = LeadData/100;

 // ADS1194_ReadData(cal_0mv);
//  *caldatum = cal_0mv[4];             //
  
  //1mV校准
 // ADS1194_Init_TestSignal();
 // ADS1194_ReadData(LEADII_DATA);
//  *caloffset = cal_1mv[4];            //
}

/*****************************************  http://www.hellowin.cn  ************************************/
