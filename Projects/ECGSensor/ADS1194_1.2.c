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
#include  "ads1194.h"
#include "BLE_SEND.h"

/**************************************************************************************************
 *                                       FUNCTIONS - API
 **************************************************************************************************/
void ADS1194_WriteReg(uint8 , uint8 );
uint8 ADS1194_ReadReg(uint8 );
uint8 ADS1194_ReadReg(uint8 );
uint8 ADS1194_ReadDataContinuous(uint8 *);
void ADS1194_WriteData(uint8 , uint8 );
void ADS1194_WriteCommand(uint8 );
void ADS1194_Conversion_Enable( void );
void ADS1194_WakeUp( void );
void ADS1194_Sleep( void );
void ADS1194_SoftReset( void );
void ADS1194_SoftStart( void );
void ADS1194_StartReadDataContinuous( void );
void ADS1194_StopReadDataContinuous( void );
uint8 ADS1194_ID( void );
void ADS1194_WriteGPIO(uint8 , uint8 );
uint8 ADS1194_ReadGPIO(uint8 );
void ADS1194_ChipEnable ( void );
void ADS1194_ChipDisable ( void );
void ADS1194_DataReady ( void );
void ADS1194_PowerDown ( void );
void ADS1194_PowerUp ( void );
void ADS1194_Init( void );
void ADS1194_Init_StartUp( void );
void ADS1194_Init_TestSignal( void );
void ADS1194_Init_Default( void );



//extern void Rev_data_analy(uint8* data,uint8 len);
/********************************************************************************************************
* Function Name : ADS1194_WriteReg
* Description :
* Input     :
* Output     : 
* Return     :
********************************************************************************************************/
void ADS1194_WriteReg(uint8 address, uint8 value)
{
  HalEcg_CSEnable();                              // CS使能
  HalEcg_WriteByte(SDATAC);                       // 停止RDATAC模式
  HalEcg_WriteByte(WREG | address & 0x1F);        // 寄存器地址
  HalEcg_WriteByte(0x00);                         // 写1个字节
  HalEcg_WriteByte(value);                        // 写数据
  HalEcg_CSDisable();                             // 
}

/********************************************************************************************************
* Function Name : ADS1194_WriteCommand
* Description :
* Input     :
* Output     : 
* Return     :
********************************************************************************************************/
void ADS1194_WriteCommand(uint8 command)
{
  HalEcg_CSEnable();                              // CS使能
  HalEcg_WriteByte(command);                      // 写数据
  HalEcg_CSDisable();                             // 
}


/********************************************************************************************************
* Function Name : ADS1194_ReadReg
* Description :
* Input     :
* Output     : 
* Return     :
********************************************************************************************************/
uint8 ADS1194_ReadReg(uint8 address)
{
  uint8 read;
  HalEcg_CSEnable();                             // CS使能
  HalEcg_WriteByte(SDATAC);                      // 停止RDATAC模式
  HalEcg_WriteByte(RREG | address & 0x1F);       // 寄存器地址
  HalEcg_WriteByte(0x00);                        // 读1个字节
  HalEcg_ReadByte(&read, 0xFF);                  // 读数据
  HalEcg_CSDisable();                            //
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
  // uint8 ch =osal_rand()%14+'a';
  // ADS1194_StartReadDataContinuous();       // 发送RDATAC命令，开始连续读数据

  ADS1194_WriteCommand(SDATAC);
  ADS1194_WriteCommand(RDATAC);
  ADS1194_SoftStart();
  ADS1194_ChipEnable();
  //ADS1194_DataReady(); 

  ADS1194_ChipDisable(); 
  // ADS1194_StopReadDataContinuous();        // 发送SDATAC命令，停止连续读寄存器；
  return 1;    
}

/********************************************************************************************************
* Function Name : ADS1194_ReadSingleData
* Description :
* Input     :
* Output     : 
* Return     :
********************************************************************************************************/

/*

uint8 ADS1194_ReadSingleData(uint8 *pLoff, uint8 *pLeadI, uint8 *pLeadII, uint8 *pLeadC1)
{
    uint8 LOFF_STATP, LOFF_STATN, LEAD_I, LEAD_II, LEAD_C1;
    

    LOFF_STATP  = (((data[0]&0x0f)<<4)|((data[1]&0xf0)>>4));
    LOFF_STATN  = (((data[1]&0x0f)<<4)|((data[2]&0xf0)>>4));
    *pLeadI     = data[4];                                      //16bit的数据只取低8位有效数据；
    *pLeadII    = data[6];                                      //16bit的数据只取低8位有效数据；
    *LEAD_C1    = data[8];                                      //16bit的数据只取低8位有效数据；   
}
*/


/********************************************************************************************************
* Function Name : ADS1194_ReadData
* Description :
* Input     :
* Output     : 
* Return     :
********************************************************************************************************/
void ADS1194_ReadData(uint8 *ptr)
{
  uint8 addr[11];
  uint8 index;
  // uint8 ch =osal_rand()%14+'a';
  // ADS1194_StartReadDataContinuous();       // 发送RDATAC命令，开始连续读数据
  
  ADS1194_WriteCommand(SDATAC);
  ADS1194_WriteCommand(RDATA);
  ADS1194_SoftStart();
  ADS1194_DataReady();
  ADS1194_ChipEnable(); 
  for(index=0; index<11; index++)             // 88Bit(24+4*16)连续11个Byte的数据
  {
    HalEcg_ReadByte(&addr[index], 0xFF);
    *(ptr+index) = addr[index];
  }
  ADS1194_ChipDisable(); 
  
  // ADS1194_StopReadDataContinuous();        // 发送SDATAC命令，停止连续读寄存器；
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
void ADS1194_Conversion_Enable( void )
{ 
  
}

//ADS1194??D?
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
  ADS1194_WriteCommand(SDATAC);
  ADS1194_SoftStart();
  ADS1194_WriteCommand(RDATAC);
  // ADS1194_DataReady();     //通过中断读取ECG数据，不需要查询ECG_DRDY电平
  ADS1194_ChipEnable();
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
void ADS1194_ChipEnable ( void )    // ADS1194 module uses GPIO as the SPI CS
{
  HalEcg_CSEnable(); 
}

void ADS1194_ChipDisable ( void )   // ADS1194 uses GPIO for SPI CS
{
  HalEcg_CSDisable(); 
}

/**********************************************************************************************************
* ADS1194 Functions                                                                        *
**********************************************************************************************************/
void ADS1194_DataReady ( void )
{
  while(HalEcg_DrdyStatus());      // ADS1194等待DRDY信号变低，采集数据转换完成，可以读取数据
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
void ADS1194_Init( void )
{
  HalEcgInit();
  ADS1194_Init_StartUp();
  ADS1194_Init_Default();
  //ADS1194_Init_TestSignal();
  ADS1194_StartReadDataContinuous();
}

// ADS1194初始化启动
void ADS1194_Init_StartUp( void )
{  
  ADS1194_PowerUp();                  // 上电启动：打开AVDD电源
  ADS1194_ChipEnable();               // CS使能
  ADS1194_SoftReset();                // 软件复位
//  ADS1194_WriteReg(CONFIG3, 0x80);  // 
//  ADS1194_WriteReg(CONFIG1, 0x06);  //
//  ADS1194_WriteReg(CONFIG2, 0x00);  //
//  ADS1194_WriteReg(CH1SET,  0x01);  // 
//  ADS1194_WriteReg(CH2SET,  0x01);  // 
//  ADS1194_WriteReg(CH3SET,  0x01);  // 
//  ADS1194_SoftStart();
}

// ADS1194默认初始化配置:2Hz方波测试信号
void ADS1194_Init_TestSignal( void )
{
  HalEcg_CSEnable();                // CS使能
  HalEcg_WriteByte(SDATAC);         // 停止RDATAC模式
  HalEcg_WriteByte(WREG | CONFIG1); // 起始寄存器地址：CONFIG1
  HalEcg_WriteByte(0x10);           // 连续写17个寄存器
  HalEcg_WriteByte(0x66);           // CONFIG1   Bit 5 CLK_EN:1;  Bits[2:0]  DR[2:0]=110 
  HalEcg_WriteByte(0x30);           // CONFIG2   Bit 4 INT_TEST:1 = Test signals are generated internally  
  HalEcg_WriteByte(0xCC);           // CONFIG3      
  HalEcg_WriteByte(0xE3);           // LOFF        
  HalEcg_WriteByte(0x05);           // CH1SET    Bits[2:0] MUXn[2:0]:101 = Test signal  
  HalEcg_WriteByte(0x05);           // CH2SET    Bits[2:0] MUXn[2:0]:101 = Test signal  
  HalEcg_WriteByte(0x05);           // CH3SET    Bits[2:0] MUXn[2:0]:101 = Test signal   
  HalEcg_WriteByte(0x81);           // CH4SET      
  HalEcg_WriteByte(0x81);           // CH5SET      
  HalEcg_WriteByte(0x81);           // CH6SET      
  HalEcg_WriteByte(0x81);           // CH7SET      
  HalEcg_WriteByte(0x81);           // CH8SET      
  HalEcg_WriteByte(0x01);           // RLD_SENSP    
  HalEcg_WriteByte(0x01);           // RLD_SENSN    
  HalEcg_WriteByte(0x07);           // LOFF_SENSP  
  HalEcg_WriteByte(0x07);           // LOFF_SENSN  
  HalEcg_WriteByte(0x00);           // LOFF_FLIP    

  HalEcg_CSDisable();               // 两个寄存器地址不连续，需要重新设置起始寄存器地址
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

// ADS1194默认初始化配置:正常输入模式
void ADS1194_Init_Default( void )
{
  HalEcg_CSEnable();                // CS使能
  HalEcg_WriteByte(SDATAC);         // 停止RDATAC模式
  HalEcg_WriteByte(WREG | CONFIG1); // 起始寄存器地址：CONFIG1
  HalEcg_WriteByte(0x10);           // 连续写17个寄存器
  HalEcg_WriteByte(0x66);           // CONFIG1   Bit 5 CLK_EN:1;  Bits[2:0]  DR[2:0]=110     
  HalEcg_WriteByte(0x20);           // CONFIG2   Bit 4 INT_TEST:0 = Test signals are driven externally (default)   
  HalEcg_WriteByte(0xCC);           // CONFIG3      
  HalEcg_WriteByte(0xE3);           // LOFF        
  HalEcg_WriteByte(0x00);           // CH1SET    Bits[2:0] MUXn[2:0]:000 = Normal electrode input    
  HalEcg_WriteByte(0x00);           // CH2SET    Bits[2:0] MUXn[2:0]:000 = Normal electrode input   
  HalEcg_WriteByte(0x00);           // CH3SET    Bits[2:0] MUXn[2:0]:000 = Normal electrode input   
  HalEcg_WriteByte(0x81);           // CH4SET      
  HalEcg_WriteByte(0x81);           // CH5SET      
  HalEcg_WriteByte(0x81);           // CH6SET      
  HalEcg_WriteByte(0x81);           // CH7SET      
  HalEcg_WriteByte(0x81);           // CH8SET      
  HalEcg_WriteByte(0x01);           // RLD_SENSP    
  HalEcg_WriteByte(0x01);           // RLD_SENSN    
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


/*****************************************  http://www.hellowin.cn  ************************************/
