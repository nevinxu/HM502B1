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
  HalEcg_CSEnable();                              // CSʹ��
  HalEcg_WriteByte(SDATAC);                       // ֹͣRDATACģʽ
  HalEcg_WriteByte(WREG | address & 0x1F);        // �Ĵ�����ַ
  HalEcg_WriteByte(0x00);                         // д1���ֽ�
  HalEcg_WriteByte(value);                        // д����
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
  HalEcg_CSEnable();                              // CSʹ��
  HalEcg_WriteByte(command);                      // д����
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
  HalEcg_CSEnable();                             // CSʹ��
  HalEcg_WriteByte(SDATAC);                      // ֹͣRDATACģʽ
  HalEcg_WriteByte(RREG | address & 0x1F);       // �Ĵ�����ַ
  HalEcg_WriteByte(0x00);                        // ��1���ֽ�
  HalEcg_ReadByte(&read, 0xFF);                  // ������
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
  // ADS1194_StartReadDataContinuous();       // ����RDATAC�����ʼ����������

  ADS1194_WriteCommand(SDATAC);
  ADS1194_WriteCommand(RDATAC);
  ADS1194_SoftStart();
  ADS1194_ChipEnable();
  //ADS1194_DataReady(); 

  ADS1194_ChipDisable(); 
  // ADS1194_StopReadDataContinuous();        // ����SDATAC���ֹͣ�������Ĵ�����
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
    *pLeadI     = data[4];                                      //16bit������ֻȡ��8λ��Ч���ݣ�
    *pLeadII    = data[6];                                      //16bit������ֻȡ��8λ��Ч���ݣ�
    *LEAD_C1    = data[8];                                      //16bit������ֻȡ��8λ��Ч���ݣ�   
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
  // ADS1194_StartReadDataContinuous();       // ����RDATAC�����ʼ����������
  
  ADS1194_WriteCommand(SDATAC);
  ADS1194_WriteCommand(RDATA);
  ADS1194_SoftStart();
  ADS1194_DataReady();
  ADS1194_ChipEnable(); 
  for(index=0; index<11; index++)             // 88Bit(24+4*16)����11��Byte������
  {
    HalEcg_ReadByte(&addr[index], 0xFF);
    *(ptr+index) = addr[index];
  }
  ADS1194_ChipDisable(); 
  
  // ADS1194_StopReadDataContinuous();        // ����SDATAC���ֹͣ�������Ĵ�����
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
*ADS1194����
**********************************************************************************************************/
void ADS1194_Conversion_Enable( void )
{ 
  
}

//ADS1194??D?
void ADS1194_WakeUp( void )
{
  ADS1194_WriteCommand(WAKEUP);
}

//ADS1194����
void ADS1194_Sleep( void )
{
  ADS1194_WriteCommand(STANDBY);
}

//ADS1194�����λ����
void ADS1194_SoftReset( void )
{
  ADS1194_WriteCommand(RESET);
}

//ADS1194�����ʼADCת��
void ADS1194_SoftStart( void )
{ 
  ADS1194_WriteCommand(START);
}

//ADS1194���ֹͣADCת��
void ADS1194_SoftStop( void )
{
  ADS1194_WriteCommand(STOP);
}

//ADS1194��ʼ����������ģʽ
void ADS1194_StartReadDataContinuous( void )
{
  ADS1194_WriteCommand(SDATAC);
  ADS1194_SoftStart();
  ADS1194_WriteCommand(RDATAC);
  // ADS1194_DataReady();     //ͨ���ж϶�ȡECG���ݣ�����Ҫ��ѯECG_DRDY��ƽ
  ADS1194_ChipEnable();
}

//ADS1194ֹͣ����������ģʽ
void ADS1194_StopReadDataContinuous( void )
{
  ADS1194_WriteCommand(SDATAC);
  ADS1194_ChipDisable();
}

//��ADS1194�豸ID
uint8 ADS1194_ID( void )
{   
  return(ADS1194_ReadReg(ID));
}

//дADS1194 GPIO��
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

//ADS1194��GPIO�Ĵ���ֵ
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
  while(HalEcg_DrdyStatus());      // ADS1194�ȴ�DRDY�źű�ͣ��ɼ�����ת����ɣ����Զ�ȡ����
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
// ADS1194��ʼ��
void ADS1194_Init( void )
{
  HalEcgInit();
  ADS1194_Init_StartUp();
  ADS1194_Init_Default();
  //ADS1194_Init_TestSignal();
  ADS1194_StartReadDataContinuous();
}

// ADS1194��ʼ������
void ADS1194_Init_StartUp( void )
{  
  ADS1194_PowerUp();                  // �ϵ���������AVDD��Դ
  ADS1194_ChipEnable();               // CSʹ��
  ADS1194_SoftReset();                // �����λ
//  ADS1194_WriteReg(CONFIG3, 0x80);  // 
//  ADS1194_WriteReg(CONFIG1, 0x06);  //
//  ADS1194_WriteReg(CONFIG2, 0x00);  //
//  ADS1194_WriteReg(CH1SET,  0x01);  // 
//  ADS1194_WriteReg(CH2SET,  0x01);  // 
//  ADS1194_WriteReg(CH3SET,  0x01);  // 
//  ADS1194_SoftStart();
}

// ADS1194Ĭ�ϳ�ʼ������:2Hz���������ź�
void ADS1194_Init_TestSignal( void )
{
  HalEcg_CSEnable();                // CSʹ��
  HalEcg_WriteByte(SDATAC);         // ֹͣRDATACģʽ
  HalEcg_WriteByte(WREG | CONFIG1); // ��ʼ�Ĵ�����ַ��CONFIG1
  HalEcg_WriteByte(0x10);           // ����д17���Ĵ���
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

  HalEcg_CSDisable();               // �����Ĵ�����ַ����������Ҫ����������ʼ�Ĵ�����ַ
  HalEcg_WaitUs(5);
  HalEcg_CSEnable(); 
  HalEcg_WriteByte(WREG | GPIO);    // ��ʼ�Ĵ�����ַ:GPIO
  HalEcg_WriteByte(0x05);           // ����д6���Ĵ���
  HalEcg_WriteByte(0x0F);           // GPIO        
  HalEcg_WriteByte(0x00);           // PACE        
  HalEcg_WriteByte(0x00);           // RESERVED        
  HalEcg_WriteByte(0x02);           // CONFIG4      
  HalEcg_WriteByte(0x09);           // WCT1        
  HalEcg_WriteByte(0xC2);           // WCT2
  HalEcg_CSDisable();  
}

// ADS1194Ĭ�ϳ�ʼ������:��������ģʽ
void ADS1194_Init_Default( void )
{
  HalEcg_CSEnable();                // CSʹ��
  HalEcg_WriteByte(SDATAC);         // ֹͣRDATACģʽ
  HalEcg_WriteByte(WREG | CONFIG1); // ��ʼ�Ĵ�����ַ��CONFIG1
  HalEcg_WriteByte(0x10);           // ����д17���Ĵ���
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
  HalEcg_WriteByte(WREG | GPIO);    // ��ʼ�Ĵ�����ַ:GPIO
  HalEcg_WriteByte(0x05);           // ����д6���Ĵ���
  HalEcg_WriteByte(0x0F);           // GPIO        
  HalEcg_WriteByte(0x00);           // PACE        
  HalEcg_WriteByte(0x00);           // RESERVED        
  HalEcg_WriteByte(0x02);           // CONFIG4      
  HalEcg_WriteByte(0x09);           // WCT1        
  HalEcg_WriteByte(0xC2);           // WCT2
  HalEcg_CSDisable();  
}


/*****************************************  http://www.hellowin.cn  ************************************/
