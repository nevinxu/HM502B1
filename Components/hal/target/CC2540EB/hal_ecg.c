#include <ioCC2540.h>
#include "hal_types.h"
#include "hal_defs.h"
#include <string.h>

#include "hal_ecg.h"
#include "OnBoard.h"
#include "hal_drivers.h"
#include "hal_assert.h"

#if (defined HAL_ECG) && (HAL_ECG == TRUE)

/**************************************************************************************************
 *                                          CONSTANTS
 **************************************************************************************************/
/* ECG lines */           
#define HAL_ECG_MISO_PORT               0       // P0_2 - ECG_ECG_MISO
#define HAL_ECG_MISO_PIN                2   

#define HAL_ECG_MOSI_PORT               0       // P0_3 - ECG_ECG_MOSI
#define HAL_ECG_MOSI_PIN                3   

#define HAL_ECG_CS_PORT                 0       // P0_4 - ECG_ECG_CS
#define HAL_ECG_CS_PIN                  4   

#define HAL_ECG_CLK_PORT                0       // P0_5 - ECG_ECG_CLK
#define HAL_ECG_CLK_PIN                 5       

/* ECG control lines */
//#define HAL_ECG_DRDY_PORT               0       // P0_6 - ECG_DRDY
//#define HAL_ECG_DRDY_PIN                6

#define HAL_ECG_POWER_PORT              1       // P1_5 - AVDD_PWREN
#define HAL_ECG_POWER_PIN               5

/* SPI settings */
#define HAL_ECG_CLOCK_POL_HI            0x80    // CPOL = 1
#define HAL_ECG_CLOCK_POL_LO            0x00
#define HAL_ECG_CLOCK_PHA_1             0x40    // CPHA = 1
#define HAL_ECG_CLOCK_PHA_0             0x00
#define HAL_ECG_TRANSFER_MSB_FIRST      0x20    
#define HAL_ECG_TRANSFER_LSB_FIRST      0x00    // ORDER = LSB First

/* ECG_DRDY at P0.6 */
#define HAL_ECG_DRDY_PORT               P0
#define HAL_ECG_DRDY_BIT                BV(6)
#define HAL_ECG_DRDY_SEL                P0SEL
#define HAL_ECG_DRDY_DIR                P0DIR

/* ECG_DRDY interrupts */
#define HAL_ECG_DRDY_IEN                IEN1    /* CPU interrupt mask register */
#define HAL_ECG_DRDY_ICTL               P0IEN   /* Port Interrupt Control register */
#define HAL_ECG_DRDY_ICTLBIT            BV(6)   /* P0IEN - P0.6 enable/disable bit */
#define HAL_ECG_DRDY_IENBIT             BV(5)   /* Mask bit for all of Port_0 */
#define HAL_ECG_DRDY_PXIFG              P0IFG   /* Interrupt flag at source */

#define HAL_ECG_DRDY_PXINP              P0INP   /* P0 Pullup/Pulldown */
#define HAL_ECG_DRDY_PXINP_PDUP         P2INP   /* Pullup */

#define HAL_ECG_DRDY_RISING_EDGE        0       // ECG_DRDY上升沿触发
#define HAL_ECG_DRDY_FALLING_EDGE       1       // ECG_DRDY下降沿触发

#define HAL_ECG_DRDY_EDGE               BV(0)   // ECG_DRDY下降沿触发

/**************************************************************************************************
 *                                           MACROS
 **************************************************************************************************/

#define HAL_IO_SET(port, pin, val)                HAL_IO_SET_PREP(port, pin, val)
#define HAL_IO_SET_PREP(port, pin, val)           st( P##port##_##pin## = val;)

#define HAL_CONFIG_IO_OUTPUT(port, pin, val)      HAL_CONFIG_IO_OUTPUT_PREP(port, pin, val)
#define HAL_CONFIG_IO_OUTPUT_PREP(port, pin, val) st( P##port##SEL &= ~BV(pin);\
                                                      P##port##DIR |= BV(pin);\
                                                      P##port##_##pin## = val;)

#define HAL_CONFIG_IO_INPUT(port, pin, val)       HAL_CONFIG_IO_INPUT_PREP(port, pin, val)
#define HAL_CONFIG_IO_INPUT_PREP(port, pin, val)  st( P##port##SEL &= ~BV(pin);\
                                                      P##port##DIR &= ~BV(pin);\
                                                      P##port##_##pin## = val;)

#define HAL_CONFIG_IO_PERIPHERAL(port, pin)      HAL_CONFIG_IO_PERIPHERAL_PREP(port, pin)
#define HAL_CONFIG_IO_PERIPHERAL_PREP(port, pin) st( P##port##SEL |= BV(pin);)

/**************************************************************************************************
 *                                       FUNCTIONS - API
 **************************************************************************************************/
#if (HAL_ECG == TRUE)

#endif // #if (HAL_ECG == TRUE)

/***************************************************************************************************
 * @fn      HalEcgInit
 *
 * @brief
 * 
 * @param 
 *  
 * @return  None
 ***************************************************************************************************/
void HalEcgInit(void)
{
  #if (HAL_ECG == TRUE)
    HalEcg_ConfigIO();
    HalEcg_ConfigSPI();
  #endif
}

/**************************************************************************************************
 *                                    HARDWARE ECG
 **************************************************************************************************/

/**************************************************************************************************
 * @fn      HalEcg_ConfigIO
 *
 * @brief   Configure IO lines needed for ECG control.
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalEcg_ConfigIO(void)
{
  /* ECG_POWER 配置为输出，默认为低电平 */
  HAL_CONFIG_IO_OUTPUT(HAL_ECG_POWER_PORT,  HAL_ECG_POWER_PIN,  0);
  
  /* ECG_CS 配置为输出，默认为高电平 */
  HAL_CONFIG_IO_OUTPUT(HAL_ECG_CS_PORT,  HAL_ECG_CS_PIN,  1);
  
  /* ECG_DRDY 初始化为GPIO输入 */
  // HAL_CONFIG_IO_INPUT(HAL_ECG_DRDY_PORT,  HAL_ECG_DRDY_PIN,  1);
  HAL_ECG_DRDY_SEL &= ~(HAL_ECG_DRDY_BIT);     /* Set pin function to GPIO */
  HAL_ECG_DRDY_DIR &= ~(HAL_ECG_DRDY_BIT);     /* Set pin direction to Input */
  
  HAL_ECG_DRDY_PXINP &= ~(HAL_ECG_DRDY_BIT);   /* P0.6 Pullup/Pulldown输入模式 */
  HAL_ECG_DRDY_PXINP_PDUP &= ~BV(5);           /* P0上拉输入模式 */
  
  HalEcg_InterruptEnable();                    /* 使能ECG DRDY引脚中断 */
}

/**************************************************************************************************
 * @fn      HalEcg_ConfigSPI
 *
 * @brief   Configure SPI lines needed for talking to ECG.
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalEcg_ConfigSPI(void)
{
  /* UART/ECG Peripheral configuration */
  uint8 baud_exponent;
  uint8 baud_mantissa;

  /* Set ECG on UART 0 alternative 1 */
  PERCFG |= 0x00;

  /* Configure clk, master out and master in lines */
  HAL_CONFIG_IO_PERIPHERAL(HAL_ECG_CLK_PORT,  HAL_ECG_CLK_PIN);
  HAL_CONFIG_IO_PERIPHERAL(HAL_ECG_MOSI_PORT, HAL_ECG_MOSI_PIN);
  HAL_CONFIG_IO_PERIPHERAL(HAL_ECG_MISO_PORT, HAL_ECG_MISO_PIN);

  /* Set ECG speed to 1 MHz (the values assume system clk of 32MHz)
   * Confirm on board that this results in 1MHz ECG clk. */
  baud_exponent = 15;
  baud_mantissa =  0;

  /* Configure ECG */
  U0UCR  = 0x80;          /* Flush and goto IDLE state. 8-N-1. */
  U0CSR  = 0x00;          /* SPI mode, master. */
  U0GCR  = HAL_ECG_CLOCK_POL_LO | HAL_ECG_CLOCK_PHA_1 | HAL_ECG_TRANSFER_MSB_FIRST | baud_exponent;
  U0BAUD = baud_mantissa;
}


/**************************************************************************************************
 * @fn      HalEcg_DrdyStatus
 *
 * @brief   
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
uint8 HalEcg_DrdyStatus(void)
{
  return((HAL_ECG_DRDY_PORT & HAL_ECG_DRDY_BIT));
}

/**************************************************************************************************
 * @fn      HalEcg_PowerEnable
 *
 * @brief   
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalEcg_PowerEnable(void)
{
  /* ECG_POWER 配置为输出，默认为高电平 */
  HAL_CONFIG_IO_OUTPUT(HAL_ECG_POWER_PORT,  HAL_ECG_POWER_PIN,  1);
  HalEcg_WaitMs(50);          // 延时50ms(>32.01ms+40us+10ms)
                              // 等待LDO上电稳定，ADS1194上电复位和振荡器启动时间
                              // 32.01ms:ADS1194 tPOR+tRST
                              // 40us:LDO电压开启稳定时间
                              // 10ms:2.048MHz有源晶振Start-up Time。
}

/**************************************************************************************************
 * @fn      HalEcg_PowerDisable
 *
 * @brief   
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalEcg_PowerDisable(void)
{
  /* ECG_POWER 配置为输出，默认为低电平 */
  HAL_CONFIG_IO_OUTPUT(HAL_ECG_POWER_PORT,  HAL_ECG_POWER_PIN,  0);
  HalEcg_WaitMs(1);         // 等待LDO关断
}

/**************************************************************************************************
 * @fn      HalEcg_InterruptEnable
 *
 * @brief   
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalEcg_InterruptEnable(void)
{
  /* ECG_DRDY Interrupt configuration */
  // PICTL &= ~HAL_ECG_DRDY_EDGE;                   /* Clear the edge bit */
  PICTL |= HAL_ECG_DRDY_EDGE;                       /* Set the edge bit to set falling edge to give int */
  //HAL_ECG_DRDY_PXIFG  = 0;                        /* P0IFG全部清0 */
  HAL_ECG_DRDY_ICTL   |= HAL_ECG_DRDY_ICTLBIT;      /* enable interrupt generation at port */
  HAL_ECG_DRDY_PXIFG  &= ~(HAL_ECG_DRDY_BIT);       /* Clear any pending interrupt */
  HAL_ECG_DRDY_IEN    |= HAL_ECG_DRDY_IENBIT;       /* enable CPU interrupt */
}

/**************************************************************************************************
 * @fn      HalEcg_CSEnable
 *
 * @brief   
 *
 * @param   
 *
 * @return  
 **************************************************************************************************/
void HalEcg_CSEnable(void)
{
  HAL_CONFIG_IO_OUTPUT(HAL_ECG_CS_PORT,  HAL_ECG_CS_PIN,  0);
  HalEcg_WaitUs(5);
}

/**************************************************************************************************
 * @fn      HalEcg_CSDisable
 *
 * @brief   
 *
 * @param   
 *
 * @return  
 **************************************************************************************************/
void HalEcg_CSDisable(void)
{
  HalEcg_WaitUs(5);
  HAL_CONFIG_IO_OUTPUT(HAL_ECG_CS_PORT,  HAL_ECG_CS_PIN,  1);
  HalEcg_WaitUs(5);               // 为下次CS使能预留一点时间
}

/**************************************************************************************************
 * @fn      HalEcg_WriteByte
 *
 * @brief   
 *
 * @param   
 *
 * @return  
 **************************************************************************************************/
void HalEcg_WriteByte(uint8 write)
{
  U0CSR &= ~ BV(1);               // 清TX_BYTE位
  U0DBUF = write;
  while (!(U0CSR & BV(1)));       // 等待TX_BYTE位置位
}

/**************************************************************************************************
 * @fn      HalEcg_ReadByte
 *
 * @brief   
 *
 * @param   
 *
 * @return  
 **************************************************************************************************/
//uint8 HalEcg_ReadByte(void)
//{
//  // U0CSR &= ~0x02;              // 清TX_BYTE位
//  // U0DBUF = write;
//  U0CSR &= ~ BV(1);               // 清TX_BYTE位
//  U0DBUF = 0xFF;
//  while (!(U0CSR & BV(1)));      // 等待TX_BYTE位置位
//  return (U0DBUF);
//}

/**************************************************************************************************
 * @fn      HalEcg_ReadByte
 *
 * @brief   
 *
 * @param   
 *
 * @return  
 **************************************************************************************************/
void HalEcg_ReadByte(uint8 *read, uint8 write)
{
  U0CSR &= ~ BV(1);                 // 清RX_BYTE位
  U0DBUF = write;
  while (!(U0CSR & BV(1)));         // 等待TX_BYTE位置位
  *read = U0DBUF;
}

/**************************************************************************************************
 * @fn      HalEcg_WaitMs
 *
 * @brief   wait for x ms. @ 32MHz MCU clock it takes 1000 us for 1 ms delay.
 *
 * @param   x ms. range[0-65536]
 *
 * @return  None
 **************************************************************************************************/
void HalEcg_WaitMs(uint16 milliSecs)
{
  while(milliSecs--)
  {
    HalEcg_WaitUs(1000);
  }
}


/**************************************************************************************************
 * @fn      HalEcg_WaitUs
 *
 * @brief   wait for x us. @ 32MHz MCU clock it takes 32 "nop"s for 1 us delay.
 *
 * @param   x us. range[0-65536]
 *
 * @return  None
 **************************************************************************************************/
void HalEcg_WaitUs(uint16 microSecs)
{
  while(microSecs--)
  {
    /* 32 NOPs == 1 usecs */
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");
  }
}



/**************************************************************************************************
 * @fn      getHeadBufAndFree getTailBufAndFilled
 *
 * @brief   
 *
 * @param
 *
 * @return
 **************************************************************************************************/
#define MAX_BUF_SIZE 6+1
static char buffer[MAX_BUF_SIZE][11];
static char indexTail =0;
static char indexHead =0;

char *getHeadBufAndFree()
{
    char *firstBuf = NULL;
    //CLOSE ISR
    if (indexHead != indexTail)
    {
        firstBuf = buffer[indexHead];
        indexHead = (indexHead+1)%MAX_BUF_SIZE;
    }
    //OPEN ISR
    return firstBuf;
}

char *getTailBufAndFilled()
{
    //未考虑BUFF溢出
    char *tailBuf = buffer[indexTail];
    //CLOSE ISR
    indexTail  = (indexTail+1)%MAX_BUF_SIZE;
    //OPEN ISR
    return tailBuf;
}


/**************************************************************************************************
 * @fn      halProcessECGInterrupt
 *
 * @brief   Checks to see if it's a valid ECG Data Ready interrupt.
 *
 * @param
 *
 * @return
 **************************************************************************************************/
extern uint8 ECGSensor_TaskID; 

#define ECG_PERIODIC_EVT                              0x0002

void halProcessECGInterrupt (void)
{
    uint8 *addr;
    uint8 index;

    #if defined ( CC2540_ECG )
        if( HAL_ECG_DRDY_PXIFG & HAL_ECG_DRDY_BIT)          /* Interrupt Flag has been set by ECG_DRDY */
        {
            HAL_ECG_DRDY_PXIFG &= ~(HAL_ECG_DRDY_BIT);      /* Clear Interrupt Flag */

            addr = (uint8 *)getTailBufAndFilled();
            for(index=0; index<7; index++)                 // 88Bit(24+4*16)连续11个Byte的数据
            {
                HalEcg_ReadByte(&addr[index], 0xFF);
            }
            
            osal_set_event(ECGSensor_TaskID, ECG_PERIODIC_EVT);
        }
    #endif
}
#endif //#if (defined HAL_ECG) && (HAL_ECG == TRUE)


