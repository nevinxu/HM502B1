/*******************************************************************************
  Filename:       hal_board_cfg_ecg.h
  Revised:        $Date: 2012-09-07 14:26:14 -0700 (Fri, 07 Sep 2012) $
  Revision:       $Revision: 31498 $

  Description:

  Abstract board-specific registers, addresses, & initialization for H/W based on the
  Texas Instruments CC254x (8051 core).


  Copyright 2006-2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED �AS IS? WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
*******************************************************************************/
#ifndef HAL_BOARD_CFG_ECG_H
#define HAL_BOARD_CFG_ECG_H

#ifdef __cplusplus
extern "C"
{
#endif


/*******************************************************************************
 * INCLUDES
 */

#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"

/* Board Identifier */

#if !defined (CC2540_ECG) 
  #define CC2540_ECG                    //Define the Board Identifier to CC2540_ECG
#endif



/* Clock Speed */

#define HAL_CPU_CLOCK_MHZ             32

/* Sleep Clock */

#define EXTERNAL_CRYSTAL_OSC          0x00  // external 32kHz XOSC
#define INTERNAL_RC_OSC               0x80  // internal 32kHz RCOSC

/*
 * If not using power management, assume that the 32kHz crystal is not
 * installed. Even if a 32kHz crystal is present on the board, it will
 * never get used since device does not ever go to sleep. By forcing
 * OSC32K_CRYSTAL_INSTALLED to FALSE, we avoid mismatches between
 * libraries built with power management off, and applications in which
 * power management is not used.
 */
#if ( !defined ( POWER_SAVING ) ) && ( !defined ( OSC32K_CRYSTAL_INSTALLED ) )
  #define OSC32K_CRYSTAL_INSTALLED FALSE
#endif



/* 32 kHz clock source select in CLKCONCMD */
#if !defined (OSC32K_CRYSTAL_INSTALLED) || (defined (OSC32K_CRYSTAL_INSTALLED) && (OSC32K_CRYSTAL_INSTALLED == TRUE))
  #define OSC_32KHZ  EXTERNAL_CRYSTAL_OSC /* external 32 KHz xosc */
#else
#define OSC_32KHZ                     INTERNAL_RC_OSC
#endif

/* LED Configuration */


#if defined (CC2540_ECG)
  #define HAL_NUM_LEDS            2
#endif

#define HAL_LED_BLINK_DELAY()   st( { volatile uint32 i; for (i=0; i<0x5800; i++) { }; } )

#if defined(CC2540_ECG)
  /* LED1 - P1.0 - LED_RUN Green - D300   */
  #define LED1_BV                     BV(0)
  #define LED1_SBIT                   P1_0
  #define LED1_DDR                    P1DIR
  #define LED1_POLARITY               ACTIVE_HIGH
  
  /* LED2 - P1.4 - LED_BT_STATE Green - D301   */
  #define LED2_BV                     BV(4)
  #define LED2_SBIT                   P1_4
  #define LED2_DDR                    P1DIR
  #define LED2_POLARITY               ACTIVE_HIGH
  
  /* TESTMODE_SELECT - P2.0 �͵�ƽ������Թ���ģʽ*/
  #define TESTMODE_SELECT_BV           BV(0)
  #define TESTMODE_SELECT_SBIT         P2_0
  #define TESTMODE_SELECT_DDR          P2DIR
  #define TESTMODE_SELECT_POLARITY     ACTIVE_HIGH
  
  /* AVDD_PWREN - P1.5 */
#define ACTIVE_LOW                     !
#define ACTIVE_HIGH                    !!    /* double negation forces result to be '1' */
  #define AVDD_PWREN_BV               BV(5)
  #define AVDD_PWREN_SBIT             P1_5
  #define AVDD_PWREN_DDR              P1DIR
  #define AVDD_PWREN_POLARITY         ACTIVE_HIGH

#endif  // #if defined(CC2540_ECG)
  
  /* Joystick Center Press */
  #define PUSH2_BV                       BV(0)
  #define PUSH2_SBIT                     P2_0
  #define PUSH2_POLARITY                 ACTIVE_HIGH

/* OSAL NV implemented by internal flash pages. */

// Flash is partitioned into 8 banks of 32 KB or 16 pages.
#define HAL_FLASH_PAGE_PER_BANK        16

// Flash is constructed of 128 pages of 2 KB.
#define HAL_FLASH_PAGE_PHYS            2048

// SNV can use a larger logical page size to accomodate more or bigger items or extend lifetime.
#define HAL_FLASH_PAGE_SIZE            HAL_FLASH_PAGE_PHYS
#define HAL_FLASH_WORD_SIZE            4

// CODE banks get mapped into the XDATA range 8000-FFFF.
#define HAL_FLASH_PAGE_MAP             0x8000

// The last 16 bytes of the last available page are reserved for flash lock bits.
#define HAL_FLASH_LOCK_BITS            16

// NV page definitions must coincide with segment declaration in project *.xcl file.
#if defined NON_BANKED
#define HAL_NV_PAGE_END                30
#elif defined HAL_BOARD_CC2541S
#define HAL_NV_PAGE_END                125
#else
#define HAL_NV_PAGE_END                126
#endif

// Re-defining Z_EXTADDR_LEN here so as not to include a Z-Stack .h file.
#define HAL_FLASH_IEEE_SIZE            8
#define HAL_FLASH_IEEE_PAGE            (HAL_NV_PAGE_END+1)
#define HAL_FLASH_IEEE_OSET            (HAL_FLASH_PAGE_SIZE - HAL_FLASH_LOCK_BITS - HAL_FLASH_IEEE_SIZE)
#define HAL_INFOP_IEEE_OSET            0xC

#define HAL_NV_PAGE_CNT                2
#define HAL_NV_PAGE_BEG                (HAL_NV_PAGE_END-HAL_NV_PAGE_CNT+1)

// Used by DMA macros to shift 1 to create a mask for DMA registers.
#define HAL_NV_DMA_CH                  0
#define HAL_DMA_CH_RX                  3
#define HAL_DMA_CH_TX                  4

#define HAL_NV_DMA_GET_DESC()  HAL_DMA_GET_DESC0()
#define HAL_NV_DMA_SET_ADDR(a) HAL_DMA_SET_ADDR_DESC0((a))

// The special areas for SBL must be coordinated with the linker command file; unfortunately,
// this coordination cannot be enforced by the IAR tool.
// Note how SBL "addresses" are actually flash word offsets.
#define HAL_SBL_IMG_BEG           (uint16)(0x800 / HAL_FLASH_WORD_SIZE)
#define HAL_SBL_IMG_CRC           (uint16)(0x890 / HAL_FLASH_WORD_SIZE)
#define HAL_SBL_IMG_END           (uint16)((uint32)HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE * HAL_NV_PAGE_BEG)
/* Critical Vdd Monitoring to prevent flash damage or radio lockup. */

// Vdd/3 / Internal Reference X ENOB --> (Vdd / 3) / 1.15 X 127
#define VDD_2_0  74   // 2.0 V required to safely read/write internal flash.
#define VDD_2_7  100  // 2.7 V required for the Numonyx device.

#define VDD_MIN_RUN   VDD_2_0
#define VDD_MIN_NV   (VDD_2_0+4)  // 5% margin over minimum to survive a page erase and compaction.
#define VDD_MIN_XNV  (VDD_2_7+5)  // 5% margin over minimum to survive a page erase and compaction.

/*******************************************************************************
 * MACROS
 */

/* Cache Prefetch Control */

#define PREFETCH_ENABLE()  st( FCTL = 0x08; )
#define PREFETCH_DISABLE() st( FCTL = 0x04; )

/* Setting Clocks */

// switch to the 16MHz HSOSC and wait until it is stable
#define SET_OSC_TO_HSOSC()                                                     \
{                                                                              \
  CLKCONCMD = (CLKCONCMD & 0x80) | CLKCONCMD_16MHZ;                            \
  while ( (CLKCONSTA & ~0x80) != CLKCONCMD_16MHZ );                            \
}

// switch to the 32MHz XOSC and wait until it is stable
#define SET_OSC_TO_XOSC()                                                      \
{                                                                              \
  CLKCONCMD = (CLKCONCMD & 0x80) | CLKCONCMD_32MHZ;                            \
  while ( (CLKCONSTA & ~0x80) != CLKCONCMD_32MHZ );                            \
}

// set 32kHz OSC and wait until it is stable
#define SET_32KHZ_OSC()                                                        \
{                                                                              \
  CLKCONCMD = (CLKCONCMD & ~0x80) | OSC_32KHZ;                                 \
  while ( (CLKCONSTA & 0x80) != OSC_32KHZ );                                   \
}

#define START_HSOSC_XOSC()                                                     \
{                                                                              \
  SLEEPCMD &= ~OSC_PD;            /* start 16MHz RCOSC & 32MHz XOSC */         \
  while (!(SLEEPSTA & XOSC_STB)); /* wait for stable 32MHz XOSC */             \
}

#define STOP_HSOSC()                                                           \
{                                                                              \
  SLEEPCMD |= OSC_PD;             /* stop 16MHz RCOSC */                       \
}

/* Board Initialization */
#define HAL_BOARD_INIT()                                                       \
{                                                                              \
  /* Set to 16Mhz to set 32kHz OSC, then back to 32MHz */                      \
  START_HSOSC_XOSC();                                                          \
  SET_OSC_TO_HSOSC();                                                          \
  SET_32KHZ_OSC();                                                             \
  SET_OSC_TO_XOSC();                                                           \
  STOP_HSOSC();                                                                \
                                                                               \
  /* Enable cache prefetch mode. */                                            \
  PREFETCH_ENABLE();                                                           \
}

/* Debounce */
#define HAL_DEBOUNCE(expr)    { int i; for (i=0; i<500; i++) { if (!(expr)) i = 0; } }

/* ----------- Push Buttons ---------- */
#define HAL_PUSH_BUTTON1()        (0)
#define HAL_PUSH_BUTTON2()        (0)
#define HAL_PUSH_BUTTON3()        (0)
#define HAL_PUSH_BUTTON4()        (0)
#define HAL_PUSH_BUTTON5()        (0)
#define HAL_PUSH_BUTTON6()        (0)

/* LED's */
#if defined (CC2540_ECG) 
  #define HAL_TURN_OFF_LED1()       st( LED1_SBIT = LED1_POLARITY (0); )
  #define HAL_TURN_OFF_LED2()       st( LED2_SBIT = LED2_POLARITY (0); )
  #define HAL_TURN_OFF_LED3()       HAL_TURN_OFF_LED1()
  #define HAL_TURN_OFF_LED4()       HAL_TURN_OFF_LED2()

  #define HAL_TURN_ON_LED1()        st( LED1_SBIT = LED1_POLARITY (1); )
  #define HAL_TURN_ON_LED2()        st( LED2_SBIT = LED2_POLARITY (1); )
  #define HAL_TURN_ON_LED3()        HAL_TURN_ON_LED1()
  #define HAL_TURN_ON_LED4()        HAL_TURN_ON_LED2()

  #define HAL_TOGGLE_LED1()         st( if (LED1_SBIT) { LED1_SBIT = 0; } else { LED1_SBIT = 1;} )
  #define HAL_TOGGLE_LED2()         st( if (LED2_SBIT) { LED2_SBIT = 0; } else { LED2_SBIT = 1;} )
  #define HAL_TOGGLE_LED3()         HAL_TOGGLE_LED1()
  #define HAL_TOGGLE_LED4()         HAL_TOGGLE_LED2()
  #define HAL_STATE_LED1()          (LED1_POLARITY (LED1_SBIT))
  #define HAL_STATE_LED2()          (LED2_POLARITY (LED2_SBIT))
  #define HAL_STATE_LED3()          HAL_STATE_LED1()
  #define HAL_STATE_LED4()          HAL_STATE_LED2()

#endif



///* XNV */
//#define XNV_SPI_BEGIN()             st(P1_3 = 0;)
//#define XNV_SPI_TX(x)               st(U1CSR &= ~0x02; U1DBUF = (x);)
//#define XNV_SPI_RX()                U1DBUF
//#define XNV_SPI_WAIT_RXRDY()        st(while (!(U1CSR & 0x02));)
//#define XNV_SPI_END()               st(P1_3 = 1;)
//
//// The TI reference design uses UART1 Alt. 2 in SPI mode.
//#define XNV_SPI_INIT() \
//st( \
//  /* Mode select UART1 SPI Mode as master. */\
//  U1CSR = 0; \
//  \
//  /* Setup for 115200 baud. */\
//  U1GCR = 11; \
//  U1BAUD = 216; \
//  \
//  /* Set bit order to MSB */\
//  U1GCR |= BV(5); \
//  \
//  /* Set UART1 I/O to alternate 2 location on P1 pins. */\
//  PERCFG |= 0x02;  /* U1CFG */\
//  \
//  /* Select peripheral function on I/O pins but SS is left as GPIO for separate control. */\
//  P1SEL |= 0xE0;  /* SELP1_[7:4] */\
//  /* P1.1,2,3: reset, LCD CS, XNV CS. */\
//  P1SEL &= ~0x0E; \
//  P1 |= 0x0E; \
//  P1_1 = 0; \
//  P1DIR |= 0x0E; \
//  \
//  /* Give UART1 priority over Timer3. */\
//  P2SEL &= ~0x20;  /* PRI2P1 */\
//  \
//  /* When SPI config is complete, enable it. */\
//  U1CSR |= 0x40; \
//  /* Release XNV reset. */\
//  P1_1 = 1; \
//)


/* Driver Configuration */

/* Set to TRUE enable H/W TIMER usage, FALSE disable it */
#ifndef HAL_TIMER
#define HAL_TIMER FALSE
#endif

/* Set to TRUE enable ADC usage, FALSE disable it */
#ifndef HAL_ADC
#define HAL_ADC TRUE
#endif

/* Set to TRUE enable DMA usage, FALSE disable it */
#ifndef HAL_DMA
#define HAL_DMA TRUE   //Default is TRUE
#endif

/* Set to TRUE enable Flash access, FALSE disable it */
#ifndef HAL_FLASH
#define HAL_FLASH TRUE
#endif

/* Set to TRUE enable AES usage, FALSE disable it */
#ifndef HAL_AES
#define HAL_AES TRUE
#endif

#ifndef HAL_AES_DMA
#define HAL_AES_DMA TRUE
#endif

/* Set to TRUE enable LCD usage, FALSE disable it */
#ifndef HAL_LCD
#define HAL_LCD FALSE
#endif

/* Set to TRUE enable ECG usage, FALSE disable it */
#ifndef HAL_ECG
#define HAL_ECG TRUE      // ECG�¶����HAL_ECG
#endif


/* Set to TRUE enable LED usage, FALSE disable it */
#ifndef HAL_LED
#define HAL_LED TRUE
#endif
#if (!defined BLINK_LEDS) && (HAL_LED == TRUE)
#define BLINK_LEDS
#endif

/* Set to TRUE enable KEY usage, FALSE disable it */
#ifndef HAL_KEY
#define HAL_KEY TRUE
#endif

/* Set to TRUE enable UART usage, FALSE disable it */
#ifndef HAL_UART
#define HAL_UART TRUE       
#define HAL_UART_ISR  2
#define HAL_UART_DMA  0
#define HAL_UART_USB  0
#endif

// Used to set P2 priority - USART0 over USART1 if both are defined.
#if ((HAL_UART_DMA == 1) || (HAL_UART_ISR == 1))
#define HAL_UART_PRIPO             0x00
#else
#define HAL_UART_PRIPO             0x40
#endif


#ifdef __cplusplus
}
#endif

#endif /* HAL_BOARD_CFG_ECG_H */

/*******************************************************************************
*/