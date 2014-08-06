/**************************************************************************************************
  Filename:       hal_key.c
  Revised:        $Date: 2010-09-15 19:02:45 -0700 (Wed, 15 Sep 2010) $
  Revision:       $Revision: 23815 $

  Description:    This file contains the interface to the HAL KEY Service.


  Copyright 2006-2010 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
**************************************************************************************************/
/*********************************************************************
 NOTE: If polling is used, the hal_driver task schedules the KeyRead()
       to occur every 100ms.  This should be long enough to naturally
       debounce the keys.  The KeyRead() function remembers the key
       state of the previous poll and will only return a non-zero
       value if the key state changes.

 NOTE: If interrupts are used, the KeyRead() function is scheduled
       25ms after the interrupt occurs by the ISR.  This delay is used
       for key debouncing.  The ISR disables any further Key interrupt
       until KeyRead() is executed.  KeyRead() will re-enable Key
       interrupts after executing.  Unlike polling, when interrupts
       are enabled, the previous key state is not remembered.  This
       means that KeyRead() will return the current state of the keys
       (not a change in state of the keys).

 NOTE: If interrupts are used, the KeyRead() fucntion is scheduled by
       the ISR.  Therefore, the joystick movements will only be detected
       during a pushbutton interrupt caused by S1 or the center joystick
       pushbutton.

 NOTE: When a switch like S1 is pushed, the S1 signal goes from a normally
       high state to a low state.  This transition is typically clean.  The
       duration of the low state is around 200ms.  When the signal returns
       to the high state, there is a high likelihood of signal bounce, which
       causes a unwanted interrupts.  Normally, we would set the interrupt
       edge to falling edge to generate an interrupt when S1 is pushed, but
       because of the signal bounce, it is better to set the edge to rising
       edge to generate an interrupt when S1 is released.  The debounce logic
       can then filter out the signal bounce.  The result is that we typically
       get only 1 interrupt per button push.  This mechanism is not totally
       foolproof because occasionally, signal bound occurs during the falling
       edge as well.  A similar mechanism is used to handle the joystick
       pushbutton on the DB.  For the EB, we do not have independent control
       of the interrupt edge for the S1 and center joystick pushbutton.  As
       a result, only one or the other pushbuttons work reasonably well with
       interrupts.  The default is the make the S1 switch on the EB work more
       reliably.

*********************************************************************/

/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/
#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "hal_drivers.h"
#include "hal_adc.h"
#include "hal_key.h"
#include "osal.h"

#include "hal_led.h"

#include "hal_ecg.h"      // ECGͷ�ļ�

#if (defined HAL_KEY) && (HAL_KEY == TRUE)

/**************************************************************************************************
 *                                              MACROS
 **************************************************************************************************/
#if  defined (CC2540_ECG)
#define HAL_KEY_CLR_INT()  st ( P0IFG = 0; P0IF = 0;)     // PxIFG has to be cleared before PxIF
#endif  // defined (CC2540_EVM)

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/
#define HAL_KEY_RISING_EDGE       0
#define HAL_KEY_FALLING_EDGE      1

#define HAL_KEY_DEBOUNCE_VALUE    25

/* CPU port interrupt */
#define HAL_KEY_CPU_PORT_0_IF     P0IF
#define HAL_KEY_CPU_PORT_1_IF     P1IF
#define HAL_KEY_CPU_PORT_2_IF     P2IF

#if defined ( CC2540_ECG )
/* SW_1 is at P0.0 */
#define HAL_KEY_SW_1_PORT         P0
#define HAL_KEY_SW_1_BIT          BV(0)
#define HAL_KEY_SW_1_SEL          P0SEL
#define HAL_KEY_SW_1_DIR          P0DIR
#endif //  #if defined ( CC2540_ECG )

#if defined ( CC2540_ECG )
#define HAL_KEY_SW_1_IEN          IEN1  /* CPU interrupt mask register */
#define HAL_KEY_SW_1_IENBIT       BV(5) /* Bit5:P0IE.Mask bit for all of Port_0 */
#define HAL_KEY_SW_1_ICTL         P0IEN /* Port Interrupt Control register */
#define HAL_KEY_SW_1_ICTLBIT      BV(0) /* P0IEN - P0.0 enable/disable bit */
#define HAL_KEY_SW_1_PXIFG        P0IFG /* Interrupt flag at source */
#define HAL_KEY_SW_1_PXINP        P0INP /* pull up */

#define HAL_KEY_SW_PXINP12        P2INP   /* pull up */
#endif  // #if defined ( CC2540_ECG )

#if defined ( CC2540_ECG )
#define HAL_KEY_SW_1_EDGEBIT      BV(0)
#else
#define HAL_KEY_SW_1_EDGEBIT      BV(1)
#endif


//ECG_DRDY
#define HAL_ECG_RISING_EDGE       0
#define HAL_ECG_FALLING_EDGE      1

#define HAL_ECG_DEBOUNCE_VALUE    25

/* CPU port interrupt */
#define HAL_ECG_CPU_PORT_0_IF P0IF

#if defined ( CC2540_ECG )
/* ECG_DRDY is at P0.6 */
#define HAL_ECG_DRDY_PORT         P0
#define HAL_ECG_DRDY_BIT          BV(6)
#define HAL_ECG_DRDY_SEL          P0SEL
#define HAL_ECG_DRDY_DIR          P0DIR
#endif //  #if defined ( CC2540_ECG )

#if defined ( CC2540_ECG )
#define HAL_ECG_DRDY_IEN          IEN1    /* CPU interrupt mask register */
#define HAL_ECG_DRDY_IENBIT       BV(5)   /* Bit5:P0IE.Mask bit for all of Port_0 */
#define HAL_ECG_DRDY_ICTL         P0IEN   /* Port Interrupt Control register */
#define HAL_ECG_DRDY_ICTLBIT      BV(6)   /* P0IEN - P0.6 enable/disable bit */
#define HAL_ECG_DRDY_PXIFG        P0IFG   /* Interrupt flag at source */
#define HAL_ECG_DRDY_PXINP        P0INP   /* pull up */

#define HAL_ECG_DRDY_PXINP12      P2INP   /* pull up */
#endif  // #if defined ( CC2540_ECG )

#if defined ( CC2540_ECG )
#define HAL_ECG_DRDY_EDGEBIT      BV(0)
#else
#define HAL_ECG_DRDY_EDGEBIT      BV(1)
#endif  // #if defined ( CC2540_ECG )

/**************************************************************************************************
 *                                            TYPEDEFS
 **************************************************************************************************/


/**************************************************************************************************
 *                                        GLOBAL VARIABLES
 **************************************************************************************************/
static uint8 halKeySavedKeys;     /* used to store previous key state in polling mode */
static uint32 halKeySaveKeysTime; //��������ʱ��
static halKeyCBack_t pHalKeyProcessFunction;
static uint8 HalKeyConfigured;
bool Hal_KeyIntEnable;            /* interrupt enable/disable flag */

/**************************************************************************************************
 *                                        FUNCTIONS - Local
 **************************************************************************************************/
void halProcessKeyInterrupt(void);
uint8 halGetJoyKeyInput(void);



/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/


/**************************************************************************************************
 * @fn      HalKeyInit
 *
 * @brief   Initilize Key Service
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void HalKeyInit( void )
{
  /* Initialize previous key to 0 */
  halKeySavedKeys = 0;

#if defined ( CC2540_ECG )
  HAL_KEY_SW_1_SEL &= ~(HAL_KEY_SW_1_BIT);    /* Set pin function to GPIO */
  HAL_KEY_SW_1_DIR &= ~(HAL_KEY_SW_1_BIT);    /* Set pin direction to Input */
  
  HAL_KEY_SW_1_PXINP &= ~(HAL_KEY_SW_1_BIT);  /* P0.0 I/O input mode:pullup/pulldown */
  HAL_KEY_SW_PXINP12 &= ~0x20;                /* Port 0 pullup select */
#endif

  /* Initialize callback function */
  pHalKeyProcessFunction  = NULL;

  /* Start with key is not configured */
  HalKeyConfigured = FALSE;
}


/**************************************************************************************************
 * @fn      HalKeyConfig
 *
 * @brief   Configure the Key serivce
 *
 * @param   interruptEnable - TRUE/FALSE, enable/disable interrupt
 *          cback - pointer to the CallBack function
 *
 * @return  None
 **************************************************************************************************/
void HalKeyConfig (bool interruptEnable, halKeyCBack_t cback)
{
  /* Enable/Disable Interrupt or */
  Hal_KeyIntEnable = interruptEnable;

  /* Register the callback fucntion */
  pHalKeyProcessFunction = cback;

  /* Determine if interrupt is enable or not */
  if (Hal_KeyIntEnable)
  {
#if defined ( CC2540_ECG )

    HAL_KEY_SW_1_ICTL   |= HAL_KEY_SW_1_ICTLBIT;    /* enable interrupt generation at port */
    HAL_KEY_SW_1_PXIFG  &= ~(HAL_KEY_SW_1_BIT);     /* Clear any pending interrupt */
    HAL_KEY_SW_1_IEN    |= HAL_KEY_SW_1_IENBIT;     /* enable CPU interrupt */

    /* Rising/Falling edge configuratinn */
    // PICTL &= ~(HAL_KEY_SW_1_EDGEBIT);    /* Clear the edge bit */
    PICTL |= HAL_KEY_SW_1_EDGEBIT;
#else


    /* For falling edge, the bit must be set. */
  #if (HAL_KEY_SW_6_EDGE == HAL_KEY_FALLING_EDGE)

  #endif


    /* Interrupt configuration:
     * - Enable interrupt generation at the port
     * - Enable CPU interrupt
     * - Clear any pending interrupt
     */
    HAL_KEY_SW_6_ICTL |= HAL_KEY_SW_6_ICTLBIT;
    HAL_KEY_SW_6_IEN |= HAL_KEY_SW_6_IENBIT;
    HAL_KEY_SW_6_PXIFG = ~(HAL_KEY_SW_6_BIT);



    /* Rising/Falling edge configuratinn */

    HAL_KEY_JOY_MOVE_ICTL &= ~(HAL_KEY_JOY_MOVE_EDGEBIT);    /* Clear the edge bit */
    /* For falling edge, the bit must be set. */
  #if (HAL_KEY_JOY_MOVE_EDGE == HAL_KEY_FALLING_EDGE)
    HAL_KEY_JOY_MOVE_ICTL |= HAL_KEY_JOY_MOVE_EDGEBIT;
  #endif


    /* Interrupt configuration:
     * - Enable interrupt generation at the port
     * - Enable CPU interrupt
     * - Clear any pending interrupt
     */
    HAL_KEY_JOY_MOVE_ICTL |= HAL_KEY_JOY_MOVE_ICTLBIT;
    HAL_KEY_JOY_MOVE_IEN |= HAL_KEY_JOY_MOVE_IENBIT;
    HAL_KEY_JOY_MOVE_PXIFG = ~(HAL_KEY_JOY_MOVE_BIT);
#endif // !CC2540_ECG

    /* Do this only after the hal_key is configured - to work with sleep stuff */
    if (HalKeyConfigured == TRUE)
    {
      osal_stop_timerEx(Hal_TaskID, HAL_KEY_EVENT);  /* Cancel polling if active */
    }
  }
  else    /* Interrupts NOT enabled */
  {
#if defined ( CC2540_ECG )
    HAL_KEY_SW_1_ICTL &= ~(HAL_KEY_SW_1_ICTLBIT);   /* don't generate interrupt */
    HAL_KEY_SW_1_IEN  &= ~(HAL_KEY_SW_1_IENBIT);    /* Clear interrupt enable bit */
#endif  // !CC2540_ECG

    osal_set_event(Hal_TaskID, HAL_KEY_EVENT);
  }

  /* Key now is configured */
  HalKeyConfigured = TRUE;
}


/**************************************************************************************************
 * @fn      HalKeyRead
 *
 * @brief   Read the current value of a key
 *
 * @param   None
 *
 * @return  keys - current keys status
 **************************************************************************************************/
uint8 HalKeyRead ( void )
{
  uint8 keys = 0;

#if defined (CC2540_ECG)
  if (!(HAL_KEY_SW_1_PORT & HAL_KEY_SW_1_BIT))    /* Key is active low */
  {
    keys |= HAL_KEY_SW_1;
  }
#endif  // #if defined (CC2540_ECG)
  return keys;
}

uint32 HalKeyDownMS(void)
{
  return osal_GetSystemClock()-halKeySaveKeysTime;
}

/**************************************************************************************************
 * @fn      HalKeyPoll
 *
 * @brief   Called by hal_driver to poll the keys
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalKeyPoll (void)
{
  uint8 keys = 0;
  uint8 notify = 0;
#if defined (CC2540_ECG)
  if (!(HAL_KEY_SW_1_PORT & HAL_KEY_SW_1_BIT))    /* Key is active low */
  {
    keys |= HAL_KEY_SW_1;
  }
#endif  // #if defined (CC2540_ECG)

  /* If interrupts are not enabled, previous key status and current key status
   * are compared to find out if a key has changed status.
   */
  if (!Hal_KeyIntEnable)
  {
    if (keys == halKeySavedKeys)
    {
      if (HalKeyDownMS() > 1500)      //����
      {
          extern uint8 OnboardKeyIntEnable;
          extern void HalKeyConfig (bool interruptEnable, halKeyCBack_t cback);
          extern void OnBoard_KeyCallback ( uint8 keys, uint8 state);
          uint8 OnBoard_SendKeys( uint8 keys, uint8 state );
          
          OnBoard_SendKeys( keys, 4);
          OnboardKeyIntEnable = HAL_KEY_INTERRUPT_ENABLE;
          HalKeyConfig( OnboardKeyIntEnable, OnBoard_KeyCallback);  
      }
      /* Exit - since no keys have changed */
      return;
    }
    else
    {
      notify = 1;
    }
  }
  else
  {
    /* Key interrupt handled here */
    if (keys)
    {
      notify = 1;
    }
  }

  /* Store the current keys for comparation next time */
  halKeySavedKeys = keys;  
  halKeySaveKeysTime = osal_GetSystemClock();
  
  /* Invoke Callback if new keys were depressed */
  if (notify && (pHalKeyProcessFunction))
  {
    (pHalKeyProcessFunction) (keys, HAL_KEY_STATE_NORMAL);
    
  }
}

#if !defined ( CC2540_ECG )
/**************************************************************************************************
 * @fn      halGetJoyKeyInput
 *
 * @brief   Map the ADC value to its corresponding key.
 *
 * @param   None
 *
 * @return  keys - current joy key status
 **************************************************************************************************/
uint8 halGetJoyKeyInput(void)
{
  /* The joystick control is encoded as an analog voltage.
   * Read the JOY_LEVEL analog value and map it to joy movement.
   */
  uint8 adc;
  uint8 ksave0 = 0;
  uint8 ksave1;

  /* Keep on reading the ADC until two consecutive key decisions are the same. */
  do
  {
    ksave1 = ksave0;    /* save previouse key reading */

    adc = HalAdcRead (HAL_KEY_JOY_CHN, HAL_ADC_RESOLUTION_8);

    if ((adc >= 2) && (adc <= 38))
    {
       ksave0 |= HAL_KEY_UP;
    }
    else if ((adc >= 74) && (adc <= 88))
    {
      ksave0 |= HAL_KEY_RIGHT;
    }
    else if ((adc >= 60) && (adc <= 73))
    {
      ksave0 |= HAL_KEY_LEFT;
    }
    else if ((adc >= 39) && (adc <= 59))
    {
      ksave0 |= HAL_KEY_DOWN;
    }
    else if ((adc >= 89) && (adc <= 100))
    {
      ksave0 |= HAL_KEY_CENTER;
    }
  } while (ksave0 != ksave1);

  return ksave0;
}
#endif

/**************************************************************************************************
 * @fn      halProcessKeyInterrupt
 *
 * @brief   Checks to see if it's a valid key interrupt, saves interrupt driven key states for
 *          processing by HalKeyRead(), and debounces keys by scheduling HalKeyRead() 25ms later.
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void halProcessKeyInterrupt (void)
{
  bool valid=FALSE;

#if defined ( CC2540_ECG )
  if( HAL_KEY_SW_1_PXIFG & HAL_KEY_SW_1_BIT)    /* Interrupt Flag has been set by SW1 */
  {
    HAL_KEY_SW_1_PXIFG &= ~(HAL_KEY_SW_1_BIT);   /* Clear Interrupt Flag */
    valid = TRUE;

    if (valid)
    {
      osal_start_timerEx (Hal_TaskID, HAL_KEY_EVENT, HAL_KEY_DEBOUNCE_VALUE);
    }
  }
#endif  
}

/**************************************************************************************************
 * @fn      HalKeyEnterSleep
 *
 * @brief  - Get called to enter sleep mode
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void HalKeyEnterSleep ( void )
{
}

/**************************************************************************************************
 * @fn      HalKeyExitSleep
 *
 * @brief   - Get called when sleep is over
 *
 * @param
 *
 * @return  - return saved keys
 **************************************************************************************************/
uint8 HalKeyExitSleep ( void )
{
  /* Wake up and read keys */
  return ( HalKeyRead () );
}


/***************************************************************************************************
 *                                    INTERRUPT SERVICE ROUTINE
 ***************************************************************************************************/
#if !defined ( CC2540_USB_DONGLE )
/**************************************************************************************************
 * @fn      halKeyPort0Isr
 *
 * @brief   Port0 ISR
 *
 * @param
 *
 * @return
 **************************************************************************************************/
HAL_ISR_FUNCTION( halKeyPort0Isr, P0INT_VECTOR )
{
#if defined ( CC2540_ECG )
  if (HAL_KEY_SW_1_PXIFG & HAL_KEY_SW_1_BIT)
  {
    halProcessKeyInterrupt(); 
  }
  else if(HAL_ECG_DRDY_PXIFG & HAL_ECG_DRDY_BIT)
  {
    halProcessECGInterrupt(); 
  }

  /*
    Clear the CPU interrupt flag for Port_0
    PxIFG has to be cleared before PxIF
  */
  HAL_KEY_CLR_INT();
#endif
}
#else  // #if defined ( CC2540_ECG )


#endif
#else


void HalKeyInit(void){}
void HalKeyConfig(bool interruptEnable, halKeyCBack_t cback){}
uint8 HalKeyRead(void){ return 0;}
void HalKeyPoll(void){}

#endif /* HAL_KEY */





/**************************************************************************************************
**************************************************************************************************/



