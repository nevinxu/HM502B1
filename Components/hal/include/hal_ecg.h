/******************************************* Copyright (c) **********************************************
**                         Zhejiang Helowin Medical technology  Co.,LTD.
**
**                                   http://www.hellowin.cn
**
********************************************************************************************************/
#ifndef  HAL_ECG_H
#define  HAL_ECG_H

#include "hal_types.h"
#include "hal_board.h"

/******************************************* Copyright (c) **********************************************
Config
********************************************************************************************************/


/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/
#define HAL_ECG_RISING_EDGE   0
#define HAL_ECG_FALLING_EDGE  1


/* Control macros */
//#define HAL_ECG TURE

/******************************************* Copyright (c) **********************************************
struct define
********************************************************************************************************/


/******************************************* Copyright (c) **********************************************
Macro Define
********************************************************************************************************/



/******************************************* Copyright (c) **********************************************
Export function
********************************************************************************************************/
extern void HalEcgInit(void);
extern void HalEcg_ConfigIO(void);
extern uint8 HalEcg_DrdyStatus(void);
extern void HalEcg_PowerEnable(void);
extern void HalEcg_PowerDisable(void);
extern void HalEcg_InterruptEnable(void);
extern void HalEcg_ConfigSPI(void);
extern void HalEcg_CSEnable(void);
extern void HalEcg_CSDisable(void);
extern void HalEcg_WriteByte(uint8);
// extern uint8 HalEcg_ReadByte(void);
extern void HalEcg_ReadByte(uint8 *read, uint8 write);
extern void HalEcg_WaitMs(uint16);
extern void HalEcg_WaitUs(uint16);
extern void halProcessECGInterrupt (void);


#endif  // #ifndef  HAL_ECG_H

/*****************************************  http://www.hellowin.cn  ************************************/



