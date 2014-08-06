/******************************************* Copyright (c) ****************************************
**                         Zhejiang Helowin Medical technology  Co.,LTD.
**                                   http://www.hellowin.cn
**Filename:       ECGSensor.c
**Revised:        $Date: 2012-10-08 $
**Revision:       $Revision: V1.0.0 $
**Description:    This file contains the Simple BLE Peripheral sample application 
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.
**************************************************************************************************/

/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/
#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "hal_ecg.h"          // ECG spi驱动
#include "gatt.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "battservice.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"
#include "hal_uart.h"
#include "BLE_SEND.h"
#include <string.h>

#include "Npi.h"


#if defined( CC2540_ECG )
  #include "simplekeys.h"
  #include "ads1194.h"        // ads1194头文件
#endif

#if defined ( PLUS_BROADCASTER )
  #include "peripheralBroadcaster.h"
#else
  #include "peripheral.h"
#endif

#include "gapbondmgr.h"

#include "ECGSensor.h"

/*********************************************************************
** MACROS
*********************************************************************/

/*********************************************************************
** CONSTANTS
*********************************************************************/

#define DEFAULT_BATT_PERIOD                       5000    // Battery measurement period in ms
#define DEFAULT_CALIBRATIN_PERIOD                 5000
#define DEFAULT_UART_PERIOD                          1
#define DEFAULT_ECG_PERIOD                        4      // How often to perform ECG periodic event in ms
#define ECG_PERIODIC_EVT_PERIOD                   1       // How often to perform periodic event in ms


// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely 

#if defined ( CC2540_ECG )
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
#else
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
#endif  // defined ( CC2540_ECG )

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL    10

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     20

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          50

// Battery level is critical when it is less than this %
#define DEFAULT_BATT_CRITICAL_LEVEL           6 

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

#if defined ( PLUS_BROADCASTER )
  #define ADV_IN_CONN_WAIT                    500 // delay 500 ms
#endif


/*********************************************************************
** TYPEDEFS
*********************************************************************/

/*********************************************************************
 * GLOBAL VARIABLES
 */

#define RX_BUF_MAXLEN     56
#define TX_BUF_MAXLEN     56
#define USE_UART_PORT     HAL_UART_PORT_1
static uint8 SerialApp_Buf[64]; //串口数据buf
static uint8 UartCommand = 0;
/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 ECGSensor_TaskID;   // Task ID for internal task/event processing
uint8 ECGDatum;
uint8 ECGOffset;


uint8 ECGRate = 1;
uint8 ECGGain = 1;

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x14,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x53,   // 'S'
  0x69,   // 'i'
  0x6d,   // 'm'
  0x70,   // 'p'
  0x6c,   // 'l'
  0x65,   // 'e'
  0x42,   // 'B'
  0x4c,   // 'L'
  0x45,   // 'E'
  0x50,   // 'P'
  0x65,   // 'e'
  0x72,   // 'r'
  0x69,   // 'i'
  0x70,   // 'p'
  0x68,   // 'h'
  0x65,   // 'e'
  0x72,   // 'r'
  0x61,   // 'a'
  0x6c,   // 'l'
  
  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),  
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),  

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm  
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] = 
{ 
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral                                      ·············
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16( SK_SERV_UUID ),
  HI_UINT16( SK_SERV_UUID ),

};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "HM502B1";

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void ECGSensor_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
//static void ECGPeriodicTask( void );
static void BattPeriodicTask( void );
static void simpleProfileChangeCB( uint8 paramID );

#if defined( CC2540_ECG )
static void ECGSensor_HandleKeys( uint8 shift, uint8 keys );
#endif

static void ECGSensorBattCB(uint8 event);

static char *bdAddr2Str( uint8 *pAddr ) ;

void uart_open(void);
void uart_rx_cback(uint8 port, uint8 event);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t ECGSensor_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t ECGSensor_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t ECGSensor_SimpleProfileCBs =
{
  simpleProfileChangeCB    // Charactersitic value change callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      ECGSensor_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */

void delay(unsigned int times)
{
  unsigned int x,y;
  for(x=0;x<100;x++)
    for(y=0;y<times;y++);
}


void ECGSensor_Init( uint8 task_id )
{
  ECGSensor_TaskID = task_id;

  #ifdef POWER_SAVING
    (void)osal_pwrmgr_task_state( task_id, PWRMGR_CONSERVE );
  #endif
    
    
  uart_open();
    
  // Setup the GAP Peripheral Role Profile

  // 对于CC2540_ECG, 设备初始化时，进行广播
  uint8 initial_advertising_enable = TRUE;

  // By setting this to zero, the device will go into the waiting state after
  // being discoverable for 30.72 second, and will not being advertising again
  // until the enabler is set back to TRUE
  uint16 gapRole_AdvertOffTime = 0;
    
  uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
  uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
  uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
  uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
  uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

  // Set the GAP Role Parameters
  GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
  GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
  
  GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
  GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
  
  GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
  GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
  GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
  GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
  GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

// Set advertising interval

  uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;
  GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
  GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
  GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
  GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );


// Setup the GAP Bond Manager

  uint32 passkey = 0; // passkey "000000"
  uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
  uint8 mitm = TRUE;
  uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
  uint8 bonding = TRUE;
  GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
  GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
  GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
  GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
  GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding ); 
   
  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
  
  // 设置Battery Characteristic Values
  uint8 critical = DEFAULT_BATT_CRITICAL_LEVEL;
  Batt_SetParameter( BATT_PARAM_CRITICAL_LEVEL, sizeof (uint8 ), &critical ); 
  Batt_AddService( );                             // 电池电量服务
  //Register for Battery service callback;
  Batt_Register ( ECGSensorBattCB );


// Setup the SimpleProfile Characteristic Values
  uint8 charValue1 = 1;
  uint8 charValue2 = 2;
  uint8 charValue3 = 3;
  uint8 charValue4 = 4; 
  uint8 charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };
  SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint8 ), &charValue1 );
  SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, sizeof ( uint8 ), &charValue2 );
  SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( uint8 ), &charValue3 );
  SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 ), &charValue4 );
  SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, charValue5 );
 
  // Simple Keys Profile
  SK_AddService( GATT_ALL_SERVICES );
  // Register for all key events - This app will handle all key events
  RegisterForKeys( ECGSensor_TaskID );  


  // Register callback with SimpleGATTprofile
  VOID SimpleProfile_RegisterAppCBs( &ECGSensor_SimpleProfileCBs );

  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1), HAL_LED_MODE_OFF ); 
  HalLedSet( (HAL_LED_2), HAL_LED_MODE_ON ); 
  
  // Setup a delayed profile startup
  osal_set_event( ECGSensor_TaskID, START_DEVICE_EVT ); 
  
  // 初始化ADS1194
  ADS1194_Init(ECGRate,ECGGain);
}
 

/*********************************************************************
 * @fn      ECGSensorBattCB
 *
 * @brief   Callback function for battery service.
 *
 * @param   event - service event
 *
 * @return  none
 */
static void ECGSensorBattCB(uint8 event)
{
  if (event == BATT_LEVEL_NOTI_ENABLED)
  {
    // if connected start periodic measurement
    if (gapProfileState == GAPROLE_CONNECTED)
    {
      osal_start_timerEx( ECGSensor_TaskID, BATT_PERIODIC_EVT, DEFAULT_BATT_PERIOD );
    } 
  }
  else if (event == BATT_LEVEL_NOTI_DISABLED)
  {
    // stop periodic measurement
    osal_stop_timerEx( ECGSensor_TaskID, BATT_PERIODIC_EVT );
  }
}

/*********************************************************************
 * @fn      ECGSensor_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 ECGSensor_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( ECGSensor_TaskID )) != NULL )
    {
      ECGSensor_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &ECGSensor_PeripheralCBs );   
    // Start Bond Manager
    VOID GAPBondMgr_Register( &ECGSensor_BondMgrCBs );
//    osal_start_timerEx( ECGSensor_TaskID, BATT_PERIODIC_EVT, DEFAULT_BATT_PERIOD );     
//    osal_start_timerEx( ECGSensor_TaskID, UART_PERIODIC_EVT, DEFAULT_UART_PERIOD );  
    return ( events ^ START_DEVICE_EVT );
  }

  if ( events & ECG_PERIODIC_EVT )
  {
    // Perform periodic ECG task
    if (gapProfileState == GAPROLE_CONNECTED)
    {
        char *getHeadBufAndFree();
        uint8 *data =(uint8*)getHeadBufAndFree();
        if (data != NULL)
        {
            Rev_data_analy(data,7);
        } 
        #ifdef UARTDEBUG
        SerialApp_Buf[0]++;
        HalUARTWrite(USE_UART_PORT, SerialApp_Buf,1);    
        #endif
        LedFlashContinuous();
    }  
    return (events ^ ECG_PERIODIC_EVT);
  } 
   
  if ( events & BATT_PERIODIC_EVT )    //电池电量获取函数
  {
    // Perform periodic battery task
    BattPeriodicTask();
    osal_stop_timerEx(ECGSensor_TaskID, BATT_PERIODIC_EVT);
  //  osal_start_timerEx( ECGSensor_TaskID, BATT_PERIODIC_EVT, DEFAULT_BATT_PERIOD );  
    return (events ^ BATT_PERIODIC_EVT);
  } 

  if ( events & CALIBRATION_PERIODIC_EVT )
  {
    return (events ^ CALIBRATION_PERIODIC_EVT);
  }    
  
  if ( events & UART_PERIODIC_EVT )
  {
    #ifdef UARTDEBUG
    memset(SerialApp_Buf,0,sizeof(SerialApp_Buf));
    if(UartCommand == 1)
    {
        strcpy(SerialApp_Buf,"Initialized\n");
        HalUARTWrite(USE_UART_PORT, SerialApp_Buf,sizeof("Initialized\n"));   
    }
    else if(UartCommand == 2)
    {
        strcpy(SerialApp_Buf,"Advertising\n");
        HalUARTWrite(USE_UART_PORT, SerialApp_Buf,sizeof("Advertising\n"));  
    }
    UartCommand = 0;
    osal_start_timerEx( ECGSensor_TaskID, UART_PERIODIC_EVT, DEFAULT_UART_PERIOD );
    #endif
    return (events ^ UART_PERIODIC_EVT);
  } 
  
  
  return 0;
}

/*********************************************************************
 * @fn      ECGSensor_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void ECGSensor_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      ECGSensor_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;      
  default:
    // do nothing
    break;
  }
}

/*********************************************************************
 * @fn      ECGSensor_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void ECGSensor_HandleKeys( uint8 shift, uint8 keys )
{
  uint8 SK_Keys = 0;

  VOID shift;  // Intentionally unreferenced parameter
  if ( keys & HAL_KEY_SW_1 )
  {
    SK_Keys |= SK_KEY_LEFT;
    
    HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON );       //
    
  }
  
  if ( keys & HAL_KEY_SW_2 )
  {
    SK_Keys |= SK_KEY_RIGHT;
    
    // Setup Battery Characteristic Values
    {
      uint8 clientbattLevel;
      Batt_SetParameter( BATT_PARAM_LEVEL, sizeof (uint8 ), &clientbattLevel );
    }
  }

  // Set the value of the keys state to the Simple Keys Profile;
  // This will send out a notification of the keys state if enabled
  SK_SetParameter( SK_KEY_ATTR, sizeof ( uint8 ), &SK_Keys );
}


/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {    
        uint8 ownAddress[B_ADDR_LEN];
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];
        
        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
    
        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];
    
        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;
    
        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];
        
        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
        UartCommand = 1;
        #ifdef UARTDEBUG
        memset(SerialApp_Buf,0,sizeof(SerialApp_Buf));
        strcpy(SerialApp_Buf,"Initialized\nMAC: ");
        strcat(SerialApp_Buf,bdAddr2Str(ownAddress));
        strcat(SerialApp_Buf,"\n");
        HalUARTWrite(USE_UART_PORT, SerialApp_Buf,sizeof(SerialApp_Buf));    
        #endif
        
      }
      break;
      
    case GAPROLE_ADVERTISING:
      {
        HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON ); 
        UartCommand = 2;
        #ifdef UARTDEBUG
        memset(SerialApp_Buf,0,sizeof(SerialApp_Buf));
        strcpy(SerialApp_Buf,"Advertising\n");
        HalUARTWrite(USE_UART_PORT, SerialApp_Buf,sizeof("Advertising\n"));   
        #endif
      }
      break;

    case GAPROLE_CONNECTED:
      {
        HalLedSet ( HAL_LED_1, HAL_LED_MODE_FLASH );                              //LED1间隔闪烁 
        #ifdef UARTDEBUG
        memset(SerialApp_Buf,0,sizeof(SerialApp_Buf));
        strcpy(SerialApp_Buf,"Connected\n");
        HalUARTWrite(USE_UART_PORT, SerialApp_Buf,sizeof("Connected\n"));   
        #endif
        ADS1194_StartReadDataContinuous();       
      }
      break;      

    case GAPROLE_WAITING:
      {
        HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON );       //
        uint8 once_advertising_enable = TRUE;
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &once_advertising_enable );  //重新发送Advertising，断开之后的重连
        #ifdef UARTDEBUG
        memset(SerialApp_Buf,0,sizeof(SerialApp_Buf));
        strcpy(SerialApp_Buf,"Disconnected\n");
        HalUARTWrite(USE_UART_PORT, SerialApp_Buf,sizeof(SerialApp_Buf));   
        #endif
      }
      break;      

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
        HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON ); 
        #ifdef UARTDEBUG
        memset(SerialApp_Buf,0,sizeof(SerialApp_Buf));
        strcpy(SerialApp_Buf,"Timed Out\n");
        HalUARTWrite(USE_UART_PORT, SerialApp_Buf,sizeof(SerialApp_Buf));   
        #endif               
        uint8 once_advertising_enable = TRUE;
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &once_advertising_enable );  //重新发送Advertising，断开之后的重连
        #ifdef UARTDEBUG
        memset(SerialApp_Buf,0,sizeof(SerialApp_Buf));
        strcpy(SerialApp_Buf,"Try connect again\n");
        HalUARTWrite(USE_UART_PORT, SerialApp_Buf,sizeof(SerialApp_Buf));   
        #endif 
      }
      break;      

    case GAPROLE_ERROR:
      {
        #ifdef UARTDEBUG
        memset(SerialApp_Buf,0,sizeof(SerialApp_Buf));
        strcpy(SerialApp_Buf,"Error\n");
        HalUARTWrite(USE_UART_PORT, SerialApp_Buf,sizeof(SerialApp_Buf));   
        #endif 
        uint8 once_advertising_enable = TRUE;
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &once_advertising_enable );  //重新发送Advertising，断开之后的重连

      }
      break;      
      
    default:
      {
        #ifdef UARTDEBUG
        memset(SerialApp_Buf,0,sizeof(SerialApp_Buf));
        strcpy(SerialApp_Buf,"Hellowin\n");
        HalUARTWrite(USE_UART_PORT, SerialApp_Buf,sizeof(SerialApp_Buf));   
        #endif 
      }        
      break;
    
  }
  
  gapProfileState = newState;

#if !defined( CC2540_ECG )  
  VOID gapProfileState;     // added to prevent compiler warning with
                            // "CC2540 Slave" configurations
#endif

  
}

/*********************************************************************
 * @fn      ECGPeriodicTask()
 *
 * @brief   Perform a periodic application task. This function gets
 *          called every five seconds as a result of the ECG_PERIODIC_EVT
 *          OSAL event. In this example, the value of the third 
 *          characteristic in the SimpleGATTProfile service is retrieved
 *          from the profile, and then copied into the value of the
 *          the fourth characteristic.
 *
 * @param   none
 *
 * @return  none
 */
 
/*********************************************************************
static uint8 ecg_grb_buf[];

static void ECGPeriodicTask( void )
{
    if (gapProfileState == GAPROLE_CONNECTED)
    {
        send ECG measurement notification
        ECGMeasNotify();
        Restart timer
        osal_start_reload_timer( ECGSensor_TaskID, ECG_PERIODIC_EVT, DEFAULT_ECG_PERIOD );
    }
  
    osal_start_timerEx( ECGSensor_TaskID, ECG_PERIODIC_EVT, DEFAULT_ECG_PERIOD );
    HalUARTWrite(HAL_UART_PORT_1, "ECGPeriodicTask\n", 17);   //
  
    //ECG采集数据通过UART口发送到PC
    databuf[0]='E';       // 数据头：字符"E"
    if(ADS1194_ReadDataContinuous(&databuf[0]))
    {
        HalUARTWrite(HAL_UART_PORT_1,databuf, 11); 
        Rev_data_analy(databuf,11);//
        ECGSensor_Cnt++;
        memcpy(databuf,&ecg_grb_buf[ECGSensor_Cnt*11],11);
    }
    
    if(ECGSensor_Cnt>10)
    {
        ECGSensor_Cnt=0;
        ecg_grb_buf[60]='E';
        HalUARTWrite(HAL_UART_PORT_1,ecg_grb_buf, 110);   //
    }
    ADS1194_StartReadDataContinuous();
}
*********************************************************************/



/*********************************************************************
 * @fn      BattPeriodicTask
 *
 * @brief   Perform a periodic task for battery measurement.
 *
 * @param   none
 *
 * @return  none
 */
static void BattPeriodicTask( void )
{
  if (gapProfileState == GAPROLE_CONNECTED)
  {
    // perform battery level check
    Batt_MeasLevel( );
    
    // Restart timer
  //  osal_start_timerEx( ECGSensor_TaskID, BATT_PERIODIC_EVT, DEFAULT_BATT_PERIOD );
  }
  else
  {
//    // perform battery level check
//    Batt_MeasLevel( );
//    
//    // Restart timer
//    osal_start_timerEx( ECGSensor_TaskID, BATT_PERIODIC_EVT, DEFAULT_BATT_PERIOD );
//    
//    HalUARTWrite(HAL_UART_PORT_1,"100%", 4);   //
  }
}

/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void simpleProfileChangeCB( uint8 paramID )
{
  uint8 newValue;
  
  switch( paramID )
  {
    case SIMPLEPROFILE_CHAR1:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, &newValue );
      ECGRate = newValue;
      Reset_ADS1194_Mode(ECGRate,ECGGain);
 //     ADS1194_Init(ECGRate,ECGGain);

      break;
  
    case SIMPLEPROFILE_CHAR3:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &newValue );
      ECGGain = newValue;
      Reset_ADS1194_Mode(ECGRate,ECGGain);
  //    ADS1194_Init(ECGRate,ECGGain);
      break;
  
    default:
      // should not reach here!
      break;
  }  
}

/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )  //
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;
  
  *pStr++ = '0';
  *pStr++ = 'x';
  
  // Start from end of addr
  pAddr += B_ADDR_LEN;
  
  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }
  
  *pStr = 0;
  
  return str;
}


/*********************************************************************
 * @fn      uart_open
 *
 * @brief   
 *
 * @return  none
 */

#define RX_BUF_LEN     56

void uart_open(void)
{
  halUARTCfg_t uartConfig;
  
  uartConfig.configured           = TRUE;              
  uartConfig.baudRate             = HAL_UART_BR_115200;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = 48;
  uartConfig.rx.maxBufSize        = RX_BUF_MAXLEN;
  uartConfig.tx.maxBufSize        = TX_BUF_MAXLEN;
  
  uartConfig.idleTimeout          = 1;   
  uartConfig.intEnable            = TRUE;              
  uartConfig.callBackFunc         = (halUARTCBack_t)&uart_rx_cback;
  
  HalUARTOpen (HAL_UART_PORT_1, &uartConfig);
} 

void uart_rx_cback(uint8 port, uint8 event)
{
  uint8 SerialApp_TxLen;
  uint8 usbEvent = event;
  switch(usbEvent)
  {
  case HAL_UART_RX_FULL:
    
  case HAL_UART_RX_ABOUT_FULL:
    
  case HAL_UART_RX_TIMEOUT:
//    delay(1);//防止不丢数据
    SerialApp_TxLen = HalUARTRead(HAL_UART_PORT, SerialApp_Buf, 50);
    if(SerialApp_TxLen)
    {     
      
    }
    break;
    
  case HAL_UART_TX_FULL:
    break;
    
  case HAL_UART_TX_EMPTY:
    break;
    
  default:
    break;
  }
}

/*********************************************************************
*********************************************************************/
