#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "hal_led.h"
#include "hal_key.h"
#include "gatt.h"
#include "ll.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "simpleGATTprofile.h"
#include "simpleBLECentral.h"
#include "hal_uart.h"
#include "string.h"

/*********************************************************************
 * MACROS
 */

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15
#define UART_PORT                         HAL_UART_PORT_1

/*********************************************************************
 * CONSTANTS
 */
// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  8   //最大搜索到的设备个数

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 4000

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      400

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           600

// Default passcode
#define DEFAULT_PASSCODE                      19655

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          FALSE
// Application states

#define USB_RX_BUF_LEN     100
#define USB_TX_BUF_LEN     100
static uint8 SerialApp_Buf[64]; //USB数据buf

enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_SVC,                 // Service discovery
  #if !defined(MULT_CHAR_DISC)
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
  #else
  BLE_DISC_STATE_CHAR1,
  BLE_DISC_STATE_CHAR2  
  #endif
};

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Task ID for internal task/event processing
static __xdata uint8 simpleBLETaskId;

// GAP GATT Attributes
static const uint8 simpleBLEDeviceName[GAP_DEVICE_NAME_LEN] = "HM502  TEST";

// Number of scan results and scan result index
static __xdata uint8 simpleBLEScanRes;
static __xdata uint8 simpleBLEScanIdx;

// Scan result list
static __xdata gapDevRec_t simpleBLEDevList[DEFAULT_MAX_SCAN_RES];

// Scanning state
static __xdata uint8 simpleBLEScanning = FALSE;

// RSSI polling state
static __xdata uint8 simpleBLERssi = FALSE;


#if defined (MULT_PERIPHERAL_CONN)
#define MAX_PERIPHERAL_NUM 2     //最大从设备连接个数
uint8 __xdata connectedPeripheralNum = 0;
#endif

// Connection handle of current connection 
#if defined (MULT_PERIPHERAL_CONN)
static __xdata uint16 simpleBLEConnHandle[MAX_PERIPHERAL_NUM] = {GAP_CONNHANDLE_INIT};
#else
static __xdata uint16 simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
#endif
// Application state
static __xdata uint8 simpleBLEState = BLE_STATE_IDLE;

// Discovery state
static __xdata uint8 simpleBLEDiscState = BLE_DISC_STATE_IDLE;

// Discovered service start and end handle
static __xdata uint16 simpleBLESvcStartHdl = 0;
static __xdata uint16 simpleBLESvcEndHdl = 0;

// Discovered characteristic handle
#if !defined(MULT_CHAR_DISC)
static __xdata uint16 simpleBLECharHdl = 0;
#else
static __xdata uint16 simpleBLECharHdl[MAX_PERIPHERAL_NUM] = {0};
#endif
// Value to write
static __xdata uint8 simpleBLECharVal = 0;

// Value read/write toggle
static __xdata bool simpleBLEDoWrite = FALSE;

// GATT read/write procedure state
static __xdata bool simpleBLEProcedureInProgress = FALSE;

static __xdata uint8 CommandFlag;   //命令接收标志

uint8 ECGSensor_TaskID; 

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg );
static void simpleBLECentralRssiCB( uint16 connHandle, int8  rssi );
static void simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent );
static void simpleBLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs );
static void simpleBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status );
static void simpleBLECentral_HandleKeys( uint8 shift, uint8 keys );
static void simpleBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg );
static void simpleBLECentralStartDiscovery( void );
static bool simpleBLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen );
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType );
static char *bdAddr2Str ( uint8 *pAddr );

static void usb_uart_rx_cback(uint8 port, uint8 event);
static void usb_uart_open(void);

/*****************************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static const gapCentralRoleCB_t simpleBLERoleCB =
{
  simpleBLECentralRssiCB,       // RSSI callback
  simpleBLECentralEventCB       // Event callback
};

// Bond Manager Callbacks
static const gapBondCBs_t simpleBLEBondCB =
{
  simpleBLECentralPasscodeCB,
  simpleBLECentralPairStateCB
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLECentral_Init
 *
 * @brief   Initialization function for the Simple BLE Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */

/******************************************************************************/
void  delay(unsigned int i)
{
  unsigned int x,y;
  for(x = 0; x <= 2000;x++)
    for(y = 0;y < i;y++);
}
/******************************************************************************/

/******************************************************************************/
void SimpleBLECentral_Init( uint8 task_id )
{
  simpleBLETaskId = task_id;

  // Setup Central Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPCentralRole_SetParameter ( GAPCENTRALROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
  }
  
  // Setup GAP
  GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (uint8 *) simpleBLEDeviceName );

  // Setup the GAP Bond Manager
  {
    uint32 passkey = DEFAULT_PASSCODE;
    uint8 pairMode = DEFAULT_PAIRING_MODE;
    uint8 mitm = DEFAULT_MITM_MODE;
    uint8 ioCap = DEFAULT_IO_CAPABILITIES;
    uint8 bonding = DEFAULT_BONDING_MODE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );
  }  

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd( simpleBLETaskId );

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes

  // Register for all key events - This app will handle all key events
  RegisterForKeys( simpleBLETaskId );
  
  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1), HAL_LED_MODE_ON ); 
  HalLedSet( (HAL_LED_2), HAL_LED_MODE_ON ); 
  // Setup a delayed profile startup
  osal_set_event( simpleBLETaskId, START_DEVICE_EVT );
  
  
  usb_uart_open();
  #ifdef UARTDEBUG
  memset(SerialApp_Buf,0,sizeof(SerialApp_Buf));
  strcpy(SerialApp_Buf,"\r\nBLECentral  Start............");
  HalUARTWrite(UART_PORT, SerialApp_Buf,sizeof(SerialApp_Buf));  
  #endif
 }
/******************************************************************************/


/******************************************************************************/
void StartNotifySet(void)
{
  attWriteReq_t req;
  uint8 status;
        
  req.handle = 0x3C;
  req.len = 2;
  req.value[0] = 0x01;//simpleBLECharVal;
  req.value[1] = 0x00;//simpleBLECharVal;//lance
  req.sig = 0;
  req.cmd = 0;
  status=GATT_WriteCharValue( simpleBLEConnHandle, &req, simpleBLETaskId ); 
}
/******************************************************************************/



/*********************************************************************
 * @fn      SimpleBLECentral_ProcessEvent
 *
 * @brief   Simple BLE Central Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLECentral_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  /******************************************************************************/
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLETaskId )) != NULL )
    {
      simpleBLECentral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  /******************************************************************************/
  if ( events & START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPCentralRole_StartDevice( (gapCentralRoleCB_t *) &simpleBLERoleCB );

    // Register with bond manager after starting device
    GAPBondMgr_Register( (gapBondCBs_t *) &simpleBLEBondCB );
    osal_start_timerEx(simpleBLETaskId, UARTSEND_EVT,DEFAULT_UARTSEND_PERIOD);   //串口发送定时器
    return ( events ^ START_DEVICE_EVT );
  }
  /******************************************************************************/
  if ( events & START_DISCOVERY_EVT )
  {
    simpleBLECentralStartDiscovery( );   
    return ( events ^ START_DISCOVERY_EVT );
  }
  /******************************************************************************/
  if ( events & UARTSEND_EVT )
  {
    if(CommandFlag ==  DEVICESTART)
    {
      CommandFlag = 0;
      #ifdef UARTDEBUG
      memset(SerialApp_Buf,0,sizeof(SerialApp_Buf));
      strcpy(SerialApp_Buf,"HM502B");   //start command
      HalUARTWrite(HAL_UART_PORT, SerialApp_Buf,sizeof(SerialApp_Buf));   
      #endif
    }
    osal_start_timerEx(simpleBLETaskId, UARTSEND_EVT,DEFAULT_UARTSEND_PERIOD);
    return ( events ^ UARTSEND_EVT );
  }
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLECentral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      simpleBLECentral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;

    case GATT_MSG_EVENT:
      simpleBLECentralProcessGATTMsg( (gattMsgEvent_t *) pMsg );
      break;
  }
}

/*********************************************************************
 * @fn      simpleBLECentral_HandleKeys
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
uint8 gStatus;
static void simpleBLECentral_HandleKeys( uint8 shift, uint8 keys )
{
  uint8 buffer[2];
  static  uint8 __xdata status = 0;
  
  (void)shift;  // Intentionally unreferenced parameter
/******************************************************************************/
  if ( keys & HAL_KEY_UP )
  {
    // Start or stop discovery
    #if defined (MULT_PERIPHERAL_CONN)
    if (connectedPeripheralNum < MAX_PERIPHERAL_NUM)
    #else
    if ( simpleBLEState != BLE_STATE_CONNECTED )
    #endif
    {
      if ( !simpleBLEScanning )
      {
        simpleBLEScanning = TRUE;
        simpleBLEScanRes = 0;
        
        GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                       DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                       DEFAULT_DISCOVERY_WHITE_LIST );              
        #ifdef UARTDEBUG
        memset(SerialApp_Buf,0,sizeof(SerialApp_Buf));
        strcpy(SerialApp_Buf,"Device Discovering......");       
        HalUARTWrite(HAL_UART_PORT, SerialApp_Buf,sizeof(SerialApp_Buf));   
        #endif
      }
      else
      {
        GAPCentralRole_CancelDiscovery();  
        #ifdef UARTDEBUG
        memset(SerialApp_Buf,0,sizeof(SerialApp_Buf));
        strcpy(SerialApp_Buf,"Cancel Discover");
        HalUARTWrite(HAL_UART_PORT, SerialApp_Buf,sizeof(SerialApp_Buf)); 
        #endif      
      }
      
    }
    else if ( simpleBLEState == BLE_STATE_CONNECTED &&
    #if !defined(MULT_CHAR_DISC)
    simpleBLECharHdl != 0 &&
    #else
    // Here only manipulate char1's value
    simpleBLECharHdl[0] != 0 &&
    #endif
    simpleBLEProcedureInProgress == FALSE )
    {
      StartNotifySet();   //启动notify功能
      #ifdef UARTDEBUG
      memset(SerialApp_Buf,0,sizeof(SerialApp_Buf));
      strcpy(SerialApp_Buf,"StartNotifySet......");
      HalUARTWrite(HAL_UART_PORT, SerialApp_Buf,sizeof(SerialApp_Buf));   
      #endif
      // Do a read or write as long as no other read or write is in progress
//      if ( simpleBLEDoWrite )
//      {
//        // Do a write
//        attWriteReq_t req;
//        
//        #if !defined(MULT_CHAR_DISC)
//        req.handle = simpleBLECharHdl;
//        #else
//        req.handle = simpleBLECharHdl[0];
//        #endif
//        req.len = 1;
//        req.value[0] = simpleBLECharVal;
//        req.sig = 0;
//        req.cmd = 0;
//        #if defined (MULT_PERIPHERAL_CONN)
//        status = GATT_WriteCharValue( simpleBLEConnHandle[0], &req, simpleBLETaskId );
//        #else
//        status = GATT_WriteCharValue( simpleBLEConnHandle, &req, simpleBLETaskId );         
//        #endif
//      }
//      else
//      {
//        // Do a read
//        attReadReq_t req;
//        
//        #if !defined(MULT_CHAR_DISC)
//        req.handle = simpleBLECharHdl;
//        #else
//        req.handle = simpleBLECharHdl[0];
//        #endif
//
//        #if defined (MULT_PERIPHERAL_CONN)
//        status = GATT_ReadCharValue( simpleBLEConnHandle[0], &req, simpleBLETaskId );
//        #else
//        status = GATT_ReadCharValue( simpleBLEConnHandle, &req, simpleBLETaskId );         
//        #endif
//      }
      if ( status == SUCCESS )
      {
        simpleBLEProcedureInProgress = TRUE;
        simpleBLEDoWrite = !simpleBLEDoWrite;
      }
    }    
  }
/******************************************************************************/
  if ( keys & HAL_KEY_LEFT )
  {
    // Display discovery results
    if ( !simpleBLEScanning && simpleBLEScanRes > 0 )
    {
        if ( simpleBLEScanIdx >= simpleBLEScanRes )
        {
          simpleBLEScanIdx = 0;
        }
          
        #ifdef UARTDEBUG
        memset(SerialApp_Buf,0,sizeof(SerialApp_Buf));
        strcpy(SerialApp_Buf,"Select Device to connect\r\nMAC: ");
        strcat(SerialApp_Buf,bdAddr2Str(simpleBLEDevList[simpleBLEScanIdx].addr));
        HalUARTWrite(HAL_UART_PORT, SerialApp_Buf,sizeof(SerialApp_Buf));
        #endif
    }
  }
/******************************************************************************/
  if ( keys & HAL_KEY_RIGHT )
  {
    // Connection update
    if ( simpleBLEState == BLE_STATE_CONNECTED )
    {
#if defined (MULT_PERIPHERAL_CONN)
      GAPCentralRole_UpdateLink( simpleBLEConnHandle[0],
#else
      GAPCentralRole_UpdateLink( simpleBLEConnHandle,
#endif
                                 DEFAULT_UPDATE_MIN_CONN_INTERVAL,
                                 DEFAULT_UPDATE_MAX_CONN_INTERVAL,
                                 DEFAULT_UPDATE_SLAVE_LATENCY,
                                 DEFAULT_UPDATE_CONN_TIMEOUT );
    }
  }
/******************************************************************************/  
  if ( keys & HAL_KEY_CENTER )
  {
    uint8 addrType;
    uint8 *peerAddr;
    
    // Connect or disconnect
    #if defined (MULT_PERIPHERAL_CONN)
    if ( simpleBLEState == BLE_STATE_IDLE 
        || connectedPeripheralNum < MAX_PERIPHERAL_NUM)
    #else
    if ( simpleBLEState == BLE_STATE_IDLE )
    #endif
    {
      // if there is a scan result
      if ( simpleBLEScanRes > 0 )
      {
        // connect to current device in scan result
        peerAddr = simpleBLEDevList[simpleBLEScanIdx].addr;
        addrType = simpleBLEDevList[simpleBLEScanIdx].addrType;
      
        simpleBLEState = BLE_STATE_CONNECTING;
        
        GAPCentralRole_EstablishLink( DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                      DEFAULT_LINK_WHITE_LIST,
                                      addrType, peerAddr );
        
        #ifdef UARTDEBUG
        memset(SerialApp_Buf,0,sizeof(SerialApp_Buf));
        strcpy(SerialApp_Buf,"Connect to  ");
        strcat(SerialApp_Buf,bdAddr2Str( peerAddr ));
        HalUARTWrite(HAL_UART_PORT, SerialApp_Buf,sizeof(SerialApp_Buf)); 
        #endif 
      } 
    }
    else if ( simpleBLEState == BLE_STATE_CONNECTING ||
              simpleBLEState == BLE_STATE_CONNECTED )
    {
      // disconnect
      simpleBLEState = BLE_STATE_DISCONNECTING;

      #if defined (MULT_PERIPHERAL_CONN)
      gStatus = GAPCentralRole_TerminateLink( simpleBLEConnHandle[0] );
      gStatus = GAPCentralRole_TerminateLink( simpleBLEConnHandle[1] );
      #else
      gStatus = GAPCentralRole_TerminateLink( simpleBLEConnHandle );
      #endif      
      #ifdef UARTDEBUG
      memset(SerialApp_Buf,0,sizeof(SerialApp_Buf));
      strcpy(SerialApp_Buf,"Disconnected");
      HalUARTWrite(HAL_UART_PORT, SerialApp_Buf,sizeof(SerialApp_Buf)); 
      #endif    
    }
  }
  if ( keys & HAL_KEY_DOWN )
  {
    // Start or cancel RSSI polling
    if ( simpleBLEState == BLE_STATE_CONNECTED )
    {
      if ( !simpleBLERssi )
      {
        simpleBLERssi = TRUE;
        #if defined (MULT_PERIPHERAL_CONN)
        GAPCentralRole_StartRssi( simpleBLEConnHandle[0], DEFAULT_RSSI_PERIOD );
        #else
        GAPCentralRole_StartRssi( simpleBLEConnHandle, DEFAULT_RSSI_PERIOD );
        #endif
      }
      else
      {
        simpleBLERssi = FALSE;
        #if defined (MULT_PERIPHERAL_CONN)
        GAPCentralRole_CancelRssi( simpleBLEConnHandle[0] );
        #else
        GAPCentralRole_CancelRssi( simpleBLEConnHandle );		
        #endif		       
      }
    }
  }
}

/*********************************************************************
 * @fn      simpleBLECentralProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg )  //GATT  读写 消息
{
  uint8 __xdata *dP1;
  uint8 __xdata *buffff;
  if ( simpleBLEState != BLE_STATE_CONNECTED )
  {
    // In case a GATT message came after a connection has dropped,
    // ignore the message
    return;
  }

  if ( ( pMsg->method == ATT_READ_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ ) ) )
  {
    if ( pMsg->method == ATT_ERROR_RSP )
    {
      uint8 status = pMsg->msg.errorRsp.errCode;
    }
    else
    {
      // After a successful read, display the read value
      uint8 valueRead = pMsg->msg.readRsp.value[0];      
      #ifdef UARTDEBUG
      HalUARTWrite(HAL_UART_PORT_0, "Reading Char.............", 20);  
      #endif        
    }
    
    simpleBLEProcedureInProgress = FALSE;
  }
  else if ( ( pMsg->method == ATT_WRITE_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ ) ) )
  {
    
    if ( pMsg->method == ATT_ERROR_RSP == ATT_ERROR_RSP )
    {
      uint8 status = pMsg->msg.errorRsp.errCode;      
    }
    else
    {
      
    }
    simpleBLEProcedureInProgress = FALSE;    
  }
  else if (pMsg->method == ATT_HANDLE_VALUE_NOTI)
  {
    uint8 valueReadSize = pMsg->msg.handleValueNoti.len; 
    {
      HalUARTWrite(HAL_UART_PORT, pMsg->msg.handleValueNoti.value, pMsg->msg.handleValueNoti.len); 
    }
  }
  else if ( simpleBLEDiscState != BLE_DISC_STATE_IDLE )
  {
    simpleBLEGATTDiscoveryEvent( pMsg );
  }
  
}

/*********************************************************************
 * @fn      simpleBLECentralRssiCB
 *
 * @brief   RSSI callback.
 *
 * @param   connHandle - connection handle
 * @param   rssi - RSSI
 *
 * @return  none
 */
static void simpleBLECentralRssiCB( uint16 connHandle, int8 rssi )
{
  uint8 buffer[4];
  rssi = 255-rssi+1;
  if(rssi<10)
  {
    buffer[0] = '0' + rssi%10;
    buffer[1] = 0;
  }
  else if(rssi<100)
  {
    buffer[0] = '0' + rssi/10;
    buffer[1] = '0' + rssi%10;
    buffer[2] = 0;
  }
  else
  {
    buffer[0] = '0' + rssi/100;
    buffer[1] = '0' + rssi%100/10;
    buffer[2] = '0' + rssi%10;
    buffer[3] = 0;
  }
  #ifdef UARTDEBUG
  memset(SerialApp_Buf,0,sizeof(SerialApp_Buf));
  strcpy(SerialApp_Buf,"HM502I  RSSI:  -");   //RSSI值
  strcat(SerialApp_Buf,buffer);
  HalUARTWrite(HAL_UART_PORT, SerialApp_Buf,sizeof(SerialApp_Buf)); 
  rssi++;
  #endif
#ifdef LCDDISPLAY  
    LCD_WRITE_STRING_VALUE( "RSSI -dB:", (uint8) (-rssi), 10, HAL_LCD_LINE_1 );
#endif
}

/*********************************************************************
 * @fn      simpleBLECentralEventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent )
{
  uint8 buffer[2];
  uint8 __xdata *dP1;
  uint8 __xdata *buffff;
  
  static  uint8 __xdata status = 0;
  
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:  
      {
        #ifdef UARTDEBUG
        memset(SerialApp_Buf,0,sizeof(SerialApp_Buf));
        strcpy(SerialApp_Buf,"BLE Central MAC: ");
        strcat(SerialApp_Buf,bdAddr2Str( pEvent->initDone.devAddr ) );
        HalUARTWrite(HAL_UART_PORT, SerialApp_Buf,sizeof(SerialApp_Buf)); 
        #endif 
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        // if filtering device discovery results based on service UUID
        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE )
        {
          if ( simpleBLEFindSvcUuid( SIMPLEPROFILE_SERV_UUID,
                                     pEvent->deviceInfo.pEvtData,
                                     pEvent->deviceInfo.dataLen ) )
          {
            simpleBLEScanning = FALSE;       
            GAPCentralRole_CancelDiscovery();
            simpleBLEAddDeviceInfo( pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType );
          }
        }
      }
      break;
      
    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        // discovery complete
        simpleBLEScanning = FALSE;
        // if not filtering device discovery results based on service UUID
        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE )
        {
          // Copy results
          simpleBLEScanRes = pEvent->discCmpl.numDevs;
          osal_memcpy( simpleBLEDevList, pEvent->discCmpl.pDevList,
                       (sizeof( gapDevRec_t ) * pEvent->discCmpl.numDevs) );
        }
        buffer[0] = '0'+ simpleBLEScanRes;
        buffer[1] = 0;
        #ifdef UARTDEBUG
        memset(SerialApp_Buf,0,sizeof(SerialApp_Buf));
        strcpy(SerialApp_Buf,"Find Device Number :  ");
        strcat(SerialApp_Buf,buffer);
        HalUARTWrite(HAL_UART_PORT, SerialApp_Buf,sizeof(SerialApp_Buf)); 
        #endif 
        
        if ( simpleBLEScanRes > 0 )
        {
          #ifdef UARTDEBUG
          delay(100);
          memset(SerialApp_Buf,0,sizeof(SerialApp_Buf));
          strcpy(SerialApp_Buf,"HM502S");   //搜索结果命令
          if(simpleBLEScanRes<=3)
          {
            for(uint8 i = 0;i<simpleBLEScanRes;i++)
            {
              strcat(SerialApp_Buf,bdAddr2Str(simpleBLEDevList[i].addr));
              strcat(SerialApp_Buf,"\r\n");
            }
            HalUARTWrite(HAL_UART_PORT, SerialApp_Buf,sizeof(SerialApp_Buf)); 
          }
          else
          {
            for(uint8 i = 0;i<3;i++)
            {
              strcat(SerialApp_Buf,bdAddr2Str(simpleBLEDevList[i].addr));
              strcat(SerialApp_Buf,"\r\n");
            }
            HalUARTWrite(HAL_UART_PORT, SerialApp_Buf,sizeof(SerialApp_Buf)); 
//            delay(1000);
//            memset(SerialApp_Buf,0,sizeof(SerialApp_Buf));
//            strcpy(SerialApp_Buf,"HM502s");   //搜索结果命令
//            for(uint8 i = 3;i<simpleBLEScanRes;i++)
//            {
//              strcat(SerialApp_Buf,bdAddr2Str(simpleBLEDevList[i].addr));
//              strcat(SerialApp_Buf,"\r\n");
//            }
//            HalUARTWrite(HAL_UART_PORT, SerialApp_Buf,sizeof(SerialApp_Buf)); 
          }
          #endif 
        }

        // initialize scan index to last device
        simpleBLEScanIdx = simpleBLEScanRes;

      }
      break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
        if ( pEvent->gap.hdr.status == SUCCESS )
        {          
          simpleBLEState = BLE_STATE_CONNECTED;
          #if defined (MULT_PERIPHERAL_CONN)
          simpleBLEConnHandle[connectedPeripheralNum] = pEvent->linkCmpl.connectionHandle;
          connectedPeripheralNum++;
          // Just for demo, we do not actually save both peripheral's characteristic values handles, 
          // after second peripheral is connected, clean simpleBLECharHdl, for second peripheral's service discovery
          simpleBLECharHdl[0] = 0;
          simpleBLECharHdl[1] = 0;
          #else
          simpleBLEConnHandle = pEvent->linkCmpl.connectionHandle;
          #endif
          simpleBLEProcedureInProgress = TRUE;    

          // If service discovery not performed initiate service discovery
          #if !defined(MULT_CHAR_DISC)
          if ( simpleBLECharHdl == 0 )
          #else
          // There is no characteristic records in the list, do the service discovery.
          if ( simpleBLECharHdl[0] == 0 )
          #endif
          {
            osal_start_timerEx( simpleBLETaskId, START_DISCOVERY_EVT, DEFAULT_SVC_DISCOVERY_DELAY );
          }
          
          #ifdef UARTDEBUG
          memset(SerialApp_Buf,0,sizeof(SerialApp_Buf));
          strcpy(SerialApp_Buf,"HM502N");   //notify命令使能
          strcat(SerialApp_Buf,"Connected\r\n");
          strcat(SerialApp_Buf,"MAC:  ");
          strcat(SerialApp_Buf,bdAddr2Str( pEvent->linkCmpl.devAddr ));
          HalUARTWrite(HAL_UART_PORT, SerialApp_Buf,sizeof(SerialApp_Buf)); 
          #endif 
        }
        else
        {
          simpleBLEState = BLE_STATE_IDLE;
          #if defined (MULT_PERIPHERAL_CONN)
          simpleBLEConnHandle[connectedPeripheralNum] = GAP_CONNHANDLE_INIT;
          #else
          simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
          #endif
          simpleBLERssi = FALSE;
          simpleBLEDiscState = BLE_DISC_STATE_IDLE;
          
          dP1 = (uint8 *) _ltoa(pEvent->gap.hdr.status, buffff, 10);
          #ifdef UARTDEBUG
          memset(SerialApp_Buf,0,sizeof(SerialApp_Buf));
          strcpy(SerialApp_Buf,"Connect Failed\r\n");  
          strcat(SerialApp_Buf,"Reason:  ");
          strcat(SerialApp_Buf,dP1);
          HalUARTWrite(HAL_UART_PORT, SerialApp_Buf,sizeof(SerialApp_Buf)); 
          #endif          
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        simpleBLEState = BLE_STATE_IDLE;
        #if defined (MULT_PERIPHERAL_CONN)
        connectedPeripheralNum--;
          // Make sure both links are disconnected.
        if (connectedPeripheralNum == 0)
        {
              simpleBLEConnHandle[0] = GAP_CONNHANDLE_INIT;
              simpleBLEConnHandle[1] = GAP_CONNHANDLE_INIT;
              simpleBLEDiscState = BLE_DISC_STATE_IDLE;
        }
        #else
        simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
        #endif
        simpleBLERssi = FALSE;
        #if !defined (MULT_PERIPHERAL_CONN)
        simpleBLEDiscState = BLE_DISC_STATE_IDLE;
        #endif

        #if !defined(MULT_CHAR_DISC)
        simpleBLECharHdl = 0;
        #else
        simpleBLECharHdl[0] = 0;
        simpleBLECharHdl[1] = 0;
        #endif
        
        simpleBLEProcedureInProgress = FALSE;
          
        dP1 = (uint8 *) _ltoa(pEvent->linkTerminate.reason, buffff, 10);       
        #ifdef UARTDEBUG
          memset(SerialApp_Buf,0,sizeof(SerialApp_Buf));
          
          strcpy(SerialApp_Buf,"HM502HDisconnected\r\n");     //断开连接
          strcat(SerialApp_Buf,"Reason:  ");
          strcat(SerialApp_Buf,dP1);
          HalUARTWrite(HAL_UART_PORT, SerialApp_Buf,sizeof(SerialApp_Buf)); 
        #endif 
      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {       
        #ifdef UARTDEBUG
        memset(SerialApp_Buf,0,sizeof(SerialApp_Buf));
        strcpy(SerialApp_Buf,"Param Update");  
        HalUARTWrite(HAL_UART_PORT, SerialApp_Buf,sizeof(SerialApp_Buf)); 
        #endif 
      }
      break;
      
    default:
      break;
  }
}

/*********************************************************************
 * @fn      pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void simpleBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
  if ( state == GAPBOND_PAIRING_STATE_STARTED )
  {

  }
  else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )
  {
    if ( status == SUCCESS )
    {

    }
    else
    {

    }
  }
  else if ( state == GAPBOND_PAIRING_STATE_BONDED )
  {
    if ( status == SUCCESS )
    {

    }
  }
}

/*********************************************************************
 * @fn      simpleBLECentralPasscodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void simpleBLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs )
{
#if (HAL_LCD == TRUE)

  uint32  passcode;
  uint8    str[7];

  // Create random passcode
  LL_Rand( ((uint8 *) &passcode), sizeof( uint32 ));
  passcode %= 1000000;
  
  // Display passcode to user
  if ( uiOutputs != 0 )
  {

  }
  
  // Send passcode response
  GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, passcode );
#endif
}

/*********************************************************************
 * @fn      simpleBLECentralStartDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void simpleBLECentralStartDiscovery( void )
{
  uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID),
                                   HI_UINT16(SIMPLEPROFILE_SERV_UUID) }; 
  // Initialize cached handles

  #if !defined(MULT_CHAR_DISC)
  simpleBLESvcStartHdl = simpleBLESvcEndHdl = simpleBLECharHdl = 0;
  #else
  simpleBLESvcStartHdl = simpleBLESvcEndHdl = 0;
  #endif
  simpleBLEDiscState = BLE_DISC_STATE_SVC;
  
  // Discovery simple BLE service
  #if defined (MULT_PERIPHERAL_CONN)
  GATT_DiscPrimaryServiceByUUID( simpleBLEConnHandle[connectedPeripheralNum - 1],
  #else
  GATT_DiscPrimaryServiceByUUID( simpleBLEConnHandle,
  #endif   
                                 uuid,
                                 ATT_BT_UUID_SIZE,
                                 simpleBLETaskId );

}

/*********************************************************************
 * @fn      simpleBLEGATTDiscoveryEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */

#if !defined (MULT_CHAR_DISC)
static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg )
{
  static unsigned  char i[1] = {0x01};
  attReadByTypeReq_t req;
  
  i[0]++;
  if ( simpleBLEDiscState == BLE_DISC_STATE_SVC )
  {           
    // Service found, store handles
    if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
         pMsg->msg.findByTypeValueRsp.numInfo > 0 )
    {
      simpleBLESvcStartHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].handle;
      simpleBLESvcEndHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;
    }
    
    // If procedure complete
    if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
           pMsg->hdr.status == bleProcedureComplete ) ||
         ( pMsg->method == ATT_ERROR_RSP ) )
    {
      if ( simpleBLESvcStartHdl != 0 )
      {
        // Discover characteristic
        simpleBLEDiscState = BLE_DISC_STATE_CHAR;
        
        req.startHandle = simpleBLESvcStartHdl;
        req.endHandle = simpleBLESvcEndHdl;
        req.type.len = ATT_BT_UUID_SIZE;
        req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);
        req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);

        GATT_ReadUsingCharUUID( simpleBLEConnHandle, &req, simpleBLETaskId );
      }
    }
  }
  else if ( simpleBLEDiscState == BLE_DISC_STATE_CHAR )
  {
    // Characteristic found, store handle
    if ( pMsg->method == ATT_READ_BY_TYPE_RSP && 
         pMsg->msg.readByTypeRsp.numPairs > 0 )
    {
      simpleBLECharHdl = BUILD_UINT16( pMsg->msg.readByTypeRsp.dataList[0],
                                       pMsg->msg.readByTypeRsp.dataList[1] );
      
      #ifdef UARTDEBUG
      memset(SerialApp_Buf,0,sizeof(SerialApp_Buf));
      strcpy(SerialApp_Buf,"Simple Svc Found");  
      HalUARTWrite(HAL_UART_PORT, SerialApp_Buf,sizeof(SerialApp_Buf)); 
      #endif 
      simpleBLEProcedureInProgress = FALSE;
    }     
    simpleBLEDiscState = BLE_DISC_STATE_IDLE;    
  }    
}

#else
static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg )
{ 
  attReadByTypeReq_t req;  
   if ( simpleBLEDiscState == BLE_DISC_STATE_SVC )   
   {    
      // Service found, store handles    
      if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&       
       pMsg->msg.findByTypeValueRsp.numInfo > 0 )   
      {      
        simpleBLESvcStartHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].handle;  
        simpleBLESvcEndHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;    
       }    
  
      // If procedure complete    
      if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  &&           
      pMsg->hdr.status == bleProcedureComplete ) ||         
      ( pMsg->method == ATT_ERROR_RSP ) )   
      {      
        if ( simpleBLESvcStartHdl != 0 )      
        {        
          // Discover characteristic       
          simpleBLEDiscState = BLE_DISC_STATE_CHAR1;      
          req.startHandle = simpleBLESvcStartHdl;    
          req.endHandle = simpleBLESvcEndHdl;    
          req.type.len = ATT_BT_UUID_SIZE;
          req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);    
          req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);      
          #if defined (MULT_PERIPHERAL_CONN)
          GATT_ReadUsingCharUUID( simpleBLEConnHandle[connectedPeripheralNum - 1], &req, simpleBLETaskId );     
          #else
          GATT_ReadUsingCharUUID( simpleBLEConnHandle, &req, simpleBLETaskId );     
          #endif
        }    
      }  
  }  
  else if ( simpleBLEDiscState == BLE_DISC_STATE_CHAR1 )  
  {    
    // Characteristic found, store handle    
     if ( pMsg->method == ATT_READ_BY_TYPE_RSP &&      
        pMsg->msg.readByTypeRsp.numPairs > 0 )
        {      
           simpleBLECharHdl[0] = BUILD_UINT16( pMsg->msg.readByTypeRsp.dataList[0],                         
           pMsg->msg.readByTypeRsp.dataList[1] );      
    
           LCD_WRITE_STRING( "CHAR 1 Found", HAL_LCD_LINE_1 );      
           simpleBLEProcedureInProgress = TRUE;      
       }
     else // pMsg->msg.readByTypeRsp.numPairs is 0.
     {
       simpleBLEDiscState = BLE_DISC_STATE_CHAR2;    
       req.startHandle = simpleBLESvcStartHdl;  
       req.endHandle = simpleBLESvcEndHdl;   
       req.type.len = ATT_BT_UUID_SIZE;    
       req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR2_UUID);   
       req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR2_UUID);    
        #if defined (MULT_PERIPHERAL_CONN)
       GATT_ReadUsingCharUUID( simpleBLEConnHandle[connectedPeripheralNum - 1], &req, simpleBLETaskId );  
        #else
       GATT_ReadUsingCharUUID( simpleBLEConnHandle, &req, simpleBLETaskId );  
        #endif
     }
  }  
  
  else if (simpleBLEDiscState == BLE_DISC_STATE_CHAR2)  
  {     // Characteristic found, store handle    
    if ( pMsg->method == ATT_READ_BY_TYPE_RSP &&     
    pMsg->msg.readByTypeRsp.numPairs > 0 )    
    {      
      simpleBLECharHdl[1] = BUILD_UINT16( pMsg->msg.readByTypeRsp.dataList[0],                                      
      pMsg->msg.readByTypeRsp.dataList[1] );     
      LCD_WRITE_STRING( "CHAR 2 Found", HAL_LCD_LINE_2 );      
      simpleBLEProcedureInProgress = FALSE;  
    }    
    simpleBLEDiscState = BLE_DISC_STATE_IDLE;
  }
}
#endif
/*********************************************************************
 * @fn      simpleBLEFindSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool simpleBLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen )
{
  uint8 adLen;
  uint8 adType;
  uint8 *pEnd;
  
  pEnd = pData + dataLen - 1;
  
  // While end of data not reached
  while ( pData < pEnd )
  {
    // Get length of next AD item
    adLen = *pData++;
    if ( adLen > 0 )
    {
      adType = *pData;
      
      // If AD type is for 16-bit service UUID
      if ( adType == GAP_ADTYPE_16BIT_MORE || adType == GAP_ADTYPE_16BIT_COMPLETE )
      {
        pData++;
        adLen--;
        
        // For each UUID in list
        while ( adLen >= 2 && pData < pEnd )
        {
          // Check for match
          if ( pData[0] == LO_UINT16(uuid) && pData[1] == HI_UINT16(uuid) )
          {
            // Match found
            return TRUE;
          }
          
          // Go to next
          pData += 2;
          adLen -= 2;
        }
        
        // Handle possible erroneous extra byte in UUID list
        if ( adLen == 1 )
        {
          pData++;
        }
      }
      else
      {
        // Go to next item
        pData += adLen;
      }
    }
  }
  
  // Match not found
  return FALSE;
}

/*********************************************************************
 * @fn      simpleBLEAddDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType )
{
  uint8 i;
  
  // If result count not at max
  if ( simpleBLEScanRes < DEFAULT_MAX_SCAN_RES )
  {
    // Check if device is already in scan results
    for ( i = 0; i < simpleBLEScanRes; i++ )
    {
      if ( osal_memcmp( pAddr, simpleBLEDevList[i].addr , B_ADDR_LEN ) )
      {
        return;
      }
    }
    
    // Add addr to scan result list
    osal_memcpy( simpleBLEDevList[simpleBLEScanRes].addr, pAddr, B_ADDR_LEN );
    simpleBLEDevList[simpleBLEScanRes].addrType = addrType;
    
    // Increment scan result count
    simpleBLEScanRes++;
  }
}

/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
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

/******************************************************************************/
void usb_uart_open(void)
{
  halUARTCfg_t uartConfig;
  
  uartConfig.configured           = TRUE;              
  uartConfig.baudRate             = HAL_UART_BR_115200;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = 48;
  uartConfig.rx.maxBufSize        = USB_RX_BUF_LEN;
  uartConfig.tx.maxBufSize        = USB_TX_BUF_LEN;
  
  uartConfig.idleTimeout          = 100;   
  uartConfig.intEnable            = TRUE;              
  uartConfig.callBackFunc         = (halUARTCBack_t)&usb_uart_rx_cback;
  
  HalUARTOpen (UART_PORT, &uartConfig);
} 
/******************************************************************************/
void usb_uart_rx_cback(uint8 port, uint8 event)
{
  static uint8 cmdData = 0;
  uint8 SerialApp_TxLen;
  uint8 usbEvent = event;
  switch(usbEvent)
  {
  case HAL_UART_RX_FULL:
    
  case HAL_UART_RX_ABOUT_FULL:
    
  case HAL_UART_RX_TIMEOUT:
    delay(1);//防止不丢数据
    SerialApp_TxLen = HalUARTRead(HAL_UART_PORT, SerialApp_Buf, 50);
    if(SerialApp_TxLen)
    {     
      if(('H' == SerialApp_Buf[0] )&&('M' == SerialApp_Buf[1]) &&('5' == SerialApp_Buf[2])&&('0' == SerialApp_Buf[3])&&('2' == SerialApp_Buf[4]))
      {
        if('A' == SerialApp_Buf[5])    //搜索设备
        {
//          #if defined (MULT_PERIPHERAL_CONN)
//          if (connectedPeripheralNum < MAX_PERIPHERAL_NUM)
//          #else
//          if ( simpleBLEState != BLE_STATE_CONNECTED )
//          #endif
          {
            cmdData = HAL_KEY_UP;
            simpleBLECentral_HandleKeys( 1, cmdData);
          }
        }
        else if('B' == SerialApp_Buf[5])       //选择设备
        {
          simpleBLEScanIdx = SerialApp_Buf[6];
          cmdData = HAL_KEY_LEFT;
          simpleBLECentral_HandleKeys( 1, cmdData);
        }
        else if('C' == SerialApp_Buf[5])
        {
          cmdData = HAL_KEY_CENTER;
          simpleBLECentral_HandleKeys( 1, cmdData);
        }
        else if('D' == SerialApp_Buf[5])
        {
          cmdData = HAL_KEY_RIGHT;
        }
        else if('E' == SerialApp_Buf[5])
        {
          cmdData = HAL_KEY_DOWN;
          simpleBLECentral_HandleKeys( 1, cmdData);
        }
        else if('F' == SerialApp_Buf[5])   //串口开始连接命令
        {
          CommandFlag = DEVICESTART;
        }
        else if('G' == SerialApp_Buf[5])   //关闭连接的设备
        {
          if ( simpleBLEState == BLE_STATE_CONNECTING ||
              simpleBLEState == BLE_STATE_CONNECTED )
          {
            cmdData = HAL_KEY_CENTER;
          }
        }
      }
      
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
/******************************************************************************/