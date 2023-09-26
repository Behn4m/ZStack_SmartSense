/**************************************************************************************************
  Filename:       zcl_LIGHT.c
  Revised:        $Date: 2014-10-24 16:04:46 -0700 (Fri, 24 Oct 2014) $
  Revision:       $Revision: 40796 $


  Description:    Zigbee Cluster Library - sample device application.


  Copyright 2006-2014 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
  This application implements a ZigBee HA 1.2 Light. It can be configured as an
  On/Off light, or as a dimmable light. The following flags must be defined in
  the compiler's pre-defined symbols.

  ZCL_ON_OFF
  ZCL_LEVEL_CTRL    (only if dimming functionality desired)
  HOLD_AUTO_START
  ZCL_EZMODE

  This device supports all mandatory and optional commands/attributes for the
  OnOff (0x0006) and LevelControl (0x0008) clusters.

  SCREEN MODES
  ----------------------------------------
  Main:
    - SW1: Toggle local light
    - SW2: Invoke EZMode
    - SW4: Enable/Disable local permit join
    - SW5: Go to Help screen
  ----------------------------------------
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "MT_SYS.h"

#include "nwk_util.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_ezmode.h"
#include "zcl_diagnostic.h"

#include "zcl_SmartSense.h"

#include "onboard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "SHT2x.h"

/*mine*/
#include "zcl_ms.h"
#include "hal_adc.h"
#include "aps_groups.h"   
#include <stdio.h>

/*************************/   

#if ( defined (ZGP_DEVICE_TARGET) || defined (ZGP_DEVICE_TARGETPLUS) \
      || defined (ZGP_DEVICE_COMBO) || defined (ZGP_DEVICE_COMBO_MIN) )
#include "zgp_translationtable.h"
  #if (SUPPORTED_S_FEATURE(SUPP_ZGP_FEATURE_TRANSLATION_TABLE))
    #define ZGP_AUTO_TT
  #endif
#endif

#if (defined HAL_BOARD_ZLIGHT) || (defined HAL_PWM)
#include "math.h"
#include "hal_timer.h"
#endif

#include "NLMEDE.h"




/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#if (defined HAL_BOARD_ZLIGHT)
#define LEVEL_MAX                 0xFE
#define LEVEL_MIN                 0x0
#define GAMMA_VALUE               2
#define PWM_FULL_DUTY_CYCLE       1000
#elif (defined HAL_PWM)
#define LEVEL_MAX                 0xFE
#define LEVEL_MIN                 0x0
#define GAMMA_VALUE               2
#define PWM_FULL_DUTY_CYCLE       100
#endif

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
byte zclSmartSense_TaskID;
uint8 zclSmartSenseSeqNum;


/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 temp; //mine
uint16 adc_value;//mine

uint8 error =0;//sht20
nt16 sT; //variable for raw temperature ticks
float temperatureC; //variable for temperature[°C] as float
float humidityRH; //variable for relative humidity as float

uint8 Mv_Cnt;//mine
bool PIR_flag = TRUE;//mine
uint8 zclOccupancySensor_LastOccupancy = 0x00;//mine

afAddrType_t zclLightSensor_DstAddr;/*mine*/
afAddrType_t zclOccupancySensor_DstAddr;/*mine*/
afAddrType_t zclTemperatureSensor_DstAddr;/*mine*/
afAddrType_t zclRHumiditySensor_DstAddr;/*mine*/

aps_Group_t light_group1; /*mine*/
aps_Group_t light_group2; /*mine*/

//!!!

#ifdef ZCL_EZMODE
static void zclLight1_ProcessZDOMsgs( zdoIncomingMsg_t *pMsg );
static void zclLight1_EZModeCB( zlcEZMode_State_t state, zclEZMode_CBData_t *pData );


// register EZ-Mode with task information (timeout events, callback, etc...)
static const zclEZMode_RegisterData_t zclLight1_RegisterEZModeData =
{
  &zclSmartSense_TaskID,
  LIGHT_EZMODE_NEXTSTATE_EVT,
  LIGHT_EZMODE_TIMEOUT_EVT,
  &zclSmartSenseSeqNum,
  zclLight1_EZModeCB
};

#else
uint16 bindingInClusters[] =
{
  ZCL_CLUSTER_ID_GEN_ON_OFF
#ifdef ZCL_LEVEL_CTRL
  , ZCL_CLUSTER_ID_GEN_LEVEL_CONTROL
#endif
};
#define zclLight1_BINDINGLIST (sizeof(bindingInClusters) / sizeof(bindingInClusters[0]))

#endif  // ZCL_EZMODE


uint8 giLightScreenMode = LIGHT_MAINMODE;   // display the main screen mode first

uint8 gPermitDuration = 0;    // permit joining default to disabled

devStates_t zclLight1_NwkState = DEV_INIT;



/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void zclLight_HandleKeys( byte shift, byte keys );
static void zclSmartSense_ReadSensors(void);//mine
static void zclSmartSense_CheckPIR(void);//mine
static void zclSmartSense_PIR_SenseMv(void);//mine
static void zclSendReport(byte SensedValue);//mine


static void zclLight1_BasicResetCB( void );
static void zclLight1_IdentifyCB( zclIdentify_t *pCmd );
static void zclLight1_IdentifyQueryRspCB( zclIdentifyQueryRsp_t *pRsp );
static void zclLight1_ProcessIdentifyTimeChange( void );


// Functions to process ZCL Foundation incoming Command/Response messages
static void zclLight1_ProcessIncomingMsg( zclIncomingMsg_t *msg );
#ifdef ZCL_READ
static uint8 zclLight1_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclLight1_ProcessInReadCmd( zclIncomingMsg_t *pInMsg );//mine
#endif
#ifdef ZCL_WRITE
static uint8 zclLight1_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg );
#endif
static uint8 zclLight1_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg );
#ifdef ZCL_DISCOVER
static uint8 zclLight1_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclLight1_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclLight1_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg );
#endif



/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclLight1_CmdCallbacks =
{
  zclLight1_BasicResetCB,            // Basic Cluster Reset command
  zclLight1_IdentifyCB,              // Identify command
#ifdef ZCL_EZMODE
  NULL,                                   // Identify EZ-Mode Invoke command
  NULL,                                   // Identify Update Commission State command
#endif
  NULL,                                   // Identify Trigger Effect command
  zclLight1_IdentifyQueryRspCB,      // Identify Query Response command
  NULL,                 // On/Off cluster commands
  NULL,                                   // On/Off cluster enhanced command Off with Effect
  NULL,                                   // On/Off cluster enhanced command On with Recall Global Scene
  NULL,                                   // On/Off cluster enhanced command On with Timed Off
#ifdef ZCL_GROUPS
  NULL,                                   // Group Response commands
#endif
#ifdef ZCL_SCENES
  NULL,                                  // Scene Store Request command
  NULL,                                  // Scene Recall Request command
  NULL,                                  // Scene Response command
#endif
#ifdef ZCL_ALARMS
  NULL,                                  // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,                                  // Get Event Log command
  NULL,                                  // Publish Event Log command
#endif
  NULL,                                  // RSSI Location command
  NULL                                   // RSSI Location Response command
};

/*********************************************************************
 * @fn          zclLight_Init
 *
 * @brief       Initialization function for the zclGeneral layer.
 *
 * @param       none
 *
 * @return      none
 */
void zclSmartSense_Init( byte task_id )
{
  zclSmartSense_TaskID = task_id;


  zclLightSensor_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  zclLightSensor_DstAddr.endPoint = 8;
  zclLightSensor_DstAddr.addr.shortAddr = 0;
  
  zclOccupancySensor_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  zclOccupancySensor_DstAddr.endPoint = 8;
  zclOccupancySensor_DstAddr.addr.shortAddr = 0;
  
  zclTemperatureSensor_DstAddr.addrMode=(afAddrMode_t)Addr16Bit;/*mine*/
  zclTemperatureSensor_DstAddr.endPoint=8;
  zclTemperatureSensor_DstAddr.addr.shortAddr=0;
        
  zclRHumiditySensor_DstAddr.addrMode=(afAddrMode_t)Addr16Bit;/*mine*/
  zclRHumiditySensor_DstAddr.endPoint=8;/*mine*/
  zclRHumiditySensor_DstAddr.addr.shortAddr=0;/*mine*/
  
  
  /*******************************/
  
  // This app is part of the Home Automation Profile
  zclHA_Init( &zclLightSensor_SimpleDesc );/*mine*/ 
  zclHA_Init( &zclOccupancySensor_SimpleDesc );/*mine*/ 
  zclHA_Init( &zclTemperatureSensor_SimpleDesc );/*mine*/ 
  zclHA_Init( &zclRHumiditySensor_SimpleDesc );/*mine*/ 
  
  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( LIGHT1_ENDPOINT, &zclLight1_CmdCallbacks );

  // Register the application's attribute list 
  zcl_registerAttrList( OCCUPANCYSENSOR_ENDPOINT, zclOccupancySensor_NumAttributes, zclLightSensor_Attrs );/*mine*/ 
  zcl_registerAttrList( LIGHTSENSOR_ENDPOINT, zclLightSensor_NumAttributes, zclOccupancySensor_Attrs );/*mine*/ 
  zcl_registerAttrList( TEMPERATURESENSOR_ENDPOINT, zclTempratureSensor_NumAttributes, zclTempratureSensor_Attrs );/*mine*/
  zcl_registerAttrList( RHUMIDITYSENSOR_ENDPOINT, zclRHumiditySensor_NumAttributes, zclRHumiditySensor_Attrs );/*mine*/   
  // Register the Application to receive the unprocessed Foundation command/response messages
  zcl_registerForMsg( zclSmartSense_TaskID );

#ifdef ZCL_DISCOVER
  // Register the application's command list
  zcl_registerCmdList( OCCUPANCYSENSOR_ENDPOINT, zclOccupancySensorCmdsArraySize, zclOccupancySensor_Cmds );
  zcl_registerCmdList( LIGHTSENSOR_ENDPOINT, zclLightSensorCmdsArraySize, zclLightSensor_Cmds );
  zcl_registerCmdList( TEMPERATURESENSOR_ENDPOINT, zclTemperatureSensorCmdsArraySize, zclTemperatureSensor_Cmds );
  zcl_registerCmdList( RHUMIDITYSENSOR_ENDPOINT, zclRHumiditySensorCmdsArraySize, zclRHumiditySensor_Cmds );
#endif

  // Register for all key events - This app will handle all key events
  RegisterForKeys( zclSmartSense_TaskID );

#ifdef ZCL_EZMODE
  // Register EZ-Mode
  zcl_RegisterEZMode( &zclLight1_RegisterEZModeData );

  // Register with the ZDO to receive Match Descriptor Responses
  ZDO_RegisterForZDOMsg(task_id, Match_Desc_rsp);
#endif


#ifdef ZCL_DIAGNOSTIC
  // Register the application's callback function to read/write attribute data.
  // This is only required when the attribute data format is unknown to ZCL.
  zcl_registerReadWriteCB( LIGHT1_ENDPOINT, zclDiagnostic_ReadWriteAttrCB, NULL );

  if ( zclDiagnostic_InitStats() == ZSuccess )
  {
    // Here the user could start the timer to save Diagnostics to NV
  }
#endif


#ifdef ZGP_AUTO_TT
  zgpTranslationTable_RegisterEP ( &zclLight1_SimpleDesc );
#endif
  
  
  //farhad  
    error |= SHT2x_SoftReset(); // soft reset sht20
    uint8 buf1[30]="SHT2x_SoftReset\n\r";
    HalUARTWrite(HAL_UART_PORT_0,buf1,19);

//  HalLedBlink(HAL_LED_Identify, 0,50,500);// blink led - CPU heartbeat      
  
  // start a reload timer.On timer OVF we check ADC for measuring Battery Voltage
  //osal_start_reload_timer( zclSmartSense_TaskID,LIGHT_ADC_TIMER_EVT,5000); 
  osal_start_timerEx( zclSmartSense_TaskID, SENSORS_ADC_TIMER_EVT,15000);  

}

/*********************************************************************
 * @fn          zclSample_event_loop
 *
 * @brief       Event Loop Processor for zclGeneral.
 *
 * @param       none
 *
 * @return      none
 */
uint16 zclSmartSense_event_loop( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;

  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( zclSmartSense_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      {
#ifdef ZCL_EZMODE
        case ZDO_CB_MSG:
          zclLight1_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;
#endif
        case ZCL_INCOMING_MSG:
          // Incoming ZCL Foundation command/response messages
          zclLight1_ProcessIncomingMsg( (zclIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          zclLight_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case ZDO_STATE_CHANGE:
          zclLight1_NwkState = (devStates_t)(MSGpkt->hdr.status);

          // now on the network
          if ( (zclLight1_NwkState == DEV_ZB_COORD) ||
               (zclLight1_NwkState == DEV_ROUTER)   ||
               (zclLight1_NwkState == DEV_END_DEVICE) )
          {
            giLightScreenMode = LIGHT_MAINMODE;
#ifdef ZCL_EZMODE
            zcl_EZModeAction( EZMODE_ACTION_NETWORK_STARTED, NULL );
#endif // ZCL_EZMODE
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & LIGHT_IDENTIFY_TIMEOUT_EVT )
  {
    if ( zclLight1_IdentifyTime > 0 )
      zclLight1_IdentifyTime--;
    zclLight1_ProcessIdentifyTimeChange();

    return ( events ^ LIGHT_IDENTIFY_TIMEOUT_EVT );
  }

  if ( events & LIGHT_MAIN_SCREEN_EVT )
  {


    return ( events ^ LIGHT_MAIN_SCREEN_EVT );
  }

#ifdef ZCL_EZMODE
#if (defined HAL_BOARD_ZLIGHT)
  // event to start EZMode on startup with a delay
  if ( events & LIGHT_START_EZMODE_EVT )
  {
    // Invoke EZ-Mode
    zclEZMode_InvokeData_t ezModeData;

    // Invoke EZ-Mode
    ezModeData.endpoint = LIGHT1_ENDPOINT; // endpoint on which to invoke EZ-Mode
    if ( (zclLight1_NwkState == DEV_ZB_COORD) ||
         (zclLight1_NwkState == DEV_ROUTER)   ||
         (zclLight1_NwkState == DEV_END_DEVICE) )
    {
      ezModeData.onNetwork = TRUE;      // node is already on the network
    }
    else
    {
      ezModeData.onNetwork = FALSE;     // node is not yet on the network
    }
    ezModeData.initiator = FALSE;          // OnOffLight is a target
    ezModeData.numActiveOutClusters = 0;
    ezModeData.pActiveOutClusterIDs = NULL;
    ezModeData.numActiveInClusters = 0;
    ezModeData.pActiveOutClusterIDs = NULL;
    zcl_InvokeEZMode( &ezModeData );

    return ( events ^ LIGHT_START_EZMODE_EVT );
  }
#endif // #if (defined HAL_BOARD_ZLIGHT)

  // going on to next state
  if ( events & LIGHT_EZMODE_NEXTSTATE_EVT )
  {
    zcl_EZModeAction ( EZMODE_ACTION_PROCESS, NULL );   // going on to next state
    return ( events ^ LIGHT_EZMODE_NEXTSTATE_EVT );
  }

  // the overall EZMode timer expired, so we timed out
  if ( events & LIGHT_EZMODE_TIMEOUT_EVT )
  {
    zcl_EZModeAction ( EZMODE_ACTION_TIMED_OUT, NULL ); // EZ-Mode timed out
    return ( events ^ LIGHT_EZMODE_TIMEOUT_EVT );
  }
#endif // ZLC_EZMODE

#ifdef ZCL_LEVEL_CTRL
  if ( events & LIGHT_LEVEL_CTRL_EVT )
  {
    zclLight1_AdjustLightLevel();
    return ( events ^ LIGHT_LEVEL_CTRL_EVT );
  }
#endif
  
  /***********************mine***********************************/  
  
  if ( events & SENSORS_ADC_TIMER_EVT )
  {
    osal_stop_timerEx( zclSmartSense_TaskID, SENSORS_ADC_TIMER_EVT);    
    zclSmartSense_ReadSensors(); 
    return ( events ^ SENSORS_ADC_TIMER_EVT );
  }
  
  
  if ( events & zclOccupancySensor_UtoO_TIMER_EVT )
  {
    zclSmartSense_CheckPIR(); 
    return ( events ^ zclOccupancySensor_UtoO_TIMER_EVT );
  }
    

  if ( events & zclOccupancySensor_OtoU_TIMER_EVT )
  {
    zclSmartSense_CheckPIR();    
    return ( events ^ zclOccupancySensor_OtoU_TIMER_EVT );
  }  

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      zclLight_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_5
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void zclLight_HandleKeys( byte shift, byte keys )
{
  if ( keys & HAL_KEY_SW_1 )
  {
    zclSmartSense_PIR_SenseMv();
  }

  if ( keys & HAL_KEY_SW_2 )
  {
    //ZDOInitDevice( 0 );//farhad
  }

}

/*********************************************************************
 * @fn      zclLight1_ProcessIdentifyTimeChange
 *
 * @brief   Called to process any change to the IdentifyTime attribute.
 *
 * @param   none
 *
 * @return  none
 */
static void zclLight1_ProcessIdentifyTimeChange( void )
{
  if ( zclLight1_IdentifyTime > 0 )
  {
    osal_start_timerEx( zclSmartSense_TaskID, LIGHT_IDENTIFY_TIMEOUT_EVT, 1000 );
    //HalLedBlink ( HAL_LED_Identify, 0xFF, HAL_LED_DEFAULT_DUTY_CYCLE, HAL_LED_DEFAULT_FLASH_TIME );
  }
  else
  {
#ifdef ZCL_EZMODE
    if ( zclLight1_IdentifyCommissionState & EZMODE_COMMISSION_OPERATIONAL )
    {
      HalLedSet ( HAL_LED_Identify, HAL_LED_MODE_ON );
    }
    else
    {
      HalLedSet ( HAL_LED_Identify, HAL_LED_MODE_OFF );
    }
#endif

    osal_stop_timerEx( zclSmartSense_TaskID, LIGHT_IDENTIFY_TIMEOUT_EVT );
  }
}

/*********************************************************************
 * @fn      zclLight1_BasicResetCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to set all the Basic Cluster attributes to default values.
 *
 * @param   none
 *
 * @return  none
 */
static void zclLight1_BasicResetCB( void )
{
  NLME_LeaveReq_t leaveReq;
  // Set every field to 0
  osal_memset( &leaveReq, 0, sizeof( NLME_LeaveReq_t ) );

  // This will enable the device to rejoin the network after reset.
  leaveReq.rejoin = TRUE;

  // Set the NV startup option to force a "new" join.
  zgWriteStartupOptions( ZG_STARTUP_SET, ZCD_STARTOPT_DEFAULT_NETWORK_STATE );

  // Leave the network, and reset afterwards
  if ( NLME_LeaveReq( &leaveReq ) != ZSuccess )
  {
    // Couldn't send out leave; prepare to reset anyway
    ZDApp_LeaveReset( FALSE );
  }
}

/*********************************************************************
 * @fn      zclLight1_IdentifyCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Command for this application.
 *
 * @param   srcAddr - source address and endpoint of the response message
 * @param   identifyTime - the number of seconds to identify yourself
 *
 * @return  none
 */
static void zclLight1_IdentifyCB( zclIdentify_t *pCmd )
{
  zclLight1_IdentifyTime = pCmd->identifyTime;
  zclLight1_ProcessIdentifyTimeChange();
}

/*********************************************************************
 * @fn      zclLight1_IdentifyQueryRspCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Query Response Command for this application.
 *
 * @param   srcAddr - requestor's address
 * @param   timeout - number of seconds to identify yourself (valid for query response)
 *
 * @return  none
 */
static void zclLight1_IdentifyQueryRspCB(  zclIdentifyQueryRsp_t *pRsp )
{
  (void)pRsp;
#ifdef ZCL_EZMODE
  {
    zclEZMode_ActionData_t data;
    data.pIdentifyQueryRsp = pRsp;
    zcl_EZModeAction ( EZMODE_ACTION_IDENTIFY_QUERY_RSP, &data );
  }
#endif
}

/******************************************************************************
 *
 *  Functions for processing ZCL Foundation incoming Command/Response messages
 *
 *****************************************************************************/

/*********************************************************************
 * @fn      zclLight1_ProcessIncomingMsg
 *
 * @brief   Process ZCL Foundation incoming message
 *
 * @param   pInMsg - pointer to the received message
 *
 * @return  none
 */
static void zclLight1_ProcessIncomingMsg( zclIncomingMsg_t *pInMsg )
{
  switch ( pInMsg->zclHdr.commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ:
      zclLight1_ProcessInReadCmd(pInMsg);
      break;    
    case ZCL_CMD_READ_RSP:
      zclLight1_ProcessInReadRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE_RSP:
      zclLight1_ProcessInWriteRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_REPORT
    // Attribute Reporting implementation should be added here
    case ZCL_CMD_CONFIG_REPORT:
      // zclLight1_ProcessInConfigReportCmd( pInMsg );
      break;

    case ZCL_CMD_CONFIG_REPORT_RSP:
      // zclLight1_ProcessInConfigReportRspCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG:
      // zclLight1_ProcessInReadReportCfgCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG_RSP:
      // zclLight1_ProcessInReadReportCfgRspCmd( pInMsg );
      break;

    case ZCL_CMD_REPORT:
      // zclLight1_ProcessInReportCmd( pInMsg );
      break;
#endif
    case ZCL_CMD_DEFAULT_RSP:
      zclLight1_ProcessInDefaultRspCmd( pInMsg );
      break;
#ifdef ZCL_DISCOVER
    case ZCL_CMD_DISCOVER_CMDS_RECEIVED_RSP:
      zclLight1_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_CMDS_GEN_RSP:
      zclLight1_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_RSP:
      zclLight1_ProcessInDiscAttrsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_EXT_RSP:
      zclLight1_ProcessInDiscAttrsExtRspCmd( pInMsg );
      break;
#endif
    default:
      break;
  }

  if ( pInMsg->attrCmd )
    osal_mem_free( pInMsg->attrCmd );
}

#ifdef ZCL_READ
/***********************************************************




*************************************************************/
static uint8 zclLight1_ProcessInReadCmd( zclIncomingMsg_t *pInMsg )
{
 


    return(TRUE);
}




/*********************************************************************
 * @fn      zclLight1_ProcessInReadRspCmd
 *
 * @brief   Process the "Profile" Read Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclLight1_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclReadRspCmd_t *readRspCmd;
  uint8 i;

  readRspCmd = (zclReadRspCmd_t *)pInMsg->attrCmd;
  for (i = 0; i < readRspCmd->numAttr; i++)
  {
    // Notify the originator of the results of the original read attributes
    // attempt and, for each successfull request, the value of the requested
    // attribute
  }

  return ( TRUE );
}
#endif // ZCL_READ

#ifdef ZCL_WRITE
/*********************************************************************
 * @fn      zclLight1_ProcessInWriteRspCmd
 *
 * @brief   Process the "Profile" Write Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclLight1_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclWriteRspCmd_t *writeRspCmd;
  uint8 i;

  writeRspCmd = (zclWriteRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < writeRspCmd->numAttr; i++ )
  {
    // Notify the device of the results of the its original write attributes
    // command.
  }

  return ( TRUE );
}
#endif // ZCL_WRITE

/*********************************************************************
 * @fn      zclLight1_ProcessInDefaultRspCmd
 *
 * @brief   Process the "Profile" Default Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclLight1_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;

  // Device is notified of the Default Response command.
  (void)pInMsg;

  return ( TRUE );
}

#ifdef ZCL_DISCOVER
/*********************************************************************
 * @fn      zclLight1_ProcessInDiscCmdsRspCmd
 *
 * @brief   Process the Discover Commands Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclLight1_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverCmdsCmdRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverCmdsCmdRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numCmd; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}

/*********************************************************************
 * @fn      zclLight1_ProcessInDiscAttrsRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclLight1_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsRspCmd_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}

/*********************************************************************
 * @fn      zclLight1_ProcessInDiscAttrsExtRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Extended Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclLight1_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsExtRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsExtRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}
#endif // ZCL_DISCOVER

#if ZCL_EZMODE
/*********************************************************************
 * @fn      zclLight1_ProcessZDOMsgs
 *
 * @brief   Called when this node receives a ZDO/ZDP response.
 *
 * @param   none
 *
 * @return  status
 */
static void zclLight1_ProcessZDOMsgs( zdoIncomingMsg_t *pMsg )
{
  zclEZMode_ActionData_t data;
  ZDO_MatchDescRsp_t *pMatchDescRsp;

  // Let EZ-Mode know of the Simple Descriptor Response
  if ( pMsg->clusterID == Match_Desc_rsp )
  {
    pMatchDescRsp = ZDO_ParseEPListRsp( pMsg );
    data.pMatchDescRsp = pMatchDescRsp;
    zcl_EZModeAction( EZMODE_ACTION_MATCH_DESC_RSP, &data );
    osal_mem_free( pMatchDescRsp );
  }
}

/*********************************************************************
 * @fn      zclLight1_EZModeCB
 *
 * @brief   The Application is informed of events. This can be used to show on the UI what is
*           going on during EZ-Mode steering/finding/binding.
 *
 * @param   state - an
 *
 * @return  none
 */
static void zclLight1_EZModeCB( zlcEZMode_State_t state, zclEZMode_CBData_t *pData )
{
#ifdef LCD_SUPPORTED
  char *pStr;
  uint8 err;
#endif

  // time to go into identify mode
  if ( state == EZMODE_STATE_IDENTIFYING )
  {
#ifdef LCD_SUPPORTED
    HalLcdWriteString( "EZMode", HAL_LCD_LINE_2 );
#endif

    zclLight1_IdentifyTime = ( EZMODE_TIME / 1000 );  // convert to seconds
    zclLight1_ProcessIdentifyTimeChange();
  }

  // autoclosing, show what happened (success, cancelled, etc...)
  if( state == EZMODE_STATE_AUTOCLOSE )
  {
#ifdef LCD_SUPPORTED
    pStr = NULL;
    err = pData->sAutoClose.err;
    if ( err == EZMODE_ERR_SUCCESS )
    {
      pStr = "EZMode: Success";
    }
    else if ( err == EZMODE_ERR_NOMATCH )
    {
      pStr = "EZMode: NoMatch"; // not a match made in heaven
    }
    if ( pStr )
    {
      if ( giLightScreenMode == LIGHT_MAINMODE )
      {
        HalLcdWriteString ( pStr, HAL_LCD_LINE_2 );
      }
    }
#endif
  }

  // finished, either show DstAddr/EP, or nothing (depending on success or not)
  if( state == EZMODE_STATE_FINISH )
  {
    // turn off identify mode
    zclLight1_IdentifyTime = 0;
    zclLight1_ProcessIdentifyTimeChange();

#ifdef LCD_SUPPORTED
    // if successful, inform user which nwkaddr/ep we bound to
    pStr = NULL;
    err = pData->sFinish.err;
    if( err == EZMODE_ERR_SUCCESS )
    {
      // already stated on autoclose
    }
    else if ( err == EZMODE_ERR_CANCELLED )
    {
      pStr = "EZMode: Cancel";
    }
    else if ( err == EZMODE_ERR_BAD_PARAMETER )
    {
      pStr = "EZMode: BadParm";
    }
    else if ( err == EZMODE_ERR_TIMEDOUT )
    {
      pStr = "EZMode: TimeOut";
    }
    if ( pStr )
    {
      if ( giLightScreenMode == LIGHT_MAINMODE )
      {
        HalLcdWriteString ( pStr, HAL_LCD_LINE_2 );
      }
    }
#endif
    // show main UI screen 3 seconds after binding
    osal_start_timerEx( zclSmartSense_TaskID, LIGHT_MAIN_SCREEN_EVT, 3000 );
  }
}
#endif // ZCL_EZMODE
/*********************************************************************
 * @fn      zclSmartSense_ReadSensors
 *
 * @brief  
 *
 * @param   none
 *
 * @return  none
 */
static void zclSmartSense_ReadSensors(void)
{
    //*****Read LightSensor Value*****
    char buffer[30];
//    float temp;
    HalAdcSetReference(HAL_ADC_REF_AVDD);
    adc_value = HalAdcRead (HAL_ADC_CHN_AIN7, HAL_ADC_RESOLUTION_14);//read LDR
//    temp=((adc_value*3.3)/16384);// voltage value
//    temp=temp/10000;//current value
//    zclLightSensor_MeasuredValue=temp *100;
    zclLightSensor_MeasuredValue=adc_value;
    sprintf(buffer,"Illuminance :%d \n\r",zclLightSensor_MeasuredValue);
    HalUARTWrite(HAL_UART_PORT_0,buffer,osal_strlen(buffer)); 
    zclSendReport(ILLUMINANCE);
   
    //*****Read Temperature Sensor Value(SHT2X)*****
    char buffer1[30];
    error |= SHT2x_MeasurePoll(TEMP, &sT);
    temperatureC = SHT2x_CalcTemperatureC(sT.u16);
    uint16 temp1 = temperatureC *100;
    zclTemperatureSensor_Measured_Value=temp1;
    sprintf(buffer1,"Temperature :%d \n\r",temp1);
    HalUARTWrite(HAL_UART_PORT_0,buffer1,osal_strlen(buffer1));   
    zclSendReport(TEMPERATURE);
    
    //*****Read RHumidity Sensor Value(SHT2X)*****
    char buffer2[30];
    error |= SHT2x_MeasurePoll(HUMIDITY, &sT);
    humidityRH = SHT2x_CalcRH(sT.u16);
    uint16 rh = humidityRH *100;
    zclRHumiditySensor_Measured_Value=rh;
    sprintf(buffer2,"RHumidity:%d \n\r",rh); 
    HalUARTWrite(HAL_UART_PORT_0,buffer2,osal_strlen(buffer2));
    zclSendReport(RHUMIDITY);      
//    
//    //*****Read Battery Voltage*****
//    HalAdcSetReference(HAL_ADC_REF_125V);
//    adc_value = HalAdcRead (HAL_ADC_CHN_VDD3, HAL_ADC_RESOLUTION_8);
//    adc_value = adc_value *(3.57/128)*100;
//    zclNode_BatteryVoltage = adc_value;   
//    //check if battery is low   
//    if (zclNode_BatteryVoltage < MIN_BAT_VOLTAGE)
//    {
//      zclNode_AlarmMask = BAT_ALARM_MASK_VOLT_2_LOW;
//      zclGeneral_SendAlarm( OCCUPANCYSENSOR_ENDPOINT, &zclOccupancySensor_DstAddr,
//                               ALARM_CODE_BAT_VOLT_MIN_THRES_BAT_SRC_1, ZCL_CLUSTER_ID_GEN_POWER_CFG,
//                                TRUE, zclSmartSenseSeqNum); 
//    }
//    else
//    {
//      zclNode_AlarmMask = 0x00;
//    }
    
    
    osal_start_timerEx ( zclSmartSense_TaskID, SENSORS_ADC_TIMER_EVT,5000);    
}


/*********************************************************************
 * @fn      zclSmartSense_PIR_SenseMv
 *
 * @brief  
 *
 * @param   none
 *
 * @return  none
 */
static void zclSmartSense_PIR_SenseMv(void)
{
  Mv_Cnt++;
  if(Mv_Cnt == zclOccupancySensor_UtoOThresh)
    zclSmartSense_CheckPIR();
    
  if( PIR_flag )
  {
    osal_start_timerEx( zclSmartSense_TaskID, zclOccupancySensor_UtoO_TIMER_EVT,(zclOccupancySensor_UtoODelay*1000) );
    PIR_flag = FALSE;
  }
}

/****************************************************/



/****************************************************/
static void zclSmartSense_CheckPIR(void)
{
  
  //must disable interrupts here
  if(Mv_Cnt >= zclOccupancySensor_UtoOThresh)
  {
      Mv_Cnt = 0 ;
      PIR_flag = TRUE;
      zclOccupancySensor_Occupancy = 0x01;
      osal_stop_timerEx( zclSmartSense_TaskID, zclOccupancySensor_OtoU_TIMER_EVT );
      osal_start_timerEx( zclSmartSense_TaskID, zclOccupancySensor_OtoU_TIMER_EVT,(zclOccupancySensor_OtoUDelay*1000) );
  }
  else
  {
      Mv_Cnt = 0 ;
      PIR_flag = TRUE ;
      zclOccupancySensor_Occupancy = 0x00;
  }
  
  if ( zclOccupancySensor_Occupancy != zclOccupancySensor_LastOccupancy)
  {
    zclOccupancySensor_LastOccupancy = zclOccupancySensor_Occupancy;
    switch(zclOccupancySensor_LastOccupancy)
    {
    case 0x01 :
      HalLedSet(HAL_LED_Identify,HAL_LED_MODE_ON);
      break;
    case 0x00:
      HalLedSet(HAL_LED_Identify,HAL_LED_MODE_OFF);
      break;
    }
    
    zclSendReport(OCCUPANCY);
  }
 
}



/*********************************************************************
 * @fn      zclOccupancySensor_SendOccupancy
 *
 * @brief  
 *
 * @param   none
 *
 * @return  none
 */
static void zclSendReport(byte SensedValue)
{
  zclReportCmd_t *pReportCmd;
//  HalLedSet(HAL_LED_Identify,HAL_LED_MODE_TOGGLE);
  switch(SensedValue)
  {
  case TEMPERATURE:
#ifdef ZCL_REPORT
    
    pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
    if ( pReportCmd != NULL )
    {
      pReportCmd->numAttr = 1;
      pReportCmd->attrList[0].attrID = ATTRID_MS_TEMPERATURE_MEASURED_VALUE;
      pReportCmd->attrList[0].dataType = ZCL_DATATYPE_INT16;
      pReportCmd->attrList[0].attrData =(void *)&zclTemperatureSensor_Measured_Value;
      
      zcl_SendReportCmd( TEMPERATURESENSOR_ENDPOINT, &zclTemperatureSensor_DstAddr,
                        ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
                        pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSmartSenseSeqNum++ );
    }
    
    osal_mem_free( pReportCmd );
#endif  // ZCL_REPORT    
    break;
/****************************/    
  case RHUMIDITY:
#ifdef ZCL_REPORT
    
    pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
    if ( pReportCmd != NULL )
    {
      pReportCmd->numAttr = 1;
      pReportCmd->attrList[0].attrID = ATTRID_MS_RELATIVE_HUMIDITY_MEASURED_VALUE;
      pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT16;
      pReportCmd->attrList[0].attrData =(void *)&zclRHumiditySensor_Measured_Value;
      
      zcl_SendReportCmd( RHUMIDITYSENSOR_ENDPOINT, &zclRHumiditySensor_DstAddr,
                        ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY,
                        pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSmartSenseSeqNum++ );
    }
    
    osal_mem_free( pReportCmd );
#endif  // ZCL_REPORT     
    break;
/****************************/     
  case OCCUPANCY :
#ifdef ZCL_REPORT
    pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
    if ( pReportCmd != NULL )
    {
      pReportCmd->numAttr = 1;
      pReportCmd->attrList[0].attrID = ATTRID_MS_OCCUPANCY_SENSING_CONFIG_OCCUPANCY;
      pReportCmd->attrList[0].dataType = ZCL_DATATYPE_BITMAP8;
      pReportCmd->attrList[0].attrData =(void *)&zclOccupancySensor_Occupancy;
      
      zcl_SendReportCmd( OCCUPANCYSENSOR_ENDPOINT, &zclOccupancySensor_DstAddr,
                        ZCL_CLUSTER_ID_MS_OCCUPANCY_SENSING,
                        pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSmartSenseSeqNum++ );
    }
    
    osal_mem_free( pReportCmd );
#endif  // ZCL_REPORT 
    break;
/*****************************/     
  case ILLUMINANCE:
#ifdef ZCL_REPORT
    pReportCmd = osal_mem_alloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
    if ( pReportCmd != NULL )
    {
      pReportCmd->numAttr = 1;
      pReportCmd->attrList[0].attrID = ATTRID_MS_ILLUMINANCE_MEASURED_VALUE;
      pReportCmd->attrList[0].dataType = ZCL_DATATYPE_INT16;
      pReportCmd->attrList[0].attrData =(void *)&zclLightSensor_MeasuredValue;
      
      zcl_SendReportCmd( LIGHTSENSOR_ENDPOINT, &zclLightSensor_DstAddr,
                        ZCL_CLUSTER_ID_MS_ILLUMINANCE_MEASUREMENT,
                        pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, TRUE, zclSmartSenseSeqNum++ );
    }
    
    osal_mem_free( pReportCmd );
#endif  // ZCL_REPORT
    break;
  } // switch(SensedValue)
}


/****************************************************************************
****************************************************************************/


