/**************************************************************************************************
  Filename:       zcl_LIGHT.h
  Revised:        $Date: 2014-06-19 08:38:22 -0700 (Thu, 19 Jun 2014) $
  Revision:       $Revision: 39101 $

  Description:    This file contains the Zigbee Cluster Library Home
                  Automation Sample Application.


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
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

#ifndef ZCL_LIGHT_H
#define ZCL_LIGHT_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "zcl.h"

/*********************************************************************
 * CONSTANTS
 */
#define LIGHT1_ENDPOINT                  10
#define LIGHT2_ENDPOINT                  11
#define OCCUPANCYSENSOR_ENDPOINT         12
#define LIGHTSENSOR_ENDPOINT             13  
#define TEMPERATURESENSOR_ENDPOINT       14
#define RHUMIDITYSENSOR_ENDPOINT         15  
  
#define LIGHT_OFF                       0x00
#define LIGHT_ON                        0x01

// Application Events
#define LIGHT_IDENTIFY_TIMEOUT_EVT              0x0001
#define LIGHT_POLL_CONTROL_TIMEOUT_EVT          0x0002
#define LIGHT_EZMODE_TIMEOUT_EVT                0x0004
#define LIGHT_EZMODE_NEXTSTATE_EVT              0x0008
#define LIGHT_MAIN_SCREEN_EVT                   0x0010
#define LIGHT_LEVEL_CTRL_EVT                    0x0020
#define LIGHT_START_EZMODE_EVT                  0x0040 
#define SENSORS_ADC_TIMER_EVT                   0x0080 
#define zclOccupancySensor_UtoO_TIMER_EVT       0x0100
#define zclOccupancySensor_OtoU_TIMER_EVT       0x0200
#define FACTORY_RESET_NWK_JOIN_EVT              0x0400
  
// Application Display Modes
#define LIGHT_MAINMODE      0x00
#define LIGHT_HELPMODE      0x01

#define MIN_BAT_VOLTAGE     330
#define MIN_ILLUMINANCE     3000
  
#define TEMPERATURE     1
#define RHUMIDITY       2
#define OCCUPANCY       3
#define ILLUMINANCE     4

/*********************************************************************
 * MACROS
 */
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * VARIABLES
 */
extern SimpleDescriptionFormat_t zclLight1_SimpleDesc;
extern SimpleDescriptionFormat_t zclLight2_SimpleDesc; //mine
extern SimpleDescriptionFormat_t zclOccupancySensor_SimpleDesc; //mine
extern SimpleDescriptionFormat_t zclLightSensor_SimpleDesc; //mine
extern SimpleDescriptionFormat_t zclTemperatureSensor_SimpleDesc; //mine
extern SimpleDescriptionFormat_t zclRHumiditySensor_SimpleDesc; //mine

extern CONST zclCommandRec_t zclLight1_Cmds[];
extern CONST zclCommandRec_t zclLight2_Cmds[];//mine
extern CONST zclCommandRec_t zclOccupancySensor_Cmds[];//mine
extern CONST zclCommandRec_t zclLightSensor_Cmds[];//mine
extern CONST zclCommandRec_t zclTemperatureSensor_Cmds[];//mine
extern CONST zclCommandRec_t zclRHumiditySensor_Cmds[];//mine

extern CONST uint8 zclLight1CmdsArraySize;
extern CONST uint8 zclLight2CmdsArraySize;
extern CONST uint8 zclOccupancySensorCmdsArraySize;
extern CONST uint8 zclLightSensorCmdsArraySize;
extern CONST uint8 zclTemperatureSensorCmdsArraySize;
extern CONST uint8 zclRHumiditySensorCmdsArraySize;

// attribute list
extern CONST zclAttrRec_t zclLight1_Attrs[];
extern CONST uint8 zclLight1_NumAttributes;

extern CONST zclAttrRec_t zclLight2_Attrs[];//mine
extern CONST uint8 zclLight2_NumAttributes;//mine

extern CONST zclAttrRec_t zclOccupancySensor_Attrs[];//mine
extern CONST uint8 zclOccupancySensor_NumAttributes;//mine

extern CONST zclAttrRec_t zclLightSensor_Attrs[];//mine
extern CONST uint8 zclLightSensor_NumAttributes;//mine

extern CONST zclAttrRec_t zclTempratureSensor_Attrs[];//mine
extern CONST uint8 zclTempratureSensor_NumAttributes;//mine

extern CONST zclAttrRec_t zclRHumiditySensor_Attrs[];//mine
extern CONST uint8 zclRHumiditySensor_NumAttributes;//mine

// Identify attributes
extern uint16 zclLight1_IdentifyTime;
extern uint8  zclLight1_IdentifyCommissionState;

extern uint16 zclLight2_IdentifyTime;//mine
extern uint8  zclLight2_IdentifyCommissionState;//mine

extern uint16 zclOccupancySensor_IdentifyTime;//mine
extern uint8  zclOccupancySensor_IdentifyCommissionState;//mine

extern uint16 zclLightSensor_IdentifyTime;//mine
extern uint8  zclLightSensor_IdentifyCommissionState;//mine

extern uint16 zclTemperatureSensor_IdentifyTime;//mine
extern uint8  zclTemperatureSensor_IdentifyCommissionState;//mine

extern uint16 zclRHumiditySensor_IdentifyTime;//mine
extern uint8  zclRHumiditySensor_IdentifyCommissionState;//mine

// OnOff attributes
extern uint8  zclLight1_OnOff;
extern uint8  zclLight2_OnOff;

// Level Control Attributes
#ifdef ZCL_LEVEL_CTRL
extern uint8  zclLight1_LevelCurrentLevel;
extern uint16 zclLight1_LevelRemainingTime;
extern uint16 zclLight1_LevelOnOffTransitionTime;
extern uint8  zclLight1_LevelOnLevel;
extern uint16 zclLight1_LevelOnTransitionTime;
extern uint16 zclLight1_LevelOffTransitionTime;
extern uint8  zclLight1_LevelDefaultMoveRate;

/*mine*/
extern uint8  zclLight2_LevelCurrentLevel;
extern uint16 zclLight2_LevelRemainingTime;
extern uint16 zclLight2_LevelOnOffTransitionTime;
extern uint8  zclLight2_LevelOnLevel;
extern uint16 zclLight2_LevelOnTransitionTime;
extern uint16 zclLight2_LevelOffTransitionTime;
extern uint8  zclLight2_LevelDefaultMoveRate;
#endif


//*****Illuminance Measurement Attributes*****
extern uint16 zclLightSensor_MeasuredValue;
//extern uint16 zclLightSensor_MinMeasuredValue;
//extern uint16 zclLightSensor_MaxMeasuredValue;
//*****Occupancy Sensing Attributes*****
//Occupancy Sensor Information Attribute Set
extern uint8 zclOccupancySensor_Occupancy;
extern uint8 zclOccupancySensor_OccupancySensorType;
//Attributes of the PIR Configuration Attribute Set
extern uint16 zclOccupancySensor_OtoUDelay;
extern uint16 zclOccupancySensor_UtoODelay;
extern uint8 zclOccupancySensor_UtoOThresh;
//Temperature Measurement attributes
extern int16 zclTemperatureSensor_Measured_Value;
extern int16 zclTemperatureSensor_Min_Measured_Value;
extern int16 zclTemperatureSensor_Max_Measured_Value;
//Relative Humidity Attributes
extern uint16 zclRHumiditySensor_Measured_Value;
extern uint16 zclRHumiditySensor_Min_Measured_Value;
extern uint16 zclRHumiditySensor_Max_Measured_Value;
//Power Configuration Cluster
extern uint16 zclNode_BatteryVoltage;
extern uint8 zclNode_AlarmMask;
//Alarms Cluster


/*end of mine*/

/*********************************************************************
 * FUNCTIONS
 */

 /*
  * Initialization for the task
  */
extern void zclSmartSense_Init( byte task_id );

/*
 *  Event Process for the task
 */
extern UINT16 zclSmartSense_event_loop( byte task_id, UINT16 events );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ZCL_LIGHT_H */
