/**************************************************************************************************
  Filename:       hal_i2c.h
  Revised:        $Date: 2013-03-06 13:50:31 -0800 (Wed, 06 Mar 2013) $
  Revision:       $Revision: 33395 $

  Description:    This file contains the interface to the ADC Service.


  Copyright 2005-2010 Texas Instruments Incorporated. All rights reserved.

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

#ifndef HAL_I2C_H
#define HAL_I2C_H

#ifdef __cplusplus
extern "C"
{
  

#endif

/**************************************************************************************************
 * INCLUDES
 **************************************************************************************************/
#include "Typesdef.h"


/**************************************************************************************************
 * CONSTANTS
 **************************************************************************************************/
//---------- Defines -----------------------------------------------------------
//I2C ports
//The communication on SDA and SCL is done by switching pad direction
//For a low level on SCL or SDA, direction is set to output. For a high level on
//SCL or SDA, direction is set to input. (pull up resistor active)
//#define SDA BSP_I2C_SDA //SDA on I/O P38 defines direction (input=1/output=0)
//#define SDA_CONF P3H_bit.no0 //SDA level on output direction
//#define SCL BSP_I2C_SCL //SCL on I/O P39 defines direction (input=1/output=0)
//#define SCL_CONF P3H_bit.no1 //SCL level on output direction
//---------- Enumerations ------------------------------------------------------
// I2C level
   typedef enum{
     LOW = 0,
     HIGH = 1,
   }etI2cLevel;
// I2C acknowledge
   typedef enum{
     ACK = 0,
     NO_ACK = 1,
   }etI2cAck;

/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/
extern void HalI2CInit(void) ; 

//==============================================================================
extern void I2c_Init (void);
//==============================================================================
//Initializes the ports for I2C interface
//==============================================================================
extern void I2c_StartCondition (void);
//==============================================================================
// writes a start condition on I2C-bus
// input : -
// output: -
// return: -
// note : timing (delay) may have to be changed for different microcontroller
// _____
// SDA: |_____
// _______
// SCL : |___
//==============================================================================
extern void I2c_StopCondition (void);
//==============================================================================
// writes a stop condition on I2C-bus
// input : -
// output: -
// return: -
// note : timing (delay) may have to be changed for different microcontroller

//==============================================================================
extern u8t I2c_WriteByte (u8t txByte);
//==============================================================================

//==============================================================================
extern u8t I2c_ReadByte (etI2cAck ack);
//==============================================================================


/**************************************************************************************************
**************************************************************************************************/

#ifdef __cplusplus
}
#endif

#endif
