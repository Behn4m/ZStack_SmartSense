/**************************************************************************************************
  Filename:       hal_i2c.c
  Revised:        $Date: 2013-05-20 10:14:45 -0700 (Mon, 20 May 2013) $
  Revision:       $Revision: 34373 $

  Description:    This file contains the interface to the HAL ADC.


  Copyright 2013 Texas Instruments Incorporated. All rights reserved.

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

/**************************************************************************************************
 *                                           INCLUDES
 **************************************************************************************************/

#include "hw_ints.h"
#include "hw_memmap.h"
#include "gpio.h"
#include "interrupt.h"
#include "ioc.h"
#include "hw_ioc.h"
#include "sys_ctrl.h"
#include "hw_i2cm.h"
#include "hw_i2cs.h"
#include "i2c.h"
#include "hal_i2c.h"
#include "bsp.h"   
   
#include <Typesdef.h>      


/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/


/* ------------------------------------------------------------------------------------------------
 *                                       Local Variables
 * ------------------------------------------------------------------------------------------------
 */



/**************************************************************************************************
 * @fn      HalAdcInit
 *
 * @brief   Initialize ADC Service
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalI2CInit (void)
{
#if (HAL_I2C == TRUE)
  
    //
    //  The I2C peripheral must be enabled before use.
    //
    SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_I2C);
    
    //
    // Do reset of I2C module
    //
    SysCtrlPeripheralReset(SYS_CTRL_PERIPH_I2C);

    //
    // Configure I2C pins
    //
    GPIOPinTypeI2C(BSP_I2C_BASE, BSP_I2C_SCL);
    GPIOPinTypeI2C(BSP_I2C_BASE, BSP_I2C_SDA);

    
    // Configure pins as peripheral input and output
    
    IOCPinConfigPeriphInput(BSP_I2C_BASE, BSP_I2C_SCL, 
                            IOC_I2CMSSCL);
    IOCPinConfigPeriphInput(BSP_I2C_BASE, BSP_I2C_SDA,
                            IOC_I2CMSSDA);    
    IOCPinConfigPeriphOutput(BSP_I2C_BASE, BSP_I2C_SCL,
                             IOC_MUX_OUT_SEL_I2C_CMSSCL);
    IOCPinConfigPeriphOutput(BSP_I2C_BASE, BSP_I2C_SDA,
                             IOC_MUX_OUT_SEL_I2C_CMSSDA);   
     //
    // Enable and initialize the I2C master module.  Use the system clock for
    // the I2C module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.  For this example we will use a data rate of 100kbps.   
    I2CMasterInitExpClk(SysCtrlClockGet(), false);    

#endif
}

//==============================================================================
void I2c_Init ()
//==============================================================================
{

}
//==============================================================================
void I2c_StartCondition ()
//==============================================================================
{

}
//==============================================================================
void I2c_StopCondition ()
//==============================================================================
{

}
//==============================================================================
u8t I2c_WriteByte (u8t txByte)
//==============================================================================
{
  

  return 0;
}
//==============================================================================
u8t I2c_ReadByte (etI2cAck ack)
//==============================================================================
{

  return 0;
}

/**************************************************************************************************
**************************************************************************************************/
