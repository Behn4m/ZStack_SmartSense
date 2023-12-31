/**************************************************************************************************
  Filename:       hal_sys_ctrl.c
  Revised:        $Date: 2014-06-10 13:40:34 -0700 (Tue, 10 Jun 2014) $
  Revision:       $Revision: 38940 $

  Description:    This module contains the HAL sys control and 
  power management procedures for the CC2538.

  Copyright 2011-2014 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "hw_sys_ctrl.h"
#include "sys_ctrl.h"
#include "hw_gpio.h"
#include "gpio.h"
#include "hal_mcu.h"
#include "hal_board_cfg.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Functions
 * ------------------------------------------------------------------------------------------------
 */

/**************************************************************************************************
 * @fn          SysCtrlRunSetting
 *
 * @brief       Setup which peripherals are enabled/disabled while in active/run state
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************/
void SysCtrlRunSetting(void)
{
  /* Disable unused peripherals and enable used periperals */
  /* Disable General Purpose Timers 0, 1, 2, 3 when running */
  SysCtrlPeripheralDisable(SYS_CTRL_PERIPH_GPT0);
  SysCtrlPeripheralDisable(SYS_CTRL_PERIPH_GPT1);
  SysCtrlPeripheralDisable(SYS_CTRL_PERIPH_GPT2);
  SysCtrlPeripheralDisable(SYS_CTRL_PERIPH_GPT3);
  
  /* Disable SSI 0, 1 when running */
  SysCtrlPeripheralDisable(SYS_CTRL_PERIPH_SSI0);
  SysCtrlPeripheralDisable(SYS_CTRL_PERIPH_SSI1);
  
  /* Enable UART 0, 1 when running */
  SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_UART0);
  SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_UART1);
  
  SysCtrlPeripheralReset(SYS_CTRL_PERIPH_AES);
  SysCtrlPeripheralReset(SYS_CTRL_PERIPH_PKA);
  
  /* Enable AES and PKA while running and disable I2C */
  SysCtrlPeripheralDisable(SYS_CTRL_PERIPH_I2C);
  SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_PKA);
  SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_AES);
  
  /* 
   * Enable RFC during run. Please note that this setting is 
   * only valid for PG2.0. For PG1.0 since the RFC is always on, 
   * this is only a dummy  instruction
   */
  SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_RFC);
}

/**************************************************************************************************
 * @fn          SysCtrlSleepSetting
 *
 * @brief       Setup which peripherals are enabled/disabled in Sleep
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************/
void SysCtrlSleepSetting(void)
{
  /* Disable General Purpose Timers 0, 1, 2, 3 during sleep */
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_GPT0);
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_GPT1);
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_GPT2);
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_GPT3);
  
  /* Disable SSI 0, 1 during sleep */
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_SSI0);
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_SSI1);
  
  /* Disable UART 0, 1 during sleep */
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_UART0);
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_UART1);
  
  /* Disable I2C, PKA, AES during sleep */
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_I2C);
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_PKA);
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_AES);
  
  /* 
   * Disable RFC during sleep. Please note that this setting is 
   * only valid for PG2.0. For PG1.0 this is just a dummy instruction. 
   */ 
  SysCtrlPeripheralSleepDisable(SYS_CTRL_PERIPH_RFC);
}

/**************************************************************************************************
 * @fn          SysCtrlDeepSleepSetting
 *
 * @brief       Setup which peripherals are enabled/disabled in Deep Sleep
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void SysCtrlDeepSleepSetting(void)
{
  /* Disable General Purpose Timers 0, 1, 2, 3 during deep sleep */
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_GPT0);
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_GPT1);
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_GPT2);
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_GPT3);
  
  /* Disable SSI 0, 1 during deep sleep */
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_SSI0);
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_SSI1);
  
  /* Disable UART 0, 1 during deep sleep */
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_UART0);
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_UART1);
  
  /* Disable I2C, PKA, AES during deep sleep */
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_I2C);
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_PKA);
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_AES);
  
  /* 
   * Disable RFC during deep sleep. Please note that this setting is 
   * only valid for PG2.0. For PG1.0 this is just a dummy instruction. 
   */ 
  SysCtrlPeripheralDeepSleepDisable(SYS_CTRL_PERIPH_RFC);
}

/**************************************************************************************************
 * @fn          SysCtrlWakeupSetting
 *
 * @brief       Setup which peripherals can/cannot wakeup the processor
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void SysCtrlWakeupSetting(void)
{ 
  /* GPIO A, C and SM Timer can wake up the processor */
  GPIOIntWakeupEnable(GPIO_IWE_PORT_A);
  GPIOIntWakeupDisable(GPIO_IWE_PORT_B);
  GPIOIntWakeupEnable(GPIO_IWE_PORT_C);
  GPIOIntWakeupDisable(GPIO_IWE_PORT_D);
  GPIOIntWakeupDisable(GPIO_IWE_USB);
  GPIOIntWakeupEnable(GPIO_IWE_SM_TIMER);
  
  /* Setup GPIO A, C as a falling edge  */
  GPIOPowIntTypeSet(BSP_KEY_BASE, BSP_KEY_1,GPIO_POW_FALLING_EDGE);//mine
  //GPIOPowIntTypeSet(BSP_KEY_SEL_BASE, BSP_KEY_SELECT, GPIO_POW_FALLING_EDGE);
  
}

/**************************************************************************************************
 * @fn          SysCtrlPowIntEnableSetting
 *
 * @brief       Enable power-up interrupt for the specified port, using 
 *              GPIO_PI_IEN register
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void SysCtrlPowIntEnableSetting(void)
{
  GPIOPowIntEnable(BSP_KEY_BASE, BSP_KEY_1);// mine
  //GPIOPowIntEnable(BSP_KEY_SEL_BASE, BSP_KEY_SELECT); 
}

/**************************************************************************************************
 * @fn          SysCtrlPowIntDisableSetting
 *
 * @brief       Disable power-up interrupt for the specified port, using 
 *              GPIO_PI_IEN register
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void SysCtrlPowIntDisableSetting(void)
{
  GPIOPowIntDisable(BSP_KEY_BASE, BSP_KEY_1);//mine
 // GPIOPowIntDisable(BSP_KEY_SEL_BASE, BSP_KEY_SELECT); 
}

/**************************************************************************************************
 * @fn          SysCtrlPowIntDisableSetting
 *
 * @brief       Clear the Power Interrupt registers
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void SysCtrlPowIntClear(void)
{
  GPIOPowIntClear(BSP_KEY_BASE, BSP_KEY_1);//mine
  //GPIOPowIntClear(BSP_KEY_SEL_BASE,BSP_KEY_SELECT);
}

/**************************************************************************************************
 * @fn          SysCtrlClockStartupSetting
 *
 * @brief       Setup clock startup sequence
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void SysCtrlClockStartSetting(void)
{
  /* Setup the clock startup sequence to 32 MHz external 
   * osc and 32k sourced from external oscillator
   */
  IOCPadConfigSet(GPIO_D_BASE, 0xC0, IOC_OVERRIDE_ANA);
  SysCtrlClockSet(OSC_32KHZ, false, SYS_CTRL_SYSDIV_32MHZ);
}

