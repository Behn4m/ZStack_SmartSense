//==============================================================================
// S E N S I R I O N AG, Laubisruetistr. 50, CH-8712 Staefa, Switzerland
//==============================================================================
// Project : SHT2x Sample Code (V1.2)
// File : SHT2x.c
// Author : MST
// Controller: NEC V850/SG3 (uPD70F3740)
// Compiler : IAR compiler for V850 (3.50A)
// Brief : Sensor layer. Functions for sensor access
//==============================================================================
//---------- Includes ----------------------------------------------------------
#include "SHT2x.h"
#include "hal_i2c.h"
#include "bsp.h"

const u16t POLYNOMIAL = 0x131; //P(x)=x^8+x^5+x^4+1 = 100110001
//==============================================================================
u8t SHT2x_CheckCrc(u8t data[], u8t nbrOfBytes, u8t checksum)
//==============================================================================
{
  u8t crc = 0;
  u8t byteCtr;
  //calculates 8-Bit checksum with given polynomial
  for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
  { crc ^= (data[byteCtr]);
  for (u8t bit = 8; bit > 0; --bit)
  { if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
  else crc = (crc << 1);
  }
  }
  if (crc != checksum) return CHECKSUM_ERROR;
  else return 0;
}
//===========================================================================
u8t SHT2x_ReadUserRegister(u8t *pRegisterValue)
//===========================================================================
{
  u8t checksum; //variable for checksum byte
  u8t error=0; //variable for error code
  I2c_StartCondition();
  error |= I2c_WriteByte (I2C_ADR_W);
  error |= I2c_WriteByte (USER_REG_R);
  I2c_StartCondition();
  error |= I2c_WriteByte (I2C_ADR_R);
  *pRegisterValue = I2c_ReadByte(ACK);
  checksum=I2c_ReadByte(NO_ACK);
  error |= SHT2x_CheckCrc (pRegisterValue,1,checksum);
  I2c_StopCondition();
  return error;
}
//===========================================================================
u8t SHT2x_WriteUserRegister(u8t *pRegisterValue)
//===========================================================================
{
  u8t error=0; //variable for error code
  I2c_StartCondition();
  error |= I2c_WriteByte (I2C_ADR_W);
  error |= I2c_WriteByte (USER_REG_W);
  error |= I2c_WriteByte (*pRegisterValue);
  I2c_StopCondition();
  return error;
}
//===========================================================================
u8t SHT2x_MeasureHM(etSHT2xMeasureType eSHT2xMeasureType, nt16 *pMeasurand)
//===========================================================================
{
  u8t checksum; //checksum
  u8t data[2]; //data array for checksum verification
  u8t error=0; //error variable
  u16t i; //counting variable
  //-- write I2C sensor address and command --
  I2c_StartCondition();
  error |= I2c_WriteByte (I2C_ADR_W); // I2C Adr
  switch(eSHT2xMeasureType)
  { 
  case HUMIDITY: error |= I2c_WriteByte (TRIG_RH_MEASUREMENT_HM); break;
  case TEMP : error |= I2c_WriteByte (TRIG_T_MEASUREMENT_HM); break;
  //default: assert(0);
  }
  //-- wait until hold master is released --
  I2c_StartCondition();
  error |= I2c_WriteByte (I2C_ADR_R);
  
  //-- read two data bytes and one checksum byte --
  pMeasurand->s16.u8H = data[0] = I2c_ReadByte(ACK);
  pMeasurand->s16.u8L = data[1] = I2c_ReadByte(ACK);
  checksum=I2c_ReadByte(NO_ACK);
  //-- verify checksum --
  error |= SHT2x_CheckCrc (data,2,checksum);
  I2c_StopCondition();
  return error;
}
//===========================================================================
u8t SHT2x_MeasurePoll(etSHT2xMeasureType eSHT2xMeasureType, nt16 *pMeasurand)
//===========================================================================
{
  u8t error=0; //error variable
  I2CMasterSlaveAddrSet(I2C_ADR,false); // address is 64 and action is write
  switch(eSHT2xMeasureType)
  { 
  case HUMIDITY: 
        //
        // Place the data to be sent in the data register
        //
        I2CMasterDataPut(TRIG_RH_MEASUREMENT_POLL);

        //
        // Initiate send of data from the master.  Since the loopback
        // mode is enabled, the master and slave units are connected
        // allowing us to receive the same data that we sent out.
        //
        I2CMasterControl(I2C_MASTER_CMD_SINGLE_SEND);
    break;
  case TEMP : 
        //
        // Place the data to be sent in the data register
        //
        I2CMasterDataPut(TRIG_T_MEASUREMENT_HM);

        //
        // Initiate send of data from the master.  Since the loopback
        // mode is enabled, the master and slave units are connected
        // allowing us to receive the same data that we sent out.
        //
        I2CMasterControl(I2C_MASTER_CMD_SINGLE_SEND); 
    break;
  }
  while(I2CMasterBusy())
  {
  }
  // check I2CM_CTRL register for error
  
  I2CMasterSlaveAddrSet(I2C_ADR, true);
  I2CMasterControl(I2C_MASTER_CMD_BURST_RECEIVE_START);
  
  while(I2CMasterBusy())
  {
  };
  if(I2CMasterErr() == I2C_MASTER_ERR_NONE)
  {
    pMeasurand->s16.u8H =I2CMasterDataGet();
    I2CMasterControl(I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusy())
    {
    };
    //check for errors
    if(I2CMasterErr() == I2C_MASTER_ERR_NONE)
    {
      pMeasurand->s16.u8L = I2CMasterDataGet();
 
    } 
    else 
      return 0;
  }
  else
  {
    I2CMasterControl(I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
    return 0;
  }   

return error;  
}
//===========================================================================
u8t SHT2x_SoftReset()
//===========================================================================
{
  u8t error=0; //error variable
 
  I2CMasterSlaveAddrSet(I2C_ADR,false);
  I2CMasterDataPut(SOFT_RESET);
  I2CMasterControl(I2C_MASTER_CMD_SINGLE_SEND);
  //check for errors
  
  return error;
}
//==============================================================================
float SHT2x_CalcRH(u16t u16sRH)
//==============================================================================
{
  ft humidityRH; // variable for result
  u16sRH &= ~0x0003; // clear bits [1..0] (status bits)
  //-- calculate relative humidity [%RH] --
  humidityRH = -6.0 + 125.0/65536 * (ft)u16sRH; // RH= -6 + 125 * SRH/2^16
  return humidityRH;
}
//==============================================================================
float SHT2x_CalcTemperatureC(u16t u16sT)
//==============================================================================
{
  ft temperatureC; // variable for result
  u16sT &= ~0x0003; // clear bits [1..0] (status bits)
  //-- calculate temperature [�C] --
  temperatureC= -46.85 + 175.72/65536 *(ft)u16sT; //T= -46.85 + 175.72 * ST/2^16
  return temperatureC;
}
//==============================================================================
u8t SHT2x_GetSerialNumber(u8t u8SerialNumber[])
//==============================================================================
{
  u8t error=0; //error variable
//  //Read from memory location 1
//  I2c_StartCondition();
//  error |= I2c_WriteByte (I2C_ADR_W); //I2C address
//  error |= I2c_WriteByte (0xFA); //Command for readout on-chip memory
//  error |= I2c_WriteByte (0x0F); //on-chip memory address
//  I2c_StartCondition();
//  error |= I2c_WriteByte (I2C_ADR_R); //I2C address
//  u8SerialNumber[5] = I2c_ReadByte(ACK); //Read SNB_3
//  I2c_ReadByte(ACK); //Read CRC SNB_3 (CRC is not analyzed)
//  u8SerialNumber[4] = I2c_ReadByte(ACK); //Read SNB_2
//  I2c_ReadByte(ACK); //Read CRC SNB_2 (CRC is not analyzed)
//  u8SerialNumber[3] = I2c_ReadByte(ACK); //Read SNB_1
//  I2c_ReadByte(ACK); //Read CRC SNB_1 (CRC is not analyzed)
//  u8SerialNumber[2] = I2c_ReadByte(ACK); //Read SNB_0
//  I2c_ReadByte(NO_ACK); //Read CRC SNB_0 (CRC is not analyzed)
//  I2c_StopCondition();
//  //Read from memory location 2
//  I2c_StartCondition();
//  error |= I2c_WriteByte (I2C_ADR_W); //I2C address
//  error |= I2c_WriteByte (0xFC); //Command for readout on-chip memory
//  error |= I2c_WriteByte (0xC9); //on-chip memory address
//  I2c_StartCondition();
//  error |= I2c_WriteByte (I2C_ADR_R); //I2C address
//  u8SerialNumber[1] = I2c_ReadByte(ACK); //Read SNC_1
//  u8SerialNumber[0] = I2c_ReadByte(ACK); //Read SNC_0
//  I2c_ReadByte(ACK); //Read CRC SNC0/1 (CRC is not analyzed)
//  u8SerialNumber[7] = I2c_ReadByte(ACK); //Read SNA_1
//  u8SerialNumber[6] = I2c_ReadByte(ACK); //Read SNA_0
//  I2c_ReadByte(NO_ACK); //Read CRC SNA0/1 (CRC is not analyzed)
//  I2c_StopCondition();
  return error;
}