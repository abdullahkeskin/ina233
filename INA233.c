#include "INA233.h"
#include <math.h>
#include "i2c.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

int INA233_Init(float maxCurrent, float ShuntRes)
{

  INA.Addr.Slave =             INA233_DEFAULT_ADDRESS;
  INA.Addr.ClearFaults =       INA233_CLEAR_FAULTS;
  INA.Addr.RestoreDefaultAll = INA233_RESTORE_DEFAULT_ALL;
  INA.Addr.Capability =        INA233_CAPABILITY;
  INA.Addr.IoutOCWarnLimit =   INA233_IOUT_OC_WARN_LIMIT;
  INA.Addr.VinOVWarnLimit =    INA233_VIN_OV_WARN_LIMIT;
  INA.Addr.VinUVWarnLimit =    INA233_VIN_UV_WARN_LIMIT;
  INA.Addr.PinOPWarnLimit =    INA233_PIN_OP_WARN_LIMIT;
  INA.Addr.StatusByte =        INA233_STATUS_BYTE;
  INA.Addr.StatusWord =        INA233_STATUS_WORD;
  INA.Addr.StatusIout =        INA233_STATUS_IOUT;
  INA.Addr.StatusInput =       INA233_STATUS_INPUT;
  INA.Addr.StatusCML =         INA233_STATUS_CML;
  INA.Addr.StatusMFRSpec =     INA233_STATUS_MFR_SPECIFIC;
  INA.Addr.ReadEin =           INA233_READ_EIN;
  INA.Addr.ReadVin =           INA233_READ_VIN;
  INA.Addr.ReadIin =           INA233_READ_IN;
  INA.Addr.ReadVout =          INA233_READ_VOUT;
  INA.Addr.ReadIout =          INA233_READ_IOUT;
  INA.Addr.ReadPout =          INA233_READ_POUT;
  INA.Addr.ReadPin =           INA233_READ_PIN;
  INA.Addr.MFRID =             INA233_MFR_ID;
  INA.Addr.MFRModel =          INA233_MFR_MODEL;
  INA.Addr.MFRModel =          INA233_MFR_REVISION;
  INA.Addr.MFRADCConfig =      INA233_MFR_ADC_CONFIG;
  INA.Addr.MFRReadVShunt =     INA233_MFR_READ_VSHUNT;
  INA.Addr.MFRAlertMask =      INA233_MFR_ALERT_MASK;
  INA.Addr.MFRCalibration =    INA233_MFR_CALIBRATION;

  INA.Addr.MFRDeviceConfig =   INA233_MFR_DEVICE_CONFIG;
  INA.Addr.ClearEin =          INA233_CLEAR_EIN;
  INA.Addr.TiMFRID =           INA233_TI_MFR_ID;
  INA.Addr.TiMFRModel =        INA233_TI_MFR_MODEL;
  INA.Addr.TiMFRRevision =     INA233_TI_MFR_REVISION;

  INA.Coeffs.mVShunt =         INA233_M_VSHUNT_COEF;
  INA.Coeffs.rVShunt =         INA233_R_VSHUNT_COEF;
  INA.Coeffs.bVShunt =         INA233_B_VSHUNT_COEF;

  INA.Coeffs.mVBus =           INA233_M_VBUS_COEF;
  INA.Coeffs.rVBus =           INA233_R_VBUS_COEF;
  INA.Coeffs.bVBus =           INA233_B_VBUS_COEF;

  INA.Coeffs.bCur =            INA233_B_CONS_COEF;
  INA.Coeffs.bPwr =            INA233_P_CONS_COEF;

  INA.Vals.MaxCurrent =        maxCurrent;
  INA.Vals.ShuntRes =          ShuntRes;

  INA.Coeffs.m_c = 0;
  INA.Coeffs.m_p = 0;
  INA.Coeffs.R_c = 0;
  INA.Coeffs.R_p = 0;

	return 0;
}

uint16_t setCalibration(float r_shunt,float i_max,float *Current_LSB,float *Power_LSB, int16_t *mc,int8_t *Rc, int16_t *mp, int8_t *Rp,  uint8_t *ERROR)
{
  float C_LSB = 0.0;
  float P_LSB = 0.0;
  float CAL = 0.0;
  float m_c_F = 0;
  float m_p_F = 0;
  int32_t aux = 0;

  uint32_t round_done = false;

  int8_t local_R_c = 0;
  int8_t local_R_p = 0;

  uint8_t local_ERROR = 0;

  C_LSB = i_max / pow(2, 15);
  P_LSB = 25 * C_LSB;

  *Current_LSB = C_LSB * 1000000;
  *Power_LSB = P_LSB * 1000;

  CAL = 0.00512 / (r_shunt * C_LSB);

  if (CAL > 0xFFFF) {
	  local_ERROR = 1;
  }
  else
  {
	  //I2C_WriteReg16b(0x80,INA233_MFR_CALIBRATION,(uint16_t)CAL);
	  uint8_t data2IC[3];

	  data2IC[0] = INA233_MFR_CALIBRATION;  // start address
	  data2IC[1] = CAL;
	  data2IC[2] = (uint16_t)CAL >> 8;

	  if(HAL_I2C_Master_Transmit(&hi2c1, 0x80, (uint8_t*)data2IC, (uint16_t)3, (uint32_t)1000)!= HAL_OK)
	  {
		  printf("I2C:WrErr %d\n\r", HAL_I2C_GetError(&hi2c1));
	  }
  }
  m_c_F = 1/C_LSB;
  m_p_F = 1/P_LSB;

  aux = (int32_t)m_c_F;

  while((aux>32768) || (aux <-32768))
  {
    m_c_F = m_c_F / 10;
    local_R_c++;
    aux = (int32_t)m_c_F;
  }
  while(round_done == false)
  {
    aux = (int32_t)m_c_F;
    if(aux == m_c_F)
    {
      round_done = true;
    }
    else
    {
      aux = (int32_t)(m_c_F*10);
      if((aux>32768) || (aux<-32768))
      {
        round_done = true;
      }
      else
      {
        m_c_F = m_c_F*10;
        local_R_c--;
      }
    }
  }
  round_done = false;

  aux = (int32_t)m_p_F;
  while((aux>32768) || (aux<-32768))
  {
    m_p_F = m_p_F/10;
    local_R_p++;
    aux = (int32_t)m_p_F;
  }
  while(round_done == false)
  {
    aux = (int32_t)m_p_F;
    if(aux == m_p_F)
    {
      round_done = true;
    }
    else
    {
      aux = (int32_t)(m_p_F*10);
      if((aux>32768) || (aux<-32768))
      {
        round_done = true;
      }
      else
      {
        m_p_F = m_p_F*10;
        local_R_p--;
      }
    }
  }

  *mp=m_p_F;
  *mc=m_c_F;
  *Rc=local_R_c;
  *Rp=local_R_p;
  *ERROR=local_ERROR;

  m_c = (int16_t)(m_c_F);
  m_p = (int16_t)(m_p_F);
  R_c = local_R_c;
  R_p = local_R_p;

  return (uint16_t)CAL;
}

void INA233_wireReadWord(uint8_t reg, uint16_t *value)
{
   HAL_I2C_Mem_Read(&hi2c1,INA_ADDR,INA233_MFR_READ_VSHUNT,I2C_MEMADD_SIZE_8BIT,(uint8_t *)inaBuff,2,0xFFFF);

  *value = inaBuff[0];
  *value=((inaBuff[1] << 8) | *value);
}

float INA233_getShuntVoltage_mV() {
  uint16_t value=getShuntVoltage_raw();
  float vshunt;
  vshunt=(value*pow(10,-R_vs)-b_vs)/m_vs;
  return vshunt * 1000;
}

int16_t INA233_getShuntVoltage_raw() {
  uint16_t value;
  wireReadWord(INA233_MFR_READ_VSHUNT, &value);
  return (int16_t)value;}

int INA233GetValue()
{
  INA.Vals.BusVoltageRaw = I2C_ReadReg16b(INA.Addr.Slave,INA.Addr.ReadVin);
  INA.Vals.ShuntVoltageRaw = I2C_ReadReg16b(INA.Addr.Slave,INA.Addr.MFRReadVShunt);
  INA.Vals.CurrentRaw = I2C_ReadReg16b(INA.Addr.Slave,INA.Addr.ReadIin);
  INA.Vals.PowerRaw = I2C_ReadReg16b(INA.Addr.Slave,INA.Addr.ReadPin);

  INA.Vals.BusVoltage = (INA.Vals.BusVoltageRaw*pow(10,-INA.Coeffs.rVBus)-INA.Coeffs.bVBus)/INA.Coeffs.mVBus;
  INA.Vals.ShuntVoltage = (INA.Vals.ShuntVoltageRaw*pow(10,-INA.Coeffs.rVShunt)-INA.Coeffs.bVShunt)/INA.Coeffs.mVShunt;
  INA.Vals.Current = (INA.Vals.CurrentRaw*pow(10,-INA.Coeffs.R_c)-INA.Coeffs.bCur)/INA.Coeffs.m_c;
  INA.Vals.Power = (INA.Vals.PowerRaw*pow(10,-INA.Coeffs.R_p)-INA.Coeffs.bPwr)/INA.Coeffs.m_p;

  uint16_t accumulator = 0;
  uint8_t roll_over = 0;
  uint32_t sample_count = 0;
  uint32_t accumulator_24 = 0;
  uint32_t raw_av_power = 0;
  float av_power = 0;

  getEnergy_raw(&accumulator, &roll_over, &sample_count);
  accumulator_24 = (uint32_t)(roll_over)*65536 + (uint32_t)(accumulator);

  raw_av_power = (raw_av_power*pow(10,-INA.Coeffs.R_p)-INA.Coeffs.bPwr)/INA.Coeffs.m_p;
  INA.Vals.AvaragePower = raw_av_power;

	return 0;

}
float getBusVoltage_V(void)
{
  return INA.Vals.BusVoltage;
}
float getShuntVoltage_V(void)
{
  return INA.Vals.ShuntVoltage*1000;
}
float getShuntVoltage_mV(void)
{
  return INA.Vals.ShuntVoltage*1000;
}
float getCurrent_mA(void)
{
  return INA.Vals.Current*1000;
}
float getCurrent_A(void)
{
  return INA.Vals.Current;
}
float getPower_mW(void)
{
  return INA.Vals.Power*1000;
}
float getPower_W(void)
{
  return INA.Vals.Power;
}

uint16_t getBusVoltage_raw(void)
{
  return INA.Vals.BusVoltageRaw;
}
uint16_t getShuntVoltage_raw(void)
{
  return INA.Vals.ShuntVoltageRaw;
}
uint16_t getCurrent_raw(void)
{
  return INA.Vals.CurrentRaw;
}
uint16_t getPower_raw(void)
{
  return INA.Vals.PowerRaw;
}

float getAvPower_mW(void)
{
  return INA.Vals.AvaragePower*1000;
}
float getAvPower_W(void)
{
  return INA.Vals.AvaragePower;
}
void getEnergy_raw(uint16_t *accumulator, uint8_t *roll_over, uint32_t *sample_count)
{
  uint8_t evalue[6];
  uint32_t aux;
  I2C_ReadReg48b(INA.Addr.Slave,INA.Addr.ReadEin,evalue);

  /*To-Do; Check pointers and bit shift operations*/

  *accumulator=(evalue[1] << 8) | evalue[0];
  *roll_over=evalue[2];
  *sample_count=(uint32_t)(evalue[5])<< 16;
  *sample_count=((uint32_t)(evalue[4])<< 8)| *sample_count;
  *sample_count=((uint32_t)(evalue[3])| *sample_count);
}
