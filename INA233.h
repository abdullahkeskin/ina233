#ifndef __INA233_H
#define __INA233_H
#include <stdio.h>
#include <stdint.h>

#ifndef false
#define false (0)
#endif

#ifndef true
#define true  (1)
#endif
/*Slave address descriptions for A1 pin and A0 pin */
#ifndef INA233_DEFAULT_ADDRESS
#define INA233_DEFAULT_ADDRESS      INA233_ADDRESS_40H /*A1=GND A2=GND */
#endif

#define INA233_ADDRESS_40H          0x40 /*0b1000000  (A1=GND A2=GND) */
#define INA233_ADDRESS_41H          0x41 /*0b1000001  (A1=GND A2=VDD) */
#define INA233_ADDRESS_42H          0x42 /*0b1000010  (A1=GND A2=SDA) */
#define INA233_ADDRESS_43H          0x43 /*0b1000011  (A1=GND A2=SCL) */
#define INA233_ADDRESS_44H          0x44 /*0b1000100  (A1=VDD A2=GND) */
#define INA233_ADDRESS_45H          0x45 /*0b1000101  (A1=VDD A2=VDD) */
#define INA233_ADDRESS_46H          0x46 /*0b1000110  (A1=VDD A2=SDA) */
#define INA233_ADDRESS_47H          0x47 /*0b1000111  (A1=VDD A2=SCL) */
#define INA233_ADDRESS_48H          0x48 /*0b1001000  (A1=SDA A2=GND) */
#define INA233_ADDRESS_49H          0x49 /*0b1001001  (A1=SDA A2=VDD) */
#define INA233_ADDRESS_4AH          0x4A /*0b1001010  (A1=SDA A2=SDA) */
#define INA233_ADDRESS_4BH          0x4B /*0b1001011  (A1=SDA A2=SCL) */
#define INA233_ADDRESS_4CH          0x4C /*0b1001100  (A1=SCL A2=GND) */
#define INA233_ADDRESS_4DH          0x4D /*0b1001101  (A1=SCL A2=VDD) */
#define INA233_ADDRESS_4EH          0x4E /*0b1001110  (A1=SCL A2=SDA) */
#define INA233_ADDRESS_4FH          0x4F /*0b1001111  (A1=SCL A2=SCL) */

/*End of slave address descriptions */

/*Function addresses */
        /*R/W/S/C/B Access Types;Read,Write,Send byte, Clear, Block Read, */
        /*NB:Number of Data Bytes, V:Default Value, Description  */
#define INA233_CLEAR_FAULTS          0x03 /*S, 0, N/A */
#define INA233_RESTORE_DEFAULT_ALL   0x12 /*S, 0, N/A */
#define INA233_CAPABILITY            0x19 /*R, 1, 0xB0 */
#define INA233_IOUT_OC_WARN_LIMIT    0x4A /*R/W, 2, 0x7FF8 */
#define INA233_VIN_OV_WARN_LIMIT     0x57 /*R/W, 2, 0x7FF8 */
#define INA233_VIN_UV_WARN_LIMIT     0x58 /*R/W, 2, 0x0000 */
#define INA233_PIN_OP_WARN_LIMIT     0x6B /*R/W, 2, 0x7FF8 */
#define INA233_STATUS_BYTE           0x78 /*R, 1, 0x00 */
#define INA233_STATUS_WORD           0x79 /*R, 2, 0x1000 */
#define INA233_STATUS_IOUT           0x7B /*R/W/C, 1, 0x00 */
#define INA233_STATUS_INPUT          0x7C /*R/W/C, 1, 0x00*/
#define INA233_STATUS_CML            0x7E /*R/W/C, 1, 0x00 */
#define INA233_STATUS_MFR_SPECIFIC   0x80 /*R/W/C, 1, 0x20 */
#define INA233_READ_EIN              0x86 /*B, 6, 0x00/00/00/00/00/00 */
#define INA233_READ_VIN              0x88 /*R, 2, 0x0000 */
#define INA233_READ_IN               0x89 /*R, 2, 0x0000 */
#define INA233_READ_VOUT             0x8B /*R, 2, 0x0000 */
#define INA233_READ_IOUT             0x8C /*R, 2, 0x0000 */
#define INA233_READ_POUT             0x96 /*R, 2, 0x0000 */
#define INA233_READ_PIN              0x97 /*R, 2, 0x0000 */
#define INA233_MFR_ID                0x99 /*B, 2, 0x54/49 */
#define INA233_MFR_MODEL             0x9A /*B, 6, 0x49/4E/41/32/33/33h */
#define INA233_MFR_REVISION          0x9B /*R, 2, 0x41/30 */
#define INA233_MFR_ADC_CONFIG        0xD0 /*R/W, 2, 0x4127 */
#define INA233_MFR_READ_VSHUNT       0xD1 /*R, 2, 0x0000 */
#define INA233_MFR_ALERT_MASK        0xD2 /*R/W, 1, 0xF0 */
#define INA233_MFR_CALIBRATION       0xD4 /*R/W, 2, 0x0001 */
#define INA233_MFR_DEVICE_CONFIG     0xD5 /*R/W, 1, 0x02 */
#define INA233_CLEAR_EIN             0xD6 /*S, 0, N/A */
#define INA233_TI_MFR_ID             0xE0 /*R, 2, ASCII TI, 0x5449 */
#define INA233_TI_MFR_MODEL          0xE1 /*R, 2, ASCII 33 */
#define INA233_TI_MFR_REVISION       0xE2 /*R, 2, ASCII A0 */
/*End of function addresses  */

/*Coefficients */
/*  Please see for more ../Docs/ina233.pdf page:17-18
*
*   Reading telemetry data and warning thresholds
*   X = (1/m)*(Y*(10^-R)-b)
*   X = Calculated realworld value (Volt/Amps/Watts and so forth)
*   m = the slope coefficient
*   Y = a 2-byte, two's complement integer received from the device
*   b = the offset, which is a 2-byte, two's complement integer
*   R = the exponent, which is a 1-byte, two's complement integer
*   R is only necessary in systems where m is required to be an integer (for example, where m can be stored in a
* register of an integrated circuit) and R must only be large enough to yield the desired accuracy
*
*   Writing telemetry data and warning thresholds
*   Y = (m*X + b)*10^R
*   X = the realworld value (volts, amps, watts, temperature, and so forth)
*   m = the slope coefficient, a 2-byte, two's complement integer
*   Y = a 2-byte, two's complement integer written to the device
*   b = the offset which is a 2-byte, two's complement integer
*   R = the exponent, which is a 1-byte, two's complement integer
*/
/*Shunt voltage telemetry and warning */
#define INA233_M_VSHUNT_COEF         4
#define INA233_R_VSHUNT_COEF         5
#define INA233_B_VSHUNT_COEF         0
/*Bus voltage telemetry and warning */
#define INA233_M_VBUS_COEF           8
#define INA233_R_VBUS_COEF           2
#define INA233_B_VBUS_COEF           0
/*Current and power telemetry and warning */
#define INA233_B_CONS_COEF           0
#define INA233_P_CONS_COEF           0
/*End of coefficients */

/* Calibration register and scaling
*  CAL = (0.00512/(Current_LSB*Rshunt))
*  Current_LSB = Maximum_Expected_Current / (2^15)
*  Required values are Shunt Resistor and Maximum expected current
*  0.00512 and 2^15 internal fixed values
*  End of calibration register and scaling
*/

/*Data structures */
struct Addresses{
  uint8_t Slave;
  uint8_t ClearFaults;
  uint8_t RestoreDefaultAll;
  uint8_t Capability;
  uint8_t IoutOCWarnLimit;
  uint8_t VinOVWarnLimit;
  uint8_t VinUVWarnLimit;
  uint8_t PinOPWarnLimit;
  uint8_t StatusByte;
  uint8_t StatusWord;
  uint8_t StatusIout;
  uint8_t StatusInput;
  uint8_t StatusCML;
  uint8_t StatusMFRSpec;
  uint8_t ReadEin;
  uint8_t ReadVin;
  uint8_t ReadIin;
  uint8_t ReadVout;
  uint8_t ReadIout;
  uint8_t ReadPout;
  uint8_t ReadPin;
  uint8_t MFRID;
  uint8_t MFRModel;
  uint8_t MFRADCConfig;
  uint8_t MFRReadVShunt;
  uint8_t MFRAlertMask;
  uint8_t MFRCalibration;

  uint8_t MFRDeviceConfig;
  uint8_t ClearEin;
  uint8_t TiMFRID;
  uint8_t TiMFRModel;
  uint8_t TiMFRRevision;

};
struct Coefficients{
  uint32_t mVShunt;
  uint32_t rVShunt;
  uint32_t bVShunt;

  uint32_t mVBus;
  uint32_t rVBus;
  uint32_t bVBus;

  uint32_t bCur;
  uint32_t bPwr;

  uint16_t mc;
  uint16_t m_c;
  uint16_t mp;
  uint16_t m_p;
  uint8_t Rp;
  uint8_t R_p;
  uint8_t Rc;
  uint8_t R_c;


};
struct Values{
  float BusVoltage;
  float ShuntVoltage;
  float Current;
  float Power;
  float AvaragePower;

  int16_t BusVoltageRaw;
  int16_t ShuntVoltageRaw;
  int16_t CurrentRaw;
  int16_t PowerRaw;
  int16_t EnergyRaw;

  float MaxCurrent;
  float ShuntRes;

  uint8_t SystemError;

  float Current_LSB;
  float Power_LSB;



};

struct INA233{
  struct Addresses Addr;
  struct Coefficients Coeffs;
  struct Values Vals;
};

/*=========================================================================*/
/*=========================================================================
    SHUNT VOLTAGE TELEMETRY & WARNING COEFFICIENTS
    -----------------------------------------------------------------------*/
    #define m_vs                (4.0)
    #define R_vs                (5.0)
    #define b_vs                (0.0)
/*=========================================================================*/

/*=========================================================================
    BUS VOLTAGE TELEMETRY & WARNING COEFFICIENTS
    -----------------------------------------------------------------------*/
    #define m_vb                (8)
    #define R_vb                (2)
    #define b_vb                (0)
/*=========================================================================*/

/*=========================================================================
    CURRENT & POWER CONSTANT TELEMETRY & WARNING COEFFICIENTS
    -----------------------------------------------------------------------*/
    #define b_c                (0)
    #define b_p                (0)
/*=========================================================================*/

#define TI_MFR_ID 0xE0
#define TI_MFR_MODEL 0xE1
#define TI_MFR_REVISION 0xE2
#define tprint(x) HAL_UART_Transmit(&huart2,(uint8_t *)x,sizeof(x),0xFFF);
#define INA_ADDR 0x81
#define INABUFFSIZE 6
uint8_t inaBuff[INABUFFSIZE];
uint8_t dbgBuff[100];

static struct INA233 INA;

/*End of Data structures */

/*Function Prototypes */

int INA233_Init(float maxCurrent, float ShuntRes);
int INA233GetValue(void);
float getBusVoltage_V(void);
float getShuntVoltage_mV(void);
float getCurrent_mA(void);
float getPower_mW(void);
float getAvPower_mW(void);

uint16_t getBusVoltage_raw(void);
uint16_t getShuntVoltage_raw(void);
uint16_t getCurrent_raw(void);
uint16_t getPower_raw(void);

void getEnergy_raw(uint16_t *accumulator, uint8_t *roll_over, uint32_t *sample_count);
uint16_t setCalibration(float r_shunt,float i_max,float *Current_LSB,float *Power_LSB, int16_t *mc,int8_t *Rc, int16_t *mp, int8_t *Rp,  uint8_t *ERROR);

void wireReadWord(uint8_t reg, uint16_t *value);
void wireReadByte(uint8_t reg, uint8_t *value);
void wireReadBlock(uint8_t reg, uint8_t value[6]);
void wireWriteWord(uint8_t reg, uint16_t value);
void wireWriteByte (uint8_t reg, uint8_t value);
void wireSendCmd(uint8_t reg);

int16_t m_c;
int8_t R_c;
int16_t m_p;
int8_t R_p;
/*End of Function Prototypes */

#endif
