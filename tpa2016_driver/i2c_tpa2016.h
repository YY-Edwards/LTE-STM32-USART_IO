
/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __I2C_TPA2016_H
#define __I2C_TPA2016_H

/* Includes ------------------------------------------------------------------*/
#include "stdbool.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define TPA2016_SETUP 0x1
#define TPA2016_SETUP_R_EN 0x80
#define TPA2016_SETUP_L_EN 0x40
#define TPA2016_SETUP_SWS 0x20
#define TPA2016_SETUP_R_FAULT 0x10
#define TPA2016_SETUP_L_FAULT 0x08
#define TPA2016_SETUP_THERMAL 0x04
#define TPA2016_SETUP_NOISEGATE 0x01


#define TPA2016_ATK 0x2
#define TPA2016_REL 0x3
#define TPA2016_HOLD 0x4
#define TPA2016_GAIN 0x5
#define TPA2016_AGCLIMIT 0x6
#define TPA2016_AGC 0x7
#define TPA2016_AGC_OFF 0x00
#define TPA2016_AGC_2 0x01
#define TPA2016_AGC_4 0x02
#define TPA2016_AGC_8 0x03

/* Addresses defines */
//#define TPA2016_I2CADDR 0x58


//TestStatus TransferStatus;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void GPIO_Configuration(void);
void I2C_Configuration(void);

void I2C_TPA2016_Init(void);
void I2C_TPA2016_ByteWrite(u8* pBuffer, u8 WriteAddr);
//void I2C_EE_PageWrite(u8* pBuffer, u8 WriteAddr, u8 NumByteToWrite);
//void I2C_EE_BufferWrite(u8* pBuffer, u8 WriteAddr, u16 NumByteToWrite);
void I2C_TPA2016_BufferRead(u8* pBuffer, u8 ReadAddr, u16 NumByteToRead);
void I2C_TPA2016_WaitEepromStandbyState(void);

void enableChannel(bool r, bool l);

// Register #5
void setGain(int8_t g);
int8_t getGain();

// Register #3
void setReleaseControl(uint8_t release);
// Register #2
void setAttackControl(uint8_t attack);
// Register #4
void setHoldControl(uint8_t hold);

// Register #6
void setLimitLevelOn(void);
void setLimitLevelOff(void);
void setLimitLevel(uint8_t limit);
void setNoiseGateThreshold(uint8_t Threshold);

// Register #7
void setAGCCompression(uint8_t x);
void setAGCMaxGain(uint8_t x);


void I2C_Test(void);

#endif /* __I2C_TPA2016_H */



