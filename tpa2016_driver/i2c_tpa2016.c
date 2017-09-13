/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : i2c_ee.c
* Author             : MCD Application Team
* Version            : V2.0.1
* Date               : 06/13/2008
* Description        : This file provides a set of functions needed to manage the
*                      communication between I2C peripheral and I2C M24C08 EEPROM.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_i2c.h"
#include "i2c_tpa2016.h"
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define I2C_Speed              200000
#define I2C2_SLAVE_ADDRESS7    0xB0//TPA2016硬件地址
#define I2C_PageSize           8

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//u16 EEPROM_ADDRESS;

/* Private function prototypes -----------------------------------------------*/
void GPIO_Configuration(void);
void I2C_Configuration(void);

/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configure the used I/O ports pin
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
    
  /* Configure I2C2 pins: SCL and SDA */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*******************************************************************************
* Function Name  : I2C_Configuration
* Description    : I2C Configuration
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_Configuration(void)
{
  I2C_InitTypeDef  I2C_InitStructure; 

  /* I2C configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;//i2c接口模式
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;//高低电平周期
  I2C_InitStructure.I2C_OwnAddress1 = I2C2_SLAVE_ADDRESS7;//I2C2从机硬件地址
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = I2C_Speed;
  
  /* Apply I2C configuration after enabling it */
  I2C_Init(I2C2, &I2C_InitStructure);

  /* I2C Peripheral Enable */
  I2C_Cmd(I2C2, ENABLE);
  
  /*允许1字节1应答模式*/
  I2C_AcknowledgeConfig(I2C2, ENABLE);              //使能I2C接口响应
    
  //printf("\n\r I2C_Configuration----\n\r");
    
}

/*******************************************************************************
* Function Name  : I2C_TPA2016_Init
* Description    : Initializes peripherals used by the I2C TPA2016 driver.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_TPA2016_Init()
{
  /* GPIO configuration */
  GPIO_Configuration();

  /* I2C configuration */
  I2C_Configuration();

}

/*******************************************************************************
* Function Name  : I2C_EE_BufferWrite
* Description    : Writes buffer of data to the I2C EEPROM.
* Input          : - pBuffer : pointer to the buffer  containing the data to be 
*                    written to the EEPROM.
*                  - WriteAddr : EEPROM's internal address to write to.
*                  - NumByteToWrite : number of bytes to write to the EEPROM.
* Output         : None
* Return         : None
*******************************************************************************/
//void I2C_EE_BufferWrite(u8* pBuffer, u8 WriteAddr, u16 NumByteToWrite)
//{
//  u8 NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0;
//
//  Addr = WriteAddr % I2C_PageSize;  //0x00%8=0
//  count = I2C_PageSize - Addr;		//8-0=8
//  NumOfPage =  NumByteToWrite / I2C_PageSize;	//256/8	=32
//  NumOfSingle = NumByteToWrite % I2C_PageSize;	//256%8	=0
// 
//  /* If WriteAddr is I2C_PageSize aligned  */
//  if(Addr == 0) 
//  {
//    /* If NumByteToWrite < I2C_PageSize */
//    if(NumOfPage == 0) 
//    {
//      I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
//      I2C_EE_WaitEepromStandbyState();
//    }
//    /* If NumByteToWrite > I2C_PageSize */
//    else  
//    {
//      while(NumOfPage--)  //一次写8个数据
//      {
//        I2C_EE_PageWrite(pBuffer, WriteAddr, I2C_PageSize); 
//    	I2C_EE_WaitEepromStandbyState();
//        WriteAddr +=  I2C_PageSize;
//        pBuffer += I2C_PageSize;
//      }
//
//      if(NumOfSingle!=0)	//除8个以外的
//      {
//        I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
//        I2C_EE_WaitEepromStandbyState();
//      }
//    }
//  }
//  /* If WriteAddr is not I2C_PageSize aligned  */
//  else 
//  {
//    /* If NumByteToWrite < I2C_PageSize */
//    if(NumOfPage== 0) 
//    {
//      I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
//      I2C_EE_WaitEepromStandbyState();
//    }
//    /* If NumByteToWrite > I2C_PageSize */
//    else
//    {
//      NumByteToWrite -= count;
//      NumOfPage =  NumByteToWrite / I2C_PageSize;
//      NumOfSingle = NumByteToWrite % I2C_PageSize;	
//      
//      if(count != 0)
//      {  
//        I2C_EE_PageWrite(pBuffer, WriteAddr, count);
//        I2C_EE_WaitEepromStandbyState();
//        WriteAddr += count;
//        pBuffer += count;
//      } 
//      
//      while(NumOfPage--)
//      {
//        I2C_EE_PageWrite(pBuffer, WriteAddr, I2C_PageSize);
//        I2C_EE_WaitEepromStandbyState();
//        WriteAddr +=  I2C_PageSize;
//        pBuffer += I2C_PageSize;  
//      }
//      if(NumOfSingle != 0)
//      {
//        I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle); 
//        I2C_EE_WaitEepromStandbyState();
//      }
//    }
//  }  
//}

/*******************************************************************************
* Function Name  : I2C_TPA2016_ByteWrite
* Description    : Writes one byte to the I2C TPA2016.
* Input          : - pBuffer : pointer to the buffer  containing the data to be 
*                    written to the TPA2016.
*                  - WriteAddr : TPA2016's internal address to write to.
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_TPA2016_ByteWrite(u8* pBuffer, u8 WriteAddr)
{
  /* Send STRAT condition */
  I2C_GenerateSTART(I2C2, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));  

  /* Send TPA2016 address for write */
  I2C_Send7bitAddress(I2C2, I2C2_SLAVE_ADDRESS7, I2C_Direction_Transmitter);
  
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
      
  /* Send the TPA2016's internal address to write to */
  I2C_SendData(I2C2, WriteAddr);
  
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send the byte to be written */
  I2C_SendData(I2C2, *pBuffer); 
   
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  
  /* Send STOP condition */
  I2C_GenerateSTOP(I2C2, ENABLE);
  
  I2C_TPA2016_WaitEepromStandbyState();
}

/*******************************************************************************
* Function Name  : I2C_EE_PageWrite
* Description    : Writes more than one byte to the EEPROM with a single WRITE
*                  cycle. The number of byte can't exceed the EEPROM page size.
* Input          : - pBuffer : pointer to the buffer containing the data to be 
*                    written to the EEPROM.
*                  - WriteAddr : EEPROM's internal address to write to.
*                  - NumByteToWrite : number of bytes to write to the EEPROM.
* Output         : None
* Return         : None
*******************************************************************************/
//void I2C_EE_PageWrite(u8* pBuffer, u8 WriteAddr, u8 NumByteToWrite)
//{
//    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)); // Added by Najoua 27/08/2008
//    
//  /* 发送启动条件 */
//  I2C_GenerateSTART(I2C1, ENABLE);
//  
//  /* Test on EV5 and clear it */
//  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)); 
//  
//  /* Send EEPROM address for write */
//  I2C_Send7bitAddress(I2C1, EEPROM_ADDRESS, I2C_Direction_Transmitter);
//  
//  /* Test on EV6 and clear it */
//  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));  
//
//  /* Send the EEPROM's internal address to write to */    
//  I2C_SendData(I2C1, WriteAddr);  
//
//  /* Test on EV8 and clear it */
//  while(! I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
//
//  /* While there is data to be written */
//  while(NumByteToWrite--)  
//  {
//    /* Send the current byte */
//    I2C_SendData(I2C1, *pBuffer); 
//
//    /* Point to the next byte to be written */
//    pBuffer++; 
//  
//    /* Test on EV8 and clear it */
//    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
//  }
//
//  /* Send STOP condition */
//  I2C_GenerateSTOP(I2C1, ENABLE);
//}

/*******************************************************************************
* Function Name  : I2C_TPA2016_BufferRead
* Description    : Reads a block of data from the TPA2016.
* Input          : - pBuffer : pointer to the buffer that receives the data read 
*                    from the TPA2016.
*                  - ReadAddr : TPA2016's internal address to read from.
*                  - NumByteToRead : number of bytes to read from the TPA2016.
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_TPA2016_BufferRead(u8* pBuffer, u8 ReadAddr, u16 NumByteToRead)
{  
   
  if(NumByteToRead==0)	return;
  
  while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY)); // Added by Najoua 27/08/2008
    
  /*允许1字节1应答模式*/
  I2C_AcknowledgeConfig(I2C2, ENABLE); 
    
  /* Send START condition */
  I2C_GenerateSTART(I2C2, ENABLE);//发送起始位
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
  

  /* Send TPA2016 address for write *///发送器件地址(写)
  I2C_Send7bitAddress(I2C2, I2C2_SLAVE_ADDRESS7, I2C_Direction_Transmitter);
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  
  /* Clear EV6 by setting again the PE bit */
  //I2C_Cmd(I2C2, ENABLE);//这是为什么？

  /* Send the TPA2016's internal address to write to */
  I2C_SendData(I2C2, ReadAddr);  //发送地址
  /* Test on EV8 and clear it *///数据已发送
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  
  
  /* Send STRAT condition a second time */  
  I2C_GenerateSTART(I2C2, ENABLE);//起始位
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
  
  /* Send TPA2016 address for read *///器件读
  I2C_Send7bitAddress(I2C2, I2C2_SLAVE_ADDRESS7, I2C_Direction_Receiver);
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
  
  /* While there is data to be read */
  while(NumByteToRead)  
  {
    if(NumByteToRead == 1)
    {
      /* Disable Acknowledgement *///最后一位后要关闭应答的
      I2C_AcknowledgeConfig(I2C2, DISABLE);
      
      /* Send STOP Condition */
      I2C_GenerateSTOP(I2C2, ENABLE);//发送停止位
    }

    /* Test on EV7 and clear it */
    if(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED))  
    {      
      /* Read a byte from the EEPROM */
      *pBuffer = I2C_ReceiveData(I2C2);

      /* Point to the next location where the byte read will be saved */
      pBuffer++; 
      
      /* Decrement the read bytes counter */
      NumByteToRead--;        
    }   
  }

  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(I2C2, ENABLE);//再次允许应答模式
}

/*******************************************************************************
* Function Name  : I2C_TPA2016_WaitEepromStandbyState
* Description    : Wait for TPA2016 Standby state
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_TPA2016_WaitEepromStandbyState(void)      
{
  vu16 SR1_Tmp = 0;

  do
  {
    /* Send START condition */
    I2C_GenerateSTART(I2C2, ENABLE);
    /* Read I2C2 SR1 register */
    SR1_Tmp = I2C_ReadRegister(I2C2, I2C_Register_SR1);
    /* Send TPA2016 address for write */
    I2C_Send7bitAddress(I2C2, I2C2_SLAVE_ADDRESS7, I2C_Direction_Transmitter);
  }while(!(I2C_ReadRegister(I2C2, I2C_Register_SR1) & 0x0002));
  
  /* Clear AF flag */
  I2C_ClearFlag(I2C2, I2C_FLAG_AF);
    /* STOP condition */    
  I2C_GenerateSTOP(I2C2, ENABLE); // Added by Najoua 27/08/2008
}


//void I2C_Test(void)
//{
//    u16 i;
//    u8 I2c_Buf_Write[256];
//    u8 I2c_Buf_Read[256];
//
//    printf("写入的数据\n\r");
//
//    for(i=0;i<=255;i++) //填充缓冲
//    {   
//        I2c_Buf_Write[i]=i;
//        printf("0x%02X ",I2c_Buf_Write[i]);
//        if(i%16 == 15)
//        {
//            printf("\n\r");
//        }
//    }
//
//    //将I2c_Buf_Write中顺序递增的数据写入EERPOM中 
//    I2C_EE_BufferWrite(I2c_Buf_Write,EEP_Firstpage,256);	 
//
//    printf("\n\r读出的数据\n\r");
//    //将EEPROM读出数据顺序保持到I2c_Buf_Read中 
//    I2C_EE_BufferRead(I2c_Buf_Read,EEP_Firstpage,256); 
//
//    //将I2c_Buf_Read中的数据通过串口打印
//    for(i=0;i<256;i++)
//    {	
//        if(I2c_Buf_Read[i]!=I2c_Buf_Write[i])
//        {
//            printf("0x%02X ", I2c_Buf_Read[i]);
//            printf("错误:I2C EEPROM写入与读出的数据不一致\n\r");
//            return;
//        }
//        printf("0x%02X ", I2c_Buf_Read[i]);
//        if(i%16 == 15)
//        {
//            printf("\n\r");
//        }
//    }
//    printf("读写测试通过PASSED\n\r");
//}


/****TPA2016_interface****/
// Set the gain in dB!
void setGain(int8_t g) {
  if (g > 30) g = 30;
  if (g < -28) g = -28;
  
  I2C_TPA2016_ByteWrite(&g, TPA2016_GAIN);

  //write8(TPA2016_GAIN, g);
}

// for querying the gain, returns in dB
int8_t getGain(void) {
  
    //return read8(TPA2016_GAIN);
  int8_t return_value =0;
  
  I2C_TPA2016_BufferRead(&return_value, TPA2016_GAIN, 1);
  return return_value;
}

// Turn on/off right and left channels
void enableChannel(bool r, bool l) {

  //uint8_t setup = read8(TPA2016_SETUP);
  uint8_t setup =0;
  
  I2C_TPA2016_BufferRead(&setup, TPA2016_SETUP, 1);
  
  if (r)
    setup |= TPA2016_SETUP_R_EN;
  else 
    setup &= ~TPA2016_SETUP_R_EN;
  if (l)
    setup |= TPA2016_SETUP_L_EN;
  else 
    setup &= ~TPA2016_SETUP_L_EN;
  
  I2C_TPA2016_ByteWrite(&setup, TPA2016_SETUP);
  
  //write8(TPA2016_SETUP, setup);
}

// Set to OFF, 1:2, 1:4 or 1:8
void setAGCCompression(uint8_t x) {
  if (x > 3) return; // only 2 bits!
  
  //uint8_t agc = read8(TPA2016_AGC);
  
  uint8_t agc =0;
  I2C_TPA2016_BufferRead(&agc, TPA2016_AGC, 1);
  
  agc &= ~(0x03);  // mask off bottom 2 bits
  agc |= x;        // set the compression ratio.
  
  I2C_TPA2016_ByteWrite(&agc, TPA2016_AGC);
  //write8(TPA2016_AGC, agc);
}

void setReleaseControl(uint8_t release) {
  if (release > 0x3F) return; // only 6 bits!

   I2C_TPA2016_ByteWrite(&release, TPA2016_REL);
  //write8(TPA2016_REL, release);
}

void setAttackControl(uint8_t attack) {
  if (attack > 0x3F) return; // only 6 bits!

  I2C_TPA2016_ByteWrite(&attack, TPA2016_ATK);
  //write8(TPA2016_ATK, attack);
}

void setHoldControl(uint8_t hold) {
  if (hold > 0x3F) return; // only 6 bits!

  I2C_TPA2016_ByteWrite(&hold, TPA2016_HOLD);
  //write8(TPA2016_HOLD, hold);
}

// Turn on power limiter
void setLimitLevelOn(void) {
  //uint8_t agc = read8(TPA2016_AGCLIMIT);
  
   uint8_t agc =0;
  I2C_TPA2016_BufferRead(&agc, TPA2016_AGCLIMIT, 1);
  
  agc &= ~(0x80);  // mask off top bit
  
  I2C_TPA2016_ByteWrite(&agc, TPA2016_AGCLIMIT);
  //write8(TPA2016_AGCLIMIT, agc);
}

// Turn off power limiter
void setLimitLevelOff(void) {
  //uint8_t agc = read8(TPA2016_AGCLIMIT);
   uint8_t agc =0;
  I2C_TPA2016_BufferRead(&agc, TPA2016_AGCLIMIT, 1);
  
  agc |= 0x80;  // turn on top bit
  
  I2C_TPA2016_ByteWrite(&agc, TPA2016_AGCLIMIT);
  //write8(TPA2016_AGCLIMIT, agc);
}

// Set limit levels
void setLimitLevel(uint8_t limit) {
  if (limit > 31) return;

  //uint8_t agc = read8(TPA2016_AGCLIMIT);
  uint8_t agc =0;
  I2C_TPA2016_BufferRead(&agc, TPA2016_AGCLIMIT, 1);

  agc &= ~(0x1F);  // mask off bottom 5 bits
  agc |= limit;        // set the limit level.
  
  I2C_TPA2016_ByteWrite(&agc, TPA2016_AGCLIMIT);

  //write8(TPA2016_AGCLIMIT, agc);
}

void setAGCMaxGain(uint8_t x) {
  if (x > 12) return; // max gain max is 12 (30dB)
  
  //uint8_t agc = read8(TPA2016_AGC);
  
  uint8_t agc =0;
  I2C_TPA2016_BufferRead(&agc, TPA2016_AGC, 1);
  
  agc &= ~(0xF0);  // mask off top 4 bits
  agc |= (x << 4);        // set the max gain
  
  I2C_TPA2016_ByteWrite(&agc, TPA2016_AGC);
  //write8(TPA2016_AGC, agc);
}












