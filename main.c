/**
  ******************************************************************************
  * @file    keysanf_usart/main.c 
  * @author  EDWARDS
  * @version V1.0.0
  * @date    05/03/2017
  * @brief   Main program body
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "platform_config.h"
#include "i2c_tpa2016.h"

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */
    
PUTCHAR_PROTOTYPE
{
    USART_SendData(USART1, ch);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    {}
    return ch;
}

int simple_uart_put(u8 ch)
{
    USART_SendData(USART1, ch);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    {}
    return ch;
}

/** @addtogroup USART_Interrupt
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
//typedef enum { FAILED = 0, PASSED = !FAILED} TestStatus;

typedef enum key_states_e{  
    KEY_NOT_PRESS,  //初始状态，未按下
    KEY_PRESSED,  //按下
    KEY_RELEASE,   //释放
}KeyStatus_t; 

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define countof(a)   (sizeof(a) / sizeof(*(a)))
#define UART_RE_LEN    256

/* Private variables ---------------------------------------------------------*/


u8 USART_RX[UART_RE_LEN]={0};


//列如按下键为key0,则短按键值为0xA0, 长按键值为0xB0,释放键值为0xC0
volatile u8 press_counter = 0;
volatile u8 time_out = 0;
static volatile u16 current_key_value = 0x99;//当前按键值
static volatile u16 last_key_value = 0x99;//上次按键值

static volatile u16 pressed_value =0x99;//按下后返回的键值（包括短按长按）
static volatile u16 release_value =0x99;//释放返回的键值
volatile u8 first_set_volume_flag =1;//

//static KeyStatus_t KeyStatus = KEY_NOT_PRESS;

volatile u16 key1_pressed_flag = 0;//key1
volatile u16 key2_pressed_flag = 0;//key2
volatile u16 key3_pressed_flag = 0;//key3
volatile u16 key4_pressed_flag = 0;//key4
volatile u16 key5_pressed_flag = 0;//key5
volatile u16 key6_pressed_flag = 0;//key6
volatile u16 key7_pressed_flag = 0;//key7
volatile u16 key8_pressed_flag = 0;//key8

 NVIC_InitTypeDef NVIC_InitStructure;
 USART_InitTypeDef USART_InitStructure;
 DMA_InitTypeDef  DMA_InitStructure; 

/* Private function prototypes -----------------------------------------------*/
void USART2_Init(void);
void USART1_Init(void);
void LTE_GPIO_Init(void);
void NVIC_Configuration(void);
void EXTI_USER_Init(void);
void TIM3_Int_Init(u16 arr,u16 psc);
void Write_Volume(unsigned short dat);

//TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);



void Write_Volume(unsigned short dat)
{
    SCLK = 0;
    LOAD = 0;
    
    for(int i = 15; i >= 0; i--)
    {     
      DATA = (dat  >> i) & 1;
        delay_us(3);
        SCLK =1;
        delay_us(3);
        SCLK =0;       
    }
    delay_us(3);   
    LOAD = 1;
    delay_us(5);
    SCLK =1;       
}


/* Private functions ---------------------------------------------------------*/

/**
  * @brief   Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     
 //u16 ReadVale =0x00;
  delay_init(72);	    	 //延时函数初始化	

  NVIC_Configuration();//设置中断优先级分组
  
  LTE_GPIO_Init();//初始化KEY,LED,LKJ,Volume的IO模式
  
  EXTI_USER_Init();//外部中断初始化，在这里初始化8个对应按键的中断输入
  
  USART1_Init();//串口1初始化,带打印调试, 开启串口中断接收功能
  
/****
  
  I2C_Test();
  
 ****/
  
  
  //USART2_Init();
    
  //TIM3_Int_Init(100-1, 7199);//定时器3初始化,10Khz的计数频率，计数到100为10ms  
  
  GPIO_SetBits(GPIO_LED_1_2, DS1_PIN|DS2_PIN);/*关闭所有的LED指示灯*/
  
  GPIO_SetBits(GPIO_LED_3_6, RED1_PIN|YELLOW_PIN|GREEN_PIN|RED2_PIN);/*熄灭所有的信号灯*///输出高电平
  
  Write_Volume(0x0001);//set default volume at 9
  
  //delay_ms(1500); 
  //delay_ms(1500); 
  
  //Write_Volume(0x06);//set default volume at 6
  
  //delay_ms(1500); 
  //delay_ms(1500); 
  
  //Write_Volume(0x06);//set default volume at 5
  
  //printf("\r\n/***********************key scan start*********************/\r\n");
  while(1)
  {     
      GPIO_ResetBits(GPIO_LED_1_2, DS1_PIN);
      delay_ms(200); 
      GPIO_SetBits(GPIO_LED_1_2, DS1_PIN);
      delay_ms(100); 
      //GPIO_ResetBits(GPIO_LED, DS2_PIN);
      delay_ms(200); 
      //GPIO_SetBits(GPIO_LED, DS2_PIN);
      delay_ms(100);   
      if(first_set_volume_flag)
      {
          Write_Volume(0x0006);//set default volume at 5
          first_set_volume_flag =0;
          GPIO_ResetBits(GPIO_LED_3_6, RED1_PIN|YELLOW_PIN|GREEN_PIN|RED2_PIN);/*点亮所有的信号灯*///输出低电平
      }
      
      
//      GPIO_ResetBits(GPIO_LED_3_6, RED1_PIN);
//
//      delay_ms(1000); 
//      
//      GPIO_SetBits(GPIO_LED_3_6, RED1_PIN);
//      delay_ms(1000); 
//      
//      GPIO_ResetBits(GPIO_LED_3_6, YELLOW_PIN);
//      delay_ms(1000); 
//      
//      GPIO_SetBits(GPIO_LED_3_6, YELLOW_PIN);
//      delay_ms(1000); 
//      
//      GPIO_ResetBits(GPIO_LED_3_6, GREEN_PIN);
//      delay_ms(1000); 
//      
//      GPIO_SetBits(GPIO_LED_3_6, GREEN_PIN);
//      delay_ms(1000); 
//      
//      GPIO_ResetBits(GPIO_LED_3_6, RED2_PIN);
//      delay_ms(1000); 
//      
//      GPIO_SetBits(GPIO_LED_3_6, RED2_PIN);
//      delay_ms(1000); 
//      
//      
//      GPIO_ResetBits(GPIO_LED_3_6, RED1_PIN|YELLOW_PIN|GREEN_PIN|RED2_PIN);/*点亮所有的信号灯*///输出低电平
//      delay_ms(1000); 
//      delay_ms(1000); 
//      GPIO_SetBits(GPIO_LED_3_6, RED1_PIN|YELLOW_PIN|GREEN_PIN|RED2_PIN);/*关闭所有的信号灯*///输出低电平
//      delay_ms(1000); 
//      
//      GPIO_SetBits(GPIO_LKJ_5_6, LKJ5_PIN);//LKJ5输出高电平，驱动三极管导通
//      delay_ms(1000); 
//      delay_ms(1000); 
//      GPIO_ResetBits(GPIO_LKJ_5_6, LKJ5_PIN);
//      delay_ms(1000); 
//      delay_ms(1000); 
      
            
  }
        
       
 }


/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void USART1_Init(void)
{   
   GPIO_InitTypeDef GPIO_InitStructure;
   
  /*使能串口1使用的GPIO时钟*/
  RCC_APB2PeriphClockCmd(USART1_GPIO_CLK , ENABLE);

  /*使能串口1时钟*/
  RCC_APB2PeriphClockCmd(USART1_CLK, ENABLE); 
  
    /*串口1 RX管脚配置*/
  /* Configure USART1 Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = USART1_RxPin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(USART1_GPIO, &GPIO_InitStructure);

  /*串口1 TX管脚配置*/ 
  /* Configure USART1 Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = USART1_TxPin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(USART1_GPIO, &GPIO_InitStructure);
  
  /* USART1 configuration ------------------------------------------------------*/

  /* USART1 configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;               /*设置波特率为115200*/
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;/*设置数据位为8*/
  USART_InitStructure.USART_StopBits = USART_StopBits_1;     /*设置停止位为1位*/
  USART_InitStructure.USART_Parity = USART_Parity_No;        /*无奇偶校验*/
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;/*无硬件流控*/
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  /*发送和接收*/

  /*配置串口1 */
  USART_Init(USART1, &USART_InitStructure);
 
  /*使能串口1的接收中断*/
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART_ClearFlag(USART1,USART_FLAG_RXNE); 	
  
  
//  USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
//  USART_ITConfig(USART1, USART_IT_TC, DISABLE);
//  USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);//空闲中断（接收未知长度，则使用空闲中断来判断是否接收完毕） 
//  USART_ClearFlag(USART1,USART_FLAG_IDLE); 				//清USART_FLAG_IDLE标志 
//
//
//  //采用DMA方式接收  
//  USART_DMACmd(USART1,USART_DMAReq_Rx ,ENABLE);
//    
//  //开启DMA1的时钟
//  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
//  
//  /* DMA1 channel 5 configuration */ //USART1_RX  
//  DMA_DeInit(DMA1_Channel5);  
//  DMA_InitStructure.DMA_PeripheralBaseAddr      =(u32)(&USART1->DR);  			//外设串口1地址  
// 
//  DMA_InitStructure.DMA_MemoryBaseAddr          =(u32)USART_RX;
//  DMA_InitStructure.DMA_DIR                     =DMA_DIR_PeripheralSRC;   	//外设作为目的地址   //DMA_DIR_PeripheralSRC;   //外设作为DMA的源端  
//  DMA_InitStructure.DMA_BufferSize              =256; 				//BufferSize;      //传输缓冲器大小 
//  DMA_InitStructure.DMA_PeripheralInc           =DMA_PeripheralInc_Disable; 	//外设递增模式禁止   DMA_PeripheralInc_Enable;            //外设地址增加  
//  DMA_InitStructure.DMA_MemoryInc               =DMA_MemoryInc_Enable;   	//内存地址自增  
//  DMA_InitStructure.DMA_PeripheralDataSize      =DMA_PeripheralDataSize_Byte; 	//传输方式：字节   DMA_PeripheralDataSize_Word;    //字（32位）  
//  DMA_InitStructure.DMA_MemoryDataSize          =DMA_MemoryDataSize_Byte;  	//内存存储方式：字节  DMA_MemoryDataSize_Word;  
//  DMA_InitStructure.DMA_Mode                    =DMA_Mode_Normal;  		//DMA_Mode_Normal 正常模式，只传送一次;  DMA_Mode_Circular:循环模式，不停的传送;  
//  DMA_InitStructure.DMA_Priority                =DMA_Priority_VeryHigh;  	//串口1的接收作为最高优先级
//  DMA_InitStructure.DMA_M2M                     =DMA_M2M_Disable;             	//DMA_M2M_Enable;      
//  DMA_Init(DMA1_Channel5,&DMA_InitStructure); 
//
//	//使能通道5  
//  DMA_Cmd(DMA1_Channel5,ENABLE);  
  
  
  /* Enable the USART1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;	        //抢占优先级0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;			//子优先级1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* 使能串口1 */
  USART_Cmd(USART1, ENABLE);
  USART_ClearFlag(USART1,USART_FLAG_TC); //清除USART_FLAG_TC，解决第一个字节不能发出的问题 
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    {}
  


}


void USART2_Init(void)
{   
   GPIO_InitTypeDef GPIO_InitStructure;
   
  /*使能串口2使用的GPIO时钟*/
  RCC_APB2PeriphClockCmd(USART2_GPIO_CLK , ENABLE);

  /*使能串口2时钟*/
  RCC_APB1PeriphClockCmd(USART2_CLK, ENABLE); 
  
    /*串口2 RX管脚配置*/
  /* Configure USART1 Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = USART2_RxPin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(USART2_GPIO, &GPIO_InitStructure);

  /*串口2 TX管脚配置*/ 
  /* Configure USART1 Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = USART2_TxPin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(USART2_GPIO, &GPIO_InitStructure);
  
  /* USART1 configuration ------------------------------------------------------*/

  /* USART1 configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;               /*设置波特率为115200*/
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;/*设置数据位为8*/
  USART_InitStructure.USART_StopBits = USART_StopBits_1;     /*设置停止位为1位*/
  USART_InitStructure.USART_Parity = USART_Parity_No;        /*无奇偶校验*/
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;/*无硬件流控*/
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  /*发送和接收*/

  /*配置串口2 */
  USART_Init(USART2, &USART_InitStructure);
 
  /*使能串口2的发送和接收中断*/
  //USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  //USART_ITConfig(USART2, USART_IT_TC, ENABLE);
  
  /* Enable the USART1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;	        //抢占优先级0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;			//子优先级1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* 使能串口2 */
  USART_Cmd(USART2, ENABLE);
  USART_ClearFlag(USART2,USART_FLAG_TC); //清除USART_FLAG_TC，解决第一个字节不能发出的问题 
  


}



/**
  * @brief  Configures the different LTE GPIO ports.
  * @param  None
  * @retval None
  */
void LTE_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  //使能时钟
  /*使能LED灯使用的GPIO时钟*/
  /*使能KEY扫描按键使用的GPIO时钟*/
  RCC_APB2PeriphClockCmd(RCC_GPIO_KEY_5_8 | RCC_GPIO_KEY_1_4| RCC_GPIO_VOLUME_CTL, ENABLE);
  
  RCC_APB2PeriphClockCmd(RCC_GPIO_LKJ_5_6|RCC_GPIO_LKJ_1_4, ENABLE);
  
  RCC_APB2PeriphClockCmd(RCC_GPIO_LED_1_2|RCC_GPIO_LED_3_6, ENABLE);
  
   /* KEY按键使用的GPIO管脚模式*/
  GPIO_InitStructure.GPIO_Pin = KEY5_PIN|KEY6_PIN|KEY7_PIN|KEY8_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIO_KEY_5_8, &GPIO_InitStructure); 
  
  GPIO_InitStructure.GPIO_Pin = KEY1_PIN|KEY2_PIN|KEY3_PIN|KEY4_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIO_KEY_1_4, &GPIO_InitStructure); 
  
  /* LKJ使用的GPIO管脚模式*/
  GPIO_InitStructure.GPIO_Pin = LKJ1_PIN|LKJ2_PIN|LKJ3_PIN|LKJ4_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIO_LKJ_1_4, &GPIO_InitStructure); 
  
   GPIO_InitStructure.GPIO_Pin = LKJ5_PIN|LKJ6_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIO_LKJ_5_6, &GPIO_InitStructure); 
  
  

  /* LED灯使用的GPIO管脚模式*/
  GPIO_InitStructure.GPIO_Pin = DS1_PIN|DS2_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIO_LED_1_2, &GPIO_InitStructure); 
  
  GPIO_InitStructure.GPIO_Pin = RED1_PIN|YELLOW_PIN|GREEN_PIN|RED2_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIO_LED_3_6, &GPIO_InitStructure); 
  
  
  
   /* Configure Volume control pins: LOAD, SCLK, DATA */
  GPIO_InitStructure.GPIO_Pin = VOLUME_LOAD_PIN | VOLUME_DATA_PIN | VOLUME_SCLK_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIO_VOLUME_CTL, &GPIO_InitStructure);
    
  GPIO_SetBits(GPIO_VOLUME_CTL, VOLUME_LOAD_PIN);//LOAD
  GPIO_SetBits(GPIO_VOLUME_CTL, VOLUME_SCLK_PIN);//SCLK
  GPIO_SetBits(GPIO_VOLUME_CTL, VOLUME_DATA_PIN);//DATA
  
}

/**
  * @brief  Configures the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
  //NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
//  /* Enable the USART1 Interrupt */
//  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
//
//  /* Enable the USART2 Interrupt */
//  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Configures the nested vectored exti interrupt controller.
  * @param  None
  * @retval None
  */

void EXTI_USER_Init(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;

  /*使能AFIO使用的GPIO时钟*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 
  
  
   //PA0 中断线以及中断初始化配置
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0);     //把GPIOA的Pin0设置为EXTI的输入线
  EXTI_InitStructure.EXTI_Line=EXTI_Line0;                       //中断线0
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	         //中断模式设置
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //双边沿触发
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);		//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
  
  //PA1 中断线以及中断初始化配置
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource1);     //把GPIOA的Pin1设置为EXTI的输入线
  EXTI_InitStructure.EXTI_Line=EXTI_Line1;                       //中断线1
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	         //中断模式设置
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //双边沿触发
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);		//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
  
  //PA2 中断线以及中断初始化配置
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource2);     //把GPIOA的Pin2设置为EXTI的输入线
  EXTI_InitStructure.EXTI_Line=EXTI_Line2;                       //中断线2
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	         //中断模式设置
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);		//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
  
   //PA3
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource3);     //把GPIOA的Pin3设置为EXTI的输入线
  EXTI_InitStructure.EXTI_Line=EXTI_Line3;                       //中断线3
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	         
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);		//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
  
  //PC6
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource6);     //把GPIOC的Pin6设置为EXTI的输入线
  EXTI_InitStructure.EXTI_Line=EXTI_Line6;                       //中断线6
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	         
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);		
  
   //PC7
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource7);     //把GPIOC的Pin7设置为EXTI的输入线
  EXTI_InitStructure.EXTI_Line=EXTI_Line7;                       //中断线7
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	         
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);		
  
   //PC8
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource8);     //把GPIOC的Pin8设置为EXTI的输入线
  EXTI_InitStructure.EXTI_Line=EXTI_Line8;                       //中断线8
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	         
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);		//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
  
     //PC9
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource9);     //把GPIOC的Pin9设置为EXTI的输入线
  EXTI_InitStructure.EXTI_Line=EXTI_Line9;                       //中断线9
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	         
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);		//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
  

  
  
  //中断线号的优先级配置
  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//使能按键所在的外部中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;          //抢占优先级1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;		        //子优先级0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitSruct中指定的参数初始化外设NVIC的寄存器
  
  
   NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;			//使能按键所在的外部中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;          //抢占优先级1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;		        //子优先级1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitSruct中指定的参数初始化外设NVIC的寄存器
  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;			//使能按键所在的外部中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;          //抢占优先级1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;		        //子优先级2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitSruct中指定的参数初始化外设NVIC的寄存器
  
          
  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;			//使能按键所在的外部中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;	        //抢占优先级1 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;			//子优先级3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure); 
  
//  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;			//使能按键所在的外部中断通道
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;	        //抢占优先级1 
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;			//子优先级4
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//使能外部中断通道
//  NVIC_Init(&NVIC_InitStructure);
    
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//使能按键所在的外部中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;	        //抢占优先级1 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x05;			//子优先级5
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);


}

/**
  * @brief  Configures the Timer3 controller
  * @param  None
  * @retval None
  */

//这里时钟选择为APB1的2倍，而APB1为36M
//arr:自动重装值
//psc:时钟预分频数
//这里使用的是定时器3
void TIM3_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能
  
  //定时器TIM3初始化
  TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
  TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割：TDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的实践基数单位

  TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断，允许更新中断

  //中断优先级NVIC设置
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //抢占是优先级0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道是能
  NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器

  TIM_Cmd(TIM3, DISABLE);  //暂时不使能TIM3	


}

/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval PASSED: pBuffer1 identical to pBuffer2
  *   FAILED: pBuffer1 differs from pBuffer2
  */
//TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
//{
//  while(BufferLength--)
//  {
//    if(*pBuffer1 != *pBuffer2)
//    {
//      return FAILED;
//    }
//
//    pBuffer1++;
//    pBuffer2++;
//  }
//
//  return PASSED;
//}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
