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

/** @addtogroup USART_Interrupt
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
typedef enum { FAILED = 0, PASSED = !FAILED} TestStatus;

typedef enum key_states_e{  
    KEY_NOT_PRESS,  //初始状态，未按下
    KEY_PRESSED,  //按下
    KEY_RELEASE,   //释放
}KeyStatus_t; 

/* Private define ------------------------------------------------------------*/
#define TxBufferSize1   (countof(TxBuffer1) - 1)
#define TxBufferSize2   (countof(TxBuffer2) - 1)
#define RxBufferSize1   TxBufferSize2
#define RxBufferSize2   TxBufferSize1

/* Private macro -------------------------------------------------------------*/
#define countof(a)   (sizeof(a) / sizeof(*(a)))

/* Private variables ---------------------------------------------------------*/
USART_InitTypeDef USART_InitStructure;
uint8_t TxBuffer1[] = "串口中断收发示例: 串口1 -> 串口2 (中断收发)";
uint8_t TxBuffer2[] = "串口中断收发示例: 串口2 -> 串口1 (中断收发)";
uint8_t RxBuffer1[RxBufferSize1];
uint8_t RxBuffer2[RxBufferSize2];
__IO uint8_t TxCounter1 = 0x00;
__IO uint8_t TxCounter2 = 0x00;
__IO uint8_t RxCounter1 = 0x00; 
__IO uint8_t RxCounter2 = 0x00;
uint8_t NbrOfDataToTransfer1 = TxBufferSize1;
uint8_t NbrOfDataToTransfer2 = TxBufferSize2;
uint8_t NbrOfDataToRead1 = RxBufferSize1;
uint8_t NbrOfDataToRead2 = RxBufferSize2;
__IO TestStatus TransferStatus1 = FAILED; 
__IO TestStatus TransferStatus2 = FAILED;

static u8  fac_us=0;//us微秒延时倍乘数
static u16 fac_ms=0;//ms毫秒延时倍乘数


//列如按下键为key0,则短按键值为0xA0, 长按键值为0xB0,释放键值为0xC0
volatile u8 press_counter = 0;
static volatile u8  pressed_row_number =0;
static volatile u16 current_key_value = 0x99;//当前按键值
static volatile u16 last_key_value = 0x99;//上次按键值

static volatile u16 pressed_value =0x99;//按下后返回的键值（包括短按长按）
static volatile u16 release_value =0x99;//释放返回的键值

static KeyStatus_t KeyStatus = KEY_NOT_PRESS;
volatile u8 row1_pressed_flag = 0;//第一列
volatile u8 row2_pressed_flag = 0;//第二列
volatile u8 row3_pressed_flag = 0;//第三列
volatile u8 row4_pressed_flag = 0;//第四列

 NVIC_InitTypeDef NVIC_InitStructure;

/* Private function prototypes -----------------------------------------------*/
void USART1_Init(void);
void KEY_Init(void);
void NVIC_Configuration(void);
void EXTI_USER_Init(void);
void TIM3_Int_Init(u16 arr,u16 psc);
void delay_init(void);
void delay_ms(u16 xms);
void delay_us(u32 xus);

TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);

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
   
  delay_init();	    	 //延时函数初始化	

  NVIC_Configuration();//设置中断优先级分组
  
  KEY_Init();//初始化按键的IO模式
  
  EXTI_USER_Init();//外部中断初始化，在这里初始化4列对应的中断输入
  
  USART1_Init();//串口1初始化,带打印调试
  
  TIM3_Int_Init(100-1, 7199);//定时器3初始化,10Khz的计数频率，计数到100为10ms  
  
  GPIO_SetBits(GPIO_LED, DS1_PIN|DS2_PIN|DS3_PIN|DS4_PIN);/*关闭所有的LED指示灯*/
  
  printf("\r\n/***********************key scan start*********************/\r\n");
  while(1)
  {
    switch(KeyStatus)
    {
      case KEY_NOT_PRESS:
        
        //有键值被按下
        if(row1_pressed_flag | row2_pressed_flag |row3_pressed_flag |row4_pressed_flag)
        {
          //扫描行
          GPIO_ResetBits(GPIO_KEY_INTERFACE, COL1_PIN);//第一行置0
          GPIO_SetBits(GPIO_KEY_INTERFACE, COL2_PIN);//第二行置1
          pressed_row_number = row1_pressed_flag + row2_pressed_flag 
            + row3_pressed_flag +row4_pressed_flag;//列计算
          switch(pressed_row_number)
          {
            case 1:
              
              if(GPIO_ReadInputDataBit(GPIO_KEY_INTERFACE, ROW1_PIN) ==0)
                current_key_value =0x00;
              else
                current_key_value =0x04;
              
              break;
              
            case 2:
              
              if(GPIO_ReadInputDataBit(GPIO_KEY_INTERFACE, ROW2_PIN) ==0)
                current_key_value =0x01;
              else
                current_key_value =0x05;
              
              break;
              
            case 3:
              
              if(GPIO_ReadInputDataBit(GPIO_KEY_INTERFACE, ROW3_PIN) ==0)
                current_key_value =0x02;
              else
                current_key_value =0x06;
              
              break;
              
            case 4:
              
              if(GPIO_ReadInputDataBit(GPIO_KEY_INTERFACE, ROW4_PIN) ==0)
                current_key_value =0x03;
              else
                current_key_value =0x07;
              
              break;
              
            default:
              printf("key pressed_row_number err !!!\n");
              KeyStatus = KEY_NOT_PRESS;
              break;
          }
          KeyStatus = KEY_PRESSED;
          TIM_Cmd(TIM3, ENABLE);  //使能TIM3，开始计数
        }
        
        break;
        
    case KEY_PRESSED:
      
         GPIO_ResetBits(GPIO_KEY_INTERFACE, COL1_PIN);//第一行置0
         GPIO_ResetBits(GPIO_KEY_INTERFACE, COL2_PIN);//第二行置0  
          switch(current_key_value)
          {
            case 0x00:
            case 0x04:
              
              if(press_counter > 40)//大于400ms的按键
              {
                if(current_key_value == 0x00)pressed_value = 0xB0;//key0长按
                else
                  pressed_value = 0xB4;//key4长按
                  
              }
              else
              {
                if(current_key_value == 0x00)pressed_value = 0xA0;//key0短按
                else
                  pressed_value = 0xA4;//key4短按        
               }
              
              if(!row1_pressed_flag)//判断按键是否为释放
                   KeyStatus = KEY_RELEASE;
                
              break;
              
           case 0x01:
           case 0x05:
             
               if(press_counter > 40)//大于400ms的按键
              {
                if(current_key_value == 0x01)pressed_value = 0xB1;//key1长按
                else
                  pressed_value = 0xB5;//key5长按
                  
              }
              else
              {
                if(current_key_value == 0x01)pressed_value = 0xA1;//key1短按
                else
                  pressed_value = 0xA5;//key5短按        
               }
               
               if(!row2_pressed_flag)
                   KeyStatus = KEY_RELEASE;
               
              break;
              
           case 0x02:
           case 0x06:
             
               if(press_counter > 40)//大于400ms的按键
              {
                if(current_key_value == 0x02)pressed_value = 0xB2;//key2长按
                else
                  pressed_value = 0xB6;//key6长按
                  
              }
              else
              {
                if(current_key_value == 0x02)pressed_value = 0xA2;//key2短按
                else
                  pressed_value = 0xA6;//key6短按        
               }
               
               if(!row3_pressed_flag)
                   KeyStatus = KEY_RELEASE;
               
              break;
              
           case 0x03:
           case 0x07:
             
               if(press_counter > 40)//大于400ms的按键
              {
                if(current_key_value == 0x03)pressed_value = 0xB3;//key3长按
                else
                  pressed_value = 0xB7;//key7长按
                  
              }
              else
              {
                if(current_key_value == 0x03)pressed_value = 0xA3;//key3短按
                else
                  pressed_value = 0xA7;//key7短按        
               }
               
               if(!row4_pressed_flag)
                   KeyStatus = KEY_RELEASE;
               
              break;
              
          default:
              printf("key current_key_value err !!!\n");
              KeyStatus = KEY_NOT_PRESS;
              break;
          }
          
          TIM_Cmd(TIM3, DISABLE);  //关闭TIM3
          press_counter = 0;//清空计数
          
        break;
                
      case KEY_RELEASE:
        
        
        switch(pressed_value)
          {
            case 0xA0:
            case 0xB0:
              release_value = 0xC0;
              break;
              
            case 0xA1:
            case 0xB1:
              release_value = 0xC1;
              break;
              
            case 0xA2:
            case 0xB2:
              release_value = 0xC2;
              break;
              
             case 0xA3:
             case 0xB3:
               release_value = 0xC3;
               break;
               
             case 0xA4:
             case 0xB4:
               release_value = 0xC4;
               break;
               
             case 0xA5:
             case 0xB5:
               release_value = 0xC5;
               break;
               
             case 0xA6:
             case 0xB6:
               release_value = 0xC6;
               break;
               
             case 0xA7:
             case 0xB7:
               release_value = 0xC7;
               break;
               
          default:
            break;
            
          }
        
        KeyStatus = KEY_NOT_PRESS;
        
        
        
        break;
        
        
      default:
        printf("key status err !!!\n");
        KeyStatus = KEY_NOT_PRESS;
        break;
     
    
    }
    //delay_ms(1);
    
    if((pressed_value!=pressed_value) && (pressed_value !=0x99))
    printf("pressed_value : 0x%x", pressed_value);
    
    if((release_value!=release_value) &&(release_value !=0x99))
    printf("release_value : 0x%x", release_value);
  
    
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
 
  /*使能串口1的发送和接收中断*/
  //USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  //USART_ITConfig(USART1, USART_IT_TC, ENABLE);
  
  /* Enable the USART1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;	        //抢占优先级0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;			//子优先级1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* 使能串口1 */
  USART_Cmd(USART1, ENABLE);
  USART_ClearFlag(USART1,USART_FLAG_TC); //清除USART_FLAG_TC，解决第一个字节不能发出的问题 
  


}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
void KEY_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  //使能时钟
  /*使能LED灯使用的GPIO时钟*/
  /*使能KEY扫描按键使用的GPIO时钟*/
  RCC_APB2PeriphClockCmd(RCC_GPIO_LED | RCC_GPIO_KEY, ENABLE);


  /* LED灯使用的GPIO管脚模式*/
  GPIO_InitStructure.GPIO_Pin = DS1_PIN|DS2_PIN|DS3_PIN|DS4_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIO_LED, &GPIO_InitStructure); 
  
  
   /* KEY行选使用的GPIO管脚模式*/
  GPIO_InitStructure.GPIO_Pin = COL1_PIN|COL2_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIO_KEY_INTERFACE, &GPIO_InitStructure); 
  
  /* KEY列选使用的GPIO管脚模式*/
  GPIO_InitStructure.GPIO_Pin = ROW1_PIN|ROW2_PIN|ROW3_PIN|ROW4_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入
  GPIO_Init(GPIO_KEY_INTERFACE, &GPIO_InitStructure); 
  
  GPIO_ResetBits(GPIO_KEY_INTERFACE, COL1_PIN|COL2_PIN);/*扫描键盘第一行，第二行清零输出*/
  
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
  
  //PA2 中断线以及中断初始化配置
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource2);     //把GPIOA的Pin2设置为EXTI的输入线
  EXTI_InitStructure.EXTI_Line=EXTI_Line2;                       //中断线2
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	         //中断模式设置
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿触发
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);		//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
  
  //PA3
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource3);     //把GPIOA的Pin3设置为EXTI的输入线
  EXTI_InitStructure.EXTI_Line=EXTI_Line3;                       //中断线3
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	         
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);		
  
   //PA4
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource4);     //把GPIOA的Pin4设置为EXTI的输入线
  EXTI_InitStructure.EXTI_Line=EXTI_Line4;                       //中断线4
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	         
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);		
  
   //PA5
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource5);     //把GPIOA的Pin5设置为EXTI的输入线
  EXTI_InitStructure.EXTI_Line=EXTI_Line5;                       //中断线5
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	         
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);		//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
  
  
  //中断线号的优先级配置
  NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;			//使能按键所在的外部中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;          //抢占优先级1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;		        //子优先级0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitSruct中指定的参数初始化外设NVIC的寄存器
  
          
  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;			//使能按键所在的外部中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;	        //抢占优先级1 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;			//子优先级1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure); 
  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;			//使能按键所在的外部中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;	        //抢占优先级1 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;			//子优先级1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);
    
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//使能按键所在的外部中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;	        //抢占优先级1 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;			//子优先级1
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
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while(BufferLength--)
  {
    if(*pBuffer1 != *pBuffer2)
    {
      return FAILED;
    }

    pBuffer1++;
    pBuffer2++;
  }

  return PASSED;
}

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

void delay_init()	 
{

  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//此处选择外部时钟，HCLK/8
  fac_us = SystemCoreClock/8000000;//fac_us定义为系统时钟的1/8	
  fac_ms = (u16)fac_us*1000; //每个ms需要的systick时钟数

}

void delay_us(u32 xus)	//延时xus个微秒
{		
    u32 temp;	    	 
    SysTick->LOAD=xus*fac_us;                         //时间加载	  		 
    SysTick->VAL=0x00;                                //清空计数器
    SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;       //设置开始倒数计时的掩码

    do
    {
            temp=SysTick->CTRL;
    }
    while(temp&0x01&&!(temp&(1<<16)));            //等待时间到达  
            SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //关闭计数器

    SysTick->VAL =0X00;                            //清空计数器	 
}

//延时xms 72mhz下，xms<=1864
void delay_ms(u16 xms)
{	 		  	  
    u32 temp;		   
    SysTick->LOAD=(u32)xms*fac_ms; //时间加载
    SysTick->VAL =0x00;            //清空计数器
    SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //开始倒数 
    do
    {
            temp=SysTick->CTRL;
    }
    while(temp&0x01&&!(temp&(1<<16)));//等待时间到达哦
    SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //关闭计数器
    SysTick->VAL =0X00;       //清空计数器  	    
} 
/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
