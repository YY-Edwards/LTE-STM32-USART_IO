/**
  ******************************************************************************
  * @file    USART/Interrupt/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.3.0
  * @date    04/16/2010
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
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
#include "stm32f10x_it.h"
#include "platform_config.h"
#include "pro_slip.h"

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup USART_Interrupt
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

extern volatile u16 key1_pressed_flag;//第一列
extern volatile u16 key2_pressed_flag;//第二列
extern volatile u16 key3_pressed_flag;//第三列
extern volatile u16 key4_pressed_flag;//第四列
extern volatile u16 key5_pressed_flag;//第key5
extern volatile u16 key6_pressed_flag;//第key6
extern volatile u16 key7_pressed_flag;//第key7
extern volatile u16 key8_pressed_flag;//key8

extern volatile u8 press_counter;
extern void delay_ms(u16 xms);
extern volatile u8 time_out;

static u16 key_value = 0x0000;
extern u8 USART_RX[];


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
  * @brief  This function handles USART1 global interrupt request.
  * @param  None
  * @retval None
  */
u8 RxBuffer1[20];

void USART1_IRQHandler(void)
{
  uint16_t temp = 0;  
  uint16_t i = 0;
  static u8 RxCounter1 =0;
  
  u8 Rebuf[256]={0};
  
  if(USART_GetITStatus(USART1, USART_IT_PE | USART_IT_FE | USART_IT_NE) != RESET)//出错
    {   
        USART_ClearITPendingBit(USART1, USART_IT_PE | USART_IT_FE | USART_IT_NE);
        //printf("\n\r Usart1 err....\n\r");
    
    }
  
    if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)//空闲接收串口数据
    {
        DMA_Cmd(DMA1_Channel5,DISABLE);						//关闭DMA1通道5（USART1_RX） 
        //printf("\n\r-------Usart1_Clear_Flag\n");
        USART_ClearITPendingBit(USART1,USART_IT_RXNE);//这个也许是需要的哦。。。
        temp = USART1->SR;	   //貌似根据手册说，先读SR，再读DR就可以清除IDLE位。。。。
        temp = USART1->DR;	        
        temp = 256 - DMA_GetCurrDataCounter(DMA1_Channel5);	//缓冲器数量够大//用缓冲器的设定值-当前指针数值（寄存器内容在每次DMA传输后递减）=接收的数据长度。 
        //printf("\n\r-------Usart1 rx counter: %d\n\r",temp);     
        
//        RxCounter1=RxCounter1+temp;//计数
//         for(i=0; i<temp; i++)
//        {
//
//              Rebuf[i]=USART_RX[i]; //数据转移
//              //printf("\n--usart1 rx data is: 0x%02x\n",Rebuf[i]);
//            
//        }
//        if(RxCounter1>=12){
//        
//          packet_analysis(Rebuf, RxCounter1);
//          RxCounter1 = 0;
//          memset(Rebuf, 0x00, sizeof(Rebuf));
//        }
        
        memcpy(Rebuf, USART_RX, temp);
        packet_analysis(Rebuf, temp);
         
        //memset(Rebuf, 0x00, sizeof(Rebuf));
        memset(USART_RX, 0x00, 256);
        //设置传输数据长度  
        DMA_SetCurrDataCounter(DMA1_Channel5, 256);//即是通道可容纳的最大数据量。           
        //重新打开DMA1_5（USART1_RX）  
        DMA_Cmd(DMA1_Channel5,ENABLE);

      //printf("\n\r Usart1 analysis end and exit....\n\r");
    
    }
  
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
    /* Read one byte from the receive data register */
    RxBuffer1[RxCounter1++] = USART_ReceiveData(USART1);

    if(RxCounter1 == 12)
    {
      /* Disable the USART1 Receive interrupt */
      //printf("\n\r Usart1 analysis data...\n\r");
      packet_analysis(RxBuffer1, RxCounter1);
      RxCounter1 = 0;
      memset(RxBuffer1, 0x00 , 20);
      
    }
  }
  
}


/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
//定时器3中断服务程序
void TIM3_IRQHandler(void)   //TIM3中断
{
  if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
     {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //清除TIMx更新中断标志 
        //press_counter++;//计数
        time_out =1;
      }
}


/**
  * @brief  This function handles EXTI0 global interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
  static volatile u8 ReadValue =1;
  //key_value = 0xA001;
   key_value = 0xA008;
  if(EXTI_GetITStatus(EXTI_Line0)!= RESET)
  {
    delay_ms(15);//延时防抖动
    ReadValue = GPIO_ReadInputDataBit(GPIO_KEY_5_8, KEY8_PIN);
    if(ReadValue == 0)//key8按下
    {
      if(!key8_pressed_flag){
        key8_pressed_flag = 0x0001;
        data_packet_send(key_value, key8_pressed_flag);
        //printf("\r\nkey1 pressed\r\n");
        
      }
 
    }
    else//key8释放
    {
      if(key8_pressed_flag){       
        key8_pressed_flag = 0x0000;
        data_packet_send(key_value, key8_pressed_flag);
        //printf("\r\n**key1 release\r\n");
        
      }
    
    }
        
    
    EXTI_ClearITPendingBit(EXTI_Line0);
  }
  
}

/**
  * @brief  This function handles EXTI1 global interrupt request.
  * @param  None
  * @retval None
  */
void EXTI1_IRQHandler(void)
{
  static volatile u8 ReadValue =1;
  //key_value = 0xA002;
   key_value = 0xA007;
  
  if(EXTI_GetITStatus(EXTI_Line1)!= RESET)
  {
    delay_ms(15);//延时防抖动
    ReadValue = GPIO_ReadInputDataBit(GPIO_KEY_5_8, KEY7_PIN);
    if(ReadValue == 0)//key7按下
    {
      if(!key7_pressed_flag){
        key7_pressed_flag = 0x0001;
        data_packet_send(key_value, key7_pressed_flag);
        //printf("key2 pressed\r\n");
      }
    }
    else//key7释放
    {
      if(key7_pressed_flag){
        key7_pressed_flag = 0x0000;
        data_packet_send(key_value, key7_pressed_flag);
        //printf("key2 release\r\n");
      }
    
    }      
    
    EXTI_ClearITPendingBit(EXTI_Line1);
  }
  
}


/**
  * @brief  This function handles EXTI2 global interrupt request.
  * @param  None
  * @retval None
  */
void EXTI2_IRQHandler(void)
{
   static volatile u8 ReadValue =1;
   //key_value = 0xA003;
    key_value = 0xA006;
   
  if(EXTI_GetITStatus(EXTI_Line2)!= RESET)
  {
    delay_ms(15);//延时防抖动
    ReadValue = GPIO_ReadInputDataBit(GPIO_KEY_5_8, KEY6_PIN);
    if(ReadValue == 0)//key6按下
    {
      if(!key6_pressed_flag){
        key6_pressed_flag = 0x0001;
        data_packet_send(key_value, key6_pressed_flag);
        
       //printf("key3 pressed\r\n");
      }
    }
    else//key6放
    {
      if(key6_pressed_flag){
        key6_pressed_flag = 0x0000;
        data_packet_send(key_value, key6_pressed_flag);
      //  printf("key3 release\r\n");
      }
    
    }
        
    
    EXTI_ClearITPendingBit(EXTI_Line2);
  }
  
}

/**
  * @brief  This function handles EXTI3 global interrupt request.
  * @param  None
  * @retval None
  */
void EXTI3_IRQHandler(void)
{
   static volatile u8 ReadValue =1;
   //key_value = 0xA004;
    key_value = 0xA005;
   
  if(EXTI_GetITStatus(EXTI_Line3)!= RESET)
  {
    delay_ms(10);//延时防抖动
    ReadValue = GPIO_ReadInputDataBit(GPIO_KEY_5_8, KEY5_PIN);
   if(ReadValue == 0)//key5按下
    {
      if(!key5_pressed_flag){
        key5_pressed_flag = 0x0001;
        data_packet_send(key_value, key5_pressed_flag);
       //printf("key4 pressed\r\n");
      }
    }
    else//key5放
    {
      if(key5_pressed_flag){
        key5_pressed_flag = 0x0000;
        data_packet_send(key_value, key5_pressed_flag);
      //  printf("key4 release\r\n");
      }
    
    }
        
    
    EXTI_ClearITPendingBit(EXTI_Line3);
  }
  
  
}


/**
  * @brief  This function handles EXTI4 global interrupt request.
  * @param  None
  * @retval None
  */
void EXTI4_IRQHandler(void)
{
   static volatile u8 ReadValue =1;
   //key_value = 0xA005;
    key_value = 0xA004;
//   
//  if(EXTI_GetITStatus(EXTI_Line4)!= RESET)
//  {
//    delay_ms(10);//延时防抖动
//    ReadValue = GPIO_ReadInputDataBit(GPIO_KEY, KEY5_PIN);
//    if(ReadValue == 0)//key5按下
//    {
//      if(!key5_pressed_flag){
//        
//        key5_pressed_flag = 0x0001;
//        data_packet_send(key_value, key5_pressed_flag);
//        
//       //printf("key5 pressed\r\n");
//      }
//    }
//    else//key5放
//    {
//      if(key5_pressed_flag){
//        
//        key5_pressed_flag = 0x0000;
//        data_packet_send(key_value, key5_pressed_flag);
//      //  printf("key5 release\r\n");
//      }
//    
//    }
//           
//    EXTI_ClearITPendingBit(EXTI_Line4);
//  }
//  
  
}


/**
  * @brief  This function handles EXTI9_5 global interrupt request.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{
  
  static volatile u8 ReadValue =1;
  
//  if(EXTI_GetITStatus(EXTI_Line5)!= RESET)
//  {
//    //key_value = 0xA006;
//     key_value = 0xA003;
//    delay_ms(15);//延时防抖动
//    ReadValue = GPIO_ReadInputDataBit(GPIO_KEY, KEY6_PIN);
//   if(ReadValue == 0)//key6按下
//    {
//      if(!key6_pressed_flag){
//        key6_pressed_flag = 0x0001;
//        data_packet_send(key_value, key6_pressed_flag);
//       //printf("key6 pressed\r\n");
//      }
//    }
//    else//key6放
//    {
//      if(key6_pressed_flag){
//        
//        key6_pressed_flag = 0x0000;
//        data_packet_send(key_value, key6_pressed_flag);
//      //  printf("key6 release\r\n");
//      }
//    
//    }
//          
//    EXTI_ClearITPendingBit(EXTI_Line5);
//  }
  
  if(EXTI_GetITStatus(EXTI_Line6)!= RESET)
  {
    //key_value = 0xA007;
     key_value = 0xA004;
    delay_ms(15);//延时防抖动
    ReadValue = GPIO_ReadInputDataBit(GPIO_KEY_1_4, KEY4_PIN);
    if(ReadValue == 0)//key4按下
    {
      if(!key4_pressed_flag){
        key4_pressed_flag = 0x0001;
        data_packet_send(key_value, key4_pressed_flag);
       //printf("key7 pressed\r\n");
      }
    }
    else//key4放
    {
      if(key4_pressed_flag){
        key4_pressed_flag = 0x0000;
        data_packet_send(key_value, key4_pressed_flag);
      //  printf("key7 release\r\n");
      }
    
    }
         
    EXTI_ClearITPendingBit(EXTI_Line6);
  }
  
  if(EXTI_GetITStatus(EXTI_Line7)!= RESET)
  {
    //key_value = 0xA008;
     key_value = 0xA003;
    delay_ms(15);//延时防抖动
    ReadValue = GPIO_ReadInputDataBit(GPIO_KEY_1_4, KEY3_PIN);
    if(ReadValue == 0)//key3按下
    {
      if(!key3_pressed_flag){
        key3_pressed_flag = 0x0001;
        data_packet_send(key_value, key3_pressed_flag);
        //printf("key8 pressed\r\n");
      }
    }
    else//key3释放
    {
      if(key3_pressed_flag){
        key3_pressed_flag = 0x0000;
        data_packet_send(key_value, key3_pressed_flag);
      //printf("key8 release\r\n");
      }
    }
     EXTI_ClearITPendingBit(EXTI_Line7);
  }
    
   if(EXTI_GetITStatus(EXTI_Line8)!= RESET)
  {
    key_value = 0xA002;
    delay_ms(15);//延时防抖动
    ReadValue = GPIO_ReadInputDataBit(GPIO_KEY_1_4, KEY2_PIN);
    if(ReadValue == 0)//key2按下
    {
      if(!key2_pressed_flag){
        key2_pressed_flag = 0x0001;
        data_packet_send(key_value, key2_pressed_flag);
        //printf("\r\nkey9 pressed\r\n");
      }
 
    }
    else//key2释放
    {
      if(key2_pressed_flag){   
       key2_pressed_flag = 0x0000;
        data_packet_send(key_value, key2_pressed_flag);
        //printf("\r\n**key9 release\r\n");
      }
    
    }
  
    EXTI_ClearITPendingBit(EXTI_Line8);
  }
  
   
   if(EXTI_GetITStatus(EXTI_Line9)!= RESET)
  {
    key_value = 0xA001;
    delay_ms(15);//延时防抖动
    ReadValue = GPIO_ReadInputDataBit(GPIO_KEY_1_4, KEY1_PIN);
    if(ReadValue == 0)//key1按下
    {
      if(!key1_pressed_flag){
        key1_pressed_flag = 0x0001;
        data_packet_send(key_value, key1_pressed_flag);
        //printf("\r\nkey9 pressed\r\n");
      }
 
    }
    else//key1释放
    {
      if(key1_pressed_flag){   
       key1_pressed_flag = 0x0000;
        data_packet_send(key_value, key1_pressed_flag);
        //printf("\r\n**key9 release\r\n");
      }
    
    }
  
    EXTI_ClearITPendingBit(EXTI_Line9);
  }
  
 

}


/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
