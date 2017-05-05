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
extern uint8_t TxBuffer1[]; 
extern uint8_t TxBuffer2[]; 
extern uint8_t RxBuffer1[];
extern uint8_t RxBuffer2[];
extern __IO uint8_t TxCounter1;
extern __IO uint8_t TxCounter2;
extern __IO uint8_t RxCounter1; 
extern __IO uint8_t RxCounter2;
extern uint8_t NbrOfDataToTransfer1;
extern uint8_t NbrOfDataToTransfer2;
extern uint8_t NbrOfDataToRead1;
extern uint8_t NbrOfDataToRead2;

extern volatile u8 key1_pressed_flag;//第一列
extern volatile u8 key2_pressed_flag;//第二列
extern volatile u8 key3_pressed_flag;//第三列
extern volatile u8 key4_pressed_flag;//第四列
extern volatile u8 key5_pressed_flag;//第key5
extern volatile u8 key6_pressed_flag;//第key6
extern volatile u8 key7_pressed_flag;//第key7
extern volatile u8 key8_pressed_flag;//key8

extern volatile u8 press_counter;
extern void delay_ms(u16 xms);
extern volatile u8 time_out;


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
void USART1_IRQHandler(void)
{
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
    /* Read one byte from the receive data register */
    //RxBuffer1[RxCounter1++] = USART_ReceiveData(USART1);

    if(RxCounter1 == NbrOfDataToRead1)
    {
      /* Disable the USART1 Receive interrupt */
      //USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
    }
  }
  
  if(USART_GetITStatus(USART1, USART_IT_TC) != RESET)
  {   
    /* Write one byte to the transmit data register */
    //USART_SendData(USART1, TxBuffer1[TxCounter1++]);

    if(TxCounter1 == NbrOfDataToTransfer1)
    {
      /* Disable the USART1 Transmit interrupt */
      //USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
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
  
  if(EXTI_GetITStatus(EXTI_Line0)!= RESET)
  {
    delay_ms(15);//延时防抖动
    ReadValue = GPIO_ReadInputDataBit(GPIO_KEY, KEY1_PIN);
    if(ReadValue == 0)//key1按下
    {
      if(!key1_pressed_flag){
        key1_pressed_flag =1;
        printf("key1 pressed\r\n");
      }
 
    }
    else//key1释放
    {
      if(key1_pressed_flag){       
        key1_pressed_flag =0;
        printf("**key1 release\r\n");
        
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
  if(EXTI_GetITStatus(EXTI_Line1)!= RESET)
  {
    delay_ms(15);//延时防抖动
    ReadValue = GPIO_ReadInputDataBit(GPIO_KEY, KEY2_PIN);
    if(ReadValue == 0)//key2按下
    {
      if(!key2_pressed_flag){
      key2_pressed_flag =1;
      //printf("key2 pressed\r\n");
      }
    }
    else//key2释放
    {
      if(key2_pressed_flag){
        key2_pressed_flag =0;
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
  if(EXTI_GetITStatus(EXTI_Line2)!= RESET)
  {
    delay_ms(15);//延时防抖动
    ReadValue = GPIO_ReadInputDataBit(GPIO_KEY, KEY3_PIN);
    if(ReadValue == 0)//key3按下
    {
      if(!key3_pressed_flag){
        key3_pressed_flag =1;
       //printf("key3 pressed\r\n");
      }
    }
    else//key3放
    {
      if(key3_pressed_flag){
        key3_pressed_flag =0;
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
  if(EXTI_GetITStatus(EXTI_Line3)!= RESET)
  {
    delay_ms(10);//延时防抖动
    ReadValue = GPIO_ReadInputDataBit(GPIO_KEY, KEY4_PIN);
   if(ReadValue == 0)//key3按下
    {
      if(!key4_pressed_flag){
        key4_pressed_flag =1;
       //printf("key4 pressed\r\n");
      }
    }
    else//key3放
    {
      if(key4_pressed_flag){
        key4_pressed_flag =0;
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
  if(EXTI_GetITStatus(EXTI_Line4)!= RESET)
  {
    delay_ms(10);//延时防抖动
    ReadValue = GPIO_ReadInputDataBit(GPIO_KEY, KEY5_PIN);
    if(ReadValue == 0)//key5按下
    {
      if(!key5_pressed_flag){
        key5_pressed_flag =1;
       //printf("key5 pressed\r\n");
      }
    }
    else//key5放
    {
      if(key5_pressed_flag){
        key5_pressed_flag =0;
      //  printf("key5 release\r\n");
      }
    
    }
           
    EXTI_ClearITPendingBit(EXTI_Line4);
  }
  
  
}


/**
  * @brief  This function handles EXTI9_5 global interrupt request.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{
  
  static volatile u8 ReadValue =1;
  
  if(EXTI_GetITStatus(EXTI_Line5)!= RESET)
  {
    delay_ms(15);//延时防抖动
    ReadValue = GPIO_ReadInputDataBit(GPIO_KEY, KEY6_PIN);
   if(ReadValue == 0)//key6按下
    {
      if(!key6_pressed_flag){
        key6_pressed_flag =1;
       //printf("key6 pressed\r\n");
      }
    }
    else//key6放
    {
      if(key6_pressed_flag){
        key6_pressed_flag =0;
      //  printf("key6 release\r\n");
      }
    
    }
          
    EXTI_ClearITPendingBit(EXTI_Line5);
  }
  
  if(EXTI_GetITStatus(EXTI_Line6)!= RESET)
  {
    delay_ms(15);//延时防抖动
    ReadValue = GPIO_ReadInputDataBit(GPIO_KEY, KEY7_PIN);
    if(ReadValue == 0)//key7按下
    {
      if(!key7_pressed_flag){
        key7_pressed_flag =1;
       //printf("key7 pressed\r\n");
      }
    }
    else//key7放
    {
      if(key7_pressed_flag){
        key7_pressed_flag =0;
      //  printf("key7 release\r\n");
      }
    
    }
         
    EXTI_ClearITPendingBit(EXTI_Line6);
  }
  
  if(EXTI_GetITStatus(EXTI_Line7)!= RESET)
  {
    delay_ms(15);//延时防抖动
    ReadValue = GPIO_ReadInputDataBit(GPIO_KEY, KEY8_PIN);
    if(ReadValue == 0)//key8按下
    {
      if(!key8_pressed_flag){
        key8_pressed_flag =1;
        //printf("key8 pressed\r\n");
      }
    }
    else//key8释放
    {
      if(key8_pressed_flag){
      key8_pressed_flag =0;
      //printf("key8 release\r\n");
      }
    }
  }
    
   if(EXTI_GetITStatus(EXTI_Line8)!= RESET)
  {
    delay_ms(15);//延时防抖动
    ReadValue = GPIO_ReadInputDataBit(GPIO_KEY, KEY8_PIN);
    if(ReadValue == 0)//key8按下
    {
      if(!key8_pressed_flag){
        key8_pressed_flag =1;
        printf("key8 pressed\r\n");
      }
 
    }
    else//key8释放
    {
      if(key8_pressed_flag){   
        key8_pressed_flag =0;
        printf("**key8 release\r\n");
      }
    
    }
  
    EXTI_ClearITPendingBit(EXTI_Line8);
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
