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
#include "platform_config.h"
#include "hardware.h"


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


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define countof(a)   (sizeof(a) / sizeof(*(a)))
/* Private variables ---------------------------------------------------------*/
extern Queue_t QueueCommand;
volatile u8 first_set_volume_flag =1;
unsigned long * QueueCommand_lock=NULL;
    
//unsigned long QueueCommand_lockInstance, * QueueCommand_lock = &QueueCommand_lockInstance;

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
  
  QueueCommand_lock = sem_init(true);    //初始化信号互斥锁
  
  ScanKeyProtocol_t Command , * pCommand = &Command;
  memset(pCommand, 0x00, sizeof(ScanKeyProtocol_t));
  
  static unsigned long return_value=0;
  
  QueueSta_t queue_return = queue_empty;
  
  delay_init(72);	    	 //延时函数初始化	

  Hardware_Init();               //硬件接口初始化
  //delay_ms(1500); 
  //delay_ms(1500); 

  while(1)   
  {         
      return_value=sem_get(QueueCommand_lock);//lock
      if(return_value)
      {            
        queue_return=QueuePull(QueueCommand, pCommand);  
        sem_free(QueueCommand_lock);//unlock      
      }     
     
                         
      if(queue_ok == queue_return)
      {
         if(   pCommand->Header == 0xABCD
            && pCommand->Terminator == 0x00BA
            && pCommand->Checksum == pCommand->KeyValue + pCommand->Status)
         {
           
           switch(pCommand->Status)
           {
             case VolumeControl:
                SetVolume(pCommand->KeyValue & 0xFF);
                break;
             case LedDisplayOrLKJControl:
               if((pCommand->KeyValue & 0x0F) > 0)SetLedDisplay(pCommand->KeyValue & 0xFF, (pCommand->KeyValue>> 8) & 0xFF);
               else if((pCommand->KeyValue & 0xF0) > 0)SetLKJValue(pCommand->KeyValue & 0xFF, (pCommand->KeyValue>> 8) & 0xFF);
              break;
             default:break;          
           }
         }
         else
         {         
           // checksum error
         }  

         queue_return = queue_empty;//reset status
         memset(pCommand, 0x00, sizeof(ScanKeyProtocol_t));//clear buffer
         
     }   
     else
     {
   
        GPIO_ResetBits(GPIO_LED_1_2, DS1_PIN);
        delay_ms(100); 
        GPIO_SetBits(GPIO_LED_1_2, DS1_PIN);
        delay_ms(100); 
        //GPIO_ResetBits(GPIO_LED, DS2_PIN);
        //delay_ms(200); 
        //GPIO_SetBits(GPIO_LED, DS2_PIN);
        //delay_ms(100);   
        if(first_set_volume_flag)
        {
            Write_Volume(0x0006);//set default volume at 5
            first_set_volume_flag =0;
            GPIO_ResetBits(GPIO_LED_3_6, RED1_PIN|YELLOW_PIN|GREEN_PIN|RED2_PIN);/*点亮所有的信号灯*///输出低电平
        }
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
        
//  sem_destory(QueueCommand_lock);
  
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


/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2017 shjh *****END OF FILE****/
