/**
  ******************************************************************************
  * @file    keysanf_usart/platform_config.h 
  * @author  EDWARDS
  * @version V1.0.0
  * @date    05/03/2017
  * @brief   Evaluation board specific configuration file.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PLATFORM_CONFIG_H
#define __PLATFORM_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Uncomment the line corresponding to the STMicroelectronics evaluation board
   used to run the example */
#if !defined (USE_STM3210B_EVAL) &&  !defined (USE_STM3210E_EVAL) &&  !defined (USE_STM3210C_EVAL) &&  !defined (USE_STM32100B_EVAL)
 #define USE_STM3210E_EVAL
#endif

#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */   

    
    
/* Define the STM32F10x hardware depending on the used evaluation board */


  #define USART1_GPIO              GPIOA
  #define USART1_CLK               RCC_APB2Periph_USART1
  #define USART1_GPIO_CLK          RCC_APB2Periph_GPIOA
  #define USART1_RxPin             GPIO_Pin_10
  #define USART1_TxPin             GPIO_Pin_9
  #define USART1_IRQn              USART1_IRQn
  #define USART1_IRQHandler        USART1_IRQHandler

  #define USART2_GPIO              GPIOA
  #define USART2_CLK               RCC_APB1Periph_USART2
  #define USART2_GPIO_CLK          RCC_APB2Periph_GPIOA
  #define USART2_RxPin             GPIO_Pin_3
  #define USART2_TxPin             GPIO_Pin_2
  #define USART2_IRQn              USART2_IRQn
  #define USART2_IRQHandler        USART2_IRQHandler
  
  
/*LED灯相关定义*/
  #define RCC_GPIO_LED                    RCC_APB2Periph_GPIOF    /*LED使用的GPIO时钟*/
  #define LEDn                            4                       /*LED数量*/
  #define GPIO_LED                        GPIOF  //GPIOC           /*LED灯使用的GPIO组*/
  
  #define DS1_PIN                         GPIO_Pin_6              /*DS1使用的GPIO管脚*/
  #define DS2_PIN                         GPIO_Pin_7		  /*DS2使用的GPIO管脚*/
  #define DS3_PIN                         GPIO_Pin_8  	          /*DS3使用的GPIO管脚*/
  #define DS4_PIN                         GPIO_Pin_9		  /*DS4使用的GPIO管脚*/

/*KEY独立按键相关定义*///8
  #define RCC_GPIO_KEY                    RCC_APB2Periph_GPIOA    /*KEY使用的GPIO时钟*/
  #define KEYn                            8              /*KEY接口数量*/
  #define GPIO_KEY                        GPIOA                   /*KEY灯使用的GPIO组*/
  
  #define KEY1_PIN                         GPIO_Pin_0              /*K1使用的GPIO管脚*/
  #define KEY2_PIN                         GPIO_Pin_1		  /*K2使用的GPIO管脚*/
  #define KEY3_PIN                         GPIO_Pin_2 	          /*K3使用的GPIO管脚*/
  #define KEY4_PIN                         GPIO_Pin_3		  /*K4使用的GPIO管脚*/
  #define KEY5_PIN                         GPIO_Pin_4 	          /*K5使用的GPIO管脚*/
  #define KEY6_PIN                         GPIO_Pin_5		  /*K6使用的GPIO管脚*/
  #define KEY7_PIN                         GPIO_Pin_6		  /*K7使用的GPIO管脚*/
  #define KEY8_PIN                         GPIO_Pin_8//		  /*K8使用的GPIO管脚*/


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __PLATFORM_CONFIG_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
