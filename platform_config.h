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
#include <sys.h>
#include <delay.h>
#include "myqueue.h"
#include "pro_slip.h"
//#incldue <i2c_tpa2016.h>
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

  #define SUCCESS 1
  #define FAILURE 0


  #define MAX_RX_DEEP 512   
  #define CommandQueueDeep 20
     
     
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
  #define RCC_GPIO_LED_1_2                    RCC_APB2Periph_GPIOC    /*LED指示灯使用的GPIO时钟*/
  #define RCC_GPIO_LED_3_6                    RCC_APB2Periph_GPIOB    /*LED信号灯使用的GPIO时钟*/
  #define LEDn                                  6                       /*LED数量*/
  #define GPIO_LED_1_2                        GPIOC                     /*LED指示灯使用的GPIO组*/
  #define GPIO_LED_3_6                        GPIOB                     /*LED信号灯使用的GPIO组*/
  
  #define DS1_PIN                         GPIO_Pin_0              /*DS1使用的GPIO管脚*/
  #define DS2_PIN                         GPIO_Pin_1		  /*DS2使用的GPIO管脚*/
  #define RED1_PIN                        GPIO_Pin_6  	          /*DS3使用的GPIO管脚*/
  #define YELLOW_PIN                      GPIO_Pin_7		  /*DS4使用的GPIO管脚*/
  #define GREEN_PIN                       GPIO_Pin_8  	          /*DS3使用的GPIO管脚*/
  #define RED2_PIN                        GPIO_Pin_9		  /*DS4使用的GPIO管脚*/
    
/*GPIO_LKJ相关定义*/
  #define RCC_GPIO_LKJ_1_4                RCC_APB2Periph_GPIOC   /*LKJ_1-4使用的GPIO时钟*/
  #define RCC_GPIO_LKJ_5_6                RCC_APB2Periph_GPIOA   /*LKJ_5-6使用的GPIO时钟*/
  #define LKJn                            6                       /*LKJ数量*/
  #define GPIO_LKJ_1_4                    GPIOC                   /*LKJ_1-4灯使用的GPIO组*/
  #define GPIO_LKJ_5_6                    GPIOA                  /*LKJ_5-6灯使用的GPIO组*/
  
  #define LKJ1_PIN                         GPIO_Pin_2             /*LKJ1使用的GPIO管脚*/
  #define LKJ2_PIN                         GPIO_Pin_3		  /*LKJ2使用的GPIO管脚*/
  #define LKJ3_PIN                         GPIO_Pin_4  	          /*LKJ3使用的GPIO管脚*/
  #define LKJ4_PIN                         GPIO_Pin_5		  /*LKJ4使用的GPIO管脚*/
  #define LKJ5_PIN                         GPIO_Pin_7  	          /*LKJ5使用的GPIO管脚*/
  #define LKJ6_PIN                         GPIO_Pin_8		  /*LKJ6使用的GPIO管脚*/    


/*KEY独立按键相关定义*///8
  #define RCC_GPIO_KEY_5_8                    RCC_APB2Periph_GPIOA    /*KEY_5-8使用的GPIO时钟*/
  #define RCC_GPIO_KEY_1_4                    RCC_APB2Periph_GPIOC    /*KEY_1-4使用的GPIO时钟*/
  #define KEYn                                   8              /*KEY接口数量*/
  #define GPIO_KEY_1_4                        GPIOC                   /*KEY_5-8灯使用的GPIO组*/
  #define GPIO_KEY_5_8                        GPIOA                   /*KEY_1-4灯使用的GPIO组*/
  
  #define KEY1_PIN                         GPIO_Pin_9              /*K1使用的GPIO管脚*/
  #define KEY2_PIN                         GPIO_Pin_8		  /*K2使用的GPIO管脚*/
  #define KEY3_PIN                         GPIO_Pin_7 	          /*K3使用的GPIO管脚*/
  #define KEY4_PIN                         GPIO_Pin_6		  /*K4使用的GPIO管脚*/
  #define KEY5_PIN                         GPIO_Pin_3 	          /*K5使用的GPIO管脚*/
  #define KEY6_PIN                         GPIO_Pin_2		  /*K6使用的GPIO管脚*/
  #define KEY7_PIN                         GPIO_Pin_1		  /*K7使用的GPIO管脚*/
  #define KEY8_PIN                         GPIO_Pin_0		  /*K8使用的GPIO管脚*/

/*LM1971接口相关定义*/
#define RCC_GPIO_VOLUME_CTL             RCC_APB2Periph_GPIOB          /*CTL使用的GPIO时钟*/
#define CTLn                            3                             /*CTL数量*/
#define GPIO_VOLUME_CTL                 GPIOB                      /*VOLUME_CTL灯使用的GPIO组*/

#define VOLUME_LOAD_PIN                         GPIO_Pin_13              /*LOAD使用的GPIO管脚*/
#define VOLUME_DATA_PIN                         GPIO_Pin_14		  /*DATA使用的GPIO管脚*/
#define VOLUME_SCLK_PIN                         GPIO_Pin_15  	          /*SCLK使用的GPIO管脚*/

#define LOAD PBout(13)
#define SCLK PBout(15)
#define DATA PBout(14)
     
#define LED2 PCout(1)    





/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __PLATFORM_CONFIG_H */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
