#ifndef _HARDWARE_H_
#define _HARDWARE_H_
#include "pro_slip.h"
#include "platform_config.h"


/* Private function prototypes -----------------------------------------------*/

void USART2_Init(void);
void USART1_Init(void);
void LTE_GPIO_Init(void);
void NVIC_Configuration(void);
void EXTI_USER_Init(void);
void TIM3_Int_Init(u16 arr,u16 psc);
void Write_Volume(unsigned short dat);
int simple_uart_put(u8 ch);
void Hardware_Init(void);

#endif