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
    KEY_NOT_PRESS,  //��ʼ״̬��δ����
    KEY_PRESSED,  //����
    KEY_RELEASE,   //�ͷ�
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
uint8_t TxBuffer1[] = "�����ж��շ�ʾ��: ����1 -> ����2 (�ж��շ�)";
uint8_t TxBuffer2[] = "�����ж��շ�ʾ��: ����2 -> ����1 (�ж��շ�)";
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

static u8  fac_us=0;//us΢����ʱ������
static u16 fac_ms=0;//ms������ʱ������


//���簴�¼�Ϊkey0,��̰���ֵΪ0xA0, ������ֵΪ0xB0,�ͷż�ֵΪ0xC0
volatile u8 press_counter = 0;
volatile u8 time_out = 0;
static volatile u16 current_key_value = 0x99;//��ǰ����ֵ
static volatile u16 last_key_value = 0x99;//�ϴΰ���ֵ

static volatile u16 pressed_value =0x99;//���º󷵻صļ�ֵ�������̰�������
static volatile u16 release_value =0x99;//�ͷŷ��صļ�ֵ

static KeyStatus_t KeyStatus = KEY_NOT_PRESS;

volatile u8 key1_pressed_flag = 0;//key1
volatile u8 key2_pressed_flag = 0;//key2
volatile u8 key3_pressed_flag = 0;//key3
volatile u8 key4_pressed_flag = 0;//key4
volatile u8 key5_pressed_flag = 0;//key5
volatile u8 key6_pressed_flag = 0;//key6
volatile u8 key7_pressed_flag = 0;//key7
volatile u8 key8_pressed_flag = 0;//key8

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
  int counter =0;
  u16 ReadVale =0x00;
  delay_init();	    	 //��ʱ������ʼ��	

  NVIC_Configuration();//�����ж����ȼ�����
  
  KEY_Init();//��ʼ��������IOģʽ
  
  EXTI_USER_Init();//�ⲿ�жϳ�ʼ�����������ʼ��8����Ӧ�������ж�����
  
  USART1_Init();//����1��ʼ��,����ӡ����
    
  //TIM3_Int_Init(100-1, 7199);//��ʱ��3��ʼ��,10Khz�ļ���Ƶ�ʣ�������100Ϊ10ms  
  
  GPIO_SetBits(GPIO_LED, DS1_PIN|DS2_PIN|DS3_PIN|DS4_PIN);/*�ر����е�LEDָʾ��*/
  
  printf("\r\n/***********************key scan start*********************/\r\n");
  while(1)
  {     
      delay_ms(1);    
  }
        
    
    
    /******
//    switch(KeyStatus)
//    {
//      case KEY_NOT_PRESS:
//        
//        //�м�ֵ������
//        if(key1_pressed_flag | key2_pressed_flag |key3_pressed_flag |key4_pressed_flag
//           | key5_pressed_flag |key6_pressed_flag  |key7_pressed_flag |key8_pressed_flag)
//        {
//          
//          if(key1_pressed_flag)
//            current_key_value = 0x01;
//          else if(key2_pressed_flag)
//            current_key_value = 0x02;
//          else if(key3_pressed_flag)
//            current_key_value = 0x03;
//          else if(key4_pressed_flag)
//            current_key_value = 0x04;
//          else if(key5_pressed_flag)
//            current_key_value = 0x05;
//          else if(key6_pressed_flag)
//            current_key_value = 0x06;
//          else if(key7_pressed_flag)
//            current_key_value = 0x07;
//          else if(key8_pressed_flag)
//            current_key_value = 0x08;
//          else
//          {
//            
//            printf("key_pressed_flag err !!!\n");
//            KeyStatus = KEY_NOT_PRESS;
//            break;
//          }
//          KeyStatus = KEY_PRESSED;
//          TIM_Cmd(TIM3, ENABLE);  //����TIM3
//           
//        }
//          
//        break;
//        
//    case KEY_PRESSED:
//      
//          switch(current_key_value)
//          {
//            case 0x01: 
//              if(press_counter > 40)//����400ms
//                pressed_value = 0xB1;//�����ļ�ֵ
//              else
//                pressed_value = 0xA1;//�̰��ļ�ֵ
//              //�ȴ������ͷ�
//              if(!key1_pressed_flag) KeyStatus = KEY_RELEASE;
//       
//              break;
//              
//            case 0x02: 
//               if(press_counter > 40)
//                pressed_value = 0xB2;
//              else
//                pressed_value = 0xA2;
//              if(!key2_pressed_flag) KeyStatus = KEY_RELEASE;
//              
//              break;
//            case 0x03: 
//               if(press_counter > 40)
//                pressed_value = 0xB3;
//              else
//                pressed_value = 0xA3;
//              if(!key3_pressed_flag) KeyStatus = KEY_RELEASE;
//              
//              break;
//            case 0x04:
//               if(press_counter > 40)
//                pressed_value = 0xB4;
//              else
//                pressed_value = 0xA4;
//              if(!key4_pressed_flag) KeyStatus = KEY_RELEASE;
//              
//              break;
//            case 0x05: 
//               if(press_counter > 40)
//                pressed_value = 0xB5;
//              else
//                pressed_value = 0xA5;
//              if(!key5_pressed_flag) KeyStatus = KEY_RELEASE;
//             
//              break;
//          
//            case 0x06: 
//               if(press_counter > 40)
//                pressed_value = 0xB6;
//              else
//                pressed_value = 0xA6;
//              if(!key6_pressed_flag) KeyStatus = KEY_RELEASE;
//              break;
//              
//            case 0x07:
//               if(press_counter > 40)
//                pressed_value = 0xB7;
//              else
//                pressed_value = 0xA7;
//              if(!key7_pressed_flag) KeyStatus = KEY_RELEASE;
//              
//              break;
//              
//            case 0x08: 
//               if(press_counter > 40)
//                pressed_value = 0xB8;
//              else
//                pressed_value = 0xA8;
//              if(!key8_pressed_flag) KeyStatus = KEY_RELEASE;              
//              break;
//              
//            default:
//               printf("current_key_value err !!!\n");
//              KeyStatus = KEY_NOT_PRESS;
//              break;
//          
//          }
//          
//          if(KeyStatus == KEY_RELEASE)
//          {
//            TIM_Cmd(TIM3, DISABLE);  //�ر�TIM3
//            press_counter = 0;//��ռ���
//          }
//          
//        break;
//                
//      case KEY_RELEASE:
//        
//         switch(pressed_value)
//          {
//            case 0xA1:
//            case 0xB1:
//             release_value = 0xC1; 
//              break;
//              
//            case 0xA2:
//            case 0xB2:
//             release_value = 0xC2; 
//              break;
//              
//            case 0xA3:
//            case 0xB3:
//             release_value = 0xC3; 
//              break;
//              
//            case 0xA4:
//            case 0xB4:
//             release_value = 0xC4; 
//              break;
//              
//            case 0xA5:
//            case 0xB5:
//             release_value = 0xC5; 
//              break;
//              
//            case 0xA6:
//            case 0xB6:
//             release_value = 0xC6; 
//              break;
//              
//            case 0xA7:
//            case 0xB7:
//             release_value = 0xC7; 
//              break;
//              
//            case 0xA8:
//            case 0xB8:
//             release_value = 0xC8; 
//              break;
//                
//            default:
//               printf("pressed_value err !!!\n");
//              KeyStatus = KEY_PRESSED;
//              break;
//          
//          }
// 
//        
//        KeyStatus = KEY_NOT_PRESS;
//        
//        break;
//        
//        
//      default:
//        printf("key status err !!!\n");
//        KeyStatus = KEY_NOT_PRESS;
//        break;
//     
//    
//    }
//    //delay_ms(1);
//    
    *****/

   
    //delay_ms(10);
//    if((release_value !=0x99) && (pressed_value !=0x99))
//    {
//
//      last_key_value = pressed_value;
//      printf("pressed_value : 0x%X\r\n", pressed_value);
//      counter++;
//      printf("release_value : 0x%X, %d\r\n", release_value, counter);
//      
//      release_value = 0x99;
//      pressed_value = 0x99;
//    }
  
    
  }


/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void USART1_Init(void)
{   
   GPIO_InitTypeDef GPIO_InitStructure;
   
  /*ʹ�ܴ���1ʹ�õ�GPIOʱ��*/
  RCC_APB2PeriphClockCmd(USART1_GPIO_CLK , ENABLE);

  /*ʹ�ܴ���1ʱ��*/
  RCC_APB2PeriphClockCmd(USART1_CLK, ENABLE); 
  
    /*����1 RX�ܽ�����*/
  /* Configure USART1 Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = USART1_RxPin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(USART1_GPIO, &GPIO_InitStructure);

  /*����1 TX�ܽ�����*/ 
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
  USART_InitStructure.USART_BaudRate = 115200;               /*���ò�����Ϊ115200*/
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;/*��������λΪ8*/
  USART_InitStructure.USART_StopBits = USART_StopBits_1;     /*����ֹͣλΪ1λ*/
  USART_InitStructure.USART_Parity = USART_Parity_No;        /*����żУ��*/
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;/*��Ӳ������*/
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  /*���ͺͽ���*/

  /*���ô���1 */
  USART_Init(USART1, &USART_InitStructure);
 
  /*ʹ�ܴ���1�ķ��ͺͽ����ж�*/
  //USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  //USART_ITConfig(USART1, USART_IT_TC, ENABLE);
  
  /* Enable the USART1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;	        //��ռ���ȼ�0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;			//�����ȼ�1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* ʹ�ܴ���1 */
  USART_Cmd(USART1, ENABLE);
  USART_ClearFlag(USART1,USART_FLAG_TC); //���USART_FLAG_TC�������һ���ֽڲ��ܷ��������� 
  


}


/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
void KEY_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  //ʹ��ʱ��
  /*ʹ��LED��ʹ�õ�GPIOʱ��*/
  /*ʹ��KEYɨ�谴��ʹ�õ�GPIOʱ��*/
  RCC_APB2PeriphClockCmd(RCC_GPIO_LED | RCC_GPIO_KEY, ENABLE);
  
   /* KEY����ʹ�õ�GPIO�ܽ�ģʽ*/
  GPIO_InitStructure.GPIO_Pin = KEY1_PIN|KEY2_PIN|KEY3_PIN|KEY4_PIN|KEY5_PIN|KEY6_PIN|KEY7_PIN|KEY8_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIO_KEY, &GPIO_InitStructure); 
  

  /* LED��ʹ�õ�GPIO�ܽ�ģʽ*/
  GPIO_InitStructure.GPIO_Pin = DS1_PIN|DS2_PIN|DS3_PIN|DS4_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIO_LED, &GPIO_InitStructure); 
  
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

  /*ʹ��AFIOʹ�õ�GPIOʱ��*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 
  
  
   //PA0 �ж����Լ��жϳ�ʼ������
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0);     //��GPIOA��Pin0����ΪEXTI��������
  EXTI_InitStructure.EXTI_Line=EXTI_Line0;                       //�ж���0
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	         //�ж�ģʽ����
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //˫���ش���
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);		//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
  
  //PA1 �ж����Լ��жϳ�ʼ������
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource1);     //��GPIOA��Pin1����ΪEXTI��������
  EXTI_InitStructure.EXTI_Line=EXTI_Line1;                       //�ж���1
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	         //�ж�ģʽ����
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //˫���ش���
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);		//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
  
  //PA2 �ж����Լ��жϳ�ʼ������
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource2);     //��GPIOA��Pin2����ΪEXTI��������
  EXTI_InitStructure.EXTI_Line=EXTI_Line2;                       //�ж���2
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	         //�ж�ģʽ����
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);		//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
  
  //PA3
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource3);     //��GPIOA��Pin3����ΪEXTI��������
  EXTI_InitStructure.EXTI_Line=EXTI_Line3;                       //�ж���3
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	         
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);		
  
   //PA4
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource4);     //��GPIOA��Pin4����ΪEXTI��������
  EXTI_InitStructure.EXTI_Line=EXTI_Line4;                       //�ж���4
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	         
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);		
  
   //PA5
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource5);     //��GPIOA��Pin5����ΪEXTI��������
  EXTI_InitStructure.EXTI_Line=EXTI_Line5;                       //�ж���5
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	         
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);		//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
  
     //PA6
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource6);     //��GPIOA��Pin6����ΪEXTI��������
  EXTI_InitStructure.EXTI_Line=EXTI_Line6;                       //�ж���6
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	         
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);		//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
//  
//  //PA7
//  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource7);     //��GPIOA��Pin7����ΪEXTI��������
//  EXTI_InitStructure.EXTI_Line=EXTI_Line7;                       //�ж���7
//  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	         
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 
//  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//  EXTI_Init(&EXTI_InitStructure);		//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
//  
  
  //PA8
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource8);     //��GPIOA��Pin8����ΪEXTI��������
  EXTI_InitStructure.EXTI_Line=EXTI_Line8;                       //�ж���8
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	         
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);		//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
  
  
  
  //�ж��ߺŵ����ȼ�����
  
   NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//ʹ�ܰ������ڵ��ⲿ�ж�ͨ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;          //��ռ���ȼ�1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;		        //�����ȼ�0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure);  	  //����NVIC_InitSruct��ָ���Ĳ�����ʼ������NVIC�ļĴ���
  
  
   NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;			//ʹ�ܰ������ڵ��ⲿ�ж�ͨ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;          //��ռ���ȼ�1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;		        //�����ȼ�1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure);  	  //����NVIC_InitSruct��ָ���Ĳ�����ʼ������NVIC�ļĴ���
  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;			//ʹ�ܰ������ڵ��ⲿ�ж�ͨ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;          //��ռ���ȼ�1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;		        //�����ȼ�2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure);  	  //����NVIC_InitSruct��ָ���Ĳ�����ʼ������NVIC�ļĴ���
  
          
  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;			//ʹ�ܰ������ڵ��ⲿ�ж�ͨ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;	        //��ռ���ȼ�1 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;			//�����ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure); 
  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;			//ʹ�ܰ������ڵ��ⲿ�ж�ͨ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;	        //��ռ���ȼ�1 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;			//�����ȼ�4
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure);
    
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//ʹ�ܰ������ڵ��ⲿ�ж�ͨ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;	        //��ռ���ȼ�1 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x05;			//�����ȼ�5
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure);


}

/**
  * @brief  Configures the Timer3 controller
  * @param  None
  * @retval None
  */

//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr:�Զ���װֵ
//psc:ʱ��Ԥ��Ƶ��
//����ʹ�õ��Ƕ�ʱ��3
void TIM3_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��
  
  //��ʱ��TIM3��ʼ��
  TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
  TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָTDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʵ��������λ

  TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�жϣ���������ж�

  //�ж����ȼ�NVIC����
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ�����ȼ�0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ������
  NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���

  TIM_Cmd(TIM3, DISABLE);  //��ʱ��ʹ��TIM3	


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

  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//�˴�ѡ���ⲿʱ�ӣ�HCLK/8
  fac_us = SystemCoreClock/8000000;//fac_us����Ϊϵͳʱ�ӵ�1/8	
  fac_ms = (u16)fac_us*1000; //ÿ��ms��Ҫ��systickʱ����

}

void delay_us(u32 xus)	//��ʱxus��΢��
{		
    u32 temp;	    	 
    SysTick->LOAD=xus*fac_us;                         //ʱ�����	  		 
    SysTick->VAL=0x00;                                //��ռ�����
    SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;       //���ÿ�ʼ������ʱ������

    do
    {
            temp=SysTick->CTRL;
    }
    while(temp&0x01&&!(temp&(1<<16)));            //�ȴ�ʱ�䵽��  
            SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //�رռ�����

    SysTick->VAL =0X00;                            //��ռ�����	 
}

//��ʱxms 72mhz�£�xms<=1864
void delay_ms(u16 xms)
{	 		  	  
    u32 temp;		   
    SysTick->LOAD=(u32)xms*fac_ms; //ʱ�����
    SysTick->VAL =0x00;            //��ռ�����
    SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //��ʼ���� 
    do
    {
            temp=SysTick->CTRL;
    }
    while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��Ŷ
    SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //�رռ�����
    SysTick->VAL =0X00;       //��ռ�����  	    
} 
/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
