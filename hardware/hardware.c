#include "hardware.h"

NVIC_InitTypeDef NVIC_InitStructure;
USART_InitTypeDef USART_InitStructure;
DMA_InitTypeDef  DMA_InitStructure; 

volatile Queue_t UsartRxQue = NULL;
volatile Queue_t QueueCommand = NULL;

extern u8 USART_RX[256];

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
 
  /*ʹ�ܴ���1�Ľ����ж�*/
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART_ClearFlag(USART1,USART_FLAG_RXNE); 	
  
  
//  USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
//  USART_ITConfig(USART1, USART_IT_TC, DISABLE);
//  USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);//�����жϣ�����δ֪���ȣ���ʹ�ÿ����ж����ж��Ƿ������ϣ� 
//  USART_ClearFlag(USART1,USART_FLAG_IDLE); 				//��USART_FLAG_IDLE��־ 
//
//
//  //����DMA��ʽ����  
//  USART_DMACmd(USART1,USART_DMAReq_Rx ,ENABLE);
//    
//  //����DMA1��ʱ��
//  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
//  
//  /* DMA1 channel 5 configuration */ //USART1_RX  
//  DMA_DeInit(DMA1_Channel5);  
//  DMA_InitStructure.DMA_PeripheralBaseAddr      =(u32)(&USART1->DR);  			//���贮��1��ַ  
// 
//  DMA_InitStructure.DMA_MemoryBaseAddr          =(u32)USART_RX;
//  DMA_InitStructure.DMA_DIR                     =DMA_DIR_PeripheralSRC;   	//������ΪĿ�ĵ�ַ   //DMA_DIR_PeripheralSRC;   //������ΪDMA��Դ��  
//  DMA_InitStructure.DMA_BufferSize              =256; 				//BufferSize;      //���仺������С 
//  DMA_InitStructure.DMA_PeripheralInc           =DMA_PeripheralInc_Disable; 	//�������ģʽ��ֹ   DMA_PeripheralInc_Enable;            //�����ַ����  
//  DMA_InitStructure.DMA_MemoryInc               =DMA_MemoryInc_Enable;   	//�ڴ��ַ����  
//  DMA_InitStructure.DMA_PeripheralDataSize      =DMA_PeripheralDataSize_Byte; 	//���䷽ʽ���ֽ�   DMA_PeripheralDataSize_Word;    //�֣�32λ��  
//  DMA_InitStructure.DMA_MemoryDataSize          =DMA_MemoryDataSize_Byte;  	//�ڴ�洢��ʽ���ֽ�  DMA_MemoryDataSize_Word;  
//  DMA_InitStructure.DMA_Mode                    =DMA_Mode_Normal;  		//DMA_Mode_Normal ����ģʽ��ֻ����һ��;  DMA_Mode_Circular:ѭ��ģʽ����ͣ�Ĵ���;  
//  DMA_InitStructure.DMA_Priority                =DMA_Priority_VeryHigh;  	//����1�Ľ�����Ϊ������ȼ�
//  DMA_InitStructure.DMA_M2M                     =DMA_M2M_Disable;             	//DMA_M2M_Enable;      
//  DMA_Init(DMA1_Channel5,&DMA_InitStructure); 
//
//	//ʹ��ͨ��5  
//  DMA_Cmd(DMA1_Channel5,ENABLE);  
  
  
  /* Enable the USART1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;	        //��ռ���ȼ�0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;			//�����ȼ�1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* ʹ�ܴ���1 */
  USART_Cmd(USART1, ENABLE);
  USART_ClearFlag(USART1,USART_FLAG_TC); //���USART_FLAG_TC�������һ���ֽڲ��ܷ��������� 
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    {}
  


}


void USART2_Init(void)
{   
   GPIO_InitTypeDef GPIO_InitStructure;
   
  /*ʹ�ܴ���2ʹ�õ�GPIOʱ��*/
  RCC_APB2PeriphClockCmd(USART2_GPIO_CLK , ENABLE);

  /*ʹ�ܴ���2ʱ��*/
  RCC_APB1PeriphClockCmd(USART2_CLK, ENABLE); 
  
    /*����2 RX�ܽ�����*/
  /* Configure USART1 Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = USART2_RxPin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(USART2_GPIO, &GPIO_InitStructure);

  /*����2 TX�ܽ�����*/ 
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
  USART_InitStructure.USART_BaudRate = 115200;               /*���ò�����Ϊ115200*/
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;/*��������λΪ8*/
  USART_InitStructure.USART_StopBits = USART_StopBits_1;     /*����ֹͣλΪ1λ*/
  USART_InitStructure.USART_Parity = USART_Parity_No;        /*����żУ��*/
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;/*��Ӳ������*/
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  /*���ͺͽ���*/

  /*���ô���2 */
  USART_Init(USART2, &USART_InitStructure);
 
  /*ʹ�ܴ���2�ķ��ͺͽ����ж�*/
  //USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  //USART_ITConfig(USART2, USART_IT_TC, ENABLE);
  
  /* Enable the USART1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;	        //��ռ���ȼ�0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;			//�����ȼ�1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* ʹ�ܴ���2 */
  USART_Cmd(USART2, ENABLE);
  USART_ClearFlag(USART2,USART_FLAG_TC); //���USART_FLAG_TC�������һ���ֽڲ��ܷ��������� 
  


}



/**
  * @brief  Configures the different LTE GPIO ports.
  * @param  None
  * @retval None
  */
void LTE_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  //ʹ��ʱ��
  /*ʹ��LED��ʹ�õ�GPIOʱ��*/
  /*ʹ��KEYɨ�谴��ʹ�õ�GPIOʱ��*/
  RCC_APB2PeriphClockCmd(RCC_GPIO_KEY_5_8 | RCC_GPIO_KEY_1_4| RCC_GPIO_VOLUME_CTL, ENABLE);
  
  RCC_APB2PeriphClockCmd(RCC_GPIO_LKJ_5_6|RCC_GPIO_LKJ_1_4, ENABLE);
  
  RCC_APB2PeriphClockCmd(RCC_GPIO_LED_1_2|RCC_GPIO_LED_3_6, ENABLE);
  
   /* KEY����ʹ�õ�GPIO�ܽ�ģʽ*/
  GPIO_InitStructure.GPIO_Pin = KEY5_PIN|KEY6_PIN|KEY7_PIN|KEY8_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIO_KEY_5_8, &GPIO_InitStructure); 
  
  GPIO_InitStructure.GPIO_Pin = KEY1_PIN|KEY2_PIN|KEY3_PIN|KEY4_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIO_KEY_1_4, &GPIO_InitStructure); 
  
  /* LKJʹ�õ�GPIO�ܽ�ģʽ*/
  GPIO_InitStructure.GPIO_Pin = LKJ1_PIN|LKJ2_PIN|LKJ3_PIN|LKJ4_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIO_LKJ_1_4, &GPIO_InitStructure); 
  
   GPIO_InitStructure.GPIO_Pin = LKJ5_PIN|LKJ6_PIN; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIO_LKJ_5_6, &GPIO_InitStructure); 
  
  

  /* LED��ʹ�õ�GPIO�ܽ�ģʽ*/
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
  
  GPIO_SetBits(GPIO_LED_1_2, DS1_PIN|DS2_PIN);/*�ر����е�LEDָʾ��*/
  
  GPIO_SetBits(GPIO_LED_3_6, RED1_PIN|YELLOW_PIN|GREEN_PIN|RED2_PIN);/*Ϩ�����е��źŵ�*///����ߵ�ƽ
  
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
  EXTI_Init(&EXTI_InitStructure);		//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
  
  //PC6
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource6);     //��GPIOC��Pin6����ΪEXTI��������
  EXTI_InitStructure.EXTI_Line=EXTI_Line6;                       //�ж���6
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	         
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);		
  
   //PC7
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource7);     //��GPIOC��Pin7����ΪEXTI��������
  EXTI_InitStructure.EXTI_Line=EXTI_Line7;                       //�ж���7
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	         
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);		
  
   //PC8
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource8);     //��GPIOC��Pin8����ΪEXTI��������
  EXTI_InitStructure.EXTI_Line=EXTI_Line8;                       //�ж���8
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	         
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);		//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
  
     //PC9
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource9);     //��GPIOC��Pin9����ΪEXTI��������
  EXTI_InitStructure.EXTI_Line=EXTI_Line9;                       //�ж���9
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
  
//  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;			//ʹ�ܰ������ڵ��ⲿ�ж�ͨ��
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;	        //��ռ���ȼ�1 
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;			//�����ȼ�4
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//ʹ���ⲿ�ж�ͨ��
//  NVIC_Init(&NVIC_InitStructure);
    
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


int simple_uart_put(u8 ch)
{
    USART_SendData(USART1, ch);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    {}
    return ch;
}

void Hardware_Init(void)
{

  NVIC_Configuration();//�����ж����ȼ�����
  
  LTE_GPIO_Init();//��ʼ��KEY,LED,LKJ,Volume��IOģʽ
  
  EXTI_USER_Init();//�ⲿ�жϳ�ʼ�����������ʼ��8����Ӧ�������ж�����
  
  UsartRxQue = QueueCreate(MAX_RX_DEEP, 1); 
  
  USART1_Init();//����1��ʼ��,����ӡ����, ���������жϽ��չ���
  
  //USART2_Init();
    
  //TIM3_Int_Init(100-1, 7199);//��ʱ��3��ʼ��,10Khz�ļ���Ƶ�ʣ�������100Ϊ10ms  
   
  
  QueueCommand =  QueueCreate(CommandQueueDeep, sizeof(ScanKeyProtocol_t));
  
  
  Write_Volume(0x0001);//set default volume at 9



}




