#include "pro_slip.h"

extern Queue_t UsartRxQue;
extern Queue_t QueueCommand;
extern int simple_uart_put(u8 ch);
extern void Write_Volume(unsigned short dat);
extern unsigned short *  QueueCommand_lock;

char volume_value[11]={62, 20, 14, 10, 8, 6, 4, 3, 2, 1, 0};//-db=20lgR; R=Vo/Vi

void SetLedRed1(unsigned char istrunon)
{
  if(istrunon)GPIO_ResetBits(GPIO_LED_3_6, RED1_PIN);/*点亮RED1信号灯*///输出低电平，三极管截止
  else 
    GPIO_SetBits(GPIO_LED_3_6, RED1_PIN);/*关闭RED1信号灯*///输出高电平，驱动三极管导通
}

void SetLedYellow(unsigned char istrunon)
{     
   //点亮YEELOW信号灯　
   //输出低电平，三极管截止
   if(istrunon)GPIO_ResetBits(GPIO_LED_3_6, YELLOW_PIN);
   else 
     GPIO_SetBits(GPIO_LED_3_6, YELLOW_PIN);
}

void SetLedGreen(unsigned char istrunon)
{
    if(istrunon)GPIO_ResetBits(GPIO_LED_3_6, GREEN_PIN);//*点亮GREEN信号灯*///输出低电平，三极管截止
    else 
      GPIO_SetBits(GPIO_LED_3_6, GREEN_PIN);
}

void SetLedRed2(unsigned char istrunon)
{
    if(istrunon)GPIO_ResetBits(GPIO_LED_3_6, RED2_PIN);/*点亮RED2信号灯*///输出低电平，三极管截止
    else 
      GPIO_SetBits(GPIO_LED_3_6, RED2_PIN);
}

void SetVolume(unsigned char volume)
{
  if(volume > sizeof(volume_value))return;
  Write_Volume(volume_value[volume]); 
}

void SetLedDisplay(unsigned char led, unsigned char istrunon)
{
  if((led & 0xF0) > 0 )return;
  
  LED2 = !LED2; //for test
  
  switch(led)
  {
  case LedRed1:
    SetLedRed1(istrunon);
    break;
  case LedYellow:
    SetLedYellow(istrunon);
    break;
  case LedGreen:
    SetLedGreen(istrunon);
    break;
  case LedRed2:
    SetLedRed2(istrunon);
    break;
  default:
    break;
  }
}

void SetLKJBit0(unsigned char value)
{

  if(value)GPIO_ResetBits(GPIO_LKJ_1_4, LKJ1_PIN);//输出低电平，三极管截止，LKJ1上拉至高电平5V
  else
    GPIO_SetBits(GPIO_LKJ_1_4, LKJ1_PIN);//输出高电平，驱动三极管导通，LKJ1下拉至低电平0V

}

void SetLKJBit1(unsigned char value)
{
  if(value)GPIO_ResetBits(GPIO_LKJ_1_4, LKJ2_PIN);//输出低电平，三极管截止，LKJ2上拉至高电平5V
  else
    GPIO_SetBits(GPIO_LKJ_1_4, LKJ2_PIN);

}

void SetLKJBit2(unsigned char value)
{

  if(value)GPIO_ResetBits(GPIO_LKJ_1_4, LKJ3_PIN);//输出低电平，三极管截止，LKJ3上拉至高电平5V
  else
    GPIO_SetBits(GPIO_LKJ_1_4, LKJ3_PIN);

}

void SetLKJBit3(unsigned char value)
{

  
  if(value)GPIO_ResetBits(GPIO_LKJ_1_4, LKJ4_PIN);//输出低电平，三极管截止，LKJ4上拉至高电平5V
  else
    GPIO_SetBits(GPIO_LKJ_1_4, LKJ4_PIN);

}

void SetLKJBit4(unsigned char value)
{

  if(value)GPIO_ResetBits(GPIO_LKJ_5_6, LKJ5_PIN);//输出低电平，三极管截止，LKJ5上拉至高电平5V
  else
    GPIO_SetBits(GPIO_LKJ_5_6, LKJ5_PIN);

}

void SetLKJBit5(unsigned char value)
{

  if(value)GPIO_ResetBits(GPIO_LKJ_5_6, LKJ6_PIN);//输出低电平，三极管截止，LKJ6上拉至高电平5V
  else
    GPIO_SetBits(GPIO_LKJ_5_6, LKJ6_PIN);

}

void SetLKJValue(unsigned char lkjbit, unsigned char value)
{
   if((lkjbit & 0x0F) > 0 )return;
   
   LED2 = !LED2; //for test
   
  switch(lkjbit)
  {
    case LKJBit0:
      SetLKJBit0(value);
      break;
    case LKJBit1:
      SetLKJBit1(value);
      break;
    case LKJBit2:
      SetLKJBit2(value);
      break;
    case LKJBit3:
      SetLKJBit3(value);
      break;
    case LKJBit4:
      SetLKJBit4(value);
      break;
    case LKJBit5:
      SetLKJBit5(value);
      break;
    default:
      break;
  }
  
}


int data_packet_send(u16 value, u16 status)
{

  u8 len = 10;
  u8 i = 0;
  
  datapro_t datapro = (ScanKeyProtocol_t *)malloc(sizeof(ScanKeyProtocol_t));
  if(NULL == datapro)return NULL;
  
  memset(datapro, 0x00, sizeof(ScanKeyProtocol_t));
  
  datapro->Header = HEADER;
  datapro->Checksum = value + status;
  datapro->KeyValue = value;
  datapro->Status = status;
  datapro->Terminator = TERMINATOR;
   
	simple_uart_put(END);
	while (len-- > 0)
	{
		switch (*((u8 *)datapro + i))
		{
		case END:
			simple_uart_put(ESC);
			simple_uart_put(ESC_END);
			break;

		case ESC:
			simple_uart_put(ESC);
			simple_uart_put(ESC_ESC);
			break;
		default:
			simple_uart_put(*((u8 *)datapro + i));
		}
                
                i++;
	}
	simple_uart_put(END);
        
        free(datapro);
        
        return 0;
		
}

void DMA_GetInputBytes(u8 *rx_buff, u8 length)
{
  unsigned long return_value =0;
  QueueSta_t queue_status = queue_empty;
  for(u8 i=0; i<length; i++)
  {
    
    QueuePush(UsartRxQue, &rx_buff[i]);
  
    if(rx_buff[i] == END)
    {
      unsigned char buffer[sizeof(ScanKeyProtocol_t)] = {0};
      if(ReadSlipPackage(buffer, sizeof(ScanKeyProtocol_t)) > 0)
      {      
          queue_status=QueuePush(QueueCommand, buffer); 
          
      }
      else
      {
          QueueClear(UsartRxQue);
      
     }
      
      
    }
  }

}

void USART_GetInputByte(void)
{ 
  u8 c = USART_ReceiveData(USART1);
  unsigned long return_value =0;
  QueueSta_t queue_status = queue_empty;
  
  QueuePush(UsartRxQue, &c);
  
  if(c== END)
  {
    unsigned char buffer[sizeof(ScanKeyProtocol_t)] = {0};
    if(ReadSlipPackage(buffer, sizeof(ScanKeyProtocol_t)) > 0)
    {      
//        return_value=sem_get(QueueCommand_lock);//lock
//        if(return_value==true)
//        {
//         
//          queue_status=QueuePush(QueueCommand, buffer);              
//          return_value=sem_free(QueueCommand_lock);//unlock   
//                                    
//        }
        queue_status=QueuePush(QueueCommand, buffer); 
        
    }
    else
    {
        QueueClear(UsartRxQue);
    
   }
    
    
  }
}

int ReadSlipPackage(unsigned char * package, unsigned int except_packagelength)
{
    unsigned int packagelength = 0;
    unsigned char chnext = 0;
    //unsigned char c_value = 0;   
  
    while(1)
    {     
       unsigned char ch = 0;      
       if(queue_ok != QueuePull(UsartRxQue, &ch))return 0; 
       
        switch(ch)
        {
          
        case END:
          
          if(packagelength !=0)
            return packagelength;
          else
            return 0;
          
        case ESC:
                
           if(queue_ok != QueuePull(UsartRxQue, &chnext))return 0;
            if(chnext == ESC_END)
            {
              if(packagelength < except_packagelength)package[packagelength++] = END;
              else
              {
                return 0;
              }
              
            }
            else if(chnext == ESC_ESC)
            {
              if(packagelength < except_packagelength)package[packagelength++] = ESC;
              else
              {
                return 0;
              }
            }
            else
            {  
               return 0;                        
            }
           break;
           
        default:
          
            if(packagelength < except_packagelength)package[packagelength++] = ch;
              else
              {
                return 0;
              }
          
          break;
          
        }     
    }
                 
    //return 0;
}
  




