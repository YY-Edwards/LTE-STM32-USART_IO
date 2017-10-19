#include "pro_slip.h"
#include "platform_config.h"

extern Queue_t UsartRxQue;
extern Queue_t QueueCommand;
extern int simple_uart_put(u8 ch);
extern void Write_Volume(unsigned short dat);

char volume_value[11]={62, 20, 14, 10, 8, 6, 4, 3, 2, 1, 0};//-db=20lgR; R=Vo/Vi

#if 0
int packet_analysis(void *packet, u8 length)
{
  
  u16 Checksum = 0x0000;
  char buf[10];
  memset(buf, 0x00, 10);
  u8 counter = 0;
  u8 LED_ID=0;
  u8 LED_Action =0;
  
  if(length > 12)return 1;
    
  for(u8 i=0; i < length; i++){
    
    switch(*((u8 *)packet+i)){
      
      case END:
        break;
           
      case ESC:
        
          i++;//next   
         switch(*((u8 *)packet+i)){
           
           case ESC_END:
                buf[counter++] = END;
                break;
           case ESC_ESC:
                buf[counter++] = ESC;
                break;
                
        }
           
        break;
        
      default:
          
        buf[counter++] = (*((u8 *)packet+i));
       
    } 
      
  }
    
//  for(u8 j=0; j < 10; j++){
//    
//    printf("\n\r--rx protocol data is: %x\n\r",buf[j]);
//  }
   
  
      ScanKeyProtocol_t ptr;
//      unsigned char temp =0;
//     for(u8 k=0; k < 10;){//change data mode
//       
//        temp = buf[k];
//        buf[k] = buf[k+1];
//        buf[k+1] = temp;
//        k+=2;  
//       
//     }
    memcpy(&ptr, buf, 10);
    Checksum = ptr.KeyValue + ptr.Status;
    if((ptr.Header == 0xABCD) && (ptr.Terminator == 0x00BA)){
    
      if(ptr.Checksum == Checksum){
        if(ptr.Status == 0x0003){//控制音量           
            Write_Volume(volume_value[(ptr.KeyValue & 0xff)]);//write volume_value to LM1971
            //LED2 = !LED2;
            //printf("Set volume is : %02d\r\n", (u8)(ptr.KeyValue));
        }
        else if (ptr.Status == 0x0004){//控制LED/LKJ
  
             LED_ID = (ptr.KeyValue & 0xFF);//低8位
    
             LED_Action = ((ptr.KeyValue>>8)&0xFF);//高8位
              
             LED2 = !LED2;
            switch(LED_ID){
                  
                  case 0x01://RED1
                    
                      if(LED_Action == ON)
                      {
                         //GPIO_ResetBits(GPIO_LED, DS2_PIN);
                        GPIO_ResetBits(GPIO_LED_3_6, RED1_PIN);/*点亮RED1信号灯*///输出低电平，三极管截止
                      }
                      else{
                        //GPIO_SetBits(GPIO_LED, DS2_PIN);
                        GPIO_SetBits(GPIO_LED_3_6, RED1_PIN);/*关闭RED1信号灯*///输出高电平，驱动三极管导通
                      }
                      
                      break;
              
                  case 0x02://YELLOW
                        
                     if(LED_Action == ON)
                        GPIO_ResetBits(GPIO_LED_3_6, YELLOW_PIN);/*点亮YEELOW信号灯*///输出低电平，三极管截止
                      else
                        GPIO_SetBits(GPIO_LED_3_6, YELLOW_PIN);
                    
                      break;
                  
                  case 0x03://GREEEN
                    
                     if(LED_Action == ON)
                        GPIO_ResetBits(GPIO_LED_3_6, GREEN_PIN);//*点亮GREEN信号灯*///输出低电平，三极管截止
                      else
                        GPIO_SetBits(GPIO_LED_3_6, GREEN_PIN);
                      
                      break;
                  
                  case 0x04://RED2
                    
                     if(LED_Action == ON)
                        GPIO_ResetBits(GPIO_LED_3_6, RED2_PIN);/*点亮RED2信号灯*///输出低电平，三极管截止
                      else
                        GPIO_SetBits(GPIO_LED_3_6, RED2_PIN);
                    
                      break;
                      
                  case 0x10://LKJ1
                    
                     if(LED_Action == ON)
                        GPIO_ResetBits(GPIO_LKJ_1_4, LKJ1_PIN);//输出低电平，三极管截止，LKJ1上拉至高电平5V
                      else
                        GPIO_SetBits(GPIO_LKJ_1_4, LKJ1_PIN);//输出高电平，驱动三极管导通，LKJ1下拉至低电平0V
                    
                      break;
                      
                 case 0x20://LKJ2
                  
                   if(LED_Action == ON)
                      GPIO_ResetBits(GPIO_LKJ_1_4, LKJ2_PIN);//输出低电平，三极管截止，LKJ2上拉至高电平5V
                    else
                      GPIO_SetBits(GPIO_LKJ_1_4, LKJ2_PIN);
                  
                    break;
                      
                 case 0x30://LKJ3
                
                    if(LED_Action == ON)
                      GPIO_ResetBits(GPIO_LKJ_1_4, LKJ3_PIN);//输出低电平，三极管截止，LKJ3上拉至高电平5V
                    else
                      GPIO_SetBits(GPIO_LKJ_1_4, LKJ3_PIN);
                  
                    break;
                        
                case 0x40://LKJ4
                
                 if(LED_Action == ON)
                    GPIO_ResetBits(GPIO_LKJ_1_4, LKJ4_PIN);//输出低电平，三极管截止，LKJ4上拉至高电平5V
                  else
                    GPIO_SetBits(GPIO_LKJ_1_4, LKJ4_PIN);
                
                  break;
                      
                case 0x50://LKJ5
                
                 if(LED_Action == ON)
                    GPIO_ResetBits(GPIO_LKJ_5_6, LKJ5_PIN);//输出低电平，三极管截止，LKJ5上拉至高电平5V
                  else
                    GPIO_SetBits(GPIO_LKJ_5_6, LKJ5_PIN);
                
                  break;
                      
                case 0x60://LKJ6
              
                   if(LED_Action == ON)
                      GPIO_ResetBits(GPIO_LKJ_5_6, LKJ6_PIN);//输出低电平，三极管截止，LKJ6上拉至高电平5V
                    else
                      GPIO_SetBits(GPIO_LKJ_5_6, LKJ6_PIN);
                  
                    break;
                                                  
                default://Other err
                    break;
            
            }
    
         }
      }
    }
    memset(&ptr, 0x00 ,10);
    return 0;
  
  
  
  }
#endif

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


void USART_GetInputByte(void)
{ 
  u8 c = USART_ReceiveData(USART1);
  u8 temp=0;
  QueuePush(UsartRxQue, &c);
  
  if(c== END)
  {
    unsigned char buffer[sizeof(ScanKeyProtocol_t)] = {0};
    if(ReadSlipPackage(buffer) > 0)
    {      
        QueuePush(QueueCommand, buffer);
    }
    
    while(1)
    {
      if(queue_ok != QueuePull(UsartRxQue, &temp))break;
    }       
  }
}

int ReadSlipPackage(unsigned char * package)
{
  
    unsigned int packagelength = 0;
    unsigned char chnext = 0;
  
    while(1)
    {     
       unsigned char ch = 0;      
       if(queue_ok != QueuePull(UsartRxQue, &ch))return 0;
        
      switch(ch)
      {
        
      case END:
        return packagelength;
        
      case ESC:
              
       if(queue_ok != QueuePull(UsartRxQue, &chnext))return 0;
        if(chnext == ESC_END)
        {
          package[packagelength++] = END;
        }
        else if(chnext == ESC_ESC)
        {
           package[packagelength++] = ESC;
        }
        else
        {  
           return 0;                        
        }
          
      default:
        package[packagelength++] = ch;
        
        
      }     
    }
                 
    //return 0;
}
  

