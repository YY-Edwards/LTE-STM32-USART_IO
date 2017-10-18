#include "pro_slip.h"
#include "platform_config.h"

extern int simple_uart_put(u8 ch);
extern void Write_Volume(unsigned short dat);

char volume_value[11]={62, 20, 14, 10, 8, 6, 4, 3, 2, 1, 0};//-db=20lgR; R=Vo/Vi

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




