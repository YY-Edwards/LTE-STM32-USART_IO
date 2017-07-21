#include "pro_slip.h"
#include "platform_config.h"

extern int simple_uart_put(u8 ch);
extern void Write_Volume(unsigned short dat);

char volume_value[11]={62, 20, 14, 10, 8, 6, 4, 3, 2, 1, 0};//-db=20logR; R=Vo/Vi

int packet_analysis(void *packet, u8 length)
{
  
  u16 Checksum = 0x0000;
  char buf[10];
  memset(buf, 0x00, 10);
  u8 counter = 0;
  
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
        if(ptr.Status == 0x0003){           
            Write_Volume(volume_value[(ptr.KeyValue & 0xff)]);//write volume_value to LM1971
            LED4 = !LED4;
            //printf("Set volume is : %02d\r\n", (u8)(ptr.KeyValue));
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




