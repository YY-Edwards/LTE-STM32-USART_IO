#include "pro_slip.h"
#include "platform_config.h"

extern int simple_uart_put(u8 ch);

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




