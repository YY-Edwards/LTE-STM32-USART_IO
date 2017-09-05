#ifndef PRO_SLIP_H__
#define PRO_SLIP_H__
#include <stdint.h>
#include "stdlib.h"
#include "string.h"
#include "stm32f10x.h"

#pragma once

#define HEADER             (u16)0xABCD    /* indicates Header of protocol */
#define TERMINATOR         (u16)0x00BA    /* indicates Terminator of protocol */
/*little endian*/
/*header(2) + checksum(2) + key_value(2) + status(2) + terminator(2)*/
/*
checksum = key_value  + status
key_value =0xA00x;x= 1,2,3,4...8
status = 0 / 1; 0:release, 1:press (status = 0x0003 ; set volume(key_value:0x00~0x0A (attenuation); 0x00:Mute))
*/

//Add by Edwards
/*
checksum = key_value  + status

status:0x0004 //¿ØÖÆLEDÃüÁî

key_value¡ª¡ªHigh8:              //¸ß°ËÎ»
            0x00:OFF            //Ï¨Ãð
            0x01:ON             //µãÁÁ

key_value¡ª¡ªLow8:               //µÍ°ËÎ»
            0x01:RED1           //ºì1µÆ
            0x02:YELLOW         //»ÆµÆ
            0x03:GREEN          //ÂÌµÆ
            0x04:RED2           //ºì2µÆ
  
*/

#define END             (char)0xC0    /* indicates end of packet */
#define ESC             (char)0xD0    /* indicates byte stuffing */
#define ESC_END         (char)0xD1   /* ESC ESC_END means END data byte */
#define ESC_ESC         (char)0xD2   /* ESC ESC_ESC means ESC data byte */

#define ON 1
#define OFF 0

typedef struct
{
  unsigned short Header;
  unsigned short Checksum;
  unsigned short KeyValue;
  unsigned short Status;
  unsigned short Terminator;
  
}ScanKeyProtocol_t;

typedef ScanKeyProtocol_t * datapro_t;


int data_packet_send(u16 value, u16 status);

int packet_analysis(void *packet, u8 length);

#endif


















