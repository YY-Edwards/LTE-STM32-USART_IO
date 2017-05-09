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
/*header(2) + checksum(2) + key_value(2) + statux(2) + terminator(2)*/
/*
checksum = key_value  + status
key_value =0xA00x;x= 1,2,3,4...8
status = 0 / 1; 0:release, 1:press
*/

#define END             (char)0xC0    /* indicates end of packet */
#define ESC             (char)0xD0    /* indicates byte stuffing */
#define ESC_END         (char)0xD1   /* ESC ESC_END means END data byte */
#define ESC_ESC         (char)0xD2   /* ESC ESC_ESC means ESC data byte */



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

#endif


















