#ifndef _MYLOCK_H_
#define _MYLOCK_H_
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#include "stm32f10x.h"

volatile unsigned short * sem_init(unsigned short initvalue);
unsigned short sem_get(volatile unsigned short * sem);
unsigned short sem_free(volatile unsigned short * sem);
void sem_destory(volatile unsigned short * sem);


#endif