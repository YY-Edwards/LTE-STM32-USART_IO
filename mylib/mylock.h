#ifndef _MYLOCK_H_
#define _MYLOCK_H_
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#include "stm32f10x.h"

unsigned long * sem_init(unsigned long initvalue);
unsigned long sem_get(unsigned long * sem);
unsigned long sem_free(unsigned long * sem);
void sem_destory(unsigned long * sem);


#endif