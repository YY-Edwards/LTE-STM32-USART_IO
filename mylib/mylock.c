#include "mylock.h"


unsigned long * sem_init(unsigned long initvalue)
{
  unsigned long * sem = (unsigned long *)malloc(sizeof(unsigned long));
  if(NULL == sem)
  {
      return NULL;
  }
  
  memcpy(sem, &initvalue, sizeof(unsigned long));

  return sem;
}

int value=0;
unsigned long sem_get(unsigned long * sem)
{
    //int value=0;
    if(sem==NULL)return false;
    value = __LDREX(sem);
    if(value){
      return __STREX(0, sem) ? false: true;
    }
    return false;
    
}

unsigned long sem_free(unsigned long * sem)
{
    int v = 0;
    if(sem==NULL)return false;
    v = __LDREX(sem);
    if(!v){
            return __STREX(1, sem) ? false: true;
    }
    return false;

}
void sem_destory(unsigned long * sem)
{
  
  free(sem);
  sem=NULL;

}