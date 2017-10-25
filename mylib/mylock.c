#include "mylock.h"


volatile unsigned short * sem_init(unsigned short initvalue)
{
  unsigned short * sem = (unsigned short *)malloc(sizeof(unsigned short));
  if(NULL == sem)
  {
      return NULL;
  }
  
  memcpy(sem, &initvalue, sizeof(unsigned short));

  return sem;
}

int value=0;
unsigned short sem_get(volatile unsigned short * sem)
{
    //int value=0;
    if(sem==NULL)return false;
    value = __LDREXH(sem);
    if(value){
      return __STREXH(0, sem) ? (0x39): true;
    }
    return false;
    
}

unsigned short sem_free(volatile unsigned short * sem)
{
    int v = 0;
    if(sem==NULL)return false;
    v = __LDREXH(sem);
    if(!v){
        return __STREXH(1, sem) ? (0x39): true;//ע�⣬�˴���ǰ̨ʹ��ʱ���ܻ�������ò��ɵ�����������ʱ�������Ż�
    }
    return false;

}
void sem_destory(volatile unsigned short * sem)
{
  
  free((void*)sem);
  sem=NULL;

}