/*
 * rpqueue.h
 *
 * Created: 2016/11/10
 * Author: EDWARDS
 */ 


#ifndef RPQUEUE_H_
#define RPQUEUE_H_
#include "stdlib.h"
#include "string.h"



/* ���� �����ȳ�*/
/*ʹ��ָ���ǿ��Եģ�����һ��Ҫע�⣬ָ���ǲ����пռ�ģ�Ҳ����ָ������Ƿ��Ǹ�ֵ��
û�и�ֵ��ָ���ǲ��ܹ�ʹ�õģ�ʹ�õĻ�������*/

#define TRUE 1
#define FALSE 0

#pragma pack(1)

typedef struct node{ 

	/* �ڵ����� */
  char *data;
  struct node *next;
  
} m_qnode;


typedef struct {  
	
	//�����������ָ�룬ָ����׺Ͷ�β���м�ڵ���Ȼ�ǵ�����Ľڵ�
    m_qnode *front; /* ����ָ�� */
    m_qnode *rear; /* ��βָ�� */
	
} m_qpointer;


typedef struct{


	int(*QueueInit)(m_qpointer *qp, unsigned short deep, unsigned short elementsize);
	int(*QueuePush)(m_qpointer *qp, char *i_buffer, unsigned int length);
	int(*QueuePop)(m_qpointer *qp, char *o_buffer, unsigned int length);


}rpqueuefunc_t;



#pragma pack()


int queueInit(m_qpointer *qp, unsigned short deep, unsigned short elementsize);
int queuePush(m_qpointer *qp, char *i_buffer, unsigned int length);
int isEmpty(m_qpointer *qp);
int queuePop(m_qpointer *qp, char *o_buffer, unsigned int length);


#endif













