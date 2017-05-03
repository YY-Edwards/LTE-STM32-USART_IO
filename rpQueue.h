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



/* 队列 先入先出*/
/*使用指针是可以的，但是一定要注意，指针是不是有空间的，也就是指针变量是否是赋值的
没有赋值的指针是不能够使用的，使用的话会出错的*/

#define TRUE 1
#define FALSE 0

#pragma pack(1)

typedef struct node{ 

	/* 节点类型 */
  char *data;
  struct node *next;
  
} m_qnode;


typedef struct {  
	
	//这仅仅是两个指针，指向队首和队尾，中间节点仍然是单链表的节点
    m_qnode *front; /* 队首指针 */
    m_qnode *rear; /* 队尾指针 */
	
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













