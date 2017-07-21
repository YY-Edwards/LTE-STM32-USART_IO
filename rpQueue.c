/*
* rpqueue.c
*
* Created: 2016/11/10
* Author: EDWARDS
*/
#include "rpQueue.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>


int queueInit(m_qpointer *qp, unsigned short deep, unsigned short elementsize)
{    
    if ((0 == deep) || (0 == elementsize)){
    
            return FALSE;
    }
	//队列的初始化
    m_qnode *q;

    /*为q分配指向的一段内存空间*/
    q = (m_qnode *)malloc(sizeof(m_qnode));
    if(q==NULL)return (-1);
    memset(q, 0x00, sizeof(m_qnode));
    //bzero(q, sizeof(m_qnode));

    /*为q->data分配指向的一段内存空间*/
    q->data = (char *)malloc(deep * elementsize);
    if (q->data == NULL)return (-1);
    memset(q->data, 0x00, deep * elementsize);
    //bzero((q->data), deep * elementsize);//clear

    //printf("sizeof(m_qnode) is : %d\n", sizeof(m_qnode));

    qp->front = q;        //队首和队尾共同指向同一个节点/
    qp->front->next = NULL;
    qp->rear = qp->front;

        //printf("2.sizeof(q->data) is :%d\n", sizeof(q->data));
        //bzero((q->data), sizeof(q->data));//clear

    return TRUE;
    
}


int queuePush(m_qpointer *qp, char *i_buffer, unsigned int length)
{    //入队列
    m_qnode *p; 
    p = (m_qnode *)malloc(sizeof(m_qnode));
    if(p==NULL)return (-1);
    memset(p, 0x00, sizeof(m_qnode));
    //bzero(p, sizeof(m_qnode));

    p->data = (char *)malloc(length);
    if (p->data == NULL)return (-1);
    memset(p->data, 0x00, length);
    //bzero((p->data), length);//clear


    if (qp == NULL)
    {	
        //printf("qp is NULL \n");
        free(p->data);
        free(p);
        p->data = NULL;
        p = NULL;
        return (-3);
    }

    memcpy(p->data, i_buffer, length);  
    p->next = NULL;
    qp->rear->next = p;
    qp->rear = p;

	//printf("-4.0-\n");

    return TRUE;
}

int isEmpty(m_qpointer *qp)	
{    //判断队列是否为空
    if (qp->front == qp->rear)
        return TRUE;
    return FALSE; /*这句不能少，少了会出错的*/
	
}


int queuePop(m_qpointer *qp, char *o_buffer, unsigned int length)
{    
	//出队列
    m_qnode *q;
    q = (m_qnode *)malloc(sizeof(m_qnode));
    if(q==NULL)return (-1);
    memset(q, 0x00, sizeof(m_qnode));
    //bzero(q, sizeof(m_qnode));

    q->data = (char *)malloc(length);
    if (q->data == NULL)return (-1);
    memset(q->data, 0x00, length);
    //bzero((q->data), length);//clear
    
    //printf("4.1\n");

    if (qp == NULL){
        //printf("qp is NULL \n");
        free(q->data);
        free(q);
        q->data = NULL;
        q = NULL;
        return (-3);
    }

    if (isEmpty(qp)) {//判断

        free(q->data);
        free(q);
        q->data = NULL;
        q = NULL;
        return (-2);
    }
        
	//printf("4.2\n");
	
    q = qp->front->next;//返回队首的数据包地址
    memcpy(o_buffer, q->data, length);	
    qp->front->next = q->next;
    if (qp->rear == q) //判断是不是已经到了队列的最后
    {
        qp->rear = qp->front;
    }
    
    free(q->data);
    q->data = NULL;
    free(q);
    q = NULL;

	//printf("-4.1-\n");
	
    return TRUE;
}















