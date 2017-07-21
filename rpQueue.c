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
	//���еĳ�ʼ��
    m_qnode *q;

    /*Ϊq����ָ���һ���ڴ�ռ�*/
    q = (m_qnode *)malloc(sizeof(m_qnode));
    if(q==NULL)return (-1);
    memset(q, 0x00, sizeof(m_qnode));
    //bzero(q, sizeof(m_qnode));

    /*Ϊq->data����ָ���һ���ڴ�ռ�*/
    q->data = (char *)malloc(deep * elementsize);
    if (q->data == NULL)return (-1);
    memset(q->data, 0x00, deep * elementsize);
    //bzero((q->data), deep * elementsize);//clear

    //printf("sizeof(m_qnode) is : %d\n", sizeof(m_qnode));

    qp->front = q;        //���׺Ͷ�β��ָͬ��ͬһ���ڵ�/
    qp->front->next = NULL;
    qp->rear = qp->front;

        //printf("2.sizeof(q->data) is :%d\n", sizeof(q->data));
        //bzero((q->data), sizeof(q->data));//clear

    return TRUE;
    
}


int queuePush(m_qpointer *qp, char *i_buffer, unsigned int length)
{    //�����
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
{    //�ж϶����Ƿ�Ϊ��
    if (qp->front == qp->rear)
        return TRUE;
    return FALSE; /*��䲻���٣����˻�����*/
	
}


int queuePop(m_qpointer *qp, char *o_buffer, unsigned int length)
{    
	//������
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

    if (isEmpty(qp)) {//�ж�

        free(q->data);
        free(q);
        q->data = NULL;
        q = NULL;
        return (-2);
    }
        
	//printf("4.2\n");
	
    q = qp->front->next;//���ض��׵����ݰ���ַ
    memcpy(o_buffer, q->data, length);	
    qp->front->next = q->next;
    if (qp->rear == q) //�ж��ǲ����Ѿ����˶��е����
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















