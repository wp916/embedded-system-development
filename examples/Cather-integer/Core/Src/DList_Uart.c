#include "Dlist_Uart.h"
#include "freertos.h"
#include "task.h"	//实现FreeRTOS的任务挂起和启动（临界段和开关中断）
#include <string.h>	//数据拷贝

#include "usart.h"

void CopyData_DList(DNode* pnode, const uint8_t* data,const uint16_t dataLength)
{
	pnode->pdata = (uint8_t *)mymalloc(SRAMIN,dataLength *sizeof(uint8_t));
	pnode->dataLength=dataLength;
//	for(uint16_t i=0 ; i<dataLength; i++) {pnode->pdata[i]=data[i];}
	memcpy(pnode->pdata,data,dataLength*sizeof(uint8_t));
}

void DeleteData_DList(DNode* pnode)
{	
	myfree(SRAMIN,pnode->pdata);
	myfree(SRAMIN,pnode);
}

void Create_DList(DList* dList)
{	
	uint8_t a='\r';
	CopyData_DList(&(dList->head),&a,1);
	CopyData_DList(&(dList->tail),&a,1);
	
	dList->length = 0;
	dList->loadingFirstFlag = 0;
	dList->head.next = &(dList->tail);
	dList->tail.prev = &(dList->head);
}

void PushHead_DList(DList* dList, const uint8_t* data,const uint16_t dataLength)
{
	dList->loadingFirstFlag = 1 ;
	DNode* pInsert = (DNode*)mymalloc(SRAMIN,sizeof(DNode));
	
	taskDISABLE_INTERRUPTS();
//	uint32_t tasklock=taskENTER_CRITICAL_FROM_ISR();
	dList->length++;
	pInsert->prev = &(dList->head);
	pInsert->next = dList->head.next;
	dList->head.next->prev = pInsert;
	dList->head.next=pInsert;	
	taskENABLE_INTERRUPTS();
//	taskEXIT_CRITICAL_FROM_ISR(tasklock);
	
	CopyData_DList(pInsert, data,dataLength);
	dList->loadingFirstFlag = 0 ;
}

void PushTail_DList(DList* dList, const uint8_t* data,const uint16_t dataLength)
{
	if(dList->length==0)	{dList->loadingFirstFlag = 1 ;}	
	
	DNode* pInsert = (DNode*)mymalloc(SRAMIN,sizeof(DNode));
	
	taskDISABLE_INTERRUPTS();
//	uint32_t tasklock=taskENTER_CRITICAL_FROM_ISR();
	dList->length++;
	pInsert->next = &(dList->tail);
	pInsert->prev = dList->tail.prev;
	dList->tail.prev->next = pInsert;
	dList->tail.prev=pInsert;	
	taskENABLE_INTERRUPTS();
//	taskEXIT_CRITICAL_FROM_ISR(tasklock);
	
	CopyData_DList(pInsert, data, dataLength);
	dList->loadingFirstFlag = 0 ;
}

void PopHead_DList(DList* dList,uint8_t *pdata,uint16_t* length)
{
	//如果链表已经为空
	if (dList->length == 0) { return ; }	
	
	taskDISABLE_INTERRUPTS();
//	uint32_t tasklock=taskENTER_CRITICAL_FROM_ISR();
	dList->length--;
	DNode* pNode = dList->head.next;
	pNode->next->prev = &(dList->head);
	dList->head.next = pNode->next;	
	taskENABLE_INTERRUPTS();
//	taskEXIT_CRITICAL_FROM_ISR(tasklock);
	
	*length = pNode->dataLength;
	memcpy(pdata,pNode->pdata,pNode->dataLength*sizeof(uint8_t));
	
	DeleteData_DList(pNode);	
}

void PopTail_DList(DList* dList,uint8_t *pdata,uint16_t* length)
{
	//如果链表已经为空
	if (dList->length == 0) { return ; }	
	
	taskDISABLE_INTERRUPTS();
//	uint32_t tasklock=taskENTER_CRITICAL_FROM_ISR();
	dList->length--;
	DNode* pNode = dList->tail.prev;
	pNode->prev->next = &(dList->tail);
	dList->tail.prev =pNode->prev;	
	taskENABLE_INTERRUPTS();
//	taskEXIT_CRITICAL_FROM_ISR(tasklock);
	
	*length = pNode->dataLength;
	memcpy(pdata,pNode->pdata,pNode->dataLength*sizeof(uint8_t));
	
	DeleteData_DList(pNode);
}

void Print_DList(DList* dList)
{
	if (dList->length == 0)
	{
		printf("Empty DoublyLinkedList");
		return;
	}

	DNode* pMove;	
	pMove = dList->head.next;
	while (pMove != &(dList->tail))
	{
		HAL_UART_Transmit(&huart1,pMove->pdata,pMove->dataLength,20);//非阻塞方式打印,串口1
		pMove = pMove->next;
	}
	printf("\r\n");
}

void PrintReverse_DList(DList* dList)
{
	if (dList->length == 0)
	{
		printf("Empty DoublyLinkedList");
		return;
	}

	DNode* pMove;
	pMove = dList->tail.prev;
	while (pMove != &(dList->head))
	{
		HAL_UART_Transmit(&huart1,pMove->pdata,pMove->dataLength,20);//非阻塞方式打印,串口1
		pMove = pMove->prev;
	}
	printf("\r\n");
}

//void Clear_DList(DList* dList)
//{
//	while(dList->length > 0)
//	{
//		PopHead_DList(dList);
//	}
//}

