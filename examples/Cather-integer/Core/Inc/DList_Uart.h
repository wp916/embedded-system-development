//*****************************************************************************
//实现带头结点、尾结点的双向链表,专用于串口的数据结构
//
//基于C语言的双向链表
//双向链表的几个性质
// 1：链表必存在pHead和PTail两个指针，如果链表为空，两指针=NULL
// 如果链表只有一个节点，则两指针皆指向此地址
//2：pHead和Ptail两个指针应该在整个程序中保持不变，并且是链表真正的头和尾，
//不要将链表中的某一段专门拿出来做处理，这样可能导致链表被切断。
//比如 1←→2←→3←→4←→5←→6←→7←→8←→9  不要只处理 4←→5←→6这一段，
//否则可能变成1←→2←→3  4←→5←→6  7←→8←→9 三段，并且想要再找回来很难


#ifndef _DOUBLYLINKEDLIST_UART_H_
#define _DOUBLYLINKEDLIST_UART_H_

#include <stdio.h>
//#include <stdlib.h>
#include "malloc.h"
#include "stdint.h"

//*****************************************************************************

//每个节点所要存储的数据
typedef struct _DNode
{	
	struct _DNode* prev;
	struct _DNode* next;
	
	uint8_t * pdata;	//存储数据的首地址
	uint16_t dataLength;//数据长度
}DNode;

//链表
typedef struct _DList
{
	//一个链表，必须永远有头有尾，为了简化程序，就假设了这两个空位置
	DNode head;
	DNode tail;
	
	volatile uint8_t loadingFirstFlag;	//0:数据写入操作完成；1：正在写入数据
	volatile uint16_t length;	//链表实际长度（不算头尾）
}DList;
//*****************************************************************************


//创建两块内存，并将头尾分别指向他们
void Create_DList(DList *dList);

//向链表头部插入元素
void PushHead_DList(DList *dList, const uint8_t* data,const uint16_t dataLength);

//向链表尾部插入元素
void PushTail_DList(DList *dList, const uint8_t* data,const uint16_t dataLength);

//删除链表头部元素
//注意：这只是将节点从链表中移除，但数据内存没有清除，此节点内存需要手动释放
void PopHead_DList(DList *dList,uint8_t *pdata,uint16_t* length);

//删除链表尾部元素
//注意：这只是将节点从链表中移除，但数据内存没有清除，此节点内存需要手动释放
void PopTail_DList(DList *dList,uint8_t *pdata,uint16_t* length);

//正序打印链表
void Print_DList(DList *dList);

//逆序打印链表
void PrintReverse_DList(DList *dList);

//清除链表中所有元素，置为空表
//void Clear_DList(DList *dList);
//*****************************************************************************


#endif // !_DOUBLYLINKEDLIST_H_
