#include "Fos_Uart.h"
#include "DList_Uart.h"

#include "freertos.h"
#include "task.h"
#include "cmsis_os.h"	//定时器相关句柄定义
#include <string.h>	//数据拷贝

/***************************************************************主要发送逻辑***********************************************************************
								 +-------------------------------------------------------------------+                           
								 |    FOS_UART_Transmit;FOS_UART_Transmit_IT;FOS_UART_Transmit_DMA   |                           
								 +-------------------------------------------------------------------+                           
																  |                                                            
																  V                                                            
											 Y /-------------------------------------\ N                                       
							   +---------------|         此串口有数据正在发送        |---------------+                         
							   |               \-------------------------------------/               |                         
							   |                                                                     |                         
							   V                                                                     V                         
		 +-------------------------------------------+                         +-------------------------------------------+   
		 |           将发送数据添加到缓冲队列        |                         |      用HAL_UART_Transmit_DMA发送数据      |   
		 +-------------------------------------------+                         +-------------------------------------------+   
							   |                                                                     |                         
							   |                                                                     |                         
							   |   +-------------------------------------------------------------+   |                         
							   +-->|           当DMA发送完成后触发DMA发送完成回调函数            |<--+                         
								   +-------------------------------------------------------------+                             
																  |                                                            
																  V                                                            
											/-------------------------------------------\ N                                    
											|         此串口的发送缓冲区有数据          |-------------------------------------+
											\-------------------------------------------/                                     |
																  | Y                                                         |
																  V                                                           |
								Y /---------------------------------------------------------------\ N                         |
							  +---|           没有新的发送数据正在向缓冲链表头部写入              |---+                       |
							  |   \---------------------------------------------------------------/   |                       |
							  |                                                                       |                       |
							  V                                                                       V                       |
		  +---------------------------------------+                               +---------------------------------------+   |
		  |        提取出缓冲链表头部数据         |                               |          打开循环检测发送功能；       |   |
		  +---------------------------------------+                               +---------------------------------------+   |
							  |                                                                       |                       |
							  V                                                                       |                       |
		+-------------------------------------------+                                                 |                       |
		|       用HAL_UART_Transmit_DMA发送数据     |                                                 |                       |
		+-------------------------------------------+                                                 |                       |
							  |                                                                       |                       |
							  +---------------------------------->O<----------------------------------+                       |
																  |                                                           |
																  V                                                           |
											   +-------------------------------------+                                        |
											   |         循环检测发送功能开启        |<---------------------------------------+
											   +-------------------------------------+                                         
																  |                                                            
																  V                                                            
										 N /---------------------------------------------\                                     
	+--------------------------------------|        任何串口的发送缓冲区有数据           |<-----------------------+            
	|                                      \---------------------------------------------/                        |            
	|                                                             | Y                                             |            
	|                                                             V                                               |            
	|                           Y /---------------------------------------------------------------\ N             |            
	|                         +---|           没有新的发送数据正在向缓冲链表头部写入              |---+           |            
	|                         |   \---------------------------------------------------------------/   |           |            
	|                         |                                                                       |           |            
	|                         V                                                                       V           |            
	|     +---------------------------------------+                                           +---------------+   |            
	|     |        提取出缓冲链表头部数据         |                                           |   延时1ms     |   |            
	|     +---------------------------------------+                                           +---------------+   |            
	|                         |                                                                       |           |            
	|                         V                                                                       |           |            
	|   +-------------------------------------------+                                                 |           |            
	|   |      用HAL_UART_Transmit_DMA发送数据      |                                                 |           |            
	|   +-------------------------------------------+                                                 |           |            
	|                         |                                                                       |           |            
	|                         +---------------------------------->O<----------------------------------+           |            
	|                                                             |                                               |            
	|                                                             V                                               |            
	|                                                             O-----------------------------------------------+            
	|                                                                                                                          
	|                                                                                                                          
	+------------------------------------------------------------>O                 
						  
**************************************************************主要发送逻辑************************************************************************/


/****************************定长数据接收逻辑**********************************

	初始化：
			  +---------------------------------------------+      
			  |          使能DMA半完成和完成接收中断        |      
			  +---------------------------------------------+      
									 |                             
									 V                             
		+---------------------------------------------------------+
		|            设置DMA接收数组长度 = 2*返回数据长度         |
		+---------------------------------------------------------+
									 |                             
									 V                             
		+---------------------------------------------------------+
		|          设置DMA接收数据长度 = 2*返回数据长度           |
		+---------------------------------------------------------+
									


	半完成中断：
					 +-------------------------------+             
					 |        DMA半完成中断触发      |             
					 +-------------------------------+             
									 |                             
									 V                             
				 +---------------------------------------+         
				 |        拷贝接收数组前半段数据         |         
				 +---------------------------------------+         
									 |                             
									 V                             
				  +-------------------------------------+          
				  |         触发串口接收回调函数        |          
				  +-------------------------------------+          
									 |                             
									 V                             
						+-------------------------+                
						|       用户处理数据      |                
						+-------------------------+                
		
	
	完成中断：                            
					   +---------------------------+               
					   |      DMA完成中断触发      |               
					   +---------------------------+     
									 |                             
									 V                             
				 +---------------------------------------+         
				 |        拷贝接收数组后半段数据         |         
				 +---------------------------------------+         
									 |                             
									 V                             
				  +-------------------------------------+          
				  |        触发串口接收回调函数         |          
				  +-------------------------------------+          
									 |                             
									 V                             
						+-------------------------+                
						|       用户处理数据      |                
						+-------------------------+                
									 |                             
									 V                             
			  +---------------------------------------------+      
			  |         使能DMA半完成和完成接收中断         |      
			  +---------------------------------------------+      


****************************定长数据接收逻辑**********************************/

/*不定长数据接收逻辑和最通用的串口DMA + 串口IDLE中断 逻辑一模一样*/


/************************************ 全局变量定义************************************************/	
//串口缓冲结构体
typedef struct _UartBufferHandle{
	UART_HandleTypeDef *huart;
	uint8_t id;		//串口号ID
	DList uartDList;
	volatile uint8_t busyFlag;	//1:串口忙；0：串口空闲(他决定新数据是立即发送还是暂存缓冲区)
}UartBufferHandle;

#if (USE_UART1 > 0)	//使用串口1
UartBufferHandle uart1Buffer={
	.huart=&huart1,
	.id=1,
	.busyFlag =0,};
uint8_t uart1_tx[UART1_TX_MAX];
#endif
#if (USE_UART1 == 2)	//确定接收数据长度的接收方式
	uint8_t uart1_Rx_Hfcplt_Buff[UART1_RX_MAX];
	uint8_t uart1_Rx_Cplt_Buff[UART1_RX_MAX];
	uint8_t uart1_Rx_Buff[UART1_RX_MAX*2];
#endif	
#if (USE_UART1 == 3)	//不确定接收数据长度的接收方式	
	uint8_t uart1_Rx_Buff[UART1_RX_MAX];
#endif

#if (USE_UART2 > 0)	//使用串口2
UartBufferHandle uart2Buffer={
	.huart=&huart2,
	.id=2,
	.busyFlag =0,};
uint8_t uart2_tx[UART2_TX_MAX];
#endif
#if (USE_UART2 == 2)	//确定接收数据长度的接收方式
	uint8_t uart2_Rx_Hfcplt_Buff[UART2_RX_MAX];
	uint8_t uart2_Rx_Cplt_Buff[UART2_RX_MAX];
	uint8_t uart2_Rx_Buff[UART2_RX_MAX*2];
#endif	
#if (USE_UART2 == 3)	//不确定接收数据长度的接收方式	
	uint8_t uart2_Rx_Buff[UART2_RX_MAX];
#endif
	
#if (USE_UART3 > 0)	//使用串口3
UartBufferHandle uart3Buffer={
	.huart=&huart3,
	.id=3,
	.busyFlag =0,};
uint8_t uart3_tx[UART3_TX_MAX];
#endif
#if (USE_UART3 == 2)	//确定接收数据长度的接收方式
	uint8_t uart3_Rx_Hfcplt_Buff[UART3_RX_MAX];
	uint8_t uart3_Rx_Cplt_Buff[UART3_RX_MAX];
	uint8_t uart3_Rx_Buff[UART3_RX_MAX*2];
#endif	
#if (USE_UART3 == 3)	//不确定接收数据长度的接收方式	
	uint8_t uart3_Rx_Buff[UART3_RX_MAX];
#endif
	
#if (USE_UART4 > 0)	//使用串口4
UartBufferHandle uart4Buffer={
	.huart=&huart4,
	.id=4,
	.busyFlag =0,};
uint8_t uart4_tx[UART4_TX_MAX];
#endif
#if (USE_UART4 == 2)	//确定接收数据长度的接收方式
	uint8_t uart4_Rx_Hfcplt_Buff[UART4_RX_MAX];
	uint8_t uart4_Rx_Cplt_Buff[UART4_RX_MAX];
	uint8_t uart4_Rx_Buff[UART4_RX_MAX*2];
#endif	
#if (USE_UART4 == 3)	//不确定接收数据长度的接收方式	
	uint8_t uart4_Rx_Buff[UART4_RX_MAX];
#endif

UART_Rx_CallBack UART1_Rx_CallBack; //串口1的接收回调函数
UART_Rx_CallBack UART2_Rx_CallBack; //串口2的接收回调函数
UART_Rx_CallBack UART3_Rx_CallBack; //串口3的接收回调函数
UART_Rx_CallBack UART4_Rx_CallBack; //串口4的接收回调函数
	
/********************************* 函数声明************************************************/	
UartBufferHandle* GetBufferHandle(UART_HandleTypeDef *huart);
void StrCombine(uint8_t*array,uint16_t *realSize,const char* str,va_list arp);
void UART1_ReceiveDistributeCllback(UART_HandleTypeDef *huart,uint16_t Size);
void UART_SoftTimerCallBack(void *argument);

void UART1_RxHCplt_CallBack(UART_HandleTypeDef *huart);        			/*!< UART Rx Half Complete Callback        */
void UART1_RxCplt_CallBack(UART_HandleTypeDef *huart);            		/*!< UART Rx Complete Callback             */
void UART2_RxHCplt_CallBack(UART_HandleTypeDef *huart);        			/*!< UART Rx Half Complete Callback        */
void UART2_RxCplt_CallBack(UART_HandleTypeDef *huart);            		/*!< UART Rx Complete Callback             */
void UART3_RxHCplt_CallBack(UART_HandleTypeDef *huart);        			/*!< UART Rx Half Complete Callback        */
void UART3_RxCplt_CallBack(UART_HandleTypeDef *huart);            		/*!< UART Rx Complete Callback             */
void UART4_RxHCplt_CallBack(UART_HandleTypeDef *huart);        			/*!< UART Rx Half Complete Callback        */
void UART4_RxCplt_CallBack(UART_HandleTypeDef *huart);            		/*!< UART Rx Complete Callback             */
void UART1_IDLE_CallBack(UART_HandleTypeDef *huart,uint16_t Size);
void UART2_IDLE_CallBack(UART_HandleTypeDef *huart,uint16_t Size);
void UART3_IDLE_CallBack(UART_HandleTypeDef *huart,uint16_t Size);
void UART4_IDLE_CallBack(UART_HandleTypeDef *huart,uint16_t Size);
void UART1_TxCplt_CallBack(UART_HandleTypeDef *huart);            		/*!< UART Tx Complete Callback             */
void UART2_TxCplt_CallBack(UART_HandleTypeDef *huart);            		/*!< UART Tx Complete Callback             */
void UART3_TxCplt_CallBack(UART_HandleTypeDef *huart);            		/*!< UART Tx Complete Callback             */
void UART4_TxCplt_CallBack(UART_HandleTypeDef *huart);            		/*!< UART Tx Complete Callback             */

#if (USE_UART1 >= 2)
void _UART1_Rx_CallBack(UART_HandleTypeDef *huart,const uint8_t *pdata,const uint16_t size) {}
#endif
#if (USE_UART2 >= 2)
void _UART2_Rx_CallBack(UART_HandleTypeDef *huart,const uint8_t *pdata,const uint16_t size) {}
#endif
#if (USE_UART3 >= 2)
void _UART3_Rx_CallBack(UART_HandleTypeDef *huart,const uint8_t *pdata,const uint16_t size) {}
#endif
#if (USE_UART4 >= 2)
void _UART4_Rx_CallBack(UART_HandleTypeDef *huart,const uint8_t *pdata,const uint16_t size) {}
#endif
	
volatile uint8_t timerOpenFlag = 0;	//1：定时器打开；0：定时器关闭
	
osThreadId_t UartSendTaskHandle;
const osThreadAttr_t uartSendTask_attributes = {
  .name = "uartSendTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

#if USART1_DEVICE_NUM
//串口1上绑定的设备及其处理函数
void* uart1_device[USART1_DEVICE_NUM];
UART1_ReceiveDecodeFuc Uart1_ReceivedDecode[USART1_DEVICE_NUM];
uint8_t uart1_receive[UART1_RX_MAX];		//串口返回数据存储位置	
#endif

//如果在“会被Usmart调用的函数”中通过串口发送数据，需要延迟至usmart主程序发送完成后再发送，否则会出问题
volatile uint8_t usmartSendFlag=0;

void UART_Error_CallBack(UART_HandleTypeDef *huart);       				/*!< UART Error Callback                   */
void UART_ABORT_COMPLETE_CallBack(UART_HandleTypeDef *huart);         	/*!< UART Abort Complete Callback          */
void UART_ABORT_TRANSMIT_COMPLETE_CallBack(UART_HandleTypeDef *huart); 	/*!< UART Abort Transmit Complete Callback */
void UART_ABORT_RECEIVE_COMPLETE_CallBack(UART_HandleTypeDef *huart);  	/*!< UART Abort Receive Complete Callback  */

/****************************** 函数的实现机制 public************************************************/
/**
* @breif	初始化串口1的数据缓冲区
*/

uint8_t FOS_InitUartBuffer(void)
{
	static uint8_t flag=1;
	if(flag)
	{	
/*---------------------------------------串口1配置--------------------------------------------*/
#if (USE_UART1 > 0)	//使用串口1		
		Create_DList(&(uart1Buffer.uartDList));	
		HAL_UART_RegisterCallback(&huart1,HAL_UART_TX_COMPLETE_CB_ID,	 UART1_TxCplt_CallBack);		
#endif		
#if (USE_UART1 == 2)	//确定接收数据长度的接收方式
		//注册串口完成、半完成接收中断到指定函数
		HAL_UART_RegisterCallback(&huart1,HAL_UART_RX_HALFCOMPLETE_CB_ID,UART1_RxHCplt_CallBack);
		HAL_UART_RegisterCallback(&huart1,HAL_UART_RX_COMPLETE_CB_ID,	 UART1_RxCplt_CallBack);
		UART1_Rx_CallBack = _UART1_Rx_CallBack;
		//启用DMA接收
		HAL_UART_Receive_DMA(&huart1,uart1_Rx_Buff,UART1_RX_MAX*2);
#endif
#if (USE_UART1 == 3)	//不确定接收数据长度的接收方式	
		//使能空闲串口中断
		__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);	//Idle line detection interrupt 空闲总线中断		
		//注册串口闲时接收中断到自己的自定义函数
		HAL_UART_RegisterRxEventCallback(&huart1,UART1_ReceiveDistributeCllback);
		//启用DMA接收
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,uart1_receive,UART1_RX_MAX);
#endif
		
/*---------------------------------------串口2配置--------------------------------------------*/
#if (USE_UART2 > 0)	//使用串口2
		Create_DList(&(uart2Buffer.uartDList));	
		HAL_UART_RegisterCallback(&huart2,HAL_UART_TX_COMPLETE_CB_ID,	 UART2_TxCplt_CallBack);
#endif		
#if (USE_UART2 == 2)	//确定接收数据长度的接收方式
		//注册串口完成、半完成接收中断到指定函数
		HAL_UART_RegisterCallback(&huart2,HAL_UART_RX_HALFCOMPLETE_CB_ID,UART2_RxHCplt_CallBack);
		HAL_UART_RegisterCallback(&huart2,HAL_UART_RX_COMPLETE_CB_ID,	 UART2_RxCplt_CallBack);
		UART2_Rx_CallBack = _UART2_Rx_CallBack;
		HAL_UART_Receive_DMA(&huart2,uart2_Rx_Buff,UART2_RX_MAX*2);
#endif
#if (USE_UART2 == 3)	//不确定接收数据长度的接收方式	
		//使能空闲串口中断
		__HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
		//注册串口闲时接收中断到自己的自定义函数
		HAL_UART_RegisterRxEventCallback(&huart2,UART2_IDLE_CallBack);
		UART2_Rx_CallBack = _UART2_Rx_CallBack;
		//启用DMA接收
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2,uart2_Rx_Buff,UART2_RX_MAX);
#endif

/*---------------------------------------串口3配置--------------------------------------------*/
#if (USE_UART3 > 0)	//使用串口3
		Create_DList(&(uart3Buffer.uartDList));	
		HAL_UART_RegisterCallback(&huart3,HAL_UART_TX_COMPLETE_CB_ID,	 UART3_TxCplt_CallBack);
#endif		
#if (USE_UART3 == 2)
		HAL_UART_RegisterCallback(&huart3,HAL_UART_RX_HALFCOMPLETE_CB_ID,UART3_RxHCplt_CallBack);
		HAL_UART_RegisterCallback(&huart3,HAL_UART_RX_COMPLETE_CB_ID,	 UART3_RxCplt_CallBack);
		UART3_Rx_CallBack = _UART3_Rx_CallBack;
		HAL_UART_Receive_DMA(&huart3,uart3_Rx_Buff,UART3_RX_MAX*2);
#endif
#if (USE_UART3 == 3)
		__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
		HAL_UART_RegisterRxEventCallback(&huart3,UART3_IDLE_CallBack);
		UART3_Rx_CallBack = _UART3_Rx_CallBack;
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3,uart3_Rx_Buff,UART3_RX_MAX);
#endif

/*---------------------------------------串口4配置--------------------------------------------*/
#if (USE_UART4 > 0)	//使用串口4
		Create_DList(&(uart4Buffer.uartDList));
		HAL_UART_RegisterCallback(&huart4,HAL_UART_TX_COMPLETE_CB_ID,	 UART4_TxCplt_CallBack);
#endif		
#if (USE_UART4 == 2)
		HAL_UART_RegisterCallback(&huart4,HAL_UART_RX_HALFCOMPLETE_CB_ID,UART4_RxHCplt_CallBack);
		HAL_UART_RegisterCallback(&huart4,HAL_UART_RX_COMPLETE_CB_ID,	 UART4_RxCplt_CallBack);
		UART4_Rx_CallBack = _UART4_Rx_CallBack;
		HAL_UART_Receive_DMA(&huart4,uart4_Rx_Buff,UART4_RX_MAX*2);
#endif
#if (USE_UART4 == 3)
		__HAL_UART_ENABLE_IT(&huart4,UART_IT_IDLE);
		HAL_UART_RegisterRxEventCallback(&huart4,UART4_IDLE_CallBack);
		UART4_Rx_CallBack = _UART4_Rx_CallBack;
		HAL_UARTEx_ReceiveToIdle_DMA(&huart4,uart4_Rx_Buff,UART4_RX_MAX);
#endif


#if USE_DEBUG_TOOlS			
//		__HAL_UART_ENABLE_IT(&huart1,UART_IT_CTS);	//CTS change interrupt CTS中断
//		__HAL_UART_ENABLE_IT(&huart1,UART_IT_LBD);	//LIN Break detection interrupt LIN中断检测中断
//		__HAL_UART_ENABLE_IT(&huart1,UART_IT_TXE);	//Transmit Data Register empty interrupt 发送中断
//		__HAL_UART_ENABLE_IT(&huart1,UART_IT_TC);	//Transmission complete interrupt 传输完成中断
//		__HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);	//Receive Data register not empty interrupt 接收中断
//		__HAL_UART_ENABLE_IT(&huart1,UART_IT_PE);	//Parity Error interrupt 奇偶错误中断
		__HAL_UART_ENABLE_IT(&huart1,UART_IT_ERR);	//Error interrupt(Frame error, noise error, overrun error) 错误中断		
		
		HAL_UART_RegisterCallback(&huart1,HAL_UART_ERROR_CB_ID,			 		 UART_Error_CallBack);
		HAL_UART_RegisterCallback(&huart1,HAL_UART_ABORT_COMPLETE_CB_ID,		 UART_ABORT_COMPLETE_CallBack);
		HAL_UART_RegisterCallback(&huart1,HAL_UART_ABORT_TRANSMIT_COMPLETE_CB_ID,UART_ABORT_TRANSMIT_COMPLETE_CallBack);
		HAL_UART_RegisterCallback(&huart1,HAL_UART_ABORT_RECEIVE_COMPLETE_CB_ID, UART_ABORT_RECEIVE_COMPLETE_CallBack);
//		HAL_UART_RegisterCallback(&huart1,HAL_UART_WAKEUP_CB_ID,				 UART_WAKEUP_CallBack);
#endif
		
		
#if USE_USMART		
		usmart_init();
		//注册串口闲时接收中断到自己的自定义函数
		HAL_UART_RegisterRxEventCallback(&huart1,UART1_ReceiveDistributeCllback);
		//启用DMA接收
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,uart1_receive,UART1_RX_MAX);
#endif


		UartSendTaskHandle = osThreadNew(UART_SoftTimerCallBack, NULL, &uartSendTask_attributes);
		flag=0;
	}
	return 1;
}

/**
* @breif	为 HAL_UART_Transmit 函数添加数据缓冲功能
* @notice	其底层仍然是DMA发送，但会尽最大可能优先发送
*/
HAL_StatusTypeDef FOS_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout)
{	
#if USE_USMART
	if((huart == &huart1) && (usmartSendFlag))
	{
		//数据会插队优先发送
		PushHead_DList(&(uart1Buffer.uartDList),pData,Size);
		return HAL_OK;
	}
#endif
		
	UartBufferHandle *uartBuffer = GetBufferHandle(huart);
	if(uartBuffer->busyFlag)
	{
#if USE_DEBUG_TOOlS
		if(uartBuffer->uartDList.length == USART_BUFFER_ALARM )
		{printf("uart%d to much buffer\r\n",uartBuffer->id);}
#endif
		//数据会插队优先发送
		PushHead_DList(&(uartBuffer->uartDList),pData,Size);
		return HAL_OK;
	}
	else 
	{
		uartBuffer->busyFlag = 1 ;
		return HAL_UART_Transmit_DMA(huart,pData,Size);
	}
}

/**
* @breif 为 HAL_UART_Transmit_DMA 函数添加数据缓冲功能
*/
HAL_StatusTypeDef FOS_UART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)
{
#if USE_USMART
	if((huart == &huart1) && (usmartSendFlag))
	{
		//数据会插队优先发送
		PushHead_DList(&(uart1Buffer.uartDList),pData,Size);
		return HAL_OK;
	}
#endif
	
	UartBufferHandle *uartBuffer = GetBufferHandle(huart);
	if(uartBuffer->busyFlag)
	{
#if USE_DEBUG_TOOlS
		//数据会按先后顺序排队发送
		if(uartBuffer->uartDList.length == USART_BUFFER_ALARM )
		{printf("uart%d to much buffer\r\n",uartBuffer->id);}
#endif
		PushTail_DList(&(uartBuffer->uartDList),pData,Size);
		return HAL_OK;
	}
	else 
	{
		uartBuffer->busyFlag = 1 ; 
		return HAL_UART_Transmit_DMA(huart,pData,Size);
	}
}

/**
* @breif 与printf相同用法的格式化字符串输出函数
			底层实现是非阻塞式DMA排队发送，相比printf 可以解决多个printf同时调用，数据交叉发送的bug */
void FOS_printf(const char* str,...)
{
	va_list arp;
	va_start(arp,str);		

	//为 FOS_printf准备的发送字符串
	uint8_t printArray[UART1_TX_MAX];
	uint16_t printArrayIndex=0;	
	
	//变长参数栈始点在str
	StrCombine(printArray,&printArrayIndex,str,arp);

	printArray[printArrayIndex]='\0';	
	va_end(arp);
	
	if((uart1Buffer.busyFlag)	||	(usmartSendFlag))
	{
#if USE_DEBUG_TOOlS
		if(uart1Buffer.uartDList.length == USART_BUFFER_ALARM )	
		{
			static uint8_t info[]={"uart1 to much buffer\r\n"};
			HAL_UART_Transmit(&huart1,info,sizeof(info),10);
		}
#endif
		//数据会按先后顺序排队发送
		PushTail_DList(&(uart1Buffer.uartDList),printArray,printArrayIndex);
		return ;
	}
	else 
	{
		uart1Buffer.busyFlag = 1 ; 
		static uint8_t info[UART1_TX_MAX];
		memcpy(info,printArray,UART1_TX_MAX);
		HAL_UART_Transmit_DMA(&huart1,info,printArrayIndex);
	}
}
	
#if USART1_DEVICE_NUM
uint8_t uart1_device_Num=0;
/**
* @brief	为UART1绑定新的设备及其对应的处理函数
* @param	device：设备结构体
* @param	fuc：此设备返回数据的解析函数
*/
uint8_t UART1_AddDevice(void* device,UART1_ReceiveDecodeFuc fuc)
{
#if USE_DEBUG_TOOlS
	if (uart1_device_Num>=USART1_DEVICE_NUM)	
	{
		static uint8_t info[]={"Too much UART1 device"};
		HAL_UART_Transmit(&huart1,info,sizeof(info),10);
		return 0;
	}
#endif
	
	uart1_device[uart1_device_Num]=device;
	Uart1_ReceivedDecode[uart1_device_Num]=fuc;
	uart1_device_Num++;
	return 1;
}
#endif



/****************************** private 函数************************************************/
/**
* @brief	串口发送完成回调函数
*/
#if (USE_UART1 > 0)
void UART1_TxCplt_CallBack(UART_HandleTypeDef *huart)
{
	if(uart1Buffer.uartDList.length > 0)
	{
		if(uart1Buffer.uartDList.loadingFirstFlag == 0)	//缓冲区的头部数据存储完全
		{
			uint16_t length2;
			PopHead_DList(&(uart1Buffer.uartDList),uart1_tx,&length2);
			HAL_UART_Transmit_DMA(&huart1,uart1_tx,length2);
		}
		else { if(timerOpenFlag == 0) {timerOpenFlag = 1; xTaskResumeFromISR(UartSendTaskHandle); } }
	}	
	else{uart1Buffer.busyFlag = 0 ;}
}
#endif

#if (USE_UART2 > 0)
void UART2_TxCplt_CallBack(UART_HandleTypeDef *huart)
{
	if(uart2Buffer.uartDList.length > 0)
	{
		if(uart2Buffer.uartDList.loadingFirstFlag == 0)	//缓冲区的头部数据存储完全
		{
			uint16_t length2;
			PopHead_DList(&(uart2Buffer.uartDList),uart2_tx,&length2);
			HAL_UART_Transmit_DMA(&huart2,uart2_tx,length2);
		}
		else { if(timerOpenFlag == 0) {timerOpenFlag = 1; xTaskResumeFromISR(UartSendTaskHandle); } }
	}	
	else{uart2Buffer.busyFlag = 0 ;}
}
#endif

#if (USE_UART3 > 0)
void UART3_TxCplt_CallBack(UART_HandleTypeDef *huart)
{
	if(uart3Buffer.uartDList.length > 0)
	{
		if(uart3Buffer.uartDList.loadingFirstFlag == 0)	//缓冲区的头部数据存储完全
		{
			uint16_t length2;
			PopHead_DList(&(uart3Buffer.uartDList),uart3_tx,&length2);
			HAL_UART_Transmit_DMA(&huart3,uart3_tx,length2);
		}
		else { if(timerOpenFlag == 0) {timerOpenFlag = 1; xTaskResumeFromISR(UartSendTaskHandle); } }
	}	
	else{uart3Buffer.busyFlag = 0 ;}
}
#endif

#if (USE_UART4 > 0)
void UART4_TxCplt_CallBack(UART_HandleTypeDef *huart)
{
	if(uart4Buffer.uartDList.length > 0)
	{
		if(uart4Buffer.uartDList.loadingFirstFlag == 0)	//缓冲区的头部数据存储完全
		{
			uint16_t length2;
			PopHead_DList(&(uart4Buffer.uartDList),uart4_tx,&length2);
			HAL_UART_Transmit_DMA(&huart4,uart4_tx,length2);
		}
		else { if(timerOpenFlag == 0) {timerOpenFlag = 1; xTaskResumeFromISR(UartSendTaskHandle); } }
	}	
	else{uart4Buffer.busyFlag = 0 ;}
}
#endif

/**
* @brief	串口发送检测定时器，
*/
void UART_SoftTimerCallBack(void *argument)
{
while(1)
{
	uint8_t Flag=0;
/*---------------------------------------串口1发送检测--------------------------------------------*/
#if (USE_UART1 >0)	
	if(huart1.hdmatx->State == HAL_DMA_STATE_READY) //数据发送完成中断
	{
		if(uart1Buffer.uartDList.length > 0)
		{
			if( (uart1Buffer.uartDList.loadingFirstFlag == 0) &&	//缓冲区的头部数据存储完全
				(usmartSendFlag == 0) )	//usmart功能没被调用
			{
				uint16_t length1;
				PopHead_DList(&(uart1Buffer.uartDList),uart1_tx,&length1);
				HAL_UART_Transmit_DMA(&huart1,uart1_tx,length1);
			}
		}	
		else { uart1Buffer.busyFlag = 0; Flag |= 1; }			
	}
#else
	Flag |= 0x01;
#endif
	
/*---------------------------------------串口2发送检测--------------------------------------------*/
#if (USE_UART2 >0)	
	if(huart2.hdmatx->State == HAL_DMA_STATE_READY)	//数据发送完成中断
	{	
		if(uart2Buffer.uartDList.length > 0)
		{
			if(uart2Buffer.uartDList.loadingFirstFlag == 0)	//缓冲区的头部数据存储完全
			{
				uint16_t length2;
				PopHead_DList(&(uart2Buffer.uartDList),uart2_tx,&length2);
				HAL_UART_Transmit_DMA(&huart2,uart2_tx,length2);
			}
		}	
		else { uart2Buffer.busyFlag = 0;  Flag |= 2; }	
	}
#else
	Flag |= 2;
#endif
	
/*---------------------------------------串口3发送检测--------------------------------------------*/
#if (USE_UART3 >0)	
	if(huart3.hdmatx->State == HAL_DMA_STATE_READY)	//数据发送完成中断
	{
		if(uart3Buffer.uartDList.length > 0)
		{
			if(uart3Buffer.uartDList.loadingFirstFlag == 0)	//缓冲区的头部数据存储完全
			{
				uint16_t length3;
				PopHead_DList(&(uart3Buffer.uartDList),uart3_tx,&length3);
				HAL_UART_Transmit_DMA(&huart3,uart3_tx,length3);
			}
		}	
		else { uart3Buffer.busyFlag = 0;  Flag |= 4;}	
	}
#else
	Flag |= 4;
#endif
	
/*---------------------------------------串口4发送检测--------------------------------------------*/
#if (USE_UART4 >0)	
	if(huart4.hdmatx->State == HAL_DMA_STATE_READY)	//数据发送完成中断
	{			
		if(uart4Buffer.uartDList.length > 0)
		{
			if(uart4Buffer.uartDList.loadingFirstFlag == 0)	//缓冲区的头部数据存储完全
			{
				uint16_t length4;
				PopHead_DList(&(uart4Buffer.uartDList),uart4_tx,&length4);
				HAL_UART_Transmit_DMA(&huart4,uart4_tx,length4);
			}
		}	
		else { uart4Buffer.busyFlag = 0; Flag |= 8;}	
	}
#else
	Flag |= 8;
#endif
	
	if(Flag==15)
	{
		timerOpenFlag = 0;
		vTaskSuspend(UartSendTaskHandle);
	}
	osDelay(1);
}
}

/**
* @breif	将huart 串口的回调函数绑定到fuc上	*/
void Register_RX_CallBack(UART_HandleTypeDef *huart,UART_Rx_CallBack fuc)
{
#if (USE_UART1 == 2)
	if(huart == &huart1)	{UART1_Rx_CallBack = fuc; return;}
#endif
#if (USE_UART1 == 3)
	if(huart == &huart1)	
	{
		static uint8_t info[]={"Uart1 Register fail,please use UART1_AddDevice fuction\r\n"};
		FOS_UART_Transmit_DMA(&huart1,info,sizeof(info));
		return;
	}
#endif
#if (USE_UART2 > 1)
	if(huart == &huart2)	{UART2_Rx_CallBack = fuc; return;}
#endif
#if (USE_UART3 > 1)
	if(huart == &huart3)	{UART3_Rx_CallBack = fuc; return;}
#endif
#if (USE_UART4 > 1)
	if(huart == &huart4)	{UART4_Rx_CallBack = fuc; return;}
#endif
}

/*---------------------------------------串口1接收函数--------------------------------------------*/
#if (USE_UART1 == 2)
void UART1_RxHCplt_CallBack(UART_HandleTypeDef *huart)        			/*!< UART Rx Half Complete Callback        */
{
	memcpy(uart1_Rx_Buff,uart1_Rx_Hfcplt_Buff,UART1_RX_MAX*sizeof(uint8_t));
	UART1_Rx_CallBack(&huart1,uart1_Rx_Hfcplt_Buff,UART1_RX_MAX);	
}

void UART1_RxCplt_CallBack(UART_HandleTypeDef *huart)            		/*!< UART Rx Complete Callback             */
{
	memcpy(uart1_Rx_Buff+UART1_RX_MAX,uart1_Rx_Cplt_Buff,UART1_RX_MAX*sizeof(uint8_t));
	UART1_Rx_CallBack(&huart1,uart1_Rx_Cplt_Buff,UART1_RX_MAX);	
	//启用DMA接收
	HAL_UART_Receive_DMA(&huart1,uart1_Rx_Buff,UART1_RX_MAX*2);
}
#endif

/**
* @breif 串口1接收到数据后分发给绑定在串口1上的设备，依次判断是否是自己的数据包
*/
#if (USE_UART1 == 3)
void UART1_ReceiveDistributeCllback(UART_HandleTypeDef *huart,uint16_t Size)
{
	static uint8_t bugflag=1;
	if(bugflag)
	{
		bugflag=0;	//这里有个小bug，整个程序刚开始时，第一次总是接到一帧空数据，但是以后就都正常了，如果对此不介意，可以注释掉这一段
		//启用DMA接收
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,uart1_Rx_Buff,UART1_RX_MAX);
		return ;
	}
	uint16_t numi=0;
	for(;numi < uart1_device_Num; numi++)
	{
		if(Uart1_ReceivedDecode[numi](uart1_Rx_Buff,Size,uart1_device[numi]))	
		{
			//启用DMA接收
			HAL_UARTEx_ReceiveToIdle_DMA(&huart1,uart1_Rx_Buff,UART1_RX_MAX);
			return;
		}
	}

#if USE_USMART
	usmartSendFlag=1;	//因为usmart发送指令时，没添加缓冲，因此在此阶段只发送uamart中指令到上位机
	usmart_scan(uart1_Rx_Buff,Size);
	usmartSendFlag=0;
#endif

	//启用DMA接收
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,uart1_Rx_Buff,UART1_RX_MAX);

}
#endif	

/*---------------------------------------串口2接收函数--------------------------------------------*/
#if (USE_UART2 == 2)
void UART2_RxHCplt_CallBack(UART_HandleTypeDef *huart)
{
	memcpy(uart2_Rx_Hfcplt_Buff,uart2_Rx_Buff,UART2_RX_MAX*sizeof(uint8_t));
	UART2_Rx_CallBack(&huart2,uart2_Rx_Hfcplt_Buff,UART2_RX_MAX);	
}

void UART2_RxCplt_CallBack(UART_HandleTypeDef *huart)
{
	memcpy(uart2_Rx_Cplt_Buff,uart2_Rx_Buff+UART2_RX_MAX,UART2_RX_MAX*sizeof(uint8_t));
	UART2_Rx_CallBack(&huart2,uart2_Rx_Cplt_Buff,UART2_RX_MAX);	
	//启用DMA接收
	HAL_UART_Receive_DMA(&huart2,uart2_Rx_Buff,UART2_RX_MAX*2);
}
#endif

#if (USE_UART2 ==3)
void UART2_IDLE_CallBack(UART_HandleTypeDef *huart,uint16_t Size)
{
	UART2_Rx_CallBack(&huart2,uart2_Rx_Buff,Size);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2,uart2_Rx_Buff,UART2_RX_MAX);
}
#endif
	
/*---------------------------------------串口3接收函数--------------------------------------------*/
#if (USE_UART3 == 2)
void UART3_RxHCplt_CallBack(UART_HandleTypeDef *huart)
{
	memcpy(uart3_Rx_Hfcplt_Buff,uart3_Rx_Buff,UART3_RX_MAX*sizeof(uint8_t));
	UART3_Rx_CallBack(&huart3,uart3_Rx_Hfcplt_Buff,UART3_RX_MAX);	
}

void UART3_RxCplt_CallBack(UART_HandleTypeDef *huart)
{
	memcpy(uart3_Rx_Cplt_Buff,uart3_Rx_Buff+UART3_RX_MAX,UART3_RX_MAX*sizeof(uint8_t));
	UART3_Rx_CallBack(&huart3,uart3_Rx_Cplt_Buff,UART3_RX_MAX);	
	//启用DMA接收
	HAL_UART_Receive_DMA(&huart3,uart3_Rx_Buff,UART3_RX_MAX*2);
}
#endif

#if (USE_UART3 ==3)
void UART3_IDLE_CallBack(UART_HandleTypeDef *huart,uint16_t Size)
{
	UART3_Rx_CallBack(&huart3,uart3_Rx_Buff,Size);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3,uart3_Rx_Buff,UART3_RX_MAX);
}
#endif

/*---------------------------------------串口4接收函数--------------------------------------------*/
#if (USE_UART4 == 2)
void UART4_RxHCplt_CallBack(UART_HandleTypeDef *huart)
{
	memcpy(uart4_Rx_Hfcplt_Buff,uart4_Rx_Buff,UART4_RX_MAX*sizeof(uint8_t));
	UART4_Rx_CallBack(&huart4,uart4_Rx_Hfcplt_Buff);	
}

void UART4_RxCplt_CallBack(UART_HandleTypeDef *huart)
{
	memcpy(uart4_Rx_Cplt_Buff,uart4_Rx_Buff+UART4_RX_MAX,UART4_RX_MAX*sizeof(uint8_t));
	UART4_Rx_CallBack(&huart4,uart4_Rx_Cplt_Buff);	
	//启用DMA接收
	HAL_UART_Receive_DMA(&huart4,uart4_Rx_Buff,UART4_RX_MAX*2);
}
#endif

#if (USE_UART4 ==3)
void UART4_IDLE_CallBack(UART_HandleTypeDef *huart,uint16_t Size)
{
	UART4_Rx_CallBack(&huart4,uart4_Rx_Buff,Size);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart4,uart4_Rx_Buff,UART4_RX_MAX);
}
#endif

/**
* @breif 通过串口号，返回串口缓冲区句柄
*/
UartBufferHandle* GetBufferHandle(UART_HandleTypeDef *huart)
{
#if (USE_UART1 > 0)
	if(huart == &huart1)	return &uart1Buffer;
#endif
#if (USE_UART2 > 0)
	if(huart == &huart2)	return &uart2Buffer;
#endif
#if (USE_UART3 > 0)
	if(huart == &huart3)	return &uart3Buffer;
#endif
#if (USE_UART4 > 0)
	if(huart == &huart4)	return &uart4Buffer;
#endif
	return NULL;
}

/**
* @breif Fos_printf 的字符串底层解析函数
*/
void StrCombine(uint8_t*array,uint16_t *realSize,const char* str,va_list arp)
{
	unsigned char f,r,fl=0,l=3,lt,jj;		//默认留3位小数,小数可留0~7位
	unsigned char i,j,w,lp;
	unsigned long v;
	char c, d, s[16], *p;
	int res, chc, cc,ll;
	int kh,kl,pow=1;																
	double k;

	for (cc=res=0;cc!=-1;res+=cc) 		   			//解析格式化字符串，且输出
	{
		if((*realSize) >= UART1_TX_MAX)	
		{
			static uint8_t info[]={"\r\nUART1_TX_MAX too small\r\n"};
			FOS_UART_Transmit_DMA(&huart1,info,sizeof(info));
			return;
		}
		c = *str++;						   			//每一轮取一个字符
//---------------------------------//读取到'\0',结束程序
		if (c == 0) break;					
//---------------------------------//读取到非'%'符号时
		if (c != '%') {						
//			myputc(c);
			*array=c;
			array++;
			*realSize=(*realSize)+1;
			continue;
		}
//---------------------------------//读取到'%'符号时
		w=f=0;
		k=0;
		lp=0;
		c=*str++;				   					//越过'%'，读其格式
		if (c == '0') {								//%0，0填充
			f = 1; c = *str++;						//f0填充标记位置1，读取下一个字符
		} 
		else if (c == '-') {							//%-，左对齐（左边填空格）
				f = 2; c = *str++;					//f左对齐标记位置1，读取下一个字符
			}
		else if (c == '.') {						//%.3f表示留3位小数
			fl=1;c=*str++;
		}

		while (((c)>='0')&&((c)<='9')) {			//"%030"，将30转换为数字
			if(fl==1){
				lp=lp*10+c-'0';		
				c=*str++;
			}
			else{
				w=w*10+c-'0';		
				c=*str++;
			}								//将数字读完
		}
		if(fl==1) l=(lp>7)? 7:lp;
		if (c == 'l' || c == 'L') {					//%ld等长形数
			f |= 4; c = *str++;		 				//f长形标记位置1，读下一个字符
		}
		if (!c) break;								//如果此时无字符，结束输出
//---------------------------------//处理格式化标识符（d,s,c,x,o,f）
		d = c;						
		if (((c)>='a')&&((c)<='z')) d -= 0x20;		//如果是小写，划归成大写处理
		switch (d) {								//分类%*的情况

		case 'S' :					/* String */
			p = va_arg(arp,char*);					//取字符串变量
			for(j=0;p[j];j++);						//长度计算
			ll=j;
			chc = 0;
			if (!(f&2)) {							//不用左对齐，左边就要补空格%06s
				while (j++ < w) 
				{
//					myputc(c);
					*array=c;
					array++;
					*realSize=(*realSize)+1;
					chc+=1;					
				}
			}

			jj=0;	
			while (p[jj]!='\0')
			{		
//				myputc(p[jj]);	
				*array=c;
				array++;
				*realSize=(*realSize)+1;
				jj++;
			}
											
			chc+=ll;
			while (j++ < w) 						//左对齐，左边就不用空格，右边填空格%-06s
			{
//				myputc(' ');
				*array=c;
				array++;
				*realSize=(*realSize)+1;
				chc+=1;						
			}	 
			cc = chc;
			continue;

		case 'C' :
		{					/* Character */
//			myputc((char)va_arg(arp,int));
			*array=(char)va_arg(arp,int);
			array++;
			*realSize=(*realSize)+1;
			continue;
		}

		case 'F' :											//默认保留3位小数
		{													/* double(64位)/float(32位) */												
			k=va_arg(arp,double);
			if(k<0){
				l|=8;
				k*=-1;										//负数置低位1
			}
			kh=(int)k;										//整数小数分离
			pow=1;
			lt=l&7;
			while((lt-1)>=0){
				pow*=10;
				lt--;
			}
			kl=(int)(pow*(k-kh));				
			i=0;

			lt=l&7;
			while(lt--){												//存入小数部分
				if(kl){
					d=(char)(kl%10);							//按10进制取余，即取个位，化归为字符处理
					kl/=10;										//递进								
					s[i++]='0'+d;								//倒叙存入s
				}
				else s[i++]='0';
			}

			s[i++]='.';										//加入小数点
			do{												//存入整数部分
				d=(char)(kh%10);								//按10进制取余，即取个位，化归为字符处理
				kh/=10;										//递进
				s[i++]='0'+d;								//倒叙存入s
			}while(kh && i<sizeof s /sizeof s[0]);
			if (l&8)s[i++]='-';								//添加符号（注意低位在前）
			fl=0;	
			goto PRT;
		}
		case 'B' :					/* Binary */
			r = 2; break;
		case 'O' :					/* Octal */
			r = 8; break;
		case 'D' :					/* Signed decimal */
		case 'U' :					/* Unsigned decimal */
			r = 10; break;
		case 'X' :					/* Hexdecimal */
			r = 16; break;
		default:{					/* Unknown type (pass-through) */
//				myputc(c);
				*array=c;
				array++;
				*realSize=(*realSize)+1;			
				cc=1;
				continue;
			}
		}

		/* Get an argument and put it in numeral */
		/*取变量  是%ld型？		有则按long取变量			没有l但有d则按int取变量						没有l也没有d则先按unsigned int取变量在做处理	*/
		v =(f&4)?(unsigned long)va_arg(arp, long):((d=='D')?(unsigned long)(long)va_arg(arp, int):(unsigned long)va_arg(arp, unsigned int));
		if (d == 'D' && (v & 0x80000000)) {					//是%d，且符号位为1（负数）	//只有int型有增幅之分
			v = 0 - v;										//化归为正数处理
			f |= 8;											//f符号标记位置1
		}
		i = 0;
		do {												//数字解析成字符
			d=(char)(v%r);									//按进制r取余，即取个位，化归为字符处理
			v/=r;											//递进
			if(d>9)d+=(c=='x')?0x27:0x07;					//判读，不是10进制而是16进制时，d欲先加一个间隔跳转至'A'(大写X)或'a'(小写X)
			s[i++]=d+'0';									//化归为字符串处理（注意低位在前）
		} while (v && i<sizeof s /sizeof s[0]);				//（i<sizeof s /sizeof s[0]）防止含0类数字误判
		if (f & 8) s[i++] = '-';							//添加符号（注意低位在前）
PRT:		
		j=i;
		d=(f&1)?'0':' ';									//判断0填充还是左对齐
		while (!(f&2)&&j++<w)
		{
//			myputc(d);
			*array=d;
			array++;
			*realSize=(*realSize)+1;			
		}
		do 
		{
//			myputc(s[--i]);	
			*array=s[--i];
			array++;
			*realSize=(*realSize)+1;
		}while(i);

		while (j++ < w) 
		{
//			myputc(d);	
			*array=d;
			array++;
			*realSize=(*realSize)+1;			
		}
	}
}

/****************************** printf 结构化字符串组合函数的实现************************************************/
//加入以下代码,支持printf函数,而不需要选择use MicroLIB    

#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 
 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	  
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (uint8_t) ch;      
	return ch;     
//	while(huart1.gState != HAL_UART_STATE_READY){}
//	HAL_UART_Transmit_DMA(&huart1,(uint8_t *)&ch,1);//非阻塞方式打印,串口1
//	return ch;
}
#endif


/******************************错误校验函数************************************************/
void UART_Error_CallBack(UART_HandleTypeDef *huart)
{
	//#define HAL_UART_ERROR_NONE              0x00000000U   /*!< No error            */
	//#define HAL_UART_ERROR_PE                0x00000001U   /*!< Parity error        */
	//#define HAL_UART_ERROR_NE                0x00000002U   /*!< Noise error         */
	//#define HAL_UART_ERROR_FE                0x00000004U   /*!< Frame error         */
	//#define HAL_UART_ERROR_ORE               0x00000008U   /*!< Overrun error       */
	//#define HAL_UART_ERROR_DMA               0x00000010U   /*!< DMA transfer error  */
	
	UartBufferHandle *uartBuffer = GetBufferHandle(huart);
	uint32_t error_Code = uartBuffer->huart->ErrorCode;
	if(error_Code &= HAL_UART_ERROR_PE)		{ printf("huart%d,Parity error",uartBuffer->id); }
	if(error_Code &= HAL_UART_ERROR_NE)		{ printf("huart%d,Noise error",uartBuffer->id); }
	if(error_Code &= HAL_UART_ERROR_FE)		{ printf("huart%d,Frame error",uartBuffer->id); }
	if(error_Code &= HAL_UART_ERROR_ORE)	{ printf("huart%d,Overrun error",uartBuffer->id); }
	if(error_Code &= HAL_UART_ERROR_DMA)	{ printf("huart%d,DMA transfer error",uartBuffer->id); }	
}

void UART_ABORT_COMPLETE_CallBack(UART_HandleTypeDef *huart)
{
	UartBufferHandle *uartBuffer = GetBufferHandle(huart);
	printf("huart%d,Parity error",uartBuffer->id);
}

void UART_ABORT_TRANSMIT_COMPLETE_CallBack(UART_HandleTypeDef *huart)
{
	UartBufferHandle *uartBuffer = GetBufferHandle(huart);
	printf("huart%d,ABORT_TRANSMIT_COMPLETE",uartBuffer->id);
}

void UART_ABORT_RECEIVE_COMPLETE_CallBack(UART_HandleTypeDef *huart)
{
	UartBufferHandle *uartBuffer = GetBufferHandle(huart);
	printf("huart%d,ABORT_RECEIVE_COMPLETE",uartBuffer->id);
}


/******************************重写串口中断函数************************************************/
#if (USE_UART1 > 0)
/**
* @brief 不用系统生成的，重写USART1_IRQHandler中断函数
*/
void USART1_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart1);
	
	#if (USE_UART1 == 2)
	if((__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE) != RESET))//idle标志被置位
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);//清除标志位
	}
	#endif
}
#endif

#if (USE_UART2 > 0)
/**
* @brief 不用系统生成的，重写USART2_IRQHandler中断函数
*/
void USART2_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart2);
	
	#if (USE_UART2 == 2)
	if((__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE) != RESET))//idle标志被置位
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);//清除标志位
	}
	#endif
}
#endif


#if (USE_UART3 > 0)
/**
* @brief 不用系统生成的，重写USART3_IRQHandler中断函数
*/
void USART3_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart3);
	
	#if (USE_UART3 == 2)
	if((__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE) != RESET))//idle标志被置位
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);//清除标志位
	}
	#endif
}
#endif


#if (USE_UART4 > 0)
/**
* @brief 不用系统生成的，重写USART4_IRQHandler中断函数
*/
void USART4_IRQHandler(void)

	HAL_UART_IRQHandler(&huart4);
	
	#if (USE_UART4 == 2)
	if((__HAL_UART_GET_FLAG(&huart4,UART_FLAG_IDLE) != RESET))//idle标志被置位
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart4);//清除标志位
	}
	#endif
}
#endif
