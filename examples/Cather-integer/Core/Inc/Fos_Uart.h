/*wangpeng HIT Y2024 M7 D3*/

/*******************************************************************************
CubeMX 配置
1、USART：使能串口DMA收发,IDLE中断（以USART2为例）
		Connectivity->USART2->Mode:Asynchronous
		Parameter Settings
				Baud Rate:115200	(根据自己工程需要设定)
				Word Length:8	(根据自己工程需要设定)
				Parity:None	(根据自己工程需要设定)
				Stop Bits:1	(根据自己工程需要设定)
				Adcanced Parameters
						Data Direction:Receive and Transmit	
		DMA Setting->add:USART2_RX;USART2_TX
		NVIC Settings:
			USART2_global interrupt:Enabled
			DMA1 channel7 global interrupt:Enabled （发送DMA中断）
		System Core->GPIO->USART
			PA3 Configuration->GPIO Pull-up/Pull-down:Pull-up（输入默认上拉，确保无信号时串口不会有误传信号）
		System Core->NVIC->Code generation
			USART1 global interrupt 的 Generate IRQ handler 选项关闭（因为代码要在此文件下重新编写）	
2、如果某个串口使用定长数据接收（USE_UARTx == 2），需要打开此串口的DMA“接收”中断
		NVIC Settings:DMA1 channel6 global interrupt 使能
   如果某个串口使用不定长数据接收（USE_UARTx == 3），需要关闭此串口的DMA“接收”中断，否则可能出现一段数据进两次中断的问题
		NVIC Settings:DMA1 channel6 global interrupt 不使能
		（不使能需要先关闭 System Core->NVIC->NVIC->Force DMA channels Interrupts）
3、将串口中断回调函数分离，不要所有中断进入一个函数后判断函数句柄，而是每个中断进入一个中断函数
		Project Manager->Advanced Settings->Register CallBack->UART:ENABLE
4、根据总程序发送需求，合理配置程序 堆区内存。
		Project Manager->Project->Linker Settings->Minimum Heap Size		
*******************************************************************************/


/*********************************************************************************
HAL库串口存在的问题：
	1、HAL库的串口通信程序是“不可重入”函数，这就导致，当外部中断和主程序中都有串口通信指令时，可能会出现随机性bug，
	此外，在Hal库中，“HAL_UART_Transmit”函数在使用任何串口发送数据时，都不允许其他串口使用此函数，否则未发送完数据就会被覆盖
	bug成因是：当主程序在用串口发送数据的过程中，发生了中断，中断中也有一次调用了串口通信函数，就会导致发送数据被覆写，甚至硬件error。
	2、虽然所有的串口都工作在一对一的控制模式下，但也有多对一的工作模式，（比如多硬件和上位机通信），
	这些通信功能再过去需要开发人员协调项目总工程分配，并不够符合程序“高内聚、低耦合原则”
	
实现功能：
	1、通过链表创建数据缓冲区，并且保证缓冲写入函数为“可重入函数”，用以解决数据冲突问题。
	也在此抛转引玉，对于其他常见通信协议在伪多线程编程模式下存在的数据冲突问题，提供一个解决思路
	
	2、上下位机的通信几乎是开发者必然会做的事情，因此，此功能包还提供了非常强大的通信调试功能，包含:
		(1)printf函数的实现，以及兼容Freertos可能会出现的多个 printf 同时调用数据缓冲的 Fos_printf 函数
		(2)带有数据缓冲的 FOS_UART_Transmit、FOS_UART_Transmit_IT、FOS_UART_Transmit_IT 函数
		(3)正点原子开发的 USMART 调试工具兼容 Freertos ，可以通过上位机发送函数及其参数，对函数进行调用，详情参考 “usmart.h”

	3、尤其对串口1，在开发中，默认设置为与上位机通信的专属串口，所有硬件的调试信息都通过串口1，因此专门为串口1添加了上位机信号分发功能，
	通过“UART1_AddDevice”函数，将可能接收上位机信息的程序绑定，可以实现对上位机数据的接收。需要注意的是，用于处理上位机信息的函数，
	其返回值应该为“UART1_ID_Match”定义的值。
	
注意：
	1、无论那个串口，发送频率最好都不要高过100Hz
	2、本功能底层完全依赖DMA发送，而UART5不含DMA接口，所有此包不支持UART5
	3、每个串口缓冲区都存在一个堆区的数据缓冲区，但一个合理的程序结构中，不应存在过多的缓冲区数据，
	此外，过多的缓冲区数据会导致控制指令的实时性变差
*********************************************************************************/


/*********************************************************************************
FOS_printf 和 FOS_ISR_printf 可以输出的格式:

0x01:格式符：
	%s,%S:字符串
	%c,%C:单个字符

	%f,%F:double,float浮点数（默认留3位小数）

	%b,%B:整型二进制
	%o,%O:整型八进制
	%d,%D:整型十进制（可有正负）
	%u,%U:整型十进制（可有正负）
	%x,%X:整型十六进制

0x02:参数（只有这三种）
	%07d：补零（数字最小显示长读7格，不足则用0在前面补齐）
	%-7d：左对齐（数字最小显示长度7格，不足则用空格在后面补齐）
	%.4f: 保留小数位数（保留4位小数，最多保留7位）

【注意】
1.程序内未对转义字符做处理，但是可以使用转义字符，可能存在未知的bug；
2.目前确知bug：使用保留小数位数功能时，保留位数超过4后输出小数值错误！
即使用%.0f，%.1f，%.2f，%.3f，%.4f时可输出正确值，使用%.5f，%.6f，%.7f时
输出值不可信！
*********************************************************************************/


/*********************************************************************************
常见 bug 汇总
1、串口通信两端没有良好共地
2、硬件无法在指定时间间隔内发送这么多数据
	以最常见 8位数据，1位停止，无校验位格式数据计算，1ms可发送数据量
	波特率：19200 : 2.13字节（uint8_t）
	波特率：115200:	12.8字节（uint8_t）
3、如果数据发送出现前几个字节正常，后面乱码情况，请检查你发送的数据数组在DMA发送中是否内存被释放掉了
*********************************************************************************/

#ifndef _FOS_UART_H_
#define _FOS_UART_H_

#include "usart.h"
#include "stdio.h"	//对printf函数的支持
#include <stdarg.h> //支持变长参数头文件
#include "usmart.h"	//正点原子的串口调试助手功能

//0:不使用串口x
//1:仅使用串口的发送缓冲功能，无接收任务；
//2:串口收发，且接收数据长度明确
//3:串口收发，但接收数据长度不定（此模式需要关闭对应串口的DMA接收中断 （是关闭DMA中断，不是关闭DMA接收功能））
#define USE_UART1 3
#define USE_UART2 3
#define USE_UART3 3
#define USE_UART4 0

//当 USE_UARTx == 2 时，UARTx_RX_MAX 代表接收数据的明确长度
//当 USE_UARTx == 3 时，UARTx_RX_MAX 代表此串口最多能一次接收的数据长度
#define UART1_RX_MAX 50
//#define UART2_RX_MAX 22
//#define UART3_RX_MAX 22
#define UART2_RX_MAX 50
#define UART3_RX_MAX 50
#define UART4_RX_MAX 22

//串口x 使用 FOS_printf 单次发送的数据最大不能超过这个值
//FOS_UART_Transmit 类函数不受此限制
#define UART1_TX_MAX 60
#define UART2_TX_MAX 10
#define UART3_TX_MAX 10
#define UART4_TX_MAX 10

//支持一些程序串口调试时才会使用的函数
//调试完成后此宏置 0 即可在编译阶段删除这些调试函数，节约系统资源
#define USE_DEBUG_TOOlS	1

#if (USE_DEBUG_TOOlS && (USE_UART1 == 3)) 
	//正点原子开发的串口调试工具，可以通过串口调用程序中的特定函数
	//根据实际需要，开启此功能
	#define USE_USMART 1
#endif

/***************************************串口的可重入发送缓冲功能************************************************/
#if USE_DEBUG_TOOlS
	/*每个串口缓冲区缓冲数据链的警告大小，超过这个大小，并不会造成任何实质性bug，但会通过串口一向上位机发送警告错误，
	一个合理的程序结构中，不应存在过多的缓冲区数据，此外，过多的缓冲区数据会导致控制指令的实时性变差*/
	#define USART_BUFFER_ALARM 2
#endif

/**
* @breif	初始化串口数据缓冲区	*/
uint8_t FOS_InitUartBuffer(void);

/**
* @breif	为 HAL_UART_Transmit 函数添加数据缓冲功能
* @notice	其底层实际上是DMA发送，因此要保证发送过程中，要发送的数据内存一直存在	*/
HAL_StatusTypeDef FOS_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout);

/**
* @breif 	兼容 HAL_UART_Transmit_IT 函数添加数据缓冲功能,有DMA了，根本没必要用IT中断发送了	*/
#define FOS_UART_Transmit_IT FOS_UART_Transmit_DMA

/**
* @breif 为 HAL_UART_Transmit_DMA 函数添加数据缓冲功能	*/
HAL_StatusTypeDef FOS_UART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size);

/**
* @breif 与printf相同用法的格式化字符串输出函数
			底层实现是非阻塞式DMA排队发送，相比printf 可以解决多个printf同时调用，数据交叉发送的bug 
* @notice 此函数的底层发送逻辑是单个单个字节发送模式，接收端不能用串口闲时中断接收			*/
void FOS_printf(const char* str,...);


/***************************************串口的 定\不定长数据接收功能************************************************/
typedef void (*UART_Rx_CallBack)(UART_HandleTypeDef *huart,const uint8_t *pdata,const uint16_t size);

/**
* @breif	将huart 串口的回调函数绑定到fuc上
* @notice	如果 USE_UART1 == 3，一定要使用 UART1_AddDevice 函数，此函数对于串口1的非定长数据接收绑定失效
			因为串口1可以服务于上位机通信，很有可能多硬件都会使用串口1资源，Register_RX_CallBack 可能会
			在你不知道的地方被其他程序挤占回调函数响应， UART1_AddDevice 则不会出现这种问题	*/
void Register_RX_CallBack(UART_HandleTypeDef *huart,UART_Rx_CallBack fuc);


/***************************************串口1与PC通信时的类总线指令分发功能*************************************/
//如果串口1是作为与上位机通讯的串口，不同硬件调试或控制时都可能接收PC的控制指令，此功能可以实现PC控制任务的分发功能
#if (USE_UART1 > 1)
	#define USART1_DEVICE_NUM 2		//串口1上绑定的接收上位机指令的设备数量（UART1_AddDevice 在总程序中调用的次数）
#endif

#if USART1_DEVICE_NUM

//专门为串口1 与上位机通信控制信号分发而定义的枚举值
typedef enum __UART1_ID_Match{
	IsMy_uart1_Data=1,
	NotMy_uart1_Data=0,
}UART1_ID_Match;

/**
* @brief  绑定在UART1上的所有设备，其处理串口上返回数据的函数的统一格式：由于确定只有串口1会出现数据拥挤现象
			因此传入参数不将 UART_HandleTypeDef *huart 作为输入参数
* @param  uint8_t* data 串口1上接受到的数据，默认传入的是 uart1_return 数组的首地址
* @param  uint16_t Size	串口1上接收到的数据大小
* @param  void*	uart1_device[i] 处理函数内部自定参数，函数调用时输入为Can1_device[i]
* @return NotMy_uart_Data:表示这个函数不处理这个数据，还需继续寻找总线上的其他处理函数处理这组数据
		  IsMy_uart_Data：这个函数处理这个数据，可以不用继续找其他函数处理了	*/
typedef uint8_t (*UART1_ReceiveDecodeFuc)(uint8_t* /* data */, uint16_t /* Size */, void* /*uart1_device[i]*/);

/**
* @brief	为UART1绑定新的设备及其对应的处理函数
* @param	device：设备结构体
* @param	fuc：此设备返回数据的解析函数	*/
uint8_t UART1_AddDevice(void* device,UART1_ReceiveDecodeFuc fuc);

#endif

#endif
