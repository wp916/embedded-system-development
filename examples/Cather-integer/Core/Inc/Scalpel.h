/*******************************************************************************
CubeMX 配置
0、参考 YS_moto.h 里的配置方法
1、USART：使能串口接收终端（以USART1为例）
		Connectivity->USART1->Mode:Asynchronous
		Parameter Settings
				Baud Rate:115200
				Word Length:8
				Parity:None
				Stop Bits:1
		Adcanced Parameters
				Data Direction:Receive and Transmit
		NVIC Settings->Enabled:选择	
2、将串口中断回调函数分离，不要所有中断进入一个函数后判断函数句柄，而是每个中断进入一个中断函数
		Project Manager->Advanced Settings->Register CallBack->UART:ENABLE
#if YS_USE_SOFT_TIMER
3、Freertos：添加YS_softTimer 软定时器
		Include parameters->Include definitions->xTimerPendFunctionCall:Enabled
		config parameters->Software definitions->TIMER_TASK_PRIORITY:小于5
#else
3、Timers：添加YS_Timer 硬件定时器
*******************************************************************************/

/*******************************************************************************
上位机指令格式：

*******************************************************************************/

#ifndef __SCALPEL_H__
#define __SCALPEL_H__

#include "freertos.h"
//#include <stdio.h>

#include "cmsis_os.h"		//定时器相关句柄定义
#include "timers.h"		//Freertos 的软定时器
#include "usart.h"

#include "YS_moto.h"

#define SCALPEL_DEBUG 1

#define TIME_INTERVAL 500			//位置序列的时间增量 ms

#define pushID 0x04
#define wireID 0x02
#define push_UART_Handle huart3
#define wire_UART_Handle huart2

extern YS_Moto gripperMoto;
extern YS_Moto wireMoto;

extern uint8_t Scalpel_Dir;		//在手动模式下的运动方向 0：后退；1：前进

typedef void (* AtPos_Callback)(void);	//Scalpel的到位回调函数

void SC_Null(void);	//回调函数的解绑空函数
extern AtPos_Callback OpenStartPos_CallBack;	//电机到达张开起始位置后会调用一次此函数
extern AtPos_Callback StretchFinishPos_CallBack;	//电机到达抓取结束位置后会调用一次此函数

//初始化夹爪模组
void ScalpelInit(void);

//由于机构问题，电机无法自动回到起始位置，因此这个函数只是把丝电机和推杆电机归位了
//把旋转电机的锁死关闭，还需要手动调整旋转电机的位置
void ScalpelGoStartPos(void);

//停止电机
void ScalpelStop(void);

//让电机从当前位置，继续正向运动
uint8_t ScalpelFront(void);

//让电机从当前位置，继续反向运动
uint8_t ScalpelBack(void);

#endif
