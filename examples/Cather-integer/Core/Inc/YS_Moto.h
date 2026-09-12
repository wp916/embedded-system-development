/*******************************************************************************
CubeMX 配置
0、参考 “FOS_Uart.h”中的CubeMX 配置要求
1、USART：使能串口DMA收发,IDLE中断（以USART2为例）
		Connectivity->USART2->Mode:Asynchronous
		Parameter Settings
				Baud Rate:115200 (根据电机实际情况设置)
				Word Length:8
				Parity:None
				Stop Bits:1
		Adcanced Parameters
				Data Direction:Receive and Transmit		
		DMA Setting->add:USART2_RX;USART2_TX
		NVIC Settings:
			USART2_global interrupt:Enabled
			DMA1 channel6 global interrrupt、DMA1 channel7 global interrupt不使能
			（不使能需要先关闭 System Core->NVIC->NVIC->Force DMA channels Interrupts）
2、将串口中断回调函数分离，不要所有中断进入一个函数后判断函数句柄，而是每个中断进入一个中断函数
		Project Manager->Advanced Settings->Register CallBack->UART:ENABLE
		
#if YS_USE_SOFT_TIMER
3、Freertos：添加YS_softTimer 软定时器
		Include parameters->Include definitions->xTimerPendFunctionCall:Enabled
		config parameters->Software definitions->TIMER_TASK_PRIORITY:小于5
#else
3、Timers：添加YS_Timer 硬件定时器
*******************************************************************************/

/**********************************************************************************
注意：
因时电机一个很麻烦的点是，大多数的操作，都会返回一个包含当前状态的数据包，
而我们是依靠稳定时间间隔，通过两次位置信息的时间差，来计算速度的，
这就要求我们严格限制对电机发送指令的时间差，随意发送含反馈数据包的指令会导致速度数据出现严重问题
**********************************************************************************/
	
#ifndef __YS_MOTO_H__
#define __YS_MOTO_H__

#include "freertos.h"
#include "usart.h"
#include "FOS_Uart.h"

#define YS_USE_SOFT_TIMER 1		//是否使用软定时器来实现定时
#define YS_DEBUG 0		//调试电机PID参数，并输出到上位机，完成后建议关闭，使用时建议单电机调试

#include "cmsis_os.h"	//定时器相关句柄定义
#if (YS_USE_SOFT_TIMER==1)
#include "timers.h"		//Freertos 的软定时器
#endif

#define YS_ASK_TIME 20		//因时电机的数据刷新周期 单位 ms
#define YS_MOTO_NUM 2		//在这个系统里，一共有几个因时的电机
#define YS_Error	10		//在这个系统里，电机到位检测时的允许误差 脉冲

/*当你的控制系统里有速度闭环时，强烈推荐设置一下这个参数，它设置了电机实际的速度计算时间间隔。
因为因时电机的状态数据包里没有速度的反馈值，所以速度更新只能靠位置差除以时间间隔得到，
但是 YS_ASK_TIME 一般会设置的较小，这就导致位置数据的波动误差除以小时间间隔带来大的速度误差，
进而导致速度环控制不稳定。因此，通过设置 velRefreshTime 测量多个 YS_ASK_TIME 的位置差除以较大时间间隔，
可以减小速度反馈值的误差波动！
此值的要求：1、YS_ASK_TIME的整数倍；2、不大于你自己设置的闭环控制周期*/
#define VEL_SAMPLE_PERIOD 100
	
typedef struct _YS_PID{
	
	float thisErr;	//
	float lastErr;	//
	float allErr;	//
	
	float returnLimit;	//输出上限
	
	float Kp;
	float Ki;
	float Kd;
	float IS;	//积分上限
} YS_PID;

//因时电机受保护的变量
//不应在YS_Moto.c以外任何地方使用
typedef struct _YS_Private YS_Private;

typedef struct _YS_Moto YS_Moto;

typedef void (*YS_CallBack)(YS_Moto* /*moto*/,void* /*fatherArgs*/);

typedef struct _YS_Moto{

	//ReadOnly
	uint8_t ID; //电机地址
	UART_HandleTypeDef *huart; //电机串口号	
	
	float nowPosi; //电机现在位置	单位mm	
	float nowVelo; //电机现在速度	单位mm/s (请仔细看下 velcRefreshTime 的说明)
	
	//ReadWrite
	YS_PID pid_V;	//速度环PID控制
//	const float veloToVolt;	//速度与电压间的转换系数 速度单位：步/s;
	uint8_t arrayDir;	//随规划路径点运动时的运动方向
		
	YS_Private* priVari;
	
	//完成指定运动后会执行一次此回调函数
	//第一个 void 为输入参数，为函数输入到位电机
	//第二个 void 为自由参数，当回调函数中需要处理某些数据时，由此参数传入
	void* fatherArgs;		//回调函数的输入参数永远是这个，可以根据需要绑定
	//电机更新数据后会调用一次这个函数
	void (*YS_RefreshData_ISR_CallBack)(YS_Moto* /*moto*/,void* /*fatherArgs*/);
	//电机到达指定位置后会调用一次这个函数
	void (*YS_P_drive_ISR_CallBack)(YS_Moto* /*moto*/,void* /*fatherArgs*/);
	//电机回到原点后会调用一次这个函数
	void (*YS_atHome_ISR_CallBack)(YS_Moto* /*moto*/,void* /*fatherArgs*/);
	//电机随一条既定轨迹运动完成后会调用一次这个函数
	void (*YS_atTrajEnd_ISR_CallBack)(YS_Moto* /*moto*/,void* /*fatherArgs*/);
} YS_Moto;

void YS_MotoInit(YS_Moto *moto, uint8_t id, UART_HandleTypeDef *huart, 
					int16_t enlarge,	//电机的位置与脉冲数换算，比如50mm要2000脉冲，此值应为40
					uint16_t zeroPosi, //电机初始位置 单位mm
					uint16_t maxPosi, //电机最大位置 单位mm
					YS_PID* pid_V);	//速度环pid参数

//立刻停止运动
void YS_Stop(YS_Moto *moto);

//直接以最短时间到达电机指定位置，他通过单次发送到位指令实现
//pos：目标位置，单位mm
uint8_t YS_FastGoto(YS_Moto *moto, float pos); 

//直接以最短时间到达电机初始位置，他通过单次发送到位指令实现
//moto:直线电机结构体
uint8_t YS_Home(YS_Moto *moto);

//电机以velc速度运动到pos位置，他通过底层闭环控制速度，到达位置。
//pos：目标位置，单位mm
//velc：目标位置，单位mm/s
uint8_t YS_Posi_drive(YS_Moto *moto, float pos, float velc);

//电机以指定速度运动
//moto:直线电机结构体
//velocity：目标速度值（单位 mm/s）
uint8_t YS_Velo_drive(YS_Moto *moto,float velocity); 

//电机跟随一条既定轨迹运动，轨迹有终止时刻 结束后会 调用 YS_atTrajEnd_ISR_CallBack 函数一次
//traj:轨迹数组首地址（单位：mm）；	length：数组长度	
//timeInterval：数据刷新时间（单位 ms）（YS_ASK_TIME的整数倍）
//dir	+1：随数组正向运动；-1：随数组反向运动（必须是这两个数）
uint8_t YS_FollowPos_Traj(YS_Moto *moto,float *traj,int length,int timeInterval, int dir);

//解绑回调函数
void YS_RefreshData_ISR_DisCallBack(YS_Moto* moto);
void YS_P_drive_ISR_DisCallBack(YS_Moto* moto);
void YS_atHome_ISR_DisCallBack(YS_Moto* moto);
void YS_atTrajEnd_DisCallBack(YS_Moto* moto);


#if YS_DEBUG
#include <string.h>
#define YS_trajlong 40

void YS_DebugTraj(YS_Moto *moto,float* traj,float a,float b);
void YS_SetPID(YS_Moto *moto,float kp,float ki,float kd,float IS);
#endif

#endif
