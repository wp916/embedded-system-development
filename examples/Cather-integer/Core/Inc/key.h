/*********************************************************************************
CubeMX 配置
1、GPIO：初始化GPIO为输入模式
		基本操作即可
		需要注意，要根据电路的实际情况配置GPIO Pull-up/Pull-down 选项
		如果硬件自带上拉或下拉，选“No pull-up and no pull-down”
		否则要根据实际电路，设定释放时的上下拉
2、完成 Fos_Uart.h 里对CubeMx的配置

Freertos：启动软定时器
		Include parameters->Include definitions->xTimerPendFunctionCall:Enabled
		config parameters->Software definitions->TIMER_TASK_PRIORITY:小于5
*********************************************************************************/

/*********************************************************************************
功能介绍：单个按键的按下、释放、单击、双击、长按操作识别
按下、释放、单击、双击操作后可以通过绑定函数，执行对应的函数操作
长按功能的绑定函数可以将长按时间作为输入参数

长按4s后执行绑定函数，简单读一下代码，即可修改长按触发时间，
也可以自己添加多个长按时间及对应的触发函数

注意：函数体执行不能占用过长时间！！！
*********************************************************************************/
#ifndef __APP_KEY_H
#define __APP_KEY_H

#include "stm32f1xx_hal.h"
#include "Fos_Uart.h"

#define KEY_NUM 2	//这个系统一共用到多少个按键

#define KEY_Debug 1	//对Key进行调试输出

//按键受保护的变量
//不应在key.c以外任何地方使用
typedef struct keyPrivate
{
	//标志位
	uint8_t statusFlag;
	//记录最近8次检测时按键电平变化
	//低位数据最新。  1：按下对应电平；0：释放对应电平
	uint8_t statusHistory;
	uint8_t timeCount;
	
	//长按4s触发函数的标志位，1：已执行过一次触发函数；0：未执行触发函数
	uint8_t t4sFlag;
}keyPrivate;

typedef struct keyHandle
{	
	/*----------------------------------ReadWrite---------------------------------------------*/
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_Pin;
	
	//枚举值:{GPIO_PIN_RESET、GPIO_PIN_SET}
	//设定什么电平代表按键按下了
	GPIO_PinState keyPushed;
		
	/******************************************************************************************
	//此按键完成指定动作时会触发一次这个函数,类似中断，可重定向函数指针
	//void* 固定为keyHandle*，显示那个按键触发了此中断；
	//注意：函数体不能占用过长时间！！！
	******************************************************************************************/
	void (*Push_CallBack) (void*);		//按下后会执行一次此函数（类似下降沿中断）
	void (*Release_CallBack) (void*);		//释放后会执行一次次函数（类似上升沿中断）
	void (*Clicked_CallBack) (void*);	//点击后会执行此函数
	void (*DoubleClicked_CallBack) (void*);		//双击后会执行此函数 
	void (*LongPush_CallBack) (void*);		//长按后会执行此函数
	void (*LongPush_4s_CallBack) (void*);		//长按4s后会执行一次此函数
	void (*LongPushRelease_CallBack) (void* ,int);		//长按释放后会执行此函数（int新参输入的是长按共经过了多少ms）


	/*----------------------------------ReadOnly---------------------------------------------*/
	//反应按键当前的状态(消抖后)
	uint8_t isPushed;	// 0：释放；1：按下
	
	keyPrivate privateValue;
		
}keyHandle;


/**
* @brief	按键初始化
* @param	keyPushed 按键被按下时，所对应的电平*/
void KeyInit(keyHandle* key, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState keyPushed);

#endif

