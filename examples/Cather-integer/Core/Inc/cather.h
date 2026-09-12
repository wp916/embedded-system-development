#ifndef __CATHER_H
#define __CATHER_H	 

#include <stdbool.h>
#include "usart.h"

#include "YS_moto.h"

#define SCALPEL_DEBUG 1

#define gripperID 0x01
#define wireID 0x02
#define gripper_UART_Handle huart2
#define wire_UART_Handle huart3

extern YS_Moto gripperMoto;
extern YS_Moto wireMoto;

typedef void (* AtPos_Callback)(void);	//Scalpel的到位回调函数
void SC_Null(void);	//回调函数的解绑空函数
extern AtPos_Callback OpenStartPos_CallBack;	//电机到达张开起始位置后会调用一次此函数
extern AtPos_Callback StretchFinishPos_CallBack;	//电机到达抓取结束位置后会调用一次此函数

//初始化捕获器
void InitScalpel(void);

//运动前的原点校准
void Locate(void);

//手术刀开始抓取运动
void Stretch(void);

//手术刀运动到step位置
void StretchStep(int step);

//手术刀开始收纳运动
void Store(void);

//手术刀停止运动
void ScalpelStop(void);

#endif

