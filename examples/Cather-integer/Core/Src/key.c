#include "key.h"

#include "cmsis_os.h"		//定时器相关句柄定义
#include "timers.h"		//Freertos 的软定时器

//按键扫描周期，按键消抖时间 ms
#define KeyScan_Time 10	
//一次点击的稳定按下的时间不能大于此值 单位 ms
//超过此值视为开始一次长按
#define ClickMax_Time 600
//一次双击的中间稳定释放按键时间不能超过此值 单位 ms
#define mayDClickMaxInterval 400

//下面的宏定义要求实际为整数
#define ClickMax_COUNT (ClickMax_Time/KeyScan_Time)
#define mayDClickMaxInter_COUNT (mayDClickMaxInterval/KeyScan_Time)

/****************************************************************************************************************
-------------------------------------------控制逻辑详解----------------------------------------------------------

1、每次进入中断，都会通过 statusHistory 消抖处理以及判断实际按下释放状态，
并且每次消抖判断完成后的按键状态切换，都会调用一次 Push_CallBack 或 Release_CallBack 

2、t1=ClickMax_Time	;t2=mayDClickMaxInterval;


                                                    +-------------+                                     
                                                    |    释放     |                                     
                                                    +-------------+                                     
                                                           |                                            
                                                           V                                            
                                                +---------------------+                                 
                                                |      检测到按下     |                                 
                                                +---------------------+                                 
                                                           |                                            
                                                           V                                            
                                          Y /-----------------------------\ N                           
                                +-----------|       按下保持时间<=t1      |-----------+                 
                                |           \-----------------------------/           |                 
                                |                                                     |                 
                                V                                                     V                 
               Y /-----------------------------\ N                       +-------------------------+    
             +---|      释放保持时间<=t2       |---+                     |       视为一次长按      |    
             |   \-----------------------------/   |                     +-------------------------+    
             |                                     |                                  |                 
             V                                     V                                  V                 
+-------------------------+           +-------------------------+        +-------------------------+    
|      视为一次双击       |           |       视为一次单击      |        |       检测长按释放      |    
+-------------------------+           +-------------------------+        +-------------------------+    
             |                                     |                                  |                 
             +----------------->O<-----------------+                                  V                 
                                |                                    +---------------------------------+
                                |                                    |        检测长按是否超过4s       |
                                |                                    +---------------------------------+
                                |                                                     |                 
                                +------------------------->O<-------------------------+                 



****************************************************************************************************************/


//控制定时器，由YS_ASK_TIME 确定定时周期
osTimerId_t KEY_STimerHandle;
const osTimerAttr_t KEY_STimer_attributes = {
.name = "KEY_SoftTimer"};
//软定时器回调函数
void KEY_SoftTimerCallback(void *argument);

keyHandle* keyList[KEY_NUM];

//一个空函数，为“Clicked_CallBack、DoubleClicked_CallBack”指向一个地址,防止野指针错误
void KeyCallBack(void *keyHandle)	{}
void LongPushRelease_CallBack(void *keyHandle,int time)	{}

typedef enum KEY_Status{
	release=0,
	push=1,
	mayClickOrDClick=2,
	longPush=3,
	afterDClicked=4,	
} KEY_Status;

/*******************************************************************************
* Function Name  : KeyInit
* Description    : 按键初始化
* Input          : key：按键句柄
* Output         : None
* Return         : None
*******************************************************************************/
void KeyInit(keyHandle* key, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState keyPushed)
{
	static uint8_t onetimeFlag=1;
	if(onetimeFlag)//软定时器启动一次就行
	{		
		KEY_STimerHandle= osTimerNew(KEY_SoftTimerCallback, osTimerPeriodic, NULL, &KEY_STimer_attributes);
		osTimerStart(KEY_STimerHandle,KeyScan_Time);
#if KEY_Debug
//		UART_Info _info={.AH=NULL,.AHLength = 0,.dataLength = 0};
//		FOS_InitUartBuffer(&huart1,_info);
#endif
		onetimeFlag=0;
	}
	
	static int i=0;
	if (i>=KEY_NUM)	
	{
#if KEY_Debug
//		FOS_printf("too much YS_moto init!");
#endif
		return;
	}

	key->GPIOx=GPIOx;
	key->GPIO_Pin=GPIO_Pin;
	key->keyPushed=keyPushed;
	
	if(key->Push_CallBack==NULL)			key->Push_CallBack=KeyCallBack;
	if(key->Release_CallBack==NULL)			key->Release_CallBack=KeyCallBack;
	if(key->Clicked_CallBack==NULL)			key->Clicked_CallBack=KeyCallBack;
	if(key->DoubleClicked_CallBack==NULL)	key->DoubleClicked_CallBack=KeyCallBack;
	if(key->LongPush_CallBack==NULL)		key->LongPush_CallBack=KeyCallBack;
	if(key->LongPush_4s_CallBack==NULL)		key->LongPush_4s_CallBack=KeyCallBack;
	if(key->LongPushRelease_CallBack==NULL)	key->LongPushRelease_CallBack=LongPushRelease_CallBack;

	keyList[i]=key;
	i++;

}

/*******************************************************************************
* Function Name  : RealKeyStatus
* Description    : 按键消抖，判断按键当前的真实状态
* Input          : key：按键句柄
* Output         : None
* Return         : None
*******************************************************************************/
void RealKeyStatus(keyHandle* key)
{
	static GPIO_PinState KEY_PIN;
	KEY_PIN=HAL_GPIO_ReadPin(key->GPIOx,key->GPIO_Pin);
	
	if(KEY_PIN==key->keyPushed)	//按下状态
	{
		//上四次状态都为按下
		if((key->privateValue.statusHistory&0X0F)==(0X0F))	
		{	
			key->isPushed=1;
			//上第五次状态为释放（按下时可能存着抖动，所以不检测长期稳定释放状态）
			if((key->privateValue.statusHistory&0X10)==(0X00))	
			{
				key->Push_CallBack(key);
			}
		}
		key->privateValue.statusHistory=key->privateValue.statusHistory<<1;		
		key->privateValue.statusHistory++;
	}
	else	//释放状态
	{
		//上四次状态都为释放
		if((key->privateValue.statusHistory&0X0F)==(0X00))	
		{	
			key->isPushed=0;
			//上第五次状态为按下（按下时可能存着抖动，所以不检测长期稳定按下状态）
			if((key->privateValue.statusHistory&0X10)==(0X10))	
			{
				key->Release_CallBack(key);
			}
		}
		key->privateValue.statusHistory=key->privateValue.statusHistory<<1;		
	}
}

/*******************************************************************************
* Function Name  : KeyStatus
* Description    : 根据电平变化判断操作，并调用中断函数
* Input          : key：按键句柄
* Output         : None
* Return         : None
*******************************************************************************/
void KeyStatus(keyHandle* key)
{	
	RealKeyStatus(key);
	switch(key->privateValue.statusFlag)
	{
		case release:
		{
			key->privateValue.timeCount=0;
			if(key->isPushed==1)	//第一次被按下
			{
				key->privateValue.statusFlag=push;				
			}			
			break;
		}
		case push:
		{
			key->privateValue.timeCount++;
			if(key->privateValue.timeCount>ClickMax_COUNT)
			{
				key->LongPush_CallBack(key);
				key->privateValue.statusFlag=longPush;
			}
			else if(key->isPushed==0)	//按键释放，观察稳定按下时间
			{
				key->privateValue.statusFlag=mayClickOrDClick;
				key->privateValue.timeCount=0;
			}
			break;
		}
		case longPush:
		{
			key->privateValue.timeCount++;

			if(key->isPushed==0)
			{
				key->LongPushRelease_CallBack(key,key->privateValue.timeCount*KeyScan_Time);				
				key->privateValue.statusFlag=release;
				key->privateValue.timeCount=0;
				key->privateValue.t4sFlag=0;	//重新复位4s触发函数
			}

			if(key->privateValue.timeCount>(uint8_t)400	&&		//400 = 长按时间(4000ms)	/	KeyScan_Time(10ms)
				key->privateValue.t4sFlag==0)
			{
				key->privateValue.t4sFlag=1;	//长按4s触发函数只执行一次
				key->LongPush_4s_CallBack(key);
			}
			break;
		}
		case mayClickOrDClick:
		{
			key->privateValue.timeCount++;
			if(key->privateValue.timeCount>mayDClickMaxInter_COUNT)
			{
				key->Clicked_CallBack(key);				
				key->privateValue.statusFlag=release;
				key->privateValue.timeCount=0;
			}
			else if(key->isPushed==1)
			{				
				key->DoubleClicked_CallBack(key);
				key->privateValue.statusFlag=afterDClicked;
//				key->privateValue.timeCount=0;
			}
			break;
		}
		case afterDClicked:		//要在这个状态等待按键松开，否则很可能触发一次点击事件
		{
			if(key->isPushed==0)
			{
				key->privateValue.statusFlag=release;
				key->privateValue.timeCount=0;
			}
		}
		default:break;
	}
}

//软定时器回调函数,刷新按键状态
void KEY_SoftTimerCallback(void *argument)
{
	for(int i=0;i<KEY_NUM;i++)
	{
		KeyStatus(keyList[i]);
	}
}
