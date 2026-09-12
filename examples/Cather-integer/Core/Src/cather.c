#include "cather.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "main.h"
#include "stmflash.h"
#include <string.h>

#define _DeataT 0.07	//控制时间间隔
#define arrayLong 201	//PT数组长度

/**************************变量定义**************************/

int _countNow;

float v=2.0;	//夹爪速度
float t1=9.8;	//运动时间分区，要能被DeataT整除
float t2=14.0;	//运动时间分区，要能被DeataT整除
float a=0.2043;	//sin(11.7885) 夹爪之间张开角度
float e=4.0;	//收纳时丝的暴露长度
float k;
float b;

float GTurn[arrayLong];	
float GOrign[arrayLong];
float WOrign[arrayLong];


/*****************************掉电保存数据(需要程序实现)******************************/
//由于机械结构限制，要让运动轨迹的绝对位置添加一个修正值，防止发送机械干涉
float gripperStartPos = 43.5;
float lineStartPos = 4.50;
float gripperAddPos;

#if SCALPEL_DEBUG
//内部Flash的最后一块扇区绝对地址
uint32_t addrFlash=0x803F800;
#endif
	

/***********************函数声明*****************************/
int countOrder() { return arrayLong - _countNow; }
int countTurn() { return _countNow + 1; }	
void Function(float* LPoint,float* GPoint,float t);


/**********************************函数实现**********************************/
//初始化PT数组
void Init_PT_Count(void)
{
#if SCALPEL_DEBUG
	memcpy(&gripperStartPos,(uint32_t*)(addrFlash+0*sizeof(float)),sizeof(float));
	memcpy(&lineStartPos,(uint32_t*)(addrFlash+1*sizeof(float)),sizeof(float));	
	memcpy(&t1,(uint32_t*)(addrFlash+2*sizeof(float)),sizeof(float));	
	memcpy(&v,(uint32_t*)(addrFlash+3*sizeof(float)),sizeof(float));
	memcpy(&e,(uint32_t*)(addrFlash+4*sizeof(float)),sizeof(float));
#endif
	
	k=(((v * t2 - e) - ((v + v * a)*t1)) / (t2 - t1));
	b=((v + v * a)*t1 - ((v * t2 - e) - ((v + v * a) * t1)) * t1 / (t2 - t1));
		
	
	for (int i = 0; i < arrayLong; i++)
	{
		Function(GTurn+i,WOrign+i,i*_DeataT);
	}
	
	float Gmax=GTurn[arrayLong-1];
	for (int j = 0; j < arrayLong; j++)
	{
		GOrign[j]=Gmax - GTurn[j];
	}
}

//手术刀两驱动滑块运动的位置时间函数
void Function(float* GP,float* LP, float t)
{		
	*GP = v * t;	//PT函数表达式 夹爪
	if ((0 <= t) && (t < t1))
	{
		*LP = (v + v*a)*t;	//PT函数表达式 线
	}
	if ((t1 <= t) && (t <= t2))
	{
		*LP = k*t + b;		//PT函数表达式 线
	}
}

//用夹爪的位置换算对应时间
//(与PT中时间数组最接近时刻)
//因为坐标零点对应的就是PT中的零位置，所以不用减去坐标之间的差值
float FindTimeG(float point)
{
	_countNow =arrayLong -1 - (int)((point - gripperAddPos) / v/ _DeataT);
	return _countNow *_DeataT;
}

/***************************************全局变量****************************************/
//数据包解析
typedef union _union_float
{
	uint8_t arr[4];
	float data_float;
} union_float;

float GPoint[arrayLong]; //推杆电机 张开运动的轨迹序列
float WPoint[arrayLong]; //电热丝电机 张开运动的轨迹序列		

uint8_t TrajFlag=0;		//电机回原点时的到位标志	0B00000011 表示全部到位 

YS_PID wireMoto_pid={
	.Kp=0.6,
	.Ki=0.04,
	.Kd=0,
	.IS=1000
};

YS_PID gripperMoto_pid={
	.Kp=0.9,
	.Ki=0.2,
	.Kd=0,
	.IS=450
};

uint8_t gripperMotoFlag = 0x01;
YS_Moto gripperMoto={
	.fatherArgs = &gripperMotoFlag,};

uint8_t wireMotoFlag = 0x02;
YS_Moto wireMoto={
	.fatherArgs = &wireMotoFlag,};

osTimerId_t safe_STimerHandle;	//控制定时器
const osTimerAttr_t safe_STimer_attributes = {
.name = "safe_SoftTimer"};


/***************************************函数声明****************************************/
void RefreshTraj(void);
void safe_SoftTimerCallback(void * argument);
void Scalpel_UART_Callback(UART_HandleTypeDef *huart);
void ScalpelLocated_CallBack(void *motorx,void* fatherArgs);
void ScalpelStretched(void* moto,void* fatherArgs);
void ScalpelStored(void* moto,void* fatherArgs);
void SC_Null(void){} //回调函数的解绑空函数
	
//夹爪接收上位机控制指令的函数
uint8_t Scalpel_ReceiveCMD(uint8_t* Controlflag, uint16_t Size, void* Scalpel);
	
AtPos_Callback OpenStartPos_CallBack;	//电机到达张开起始位置后会调用一次此函数
AtPos_Callback StretchFinishPos_CallBack;	//电机到达抓取结束位置后会调用一次此函数
	

/**********************************函数实现（public）**********************************/
//初始化捕获器
void InitScalpel(void)
{
	static uint8_t onetimeFlag=0;	//初始化时仅进行一次的标志位。
	if(onetimeFlag==0)//软定时器启动一次就行
	{		
//		safe_STimerHandle= osTimerNew(safe_SoftTimerCallback, osTimerPeriodic, NULL, &safe_STimer_attributes);
//		osTimerStart(safe_STimerHandle,40);
		FOS_InitUartBuffer();
		
//		YS_MotoInit(&wireMoto,wireID,&wire_UART_Handle,
//					40,	/*enlarge*/
//					13,	/*zeroPosi*/
//					43,	/*maxPosi*/
//					&wireMoto_pid);
//		
//		YS_MotoInit(&gripperMoto,gripperID,&gripper_UART_Handle,
//					40,	/*enlarge*/
//					4,	/*zeroPosi*/
//					35,	/*maxPosi*/
//					&gripperMoto_pid);
		
		YS_MotoInit(&wireMoto,wireID,&wire_UART_Handle,
					40,	/*enlarge*/
					0,	/*zeroPosi*/
					50,	/*maxPosi*/
					&wireMoto_pid);
		
		YS_MotoInit(&gripperMoto,gripperID,&gripper_UART_Handle,
					40,	/*enlarge*/
					0,	/*zeroPosi*/
					50,	/*maxPosi*/
					&gripperMoto_pid);
						
		UART1_AddDevice(NULL,Scalpel_ReceiveCMD);

		Init_PT_Count();
		RefreshTraj();
		
		if( OpenStartPos_CallBack==NULL )		OpenStartPos_CallBack = SC_Null;
		if( StretchFinishPos_CallBack==NULL )	StretchFinishPos_CallBack = SC_Null;
		
		onetimeFlag=1;
	}
}
	
//运动前的原点校准
void Locate(void)
{
	TrajFlag=0;
	YS_FastGoto(&gripperMoto,GPoint[0]);
	YS_FastGoto(&wireMoto,WPoint[0]);
	
	gripperMoto.YS_P_drive_ISR_CallBack = (YS_CallBack)ScalpelLocated_CallBack;
	wireMoto.YS_P_drive_ISR_CallBack = (YS_CallBack)ScalpelLocated_CallBack;
}

//各电机到位检测
void ScalpelLocated_CallBack(void* motorx,void* fatherArgs)
{
	uint8_t *arg = (uint8_t *)fatherArgs;
	TrajFlag |= (*arg);	
	
	if(TrajFlag==0x03)
	{	
//		不要在这个函数里使用耗时过长的逻辑		
		YS_P_drive_ISR_DisCallBack(&wireMoto);
		YS_P_drive_ISR_DisCallBack(&gripperMoto);
		
		OpenStartPos_CallBack();	
		static uint8_t info[]={"Scalpel at start pos\r\n"};
		FOS_UART_Transmit_DMA(&huart1,info,sizeof(info));
	}
}

void StretchStep(int step)
{
	float grapperPoint;
	int count;

	grapperPoint =gripperMoto.nowPosi;
	//linePoint =PrfPosition(&LineMoto);
	//这里只是通过夹爪电机的位置分析运动位置信息，不检测两电机位置是否匹配
	FindTimeG(grapperPoint);

	count = countOrder();
	int resStep=arrayLong-step;
	count -= resStep;

	TrajFlag=0;
	YS_FollowPos_Traj(&wireMoto,WPoint + _countNow,count,(int)(_DeataT*1000),1);
	YS_FollowPos_Traj(&gripperMoto,GPoint + _countNow,count,(int)(_DeataT*1000),1);
}

//手术刀开始抓取运动
void Stretch(void)
{
	float grapperPoint;
	int count;

	grapperPoint =gripperMoto.nowPosi;
	//linePoint =PrfPosition(&LineMoto);
	//这里只是通过夹爪电机的位置分析运动位置信息，不检测两电机位置是否匹配
	FindTimeG(grapperPoint);

	count = countOrder();

	TrajFlag=0;
	wireMoto.YS_atTrajEnd_ISR_CallBack = (YS_CallBack)ScalpelStretched;
	gripperMoto.YS_atTrajEnd_ISR_CallBack = (YS_CallBack)ScalpelStretched;
	YS_FollowPos_Traj(&wireMoto,WPoint + _countNow,count,(int)(_DeataT*1000),1);
	YS_FollowPos_Traj(&gripperMoto,GPoint + _countNow,count,(int)(_DeataT*1000),1);
}

void ScalpelStretched(void* moto,void* fatherArgs)
{
	uint8_t *arg = (uint8_t *)fatherArgs;
	TrajFlag |= (*arg);	
	
	if(TrajFlag==0x03)
	{
//		不要在这个函数里使用耗时过长的逻辑
		YS_atTrajEnd_DisCallBack(&gripperMoto);
		YS_atTrajEnd_DisCallBack(&wireMoto);
		static uint8_t info[]={"Scalpel at Stretched pos\r\n"};
		FOS_UART_Transmit_DMA(&huart1,info,sizeof(info));
	}
}

//手术刀开始收纳运动
void Store(void)
{
	float grapperPoint;
	int count;

	grapperPoint =gripperMoto.nowPosi;
	//linePoint =PrfPosition(&LineMoto);
	//这里只是通过夹爪电机的位置分析运动位置信息，不检测两电机位置是否匹配
	FindTimeG(grapperPoint);
	count = countTurn();

	TrajFlag=0;
	wireMoto.YS_atTrajEnd_ISR_CallBack = (YS_CallBack)ScalpelStored;
	gripperMoto.YS_atTrajEnd_ISR_CallBack = (YS_CallBack)ScalpelStored;
	YS_FollowPos_Traj(&wireMoto,WPoint + _countNow,count,(int)(_DeataT*1000),-1);
	YS_FollowPos_Traj(&gripperMoto,GPoint + _countNow,count,(int)(_DeataT*1000),-1);
}

void ScalpelStored(void* moto,void* fatherArgs)
{
	uint8_t *arg = (uint8_t *)fatherArgs;
	TrajFlag |= (*arg);	
	
	if(TrajFlag==0x03)
	{
//		不要在这个函数里使用耗时过长的逻辑
		YS_atTrajEnd_DisCallBack(&gripperMoto);
		YS_atTrajEnd_DisCallBack(&wireMoto);
		static uint8_t info[]={"Scalpel at Stored pos\r\n"};
		FOS_UART_Transmit_DMA(&huart1,info,sizeof(info));
	}
}

//手术刀停止运动
void ScalpelStop(void)
{
	YS_Stop(&wireMoto);
	YS_Stop(&gripperMoto);
}

//!!!!界限参数需要调整
void safe_SoftTimerCallback(void * argument)
{
//	if(gripperMoto.nowPosi-wireMoto.nowPosi>10.0)
//	{
//		ScalpelStop();
//		static uint8_t info[]={"sliders distance too small\r\n"};
//		FOS_UART_Transmit_DMA(&huart1,info,sizeof(info));
//	}
//	if((gripperMoto.nowPosi<10.0) || (gripperMoto.nowPosi>40.0))
//	{
//		ScalpelStop();
//		static uint8_t info[]={"gripperMoto out of range\r\n"};
//		FOS_UART_Transmit_DMA(&huart1,info,sizeof(info));
//	}
//	if((wireMoto.nowPosi<10.0) || (wireMoto.nowPosi>40.0))
//	{
//		ScalpelStop();
//		static uint8_t info[]={"wireMoto out of range\r\n"};
//		FOS_UART_Transmit_DMA(&huart1,info,sizeof(info));
//	}	
}

//夹爪接收上位机控制指令的函数
uint8_t Scalpel_ReceiveCMD(uint8_t* Controlflag, uint16_t Size, void* Scalpel)
{
	if(Controlflag[0] != 0x09)	
	{
		return NotMy_uart1_Data;
	}
	switch(Controlflag[1])
	{
		case 0x00 :		//展开捕获器
			Stretch();
		break;
		
		case 0x01 :		//抓取
			Store();
		break;
		
		case 0x02 :		//停止运动
			ScalpelStop();
		break;
		
		case 0x03 :		//手术刀双电机回轨迹起点	
			Locate();
		break;
		
		case 0x04 :		//单电机停止运动
			switch(Controlflag[2])
			{
				case 0x01:
					YS_Stop(&gripperMoto);
				break;	
				case 0x02:
					YS_Stop(&wireMoto);
				break;	
			}					
		break;
		
		case 0x05 :
		{
			if(gripperMoto.nowPosi<30.0)
			{
				YS_FastGoto(&gripperMoto,31.0);
				static uint8_t info[]={"gripper moto in denger place, remove first\r\n"};
				FOS_UART_Transmit_DMA(&huart1,info,sizeof(info));
			}
			else 
			{	YS_FastGoto(&wireMoto,1);}
		}
		break;
		
		case 0x06 :
		{
			if(wireMoto.nowPosi<30.0)
			{
				YS_FastGoto(&wireMoto,31.0);
				static uint8_t info[]={"wire moto in denger place, remove first\r\n"};
				FOS_UART_Transmit_DMA(&huart1,info,sizeof(info));
			}
			else 
			{	YS_FastGoto(&gripperMoto,1);}
		}
		break;
				
		case 0x08 :		//通过上位机控制电机单独运动
		{
			union_float velc_union;
			velc_union.arr[0] = Controlflag[3];
			velc_union.arr[1] = Controlflag[4];
			velc_union.arr[2] = Controlflag[5];
			velc_union.arr[3] = Controlflag[6];
			float velc =velc_union.data_float;
			switch(Controlflag[2])
			{
				case 0x01:
					YS_Velo_drive(&gripperMoto,velc);
				break;	
				case 0x02:
					YS_Velo_drive(&wireMoto,velc);
				break;	
			}						
		}
		break;
		
#if SCALPEL_DEBUG		
		case 0x09 :		//向FLASH中写入调试数据
		{
			//先写入数据
			memcpy(&gripperStartPos,Controlflag+2+0*sizeof(float),sizeof(float));
			memcpy(&lineStartPos,Controlflag+2+1*sizeof(float),sizeof(float));			
			memcpy(&t1,Controlflag+2+2*sizeof(float),sizeof(float));
			memcpy(&v,Controlflag+2+3*sizeof(float),sizeof(float));
			memcpy(&e,Controlflag+2+4*sizeof(float),sizeof(float));			
			STMFLASH_Write(addrFlash,(uint16_t*)(Controlflag+2),5*sizeof(float)/sizeof(uint16_t));
//			float data[6]={1,2,3,4,5,6};
//			STMFLASH_Write(addrFlash,(uint16_t *)data,6*sizeof(float)/sizeof(uint16_t));
			//再读出数据，以验证成功写入
			memcpy(&gripperStartPos,(uint32_t*)(addrFlash+0*sizeof(float)),sizeof(float));
			memcpy(&lineStartPos,(uint32_t*)(addrFlash+1*sizeof(float)),sizeof(float));			
			memcpy(&t1,(uint32_t*)(addrFlash+2*sizeof(float)),sizeof(float));	
			memcpy(&v,(uint32_t*)(addrFlash+3*sizeof(float)),sizeof(float));
			memcpy(&e,(uint32_t*)(addrFlash+4*sizeof(float)),sizeof(float));			
			FOS_printf("flash:%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
						(double)gripperStartPos,(double)lineStartPos,(double)t1,(double)v,(double)e);
			
			Init_PT_Count();
			RefreshTraj();
		}
		break;

		case 0x10 :		//向上位机发送修正数据
		{
			FOS_printf("flash:%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
						(double)gripperStartPos,(double)lineStartPos,(double)t1,(double)v,(double)e);
		}
		break;
#endif

	}
	return IsMy_uart1_Data;
}

/**********************************函数实现（private）**********************************/
void RefreshTraj(void)
{
#if SCALPEL_DEBUG
		memcpy(&gripperStartPos,(uint32_t*)(addrFlash+0*sizeof(float)),sizeof(float));
		memcpy(&lineStartPos,(uint32_t*)(addrFlash+1*sizeof(float)),sizeof(float));	
#endif
	
	gripperAddPos=gripperStartPos-GOrign[0];
	for (int i = 0; i < arrayLong; i++)	
	{ 
		GPoint[i] = gripperAddPos+GOrign[i];
		WPoint[i] = lineStartPos+WOrign[i];
	}
}
