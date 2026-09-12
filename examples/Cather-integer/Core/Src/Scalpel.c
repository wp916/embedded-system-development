#include "Scalpel.h"
#include "Fos_Uart.h"
#include "stmflash.h"
#include <math.h>

#include <string.h>

/***************************************函数声明****************************************/
void RefreshTraj(void);
//uint8_t CheckStatus(double linePos, double rotatePos);
//return -1:失败
float FindTimeL(double point);
//return 1:成功，0:失败
uint8_t RefreshArray(double time);
uint16_t ScountOrder(void);
uint16_t OcountOrder(void);
uint8_t SC_CheckBit(uint8_t *array,int num);

void Scalpel_UART_Callback(UART_HandleTypeDef *huart);
void ScalpelLocated_CallBack(void *motorx,void* fatherArgs);
void Stretch(void);
void ScalpelStretched(void* moto,void* fatherArgs);
void Open(void);
void ScalpelOpend(void* moto,void* fatherArgs);
void SC_Null(void){} //回调函数的解绑空函数
	
//夹爪接收上位机控制指令的函数
uint8_t Scalpel_ReceiveCMD(uint8_t* Controlflag, uint16_t Size, void* Scalpel);
	
AtPos_Callback OpenStartPos_CallBack;	//电机到达张开起始位置后会调用一次此函数
AtPos_Callback MiddlePos_CallBack;	//电机到达张开结束\抓取开始位置后会调用一次此函数
AtPos_Callback StretchFinishPos_CallBack;	//电机到达抓取结束位置后会调用一次此函数
	
	
/***************************************全局变量****************************************/
//数据包解析
typedef union _union_float
{
	uint8_t arr[4];
	float data_float;
} union_float;

const float _DeataT = 0.5;	//时间间隔 单位s
const int _Olong = 46;			//展开运动数组的长度
int _OcountNow = 0;		//现在运动到展开运动数组的位置
float _OPOrign[_Olong]; //推杆电机 张开运动的轨迹序列
float _OLOrign[_Olong]; //电热丝电机 张开运动的轨迹序列	
float _OROrign[_Olong]; //旋转电机 张开运动的轨迹序列	

const int _Slong = 46;			//抓取运动数组的长度
int _ScountNow = 0;		//现在运动到抓取运动数组的位置
float _SPOrign[_Slong]; //推杆电机 张开运动的轨迹序列
float _SLOrign[_Slong]; //电热丝电机 张开运动的轨迹序列	
float _SROrign[_Slong]; //旋转电机 张开运动的轨迹序列	

uint8_t Scalpel_Dir=0;		//在手动模式下的运动方向 0：后退；1：前进
uint8_t status = 5;		//0：定位完成；1：展开运动中；2：展开完成，可切换两种状态;3:合拢运动；4:合拢运动完成 5:每个电机独立运动，需要重新定位
uint8_t TrajFlag=0;		//电机回原点时的到位标志	0B00000111 表示全部到位 

YS_PID wireMoto_pid={
	.Kp=0.6,
	.Ki=0.04,
	.Kd=0,
	.IS=1000
};

YS_PID pushMoto_pid={
	.Kp=0.5,
	.Ki=0.05,
	.Kd=0,
	.IS=450
};

uint8_t pushMotoFlag = 0x01;
YS_Moto pushMoto={
	.fatherArgs = &pushMotoFlag,};

uint8_t wireMotoFlag = 0x02;
YS_Moto wireMoto={
	.fatherArgs = &wireMotoFlag,};


/*****************************掉电保存数据(需要程序实现)******************************/
//由于机械结构限制，要让运动轨迹的绝对位置添加一个修正值，防止发送机械干涉
float pushAxis = -1.4;
float lineAxis = 9.7;
float pushCorrect = 8.7;	//
float lineCorrect =39.64;		//

#if SCALPEL_DEBUG
//内部Flash的最后一块扇区绝对地址
uint32_t addrFlash=0x803F800;
#endif
	
/**********************************函数实现（public）**********************************/
void ScalpelInit(void)
{
	YS_MotoInit(&wireMoto,wireID,&wire_UART_Handle,
				40,	/*enlarge*/
				0,	/*zeroPosi*/
				50,	/*maxPosi*/
				&wireMoto_pid);
	
	YS_MotoInit(&pushMoto,pushID,&push_UART_Handle,
				200,	/*enlarge*/
				0,	/*zeroPosi*/
				10,	/*maxPosi*/
				&pushMoto_pid);

	UART1_AddDevice(NULL,Scalpel_ReceiveCMD);
	
#if SCALPEL_DEBUG
	memcpy(&pushAxis,(uint32_t*)(addrFlash+0*sizeof(float)),sizeof(float));
	memcpy(&lineAxis,(uint32_t*)(addrFlash+1*sizeof(float)),sizeof(float));	
	memcpy(&rotateAxis,(uint32_t*)(addrFlash+2*sizeof(float)),sizeof(float));	
	memcpy(&pushCorrect,(uint32_t*)(addrFlash+3*sizeof(float)),sizeof(float));
	memcpy(&lineCorrect,(uint32_t*)(addrFlash+4*sizeof(float)),sizeof(float));
	memcpy(&rotateCorrect,(uint32_t*)(addrFlash+5*sizeof(float)),sizeof(float));	
#endif
	
	_OPOrign[0] = 8.7;   _OLOrign[0] = 2.78;
	_OPOrign[1] = 8.1;   _OLOrign[1] = 3.59;
	_OPOrign[2] = 7.59;   _OLOrign[2] = 4.43;
	_OPOrign[3] = 7.15;   _OLOrign[3] = 5.3;
	_OPOrign[4] = 6.76;   _OLOrign[4] = 6.18;
	_OPOrign[5] = 6.41;   _OLOrign[5] = 7.06;
	_OPOrign[6] = 6.11;   _OLOrign[6] = 7.95;
	_OPOrign[7] = 5.84;   _OLOrign[7] = 8.84;
	_OPOrign[8] = 5.59;   _OLOrign[8] = 9.74;
	_OPOrign[9] = 5.37;   _OLOrign[9] = 10.63;
	_OPOrign[10] = 5.16;   _OLOrign[10] = 11.52;
	_OPOrign[11] = 4.98;   _OLOrign[11] = 12.41;
	_OPOrign[12] = 4.81;   _OLOrign[12] = 13.3;
	_OPOrign[13] = 4.65;   _OLOrign[13] = 14.18;
	_OPOrign[14] = 4.5;    _OLOrign[14] = 15.07;
	_OPOrign[15] = 4.37;   _OLOrign[15] = 15.95;
	_OPOrign[16] = 4.24;   _OLOrign[16] = 16.83;
	_OPOrign[17] = 4.13;   _OLOrign[17] = 17.7;
	_OPOrign[18] = 4.02;   _OLOrign[18] = 18.57;
	_OPOrign[19] = 3.92;   _OLOrign[19] = 19.44;
	_OPOrign[20] = 3.82;   _OLOrign[20] = 20.3;
	_OPOrign[21] = 3.73;   _OLOrign[21] = 21.16;
	_OPOrign[22] = 3.65;   _OLOrign[22] = 22.01;
	_OPOrign[23] = 3.56;   _OLOrign[23] = 22.86;
	_OPOrign[24] = 3.49;   _OLOrign[24] = 23.7;
	_OPOrign[25] = 3.42;   _OLOrign[25] = 24.53;
	_OPOrign[26] = 3.35;   _OLOrign[26] = 25.36;
	_OPOrign[27] = 3.28;   _OLOrign[27] = 26.19;
	_OPOrign[28] = 3.22;   _OLOrign[28] = 27;
	_OPOrign[29] = 3.16;   _OLOrign[29] = 27.81;
	_OPOrign[30] = 3.11;   _OLOrign[30] = 28.62;
	_OPOrign[31] = 3.05;   _OLOrign[31] = 29.41;
	_OPOrign[32] = 3;  	   _OLOrign[32] = 30.2;
	_OPOrign[33] = 2.95;   _OLOrign[33] = 30.98;
	_OPOrign[34] = 2.9;    _OLOrign[34] = 31.75;
	_OPOrign[35] = 2.86;   _OLOrign[35] = 32.51;
	_OPOrign[36] = 2.81;   _OLOrign[36] = 33.27;
	_OPOrign[37] = 2.77;   _OLOrign[37] = 34.01;
	_OPOrign[38] = 2.73;   _OLOrign[38] = 34.75;
	_OPOrign[39] = 2.69;   _OLOrign[39] = 35.48;
	_OPOrign[40] = 2.66;   _OLOrign[40] = 36.2;
	_OPOrign[41] = 2.62;   _OLOrign[41] = 36.9;
	_OPOrign[42] = 2.58;   _OLOrign[42] = 37.6;
	_OPOrign[43] = 2.55;   _OLOrign[43] = 38.29;
	_OPOrign[44] = 2.52;   _OLOrign[44] = 38.97;
	_OPOrign[45] = 2.49;   _OLOrign[45] = 39.64;

	RefreshTraj();
	
	if( StretchFinishPos_CallBack==NULL )	StretchFinishPos_CallBack = SC_Null;
	if( OpenStartPos_CallBack==NULL )		OpenStartPos_CallBack = SC_Null;
}

void ScalpelStop(void)
{
	YS_Stop(&pushMoto);
	YS_Stop(&wireMoto);
}

void ScalpelGoStartPos(void)
{
	TrajFlag=0;
	YS_FastGoto(&pushMoto,_OPOrign[0]);
	YS_FastGoto(&wireMoto,_OLOrign[0]);
	
	pushMoto.YS_P_drive_ISR_CallBack = (YS_CallBack)ScalpelLocated_CallBack;
	wireMoto.YS_P_drive_ISR_CallBack = (YS_CallBack)ScalpelLocated_CallBack;
}

//各电机到位检测
void ScalpelLocated_CallBack(void* motorx,void* fatherArgs)
{
	uint8_t *arg = (uint8_t *)fatherArgs;
	TrajFlag |= (*arg);	
	
	if(TrajFlag==0x07)
	{	
//		不要在这个函数里使用耗时过长的逻辑
		status=0;			//located
				
		YS_P_drive_ISR_DisCallBack(&wireMoto);
		YS_P_drive_ISR_DisCallBack(&pushMoto);
		
		OpenStartPos_CallBack();	
		static uint8_t info[]={"Scalpel at start pos\r\n"};
		FOS_UART_Transmit_DMA(&huart1,info,sizeof(info));
	}
}

uint8_t ScalpelFront(void)
{
	//将回调函数绑定到继续抓取函数上
	MiddlePos_CallBack =Stretch;
	Open();
	return 1;
}

void Open(void)
{
	float wirePoint = wireMoto.nowPosi;
//	float rotatePoint = rotateMoto.point;
	
	//don't use fuction checkStatus,becaues moto pos error may cause estimate status error
//	if (CheckStatus(wirePoint, rotatePoint) == 1) 
		if(status != 0 )
	{
		static uint8_t info[]={"Scalpel can not open\r\n"};
		FOS_UART_Transmit_DMA(&huart1,info,sizeof(info));
		return ;
	}
	
	//这里只通过丝电机的位置分析运动位置信息，不检测两电机位置是否匹配
	float time = FindTimeL(wirePoint);
	RefreshArray(time);
	uint16_t count = OcountOrder();
	
	TrajFlag=0;
	status=1;	//opening
	wireMoto.YS_atTrajEnd_ISR_CallBack = (YS_CallBack)ScalpelOpend;
	pushMoto.YS_atTrajEnd_ISR_CallBack = (YS_CallBack)ScalpelOpend;
	YS_FollowPos_Traj(&wireMoto,_OLOrign + _OcountNow,count,(int)(_DeataT*1000),1);
	YS_FollowPos_Traj(&pushMoto,_OPOrign + _OcountNow,count,(int)(_DeataT*1000),1);
}

void ScalpelOpend(void* moto,void* fatherArgs)
{
	uint8_t *arg = (uint8_t *)fatherArgs;
	TrajFlag |= (*arg);	
	
	if(TrajFlag==0x07)
	{
//		不要在这个函数里使用耗时过长的逻辑
		status=2;		//in switch mode : opened and can stretching
		YS_atTrajEnd_DisCallBack(&pushMoto);
		YS_atTrajEnd_DisCallBack(&wireMoto);
		MiddlePos_CallBack();
		static uint8_t info[]={"Scalpel at middle pos\r\n"};
		FOS_UART_Transmit_DMA(&huart1,info,sizeof(info));
	}
}

void Stretch(void)
{
	float wirePoint = wireMoto.nowPosi;
//	float rotatePoint = rotateMoto.point;
	
	//don't use fuction checkStatus,becaues moto pos error may cause estimate status error
//	if (CheckStatus(wirePoint, rotatePoint) == 1) 
		if(status != 2 )
	{
		static uint8_t info[]={"Scalpel can not Stretch\r\n"};
		FOS_UART_Transmit_DMA(&huart1,info,sizeof(info));
		return ;
	}
	
	//这里只通过丝电机的位置分析运动位置信息，不检测两电机位置是否匹配
	float time = FindTimeL(wirePoint);
	RefreshArray(time);
	uint16_t count = ScountOrder();
	
	TrajFlag=0;
	status=3;	//stretching
	wireMoto.YS_atTrajEnd_ISR_CallBack = (YS_CallBack)ScalpelStretched;
//	pushMoto.YS_atTrajEnd_ISR_CallBack = (YS_CallBack)ScalpelStretched;
	YS_FollowPos_Traj(&wireMoto,_SLOrign + _ScountNow,count,(int)(_DeataT*1000),1);
//	YS_FollowPos_Traj(&pushMoto,_SPOrign + _ScountNow,count,(int)(_DeataT*1000),1);	//从联动结构上看，这个电机不需要运动
}

void ScalpelStretched(void* moto,void* fatherArgs)
{
	uint8_t *arg = (uint8_t *)fatherArgs;
	TrajFlag |= (*arg);	
	
	if(TrajFlag==0x06)
	{
//		不要在这个函数里使用耗时过长的逻辑
		status=4;	//stretched
//		YS_atTrajEnd_DisCallBack(&pushMoto);
		YS_atTrajEnd_DisCallBack(&wireMoto);
		
		StretchFinishPos_CallBack();
		static uint8_t info[]={"Scalpel Stretched\r\n"};
		FOS_UART_Transmit_DMA(&huart1,info,sizeof(info));
	}
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
			MiddlePos_CallBack=SC_Null;
			Open();
		break;
		
		case 0x01 :		//抓取
			Stretch();
		break;
		
		case 0x02 :		//停止运动
			ScalpelStop();
		break;
		
		case 0x03 :		//手术刀双电机回轨迹起点	
			ScalpelGoStartPos();
		break;
		
		case 0x04 :		//单电机停止运动
			status =5;	//表示现在进入电机单独运动模式，此时不允许任何联动操作
			switch(Controlflag[2])
			{
				case 0x01:
					YS_Stop(&pushMoto);
				break;	
				case 0x02:
					YS_Stop(&wireMoto);
				break;	
			}					
		break;
		
//		case 0x05 :		//手术刀抓取
//			Stretch();
//		break;		
				
		case 0x06 :		//将捕获器运动状态手动改为定位完成
			status=0;
		break;
		
		case 0x07 :		//将捕获器运动状态手动改为到达展开完成位置
			status=2;
		break;
		
		case 0x08 :		//通过上位机控制电机单独运动
		{
			union_float velc_union;
			velc_union.arr[0] = Controlflag[3];
			velc_union.arr[1] = Controlflag[4];
			velc_union.arr[2] = Controlflag[5];
			velc_union.arr[3] = Controlflag[6];
			float velc =velc_union.data_float;
			status =5;	//表示现在进入电机单独运动模式，此时不允许任何联动操作
			switch(Controlflag[2])
			{
				case 0x01:
					//YS_Posi_drive(&pushMoto,pushMoto.nowPosi+velc*0.3,velc);
					YS_Velo_drive(&pushMoto,velc);
				break;	
				case 0x02:
					//YS_Posi_drive(&wireMoto,wireMoto.nowPosi+velc*0.3,velc);
					YS_Velo_drive(&wireMoto,velc);
				break;	
			}						
		}
		break;
		
#if SCALPEL_DEBUG		
		case 0x09 :		//向FLASH中写入调试数据
		{
			//先写入数据
			memcpy(&pushAxis,Controlflag+2+0*sizeof(float),sizeof(float));
			memcpy(&lineAxis,Controlflag+2+1*sizeof(float),sizeof(float));			
			memcpy(&rotateAxis,Controlflag+2+2*sizeof(float),sizeof(float));
			memcpy(&pushCorrect,Controlflag+2+3*sizeof(float),sizeof(float));
			memcpy(&lineCorrect,Controlflag+2+4*sizeof(float),sizeof(float));			
			memcpy(&rotateCorrect,Controlflag+2+5*sizeof(float),sizeof(float));
			STMFLASH_Write(addrFlash,(uint16_t*)(Controlflag+2),6*sizeof(float)/sizeof(uint16_t));
//			float data[6]={1,2,3,4,5,6};
//			STMFLASH_Write(addrFlash,(uint16_t *)data,6*sizeof(float)/sizeof(uint16_t));
			//再读出数据，以验证成功写入
			memcpy(&pushAxis,(uint32_t*)(addrFlash+0*sizeof(float)),sizeof(float));
			memcpy(&lineAxis,(uint32_t*)(addrFlash+1*sizeof(float)),sizeof(float));			
			memcpy(&rotateAxis,(uint32_t*)(addrFlash+2*sizeof(float)),sizeof(float));	
			memcpy(&pushCorrect,(uint32_t*)(addrFlash+3*sizeof(float)),sizeof(float));
			memcpy(&lineCorrect,(uint32_t*)(addrFlash+4*sizeof(float)),sizeof(float));			
			memcpy(&rotateCorrect,(uint32_t*)(addrFlash+5*sizeof(float)),sizeof(float));	
			FOS_printf("flash:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
						(double)pushAxis,(double)lineAxis,(double)rotateAxis,
							(double)pushCorrect,(double)lineCorrect,(double)rotateCorrect);
			//RefreshTraj();
		}
		break;

		case 0x10 :		//向上位机发送修正数据
		{
			FOS_printf("flash:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
						(double)pushAxis,(double)lineAxis,(double)rotateAxis,
							(double)pushCorrect,(double)lineCorrect,(double)rotateCorrect);
		}
		break;
#endif

	}
	return IsMy_uart1_Data;
}

/**********************************函数实现（private）**********************************/
void RefreshTraj(void)
{
	float pushDelatOpen=(pushCorrect-_OPOrign[0])/(_Olong-1);
	float lineDelatOpen=(_OLOrign[_Olong-1]-lineCorrect)/(_Olong-1);
	for (int i = 0; i < _Olong; i++)	
	{ 
		_OPOrign[i] += pushAxis+pushDelatOpen*(_Olong-1-i);
		_OLOrign[i] += lineAxis-lineDelatOpen*i;
		_OROrign[i] = rotateAxis; 
	}

	float lineDelatClose=(_SLOrign[0]-lineCorrect)/(_Slong-1);
	for (int i = 0; i < _Slong; i++)
	{
		_SROrign[i] = i + rotateAxis;
		_SLOrign[i] += lineAxis-lineDelatClose*(_Slong-1-i);
		_SPOrign[i] = _OPOrign[_Olong - 1]+ pushAxis;
	}
}

//uint8_t CheckStatus(double linePos, double rotatePos)
//{
//	if (rotatePos < _SROrign[1])
//	{
//		if (linePos < _SLOrign[1])
//		{
//			status = 0;
//			return status;
//		}
//		status = 1;
//		return status;
//	}
//	status = 2;
//	return status;
//}

//return -1:失败
float FindTimeL(double point)
{
	if(status == 0) return 0;

	if (status == 1)		//true:张开运动
	{
		if (point < _OLOrign[1]) { return 0; }
		if (point > _OLOrign[_Olong - 2]) { return (_Olong - 1) * _DeataT; }
		for (int i = 1; i < _Olong; i++)
		{
			if ((point - _OLOrign[i - 1]) * (point - _OLOrign[i]) <= 0)
			{
				return i * _DeataT;
			}
		}
	}
	if (status == 2)	//切换状态位置
	{
		//这里有些问题，time在close运动中为_OTOrign[_Olong-1],在stretch中为0
		//先返回_OTOrign[_Olong-1]，根据其他情况判断选t
		return (_Olong - 1) * _DeataT;
	}
	if (status == 3)		//true:合拢运动
	{
		if (point > _SLOrign[1]) { return 0; }
		if (point < _SLOrign[_Slong-2]) { return (_Slong-1) * _DeataT; }
		for (int i = 1; i < _Slong; i++)
		{
			if ((point - _SLOrign[i - 1]) * (point - _SLOrign[i]) <= 0)
			{
				return i * _DeataT;
			}
		}
	}
	return -1;	//-1是个错误返回值
}

//return 1:成功，0:失败
uint8_t RefreshArray(double time)
{
	if ((status == 0) || (status == 1)) //张开状态中
	{
		_OcountNow = time / _DeataT;
		return 1;
	}
	if (status == 2)
	{
		_OcountNow = _Olong - 1;
		_ScountNow = 0;
		return 1;
	}
	if (status == 3)	//合拢状态中
	{
		_ScountNow = time / _DeataT;
		return 1;
	}
	return 0;
}

uint16_t OcountOrder(void) { return _Olong - _OcountNow; }
uint16_t ScountOrder(void) { return _Slong - _ScountNow; }




//电机校验和函数
//array：需要校验和数组
//num：数组大小
uint8_t SC_CheckBit(uint8_t *array,int num){
	
	uint8_t sum = 0;
	for (int i = 0; i < num; i++)	sum = sum + array[i];
	return sum;	
}
