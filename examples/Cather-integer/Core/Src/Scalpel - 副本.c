#include "Scalpel.h"
#include "Fos_Uart.h"
#include <math.h>


/***************************************函数声明****************************************/
uint8_t CheckStatus(double linePos, double rotatePos);
//return -1:失败
float FindTimeL(double point);
//return 1:成功，0:失败
uint8_t RefreshArray(double time);
uint16_t OcountOrder(void);
uint16_t ScountOrder(void);
uint8_t SC_CheckBit(uint8_t *array,int num);

void Scalpel_UART_Callback(UART_HandleTypeDef *huart);
void ScalpelLocated_CallBack(void *motorx,void* fatherArgs);
void Open(void);
void ScalpelOpend(void* moto,void* fatherArgs);
void Stretch(void);
void ScalpelStretched(void* moto,void* fatherArgs);
void SC_Null(void){} //回调函数的解绑空函数
	
//夹爪接收上位机控制指令的函数
uint8_t Scalpel_ReceiveCMD(uint8_t* Controlflag, uint16_t Size, void* Scalpel);
	
AtPos_Callback OpenStartPos_CallBack;	//电机到达张开起始位置后会调用一次此函数
AtPos_Callback MiddlePos_CallBack;	//电机到达张开结束\抓取开始位置后会调用一次此函数
AtPos_Callback StretchFinishPos_CallBack;	//电机到达抓取结束位置后会调用一次此函数
	
	
/***************************************全局变量****************************************/
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
uint8_t status = 0;		//0：展开运动；1:合拢运动；2：展开完成，可切换两种状态; 3:each moto move independent,need relocated
uint8_t TrajFlag=0;		//电机回原点时的到位标志	0B00000111 表示全部到位 

YS_PID wireMoto_pid={
	.Kp=0.6,
	.Ki=0.04,
	.Kd=0,
	.IS=1000
};

YS_PID pushMoto_pid={
	.Kp=0.9,
	.Ki=0.2,
	.Kd=0,
	.IS=450
};

M2006_PID_Parm pid_slow={
	.kp=2,
	.ki=0.2,
	.kd=0,
	.IS=2000,
};

//M2006_PID_Parm pid_fast={
//	.kp=1,
//	.ki=0.2,
//	.kd=0.2,
//	.IS=1000,
//};

uint8_t pushMotoFlag = 0x01;
YS_Moto pushMoto={
	.fatherArgs = &pushMotoFlag,};

uint8_t wireMotoFlag = 0x02;
YS_Moto wireMoto={
	.fatherArgs = &wireMotoFlag,};

uint8_t rotateMotoFlag = 0x04;	
M2006 rotateMoto={
	.fatherArgs = &rotateMotoFlag,};
	
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
	
	M2006_Init(&rotateMoto,0X01,
				(uint16_t)(_DeataT*1000),	//发送周期
				10,		//数据处理周期
				pid_slow,
				1,	//机械参数：直线运动机构
				2.0/3.0,	//外部减速比
				0	/*起始角度*/);	
				
	CAN_AddDevice(&rotateMoto,M2006_CanDataDecode);		/*********这里可以包含在M2006_init里***********/
	
	UART1_AddDevice(NULL,Scalpel_ReceiveCMD);
	
	//由于机械结构限制，要让运动轨迹的绝对位置添加一个修正值，防止发送机械干涉
	float pushAxis = 2;
	float lineAxis = 4.0;
	float rotateAxis = 0.0;
	
	_OPOrign[0] = 0;   	_OLOrign[0] = 3.7;
	_OPOrign[1] = 0.03;   _OLOrign[1] = 4.2;
	_OPOrign[2] = 0.08;   _OLOrign[2] = 4.8;
	_OPOrign[3] = 0.12;   _OLOrign[3] = 5.47;
	_OPOrign[4] = 0.18;   _OLOrign[4] = 6.18;
	_OPOrign[5] = 0.21;   _OLOrign[5] = 6.93;
	_OPOrign[6] = 0.25;   _OLOrign[6] = 7.7;
	_OPOrign[7] = 0.3;   _OLOrign[7] = 8.48;
	_OPOrign[8] = 0.34;   _OLOrign[8] = 9.27;
	_OPOrign[9] = 0.39;   _OLOrign[9] = 10.08;
	_OPOrign[10] = 0.43;   _OLOrign[10] = 10.88;
	_OPOrign[11] = 0.46;   _OLOrign[11] = 11.7;
	_OPOrign[12] = 0.5;   _OLOrign[12] = 12.51;
	_OPOrign[13] = 0.55;   _OLOrign[13] = 13.33;
	_OPOrign[14] = 0.59;   _OLOrign[14] = 14.14;
	_OPOrign[15] = 0.64;   _OLOrign[15] = 14.96;
	_OPOrign[16] = 0.68;   _OLOrign[16] = 15.77;
	_OPOrign[17] = 0.71;   _OLOrign[17] = 16.58;
	_OPOrign[18] = 0.75;   _OLOrign[18] = 17.39;
	_OPOrign[19] = 0.8;   _OLOrign[19] = 18.2;
	_OPOrign[20] = 0.84;   _OLOrign[20] = 19.01;
	_OPOrign[21] = 0.87;   _OLOrign[21] = 19.81;
	_OPOrign[22] = 0.91;   _OLOrign[22] = 20.6;
	_OPOrign[23] = 0.96;   _OLOrign[23] = 21.4;
	_OPOrign[24] = 1;   _OLOrign[24] = 22.19;
	_OPOrign[25] = 1.04;   _OLOrign[25] = 22.97;
	_OPOrign[26] = 1.08;   _OLOrign[26] = 23.75;
	_OPOrign[27] = 1.13;   _OLOrign[27] = 24.52;
	_OPOrign[28] = 1.17;   _OLOrign[28] = 25.29;
	_OPOrign[29] = 1.21;   _OLOrign[29] = 26.05;
	_OPOrign[30] = 1.25;   _OLOrign[30] = 26.8;
	_OPOrign[31] = 1.29;   _OLOrign[31] = 27.55;
	_OPOrign[32] = 1.34;   _OLOrign[32] = 28.29;
	_OPOrign[33] = 1.38;   _OLOrign[33] = 29.03;
	_OPOrign[34] = 1.42;   _OLOrign[34] = 29.75;
	_OPOrign[35] = 1.47;   _OLOrign[35] = 30.47;
	_OPOrign[36] = 1.51;   _OLOrign[36] = 31.18;
	_OPOrign[37] = 1.55;   _OLOrign[37] = 31.89;
	_OPOrign[38] = 1.6;   _OLOrign[38] = 32.58;
	_OPOrign[39] = 1.65;   _OLOrign[39] = 33.27;
	_OPOrign[40] = 1.69;   _OLOrign[40] = 33.94;
	_OPOrign[41] = 1.74;   _OLOrign[41] = 34.61;
	_OPOrign[42] = 1.79;   _OLOrign[42] = 35.27;
	_OPOrign[43] = 1.84;   _OLOrign[43] = 35.92;
	_OPOrign[44] = 1.89;   _OLOrign[44] = 36.56;
	_OPOrign[45] = 1.94;   _OLOrign[45] = 37.19;



	for (int i = 0; i < _Olong; i++)	
	{ 
		_OPOrign[i] += pushAxis;
		_OLOrign[i] += lineAxis;
		_OROrign[i] = rotateAxis; 
	}
	
	_SLOrign[0] = 37.19;
	_SLOrign[1] = 36.35;
	_SLOrign[2] = 35.51;
	_SLOrign[3] = 34.66;
	_SLOrign[4] = 33.8;
	_SLOrign[5] = 32.93;
	_SLOrign[6] = 32.04;
	_SLOrign[7] = 31.15;
	_SLOrign[8] = 30.26;
	_SLOrign[9] = 29.37;
	_SLOrign[10] = 28.47;
	_SLOrign[11] = 27.57;
	_SLOrign[12] = 26.68;
	_SLOrign[13] = 25.79;
	_SLOrign[14] = 24.91;
	_SLOrign[15] = 24.03;
	_SLOrign[16] = 23.16;
	_SLOrign[17] = 22.3;
	_SLOrign[18] = 21.46;
	_SLOrign[19] = 20.62;
	_SLOrign[20] = 19.8;
	_SLOrign[21] = 18.98;
	_SLOrign[22] = 18.19;
	_SLOrign[23] = 17.41;
	_SLOrign[24] = 16.64;
	_SLOrign[25] = 15.89;
	_SLOrign[26] = 15.16;
	_SLOrign[27] = 14.44;
	_SLOrign[28] = 13.74;
	_SLOrign[29] = 13.06;
	_SLOrign[30] = 12.39;
	_SLOrign[31] = 11.75;
	_SLOrign[32] = 11.12;
	_SLOrign[33] = 10.51;
	_SLOrign[34] = 9.92;
	_SLOrign[35] = 9.35;
	_SLOrign[36] = 8.8;
	_SLOrign[37] = 8.27;
	_SLOrign[38] = 7.76;
	_SLOrign[39] = 7.27;
	_SLOrign[40] = 6.81;
	_SLOrign[41] = 6.36;
	_SLOrign[42] = 5.94;
	_SLOrign[43] = 5.55;
	_SLOrign[44] = 5.18;
	_SLOrign[45] = 4.83;

	for (int i = 0; i < _Slong; i++)
	{
		_SROrign[i] = i + rotateAxis;
		_SLOrign[i] += lineAxis;
		_SPOrign[i] = _OPOrign[_Olong - 1]+ pushAxis;
	}
	

	if( OpenStartPos_CallBack==NULL )		OpenStartPos_CallBack = SC_Null;
	if( MiddlePos_CallBack==NULL )			MiddlePos_CallBack = SC_Null;
	if( StretchFinishPos_CallBack==NULL )	StretchFinishPos_CallBack = SC_Null;
}

void ScalpelStop(void)
{
	YS_Stop(&pushMoto);
	YS_Stop(&wireMoto);
	DJ_Lock(&rotateMoto);
}

void ScalpelGoStartPos(void)
{
	TrajFlag=0;
	YS_FastGoto(&pushMoto,_OPOrign[0]);
	YS_FastGoto(&wireMoto,_OLOrign[0]);
	DJ_Goto(&rotateMoto,_OROrign[0],30,1);
	
	pushMoto.YS_P_drive_ISR_CallBack = ScalpelLocated_CallBack;
	wireMoto.YS_P_drive_ISR_CallBack = ScalpelLocated_CallBack;
	rotateMoto.FinishGoto_ISR_CallBack = ScalpelLocated_CallBack;
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
		DJ_FinishGoto_ISR_DisCallBack(&rotateMoto);
		
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
	float rotatePoint = rotateMoto.point;
	
	//don't use fuction checkStatus,becaues moto pos error may cause estimate status error
//	if (CheckStatus(wirePoint, rotatePoint) == 1) 
		if((status == 1 )||(status == 3))
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
	
	wireMoto.YS_atTrajEnd_ISR_CallBack = ScalpelOpend;
	pushMoto.YS_atTrajEnd_ISR_CallBack = ScalpelOpend;
	rotateMoto.FinishPos_Traj_ISR_CallBack = ScalpelOpend;
	YS_FollowPos_Traj(&wireMoto,_OLOrign + _OcountNow,count,(int)(_DeataT*1000),1);
	YS_FollowPos_Traj(&pushMoto,_OPOrign + _OcountNow,count,(int)(_DeataT*1000),1);
	DJ_FollowPos_Traj(&rotateMoto,_OROrign + _OcountNow,count,1,1);
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
		DJ_FinishPos_Traj_ISR_DisCallBack(&rotateMoto);
		MiddlePos_CallBack();
		static uint8_t info[]={"Scalpel at middle pos\r\n"};
		FOS_UART_Transmit_DMA(&huart1,info,sizeof(info));
	}
}

void Stretch(void)
{
	float wirePoint = wireMoto.nowPosi;
	float rotatePoint = rotateMoto.point;
	
	//don't use fuction checkStatus,becaues moto pos error may cause estimate status error
//	if (CheckStatus(wirePoint, rotatePoint) == 1) 
		if((status == 0 )||(status == 3))
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
	
	wireMoto.YS_atTrajEnd_ISR_CallBack = ScalpelStretched;
//	pushMoto.YS_atTrajEnd_ISR_CallBack = ScalpelStretched;
	rotateMoto.FinishPos_Traj_ISR_CallBack = ScalpelStretched;
	YS_FollowPos_Traj(&wireMoto,_SLOrign + _ScountNow,count,(int)(_DeataT*1000),1);
//	YS_FollowPos_Traj(&pushMoto,_SPOrign + _ScountNow,count,(int)(_DeataT*1000),1);	//从联动结构上看，这个电机不需要运动
	DJ_FollowPos_Traj(&rotateMoto,_SROrign + _ScountNow,count,1,1);
}

void ScalpelStretched(void* moto,void* fatherArgs)
{
	uint8_t *arg = (uint8_t *)fatherArgs;
	TrajFlag |= (*arg);	
	
	if(TrajFlag==0x06)
	{
//		不要在这个函数里使用耗时过长的逻辑
		status=1;
//		YS_atTrajEnd_DisCallBack(&pushMoto);
		YS_atTrajEnd_DisCallBack(&wireMoto);
		DJ_FinishPos_Traj_ISR_DisCallBack(&rotateMoto);
		
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
		case 0x00 :		//正方向运动
			ScalpelFront();
		break;
		
//		case 0x01 :		//负方向运动
//			ScalpelBack();
//		break;
		
		case 0x02 :		//停止运动
			ScalpelStop();
		break;
		
		case 0x05 :		//手术刀双电机回轨迹起点	
			ScalpelGoStartPos();
		break;
	}
	return IsMy_uart1_Data;
}


/**********************************函数实现（private）**********************************/
uint8_t CheckStatus(double linePos, double rotatePos)
{
	if (rotatePos < _SROrign[1])
	{
		if (linePos < _SLOrign[1])
		{
			status = 0;
			return status;
		}
		status = 2;
		return status;
	}
	status = 1;
	return status;
}

//return -1:失败
float FindTimeL(double point)
{
	if (status == 1)		//true:合拢运动
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
	if (status == 0)		//true:张开运动
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
	return -1;	//-1是个错误返回值
}

//return 1:成功，0:失败
uint8_t RefreshArray(double time)
{
	if (status == 1)	//合拢状态中
	{
		_ScountNow = time / _DeataT;
		return 1;
	}
	if (status == 0)   //张开状态中
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
