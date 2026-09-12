#include "YS_moto.h"
#include "math.h"
#include "stdio.h"
#include <stdio.h>  
#include "malloc.h"

#define YS_RETURN_LEN_MAX 50	//电机接收中断的缓冲区大小，根据手册一组数据就 22 字节大小
const uint16_t posArrayLength = (VEL_SAMPLE_PERIOD / YS_ASK_TIME);
const uint32_t YS_OsDelayTime = (YS_ASK_TIME *1000.0 / YS_MOTO_NUM / 1000.0);

/**************************结构体、联合体、枚举类型（private）**************************/
// 电机运动状态
typedef enum YS_Status{
	//单次指令类运动
	YS_stop = 0, 
	YS_fastGoto = 1,	//快速到达指定位置
	YS_home=2,	//快速回到原点位置
	
	//连续速度追踪	
	YS_veloControl=3,
	YS_goto =4,
	
	//连续位置追踪
	YS_trajFollow=5,
	YS_sensorFollow=6,		
	
	//YS_trajFollow 的 子状态
	YS_lastTraj=7,
} YS_Status;


//数据包解析格式，参考因时的电器手册
typedef union _YS_ReturnData
{	
	uint16_t data;
	uint8_t arr[2];
} YS_ReturnData;

typedef struct _YS_Private{
	uint8_t* UART_control;		//串口发送数据存储位置，长度：9
	uint8_t* UART_return;		//串口接收数据存储位置，长度：YS_RETURN_LEN_MAX

	YS_Status status;		//运动状态
	
	int16_t enlarge;	//电机的位置与脉冲数换算，比如50mm要2000脉冲，此值应为40
	uint16_t zeroPosi; //电机初始位置 单位mm
	uint16_t maxPosi; //电机最大位置 单位mm
	
	//依确定轨迹数组运动所需参数
	float *allaimPosition;		//目标位置序列
	int posArrayNum;		//位置序列长度
	uint16_t aimStep;			//当前需要到达的位置
	int countNow;		//当前位置序列到达编号
	int trajDir;		//沿位置序列运动方向	1：正方向；-1：负方向
	float reachedPos;	//专指轨迹序列中的值，电机已经到达的位置
	float targetPos;	//专指轨迹序列中的值，电机下一个要到达的位置
	float targetVelc;	//专指从 targetpos到reachedPos 的速度，注意与 aimStepVelo 完全不是一个东西
	
	//依实时变换的传感器信号运动所需参数
	//注意：传感器信号必须为 float 类型，传感器信号需提前处理平滑
	float *sensorData;
	
	int timeInterval;		//运动轨迹序列的时间间隔，注意与 YS_ASK_TIME 的区别
	
	//控制和运动轨迹序列共用一个定时器，因此需要通过对控制定时器进行分频
	//以满足运动轨迹序列所需定时周期
	uint16_t frequeDivision;	//控制定时器的分频系数
	uint16_t frequeNow;		//控制定时器运行到第几个周期
	
	//控制和速度刷新共用一个定时器，因此需要通过对控制定时器进行分频
	//以满足速度刷新所需定时周期
	uint16_t velcfrequeNow;	//控制定时器运行到第几个周期

	float *posLoopArry;		//用循环列表存储采样来的位置数据 长度：posArrayLength
	uint16_t array_i;		//列表存储到了哪一位

	float aimStepVelo; //电机目标速度 单位 步/s
	
	int16_t giveVolt;	//要给电机的输入电压
	
	uint8_t mode;		//电机的运动状态
}YS_Private;


/***************************************全局变量****************************************/
YS_Moto* motoList[YS_MOTO_NUM];		//所有因时的电机都会列入此数组

osTimerId_t YS_Control_STimerHandle;	//控制定时器，由YS_ASK_TIME 确定定时周期
const osTimerAttr_t YS_Control_STimer_attributes = {
.name = "YS_SoftTimer"};

/***************************************函数声明****************************************/
//一个空函数，防止回调函数指向野指针
void YS_Null(YS_Moto* moto,void* fatherArgs) {}
	
//电机校验和函数
//array：需要校验和数组
//num：数组大小
uint8_t YS_CheckBit(uint8_t array[],int num);

//内部功能，电压驱动控制模式
//moto:直线电机结构体
uint8_t __YS_Volt_drive(YS_Moto *moto);
	
//内部功能，电压为0的停止方式
//moto:直线电机结构体
uint8_t __YS_Stop(YS_Moto *moto);
	
void YS_Moto_Data_UART_Callback(UART_HandleTypeDef *huart,const uint8_t *pdata,const uint16_t size);
void YS_SoftTimerCallback(void * argument);
float PID_Ctrl(YS_PID *pid,float aim,float now);

void MeanFliter(YS_Moto* moto,float pos);
	
/**********************************函数实现（public）**********************************/
//因时电机的初始化函数
void YS_MotoInit(YS_Moto *moto, uint8_t id, UART_HandleTypeDef *huart, 
					int16_t enlarge,	//电机的位置与脉冲数换算，比如50mm要2000脉冲，此值应为40
					uint16_t zeroPosi, //电机初始位置 单位mm
					uint16_t maxPosi, //电机最大位置 单位mm
					YS_PID* pid_V)	//速度环pid参数
{
#if (YS_USE_SOFT_TIMER==1)	
	static uint8_t onetimeFlag=0;	//初始化时仅进行一次的标志位。
	if(onetimeFlag==0)//软定时器启动一次就行
	{		
		YS_Control_STimerHandle= osTimerNew(YS_SoftTimerCallback, osTimerPeriodic, NULL, &YS_Control_STimer_attributes);
		osTimerStart(YS_Control_STimerHandle,YS_ASK_TIME);
		FOS_InitUartBuffer();
		
		onetimeFlag=1;
	}
#endif
	
	static int i=0;
	if (i>=YS_MOTO_NUM)	
	{
		static uint8_t info[]={"too much YS_moto init!\r\n"};
		FOS_UART_Transmit_DMA(&huart1,info,sizeof(info));
		return;
	}
	
	motoList[i]=moto;
	
	moto->ID=id;
	moto->huart=huart;
	moto->priVari=(YS_Private*)mymalloc(SRAMIN,sizeof(YS_Private));	
	moto->priVari->UART_control=(uint8_t *)mymalloc(SRAMIN,9*sizeof(uint8_t));
	moto->priVari->UART_return=(uint8_t *)mymalloc(SRAMIN,YS_RETURN_LEN_MAX*sizeof(uint8_t));
	moto->priVari->posLoopArry=(float *)mymalloc(SRAMIN,posArrayLength*sizeof(float));
	
	YS_Private* motoPrivari = moto->priVari;
	
	motoPrivari->enlarge=enlarge;
	motoPrivari->zeroPosi=zeroPosi;
	motoPrivari->maxPosi=maxPosi;
	
	moto->pid_V.Kp=pid_V->Kp;	moto->pid_V.Ki=pid_V->Ki;	moto->pid_V.Kd=pid_V->Kd; moto->pid_V.IS =pid_V->IS;
	moto->pid_V.allErr=0;	moto->pid_V.lastErr=0;	moto->pid_V.thisErr=0;
	moto->pid_V.returnLimit = 1000;

	moto->priVari->velcfrequeNow=1;
	
	if(moto->YS_RefreshData_ISR_CallBack == NULL) moto->YS_RefreshData_ISR_CallBack=YS_Null;
	if(moto->YS_P_drive_ISR_CallBack==NULL)	{moto->YS_P_drive_ISR_CallBack=YS_Null;}
	if(moto->YS_atHome_ISR_CallBack==NULL)	{moto->YS_atHome_ISR_CallBack=YS_Null;}
	if(moto->YS_atTrajEnd_ISR_CallBack==NULL)	{moto->YS_atTrajEnd_ISR_CallBack=YS_Null;}
	
	Register_RX_CallBack(huart,YS_Moto_Data_UART_Callback);
	
	i++;		
}

//直接以最短时间到达电机指定位置，他通过单次发送到位指令实现
//pos：目标位置，单位mm
uint8_t YS_FastGoto(YS_Moto *moto,float pos){
	YS_Private* motoPrivari = moto->priVari;
#if YS_DEBUG
	if((pos-motoPrivari->zeroPosi)*(pos-motoPrivari->maxPosi)>0)
	{
		static uint8_t info[]={"aimPos unlimitied!\r\n"};
		FOS_UART_Transmit(&huart1,info,sizeof(info),5);
		motoPrivari->status=YS_stop;
		return 0;
	}
#endif
	
	motoPrivari->aimStep=(uint16_t)(pos*(float)motoPrivari->enlarge);
	motoPrivari->status=YS_fastGoto;
	
	motoPrivari->UART_control[0]=0x55;		//帧头
	motoPrivari->UART_control[1]=0xAA;		//帧头
	motoPrivari->UART_control[2]=0x04;		//数据长度
	motoPrivari->UART_control[3]=moto->ID;	//moto ID
	motoPrivari->UART_control[4]=0x03;		//控制命令类型 向控制表内写入数据，无需回复应答帧
	motoPrivari->UART_control[5]=0x37;		//控制表索引 目标位置首地址
	motoPrivari->UART_control[6]=motoPrivari->aimStep%256;		//motoPrivari->aimPosiHigh
	motoPrivari->UART_control[7]=motoPrivari->aimStep/256;		//motoPrivari->aimPosiLow
	motoPrivari->UART_control[8]=YS_CheckBit(motoPrivari->UART_control, 8);	//校验位	

	FOS_UART_Transmit(moto->huart, motoPrivari->UART_control, 9,100);

	return 1;
}

//直接以最短时间到达电机初始位置，他通过单次发送到位指令实现
//moto:直线电机结构体
uint8_t YS_Home(YS_Moto *moto){	
	YS_Private* motoPrivari = moto->priVari;
	motoPrivari->aimStep=motoPrivari->zeroPosi*motoPrivari->enlarge;
	motoPrivari->status=YS_home;
	
	motoPrivari->UART_control[0]=0x55;		//帧头
	motoPrivari->UART_control[1]=0xAA;		//帧头
	motoPrivari->UART_control[2]=0x04;		//数据长度
	motoPrivari->UART_control[3]=moto->ID;	//moto ID
	motoPrivari->UART_control[4]=0x03;		//控制命令类型 向控制表内写入数据，无需回复应答帧
	motoPrivari->UART_control[5]=0x37;		//控制表索引 目标位置首地址
	motoPrivari->UART_control[6]=motoPrivari->aimStep%256;		//motoPrivari->zeroPosiHigh
	motoPrivari->UART_control[7]=motoPrivari->aimStep/256;		//motoPrivari->zeroPosiLow
	motoPrivari->UART_control[8]=YS_CheckBit(motoPrivari->UART_control, 8);	//校验位	

	FOS_UART_Transmit(moto->huart, motoPrivari->UART_control, 9,100);
	return 1;
}

//电机以velc速度运动到pos位置，他通过底层闭环控制速度，到达位置。
//pos：目标位置，单位mm
//velc：目标位置，单位mm/s
uint8_t YS_Posi_drive(YS_Moto *moto,float pos,float velc){
	YS_Private* motoPrivari = moto->priVari;
#if YS_DEBUG
	if((pos-motoPrivari->zeroPosi)*(pos-motoPrivari->maxPosi)>0)
	{
		static uint8_t info[]={"aimPos unlimitied!\r\n"};
		FOS_UART_Transmit(&huart1,info,sizeof(info),5);
		motoPrivari->status=YS_stop;
		return 0;
	}
#endif

	motoPrivari->aimStep=(uint16_t)(pos*(float)motoPrivari->enlarge);
	motoPrivari->aimStepVelo=(uint16_t)(velc*(float)motoPrivari->enlarge);
	motoPrivari->status=YS_goto;
	return 1;
}

//电机速度驱动函数,调用后会一直保持指定速度
//moto:直线电机结构体
//velocity：目标速度值（单位 mm/s）
uint8_t YS_Velo_drive(YS_Moto *moto,float velocity)
{
	YS_Private* motoPrivari = moto->priVari;
	motoPrivari->status=YS_veloControl;
	motoPrivari->aimStepVelo=(int16_t)(velocity*(float)motoPrivari->enlarge);	//(单位:步数/s)
	motoPrivari->giveVolt = (int16_t)PID_Ctrl(&(moto->pid_V),motoPrivari->aimStepVelo,moto->nowVelo*(float)motoPrivari->enlarge);
	return 1;
}

//电机跟随一条既定轨迹运动，轨迹有终止时刻 结束后会 调用 YS_atTrajEnd_ISR_CallBack 函数一次
//traj:轨迹数组首地址（单位：mm）；	length：数组长度	timeInterval：数据刷新时间（单位 ms）（YS_ASK_TIME的整数倍）
//dir	+1：随数组正向运动；-1：随数组反向运动（必须是这两个数）
uint8_t YS_FollowPos_Traj(YS_Moto *moto,float *traj,int length,int timeInterval, int dir)
{
	YS_Private* motoPrivari = moto->priVari;
#if YS_DEBUG
	float pos =traj[0];
	if(((int16_t)pos-(int16_t)motoPrivari->zeroPosi)*((int16_t)pos-(int16_t)motoPrivari->maxPosi)>0)
	{
		static uint8_t info[]={"aimPos unlimitied!\r\n"};
		FOS_UART_Transmit(&huart1,info,sizeof(info),5);

		motoPrivari->status=YS_stop;
		return 0;	}
#endif
	
	motoPrivari->allaimPosition=traj;
	motoPrivari->posArrayNum=length;
	motoPrivari->timeInterval=timeInterval;
	motoPrivari->frequeDivision=timeInterval/YS_ASK_TIME;
	motoPrivari->trajDir=dir;
	
	motoPrivari->countNow=0;	
	motoPrivari->reachedPos = traj[0];	//在调用准备函数时，应该已经让电机到达轨迹第一个点的位置
	motoPrivari->targetPos = traj[0];	//这个值会在真正的轨迹跟踪前被覆盖掉，但他必须存在以保证第一次循环数据的正确。
	motoPrivari->frequeNow=timeInterval/YS_ASK_TIME;	//为了直接开始运动，所以没有设为0

	motoPrivari->status=YS_trajFollow;
	return 1;
}

//void YS_Stop(YS_Moto *moto)
//{
//	YS_Private* motoPrivari = moto->priVari;
//	motoPrivari->aimStepVelo=0;
//	motoPrivari->giveVolt=0;
//	motoPrivari->status=YS_stop;	
//	
//	//本质操作是让电机到达现在已经在的位置，因为只有这个指令没有反馈值
//	//可以随便调用，不会打乱现在的反馈控制周期
//	motoPrivari->aimStep=(moto->nowPosi)*motoPrivari->enlarge;
//	
//	motoPrivari->UART_control[0]=0x55;		//帧头
//	motoPrivari->UART_control[1]=0xAA;		//帧头
//	motoPrivari->UART_control[2]=0x04;		//数据长度
//	motoPrivari->UART_control[3]=moto->ID;	//moto ID
//	motoPrivari->UART_control[4]=0x03;		//控制命令类型 向控制表内写入数据，无需回复应答帧
//	motoPrivari->UART_control[5]=0x37;		//控制表索引 目标位置首地址
//	motoPrivari->UART_control[6]=motoPrivari->aimStep%256;		//motoPrivari->aimPosiHigh
//	motoPrivari->UART_control[7]=motoPrivari->aimStep/256;		//motoPrivari->aimPosiLow
//	motoPrivari->UART_control[8]=YS_CheckBit(motoPrivari->UART_control, 8);	//校验位	
//	
//	FOS_UART_Transmit(moto->huart, motoPrivari->UART_control, 9,100);
//}

void YS_Stop(YS_Moto *moto)
{
	YS_Private* motoPrivari = moto->priVari;
	motoPrivari->aimStepVelo=0;
	motoPrivari->giveVolt=0;
	motoPrivari->status=YS_stop;	
	
	//本质操作是让电机到达现在已经在的位置，因为只有这个指令没有反馈值
	//可以随便调用，不会打乱现在的反馈控制周期
	motoPrivari->aimStep=(moto->nowPosi)*motoPrivari->enlarge;
	
	motoPrivari->UART_control[0]=0x55;		//帧头
	motoPrivari->UART_control[1]=0xAA;		//帧头
	motoPrivari->UART_control[2]=0x03;		//数据长度
	motoPrivari->UART_control[3]=moto->ID;	//moto ID
	motoPrivari->UART_control[4]=0x04;		//控制单元向电缸发送单控命令
	motoPrivari->UART_control[5]=0x00;		//无用数据位
	motoPrivari->UART_control[6]=0x14;		//暂停，即禁止电机功率驱动输出（需要运动时直接发送位置指令即可运动）
	motoPrivari->UART_control[7]=YS_CheckBit(motoPrivari->UART_control, 7);	//校验位	
	
	FOS_UART_Transmit(moto->huart, motoPrivari->UART_control, 8,100);
}

//解绑回调函数
void YS_RefreshData_ISR_DisCallBack(YS_Moto* moto)	{moto->YS_RefreshData_ISR_CallBack=YS_Null;}
void YS_P_drive_ISR_DisCallBack(YS_Moto* moto) 	{moto->YS_P_drive_ISR_CallBack=YS_Null;}
void YS_atHome_ISR_DisCallBack(YS_Moto* moto) 	{moto->YS_atHome_ISR_CallBack=YS_Null;}
void YS_atTrajEnd_DisCallBack(YS_Moto* moto)	{moto->YS_atTrajEnd_ISR_CallBack=YS_Null;} 

/**********************************函数实现（private）**********************************/
//电机校验和函数
//array：需要校验和数组
//num：数组大小
uint8_t YS_CheckBit(uint8_t array[],int num){
	
	uint8_t sum = 0;
	for (int i = 2; i < num; i++)	sum = sum + array[i];
	return sum;	
}

//内部功能，清除可以消除的电机异常状态报警（硬件问题没解决的报警没法消除）
//moto:直线电机结构体
uint8_t __YS_Clear(YS_Moto *moto){
	YS_Private* motoPrivari = moto->priVari;
	motoPrivari->UART_control[0]=0x55;		//帧头
	motoPrivari->UART_control[1]=0xAA;		//帧头
	motoPrivari->UART_control[2]=0x03;		//数据长度
	motoPrivari->UART_control[3]=moto->ID;  //moto ID
	motoPrivari->UART_control[4]=0x04;		//实现对直线伺服电缸的功能控制
	motoPrivari->UART_control[5]=0x00;		//0
	motoPrivari->UART_control[6]=0x1E;		//故障清除
	motoPrivari->UART_control[7]=YS_CheckBit(motoPrivari->UART_control, 7);	//校验位		
	
	FOS_UART_Transmit(moto->huart, motoPrivari->UART_control, 8,5);
	return 1;
}

//内部功能，位置驱动控制模式
//moto:直线电机结构体
uint8_t __YS_Pos_drive(YS_Moto *moto){
	YS_Private* motoPrivari = moto->priVari;

	motoPrivari->UART_control[0]=0x55;		//帧头
	motoPrivari->UART_control[1]=0xAA;		//帧头
	motoPrivari->UART_control[2]=0x04;		//数据长度
	motoPrivari->UART_control[3]=moto->ID;	//moto ID
	motoPrivari->UART_control[4]=0x21;		//控制命令类型
	motoPrivari->UART_control[5]=0x37;		//控制表索引 目标位置首地址
	motoPrivari->UART_control[6]=motoPrivari->aimStep%256;		//motoPrivari->aimPosiHigh
	motoPrivari->UART_control[7]=motoPrivari->aimStep/256;		//motoPrivari->aimPosiLow
	motoPrivari->UART_control[8]=YS_CheckBit(motoPrivari->UART_control, 8);	//校验位	

	FOS_UART_Transmit(moto->huart, motoPrivari->UART_control, 9,5);
	return 1;
}

//内部功能，电压驱动控制模式
//moto:直线电机结构体
uint8_t __YS_Volt_drive(YS_Moto *moto){	
	YS_Private* motoPrivari = moto->priVari;
	motoPrivari->UART_control[0] = 0x55;		//帧头
	motoPrivari->UART_control[1] = 0xAA;		//帧头
	motoPrivari->UART_control[2] = 0x04;		//数据长度
	motoPrivari->UART_control[3] = moto->ID;		//moto ID
	motoPrivari->UART_control[4] = 0x2F;		//控制命令类型
	motoPrivari->UART_control[5] = 0x00;		//0
	motoPrivari->UART_control[6] = motoPrivari->giveVolt%256;		//moto->giveVolHigh
	motoPrivari->UART_control[7] = motoPrivari->giveVolt/256;		//moto->giveVolLow
	motoPrivari->UART_control[8] = YS_CheckBit(motoPrivari->UART_control, 8);	//校验位	

	FOS_UART_Transmit(moto->huart, motoPrivari->UART_control, 9,5);
	return 1;
}

//内部功能：让电机锁定在这个位置，和外部调用的stop指令有明显的不同
uint8_t __YS_Stop(YS_Moto *moto)
{
	YS_Private* motoPrivari = moto->priVari;
	motoPrivari->aimStepVelo=0;
	motoPrivari->giveVolt=0;
	motoPrivari->status=YS_stop;	
	
	//本质操作是让电机到达现在已经在的位置，因为只有这个指令没有反馈值
	//可以随便调用，不会打乱现在的反馈控制周期
	motoPrivari->aimStep=(moto->nowPosi)*motoPrivari->enlarge;
	
	motoPrivari->UART_control[0]=0x55;		//帧头
	motoPrivari->UART_control[1]=0xAA;		//帧头
	motoPrivari->UART_control[2]=0x04;		//数据长度
	motoPrivari->UART_control[3]=moto->ID;	//moto ID
	motoPrivari->UART_control[4]=0x03;		//控制命令类型 向控制表内写入数据，无需回复应答帧
	motoPrivari->UART_control[5]=0x37;		//控制表索引 目标位置首地址
	motoPrivari->UART_control[6]=motoPrivari->aimStep%256;		//motoPrivari->aimPosiHigh
	motoPrivari->UART_control[7]=motoPrivari->aimStep/256;		//motoPrivari->aimPosiLow
	motoPrivari->UART_control[8]=YS_CheckBit(motoPrivari->UART_control, 8);	//校验位	
	
	//这里不建议使用中断，因为这个指令通常单次使用，阻塞式发送能确保稳定
	FOS_UART_Transmit(moto->huart, moto->priVari->UART_control, 9,5);
	return 1;
}

//PID控制
float PID_Ctrl(YS_PID *pid,float aim,float now){
	
	float give;
	pid->thisErr= aim-now;
	pid->allErr += pid->thisErr;
	
	if(pid->allErr >= pid->IS) give = pid->IS;
	if(pid->allErr <= -1.0*pid->IS) give = -1.0*pid->IS;
	
	give = pid->Kp * pid->thisErr + pid->Ki * pid->allErr +pid->Kd * (pid->thisErr - pid->lastErr);

	if(give >= pid->returnLimit) give = pid->returnLimit;
	if(give <= -1.0*pid->returnLimit) give = -1.0*pid->returnLimit;

  //后续处理
	pid->lastErr = pid->thisErr;
	
	return give;
}

//速度均值滤波
void MeanFliter(YS_Moto* moto,float pos)
{
	YS_Private *motoPriv = moto->priVari;
	
	moto->nowPosi = pos;	
	motoPriv->posLoopArry[motoPriv->array_i] = pos;
	
	uint16_t nextArray_i = (motoPriv->array_i + 1) % posArrayLength;
	moto->nowVelo = (pos-motoPriv->posLoopArry[nextArray_i]) * 1000.0 / (float)VEL_SAMPLE_PERIOD;	
	
	if(nextArray_i != 0) { motoPriv->array_i++;}
	else {motoPriv->array_i = 0;}
}

//软定时器回调函数
void YS_SoftTimerCallback(void *argument)
{
	/**********************************************************************************
	因时电机一个很麻烦的点是，任何的操作，都会返回一个包含当前状态的数据包，
	而我们是依靠稳定时间间隔，通过两次位置信息的时间差，来计算速度的，
	这就要求我们严格限制对电机发送指令的时间差，随意发送指令会导致速度数据出现严重问题
	**********************************************************************************/
	static YS_Moto *moto;
	for(int i=0;i<YS_MOTO_NUM;i++)
	{
		moto=motoList[i];		
		YS_Private* motoPrivari = moto->priVari;
		
		switch(motoPrivari->status)
		{
			case YS_veloControl:
			{
				//下一周期电机的电压控制值
				motoPrivari->giveVolt=(int16_t)PID_Ctrl(&(moto->pid_V),motoPrivari->aimStepVelo,moto->nowVelo*(float)motoPrivari->enlarge);			
				__YS_Volt_drive(moto);				
#if YS_DEBUG
//				//传输数据格式：ID、目标步距位置、实际步距位置、目标步距速度、实际步距速度、控制电压				
//				FOS_printf("ID:%d vt:%.3f vr:%.3f volt:%d\r\n" , 
//							moto->ID,					
//							(double)((float)motoPrivari->aimStepVelo/(float)motoPrivari->enlarge), 
//							(double)moto->nowVelo,
//							motoPrivari->giveVolt);		
#endif		
				break;
			}		
			case YS_goto:
			{
				//下一周期电机的电压控制值
				motoPrivari->giveVolt=(int16_t)PID_Ctrl(&(moto->pid_V),motoPrivari->aimStepVelo,moto->nowVelo*(float)motoPrivari->enlarge);	
				__YS_Volt_drive(moto);		
				break;
			}		
			case YS_trajFollow:
			{
				if(motoPrivari->frequeNow>=motoPrivari->frequeDivision)	//定时器计数达到了更新位置点的时间周期
				{
					motoPrivari->frequeNow=1;
					motoPrivari->countNow ++;
//					motoPrivari->reachedPos = moto->nowPosi;
					motoPrivari->reachedPos = motoPrivari->targetPos;					
					motoPrivari->targetPos = *( motoPrivari->allaimPosition += motoPrivari->trajDir );	
					motoPrivari->targetVelc = (motoPrivari->targetPos - motoPrivari->reachedPos)/(float)motoPrivari->timeInterval;								
				}
				else { motoPrivari->frequeNow++; }		
				
				motoPrivari->aimStep=(uint16_t)((motoPrivari->reachedPos + motoPrivari->targetVelc * (float)motoPrivari->frequeNow * (float)YS_ASK_TIME)
												 * (float)motoPrivari->enlarge);
				__YS_Pos_drive(moto);
#if YS_DEBUG			
					if(moto->ID == 0)
					{
//						//传输数据格式：ID、下一个轨迹点、上一个轨迹点、下一个到达点、目标步距速度				
//						FOS_printf("ID:%d tap:%.3f rap:%.3f pt:%d vr:%.3f\r\n" , 
//									moto->ID,
//									(double)motoPrivari->targetPos,
//									(double)motoPrivari->reachedPos,
//									motoPrivari->aimStep,									
//									(double)((float)motoPrivari->aimStepVelo/(float)motoPrivari->enlarge));
						
						//传输数据格式：下一个轨迹点（target array step point）、电机的实际位置、下一个到达点、下一到达位置				
//						FOS_printf("tasp:%.1f pr:%.1f pst:%d pt:%.1f\r\n" , 
						FOS_printf("%.1f %.1f %d %.1f\r\n" , 
									(double)motoPrivari->targetPos,
									(double)moto->nowPosi,
									motoPrivari->aimStep,
									(double)((float)motoPrivari->aimStep / (float)motoPrivari->enlarge));
					}
#endif				
				if(motoPrivari->countNow >= motoPrivari->posArrayNum - 1)	//轨迹到了最后一个点
				{
					if(motoPrivari->frequeNow>=motoPrivari->frequeDivision)	//定时器计数达到了更新位置点的时间周期
					motoPrivari->status = YS_lastTraj;
				}
					
				break;
			}
			default:	__YS_Clear(moto);	break;
		}
		osDelay(YS_OsDelayTime - 1);
	}
}

//数据包解析
typedef union pos
{
uint8_t arr[2];
uint16_t pos;
} pos;
pos pos1;

//重定义串口闲时中断回调函数
void YS_Moto_Data_UART_Callback(UART_HandleTypeDef *huart,const uint8_t *pdata,const uint16_t size)
{
	YS_Moto* moto;
	for(int i=0;i<YS_MOTO_NUM;i++)
	{
		if(motoList[i]->huart==huart)
		{
			moto=motoList[i];
			YS_Private* motoPrivari = moto->priVari;

			//if found the right moto,we don't need loop,this way to stop loop
			i=YS_MOTO_NUM;
			
			//数据头校核
			if((pdata[0]!=0XAA) ||
				(pdata[1]!=0X55) ||
				(pdata[2]!=0X11) )	{return;}

			//数据包接收错误，这句话本来是要有的，但一般不会出错，就不加了
//			if( pdata[21] != YS_CheckBit(pdata,21) ) return;

				
			static YS_ReturnData pos;
			pos.arr[0]	=pdata[9];	//低八位
			pos.arr[1]	=pdata[10];	//高八位
				
			MeanFliter(moto,(float)(pos.data)/(float)motoPrivari->enlarge);
			moto->YS_RefreshData_ISR_CallBack(moto,moto->fatherArgs);
				
#if YS_DEBUG
//			//传输数据格式：ID、目标步距位置、实际步距位置、目标步距速度、实际步距速度、控制电压		
//			static float oldPoint = 0;
//			static float newPoint = 0;
//			static float velAverage = 0;
//			static float velAim = 0;
//			static float posAim = 0;
//			if(moto->ID==0)
//			{
//				posAim=(float)(moto->priVari->aimStep)/(float)motoPrivari->enlarge;
//				newPoint = (float)(pos.data)/(float)motoPrivari->enlarge;				
//				velAim =(float)moto->priVari->aimStepVelo/(float)motoPrivari->enlarge;
//				velAverage = (newPoint - oldPoint)*1000.0/(float)(YS_ASK_TIME);
//				
//				// 1+ 6 * 4
//				static char str[25];
//				str[0]=0x34;
//				float data_f[6]={posAim,moto->nowPosi,velAim,moto->nowVelo,velAverage,(float)motoPrivari->giveVolt};
//				memcpy(&(str[1]),data_f,6*sizeof(float));
//				FOS_UART_Transmit_DMA(&huart1, (uint8_t *)str, sizeof(str));		
//				oldPoint =	newPoint;
//			}
					
//			//传输数据格式：ID、目标步距位置、实际步距位置、目标步距速度、实际步距速度、控制电压				
//			char str[38];
//			FOS_printf("ID:%d\tpt:%.3f\tpr:%.3f\tvt:%.3f\tvr:%.3f\tvolt:%d\r\n" , 
//						moto->ID,
//						(double)((float)motoPrivari->aimStep / (float)motoPrivari->enlarge),
//						(double)moto->nowPosi,						
//						(double)((float)motoPrivari->aimStepVelo/(float)motoPrivari->enlarge), 
//						(double)moto->nowVelo,
//						(int32_t)motoPrivari->giveVolt);	
#endif	
						
			switch(motoPrivari->status)
			{
				case YS_fastGoto:
				{
					if(-1*YS_Error < (pos.data - (int16_t)motoPrivari->aimStep) &&
						(pos.data - (int16_t)motoPrivari->aimStep) < YS_Error)
					{
						motoPrivari->status = YS_stop;
						moto->YS_P_drive_ISR_CallBack(moto,moto->fatherArgs);
					}	break;
				}
				case YS_goto:
				{
					if(-1*YS_Error < (pos.data - (int16_t)motoPrivari->aimStep) &&
						(pos.data - (int16_t)motoPrivari->aimStep) < YS_Error)
					{
						__YS_Stop(moto);
						moto->YS_P_drive_ISR_CallBack(moto,moto->fatherArgs);
					}	break;
				}
				case YS_home:
				{
					if(-1*YS_Error < (pos.data - (int16_t)motoPrivari->aimStep) &&
						(pos.data - (int16_t)motoPrivari->aimStep) < YS_Error)
					{
						__YS_Stop(moto);
						moto->YS_atHome_ISR_CallBack(moto,moto->fatherArgs);
					}	break;
				}
				case YS_lastTraj:
				{
					if(-1*YS_Error < (pos.data - (int16_t)motoPrivari->aimStep) &&
						(pos.data - (int16_t)motoPrivari->aimStep) < YS_Error)
					{
						__YS_Stop(moto);
						moto->YS_atTrajEnd_ISR_CallBack(moto,moto->fatherArgs);
					}	break;
				}
				default:break;
			}
		}		
	}
}

#if YS_DEBUG
#define YS_periodic (2.0)	//周期 5s

//调试用：让电机追踪一个正弦轨迹，然后输出目标位置和实际位置，通过对比调节比例系数
void YS_DebugTraj(YS_Moto *moto,float* traj,float a,float b)
{
	for(int i=0;i<YS_trajlong;i++)
	{
		traj[i]=a+b*sin(2.0*3.14/YS_periodic*(float)YS_ASK_TIME*20.0/1000.0*i);
	}	
	YS_FollowPos_Traj(moto,traj,YS_trajlong,YS_ASK_TIME*20,1);
}

void YS_SetPID(YS_Moto *moto,float kp,float ki,float kd,float IS)
{
	moto->pid_V.Kp=kp;
	moto->pid_V.Ki=ki;
	moto->pid_V.Kd=kd;	
	moto->pid_V.IS=IS;
}
#endif
