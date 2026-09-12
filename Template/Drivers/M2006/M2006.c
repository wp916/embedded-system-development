#include "M2006.h"
#include "cmsis_os2.h"	//系统级非阻塞延时
#include "malloc.h"		//动态内存管理

#if M2006_DEBUG
#include <string.h>
#include "math.h"
#endif

/******************主要控制逻辑**************************

1、DJ_SetVel，DJ_Goto等函数在调用时只是配置了运动模式及其控制参数，
	真正起作用是在主循环的 Switch (motoPriva->status) 里调用控制参数。
2、为了简化控制逻辑，速度环、电流环的目标值始终为 *followPoint 和 *followVel
	所指向的地址，在恒定位置或恒定速度控制模式下，将其指向一恒定值
	followPoint = &targetPoint		followVel = &targetVel
3、主循环如下：

				  +-----------------------+              
				  |     M2006反馈信号     |              
				  +-----------------------+              
							  |                          
							  V                          
			   N /-------------------------\             
	+------------|       达到接收间隔      |             
	|            \-------------------------/             
	|                         | Y                        
	|                         V                          
	|              +---------------------+               
	|              |     解析数据包      |               
	|              +---------------------+               
	|                         |                          
	|                         V                          
	|            /-------------------------\ N           
	|            |       达到发送间隔      |------------+
	|            \-------------------------/            |
	|                         | Y                       |
	|                         V                         |
	|               +-------------------+               |
	|               |      限位检查     |               |
	|               +-------------------+               |
	|                         |                         |
	|                         V                         |
	|   +-------------------------------------------+   |
	|   |        根据运动模式调整运动参数           |   |
	|   +-------------------------------------------+   |
	|                         |                         |
	|                         V                         |
	|              +---------------------+              |
	|              |     调整速度环      |              |
	|              +---------------------+              |
	|                         |                         |
	|                         V                         |
	|              +---------------------+              |
	|              |     调整电流环      |              |
	|              +---------------------+              |
	|                         |                         |
	|                         V                         |
	|            +-------------------------+            |
	|            |       发送控制指令      |            |
	|            +-------------------------+            |
	|                         |                         |
	|                         V                         |
	|                         O<------------------------+
	|                         |                          
	|                         V                          
	+------------------------>O       

*********************主要控制逻辑***************************/

//电机通信宏定义
#define M2006_FEEDBACK_ID	0x201	//请注意此值与  GM6020_FEEDBACK_ID  有冲突，M2006使用量超过4个时，要注意数据的接收
#define M2006_ID_BASE		0x200
#define M2006_ID_EXTEND		0x1FF	//请注意此值与  GM6020_ID_EXTEND  相同，M2006使用量超过4个时，要注意数据的发送
#define M2006_MAX_CURRENT	10000

CAN_TxHeaderTypeDef M2006_CAN_txHeader;	//can发送标识头
uint32_t M2006_CAN_mailbox;
uint8_t M2006_ID_List[M2006_NUM];		//每一位代表是否有电机需要控制，最多添加8个电机
int16_t M2006_Current[M2006_NUM];	//用于存储M2006的目标电流值
uint8_t M2006_BaseSend[8];	//向Can端口最终发送的指令 M2006 ID 1~4的的电机的发送电流值
#if M2006_NUM>4
uint8_t M2006_ExtendSend[8];	//向Can端口最终发送的指令 M2006 ID 5~8的的电机的发送电流值
#endif

typedef enum _M2006_Status{
	//单次指令类运动
	M2006_stop=0,	//停止就是控制电流为0
	
	//连续位置追踪	
	M2006_followFinitPosArray=1,	//跟随有限长度的位置序列
	M2006_followInfinitPosArray=2,	//无限的跟随一个值，这个值可能是传感器不断变化的值		
	M2006_lock=3,	//锁死是停在这个位置，会根据当前位置和此位置的差输出控制电流
	
	//连续速度追踪
	M2006_setVel=4,	//恒速运动，电机会一直依此速度运动
	M2006_goto=5,	//到达指定位置，到位后会触发 FinishGoto_ISR_CallBack 函数
	M2006_home=6,	//以恒定速度到达home处	
	M2006_followFinitVelArray=7,	//跟随一个有限的速度序列
	M2006_followInfinitVelArray=8,	//无限的跟随一个值，这个值可能是传感器不断变化的值	
}M2006_Status;

//M2006 的返回数据包的格式
typedef struct _M2006_data{
	int16_t angle;
	int16_t speed;
	int16_t torque;
	int16_t none;
}M2006_data;

//以联合体形式存储 M2006的返回数据
union M2006_Return{
	M2006_data data;
	uint8_t returnArray[8];
}m2006Return;

/**************Private variable declare************************/
typedef struct _M2006_Private{
	
	uint8_t ID;
	
	//为了节约系统资源，没必要以1KHz的频率处理数据，可以根据实际需求，间隔控制收发
	//发送周期，建议（10ms~100ms）根据实际情况决定，比如是按照位置序列运动，则可将控制周期设置为两位置间的运动时间间隔
	//如果是力位混合控制，则可设置到10ms，根据控制效果调节控制周期
	uint16_t sendPeriodCount,sendPeriodNow;
	//不要大于 sendPeriodCount，另外根据电机的实际运动速度计算，不要大于电机旋转半圈所用最短时间
	uint16_t recvPeriodCount,recvPeriodNow;
	
	//机械结构参数，true：直线运动机构（转一圈后位置不清零）；false；转动机构（转一圈后位置清零）
	uint8_t mechineArgs;
	float moveProportion;	//机械结构等导致的执行器位置于电机角度之间的放大关系
	float startMotoAngle;		//末端执行器的起始角度

	//1:设置了正负限位的IO口;0:没有设置正负限位的IO口
	uint8_t havePosGPIO,haveNegGPIO;
	
	float last_moto_angle;	//传感器上次测量时的电机角度
	float m_returnAngle;	//电机返回的原始电机角度数据
	int rountNum;	//电机旋转的圈数

	//PID参数(速度输入，电流输出)
	float kp;
	float ki;
	float kd;
	float IS;
	
	float err[2];   // error and last error
	float p_out;	//比例环节输出分量
	float i_out;	//积分环节输出分量
	float d_out;	//微分环节输出分量
	float output;

	uint8_t status;	//枚举量，只可设置为 M2006_Status 里的枚举值

	uint8_t lock;			//true：电机完成指定运动后，会保持在最终目标点；false：电机完成指定运动后，不再提供力矩
	
	int arrayLong;		//轨迹数组的长度
	uint8_t	dir;		//读取轨迹的数组的方向， 1：向后读；-1向前读
	int nowArrayNum;		//目前运动到轨迹数组的第几个数
	float tagetPoint;	//用于存储运动的目标到达的位置(直线运动机构)/角度（转动运动机构）
	float tagetVel;	//用于存储运动的目标速度

	float* followPoint;
	float* followVel;
}M2006_Private;

//一个空函数，防止callback函数指向野指针
void M2006_NULL(M2006* moto,void* fatherArgs) {}
	
//仅负责将电流控制指令发出，但在这之前，要设置好控制电流的具体值
void DJ_SendCMD(M2006* moto);

uint8_t M2006_Init(M2006*moto,uint8_t ID, uint16_t sendPeriod,uint16_t recvPeriod,
					M2006_PID_Parm pid,	uint8_t mechineArgs	, float moveProportion, float startPoint)
{	
	/****************************CAN总线初始化*******************************************/
	static uint8_t CAN_InitFlag=0;
	if(CAN_InitFlag==0)
	{
		M2006_CAN_txHeader.DLC=8;		// Number of bites to be transmitted max- 8
		M2006_CAN_txHeader.IDE = CAN_ID_STD;
		M2006_CAN_txHeader.RTR = CAN_RTR_DATA;
		M2006_CAN_txHeader.StdId = M2006_ID_BASE;
		M2006_CAN_txHeader.TransmitGlobalTime = DISABLE;
		
		for(int i=0;i<8;i++)	M2006_ID_List[i]=0;

		/*!!!!!!!!!!!!!!!!!这个的地方要修改一下过滤器!!!!!!!!!!!!!!!!!!!!!!!!!!*/
		my_can_filter_init_recv_all();
		CAN_InitFlag=1;
	}
		
	static int i=0;
	if (i>=M2006_NUM)	
	{
		static uint8_t info[]={"too much M2006 init!"};
		FOS_UART_Transmit_DMA(&huart1,info,sizeof(info));
		return 1;
	}

	/****************************绑定ID*******************************************/
	if (M2006_ID_List[ID - 1] == 1)	return 1;	//代表此端口已经被占，添加失败
	else { M2006_ID_List[ID - 1] = 1; }
	
	/****************************变量赋值*******************************************/			
	M2006_Private* motoPrivari=(M2006_Private*) mymalloc(SRAMIN,sizeof(M2006_Private));	
	moto->priVari = motoPrivari;
	
	motoPrivari->last_moto_angle=0;
	motoPrivari->rountNum = 0;	//电机旋转的圈数
	
	motoPrivari->err[0] = 0.0;   // error and last error
	motoPrivari->err[1] = 0.0;   // error and last error
	motoPrivari->p_out = 0.0;
	motoPrivari->i_out = 0.0;
	motoPrivari->d_out = 0.0;
	motoPrivari->output = 0.0;
	
	motoPrivari->status = M2006_stop;
		
	motoPrivari->followPoint = & (motoPrivari->tagetPoint);
	motoPrivari->followVel = & (motoPrivari->tagetVel);
	
	if(moto->RefreshData_ISR_CallBack == NULL) moto->RefreshData_ISR_CallBack=M2006_NULL;
	if(moto->FinishGoto_ISR_CallBack == NULL) moto->FinishGoto_ISR_CallBack=M2006_NULL;
	if(moto->FinishHome_ISR_CallBack == NULL) moto->FinishHome_ISR_CallBack=M2006_NULL;
	if(moto->FinishPos_Traj_ISR_CallBack == NULL) moto->FinishPos_Traj_ISR_CallBack=M2006_NULL;
	if(moto->FinishVel_Array_ISR_CallBack == NULL) moto->FinishVel_Array_ISR_CallBack=M2006_NULL;
	
	/****************************变量赋值*******************************************/
	motoPrivari->ID = ID;

	motoPrivari->sendPeriodCount=sendPeriod/recvPeriod;
	motoPrivari->recvPeriodCount=recvPeriod;
	motoPrivari->sendPeriodNow=0;
	motoPrivari->recvPeriodNow=0;
	
	if(moto->pos_GPIO==NULL)	motoPrivari->havePosGPIO=0;
	else motoPrivari->havePosGPIO=1;
	
	if(moto->neg_GPIO==NULL)	motoPrivari->haveNegGPIO=0;
	else motoPrivari->haveNegGPIO=1;
	
	motoPrivari->kp = pid.kp;
	motoPrivari->ki = pid.ki;
	motoPrivari->kd = pid.kd;
	motoPrivari->IS =pid.IS;
	
	motoPrivari->mechineArgs = mechineArgs;
	if(motoPrivari->moveProportion==0)	motoPrivari->moveProportion=1;	
	else{ motoPrivari->moveProportion = moveProportion;}

	motoPrivari->rountNum =  startPoint / motoPrivari->moveProportion * 36.0 / 360.0;
	motoPrivari->startMotoAngle = (startPoint / motoPrivari->moveProportion * 36.0) - motoPrivari->rountNum * 360.0 ;		
	motoPrivari->last_moto_angle=motoPrivari->startMotoAngle;
	
	i++;
	return 0;
}

//返回电机的ID编号
uint8_t inline DJ_Get_ID(M2006* moto)
{
	return moto->priVari->ID;
}

void DJ_Zero(M2006*moto)
{
	M2006_Private* motoPrivari = moto->priVari;
	motoPrivari->startMotoAngle = 360 - motoPrivari->m_returnAngle;
	motoPrivari->rountNum=-1;
}

void DJ_SET_PID(M2006*moto,M2006_PID_Parm pid)  
{
	M2006_Private* motoPrivari = moto->priVari;
	motoPrivari->kp = pid.kp;
	motoPrivari->ki = pid.ki;
	motoPrivari->kd = pid.kd;
	motoPrivari->IS = pid.IS;
}

/**
* @brief  !末端执行器!以指定速度运动
* @param  vel：直线运动机构中输入的是末端执行器速度，转动机构中输入的是末端执行器角速度
			电机能达到的速度最大值取决于负载，调速灵敏度取决于负载和pid参数，其底层控制的是电流环
			正负值代表运动方向。
* @return */
uint8_t DJ_SetVel(M2006*moto,float vel)
{
	M2006_Private* motoPrivari = moto->priVari;
	motoPrivari->status = M2006_setVel;
	motoPrivari->tagetVel=vel;

	motoPrivari->followVel = &(motoPrivari->tagetVel);
	return 1;
}

/**
* @brief  !末端执行器!以指定vel速度运动到pos位置
* @notice 注意，这个速度只能是正值，方向由当前位置和目标位置的差决定*/
uint8_t DJ_Goto(M2006*moto,float pos, float vel, uint8_t lock)
{	
	M2006_Private* motoPrivari = moto->priVari;
	motoPrivari->status = M2006_goto;
	motoPrivari->tagetPoint=pos;
	motoPrivari->followPoint=&(motoPrivari->tagetPoint);
	
	if(pos<moto->point)	motoPrivari->tagetVel=-1*vel;
	else	motoPrivari->tagetVel=vel;	
	motoPrivari->followVel = &(motoPrivari->tagetVel);
	
	motoPrivari->lock=lock;

	return 1;
}

uint8_t DJ_Home(M2006*moto,int8_t dir, uint8_t lock)
{
	M2006_Private* motoPrivari = moto->priVari;
	motoPrivari->status = M2006_home;
	motoPrivari->lock =lock;
	if(HAL_GPIO_ReadPin(moto->home_GPIO,moto->home_GPIO_Pin)==moto->home_Triggered)
	{
		DJ_Zero(moto);
		DJ_Goto(moto,moto->point,0,lock);
		moto->FinishHome_ISR_CallBack(moto,moto->fatherArgs);
		return 1;
	}		
	
	motoPrivari->tagetVel=dir * 20 * motoPrivari->moveProportion;
	motoPrivari->followVel = &(motoPrivari->tagetVel);

	return 1;
}

uint8_t DJ_FollowPos_Traj(M2006*moto,float* Pos, int arrayLong, uint8_t dir, uint8_t lock)
{
	M2006_Private* motoPrivari = moto->priVari;
	motoPrivari->status =M2006_followFinitPosArray;
	motoPrivari->followPoint = Pos;
	motoPrivari->dir=dir;
	if(dir == 1)	motoPrivari->tagetPoint = Pos[arrayLong - 1];
	else motoPrivari->tagetPoint = *(Pos -= (arrayLong - 1));
	motoPrivari->tagetVel = (Pos[arrayLong - 1] - moto->point) / (motoPrivari->sendPeriodCount * motoPrivari->recvPeriodCount / 1000.0);
	motoPrivari->followVel = &(motoPrivari->tagetVel);
	motoPrivari->tagetPoint = Pos[arrayLong - 1];
	motoPrivari->arrayLong = arrayLong;
	motoPrivari->nowArrayNum = 0;
	motoPrivari->lock = lock;
	
	//因为执行此函数前，默认其他程序已控制电机达到起始位置， 因此调用此函数即可立即向第二个位置运动
	motoPrivari->sendPeriodNow = motoPrivari->sendPeriodCount;	
	return 1;
}

uint8_t DJ_FollowPos_Senser(M2006*moto,float* Pos)
{
	M2006_Private* motoPrivari = moto->priVari;
	motoPrivari->status = M2006_followInfinitPosArray;
	motoPrivari->followPoint = Pos;
	motoPrivari->tagetVel = (*Pos - moto->point) / (motoPrivari->sendPeriodCount * motoPrivari->recvPeriodCount / 1000.0);
	motoPrivari->followVel = &(motoPrivari->tagetVel);
	return 1;
}

uint8_t DJ_FollowVel_Array(M2006*moto,float* vel, int arrayLong, uint8_t dir, uint8_t lock)
{
	M2006_Private* motoPrivari = moto->priVari;
	motoPrivari->status = M2006_followFinitVelArray;
	motoPrivari->followVel = vel;
	motoPrivari->arrayLong = arrayLong;
	motoPrivari->nowArrayNum = 0;
	motoPrivari->lock = lock;
	return 1;
}

uint8_t DJ_FollowVel_Senser(M2006*moto,float* vel)
{
	M2006_Private* motoPrivari = moto->priVari;
	motoPrivari->status = M2006_followInfinitVelArray;
	motoPrivari->followVel = vel;
	return 1;
}

void DJ_Stop(M2006*moto)
{
	M2006_Private* motoPrivari = moto->priVari;
	motoPrivari->status = M2006_stop;
	M2006_Current[motoPrivari->ID - 1]=0;
	DJ_SendCMD(moto);
}

void DJ_Lock(M2006*moto)
{
	M2006_Private* motoPrivari = moto->priVari;	
	motoPrivari->tagetPoint = moto->point;
	motoPrivari->followPoint= &(motoPrivari->tagetPoint);
	motoPrivari->followVel= &(motoPrivari->tagetVel);
	motoPrivari->status = M2006_lock;
}

void DJ_RefreshData_ISR_DisCallBack(M2006*moto) {moto->RefreshData_ISR_CallBack=M2006_NULL;}
void DJ_FinishHome_ISR_DisCallBack(M2006*moto) {moto->FinishHome_ISR_CallBack=M2006_NULL;}
void DJ_FinishGoto_ISR_DisCallBack(M2006*moto) {moto->FinishGoto_ISR_CallBack=M2006_NULL;}
void DJ_FinishPos_Traj_ISR_DisCallBack(M2006*moto) {moto->FinishPos_Traj_ISR_CallBack=M2006_NULL;}
void DJ_FinishVel_Array_ISR_DisCallBack(M2006*moto) {moto->FinishVel_Array_ISR_CallBack=M2006_NULL;}


/***********************************Private Fuction********************************************/
/******Private fuction declare********/
float M2006_PID(M2006*moto,float vel);

//如果x在min到max的闭区间，返回x，否则返回极限值
float LimMinMax(float x, float min, float max);

uint8_t M2006_CanDataDecode(CAN_RxHeaderTypeDef* data_handle,uint8_t* data,void* _moto)
{	
	if((data_handle->StdId <= M2006_ID_BASE) || (M2006_ID_BASE + M2006_NUM <data_handle->StdId))
	{	return NotMy_CAN_Data;	}	
	
	M2006* moto= _moto;
	M2006_Private* motoPrivari = moto->priVari;
	
	if(motoPrivari->ID != (data_handle->StdId - M2006_ID_BASE))	{	return NotMy_CAN_Data;	}	
	
	if(motoPrivari->recvPeriodNow < motoPrivari->recvPeriodCount)
	{
		motoPrivari->recvPeriodNow++;
		return IsMy_CAN_Data;
	}	
	motoPrivari->recvPeriodNow=1;
	
	/*************************开始解析数据包***********************************************/
	m2006Return.returnArray[0]=data[1];	
	m2006Return.returnArray[1]=data[0];	
	m2006Return.returnArray[2]=data[3];	
	m2006Return.returnArray[3]=data[2];	
	m2006Return.returnArray[4]=data[5];	
	m2006Return.returnArray[5]=data[4];	

	
	/****************************************************这段适合 M2006 一类有减速器的电机***********************************************************/
	motoPrivari->m_returnAngle = 0.04395 * m2006Return.data.angle;	// 0.04395 = 360 / 8191
	// 1、得到电机的角度，存于m_ang，并将其叠加起始角度值，角度范围限制在0~360度
	float m_ang=motoPrivari->m_returnAngle + motoPrivari->startMotoAngle;
	if(m_ang>360.0)	{ m_ang-=360.0; }
	moto->moto_angle = m_ang;
	
	//2、处理跨0°跳变，实际效果是当速度为正，且电机角度（m_ang）从350°左右，突变为10°时，allAngle这个值不突变（速度为负类似效果）
	int round = 0;
	if ((motoPrivari->last_moto_angle > 180.0) && (m_ang < (motoPrivari->last_moto_angle - 180.0))) { motoPrivari->rountNum++;	round = 1; }
	if ((motoPrivari->last_moto_angle < 180.0) && (m_ang > (motoPrivari->last_moto_angle + 180.0))) { motoPrivari->rountNum--;	round = -1; }		
	float allAngle = motoPrivari->rountNum * 360.0 + m_ang;
	
	//3、分别计算末端执行器位置
	if (motoPrivari->mechineArgs) 
	{ 
		//3.1 直线末端执行器的位置
		moto->point = allAngle / (36.0 / motoPrivari->moveProportion); 
	}
	else
	{
		//3.2 旋转末端执行器的位置（角度限制在0~360°）
		float enlarge = 36.0 / motoPrivari->moveProportion * 360.0;
		int remain = allAngle / enlarge;
		float ang = (allAngle - remain * enlarge) / (36.0 / motoPrivari->moveProportion);
		if (ang < 0) { ang += 360.0; }
		moto->point = ang;
	}
	
	moto->moto_speed =m2006Return.data.speed * 6.0;	// 1rpm = 6°/s (他这里说明书上有问题，反馈值是电机经过减速器（36:1）后,输出轴的转速)
	moto->torque = m2006Return.data.torque;
	
	
	moto->speed = (float)motoPrivari->moveProportion / 36.0 * moto->moto_speed;
	moto->moto_speed_ave = (m_ang + round * 360.0 - motoPrivari->last_moto_angle) / 36.0 / (float)(motoPrivari->recvPeriodCount)*1000.0;
	
	//这个方法的 moto_speed_ave 是用当前位置减去上个位置的差 除以1个时间间隔得到的
	moto->speed_ave = motoPrivari->moveProportion * moto->moto_speed_ave;
	
	motoPrivari->last_moto_angle = m_ang;	
	moto->RefreshData_ISR_CallBack(moto,moto->fatherArgs);
	/****************************************************这段适合 M2006 一类有减速器的电机***********************************************************/
	
	/******************************判断及发送控制指令***********************************************/

	//限位检查
	if(motoPrivari->havePosGPIO)
	{
		if(HAL_GPIO_ReadPin(moto->pos_GPIO,moto->pos_GPIO_Pin)==moto->pos_Triggered)	
		{
			DJ_Stop(moto);
#if M2006_DEBUG
			FOS_printf("M2006 ID %d positive limit has triggered",motoPrivari->ID);
#endif
			return IsMy_CAN_Data;
		}
	}
	if(motoPrivari->haveNegGPIO)
	{
		if(HAL_GPIO_ReadPin(moto->neg_GPIO,moto->neg_GPIO_Pin)==moto->neg_Triggered)	
		{
			DJ_Stop(moto);
#if M2006_DEBUG
			FOS_printf("M2006 ID %d negitive limit has triggered",motoPrivari->ID);
#endif
			return IsMy_CAN_Data;
		}
	}
	
	//根据运动状态做相应控制
	switch(motoPrivari->status)
	{
		case M2006_stop:	break;
		case M2006_lock:	break;
		case M2006_goto:
			//到达指定位置
			if( ((motoPrivari->tagetVel >= 0) && (moto->point >= motoPrivari->tagetPoint))	||
				((motoPrivari->tagetVel <  0) && (moto->point <= motoPrivari->tagetPoint)))
			/*****************需要修正到达 0 或 360 附近位置的问题***************************/
			{
				if(motoPrivari->lock==1) { motoPrivari->status=M2006_lock; }
				else{ motoPrivari->status=M2006_stop; }
				moto->FinishGoto_ISR_CallBack(moto,moto->fatherArgs);
			}
			break;
		case M2006_home:
			if(HAL_GPIO_ReadPin(moto->home_GPIO,moto->home_GPIO_Pin)==moto->home_Triggered)
			{
				if(motoPrivari->lock==1)	
				{
					DJ_Zero(moto);
					motoPrivari->tagetPoint =0  /* moto->point */ ;
					motoPrivari->followPoint = &(motoPrivari->tagetPoint);
					motoPrivari->followVel= &(motoPrivari->tagetVel);
					motoPrivari->status=M2006_lock; 
				}
				else
				{
					DJ_Zero(moto);
					motoPrivari->status=M2006_stop;				
				}
				moto->FinishHome_ISR_CallBack(moto,moto->fatherArgs);
			}
			break;
		case M2006_followFinitPosArray:		//有限位置跟随
			if(motoPrivari->sendPeriodNow < motoPrivari->sendPeriodCount)	{motoPrivari->sendPeriodNow++;}
			else
			{
				motoPrivari->sendPeriodNow=1;
				if (motoPrivari->nowArrayNum < motoPrivari->arrayLong - 1)	
				{
					motoPrivari->nowArrayNum++;
					motoPrivari->followPoint += motoPrivari->dir;			
				}
				else
				{
					motoPrivari->followPoint=&(motoPrivari->tagetPoint);					
					if(motoPrivari->lock==1)	{	motoPrivari->status=M2006_lock;	}
					else{ motoPrivari->status=M2006_stop; }					
					moto->FinishPos_Traj_ISR_CallBack(moto,moto->fatherArgs);				
				}
			}
			break;
		case M2006_followFinitVelArray:		//有限速度序列跟随
			if (motoPrivari->nowArrayNum < motoPrivari->arrayLong)	
			{
				motoPrivari->nowArrayNum++;
				motoPrivari->followVel += motoPrivari->dir;
			}
			else
			{				
				if(motoPrivari->lock==1)	
				{
					motoPrivari->tagetPoint=moto->point;
					motoPrivari->followPoint=&(motoPrivari->tagetPoint);
					
					motoPrivari->followVel= &(motoPrivari->tagetVel);
					motoPrivari->status=M2006_lock;
				}
				else{ motoPrivari->status=M2006_stop; }
				moto->FinishVel_Array_ISR_CallBack(moto,moto->fatherArgs);	
				break;
			}	
		default:break;
	}
	
	if(motoPrivari->status == M2006_stop)	{return IsMy_CAN_Data;}
	
	//位置追踪模式：调整速度环
	if(	( motoPrivari->status >= M2006_followFinitPosArray ) &&
		( motoPrivari->status <= M2006_lock)	)
	{	motoPrivari->tagetVel = ((*(motoPrivari->followPoint)) - moto->point) * 1000.0
				/(float)(motoPrivari->sendPeriodCount * motoPrivari->recvPeriodCount); }
	
	//调整电流环
	{M2006_Current[motoPrivari->ID - 1] = (int16_t)LimMinMax(M2006_PID(moto,*(moto->priVari->followVel)) , -10000.0 , 10000.0);}
	
	DJ_SendCMD(moto);
	
#if M2006_DEBUG
//	FOS_printf("pt:%.3f pr:%.3f vt:%.3f vr:%.3f vra:%.3f ct:%d\r\n",
//				(double)*(motoPrivari->followPoint),
//				(double)moto->point,
//				(double)*(motoPrivari->followVel),
//				(double)moto->speed,
//				(double)moto->speed_ave,
//				(int32_t)M2006_Current[motoPrivari->ID - 1]);
//	FOS_printf("%.3f\t%.3f\t%.3f\t%d\r\n",
//				(double)*(motoPrivari->followVel),
//				(double)moto->speed,
//				(double)moto->speed_ave,
//				(int32_t)M2006_Current[motoPrivari->ID - 1]);

//	FOS_printf("p:%.3f ma:%.3f v:%.3f va:%.3f mv:%.3f mva:%.3f \r\n",
//				(double)moto->point,
//				(double)moto->moto_angle,
//				(double)moto->speed,
//				(double)moto->speed_ave,
//				(double)moto->moto_speed,
//				(double)moto->moto_speed_ave);

	//传输数据格式：ID、目标步距位置、实际步距位置、目标步距速度、实际步距速度、控制电压		
	static float newPoint = 0;
	static float velAverage = 0;
	static float velAim = 0;
	static float posAim = 0;
	newPoint =moto->point;
	velAverage = moto->speed_ave;
	posAim=(*(motoPrivari->followPoint));
	velAim =(*(moto->priVari->followVel));
	// 1+ 6 * 4
	static char str[25];
	str[0]=0x34;
	float data_f[6]={posAim,newPoint,velAim,moto->speed,velAverage,(float)M2006_Current[motoPrivari->ID - 1]};
	memcpy(&(str[1]),data_f,6*sizeof(float));
	FOS_UART_Transmit(&huart1, (uint8_t *)str, sizeof(str),5);	
#endif	
	
	return IsMy_CAN_Data;
}

//如果x在min到max的闭区间，返回x，否则返回极限值
float LimMinMax(float x, float min, float max)
{
	return ((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x));
}

float M2006_PID(M2006*moto,float vel)
{
	M2006_Private* motoPrivari = moto->priVari;
	motoPrivari->err[1] = motoPrivari->err[0];
	motoPrivari->err[0] = (vel - moto->speed) / motoPrivari->moveProportion * 36.0;

	motoPrivari->p_out = motoPrivari->kp * motoPrivari->err[0];
	motoPrivari->i_out += motoPrivari->ki * motoPrivari->err[0];	
	motoPrivari->d_out = motoPrivari->kd * (motoPrivari->err[0] - motoPrivari->err[1]);
	motoPrivari->i_out =LimMinMax(motoPrivari->i_out,-1.0*motoPrivari->IS,motoPrivari->IS);	//限制饱和积分
	
	motoPrivari->output = motoPrivari->p_out + motoPrivari->i_out + motoPrivari->d_out;

	return motoPrivari->output;
}

//仅负责将电流控制指令发出，但在这之前，要设置好控制电流的具体值
void DJ_SendCMD(M2006* moto)
{
	M2006_Private* motoPrivari = moto->priVari;
	uint8_t i=motoPrivari->ID - 1;	

#if M2006_NUM>4	
	if(i<4)	{
#endif
		M2006_CAN_txHeader.StdId=M2006_ID_BASE;
		uint16_t current=M2006_Current[i];
		M2006_BaseSend[2 * i] = (current >> 8) & 0xff;
		M2006_BaseSend[2 * i + 1] = (current) & 0xff;

		/**************************检查是否邮箱空**************************************/
//		uint8_t can=CAN_TX_MAILBOX0;
		HAL_CAN_AddTxMessage(&M2006_CanHandle,&M2006_CAN_txHeader,M2006_BaseSend,&M2006_CAN_mailbox); // Send Message
#if M2006_NUM>4	
	}else{
		M2006_CAN_txHeader.StdId=M2006_ID_EXTEND;
		M2006_ExtendSend[2 * i-8] = (M2006_Current[i] >> 8) & 0xff;
		M2006_ExtendSend[2 * i - 7] = (M2006_Current[i]) & 0xff;
		
		/**************************检查是否邮箱空**************************************/
//		uint8_t can=CAN_TX_MAILBOX0;
		HAL_CAN_AddTxMessage(&M2006_CanHandle,&M2006_CAN_txHeader,M2006_ExtendSend,&canMailbox); // Send Message
	}
#endif
}


#if M2006_DEBUG
#define M2006_periodic 4.7	//周期
//调试用：让电机追踪一个正弦轨迹，然后输出目标位置和实际位置，通过对比调节比例系数
void M2006_DebugTraj(M2006 *moto,float* traj,uint16_t a,uint16_t b)
{
	M2006_Private* motoPrivari = moto->priVari;
	float timeInterval = motoPrivari->sendPeriodCount * motoPrivari->recvPeriodCount;
	for(int i=0;i<M2006_trajlong;i++)
	{
		traj[i]=a+b*sin(2*3.14/M2006_periodic*timeInterval/1000.0*i);
	}	
	DJ_FollowPos_Traj(moto,traj,M2006_trajlong,1,1);
}
#endif
