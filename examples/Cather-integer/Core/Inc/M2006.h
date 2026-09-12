/*******************************************************************************
CubeMX 配置
1、参考 Fos_Can.h 中的配置要求
2、参考 Fos_Uart.h 中的配置要求

Keil MDK配置	开启动态内存管理机制
Options for target->Target->Use Micro LIB 勾选
*******************************************************************************/

#ifndef __M2006_H__
#define __M2006_H__

#include "Fos_Can.h"	//通信基础依赖
#include "Fos_Uart.h"	//与上位机交互，发送提示信息

/*
提醒：一个潜在的硬件bug，由于每个电机都默认以1KHz的频率向总线上发送实时数据，但总线上的数据缓冲区不是无限的
因此，有一种可能性，就是当多个电机同时上电时，他们极其巧合的同时向can总线上发送数据，这时缓冲区溢出，数据丢失，
我的解决思路就是通过软件调节，去容忍这种数据丢失，具体思路如下：
电机的数据发送间隔在1ms，而我们的控制周期 M2006_SEND_PERIOD 在10ms~100ms之间，这里面有很大的容错空间，
比如设置控制周期为50ms，即使丢包了，也只是控制周期在48~52ms之间波动（因为不会老是一个电机丢包，
他肯定是多个电机，随机丢包），因此，按照一个大致的控制周期做控制，其实也不会有太大的误差。
*/

/*
关于位置和速度单位的说明：
由于M2006的控制参数是电机的电流，它通过 电机目标转速->PID->电机控制电流 实现，因此，PID参数和电机目标转速单位匹配即可
但我们的电机控制函数的位置/速度，输入都是"末端执行器"的位置和速度，因此，这里还有个 末端执行器位置/速度 -> 电机位置/速度的转换
这个转换的基本逻辑是：
	末端执行器位移 = moveProportion * 电机经自带36:1减速器减速后的输出转角位移
	直线运动机构：（mechineArgs=1）
		(电机位置 + startAngle + 360*roundNum ) * moveProportion = 末端执行器位置
	旋转运动机构：（mechineArgs=0）
	   [(电机位置 + startAngle + 360*roundNum ) * moveProportion ] % 360 = 末端执行器位置
其中 moveProportion为机械结构导致的执行器位移与电机转角之间的比例关系
原则上，这些变量的单位都统一就可以，但要求速度的时间单位为 s
*/

#define M2006_DEBUG 0		//调试电机PID参数，并输出到上位机，完成后建议关闭，使用时建议单电机调试

#define M2006_NUM 1 	//驱动M2006的总数，一定要根据实际情况修改,ID号要从1开始，连续排列!

//M2006 绑定在那条Can总线上（某些32上会有多条Can总线）
//根据 Can.h 文件里声明的 CAN_HandleTypeDef 实例修改
#define M2006_CanHandle hcan

//M2006 的发送邮箱为那个（根据硬件配置合理分配）
//根据 stm32f1xx_hal_can.h 文件里声明的 CAN_TX_MAILBOX0 定义
#define M2006_CanTxMail CAN_TX_MAILBOX0


typedef struct
{
	float kp;
	float ki;
	float kd;
	float IS;		//Integralsaturation 饱和积分，限制低速时的运动震动
}M2006_PID_Parm;

//M2006电机受保护的变量,不应在M2006.c以外任何地方使用
typedef struct _M2006_Private M2006_Private;
typedef struct _M2006 M2006;
typedef void (*M2006_CallBack)(M2006* /*moto*/,void* /*fatherArgs*/);
typedef struct _M2006{
	
	M2006_Private* priVari;	//内部变量，不允许在M2006.c以外的文件中调用修改
	
	//正负限位和home位置的IO端口
	GPIO_TypeDef *pos_GPIO;
	uint16_t pos_GPIO_Pin;	
	GPIO_TypeDef *neg_GPIO;
	uint16_t neg_GPIO_Pin;
	GPIO_TypeDef *home_GPIO;
	uint16_t home_GPIO_Pin;
	
	//枚举值:{GPIO_PIN_RESET、GPIO_PIN_SET}
	//设定什么电平代表光电门触发了
	GPIO_PinState pos_Triggered;
	GPIO_PinState neg_Triggered;
	GPIO_PinState home_Triggered;
	
	//只读量，会随新数据包刷新自动修改
	//当前末端执行器位置(直线运动机构)/角度(旋转运动机构)值
	float point;
	//当前电机角度值
	float moto_angle;
	
	//当前!末端执行器!运动速度(moveProportion*moto_speed)
	//当电机速度过小（1rpm以下，就不要用这个测速了，用speed_ave更好）
	float speed;
	//通过两次电机的位置除时间求出当前!末端执行器!运动速度(moveProportion*moto_speed_ave)
	//当电机速度过小（1rpm以下时使用）
	float speed_ave;
	//当前!电机!角速度值（单位°/s）
	//当电机速度过小（1rpm以下，就不要用这个测速了，用moto_speed_ave更好）
	int16_t moto_speed;
	//通过两次!电机!的位置除时间求出速度(单位：°/s)
	//当电机速度过小（1rpm以下时使用）
	float moto_speed_ave;
	int16_t torque;	//当前力矩
	
	//完成指定运动后会执行一次此回调函数
	//第一个 void 为输入参数，为函数输入到位电机
	//第二个 void 为自由参数，当回调函数中需要处理某些数据时，由此参数传入
	void* fatherArgs;		//回调函数的输入参数永远是这个，可以根据需要绑定
	void (*RefreshData_ISR_CallBack)(M2006* /*moto*/,void* /*fatherArgs*/);	//电机更新数据后会调用一次这个函数
	void (*FinishHome_ISR_CallBack)(M2006* /*moto*/,void* /*fatherArgs*/);
	void (*FinishGoto_ISR_CallBack)(M2006* /*moto*/,void* /*fatherArgs*/);
	void (*FinishPos_Traj_ISR_CallBack)(M2006* /*moto*/,void* /*fatherArgs*/);
	void (*FinishVel_Array_ISR_CallBack)(M2006* /*moto*/,void* /*fatherArgs*/);
	
}M2006;

/**
* @brief  初始化M2006
* @param  moto：要初始化的电机
* @param  ID：CAN总线上的ID编号，不要重复，四个M2006 范围（1~4），八个M2006 （5~8）
* @param  sendPeriod: 必须是整数,必须是recvPeriod的整倍数，单位 ms	发送周期，建议（10ms~100ms）根据实际情况决定，比如是按照位置序列运动，
			则可将控制周期设置为两位置间的运动时间间隔，如果是力位混合控制，则可设置到10ms，根据控制效果调节控制周期
* @param  recvPeriod: 必须是整数，单位 ms。根据电机的实际运动速度计算，当电机以最大转速运动时，也不要让他在一个周期内旋转超过180°
* @param  pid：pid控制参数，这个pid的控制方法有所改进，请参考控制框图
* @param  mechineArgs:机械结构参数，true：直线运动机构（转一圈后位置不清零）；false；转动机构（转一圈后位置清零）
* @param  moveProportion:机械结构导致的执行器位移与电机转角之间的比例关系( 执行器位移=moveProportion * 电机经自带36:1减速器减速后的输出转角位移 )
* @param  startPoint:电机编码器起始角度永远为0°，末端执行器初始位置/角度默认为0，如果在你的坐标系内，
			末端执行器的初始位置不为0，可在此设置末端执行器的位置 (默认为0)
* @return 0:初始化成功；1：电机ID编号冲突	*/
uint8_t M2006_Init(M2006* moto,uint8_t ID, uint16_t sendPeriod,uint16_t recvPeriod,					
					M2006_PID_Parm pid,	uint8_t mechineArgs	, float moveProportion, float startPoint);

/**
* @brief  返回电机的ID编号	*/
uint8_t inline DJ_Get_ID(M2006* moto);

/**
* @brief  将当前位置的坐标值设为0	*/
void DJ_Zero(M2006*moto);

/**
* @brief  修改PID1的参数  */
void DJ_SET_PID(M2006*moto,M2006_PID_Parm pid);

/**
* @brief  !末端执行器!以指定速度运动
* @notice 注意，这个速度只能是正值，方向由当前位置和目标位置的差决定
* @param  vel：直线运动机构中输入的是末端执行器速度，转动机构中输入的是末端执行器角速度
			电机能达到的速度最大值取决于负载，调速灵敏度取决于负载和pid参数，其底层控制的是电流环
			正负值代表运动方向。
* @return */
uint8_t DJ_SetVel(M2006*moto,float vel);

/**
* @brief  !末端执行器!到达指定的位置
* @notice 在旋转机构中，不要设成到 0 或360 位置，因为旋转机构的位置范围就是 0~360，很难停到这个位置
* @param  pos:直线运动机构中输入的是位置，转动机构中输入的是角度
* @param  vel: !末端执行器! 的运动速度
* @param  lock:到达指定位置后，true:电机仍有驱动力，外力不可扭动电机；false：电机不再有驱动力，外力可扭动电机
* @return */
uint8_t DJ_Goto(M2006*moto,float pos, float vel, uint8_t lock);

/**
* @brief  借用外部光电门定位原点,定位完成后位置会清零
* @param  dir：1：正方向寻零；-1：负方向寻零
* @return */
uint8_t DJ_Home(M2006*moto,int8_t dir , uint8_t lock);

/**
* @brief  跟随指定位置，这个量可以是提前规划好的轨迹
* @param  Pos:直线运动机构中输入的是位置，转动机构中输入的是角度
			注意！！要确保pos数组在电机运动过程中一直实际存在
* @param  arrayLong:Pos数组的长度，用于确定停止位置
* @param  dir:Pos数组的读取方向，1：向后读取位置，-1：向前读取位置
* @param  lock:到达指定位置后，true:电机仍有驱动力，外力不可扭动电机；false：电机不再有驱动力，外力可扭动电机
* @return */
uint8_t DJ_FollowPos_Traj(M2006*moto,float* Pos, int arrayLong, uint8_t dir, uint8_t lock);

/**
* @brief  跟随指定位置，这个量可以是其他传感器的实时输入数据
* @param  Pos:直线运动机构中输入的是位置，转动机构中输入的是角度
* @return */
uint8_t DJ_FollowPos_Senser(M2006*moto,float* Pos);

/**
* @brief  跟随指定速度，这个量可以是提前规划好的速度数组
* @param  vel:速度数组的第一个量
* @param  arrayLong:vel数组的长度，用于确定停止位置
* @param  dir:Pos数组的读取方向，1：向后读取位置，-1：向前读取位置
* @param  lock:到达指定位置后，true:电机仍有驱动力，外力不可扭动电机；false：电机不再有驱动力，外力可扭动电机
* @return */
uint8_t DJ_FollowVel_Array(M2006*moto,float* vel, int arrayLong, uint8_t dir, uint8_t lock);

/**
* @brief  跟随指定位置，这个量可以是其他传感器的实时输入数据
* @param  vel:会不断变化的速度值，一般来自传感器数据
			注意！！要确保vel数组在电机运动过程中一直实际存在
* @return */
uint8_t DJ_FollowVel_Senser(M2006*moto,float* vel);

/**
* @brief  停止运动	*/
void DJ_Stop(M2006*moto);	
	
/**
* @brief  锁定在当前位置	*/
void DJ_Lock(M2006*moto);
	
/**
* @brief  解除电机锁定	*/
#define DJ_Unlock	DJ_Stop
	
/**
* @brief  解析电机返回数据	*/
uint8_t M2006_CanDataDecode(CAN_RxHeaderTypeDef* data_handle,uint8_t* data,void* moto);

//解绑回调函数
void DJ_RefreshData_ISR_DisCallBack(M2006*moto);
void DJ_FinishHome_ISR_DisCallBack(M2006*moto);
void DJ_FinishGoto_ISR_DisCallBack(M2006*moto);
void DJ_FinishPos_Traj_ISR_DisCallBack(M2006*moto);
void DJ_FinishVel_Array_ISR_DisCallBack(M2006*moto);


#if M2006_DEBUG
#define M2006_trajlong 200	
//调试用：让电机追踪一个正弦( a+b*sin(wt) )轨迹，然后输出目标位置和实际位置，通过对比调节比例系数
void M2006_DebugTraj(M2006 *moto,float* traj,uint16_t a,uint16_t b);


#endif


#endif // !__ISP_MOTO_DRIVER_H__
