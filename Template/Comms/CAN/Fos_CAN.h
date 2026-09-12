#ifndef __FOS_CAN_H__
#define __FOS_CAN_H__

#include "can.h"

#define CAN1_Device_Num 1
#define CAN_DEBUG 1	//调试CAN总线，并输出到上位机，完成后建议关闭

/**
* @brief  绑定在这条can总线上的所有设备，其处理can总线上返回数据的函数的统一格式：
* @param  CAN_RxHeaderTypeDef* ID 将can总线上这个数据包的ID传入此函数
* @param  uint8_t* data 将can总线上接受到的数据传入此函数
* @param  void*	Can1_device[i] 处理函数内部自定参数，函数调用时输入为Can1_device
* @return NotMy_CAN_Data:表示这个函数不处理这个数据，还需继续寻找总线上的其他处理函数处理这组数据
		  IsMy_CAN_Data：这个函数处理这个数据，可以不用继续找其他函数处理了	*/
typedef uint8_t (*CAN_ReceiveDecodeFuc)(CAN_RxHeaderTypeDef* /*ID*/, uint8_t* /*data*/, void* /*Can1_device[i]*/);

typedef enum __CAN_ID_Match{
	IsMy_CAN_Data=1,
	NotMy_CAN_Data=0,
}CAN_ID_Match;

/**
* @Brief    CAN1的滤波器配置，不过滤任何数据
* @Retval	None	*/
void my_can_filter_init_recv_all(void);

/**
* @brief	添加这条can总线上接受的ID值
* @param	ID:配置过滤器，使过滤器可以使这个ID通过
*/
uint8_t CAN_AddDecodeID(uint16_t ID);

/**
* @brief	为这条CAN总线绑定新的设备及其对应的处理函数
* @param	device：设备结构体
* @param	fuc：此设备返回数据的解析函数
*/
uint8_t CAN_AddDevice(void* device,CAN_ReceiveDecodeFuc fuc);


#endif
