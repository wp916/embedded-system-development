#include "Fos_can.h"

#if CAN_DEBUG
#include "Fos_uart.h"
#endif

CAN_FilterTypeDef sFilterConfig;
CAN_RxHeaderTypeDef CAN_rxHeader; //CAN Bus Receive Header
uint8_t canRX[8] = {0,0,0,0,0,0,0,0};  //CAN Bus Receive Buffer
uint32_t canMailbox; //当数据发送完成时，返回是那个邮箱发送的数据

void* CAN_device[CAN1_Device_Num];
CAN_ReceiveDecodeFuc CAN_ReceivedDecode[CAN1_Device_Num];

/**
* @Brief    CAN1的滤波器配置，不过滤任何数据
* @Param	CAN_HandleTypeDef* hcan
* @Retval	None	*/
void my_can_filter_init_recv_all(void)
{
	
 
	 sFilterConfig.FilterActivation = ENABLE;//打开过滤器
	 sFilterConfig.FilterBank = 0;//过滤器0 这里可设0-13
	 sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;//采用掩码模式
	 sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;//采用32位掩码模式
	 sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;//采用FIFO0
	 
	 sFilterConfig.FilterIdHigh = 0x0000; //设置过滤器ID高16位
	 sFilterConfig.FilterIdLow = 0x0000;//设置过滤器ID低16位
	 sFilterConfig.FilterMaskIdHigh = 0x0000;//设置过滤器掩码高16位
	 sFilterConfig.FilterMaskIdLow = 0x0000;//设置过滤器掩码低16位

	if(HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)	Error_Handler();
	
	//Initialize CAN Bus
	if(HAL_CAN_Start(&hcan)!=HAL_OK)	Error_Handler();
	
	// Initialize CAN Bus Rx Interrupt
	if(HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK)	Error_Handler();
}

/**
* @brief	添加这条can总线上接受的ID值
* @param	ID:配置过滤器，使过滤器可以使这个ID通过
*/
uint8_t CAN_AddDecodeID(uint16_t ID)
{
	return 1;
}

/**
* @brief	为这条CAN总线绑定新的设备及其对应的处理函数
* @param	device：设备结构体
* @param	fuc：此设备返回数据的解析函数
*/
uint8_t CAN_AddDevice(void* device,CAN_ReceiveDecodeFuc fuc)
{
	static uint8_t device_Num=0;
	if (device_Num>=CAN1_Device_Num)	
	{
#if CAN_DEBUG
//			FOS_printf("Too much can device");
#endif		
		return 0;
	}

	CAN_device[device_Num]=device;
	CAN_ReceivedDecode[device_Num]=fuc;
	device_Num++;
	return 1;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
	HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &CAN_rxHeader, canRX); //Receive CAN bus message to canRX buffer
	
	for(int i=0;i<CAN1_Device_Num;i++)
	{
		if(CAN_ReceivedDecode[i](&CAN_rxHeader,canRX,CAN_device[i]))	return;
	}
}
