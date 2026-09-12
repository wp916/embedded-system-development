#include "real_main.h"
#include "main.h"
#include "key.h"


void UART_Error_CallBack(UART_HandleTypeDef *huart);
uint8_t Uart_ReceiveCMD(uint8_t* Controlflag, uint16_t Size, void* Scalpel);
//void KEY_Back_Clicked(void* key)
//{
//	static uint8_t info[]={"KEY_Back_Clicked\r\n"};
//	FOS_UART_Transmit_DMA(&huart1,info,sizeof(info));
//	Locate();
//}

void KEY_Back_Push(void* key)
{
	//一个安全检测，如果电机位置信息为0，可能是电机通信接收有问题
	if((wireMoto.nowPosi==0.0) || (gripperMoto.nowPosi==0.0))
	{
		static uint8_t info[]={"Moto might error, better reload!\r\n"};
		FOS_UART_Transmit_DMA(&huart1,info,sizeof(info));
		return;
	}
	static uint8_t info[]={"KEY_Back_Push\r\n"};
	FOS_UART_Transmit_DMA(&huart1,info,sizeof(info));
	Store();
}

void KEY_Back_Release(void* key)
{
	static uint8_t info[]={"KEY_Back_Release\r\n"};
	FOS_UART_Transmit_DMA(&huart1,info,sizeof(info));
	ScalpelStop();
}

void KEY_Forword_Push(void* key)
{
	//一个安全检测，如果电机位置信息为0，可能是电机通信接收有问题
	if((wireMoto.nowPosi==0.0) || (gripperMoto.nowPosi==0.0))
	{
		static uint8_t info[]={"Moto might error, better reload!\r\n"};
		FOS_UART_Transmit_DMA(&huart1,info,sizeof(info));
		return;
	}
	static uint8_t info[]={"KEY_Forword_Push\r\n"};
	FOS_UART_Transmit_DMA(&huart1,info,sizeof(info));
	Stretch();
}

void KEY_Forword_Release(void* key)
{
	static uint8_t info[]={"KEY_Forword_Release\r\n"};
	FOS_UART_Transmit_DMA(&huart1,info,sizeof(info));
	ScalpelStop();
}

keyHandle KEY_Back={
	//.Clicked_CallBack=KEY_Back_Clicked,
	.Push_CallBack=KEY_Back_Push,
	.Release_CallBack=KEY_Back_Release
};

keyHandle KEY_Forword={
//	.Clicked_CallBack=KEY_Forword_Clicked,
	.Push_CallBack=KEY_Forword_Push,
	.Release_CallBack=KEY_Forword_Release
};
	
int test=0;
void StartDefaultTask(void *argument)
{
	KeyInit(&KEY_Back, KEY_Back_GPIO_Port, KEY_Back_Pin, GPIO_PIN_RESET);
	KeyInit(&KEY_Forword, KEY_Froword_GPIO_Port, KEY_Froword_Pin, GPIO_PIN_RESET);

	FOS_InitUartBuffer();
	
	InitScalpel();
	
	UART1_AddDevice(NULL,Uart_ReceiveCMD);

	static uint8_t info[]={"Hello PC\r\n"};
	FOS_UART_Transmit_DMA(&huart1,info,sizeof(info));	
	
	for(;;)
	{
		HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
		FOS_printf("Scalpel:%.3f,%.3f\r\n",
						(double)gripperMoto.nowPosi,(double)wireMoto.nowPosi);
		osDelay(1000);
		
//		if(test%6==0)	{YS_FastGoto(&gripperMoto,10.0);	YS_FastGoto(&wireMoto,10.0);}
//		if(test%6==1)	{YS_FastGoto(&gripperMoto,20.0);	YS_FastGoto(&wireMoto,20.0);}
//		if(test%6==2)	{YS_FastGoto(&gripperMoto,30.0);	YS_FastGoto(&wireMoto,30.0);}
//		if(test%6==3)	{YS_FastGoto(&gripperMoto,40.0);	YS_FastGoto(&wireMoto,40.0);}
//		if(test%6==4)	{YS_FastGoto(&gripperMoto,30.0);	YS_FastGoto(&wireMoto,30.0);}
//		if(test%6==5)	{YS_FastGoto(&gripperMoto,20.0);	YS_FastGoto(&wireMoto,20.0);}
		
		test+=1;
	}
}

/****************************串口控制按键的使能*******************************************/
void EnableKeys()
{
	KEY_Back.Push_CallBack=KEY_Back_Push;
	KEY_Back.Release_CallBack=KEY_Back_Release;

	KEY_Forword.Push_CallBack=KEY_Forword_Push;
	KEY_Forword.Release_CallBack=KEY_Forword_Release;
}

//一个空函数，让按键指针指向他，就可以屏蔽本来的功能
void KEY_NULL(void* key){}
	
void DisableKeys()
{
	KEY_Back.Push_CallBack=KEY_NULL;
	KEY_Back.Release_CallBack=KEY_NULL;

	KEY_Forword.Push_CallBack=KEY_NULL;
	KEY_Forword.Release_CallBack=KEY_NULL;
}

//串口控制按键的使能
uint8_t Uart_ReceiveCMD(uint8_t* Controlflag, uint16_t Size, void* Scalpel)
{
	if(Controlflag[0] != 0x08)	
	{
		return NotMy_uart1_Data;
	}
	switch(Controlflag[1])
	{
		case 0x00 :		
			EnableKeys();
		break;
		
		case 0x01 :		
			DisableKeys();
		break;
	}
	return IsMy_uart1_Data;
}

/****************************DEBUG函数*******************************************/
#if YS_DEBUG
void YS_Moto_SET_PID(uint16_t kp,uint16_t ki,uint16_t kd,uint16_t IS)
{	
//	YS_SetPID(&wireMoto,(float)kp / 1000.0,(float)ki / 1000.0,(float)kd / 1000.0);
	YS_SetPID(&gripperMoto,(float)kp / 1000.0,(float)ki / 1000.0,(float)kd / 1000.0,(float)IS);
}
#endif
