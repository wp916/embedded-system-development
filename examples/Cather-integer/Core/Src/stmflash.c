#include "stmflash.h"
#include <string.h>	//数据拷贝

#include "freertos.h"
#include "task.h"

/*****************************************************************************
本程序基于ALIENTEK战舰STM32开发板V3，STM32 FLASH 驱动代码开发 
微调程序结构，增加了调试保护机制，学习代码核心思想，可直接参考正点原子相关教程
*****************************************************************************/

//FLASH起始地址
#define STM32_FLASH_BASE (0x08000000) 		//STM32 FLASH的起始地址

union {
	uint8_t data[8];
	uint16_t _16bit;
	uint32_t _32bit;
	uint64_t _64bit;
}flashTrains;

/**
*@brief	从指定地址开始读出一个8bit的数据
*@parm	ReadAddr:读取数据的起始地址
*@return返回一个8bit的数据，你根据你的实际数据类型进行转换*/
inline uint8_t STMFLASH_Read8(uint32_t faddr)
{
	return *(volatile uint8_t*)(faddr);
}

/**
*@brief	从指定地址开始读出一个16bit的数据
*@parm	ReadAddr:读取数据的起始地址
*@return返回一个16bit的数据，你根据你的实际数据类型进行转换*/
inline uint16_t STMFLASH_Read16(uint32_t faddr)
{
	memcpy(flashTrains.data,(uint8_t*)faddr,2);
	return flashTrains._16bit; 
}

/**
*@brief	从指定地址开始读出一个32bit的数据
*@parm	ReadAddr:读取数据的起始地址
*@return返回一个32bit的数据，你根据你的实际数据类型进行转换*/
inline uint32_t STMFLASH_Read32(uint32_t faddr)
{
	memcpy(flashTrains.data,(uint8_t*)faddr,4);
	return flashTrains._32bit; 
}

/**
*@brief	从指定地址开始读出一个64bit的数据
*@parm	ReadAddr:读取数据的起始地址
*@return返回一个64bit的数据，你根据你的实际数据类型进行转换*/
inline uint64_t STMFLASH_Read64(uint32_t faddr)
{
	memcpy(flashTrains.data,(uint8_t*)faddr,8);
	return flashTrains._32bit; 
}

/**
*@brief 从指定地址开始读出指定个数的数据
		并将其顺序倒置输出!!!
*@parm	faddr:读取数据的起始地址
*@parm	pBuffer:存储数据的数据指针
*@parm	NumToRead:读取多少个数据
*@parm	length:数据占据内存大小/8bit */
void STMFLASH_ReadBit(uint32_t faddr,uint8_t *pBuffer,uint16_t NumToRead,uint8_t length)   	
{
	memcpy(pBuffer,(uint8_t*)faddr,NumToRead*length);
}

#if STM32_FLASH_WREN	//如果使能了写   

//不检查的写入
//WriteAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数   
void STMFLASH_Write_NoCheck(uint32_t WriteAddr,uint16_t *pBuffer,uint16_t NumToWrite)   
{ 			 		 
	uint16_t i;
	for(i=0;i<NumToWrite;i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,WriteAddr,pBuffer[i]);
	    WriteAddr+=2;//地址增加2.
	}  
} 

//调用了“stm32f1xx_hal_flash_ex.h”里的一个本来不向用户公开的函数
extern void FLASH_PageErase(uint32_t PageAddress); 
 
uint16_t STMFLASH_BUF[STM32_PAGE_SIZE/2];//最多是2K字节

//从指定地址开始写入指定长度的数据
//WriteAddr:起始地址(此地址必须为2的倍数!!)
//pBuffer:数据指针
//NumToWrite:半字(16位)数(就是要写入的16位数据的个数.)
void STMFLASH_Write(uint32_t WriteAddr,uint16_t *pBuffer,uint16_t NumToWrite)	
{
	uint32_t secpos;	   //扇区地址
	uint16_t secoff;	   //扇区内偏移地址(16位字计算)
	uint16_t secremain; //扇区内剩余地址(16位字计算)	   
 	uint16_t i;    
	uint32_t offaddr;   //去掉0X08000000后的地址

	if( (WriteAddr<STM32_FLASH_BASE) || (WriteAddr >= (STM32_FLASH_BASE + STM32_PAGE_SIZE * STM32_FLASH_SIZE)) )return;//非法地址
	
	//禁止在写入期间切换程序
	taskDISABLE_INTERRUPTS();
	
	HAL_FLASH_Unlock();					//解锁
	offaddr=WriteAddr-STM32_FLASH_BASE;		//实际偏移地址.
	secpos=offaddr/STM32_PAGE_SIZE;			//扇区地址  0~127 for STM32F103RBT6
	secoff=(offaddr%STM32_PAGE_SIZE)/2;		//在扇区内的偏移(2个字节为基本单位.)
	secremain=STM32_PAGE_SIZE/2-secoff;		//扇区剩余空间大小   
	if(NumToWrite<=secremain)secremain=NumToWrite;//不大于该扇区范围
	while(1) 
	{	
		STMFLASH_Read16Bit(secpos*STM32_PAGE_SIZE+STM32_FLASH_BASE,(uint8_t*)STMFLASH_BUF,STM32_PAGE_SIZE/2);//读出整个扇区的内容
		for(i=0;i<secremain;i++)	//校验数据
		{
			if(STMFLASH_BUF[secoff+i]!=0XFFFF)break;//需要擦除  	  
		}
		if(i<secremain)				//需要擦除
		{
			FLASH_PageErase(secpos*STM32_PAGE_SIZE+STM32_FLASH_BASE);	//擦除这个扇区
			FLASH_WaitForLastOperation(FLASH_WAITETIME);            	//等待上次操作完成
			CLEAR_BIT(FLASH->CR, FLASH_CR_PER);							//清除CR寄存器的PER位，此操作应该在FLASH_PageErase()中完成！
																		//但是HAL库里面并没有做，应该是HAL库bug！
			for(i=0;i<secremain;i++)//复制
			{
				STMFLASH_BUF[i+secoff]=pBuffer[i];	  
			}
			STMFLASH_Write_NoCheck(secpos*STM32_PAGE_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM32_PAGE_SIZE/2);//写入整个扇区  
		}else 
		{
			FLASH_WaitForLastOperation(FLASH_WAITETIME);       	//等待上次操作完成
			STMFLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//写已经擦除了的,直接写入扇区剩余区间. 
		}
		if(NumToWrite==secremain)break;//写入结束了
		else//写入未结束
		{
			secpos++;				//扇区地址增1
			secoff=0;				//偏移位置为0 	 
		   	pBuffer+=secremain;  	//指针偏移
			WriteAddr+=secremain*2;	//写地址偏移(16位数据地址,需要*2)	   
		   	NumToWrite-=secremain;	//字节(16位)数递减
			if(NumToWrite>(STM32_PAGE_SIZE/2))secremain=STM32_PAGE_SIZE/2;//下一个扇区还是写不完
			else secremain=NumToWrite;//下一个扇区可以写完了
		}	 
	};	
	HAL_FLASH_Lock();		//上锁
	
	taskENABLE_INTERRUPTS();
}


//#if FLASH_DEBUG

//const uint16_t num=STM32_FLASH_SIZE/32+1;
////表示内存的某一页是否被用过
//uint32_t usedFlags[num];

///**
//* @brief	检查地址是否正确
//* @return	1：地址正确；0：地址有问题 */
//char AddIsRight(uint32_t addr)
//{		
//	// 判断写入地址是否在合法范围内
//	if (addr < FLASH_ADD_MIN || (addr >= (STM32_FLASH_SIZE*STM32_PAGE_SIZE+FLASH_ADD_MIN)))
//	{
//		FOS_printf("\nerror::addr:0x%x out of range", addr);
//		return 0;
//	}		
//	
//	uint16_t pageNum=(addr-FLASH_ADD_MIN)/STM32_PAGE_SIZE;
//	uint32_t flag =1;
//	flag = flag << (pageNum%32);
//	//检测页是否被使用过
//	uint16_t flag_num=flag/32;
//	if(usedFlags[flag_num] && flag)
//	{
//		FOS_printf("\nwarning:page:%d might been used in different codes", addr);
//		return 0;
//	}
//	//记录此地址被使用过
//	usedFlags[flag_num] |= flag;
//	return 1;
//}

//#endif

#endif
