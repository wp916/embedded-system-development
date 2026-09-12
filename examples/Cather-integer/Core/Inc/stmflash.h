#ifndef __STMFLASH_H__
#define __STMFLASH_H__

/*****************************************************************************
本程序基于ALIENTEK战舰STM32开发板V3，STM32 FLASH 驱动代码开发 
微调程序结构，增加了保护机制，学习代码核心思想，可直接参考正点原子相关教程

提示：
1、由于stm32不同型号Flash可用大小不同，因此定义 FLASH_TYPE 宏，来限定Flash的大小
2、其实这个功能包本身就是一个调试功能包，当功能调试好后，建议直接将调试好的数据写入
编写好的程序，这样肯定万无一失。
*****************************************************************************/

/*****************************************************************************
内存管理的重要提醒：

STM32f103ZET6的Flash相关内存地址
|	mass	|	page		|	addr					|	size
|	 主		|	页	0		|	0x0800 0000-0x080007FF	|	2k
|	 存		|	页	1		|	0x0800 0800-0x08000FFF	|	2k
|			|	页	2		|	0x0800 1000-0x080017FF	|	2k
|			|	页	3		|	0x0800 1800-0x0800FFFF	|	2k
|	 储		|	…			|	…						|	2k
|	 器		|	页	255		|	0x0807 F800-0x0807FFFF	|	2k


1、数据存储地址（addr）建议尽量往后（大)选，因为靠前(小)的地址存储的是你烧录进单片机的程序
2、由于功能包提供的写入程序必须要先擦除指定区域再重新写入数据，而擦除数据会以
页（page）为最小单位，因此，要确保不同程序之间存储数据的地址不在相同页里，
否则后写入的数据会将其他程序中存储的数据清除
*****************************************************************************/

#include "stm32f1xx_hal.h"

//用户根据自己的需要设置
#define FLASH_TYPE 1	//0:c8t6 64K；1:rct6 256K；2:zet6 512K；
#if (FLASH_TYPE==0)
#define STM32_FLASH_SIZE 	64
#define STM32_PAGE_SIZE 	1024 	 		//所选STM32的FLASH的页数的内存大小（size）
#endif
#if (FLASH_TYPE==1)
#define STM32_FLASH_SIZE 	128
#define STM32_PAGE_SIZE 	2048 	 		//所选STM32的FLASH的页数的内存大小（size）
#endif
#if (FLASH_TYPE==2)							//F103RCT6	F103ZET6
#define STM32_FLASH_SIZE 	256 	 		//所选STM32的FLASH的页数（page）
#define STM32_PAGE_SIZE 	2048 	 		//所选STM32的FLASH的页数的内存大小（size）
#endif

#define STM32_FLASH_WREN 	1              	//使能FLASH写入(0，不是能;1，使能)
#define FLASH_WAITETIME  	50000          	//FLASH等待超时时间

/*由于STM32的内存特点，输入的数据会倒着存储，这导致读取数据时要倒着重排一下内存，于是有了下面的自动读取和重拍的函数
例如： float a=1.23 其二进制表示为 0x3F9D70A4 但是你存入FLASH后，再读出来就成了0xA4709D3F，他每隔两位倒转了一次顺序
*/ 
/**
*@brief	从指定地址开始读出一个16bit的数据
*@parm	ReadAddr:读取数据的起始地址
*@return返回一个16bit的数据，你根据你的实际数据类型进行转换*/
inline uint16_t STMFLASH_Read16(uint32_t faddr);

/**
*@brief 从指定地址开始读出指定个数的数据
		并将其顺序倒置输出!!!
*@parm	faddr:读取数据的起始地址
*@parm	pBuffer:存储数据的数据指针
*@parm	NumToRead:读取多少个数据
*@parm	length:数据占据内存大小/8bit */
void STMFLASH_ReadBit(uint32_t faddr,uint8_t *pBuffer,uint16_t NumToRead,uint8_t length);
#define STMFLASH_Read8Bit(faddr,pBuffer,NumToRead) STMFLASH_ReadBit(faddr,pBuffer,NumToRead,1) //从指定地址开始读出指定个数的数据
#define STMFLASH_Read16Bit(faddr,pBuffer,NumToRead) STMFLASH_ReadBit(faddr,pBuffer,NumToRead,2) //从指定地址开始读出指定个数的数据，并每16Bit重新排序
#define STMFLASH_Read32Bit(faddr,pBuffer,NumToRead) STMFLASH_ReadBit(faddr,pBuffer,NumToRead,4) //从指定地址开始读出指定个数的数据，并每32Bit重新排序
#define STMFLASH_Read64Bit(faddr,pBuffer,NumToRead) STMFLASH_ReadBit(faddr,pBuffer,NumToRead,8) //从指定地址开始读出指定个数的数据，并每64Bit重新排序

#if STM32_FLASH_WREN
void	STMFLASH_Write(uint32_t WriteAddr,uint16_t *pBuffer,uint16_t NumToWrite);		//从指定地址开始写入指定长度的数据
#endif

#endif

















