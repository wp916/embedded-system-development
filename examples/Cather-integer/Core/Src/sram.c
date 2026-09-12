/**
 ****************************************************************************************************
 * @file        sram.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2020-04-27
 * @brief       外部SRAM 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32F103开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20200427
 * 第一次发布
 *
 ****************************************************************************************************
 */

#include "./BSP/SRAM/sram.h"
#include "./SYSTEM/usart/usart.h"

SRAM_HandleTypeDef g_sram_handler; //SRAM句柄

/**
 * @brief       初始化 外部SRAM
 * @param       无
 * @retval      无
 */
void sram_init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    FSMC_NORSRAM_TimingTypeDef fsmc_readwritetim;

    SRAM_CS_GPIO_CLK_ENABLE();      /* SRAM_CS脚时钟使能 */
    SRAM_WR_GPIO_CLK_ENABLE();      /* SRAM_WR脚时钟使能 */
    SRAM_RD_GPIO_CLK_ENABLE();      /* SRAM_RD脚时钟使能 */
    __HAL_RCC_FSMC_CLK_ENABLE();    /* 使能FSMC时钟 */
    __HAL_RCC_GPIOD_CLK_ENABLE();   /* 使能GPIOD时钟 */
    __HAL_RCC_GPIOE_CLK_ENABLE();   /* 使能GPIOE时钟 */
    __HAL_RCC_GPIOF_CLK_ENABLE();   /* 使能GPIOF时钟 */
    __HAL_RCC_GPIOG_CLK_ENABLE();   /* 使能GPIOG时钟 */

    GPIO_Initure.Pin = SRAM_CS_GPIO_PIN;
    GPIO_Initure.Mode = GPIO_MODE_AF_PP;       
    GPIO_Initure.Pull = GPIO_PULLUP;           
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH; 
    HAL_GPIO_Init(SRAM_CS_GPIO_PORT, &GPIO_Initure); /* SRAM_CS引脚模式设置 */

    GPIO_Initure.Pin = SRAM_WR_GPIO_PIN;
    HAL_GPIO_Init(SRAM_WR_GPIO_PORT, &GPIO_Initure); /* SRAM_WR引脚模式设置 */

    GPIO_Initure.Pin = SRAM_RD_GPIO_PIN;
    HAL_GPIO_Init(SRAM_RD_GPIO_PORT, &GPIO_Initure); /* SRAM_CS引脚模式设置 */

    
    /* PD0,1,4,5,8~15 */
    GPIO_Initure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9 | 
                       GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |
                       GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_Initure.Mode = GPIO_MODE_AF_PP;       /* 推挽复用 */
    GPIO_Initure.Pull = GPIO_PULLUP;           /* 上拉 */
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH; /* 高速 */
    HAL_GPIO_Init(GPIOD, &GPIO_Initure);

    /* PE0,1,7~15 */
    GPIO_Initure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 |
                       GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |
                       GPIO_PIN_15;
    HAL_GPIO_Init(GPIOE, &GPIO_Initure);

    /* PF0~5,12~15 */
    GPIO_Initure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
                       GPIO_PIN_5 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOF, &GPIO_Initure);

    /* PG0~5,10 */
    GPIO_Initure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOG, &GPIO_Initure);

    g_sram_handler.Instance = FSMC_NORSRAM_DEVICE;
    g_sram_handler.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;

    g_sram_handler.Init.NSBank = (SRAM_FSMC_NEX == 1) ? FSMC_NORSRAM_BANK1 : \
                                 (SRAM_FSMC_NEX == 2) ? FSMC_NORSRAM_BANK2 : \
                                 (SRAM_FSMC_NEX == 3) ? FSMC_NORSRAM_BANK3 : 
                                                        FSMC_NORSRAM_BANK4; /* 根据配置选择FSMC_NE1~4 */
    g_sram_handler.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;     /* 地址/数据线不复用 */
    g_sram_handler.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;                 /* SRAM */
    g_sram_handler.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;    /* 16位数据宽度 */
    g_sram_handler.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;   /* 是否使能突发访问,仅对同步突发存储器有效,此处未用到 */
    g_sram_handler.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW; /* 等待信号的极性,仅在突发模式访问下有用 */
    g_sram_handler.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;      /* 存储器是在等待周期之前的一个时钟周期还是等待周期期间使能NWAIT */
    g_sram_handler.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;       /* 存储器写使能 */
    g_sram_handler.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;              /* 等待使能位,此处未用到 */
    g_sram_handler.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;          /* 读写使用相同的时序 */
    g_sram_handler.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;  /* 是否使能同步传输模式下的等待信号,此处未用到 */
    g_sram_handler.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;              /* 禁止突发写 */
    /* FSMC读时序控制寄存器 */
    fsmc_readwritetim.AddressSetupTime = 0x00;                              /* 地址建立时间（ADDSET）为1个HCLK 1/72M=13.8ns */
    fsmc_readwritetim.AddressHoldTime = 0x00;                               /* 地址保持时间（ADDHLD）模式A未用到 */
    fsmc_readwritetim.DataSetupTime = 0x01;                                 /* 数据保存时间为2个HCLK=2*13.8=27.8ns */
    fsmc_readwritetim.BusTurnAroundDuration = 0X00;
    fsmc_readwritetim.AccessMode = FSMC_ACCESS_MODE_A;                      /* 模式A */
    HAL_SRAM_Init(&g_sram_handler, &fsmc_readwritetim, &fsmc_readwritetim);
}

/**
 * @brief       往SRAM指定地址写入指定长度数据
 * @param       pbuf    : 数据存储区
 * @param       addr    : 开始写入的地址(最大32bit)
 * @param       datalen : 要写入的字节数(最大32bit)
 * @retval      无
 */
void sram_write(uint8_t *pbuf, uint32_t addr, uint32_t datalen)
{
    for (; datalen != 0; datalen--)
    {
        *(volatile uint8_t *)(SRAM_BASE_ADDR + addr) = *pbuf;
        addr++;
        pbuf++;
    }
}

/**
 * @brief       从SRAM指定地址读取指定长度数据
 * @param       pbuf    : 数据存储区
 * @param       addr    : 开始读取的地址(最大32bit)
 * @param       datalen : 要读取的字节数(最大32bit)
 * @retval      无
 */
void sram_read(uint8_t *pbuf, uint32_t addr, uint32_t datalen)
{
    for (; datalen != 0; datalen--)
    {
        *pbuf++ = *(volatile uint8_t *)(SRAM_BASE_ADDR + addr);
        addr++;
    }
}

/*******************测试函数**********************************/

/**
 * @brief       测试函数 在SRAM指定地址写入1个字节
 * @param       addr    : 开始写入的地址(最大32bit)
 * @param       data    : 要写入的字节
 * @retval      无
 */
void sram_test_write(uint32_t addr, uint8_t data)
{
    sram_write(&data, addr, 1); /* 写入1个字节 */
}

/**
 * @brief       测试函数 在SRAM指定地址读取1个字节
 * @param       addr    : 开始读取的地址(最大32bit)
 * @retval      读取到的数据(1个字节)
 */
uint8_t sram_test_read(uint32_t addr)
{
    uint8_t data;
    sram_read(&data, addr, 1); /* 读取1个字节 */
    return data;
}
