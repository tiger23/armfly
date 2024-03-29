/*
*********************************************************************************************************
*
*	模块名称 : 串行FLASH  SST25VF016B驱动模块
*	文件名称 :  bsp_SST25VF016B.h
*	版    本 : V1.0
*	说    明 : 头文件
*	修改记录 :
*		版本号  日期       作者    说明
*		v1.0    2011-08-21 armfly  ST固件库升级到V3.5.0版本。
*
*	Copyright (C), 2010-2011, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#ifndef _BSP_SST25VF016B_H
#define _BSP_SST25VF016B_H

#include <rtthread.h>
#include <stm32f10x.h>
#include "stm32f10x_spi.h"

/* 定义串行Flash ID */
enum
{
	SST25VF016B_ID = 0xBF2541,
	MX25L1606E_ID = 0xC22015
};

#define SF_UNKNOW_NAME	"Unknow"

typedef struct
{
	uint32_t ChipID;		/* 芯片ID */
	char ChipName[16];		/* 芯片型号字符串，主要用于显示 */
	uint32_t TotalSize;		/* 总容量 */
	uint16_t PageSize;		/* 页面大小 */
}SFLASH_T;

#define SF_TOTAL_SIZE		(2*1024*1024)	/* 总容量，2M字节 */
#define SF_PAGE_SIZE    	(4*1024)		/* 页面大小，4K字节 */

void rt_hw_sf_init(void);
uint32_t sf_ReadID(void);
void sf_EraseChip(void);
void sf_EraseSector(uint32_t _uiSectorAddr);
void sf_PageWrite(uint8_t * _pBuf, uint32_t _uiWriteAddr, uint16_t _usSize);

uint8_t sf_WriteBuffer(uint8_t* _pBuf, uint32_t _uiWriteAddr, uint16_t _usWriteSize);
void sf_ReadBuffer(uint8_t * _pBuf, uint32_t _uiReadAddr, uint32_t _uiSize);

extern SFLASH_T g_tSF;

#endif
