/****************************************************************************
* Copyright (C), 2009-2010, www.armfly.com
*
* �ļ���: bsp_TEA5767.h
* ���ݼ���: FM������ģ���ͷ�ļ�
*
* �ļ���ʷ:
* �汾�� ����       ����    ˵��
* 1.0    2009-11-17 armfly  �������ļ�
*
*/

#ifndef _BSP_TEA5767_H
#define _BSP_TEA5767_H

#include <inttypes.h>
#include "bsp_i2c_gpio.h"

#define TEA5767_SLAVE_ADDRESS    0xC0

#define FM_FREQ_INIT	87500000	/* ����Ƶ�ʳ�ֵ */

#define FM_FREQ_MIN		87500000	/* ��СƵ�� */
#define FM_FREQ_MAX		108000000	/* ���Ƶ�� */
#define FREQ_STEP		100000		/* 0.1MHz = 100000KHz */
#define FREQ_COUNT		((FM_FREQ_MAX - FM_FREQ_MIN) / FREQ_STEP + 1)	/* Ƶ����� */

/* ����ѡ�� */
enum
{
	MUTE_OFF = 0,
	MUTE_ON = 1
};

/* ����ѡ�� */
enum
{
	SEARCH_OFF = 0,
	SEARCH_ON = 1
};
enum
{
	SEARCH_DOWN = 0,
	SEARCH_UP = 1
};
enum
{
	ADC_LEVEL_5 = 1,
	ADC_LEVEL_7 = 2,
	ADC_LEVEL_10 = 3
};

/* TEA5767оƬ��״̬ */
typedef struct
{
	uint8_t ucReady;				/* ��̨������־��1��ʾ���� */
	uint8_t ucBandLimit;			/* Ƶ�����ޱ�־�� 1��ʾ���˼��� */
	uint16_t usPll;					/* ��ǰPLLֵ */
	uint8_t ucStereo;				/* ��������־ 1��ʾ������ 0��ʾ������ */
	uint8_t ucIFCount;				/* IF��������� */
	uint8_t ucAdcLevel;				/* ADC��ƽ */
	uint32_t ulFreq;				/* ��ǰƵ�� */
}TEA5767_T;

void tea5767_Set(uint32_t _Freq, uint8_t _ucMuteEn, uint8_t _ucSerchEn, uint8_t _ucSearchUp, uint8_t _ucAdcLevel);
void tea5767_ReadStatus(TEA5767_T *_tStatus);

#endif
