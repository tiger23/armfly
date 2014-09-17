/*
*********************************************************************************************************
*	                                  
*	ģ������ : WM8978��ƵоƬ����ģ��
*	�ļ����� : bsp_WM8978.h
*	��    �� : V1.0
*	˵    �� : ͷ�ļ�
*	�޸ļ�¼ :
*		�汾��  ����       ����    ˵��
*		v0.1    2009-12-27 armfly  �������ļ���ST�̼���汾ΪV3.1.2
*		v1.0    2011-09-04 armfly  ST�̼���������V3.5.0�汾��
*
*	Copyright (C), 2010-2011, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#ifndef _BSP_WM8978_H
#define _BSP_WM8978_H

#include <inttypes.h>

#define WM8978_SLAVE_ADDRESS    0x34	/* WM8978 I2C�ӻ���ַ */


/* ����wm8978_Cfg()�������β� */
#define DAC_ON			1
#define DAC_OFF			0

#define AUX_ON			1
#define AUX_OFF			0

#define LINE_ON			1
#define LINE_OFF		0

#define EAR_ON			1
#define EAR_OFF			0

#define SPK_ON			1
#define SPK_OFF			0

/* ����������� */
#define VOLUME_MAX		63		/* ������� */
#define VOLUME_STEP		3		/* �������ڲ��� */

/* ���ⲿ���õĺ������� */
void wm8978_ChangeVolume(uint8_t _ucLeftVolume, uint8_t _ucRightVolume);
uint8_t wm8978_ReadVolume(void);
void wm8978_Mute(uint8_t _ucMute);
void wm8978_Reset(void);
void wm8978_PowerDown(void);
uint8_t wm8978_WriteReg(uint8_t _ucRegAddr, uint16_t _usValue);
void wm8978_Cfg(uint8_t _ucDacEn, uint8_t _ucAuxEn, uint8_t _ucLineEn, uint8_t _ucSpkEn, uint8_t _ucEarEn);
void wm8978_Dac2Ear(void);
void wm8978_Dac2Spk(void);
void wm8978_Aux2Ear(void);
void wm8978_Aux2Spk(void);
void wm8978_CfgAudioIF(uint16_t _usStandard, uint8_t _ucWordLen);
uint8_t wm8978_CheckOk(void);
void I2S_CODEC_Init(void);
void I2S_Config(uint16_t _usStandard, uint16_t _usWordLen, uint16_t _usAudioFreq);

void I2S_Start(void);
void I2S_Stop(void);

#endif
