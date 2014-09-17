/*
*********************************************************************************************************
*	                                  
*	ģ������ : WM8978��ƵоƬ����ģ��
*	�ļ����� : bsp_WM8978.c
*	��    �� : V1.0
*	˵    �� : WM8978��ƵоƬ�ײ������� ʹ��ǰ���ȵ��� wm8978_CheckOk()һ�Ρ�
*	�޸ļ�¼ :
*		�汾��  ����       ����    ˵��
*		v0.1    2009-12-27 armfly  �������ļ���ST�̼���汾ΪV3.1.2
*		v1.0    2011-09-04 armfly  ST�̼���������V3.5.0�汾��
*
*	Copyright (C), 2010-2011, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#include "stm32f10x.h"
#include "bsp_WM8978.h"
#include "bsp_i2c_gpio.h"

/*
	wm8978�Ĵ�������cash
	����I2C���߽ӿڲ�֧�ֶ�ȡ��������˼Ĵ������û������ڴ���
	�Ĵ���MAP ��WM8978.pdf �ĵ�67ҳ
	�Ĵ�����ַ��7bit�� �Ĵ���������9bit
*/
static uint16_t wm8978_RegCash[] = {
	0x000, 0x000, 0x000, 0x000, 0x050, 0x000, 0x140, 0x000,
	0x000, 0x000, 0x000, 0x0FF, 0x0FF, 0x000, 0x100, 0x0FF,
	0x0FF, 0x000, 0x12C, 0x02C, 0x02C, 0x02C, 0x02C, 0x000,
	0x032, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000,
	0x038, 0x00B, 0x032, 0x000, 0x008, 0x00C, 0x093, 0x0E9,
	0x000, 0x000, 0x000, 0x000, 0x003, 0x010, 0x010, 0x100,
	0x100, 0x002, 0x001, 0x001, 0x039, 0x039, 0x039, 0x039,
	0x001, 0x001
};

/*
*********************************************************************************************************
*	�� �� ��: wm8978_CheckOk
*	����˵��: ���I2C�����ϵ�WM8978�Ƿ�����
*	��    �Σ���
*	�� �� ֵ: 1 ��ʾ������0��ʾ������
*********************************************************************************************************
*/
uint8_t wm8978_CheckOk(void)
{
	if (i2c_CheckDevice(WM8978_SLAVE_ADDRESS) == 0)
	{
		return 1;
	}
	return 0;		
}

/*
*********************************************************************************************************
*	�� �� ��: wm8978_ReadReg
*	����˵��: ��cash�ж��ض���wm8978�Ĵ���
*	��    �Σ�_ucRegAddr �� �Ĵ�����ַ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
uint16_t wm8978_ReadReg(uint8_t _ucRegAddr)
{
	return wm8978_RegCash[_ucRegAddr];
}

/*
*********************************************************************************************************
*	�� �� ��: wm8978_WriteReg
*	����˵��: дwm8978�Ĵ���
*	��    �Σ�_ucRegAddr �� �Ĵ�����ַ
*			  _usValue ���Ĵ���ֵ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
uint8_t wm8978_WriteReg(uint8_t _ucRegAddr, uint16_t _usValue)
{
	uint8_t ucAck;

	/* ������ʼλ */
	i2c_Start();

	/* �����豸��ַ+��д����bit��0 = w�� 1 = r) bit7 �ȴ� */
	i2c_SendByte(WM8978_SLAVE_ADDRESS | I2C_WR);

	/* ���ACK */
	ucAck = i2c_WaitAck();
	if (ucAck == 1)
	{
		return 0;
	}

	/* ���Ϳ����ֽ�1 */
	i2c_SendByte(((_ucRegAddr << 1) & 0xFE) | ((_usValue >> 8) & 0x1));

	/* ���ACK */
	ucAck = i2c_WaitAck();
	if (ucAck == 1)
	{
		return 0;
	}

	/* ���Ϳ����ֽ�2 */
	i2c_SendByte(_usValue & 0xFF);

	/* ���ACK */
	ucAck = i2c_WaitAck();
	if (ucAck == 1)
	{
		return 0;
	}

	/* ����STOP */
	i2c_Stop();

	wm8978_RegCash[_ucRegAddr] = _usValue;
	return 1;
}

/*
*********************************************************************************************************
*	�� �� ��: wm8978_Dac2Ear
*	����˵��: ��ʼ��wm8978Ӳ���豸,DAC���������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void wm8978_Dac2Ear(void)
{
	wm8978_Cfg(DAC_ON, AUX_OFF, LINE_OFF, SPK_OFF, EAR_ON);
}

/*
*********************************************************************************************************
*	�� �� ��: wm8978_Dac2Spk
*	����˵��: ��ʼ��wm8978Ӳ���豸,DAC�����������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void wm8978_Dac2Spk(void)
{
	wm8978_Cfg(DAC_ON, AUX_OFF, LINE_OFF, SPK_ON, EAR_OFF);
}

/*
*********************************************************************************************************
*	�� �� ��: wm8978_Aux2Ear
*	����˵��: ��ʼ��wm8978Ӳ���豸,Aux(FM������)���������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void wm8978_Aux2Ear(void)
{
	wm8978_Cfg(DAC_OFF, AUX_ON, LINE_OFF, SPK_OFF, EAR_ON);
}

/*
*********************************************************************************************************
*	�� �� ��: wm8978_Aux2Spk
*	����˵��: ��ʼ��wm8978Ӳ���豸,Aux(FM������)�����������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void wm8978_Aux2Spk(void)
{
	wm8978_Cfg(DAC_OFF, AUX_ON, LINE_OFF, SPK_ON, EAR_OFF);
}

/*
*********************************************************************************************************
*	�� �� ��: wm8978_Cfg
*	����˵��: ��ʼ��wm8978Ӳ���豸
*	��    �Σ�
*			_ucDacEn : DAC����ͨ��ʹ��(CPUͨ��I2S�ӿڴ��͵�������Ƶ�ź�)
*			_ucAuxEn : ��������ͨ��ʹ�ܣ�FM������ģ�����Ƶ����źţ�
*			_ucLineEn : ��·����ͨ��ʹ�ܣ�V2���ǿսţ�V3�����ӵ�VS1003B����оƬ����Ƶ�����
*			_ucSpkEn : ���������ʹ��
*			_ucEarEn : �������ʹ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void wm8978_Cfg(uint8_t _ucDacEn, uint8_t _ucAuxEn, uint8_t _ucLineEn, uint8_t _ucSpkEn, uint8_t _ucEarEn)
{
	uint16_t usReg;
	
	/* pdf 67ҳ���Ĵ����б� */

	/*	REG 1
		B8		BUFDCOPEN	= x
		B7		OUT4MIXEN	= x
		B6		OUT3MIXEN	= X  �������ʱ����������Ϊ1 (for ������������)
		B5		PLLEN	= x
		b4`		MICBEN = x
		B3		BIASEN = 1		��������Ϊ1ģ��Ŵ����Ź���
		B2		BUFIOEN = x
		B1:0	VMIDSEL = 3  ��������Ϊ��00ֵģ��Ŵ����Ź���
	*/
	usReg = 0;
	if ((_ucSpkEn == 1) || (_ucEarEn == 1))
	{
		usReg = ((1 << 3) | (3 << 0));
	}
	if (_ucEarEn == 1)
	{
		usReg |= (1 << 6);
	}
	wm8978_WriteReg(1, usReg);

	/*	REG 2
		B8		ROUT1EN = 1;	�������ͨ��
		B7		LOUT1EN = 1;	�������ͨ��	
		B6		SLEEP = x;
		B5		BOOSTENR = x;
		B4		BOOSTENL = x;
		B3		INPGAENR = x;
		B2		NPPGAENL = x;
		B1		ADCENR = x;
		B0		ADCENL = x;
	*/
	usReg = 0;
	if (_ucEarEn == 1)
	{
		usReg = ((1 << 8) | (1 << 7));
	}
	wm8978_WriteReg(2, usReg);		
		
	/* REG 3
		B8	OUT4EN = 0
		B7	OUT3EN = x;		������������ڶ����ĵ���
		B6	LOUT2EN = 1;	���������ͨ��
		B5	ROUT2EN = 1;	���������ͨ��
		B4	----   = x
		B3	RMIXEN = 1;		���MIX, ����������������
		B2	LMIXEN = 1;		���MIX, ����������������
		B1	DACENR = x;		DAC��
		B0	DACENL = x;
	*/
	usReg = 0;
	if ((_ucSpkEn == 1) || (_ucEarEn == 1))
	{
		usReg |= ((1 << 3) | (1 << 2));
	}
	if (_ucEarEn == 1)
	{
		usReg |= (1 << 7);
	}
	if (_ucSpkEn == 1)
	{
		usReg |= ((1 << 6) | (1 << 5));
	}
	if (_ucDacEn == 1)
	{
		usReg |= ((1 << 1) | (1 << 0));
	}
	wm8978_WriteReg(3, usReg);

	/*
		REG 11,12
		DAC ����
	*/
	if (_ucDacEn == 1)
	{	
		#if 0	/* �˴���Ҫ��������, �����л�ʱ����״̬���ı� */
		wm8978_WriteReg(11, 255);
		wm8978_WriteReg(12, 255 | 0x100);
		#endif
	}
	else
	{
		;
	}

	/*	REG 43
		B8:6 = 0
		B5	MUTERPGA2INV = 0;	Mute input to INVROUT2 mixer
		B4	INVROUT2 = ROUT2 ����; �����������������
		B3:1	BEEPVOL = 7;	AUXR input to ROUT2 inverter gain
		B0	BEEPEN = 1;	1 = enable AUXR beep input
			
	*/
	usReg = 0;
	if (_ucSpkEn == 1)
	{
		usReg |= (1 << 4);
	} 
	if (_ucAuxEn == 1)
	{
		usReg |= ((7 << 1) | (1 << 0));
	}
	wm8978_WriteReg(43, usReg);
		
	/* REG 49
		B8:7	0
		B6		DACL2RMIX = x
		B5		DACR2LMIX = x
		B4		OUT4BOOST = 0
		B3		OUT3BOOST = 0
		B2		SPKBOOST = x	SPK BOOST
		B1		TSDEN = 1    �������ȱ���ʹ�ܣ�ȱʡ1��
		B0		VROI = 0	Disabled Outputs to VREF Resistance
	*/
	usReg = 0;
	if (_ucDacEn == 1)
	{
		usReg |= ((0 << 6) | (0 << 5));
	}
	if (_ucSpkEn == 1)
	{
		usReg |=  ((0 << 2) | (1 << 1));	/* 1.5x����,  �ȱ���ʹ�� */
	}
	wm8978_WriteReg(49, usReg);
	
	/*	REG 50    (50����������51�������������üĴ�������һ��
		B8:6	AUXLMIXVOL = 111	AUX����FM�������ź�����
		B5		AUXL2LMIX = 1		Left Auxilliary input to left channel
		B4:2	BYPLMIXVOL			����
		B1		BYPL2LMIX = 0;		Left bypass path (from the left channel input boost output) to left output mixer
		B0		DACL2LMIX = 1;		Left DAC output to left output mixer
	*/
	usReg = 0;
	if (_ucAuxEn == 1)
	{
		usReg |= ((7 << 6) | (1 << 5));
	}
	if (_ucLineEn == 1)
	{
		usReg |= ((7 << 2) | (1 << 1));
	}
	if (_ucDacEn == 1)
	{
		usReg |= (1 << 0);
	}
	wm8978_WriteReg(50, usReg);
	wm8978_WriteReg(51, usReg);

	/*	
		REG 52,53	����EAR����
		REG 54,55	����SPK����
		
		B8		HPVU		����ͬ��������������
		B7		LOUT1ZC = 1  ��λ�л�
		B6		LOUT1MUTE    0��ʾ������ 1��ʾ����
	*/
#if 0	/* �˴���Ҫ��������, ����Ӧ�ó����л����ʱ������״̬���ı� */
	if (_ucEarEn == 1)
	{
		usReg = (0x3f | (1 << 7));
		wm8978_WriteReg(52, usReg);
		wm8978_WriteReg(53, usReg | (1 << 8));
	}
	else
	{
		usReg = ((1 << 7) | (1 << 6));
		wm8978_WriteReg(52, usReg);
		wm8978_WriteReg(53, usReg | (1 << 8));
	}
	
	if (_ucSpkEn == 1)
	{
		usReg = (0x3f | (1 << 7));
		wm8978_WriteReg(54, usReg);
		wm8978_WriteReg(55, usReg | (1 << 8));
	}
	else
	{
		usReg = ((1 << 7) | (1 << 6));
		wm8978_WriteReg(54, usReg);
		wm8978_WriteReg(55, usReg | (1 << 8));
	}		
#endif	
		
	/*	REG 56   OUT3 mixer ctrl
		B6		OUT3MUTE = 1;
		B5		LDAC2OUT3 = 0;
		B4		OUT4_2OUT3
		B3		BYPL2OUT3
		B2		LMIX2OUT3	
		B1		LDAC2OUT3
	*/
	wm8978_WriteReg(56, (1 <<6));		/**/
				
	/* 	Softmute enable = 0 */
	if (_ucDacEn == 1)
	{
		wm8978_WriteReg(10, 0);
	}
}

/*
*********************************************************************************************************
*	�� �� ��: wm8978_Cfg
*	����˵��: ������������������ֵ
*	��    �Σ�_ucLeftVolume ������������ֵ
*			  _ucLRightVolume : ����������ֵ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void wm8978_ChangeVolume(uint8_t _ucLeftVolume, uint8_t _ucRightVolume)
{
#if 0
	wm8978_WriteReg(11, _ucLeftVolume);
	wm8978_WriteReg(12, _ucRightVolume | 0x100);
#else
	uint16_t regL;
	uint16_t regR;

	if (_ucLeftVolume > 0x3F)
	{
		_ucLeftVolume = 0x3F;
	}

	if (_ucRightVolume > 0x3F)
	{
		_ucRightVolume = 0x3F;
	}

	regL = _ucLeftVolume;
	regR = _ucRightVolume;

	/* �ȸ�������������ֵ */
	wm8978_WriteReg(52, regL | 0x00);

	/* ��ͬ�������������������� */
	wm8978_WriteReg(53, regR | 0x100);	/* 0x180��ʾ ������Ϊ0ʱ�ٸ��£���������������ֵġ����ա��� */

	/* �ȸ�������������ֵ */
	wm8978_WriteReg(54, regL | 0x00);

	/* ��ͬ�������������������� */
	wm8978_WriteReg(55, regR | 0x100);	/* ������Ϊ0ʱ�ٸ��£���������������ֵġ����ա��� */

#endif
}

/*
*********************************************************************************************************
*	�� �� ��: wm8978_ReadVolume
*	����˵��: ����ͨ��������.
*	��    �Σ���
*	�� �� ֵ: ��ǰ����ֵ
*********************************************************************************************************
*/
uint8_t wm8978_ReadVolume(void)
{
	return (uint8_t)(wm8978_ReadReg(52) & 0x3F );
}

/*
*********************************************************************************************************
*	�� �� ��: wm8978_Mute
*	����˵��: �������.
*	��    �Σ�_ucMute ��1�Ǿ�����0�ǲ�����.
*	�� �� ֵ: ��ǰ����ֵ
*********************************************************************************************************
*/
void wm8978_Mute(uint8_t _ucMute)
{
	uint16_t usRegValue;

	if (_ucMute == 1) /* ���� */
	{
		usRegValue = wm8978_ReadReg(52); /* Left Mixer Control */
		usRegValue |= (1u << 6);
		wm8978_WriteReg(52, usRegValue);

		usRegValue = wm8978_ReadReg(53); /* Left Mixer Control */
		usRegValue |= (1u << 6);
		wm8978_WriteReg(53, usRegValue);

		usRegValue = wm8978_ReadReg(54); /* Right Mixer Control */
		usRegValue |= (1u << 6);
		wm8978_WriteReg(54, usRegValue);

		usRegValue = wm8978_ReadReg(55); /* Right Mixer Control */
		usRegValue |= (1u << 6);
		wm8978_WriteReg(55, usRegValue);
	}
	else	/* ȡ������ */
	{
		usRegValue = wm8978_ReadReg(52);
		usRegValue &= ~(1u << 6);
		wm8978_WriteReg(52, usRegValue);

		usRegValue = wm8978_ReadReg(53); /* Left Mixer Control */
		usRegValue &= ~(1u << 6);
		wm8978_WriteReg(53, usRegValue);

		usRegValue = wm8978_ReadReg(54);
		usRegValue &= ~(1u << 6);
		wm8978_WriteReg(54, usRegValue);

		usRegValue = wm8978_ReadReg(55); /* Left Mixer Control */
		usRegValue &= ~(1u << 6);
		wm8978_WriteReg(55, usRegValue);
	}
}

/*
*********************************************************************************************************
*	�� �� ��: wm8978_Mute
*	����˵��: ��λwm8978�����еļĴ���ֵ�ָ���ȱʡֵ
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void wm8978_Reset(void)
{
	/* wm8978�Ĵ���ȱʡֵ */
	const uint16_t reg_default[] = {
	0x000, 0x000, 0x000, 0x000, 0x050, 0x000, 0x140, 0x000,
	0x000, 0x000, 0x000, 0x0FF, 0x0FF, 0x000, 0x100, 0x0FF,
	0x0FF, 0x000, 0x12C, 0x02C, 0x02C, 0x02C, 0x02C, 0x000,
	0x032, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000, 0x000,
	0x038, 0x00B, 0x032, 0x000, 0x008, 0x00C, 0x093, 0x0E9,
	0x000, 0x000, 0x000, 0x000, 0x003, 0x010, 0x010, 0x100,
	0x100, 0x002, 0x001, 0x001, 0x039, 0x039, 0x039, 0x039,
	0x001, 0x001
	};
	uint8_t i;

	wm8978_WriteReg(0x00, 0);

	for (i = 0; i < sizeof(reg_default) / 2; i++)
	{
		wm8978_RegCash[i] = reg_default[i];
	}
}

/*
*********************************************************************************************************
*	�� �� ��: wm8978_PowerDown
*	����˵��: �ر�wm8978������͹���ģʽ
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void wm8978_PowerDown(void)
{
	/*
	Set DACMU = 1 to mute the audio DACs.
	Disable all Outputs.
	Disable VREF and VMIDSEL.
	Switch off the power supplies
	*/
	uint16_t usRegValue;

	usRegValue = wm8978_ReadReg(10);
	usRegValue |= (1u <<6);
	wm8978_WriteReg(10, usRegValue);

	/* δ�� */
}

/*
*********************************************************************************************************
*	�� �� ��: I2S_GPIO_Config
*	����˵��: ����GPIO��������codecӦ��
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void I2S_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* Enable GPIOB, GPIOC and AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOG | 
	                     RCC_APB2Periph_GPIOF | RCC_APB2Periph_AFIO, ENABLE);
	
	/* I2S2 SD, CK and WS pins configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* I2S2 MCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/*
*********************************************************************************************************
*	�� �� ��: I2S_Config
*	����˵��: ����STM32��I2S����
*	��    �Σ�_usStandard : �ӿڱ�׼��I2S_Standard_Phillips, I2S_Standard_MSB �� I2S_Standard_LSB
*			  _usMCLKOutput : ��ʱ�������I2S_MCLKOutput_Enable or I2S_MCLKOutput_Disable
*			  _usAudioFreq : ����Ƶ�ʣ�I2S_AudioFreq_8K��I2S_AudioFreq_16K��I2S_AudioFreq_22K��
*							I2S_AudioFreq_44K��I2S_AudioFreq_48
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void I2S_Config(uint16_t _usStandard, uint16_t _usWordLen, uint16_t _usAudioFreq)
{
	I2S_InitTypeDef I2S_InitStructure; 

	/* �� I2S2 APB1 ʱ�� */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	
	/* ��λ SPI2 ���赽ȱʡ״̬ */
	SPI_I2S_DeInit(SPI2); 
	
	/* I2S2 �������� */
	I2S_InitStructure.I2S_Mode = I2S_Mode_MasterTx;			/* ����Ϊ������ģʽ */
	I2S_InitStructure.I2S_Standard = _usStandard;			/* �ӿڱ�׼ */
	I2S_InitStructure.I2S_DataFormat = _usWordLen;			/* ���ݸ�ʽ��16bit */
	I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Enable;	/* ��ʱ��ģʽ */
	I2S_InitStructure.I2S_AudioFreq = _usAudioFreq;			/* ��Ƶ����Ƶ�� */
	I2S_InitStructure.I2S_CPOL = I2S_CPOL_Low;  			
	I2S_Init(SPI2, &I2S_InitStructure);
	
	/* �Ƚ�ֹI2S2 TXE�ж� */ 
	SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, DISABLE);
	
	/* ʹ�� SPI2/I2S2 ���� */
	I2S_Cmd(SPI2, ENABLE);
}

/*
*********************************************************************************************************
*	�� �� ��: wm8978_CfgAudioIF
*	����˵��: ����WM8978����Ƶ�ӿ�
*	��    �Σ�
*			  _usStandard : �ӿڱ�׼��I2S_Standard_Phillips, I2S_Standard_MSB �� I2S_Standard_LSB
*			  _ucWordLen : �ֳ���16��24��32  �����������õ�20bit��ʽ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void wm8978_CfgAudioIF(uint16_t _usStandard, uint8_t _ucWordLen)
{
	uint16_t usReg;
	
	/* pdf 67ҳ���Ĵ����б� */

	/*	REG R4, ��Ƶ�ӿڿ��ƼĴ���
		B8		BCP	 = X, BCLK���ԣ�0��ʾ������1��ʾ����
		B7		LRCP = x, LRCʱ�Ӽ��ԣ�0��ʾ������1��ʾ����
		B6:5	WL = x�� �ֳ���00=16bit��01=20bit��10=24bit��11=32bit ���Ҷ���ģʽֻ�ܲ��������24bit)
		B4:3	FMT = x����Ƶ���ݸ�ʽ��00=�Ҷ��룬01=����룬10=I2S��ʽ��11=PCM
		B2		DACLRSWAP = x, ����DAC���ݳ�����LRCʱ�ӵ���߻����ұ�
		B1 		ADCLRSWAP = x������ADC���ݳ�����LRCʱ�ӵ���߻����ұ�
		B0		MONO	= 0��0��ʾ��������1��ʾ������������������Ч
	*/
	usReg = 0;
	if (_usStandard == I2S_Standard_Phillips)	/* I2S�����ֱ�׼ */
	{
		usReg |= (2 << 3);
	}
	else if (_usStandard == I2S_Standard_MSB)	/* MSB�����׼(�����) */
	{
		usReg |= (1 << 3);
	}
	else if (_usStandard == I2S_Standard_LSB)	/* LSB�����׼(�Ҷ���) */
	{
		usReg |= (0 << 3);
	}	
	else	/* PCM��׼(16λͨ��֡�ϴ������֡ͬ������16λ����֡��չΪ32λͨ��֡) */
	{
		usReg |= (3 << 3);;
	}

	if (_ucWordLen == 24)
	{
		usReg |= (2 << 5);
	}	
	else if (_ucWordLen == 32)
	{
		usReg |= (3 << 5);
	}		
	else
	{
		usReg |= (0 << 5);		/* 16bit */
	}
	wm8978_WriteReg(4, usReg);

	/* 
		R6��ʱ�Ӳ������ƼĴ���		
		MS = 0,  WM8978����ʱ�ӣ���MCU�ṩMCLKʱ��
	*/	
	wm8978_WriteReg(6, 0x000);
}

/*
*********************************************************************************************************
*	�� �� ��: I2S_CODEC_Init
*	����˵��: ����STM32 I2S��Ƶ�ӿ�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void I2S_CODEC_Init(void)
{	
	/* ����I2S2, I2C1 and GPIOF pins */
	I2S_GPIO_Config(); 

	/* ��ֹ the I2S2 TXE Interrupt  => Generate the clocks*/ 
	SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, DISABLE);
}

/*
*********************************************************************************************************
*	�� �� ��: I2S_Start
*	����˵��: ����I2S2 TXE�ж�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void I2S_Start(void)
{
	SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, ENABLE);
}

/*
*********************************************************************************************************
*	�� �� ��: I2S_Stop
*	����˵��: ֹͣI2S2 TXE�ж�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/ 
void I2S_Stop(void)
{
	SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, DISABLE);
}
