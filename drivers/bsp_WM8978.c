/*
*********************************************************************************************************
*	                                  
*	模块名称 : WM8978音频芯片驱动模块
*	文件名称 : bsp_WM8978.c
*	版    本 : V1.0
*	说    明 : WM8978音频芯片底层驱动。 使用前请先调用 wm8978_CheckOk()一次。
*	修改记录 :
*		版本号  日期       作者    说明
*		v0.1    2009-12-27 armfly  创建该文件，ST固件库版本为V3.1.2
*		v1.0    2011-09-04 armfly  ST固件库升级到V3.5.0版本。
*
*	Copyright (C), 2010-2011, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#include "stm32f10x.h"
#include "bsp_WM8978.h"
#include "bsp_i2c_gpio.h"

/*
	wm8978寄存器缓存cash
	由于I2C两线接口不支持读取操作，因此寄存器设置换存在内存中
	寄存器MAP 在WM8978.pdf 的第67页
	寄存器地址是7bit， 寄存器数据是9bit
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
*	函 数 名: wm8978_CheckOk
*	功能说明: 检查I2C总线上的WM8978是否正常
*	形    参：无
*	返 回 值: 1 表示正常，0表示不正常
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
*	函 数 名: wm8978_ReadReg
*	功能说明: 从cash中读回读回wm8978寄存器
*	形    参：_ucRegAddr ： 寄存器地址
*	返 回 值: 无
*********************************************************************************************************
*/
uint16_t wm8978_ReadReg(uint8_t _ucRegAddr)
{
	return wm8978_RegCash[_ucRegAddr];
}

/*
*********************************************************************************************************
*	函 数 名: wm8978_WriteReg
*	功能说明: 写wm8978寄存器
*	形    参：_ucRegAddr ： 寄存器地址
*			  _usValue ：寄存器值
*	返 回 值: 无
*********************************************************************************************************
*/
uint8_t wm8978_WriteReg(uint8_t _ucRegAddr, uint16_t _usValue)
{
	uint8_t ucAck;

	/* 发送起始位 */
	i2c_Start();

	/* 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传 */
	i2c_SendByte(WM8978_SLAVE_ADDRESS | I2C_WR);

	/* 检测ACK */
	ucAck = i2c_WaitAck();
	if (ucAck == 1)
	{
		return 0;
	}

	/* 发送控制字节1 */
	i2c_SendByte(((_ucRegAddr << 1) & 0xFE) | ((_usValue >> 8) & 0x1));

	/* 检测ACK */
	ucAck = i2c_WaitAck();
	if (ucAck == 1)
	{
		return 0;
	}

	/* 发送控制字节2 */
	i2c_SendByte(_usValue & 0xFF);

	/* 检测ACK */
	ucAck = i2c_WaitAck();
	if (ucAck == 1)
	{
		return 0;
	}

	/* 发送STOP */
	i2c_Stop();

	wm8978_RegCash[_ucRegAddr] = _usValue;
	return 1;
}

/*
*********************************************************************************************************
*	函 数 名: wm8978_Dac2Ear
*	功能说明: 初始化wm8978硬件设备,DAC输出到耳机
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void wm8978_Dac2Ear(void)
{
	wm8978_Cfg(DAC_ON, AUX_OFF, LINE_OFF, SPK_OFF, EAR_ON);
}

/*
*********************************************************************************************************
*	函 数 名: wm8978_Dac2Spk
*	功能说明: 初始化wm8978硬件设备,DAC输出到扬声器
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void wm8978_Dac2Spk(void)
{
	wm8978_Cfg(DAC_ON, AUX_OFF, LINE_OFF, SPK_ON, EAR_OFF);
}

/*
*********************************************************************************************************
*	函 数 名: wm8978_Aux2Ear
*	功能说明: 初始化wm8978硬件设备,Aux(FM收音机)输出到耳机
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void wm8978_Aux2Ear(void)
{
	wm8978_Cfg(DAC_OFF, AUX_ON, LINE_OFF, SPK_OFF, EAR_ON);
}

/*
*********************************************************************************************************
*	函 数 名: wm8978_Aux2Spk
*	功能说明: 初始化wm8978硬件设备,Aux(FM收音机)输出到扬声器
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void wm8978_Aux2Spk(void)
{
	wm8978_Cfg(DAC_OFF, AUX_ON, LINE_OFF, SPK_ON, EAR_OFF);
}

/*
*********************************************************************************************************
*	函 数 名: wm8978_Cfg
*	功能说明: 初始化wm8978硬件设备
*	形    参：
*			_ucDacEn : DAC输入通道使能(CPU通过I2S接口传送的数字音频信号)
*			_ucAuxEn : 辅助输入通道使能（FM收音机模块的音频输出信号）
*			_ucLineEn : 线路输入通道使能（V2板是空脚，V3板连接到VS1003B解码芯片的音频输出）
*			_ucSpkEn : 扬声器输出使能
*			_ucEarEn : 耳机输出使能
*	返 回 值: 无
*********************************************************************************************************
*/
void wm8978_Cfg(uint8_t _ucDacEn, uint8_t _ucAuxEn, uint8_t _ucLineEn, uint8_t _ucSpkEn, uint8_t _ucEarEn)
{
	uint16_t usReg;
	
	/* pdf 67页，寄存器列表 */

	/*	REG 1
		B8		BUFDCOPEN	= x
		B7		OUT4MIXEN	= x
		B6		OUT3MIXEN	= X  耳机输出时，必须设置为1 (for 安富莱开发板)
		B5		PLLEN	= x
		b4`		MICBEN = x
		B3		BIASEN = 1		必须设置为1模拟放大器才工作
		B2		BUFIOEN = x
		B1:0	VMIDSEL = 3  必须设置为非00值模拟放大器才工作
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
		B8		ROUT1EN = 1;	耳机输出通道
		B7		LOUT1EN = 1;	耳机输出通道	
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
		B7	OUT3EN = x;		耳机输出，用于耳机的地线
		B6	LOUT2EN = 1;	扬声器输出通道
		B5	ROUT2EN = 1;	扬声器输出通道
		B4	----   = x
		B3	RMIXEN = 1;		输出MIX, 耳机和扬声器公用
		B2	LMIXEN = 1;		输出MIX, 耳机和扬声器公用
		B1	DACENR = x;		DAC用
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
		DAC 音量
	*/
	if (_ucDacEn == 1)
	{	
		#if 0	/* 此处不要设置音量, 避免切换时音量状态被改变 */
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
		B4	INVROUT2 = ROUT2 反相; 用于扬声器推挽输出
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
		B1		TSDEN = 1    扬声器热保护使能（缺省1）
		B0		VROI = 0	Disabled Outputs to VREF Resistance
	*/
	usReg = 0;
	if (_ucDacEn == 1)
	{
		usReg |= ((0 << 6) | (0 << 5));
	}
	if (_ucSpkEn == 1)
	{
		usReg |=  ((0 << 2) | (1 << 1));	/* 1.5x增益,  热保护使能 */
	}
	wm8978_WriteReg(49, usReg);
	
	/*	REG 50    (50是左声道，51是右声道，配置寄存器功能一致
		B8:6	AUXLMIXVOL = 111	AUX用于FM收音机信号输入
		B5		AUXL2LMIX = 1		Left Auxilliary input to left channel
		B4:2	BYPLMIXVOL			音量
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
		REG 52,53	控制EAR音量
		REG 54,55	控制SPK音量
		
		B8		HPVU		用于同步更新左右声道
		B7		LOUT1ZC = 1  零位切换
		B6		LOUT1MUTE    0表示正常， 1表示静音
	*/
#if 0	/* 此处不要设置音量, 避免应用程序切换输出时，音量状态被改变 */
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
*	函 数 名: wm8978_Cfg
*	功能说明: 两个左右声道音量的值
*	形    参：_ucLeftVolume ：左声道音量值
*			  _ucLRightVolume : 右声道音量值
*	返 回 值: 无
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

	/* 先更新左声道缓存值 */
	wm8978_WriteReg(52, regL | 0x00);

	/* 再同步更新左右声道的音量 */
	wm8978_WriteReg(53, regR | 0x100);	/* 0x180表示 在音量为0时再更新，避免调节音量出现的“嘎哒”声 */

	/* 先更新左声道缓存值 */
	wm8978_WriteReg(54, regL | 0x00);

	/* 再同步更新左右声道的音量 */
	wm8978_WriteReg(55, regR | 0x100);	/* 在音量为0时再更新，避免调节音量出现的“嘎哒”声 */

#endif
}

/*
*********************************************************************************************************
*	函 数 名: wm8978_ReadVolume
*	功能说明: 读回通道的音量.
*	形    参：无
*	返 回 值: 当前音量值
*********************************************************************************************************
*/
uint8_t wm8978_ReadVolume(void)
{
	return (uint8_t)(wm8978_ReadReg(52) & 0x3F );
}

/*
*********************************************************************************************************
*	函 数 名: wm8978_Mute
*	功能说明: 输出静音.
*	形    参：_ucMute ：1是静音，0是不静音.
*	返 回 值: 当前音量值
*********************************************************************************************************
*/
void wm8978_Mute(uint8_t _ucMute)
{
	uint16_t usRegValue;

	if (_ucMute == 1) /* 静音 */
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
	else	/* 取消静音 */
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
*	函 数 名: wm8978_Mute
*	功能说明: 复位wm8978，所有的寄存器值恢复到缺省值
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void wm8978_Reset(void)
{
	/* wm8978寄存器缺省值 */
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
*	函 数 名: wm8978_PowerDown
*	功能说明: 关闭wm8978，进入低功耗模式
*	形    参：无
*	返 回 值: 无
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

	/* 未完 */
}

/*
*********************************************************************************************************
*	函 数 名: I2S_GPIO_Config
*	功能说明: 配置GPIO引脚用于codec应用
*	形    参：无
*	返 回 值: 无
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
*	函 数 名: I2S_Config
*	功能说明: 配置STM32的I2S外设
*	形    参：_usStandard : 接口标准，I2S_Standard_Phillips, I2S_Standard_MSB 或 I2S_Standard_LSB
*			  _usMCLKOutput : 主时钟输出，I2S_MCLKOutput_Enable or I2S_MCLKOutput_Disable
*			  _usAudioFreq : 采样频率，I2S_AudioFreq_8K、I2S_AudioFreq_16K、I2S_AudioFreq_22K、
*							I2S_AudioFreq_44K、I2S_AudioFreq_48
*	返 回 值: 无
*********************************************************************************************************
*/
void I2S_Config(uint16_t _usStandard, uint16_t _usWordLen, uint16_t _usAudioFreq)
{
	I2S_InitTypeDef I2S_InitStructure; 

	/* 打开 I2S2 APB1 时钟 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	
	/* 复位 SPI2 外设到缺省状态 */
	SPI_I2S_DeInit(SPI2); 
	
	/* I2S2 外设配置 */
	I2S_InitStructure.I2S_Mode = I2S_Mode_MasterTx;			/* 配置为主发送模式 */
	I2S_InitStructure.I2S_Standard = _usStandard;			/* 接口标准 */
	I2S_InitStructure.I2S_DataFormat = _usWordLen;			/* 数据格式，16bit */
	I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Enable;	/* 主时钟模式 */
	I2S_InitStructure.I2S_AudioFreq = _usAudioFreq;			/* 音频采样频率 */
	I2S_InitStructure.I2S_CPOL = I2S_CPOL_Low;  			
	I2S_Init(SPI2, &I2S_InitStructure);
	
	/* 先禁止I2S2 TXE中断 */ 
	SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, DISABLE);
	
	/* 使能 SPI2/I2S2 外设 */
	I2S_Cmd(SPI2, ENABLE);
}

/*
*********************************************************************************************************
*	函 数 名: wm8978_CfgAudioIF
*	功能说明: 配置WM8978的音频接口
*	形    参：
*			  _usStandard : 接口标准，I2S_Standard_Phillips, I2S_Standard_MSB 或 I2S_Standard_LSB
*			  _ucWordLen : 字长，16、24、32  （丢弃不常用的20bit格式）
*	返 回 值: 无
*********************************************************************************************************
*/
void wm8978_CfgAudioIF(uint16_t _usStandard, uint8_t _ucWordLen)
{
	uint16_t usReg;
	
	/* pdf 67页，寄存器列表 */

	/*	REG R4, 音频接口控制寄存器
		B8		BCP	 = X, BCLK极性，0表示正常，1表示反相
		B7		LRCP = x, LRC时钟极性，0表示正常，1表示反相
		B6:5	WL = x， 字长，00=16bit，01=20bit，10=24bit，11=32bit （右对齐模式只能操作在最大24bit)
		B4:3	FMT = x，音频数据格式，00=右对齐，01=左对齐，10=I2S格式，11=PCM
		B2		DACLRSWAP = x, 控制DAC数据出现在LRC时钟的左边还是右边
		B1 		ADCLRSWAP = x，控制ADC数据出现在LRC时钟的左边还是右边
		B0		MONO	= 0，0表示立体声，1表示单声道，仅左声道有效
	*/
	usReg = 0;
	if (_usStandard == I2S_Standard_Phillips)	/* I2S飞利浦标准 */
	{
		usReg |= (2 << 3);
	}
	else if (_usStandard == I2S_Standard_MSB)	/* MSB对齐标准(左对齐) */
	{
		usReg |= (1 << 3);
	}
	else if (_usStandard == I2S_Standard_LSB)	/* LSB对齐标准(右对齐) */
	{
		usReg |= (0 << 3);
	}	
	else	/* PCM标准(16位通道帧上带长或短帧同步或者16位数据帧扩展为32位通道帧) */
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
		R6，时钟产生控制寄存器		
		MS = 0,  WM8978被动时钟，由MCU提供MCLK时钟
	*/	
	wm8978_WriteReg(6, 0x000);
}

/*
*********************************************************************************************************
*	函 数 名: I2S_CODEC_Init
*	功能说明: 配置STM32 I2S音频接口
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void I2S_CODEC_Init(void)
{	
	/* 配置I2S2, I2C1 and GPIOF pins */
	I2S_GPIO_Config(); 

	/* 禁止 the I2S2 TXE Interrupt  => Generate the clocks*/ 
	SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, DISABLE);
}

/*
*********************************************************************************************************
*	函 数 名: I2S_Start
*	功能说明: 启动I2S2 TXE中断
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void I2S_Start(void)
{
	SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, ENABLE);
}

/*
*********************************************************************************************************
*	函 数 名: I2S_Stop
*	功能说明: 停止I2S2 TXE中断
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/ 
void I2S_Stop(void)
{
	SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, DISABLE);
}
