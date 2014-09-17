/*
*********************************************************************************************************
*	                                  
*	模块名称 : CAN网络演示程序。
*	文件名称 : can_network.c
*	版    本 : V1.0
*	说    明 : 演示如何实现多个CAN节点（节点的程序相同）之间的通信。
*	修改记录 :
*		版本号  日期       作者    说明
*		v1.0    2011-09-01 armfly  ST固件库V3.5.0版本。
*
*	Copyright (C), 2010-2011, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#include "stm32f10x.h"
#include <stdio.h>
#include "bsp_button.h"
#include "bsp_timer.h"
#include "bsp_led.h"
#include "can_network.h"

/* 定义全局变量 */
CanTxMsg g_tCanTxMsg;	/* 用于发送 */
CanRxMsg g_tCanRxMsg;	/* 用于接收 */
uint8_t g_ucLedNo = 0;	/* 点亮的LED灯序号，0-3 */

/* 仅允许本文件内调用的函数声明 */
static void can_Init(void);
static void can_NVIC_Config(void);
static void SendCanMsg(void);

/*
*********************************************************************************************************
*	函 数 名: can_demo
*	功能说明: CAN网络例程
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void can_demo(void)
{	
	uint8_t ucKeyCode;	   	/* 按键代码 */
	uint16_t usAutoPeriod;	/* 自动定时发送的时间间隔 */
	
	can_Init();				/* 初始化STM32 CAN硬件 */
	can_NVIC_Config();		/* 配置CAN中断 */
	
	g_ucLedNo = 0;
	/* 先关闭所有的LED，再点亮其中一个LED */
	bsp_LedOffAll();
	bsp_LedOn(g_ucLedNo + 1);

	usAutoPeriod = 100;	   /* 缺省自动重发的周期，100ms */
	while(1)
	{
		/* 定时发送, 需要按TAMPER键启动 */
		if (bsp_CheckTimer(1))
		{
			bsp_StartTimer(1, usAutoPeriod);	/* 定时周期，单位 ms */
		   	SendCanMsg(); 						/* 向CAN网络发送一包数据 */
		}

		/* 处理按键事件 */
		ucKeyCode = bsp_GetKey();
		if (ucKeyCode > 0)
		{
			/* 有键按下 */
			switch (ucKeyCode)
			{
				case KEY_DOWN_USER:		/* USER键按下 */
					/* 每按1次USER键，点亮下一个LED，并向CAN网络发送一包数据 */
					SendCanMsg(); 	/* 向CAN网络发送一包数据 */				
					break;

				case KEY_DOWN_TAMPER:	/* TAMPER键按下，表示开始自动连发 */
					bsp_StartTimer(1, 50);	/* 定时周期 50ms */
					break;

				case KEY_DOWN_WAKEUP:	/* WAKEUP键按下，表示停止自动连发 */
					bsp_StopTimer(1);	/* 定时周期 50ms */
					break;								

				case KEY_DOWN_JOY_UP:	/* 摇杆上键，提高自动重发的频率 */
					if (usAutoPeriod > 10)
					{
						usAutoPeriod -= 10;
					}
					break;

				case KEY_DOWN_JOY_DOWN:	/* 摇杆下键，降低自动重发的频率 */
					if (usAutoPeriod < 1000)
					{
						usAutoPeriod += 10;
					}
					break;
													
				default:
					break;
			}
		}
	}
}

/*
*********************************************************************************************************
*	函 数 名: SendCanMsg
*	功能说明: 发送一包数据
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void SendCanMsg(void)
{
	/* 每按1次USER键，点亮下一个LED，并向CAN网络发送一包数据 */
	if (++g_ucLedNo > 3)
	{
		g_ucLedNo = 0;
	}
	
	/* 先关闭所有的LED，再点亮其中一个LED */
	bsp_LedOffAll();
	bsp_LedOn(g_ucLedNo + 1);					
	
	/* 向CAN网络发送一包数据, 数据区第1个字节指LED灯序号 */
    g_tCanTxMsg.Data[0] = g_ucLedNo;
    CAN_Transmit(CAN1, &g_tCanTxMsg);	
}

/*
*********************************************************************************************************
*	函 数 名: can_Init
*	功能说明: 配置CAN硬件
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void can_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;	
		
	/* PB8，PB9口线设置为AFIO模式, 切换到CAN功能 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	/* 使能GPIO时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	/* 配置CAN信号接收引脚: RX */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	/* GPIO配置为上拉输入模式 */
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* 配置CAN信号发送引脚: TX */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		/* 配置为复用推挽输出 */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* 设置GPIO最大速度 */
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* 原生的CAN引脚和USB引脚是相同的口线，安富莱开发板使用引脚的重映射功能将CAN引脚切换到PB8，PB9 */
	GPIO_PinRemapConfig(GPIO_Remap1_CAN1 , ENABLE);	/* 使能CAN1的重映射 */
	
	/* 使能CAN外设时钟 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	
	CAN_DeInit(CAN1);						/* 复位CAN寄存器 */
	CAN_StructInit(&CAN_InitStructure);		/* 填充CAN参数结构体成员为缺省值 */
	
	/*
		TTCM = time triggered communication mode
		ABOM = automatic bus-off management 
		AWUM = automatic wake-up mode
		NART = no automatic retransmission
		RFLM = receive FIFO locked mode 
		TXFP = transmit FIFO priority		
	*/
	CAN_InitStructure.CAN_TTCM = DISABLE;			/* 禁止时间触发模式（不生成时间戳), T  */
	CAN_InitStructure.CAN_ABOM = DISABLE;			/* 禁止自动总线关闭管理 */
	CAN_InitStructure.CAN_AWUM = DISABLE;			/* 禁止自动唤醒模式 */
	CAN_InitStructure.CAN_NART = DISABLE;			/* 禁止仲裁丢失或出错后的自动重传功能 */
	CAN_InitStructure.CAN_RFLM = DISABLE;			/* 禁止接收FIFO加锁模式 */
	CAN_InitStructure.CAN_TXFP = DISABLE;			/* 禁止传输FIFO优先级 */
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;	/* 设置CAN为正常工作模式 */
	
	/* 
		CAN 波特率 = RCC_APB1Periph_CAN / Prescaler / (SJW + BS1 + BS2);
		
		SJW = synchronisation_jump_width 
		BS = bit_segment
		
		本例中，设置CAN波特率为500Kbps		
		CAN 波特率 = 360000000 / 6 / (1 + 6 + 5) / = 500kHz		
	*/
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;
	CAN_InitStructure.CAN_Prescaler = 6;
	CAN_Init(CAN1, &CAN_InitStructure);
	
	/* 设置CAN滤波器0 */
	CAN_FilterInitStructure.CAN_FilterNumber = 0;		/* 滤波器序号，0-13，共14个滤波器 */
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;		/* 滤波器模式，设置ID掩码模式 */
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;	/* 32位滤波 */
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;					/* 掩码后ID的高16bit */
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;					/* 掩码后ID的低16bit */
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;				/* ID掩码值高16bit */
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;				/* ID掩码值低16bit */
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;		/* 滤波器绑定FIFO 0 */
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;				/* 使能滤波器 */
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	/* 填充发送参数， 也可以再每次发送的时候填 */
	g_tCanTxMsg.StdId = 0x321;
	g_tCanTxMsg.ExtId = 0x01;
	g_tCanTxMsg.RTR = CAN_RTR_DATA;
	g_tCanTxMsg.IDE = CAN_ID_STD;
	g_tCanTxMsg.DLC = 1;

	/* 此处暂时不发送 */
}     

/*
*********************************************************************************************************
*	函 数 名: can_NVIC_Config
*	功能说明: 配置CAN中断
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/  
static void can_NVIC_Config(void)
{
	NVIC_InitTypeDef  NVIC_InitStructure;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* CAN FIFO0 消息接收中断使能 */ 
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);	
}

/*
*********************************************************************************************************
*	函 数 名: can_ISR
*	功能说明: CAN中断服务程序. 这个函数在 stm32f10x_it.c中被调用
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/ 
void can_ISR(void)
{
	CAN_Receive(CAN1, CAN_FIFO0, &g_tCanRxMsg);
	if ((g_tCanRxMsg.StdId == 0x321) && (g_tCanRxMsg.IDE == CAN_ID_STD) && (g_tCanRxMsg.DLC == 1))
	{
		g_ucLedNo = g_tCanRxMsg.Data[0];	/* 数据缓冲区第1个字节表示LED序号 */

		/* 先关闭所有的LED，再点亮其中一个LED */
		bsp_LedOffAll();					
		bsp_LedOn(g_ucLedNo + 1);
	}
}



