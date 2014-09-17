/*
*********************************************************************************************************
*	                                  
*	ģ������ : CAN������ʾ����
*	�ļ����� : can_network.c
*	��    �� : V1.0
*	˵    �� : ��ʾ���ʵ�ֶ��CAN�ڵ㣨�ڵ�ĳ�����ͬ��֮���ͨ�š�
*	�޸ļ�¼ :
*		�汾��  ����       ����    ˵��
*		v1.0    2011-09-01 armfly  ST�̼���V3.5.0�汾��
*
*	Copyright (C), 2010-2011, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#include "stm32f10x.h"
#include <stdio.h>
#include "bsp_button.h"
#include "bsp_timer.h"
#include "bsp_led.h"
#include "can_network.h"

/* ����ȫ�ֱ��� */
CanTxMsg g_tCanTxMsg;	/* ���ڷ��� */
CanRxMsg g_tCanRxMsg;	/* ���ڽ��� */
uint8_t g_ucLedNo = 0;	/* ������LED����ţ�0-3 */

/* �������ļ��ڵ��õĺ������� */
static void can_Init(void);
static void can_NVIC_Config(void);
static void SendCanMsg(void);

/*
*********************************************************************************************************
*	�� �� ��: can_demo
*	����˵��: CAN��������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void can_demo(void)
{	
	uint8_t ucKeyCode;	   	/* �������� */
	uint16_t usAutoPeriod;	/* �Զ���ʱ���͵�ʱ���� */
	
	can_Init();				/* ��ʼ��STM32 CANӲ�� */
	can_NVIC_Config();		/* ����CAN�ж� */
	
	g_ucLedNo = 0;
	/* �ȹر����е�LED���ٵ�������һ��LED */
	bsp_LedOffAll();
	bsp_LedOn(g_ucLedNo + 1);

	usAutoPeriod = 100;	   /* ȱʡ�Զ��ط������ڣ�100ms */
	while(1)
	{
		/* ��ʱ����, ��Ҫ��TAMPER������ */
		if (bsp_CheckTimer(1))
		{
			bsp_StartTimer(1, usAutoPeriod);	/* ��ʱ���ڣ���λ ms */
		   	SendCanMsg(); 						/* ��CAN���緢��һ������ */
		}

		/* �������¼� */
		ucKeyCode = bsp_GetKey();
		if (ucKeyCode > 0)
		{
			/* �м����� */
			switch (ucKeyCode)
			{
				case KEY_DOWN_USER:		/* USER������ */
					/* ÿ��1��USER����������һ��LED������CAN���緢��һ������ */
					SendCanMsg(); 	/* ��CAN���緢��һ������ */				
					break;

				case KEY_DOWN_TAMPER:	/* TAMPER�����£���ʾ��ʼ�Զ����� */
					bsp_StartTimer(1, 50);	/* ��ʱ���� 50ms */
					break;

				case KEY_DOWN_WAKEUP:	/* WAKEUP�����£���ʾֹͣ�Զ����� */
					bsp_StopTimer(1);	/* ��ʱ���� 50ms */
					break;								

				case KEY_DOWN_JOY_UP:	/* ҡ���ϼ�������Զ��ط���Ƶ�� */
					if (usAutoPeriod > 10)
					{
						usAutoPeriod -= 10;
					}
					break;

				case KEY_DOWN_JOY_DOWN:	/* ҡ���¼��������Զ��ط���Ƶ�� */
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
*	�� �� ��: SendCanMsg
*	����˵��: ����һ������
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void SendCanMsg(void)
{
	/* ÿ��1��USER����������һ��LED������CAN���緢��һ������ */
	if (++g_ucLedNo > 3)
	{
		g_ucLedNo = 0;
	}
	
	/* �ȹر����е�LED���ٵ�������һ��LED */
	bsp_LedOffAll();
	bsp_LedOn(g_ucLedNo + 1);					
	
	/* ��CAN���緢��һ������, ��������1���ֽ�ָLED����� */
    g_tCanTxMsg.Data[0] = g_ucLedNo;
    CAN_Transmit(CAN1, &g_tCanTxMsg);	
}

/*
*********************************************************************************************************
*	�� �� ��: can_Init
*	����˵��: ����CANӲ��
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void can_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;	
		
	/* PB8��PB9��������ΪAFIOģʽ, �л���CAN���� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	/* ʹ��GPIOʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	/* ����CAN�źŽ�������: RX */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	/* GPIO����Ϊ��������ģʽ */
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* ����CAN�źŷ�������: TX */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		/* ����Ϊ����������� */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* ����GPIO����ٶ� */
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* ԭ����CAN���ź�USB��������ͬ�Ŀ��ߣ�������������ʹ�����ŵ���ӳ�书�ܽ�CAN�����л���PB8��PB9 */
	GPIO_PinRemapConfig(GPIO_Remap1_CAN1 , ENABLE);	/* ʹ��CAN1����ӳ�� */
	
	/* ʹ��CAN����ʱ�� */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	
	CAN_DeInit(CAN1);						/* ��λCAN�Ĵ��� */
	CAN_StructInit(&CAN_InitStructure);		/* ���CAN�����ṹ���ԱΪȱʡֵ */
	
	/*
		TTCM = time triggered communication mode
		ABOM = automatic bus-off management 
		AWUM = automatic wake-up mode
		NART = no automatic retransmission
		RFLM = receive FIFO locked mode 
		TXFP = transmit FIFO priority		
	*/
	CAN_InitStructure.CAN_TTCM = DISABLE;			/* ��ֹʱ�䴥��ģʽ��������ʱ���), T  */
	CAN_InitStructure.CAN_ABOM = DISABLE;			/* ��ֹ�Զ����߹رչ��� */
	CAN_InitStructure.CAN_AWUM = DISABLE;			/* ��ֹ�Զ�����ģʽ */
	CAN_InitStructure.CAN_NART = DISABLE;			/* ��ֹ�ٲö�ʧ��������Զ��ش����� */
	CAN_InitStructure.CAN_RFLM = DISABLE;			/* ��ֹ����FIFO����ģʽ */
	CAN_InitStructure.CAN_TXFP = DISABLE;			/* ��ֹ����FIFO���ȼ� */
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;	/* ����CANΪ��������ģʽ */
	
	/* 
		CAN ������ = RCC_APB1Periph_CAN / Prescaler / (SJW + BS1 + BS2);
		
		SJW = synchronisation_jump_width 
		BS = bit_segment
		
		�����У�����CAN������Ϊ500Kbps		
		CAN ������ = 360000000 / 6 / (1 + 6 + 5) / = 500kHz		
	*/
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;
	CAN_InitStructure.CAN_Prescaler = 6;
	CAN_Init(CAN1, &CAN_InitStructure);
	
	/* ����CAN�˲���0 */
	CAN_FilterInitStructure.CAN_FilterNumber = 0;		/* �˲�����ţ�0-13����14���˲��� */
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;		/* �˲���ģʽ������ID����ģʽ */
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;	/* 32λ�˲� */
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;					/* �����ID�ĸ�16bit */
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;					/* �����ID�ĵ�16bit */
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;				/* ID����ֵ��16bit */
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;				/* ID����ֵ��16bit */
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;		/* �˲�����FIFO 0 */
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;				/* ʹ���˲��� */
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	/* ��䷢�Ͳ����� Ҳ������ÿ�η��͵�ʱ���� */
	g_tCanTxMsg.StdId = 0x321;
	g_tCanTxMsg.ExtId = 0x01;
	g_tCanTxMsg.RTR = CAN_RTR_DATA;
	g_tCanTxMsg.IDE = CAN_ID_STD;
	g_tCanTxMsg.DLC = 1;

	/* �˴���ʱ������ */
}     

/*
*********************************************************************************************************
*	�� �� ��: can_NVIC_Config
*	����˵��: ����CAN�ж�
*	��    �Σ���
*	�� �� ֵ: ��
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
	
	/* CAN FIFO0 ��Ϣ�����ж�ʹ�� */ 
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);	
}

/*
*********************************************************************************************************
*	�� �� ��: can_ISR
*	����˵��: CAN�жϷ������. ��������� stm32f10x_it.c�б�����
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/ 
void can_ISR(void)
{
	CAN_Receive(CAN1, CAN_FIFO0, &g_tCanRxMsg);
	if ((g_tCanRxMsg.StdId == 0x321) && (g_tCanRxMsg.IDE == CAN_ID_STD) && (g_tCanRxMsg.DLC == 1))
	{
		g_ucLedNo = g_tCanRxMsg.Data[0];	/* ���ݻ�������1���ֽڱ�ʾLED��� */

		/* �ȹر����е�LED���ٵ�������һ��LED */
		bsp_LedOffAll();					
		bsp_LedOn(g_ucLedNo + 1);
	}
}



