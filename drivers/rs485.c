//#include <rtthread.h>
#include <rtdevice.h>
#include <stm32f10x.h>
#include <stm32f10x_usart.h>

#include "rs485.h"

rt_device_t rs485_device= RT_NULL;
static char rs485_device_TX_buf[5] = { 0, 1, 2, 3, 4};
static char rs485_device_RX_buf[48];
rt_uint8_t rs485_device1_TX_flag;
rt_uint8_t rs485_device2_TX_flag;
rt_uint8_t rs485_device3_TX_flag;

void RS485_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    //RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, ENABLE); //复位串口2
    //RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, DISABLE); //停止复位

    PA1_RS485_RX_EN;        //默认为接收模式
    //PB8_RS485_RX_EN;        //默认为接收模式
    //PC9_RS485_RX_EN;        //默认为接收模式

  	/* CPU的小缺陷:串口配置好，如果直接发送，则第一个字节发送不出去，
  	如下语句解决第一个字节无法发送的问题*/
	//USART_ClearFlag(USART1, USART_FLAG_TC);     /* 清发送外成标志，Transmission Complete flag */

}
rt_err_t rs485_input(rt_device_t dev; rt_size_t size)
{
	struct rx_msg msg;
	msg.dev = dev;
	msg.size = size;
	
	rt_mq_send(rx_mq,&msg,sizeof(struct rx_msg));
	
	return RT_EOK;
}

void RS485_set_device(const char *name)
{
    rt_device_t dev = RT_NULL;

    //RT_ASSERT(rs485_device != RT_NULL);

	dev = rt_device_find(name);

	RT_ASSERT(dev != RT_NULL);

	if (dev == RT_NULL)
    {
        rt_kprintf("finsh: can not find device: %s\n", name);
        return;
    }
            
	if (rt_device_open(dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX) == RT_EOK)
    {
        if (rs485_device != RT_NULL)
        {
            /* close old finsh device */
            rt_device_close(rs485_device);
            rt_device_set_rx_indicate(dev, RT_NULL);
        }

        rs485_device = dev;
        rt_device_set_rx_indicate(dev, rs485_input);
    }

}

//RS485发送len个字节.
//buf:发送区首地址
//len:发送的字节数(为了和本代码的接收匹配,这里建议不要超过64个字节)
void RS485_Send_Data(const char *name, rt_size_t length)
{
    
    //rt_size_t length;
    rt_uint16_t old_flag;

   if (name == "uart1")
	{
		PA1_RS485_TX_EN;  //为发送模式
		
	}
	else if (name == "uart2")
	{
		PB8_RS485_TX_EN;        //为发送模式
	}
	else if (name == "uart3")
	{
		PC9_RS485_TX_EN;        //为发送模式
		}
	else
	{
		return;
	}
	

    
    length = 1;

	

    rs485_device->flag |= RT_DEVICE_FLAG_STREAM;
    //rt_device_write(rs485_device, 0, rs485_device_TX_buf, length);
	rt_device_write(rs485_device, 0, rs485_device_TX_buf, length);
    rs485_device->flag = old_flag;

	if (name == "uart1")
	{
		PA1_RS485_RX_EN;  //为接收模式
		
	}
	else if (name == "uart2")
	{
		PB8_RS485_RX_EN;        //为接收模式
	}
	else if (name == "uart3")
	{
		PC9_RS485_RX_EN;        //为接收模式
		}
	else
	{
		return;
	}

}
//RS485查询接收到的数据
//buf:接收缓存首地址
//len:读到的数据长度
void RS485_Receive_Data(const char *name, u8 *buf, rt_size_t length)
{
    rt_uint16_t old_flag;

    rs485_device = rt_device_find(name);
    old_flag = rs485_device->flag;

    rs485_device->flag |= RT_DEVICE_FLAG_INT_RX;
    rt_device_read(rs485_device, 0, buf, length);
    rs485_device->flag = old_flag;
}


void rt_hw_RS485_init(void)
{
    RS485_GPIO_Config();
	RS485_set_device("uart2");
}
void RS485Poll(void)
{
rs485_device1_TX_flag=1;
//rs485_device_TX_buf[5] = { 0, 1, 2, 3, 4};
    if (rs485_device1_TX_flag)
    {
        RS485_Send_Data("uart2", 5);

#ifdef RT_USING_FINSH
        rt_kprintf("RS485 by uart1 transmit message.\n");
#endif

    }
    if (rs485_device2_TX_flag)
    {
        //RS485_Send_Data("serial2", 2);
    }
#if 0
    if (rs485_device3_TX_flag)
    {
        RS485_Send_Data("serial3", 3);
    }
#endif
}
