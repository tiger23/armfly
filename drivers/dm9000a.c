/*
 * File      : dm9000a.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-07-01     Bernard      the first version
 */
#include <rtthread.h>
#include "dm9000a.h"

#include <netif/ethernetif.h>
#include <lwip/netif.h>
#include "lwipopts.h"
#include "stm32f10x.h"
#include "stm32f10x_fsmc.h"

#define DM9000_DEBUG      1
#if DM9000_DEBUG
#define DM9000_TRACE    rt_kprintf
#else
#define DM9000_TRACE(...)
#endif

/*
 * DM9000 interrupt line is connected to PA1
 */
//--------------------------------------------------------

#define DM9000_PHY          0x40    /* PHY address 0x01 */


#define MAX_ADDR_LEN 6

enum DM9000_PHY_mode
{
    DM9000_10MHD = 0, DM9000_100MHD = 1,
    DM9000_10MFD = 4, DM9000_100MFD = 5,
    DM9000_AUTO  = 8, DM9000_1M_HPNA = 0x10
};

enum DM9000_TYPE
{
    TYPE_DM9000E,
    TYPE_DM9000A,
    TYPE_DM9000B
};

struct rt_dm9000_eth
{
    /* inherit from ethernet device */
    struct eth_device parent;

    enum DM9000_TYPE type;
    enum DM9000_PHY_mode mode;

    rt_uint8_t imr_all;

    rt_uint8_t packet_cnt;                  /* packet I or II */
    rt_uint16_t queue_packet_len;           /* queued packet (packet II) */

    /* interface address info. */
    rt_uint8_t  dev_addr[MAX_ADDR_LEN];     /* hw address   */
};
static struct rt_dm9000_eth dm9000_device;
static struct rt_semaphore sem_ack, sem_lock;
rt_uint8_t  multicast_addr[8];

void rt_dm9000_isr(void);

static void delay_ms(rt_uint32_t ms)
{
    rt_uint32_t len;
    for (; ms > 0; ms --)
        for (len = 0; len < 100; len++);
}

/*******************************************************************************
*   函数名: dm9k_udelay
*   参  数: time ： 延迟时间，不精确，us级别
*   返  回: 无
*   功  能: 延迟函数
*/
void dm9k_udelay(uint16_t time)
{
    uint16_t i, k;

    for (i = 0; i < time; i++)
    {
        for (k = 0; k < 80; k++);
    }
    while (time--);
}

/* Read a byte from I/O port */
rt_inline rt_uint8_t dm9000_reg_read(rt_uint16_t reg)
{
    DM9000_REG_ADDR = reg;
    return (rt_uint8_t) DM9000_REG_DATA;
}

/* Write a byte to I/O port */
rt_inline void dm9000_reg_write(rt_uint16_t reg, rt_uint16_t writedata)
{
    DM9000_REG_ADDR = reg;
    DM9000_REG_DATA = writedata;
}
/*******************************************************************************
*   函数名: dm9k_reset
*   参  数: 无
*   返  回: 无
*   功  能: 对DM9000AE进行软件复位
*/
void dm9000_reset(void)
{
    dm9000_reg_write(DM9000_REG_NCR, DM9000_REG_RESET);          /* 对 DM9000A 进行软件重置 */
    dm9k_udelay(10);                                /* delay 10us */
    dm9000_reg_write(DM9000_REG_NCR, DM9000_REG_RESET);          /* 对 DM9000A 进行软件重置 */
    dm9k_udelay(10);                                /* delay 10us */

    /* 基本记存器相关设置 */
    dm9000_reg_write(DM9000_REG_IMR, DM9000_IMR_OFF);            /* 开启内存自环模式 */
    dm9000_reg_write(DM9000_REG_TCR2, DM9000_TCR2_SET);          /* 设置 LED 显示模式1:全双工亮，半双工灭 */

    /* 清除多余资讯 */
    dm9000_reg_write(DM9000_REG_NSR, 0x2c);
    dm9000_reg_write(DM9000_REG_TCR, 0x00);
    dm9000_reg_write(DM9000_REG_ISR, 0x3f);

#ifdef DM9000A_FLOW_CONTROL
    dm9000_reg_write(DM9000_REG_BPTR, DM9000_BPTR_SET);          /* 半双工流控设置 */
    dm9000_reg_write(DM9000_REG_FCTR, DM9000_FCTR_SET);          /* 全双工流控设置 */
    dm9000_reg_write(DM9000_REG_FCR, DM9000_FCR_SET);            /* 开启流控设置 */
#endif

#ifdef DM9000A_UPTO_100M
    /* DM9000A无此寄存器 */
    dm9000_reg_write(DM9000_REG_OTCR, DM9000_OTCR_SET);          /* 工作频率到 100Mhz 设置 */
#endif

#ifdef  Rx_Int_enable
    dm9000_reg_write(DM9000_REG_IMR, DM9000_IMR_SET);            /* 开启 中断模式 */
#else
    dm9000_reg_write(DM9000_REG_IMR, DM9000_IMR_OFF);            /* 关闭 中断模式 */
#endif

    dm9000_reg_write(DM9000_REG_RCR, DM9000_RCR_SET);            /* 开启 接收工能 */


}

/* Read a word from phyxcer */
rt_inline rt_uint16_t dm9000_phy_read(rt_uint16_t reg)
{
    rt_uint16_t val;

    /* Fill the phyxcer register into REG_0C */
    dm9000_reg_write(DM9000_REG_EPAR, DM9000_PHY | reg);
    dm9000_reg_write(DM9000_REG_EPCR, 0xc);  /* Issue phyxcer read command */

    delay_ms(100);      /* Wait read complete */

    dm9000_reg_write(DM9000_REG_EPCR, 0x0);  /* Clear phyxcer read command */
    val = (dm9000_reg_read(DM9000_REG_EPDRH) << 8) | dm9000_reg_read(DM9000_REG_EPDRL);

    return val;
}

/* Write a word to phyxcer */
rt_inline void dm9000_phy_write(rt_uint16_t phy_reg, rt_uint16_t value)
{
    /* Fill the phyxcer register into REG_0C */
    dm9000_reg_write(DM9000_REG_EPAR, DM9000_PHY | phy_reg);

    /* Fill the written data into REG_0D & REG_0E */
    dm9000_reg_write(DM9000_REG_EPDRL, (value & 0xff));
    dm9000_reg_write(DM9000_REG_EPDRH, ((value >> 8) & 0xff));
    dm9000_reg_write(DM9000_REG_EPCR, 0xa);  /* Issue phyxcer write command */

    delay_ms(500);      /* Wait write complete */

    //dm9000_reg_write(DM9000_REG_EPCR, 0x0);    /* Clear phyxcer write command */
    while (dm9000_reg_read(DM9000_REG_EPCR) & 0x01);                /* 查寻是否执行结束 */
    dm9000_reg_write(DM9000_REG_EPCR, 0x08);                         /* 清除写入命令 */
}

/* Set PHY operationg mode */
rt_inline void phy_mode_set(rt_uint32_t media_mode)
{
    rt_uint16_t phy_reg4 = 0x01e1, phy_reg0 = 0x1000;
    if (!(media_mode & DM9000_AUTO))
    {
        switch (media_mode)
        {
        case DM9000_10MHD:
            phy_reg4 = 0x21;
            phy_reg0 = 0x0000;
            break;
        case DM9000_10MFD:
            phy_reg4 = 0x41;
            phy_reg0 = 0x1100;
            break;
        case DM9000_100MHD:
            phy_reg4 = 0x81;
            phy_reg0 = 0x2000;
            break;
        case DM9000_100MFD:
            phy_reg4 = 0x101;
            phy_reg0 = 0x3100;
            break;
        }
        dm9000_phy_write(4, phy_reg4); /* Set PHY media mode */
        dm9000_phy_write(0, phy_reg0); /*  Tmp */
    }

    dm9000_reg_write(DM9000_REG_GPCR, 0x01); /* Let GPIO0 output */
    dm9000_reg_write(DM9000_REG_GPR, 0x00);  /* Enable PHY */
}

/* interrupt service routine */
void rt_dm9000_isr()
{
    rt_uint16_t isr_status;
    rt_uint16_t save_reg;

    save_reg = DM9000_REG_ADDR;

    /* Disable all interrupts */
    dm9000_reg_write(DM9000_REG_IMR, IMR_PAR);

    /* Got DM9000 interrupt status */
    isr_status = dm9000_reg_read(DM9000_REG_ISR);               /* Got ISR */
    dm9000_reg_write(DM9000_REG_ISR, isr_status);    /* Clear ISR status */

    DM9000_TRACE("dm9000 isr: int status %04x\n", isr_status);

    /* receive overflow */
    if (isr_status & ISR_ROS)
    {
        rt_kprintf("overflow\n");
    }

    if (isr_status & ISR_ROOS)
    {
        rt_kprintf("overflow counter overflow\n");
    }

    /* Received the coming packet */
    if (isr_status & ISR_PRS)
    {
        /* disable receive interrupt */
        dm9000_device.imr_all = IMR_PAR | IMR_PTI;

        /* a frame has been received */
        eth_device_ready(&(dm9000_device.parent));
    }

    /* Transmit Interrupt check */
    if (isr_status & ISR_PTS)
    {
        /* transmit done */
        int tx_status = dm9000_reg_read(DM9000_REG_NSR);    /* Got TX status */

        if (tx_status & (NSR_TX2END | NSR_TX1END))
        {
            dm9000_device.packet_cnt --;
            if (dm9000_device.packet_cnt > 0)
            {
                DM9000_TRACE("dm9000 isr: tx second packet\n");

                /* transmit packet II */
                /* Set TX length to DM9000 */
                dm9000_reg_write(DM9000_REG_TXPLL, dm9000_device.queue_packet_len & 0xff);
                dm9000_reg_write(DM9000_REG_TXPLH, (dm9000_device.queue_packet_len >> 8) & 0xff);

                /* Issue TX polling command */
                dm9000_reg_write(DM9000_REG_TCR, TCR_TXREQ); /* Cleared after TX complete */
            }

            /* One packet sent complete */
            rt_sem_release(&sem_ack);
        }
    }

    /* Re-enable interrupt mask */
    dm9000_reg_write(DM9000_REG_IMR, dm9000_device.imr_all);

    DM9000_REG_ADDR = save_reg;
}
void mac_set(void)
{
    int i, oft;

    for (i = 0, oft = DM9000_REG_PAR; i < 6; i++, oft++)
        dm9000_reg_write(oft, dm9000_device.dev_addr[i]);
}
void gpio_set(unsigned char s_gpio , unsigned char gp_io)
{
    if (gp_io == 0x01)
        dm9000_reg_write(DM9000_REG_GPCR , dm9000_reg_read(DM9000_REG_GPCR) & ~(0x01 << s_gpio));
    else
        dm9000_reg_write(DM9000_REG_GPCR , dm9000_reg_read(DM9000_REG_GPCR) | (0x01 << s_gpio));
}
void gpio_w(unsigned char s_gpio , unsigned char gp_hl)
{
    if (gp_hl == 0x01)
        dm9000_reg_write(DM9000_REG_GPR , dm9000_reg_read(DM9000_REG_GPR) | (0x01 << s_gpio));
    else
        dm9000_reg_write(DM9000_REG_GPR , dm9000_reg_read(DM9000_REG_GPR) & ~(0x01 << s_gpio));
}
unsigned char gpio_r(unsigned char s_gpio)
{
    return ((dm9000_reg_read(DM9000_REG_GPR) >> s_gpio) & 0x01);
}
/*-----------------------------------------------------------------------------
  设置 DM9000A MAC 、 广播 、 多播 寄存器
  -----------------------------------------------------------------------------
  返回         无
  -----------------------------------------------------------------------------*/
void dm9000_hash_table(void)
{
	rt_uint8_t i;

	/* 将MAC地址告诉lwip */
	for (i = 0; i < 6; i++)
	{
		netif_default->hwaddr[i] = dm9000_device.dev_addr[i];
	}

	/* 设置 网卡 MAC 位置，来自於 MyHardware */
	mac_set();
	
	for(i = 0; i < 8; i++) 								/* 清除 网卡多播设置 */
		dm9000_reg_write(DM9000_REG_MAR + i, 0x00);
	dm9000_reg_write(DM9000_REG_MAR + 7, 0x80);  					/* 速设置 广播包 设置 */
}
/* RT-Thread Device Interface */
/* initialize the interface */
static rt_err_t rt_dm9000_init(rt_device_t dev)
{
    int i, oft, lnk;
    rt_uint32_t value;
    unsigned char IO_chk;

    /*power on the internal PHY*/
    dm9000_reg_write(DM9000_REG_GPR, 0x00);

    /* RESET device */
    dm9000_reg_write(DM9000_REG_NCR, 0x03);
    delay_ms(1000);     /* delay 1ms */
    dm9000_reg_write(DM9000_REG_NCR, 0x00);
    dm9000_reg_write(DM9000_REG_NCR, 0x03);
    delay_ms(1000);     /* delay 1ms */

    dm9000_reg_write(DM9000_REG_NCR, 0x03);
    delay_ms(1000);     /* delay 1ms */
    dm9000_reg_write(DM9000_REG_NCR, 0x00);
    dm9000_reg_write(DM9000_REG_NCR, 0x03);
    delay_ms(1000);     /* delay 1ms */
    	
	//dm9000_hash_table();/*设置 DM9000A MAC 、 广播 、 多播 寄存器*/
	
    //dm9000_reset();

    /*Program the NCR register*/
    dm9000_reg_write(DM9000_REG_NCR, 0x00);

    dm9000_reg_write(DM9000_REG_IMR, 0x80);/*关闭中断*/
#ifdef 0
    dm9000_reg_write(0x2df, 0x80);
    gpio_set(0x00 , 0x01);
    gpio_w(0x00 , 0x00);
    dm9000_phy_write(0x00 , 0x8000); /*重置PHY*/
#endif
    /* set mac address */
    mac_set();

    /* set multicast address */
    //for (i = 0, oft = DM9000_REG_MAR; i < 8; i++, oft++)
    //    dm9000_reg_write(oft, 0xff);

    /*Clear TX&INTR status*/
    dm9000_reg_read(DM9000_REG_NSR);
    dm9000_reg_read(DM9000_REG_ISR);
    dm9000_reg_write(DM9000_REG_NSR, NSR_TX1END | NSR_TX2END);
    dm9000_reg_write(DM9000_REG_TCR, 0x00);
    dm9000_reg_write(DM9000_REG_ISR, 0x3f);

#ifdef 0
    dm9000_reg_write(DM9000_REG_TCR, 0x00);
    dm9000_reg_read(DM9000_REG_ROCR);
    dm9000_reg_write(DM9000_REG_FCR, DM9000_FCR_SET);
    dm9000_phy_write(DM9000_PHY, 0x05e1);
    dm9000_reg_write(DM9000_REG_ISR, 0xff);

    dm9000_reg_write(DM9000_REG_IMR, 0x83);

    dm9000_reg_write(DM9000_REG_RCR , DM9000_RCR_SET);
#endif
    /* identfy DM9000 */
    value  = dm9000_reg_read(DM9000_REG_VIDL);
    value |= dm9000_reg_read(DM9000_REG_VIDH) << 8;
    value |= dm9000_reg_read(DM9000_REG_PIDL) << 16;
    value |= dm9000_reg_read(DM9000_REG_PIDH) << 24;
    if (value == DM9000_ID)
    {
        rt_kprintf("dm9000 id: 0x%x\n", value);
    }
    else
    {
        return -RT_ERROR;
    }

    /* GPIO0 on pre-activate PHY */
//    dm9000_reg_write(DM9000_REG_GPR, 0x00);	            /* REG_1F bit0 activate phyxcer */
//    dm9000_reg_write(DM9000_REG_GPCR, GPCR_GEP_CNTL);    /* Let GPIO0 output */
//    dm9000_reg_write(DM9000_REG_GPR, 0x00);                 /* Enable PHY */

    /* Set PHY */
    phy_mode_set(dm9000_device.mode);

    /* Program operating register */
    dm9000_reg_write(DM9000_REG_NCR, 0x0);	/* only intern phy supported by now */
    dm9000_reg_write(DM9000_REG_TCR, 0);	    /* TX Polling clear */
    dm9000_reg_write(DM9000_REG_BPTR, 0x3f);	/* Less 3Kb, 200us */
    dm9000_reg_write(DM9000_REG_FCTR, FCTR_HWOT(3) | FCTR_LWOT(8));	/* Flow Control : High/Low Water */
    dm9000_reg_write(DM9000_REG_FCR, 0x0);	/* SH FIXME: This looks strange! Flow Control */
    dm9000_reg_write(DM9000_REG_SMCR, 0);	/* Special Mode */
    dm9000_reg_write(DM9000_REG_NSR, NSR_WAKEST | NSR_TX2END | NSR_TX1END);	/* clear TX status */
    dm9000_reg_write(DM9000_REG_ISR, 0x0f);	/* Clear interrupt status */
    dm9000_reg_write(DM9000_REG_TCR2, 0x80);	/* Switch LED to mode 1 */

    /* set mac address */
    for (i = 0, oft = 0x10; i < 6; i++, oft++)
        dm9000_reg_write(oft, dm9000_device.dev_addr[i]);
    /* set multicast address */
    for (i = 0, oft = 0x16; i < 8; i++, oft++)
        dm9000_reg_write(oft, 0xff);

    /* Activate DM9000 */
    dm9000_reg_write(DM9000_REG_RCR, RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN);	/* RX enable */
    dm9000_reg_write(DM9000_REG_IMR, IMR_PAR);

	if (dm9000_device.mode == DM9000_AUTO)
	{
	    while (!(dm9000_phy_read(1) & 0x20))
	    {
	        /* autonegation complete bit */
	        rt_thread_delay(10);
	        i++;
	        if (i == 10000)
	        {
	            rt_kprintf("could not establish link\n");
	            return 0;
	        }
	    }
	}

    /* see what we've got */
    lnk = dm9000_phy_read(PHY_DSCSR) >> 12;
    rt_kprintf("operating at ");
    switch (lnk)
    {
    case 1:
        rt_kprintf("10M half duplex ");
        break;
    case 2:
        rt_kprintf("10M full duplex ");
        break;
    case 4:
        rt_kprintf("100M half duplex ");
        break;
    case 8:
        rt_kprintf("100M full duplex ");
        break;
    default:
        rt_kprintf("unknown: %d ", lnk);
        break;
    }
    rt_kprintf("mode\n");

    dm9000_reg_write(DM9000_REG_IMR, dm9000_device.imr_all); /* Enable TX/RX interrupt mask */

    dm9000_reg_write(DM9000_REG_RCR, RCR_RXEN); /* Enable RX  */

    return RT_EOK;
}

static rt_err_t rt_dm9000_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

static rt_err_t rt_dm9000_close(rt_device_t dev)
{
    /* RESET devie */
    dm9000_phy_write(0, 0x8000);   /* PHY RESET */
    dm9000_reg_write(DM9000_REG_GPR, 0x01);  /* Power-Down PHY */
    dm9000_reg_write(DM9000_REG_IMR, 0x80);  /* Disable all interrupt */
    dm9000_reg_write(DM9000_REG_RCR, 0x00);  /* Disable RX */

    return RT_EOK;
}

static rt_size_t rt_dm9000_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_size_t rt_dm9000_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_err_t rt_dm9000_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    switch (cmd)
    {
    case NIOCTL_GADDR:
        /* get mac address */
        if (args) rt_memcpy(args, dm9000_device.dev_addr, 6);
        else return -RT_ERROR;
        break;

    default :
        break;
    }

    return RT_EOK;
}

/* ethernet device interface */
/* transmit packet. */
rt_err_t rt_dm9000_tx(rt_device_t dev, struct pbuf *p)
{
    DM9000_TRACE("dm9000 tx: %d\n", p->tot_len);

    /* lock DM9000 device */
    rt_sem_take(&sem_lock, RT_WAITING_FOREVER);

    /* disable dm9000a interrupt */
    dm9000_reg_write(DM9000_REG_IMR, IMR_PAR);

    /* Move data to DM9000 TX RAM */
    DM9000_outb(DM9000_REG_ADDR_BASE, DM9000_REG_MWCMD);

    {
        /* q traverses through linked list of pbuf's
         * This list MUST consist of a single packet ONLY */
        struct pbuf *q;
        rt_uint16_t pbuf_index = 0;
        rt_uint8_t word[2], word_index = 0;

        q = p;
        /* Write data into dm9000a, two bytes at a time
         * Handling pbuf's with odd number of bytes correctly
         * No attempt to optimize for speed has been made */
        while (q)
        {
            if (pbuf_index < q->len)
            {
                DM9000_TRACE("%02x ", ((u8_t*)q->payload)[pbuf_index]);
                word[word_index++] = ((u8_t *)q->payload)[pbuf_index++];
                if (word_index == 2)
                {
                    DM9000_outw(DM9000_REG_DATA_BASE, (word[1] << 8) | word[0]);
                    word_index = 0;
                }
            }
            else
            {
                q = q->next;
                pbuf_index = 0;
            }
        }
        /* One byte could still be unsent */
        if (word_index == 1)
        {
            DM9000_outw(DM9000_REG_DATA_BASE, word[0]);
			DM9000_TRACE("%02x ", word[0]);
        }
		DM9000_TRACE("\n");
    }

    if (dm9000_device.packet_cnt == 0)
    {
        DM9000_TRACE("dm9000 tx: first packet\n");

        dm9000_device.packet_cnt ++;
        /* Set TX length to DM9000 */
        dm9000_reg_write(DM9000_REG_TXPLL, p->tot_len & 0xff);
        dm9000_reg_write(DM9000_REG_TXPLH, (p->tot_len >> 8) & 0xff);

        /* Issue TX polling command */
        dm9000_reg_write(DM9000_REG_TCR, TCR_TXREQ); /* Cleared after TX complete */
    }
    else
    {
        DM9000_TRACE("dm9000 tx: second packet\n");

        dm9000_device.packet_cnt ++;
        dm9000_device.queue_packet_len = p->tot_len;
    }

    /* enable dm9000a interrupt */
    dm9000_reg_write(DM9000_REG_IMR, dm9000_device.imr_all);

    /* unlock DM9000 device */
    rt_sem_release(&sem_lock);

    /* wait ack */
    rt_sem_take(&sem_ack, RT_WAITING_FOREVER);

    DM9000_TRACE("dm9000 tx done\n");

    return RT_EOK;
}

/* reception packet. */
struct pbuf *rt_dm9000_rx(rt_device_t dev)
{
    struct pbuf *p;
    rt_uint32_t rxbyte;

    /* init p pointer */
    p = RT_NULL;

    /* lock DM9000 device */
    rt_sem_take(&sem_lock, RT_WAITING_FOREVER);

    /* Check packet ready or not */
    dm9000_reg_read(DM9000_REG_MRCMDX);              /* Dummy read */
    rxbyte = DM9000_inb(DM9000_REG_DATA_BASE);      /* Got most updated data */
    if (rxbyte)
    {
        rt_uint16_t rx_status, rx_len;
        rt_uint16_t *data;

        if (rxbyte > 1)
        {
            DM9000_TRACE("dm9000 rx: rx error, stop device\n");

            dm9000_reg_write(DM9000_REG_RCR, 0x00);  /* Stop Device */
            dm9000_reg_write(DM9000_REG_ISR, 0x80);  /* Stop INT request */
        }

        /* A packet ready now  & Get status/length */
        DM9000_outb(DM9000_REG_ADDR_BASE, DM9000_REG_MRCMD);

        rx_status = DM9000_inw(DM9000_REG_DATA_BASE);
        rx_len = DM9000_inw(DM9000_REG_DATA_BASE);

        DM9000_TRACE("dm9000 rx: status %04x len %d\n", rx_status, rx_len);

        /* allocate buffer */
        p = pbuf_alloc(PBUF_LINK, rx_len, PBUF_RAM);
        if (p != RT_NULL)
        {
            struct pbuf *q;
            rt_int32_t len;

            for (q = p; q != RT_NULL; q = q->next)
            {
                data = (rt_uint16_t *)q->payload;
                len = q->len;

                while (len > 0)
                {
                    *data = DM9000_inw(DM9000_REG_DATA_BASE);
                    data ++;
                    len -= 2;
                }
            }
            DM9000_TRACE("\n");
        }
        else
        {
            rt_uint16_t dummy;
            rt_int32_t len;

            DM9000_TRACE("dm9000 rx: no pbuf\n");

            /* no pbuf, discard data from DM9000 */
            data = &dummy;
            len = rx_len;
            while (len > 0)
            {
                *data = DM9000_inw(DM9000_REG_DATA_BASE);
                len -= 2;
            }
        }

        if ((rx_status & 0xbf00) || (rx_len < 0x40)
                || (rx_len > DM9000_PKT_MAX))
        {
            rt_kprintf("rx error: status %04x\n", rx_status);

            if (rx_status & 0x100)
            {
                rt_kprintf("rx fifo error\n");
            }
            if (rx_status & 0x200)
            {
                rt_kprintf("rx crc error\n");
            }
            if (rx_status & 0x8000)
            {
                rt_kprintf("rx length error\n");
            }
            if (rx_len > DM9000_PKT_MAX)
            {
                rt_kprintf("rx length too big\n");

                /* RESET device */
                dm9000_reg_write(DM9000_REG_NCR, NCR_RST);
                rt_thread_delay(1); /* delay 5ms */
            }

            /* it issues an error, release pbuf */
            pbuf_free(p);
            p = RT_NULL;
        }
    }
    else
    {
        /* restore receive interrupt */
        dm9000_device.imr_all = IMR_PAR | IMR_PTI | IMR_PRI;
        dm9000_reg_write(DM9000_REG_IMR, dm9000_device.imr_all);
    }

    /* unlock DM9000 device */
    rt_sem_release(&sem_lock);

    return p;
}

static void RCC_Configuration(void)
{
    /* enable gpiob port clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);
    /* enable FSMC clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
}

static void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    /* Enable the EXTI1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

static void GPIO_Configuration()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    /* configure PA1 as external interrupt */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Connect DM9000 EXTI Line to GPIOA Pin 1 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);

    /* Configure DM9000 EXTI Line to generate an interrupt on falling edge */
    EXTI_InitStructure.EXTI_Line    = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Clear DM9000A EXTI line pending bit */
    EXTI_ClearITPendingBit(EXTI_Line1);
}

static void FSMC_Configuration()
{
    FSMC_NORSRAMInitTypeDef FSMC_NORSRAMInitStructure;
    FSMC_NORSRAMTimingInitTypeDef p;

    /* FSMC GPIO configure */
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF
                               | RCC_APB2Periph_GPIOG, ENABLE);
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);

        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

        /*
        FSMC_D0 ~ FSMC_D3
        PD14 FSMC_D0   PD15 FSMC_D1   PD0  FSMC_D2   PD1  FSMC_D3
        */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_14 | GPIO_Pin_15;
        GPIO_Init(GPIOD, &GPIO_InitStructure);

        /*
        FSMC_D4 ~ FSMC_D12
        PE7 ~ PE15  FSMC_D4 ~ FSMC_D12
        */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10
                                      | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
        GPIO_Init(GPIOE, &GPIO_InitStructure);

        /* FSMC_D13 ~ FSMC_D15   PD8 ~ PD10 */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
        GPIO_Init(GPIOD, &GPIO_InitStructure);

        /*
        FSMC_A0 ~ FSMC_A5   FSMC_A6 ~ FSMC_A9
        PF0     ~ PF5       PF12    ~ PF15
        */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3
                                      | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
        GPIO_Init(GPIOF, &GPIO_InitStructure);

        /* FSMC_A10 ~ FSMC_A15  PG0 ~ PG5 */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
        GPIO_Init(GPIOG, &GPIO_InitStructure);

        /* FSMC_A16 ~ FSMC_A18  PD11 ~ PD13 */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
        GPIO_Init(GPIOD, &GPIO_InitStructure);

        /* FSMC_A19 ~ FSMC_A22  PE3 ~ PE6 */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
        GPIO_Init(GPIOE, &GPIO_InitStructure);

        /* RD-PD4 WR-PD5 */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
        GPIO_Init(GPIOD, &GPIO_InitStructure);

        /* NBL0-PE0 NBL1-PE1 */
        //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
        //GPIO_Init(GPIOE,&GPIO_InitStructure);

        /* NE1/NCE2 */
        //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
        //GPIO_Init(GPIOD,&GPIO_InitStructure);
        /* NE2 */
        //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
        //GPIO_Init(GPIOG,&GPIO_InitStructure);
        /* NE3 */
        //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
        //GPIO_Init(GPIOG,&GPIO_InitStructure);
        /* NE4 */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
        GPIO_Init(GPIOG, &GPIO_InitStructure);
    }
    /* FSMC GPIO configure */

    /*-- FSMC Configuration ------------------------------------------------------*/
    p.FSMC_AddressSetupTime      = 0;
    p.FSMC_AddressHoldTime       = 0;
    p.FSMC_DataSetupTime         = 4;
    p.FSMC_BusTurnAroundDuration = 0;
    p.FSMC_CLKDivision           = 0;
    p.FSMC_DataLatency           = 0;
    p.FSMC_AccessMode            = FSMC_AccessMode_A;

    FSMC_NORSRAMInitStructure.FSMC_Bank                  = FSMC_Bank1_NORSRAM4;
    FSMC_NORSRAMInitStructure.FSMC_DataAddressMux        = FSMC_DataAddressMux_Disable;
    FSMC_NORSRAMInitStructure.FSMC_MemoryType            = FSMC_MemoryType_SRAM;
    FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth       = FSMC_MemoryDataWidth_16b;
    FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode       = FSMC_BurstAccessMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait      = FSMC_AsynchronousWait_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity    = FSMC_WaitSignalPolarity_Low;
    FSMC_NORSRAMInitStructure.FSMC_WrapMode              = FSMC_WrapMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive      = FSMC_WaitSignalActive_BeforeWaitState;
    FSMC_NORSRAMInitStructure.FSMC_WriteOperation        = FSMC_WriteOperation_Enable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignal            = FSMC_WaitSignal_Disable;
    FSMC_NORSRAMInitStructure.FSMC_ExtendedMode          = FSMC_ExtendedMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WriteBurst            = FSMC_WriteBurst_Disable;
    FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;
    FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct     = &p;

    FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);

    /* Enable FSMC Bank1_SRAM Bank4 */
    FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);
}

/*******************************************************************************
*   函数名: dm9k_debug_test
*   参  数: 无
*   返  回: 无
*   功  能: 测试DM9000AE的函数,用于排错
*/
void dm9000_debug_test(void)
{
    uint32_t check_device;
    uint8_t  check_iomode;
    uint8_t  check_reg_fail = 0;
    uint8_t  check_fifo_fail = 0;
    uint16_t i;
    uint16_t j;

    dm9000_reg_write(DM9000_REG_NCR, DM9000_REG_RESET);          /* 对 DM9000A 进行软件重置 */
    dm9k_udelay(10);                                /* delay 10us */
    dm9000_reg_write(DM9000_REG_NCR, DM9000_REG_RESET);          /* 对 DM9000A 进行软件重置 */
    dm9k_udelay(10);                                /* delay 10us */

    check_device  = dm9000_reg_read(DM9000_REG_VIDL);
    check_device |= dm9000_reg_read(DM9000_REG_VIDH) << 8;
    check_device |= dm9000_reg_read(DM9000_REG_PIDL) << 16;
    check_device |= dm9000_reg_read(DM9000_REG_PIDH) << 24;

    if (check_device != 0x90000A46)
    {
        rt_kprintf("DM9K_DEBUG ==> DEIVCE NOT FOUND, SYSTEM HOLD !!\n");
        // while (1);
    }
    else
    {
        rt_kprintf("DM9K_DEBUG ==> DEIVCE FOUND !!\n");
    }

    check_iomode = dm9000_reg_read(DM9000_REG_ISR) >> 6;
    if (check_iomode != DM9000_WORD_MODE)
    {
        rt_kprintf("DM9K_DEBUG ==> DEIVCE NOT WORD MODE, SYSTEM HOLD !!\n");
        // while (1);
    }
    else
    {
        rt_kprintf("DM9K_DEBUG ==> DEIVCE IS WORD MODE !!\n");
    }

    rt_kprintf("DM9K_DEBUG ==> REGISTER R/W TEST !!\n");
    DM9000_REG_ADDR = DM9000_REG_MAR;
    for (i = 0; i < 0x0100; i++)
    {
        DM9000_REG_DATA = i;
        if (i != (DM9000_REG_DATA & 0xff))
        {
            rt_kprintf("             > error W %02x , R %02x \n", i , DM9000_REG_DATA);
            check_reg_fail = 1;
        }
    }

    if (check_reg_fail)
    {
        rt_kprintf("DM9K_DEBUG ==> REGISTER R/W FAIL, SYSTEM HOLD !!\n");
        // while (1);
    }

    rt_kprintf("DM9K_DEBUG ==> FIFO R/W TEST !!\n");
    rt_kprintf("DM9K_DEBUG ==> FIFO WRITE START POINT 0x%02x%02x \n",
               dm9000_reg_read(DM9000_REG_MWRH), dm9000_reg_read(DM9000_REG_MWRL));

    DM9000_REG_ADDR = DM9000_REG_MWCMD;
    for (i = 0; i < 0x1000; i++)
        DM9000_REG_DATA = ((i & 0xff) * 0x0101);

    rt_kprintf("DM9K_DEBUG ==> FIFO WRITE END POINT 0x%02x%02x \n",
               dm9000_reg_read(DM9000_REG_MWRH), dm9000_reg_read(DM9000_REG_MWRL));

    if ((dm9000_reg_read(DM9000_REG_MWRH) != 0x20) || (dm9000_reg_read(DM9000_REG_MWRL) != 0x00))
    {
        rt_kprintf("DM9K_DEBUG ==> FIFO WRITE FAIL, SYSTEM HOLD !!\n");
        // while (1);
    }

    dm9000_reg_read(DM9000_REG_MRCMDX);
    rt_kprintf("DM9K_DEBUG ==> FIFO READ START POINT 0x%02x%02x \n",
               dm9000_reg_read(DM9000_REG_MRRH), dm9000_reg_read(DM9000_REG_MRRL));
    dm9000_reg_read(DM9000_REG_MRCMDX);

    DM9000_REG_ADDR = DM9000_REG_MRCMD;
    for (i = 0; i < 0x1000; i++)
    {
        j = DM9000_REG_DATA;

        if (((i & 0xff) * 0x0101) != j)
        {
            //rt_kprintf("             > error W %04x , R %04x \n",
            //      ((i & 0xff) * 0x0101) , j);
            check_fifo_fail = 1;
        }
    }

    rt_kprintf("DM9K_DEBUG ==> FIFO READ END POINT 0x%02x%02x \n",
               dm9000_reg_read(DM9000_REG_MRRH), dm9000_reg_read(DM9000_REG_MRRL));

    if ((dm9000_reg_read(DM9000_REG_MRRH) != 0x20) || (dm9000_reg_read(DM9000_REG_MRRL) != 0x00))
    {
        rt_kprintf("DM9K_DEBUG ==> FIFO WRITE FAIL, SYSTEM HOLD !!\n");
        // while (1);
    }

    if (check_fifo_fail)
    {
        rt_kprintf("DM9K_DEBUG ==> FIFO R/W DATA FAIL, SYSTEM HOLD !!\n");
        // while (1);
    }

    rt_kprintf("DM9K_DEBUG ==> PACKET SEND & INT TEST !! \n");
    dm9000_reg_write(DM9000_REG_NCR, DM9000_REG_RESET);
    dm9k_udelay(10);
    dm9000_reg_write(DM9000_REG_NCR, DM9000_REG_RESET);
    dm9k_udelay(10);

    dm9000_reg_write(DM9000_REG_IMR, DM9000_IMR_OFF | DM9000_TX_INTR);

    dm9000_reg_write(DM9000_REG_TXPLH, 0x01);
    dm9000_reg_write(DM9000_REG_TXPLL, 0x00);

    // do
    // {
    dm9000_reg_write(DM9000_REG_ISR, DM9000_TX_INTR);
    rt_kprintf("DM9K_DEBUG ==> INT PIN IS OFF\n");

    DM9000_REG_ADDR = DM9000_REG_MWCMD;
    for (i = 0; i < (0x0100 / 2); i++)
    {
        if (i < 3)
            DM9000_REG_DATA = 0xffff;
        else
            DM9000_REG_DATA = i * 0x0101;
    }

    rt_kprintf("DM9K_DEBUG ==> PACKET IS SEND \n");
    dm9000_reg_write(DM9000_REG_TCR, DM9000_TCR_SET);

    while (dm9000_reg_read(DM9000_REG_TCR) & DM9000_TCR_SET) dm9k_udelay(5);
    if (dm9000_reg_read(DM9000_REG_ISR) & DM9000_TX_INTR)
        rt_kprintf("DM9K_DEBUG ==> INT PIN IS ACTIVE \n");
    else
        rt_kprintf("DM9K_DEBUG ==> INT PIN IS NOT ACTIVE \n");

    for (i = 0; i < 10; i++)
        dm9k_udelay(1000);

    // }
    // while (1);
}
#ifdef RT_USING_FINSH
#include <finsh.h>
FINSH_FUNCTION_EXPORT(dm9000_debug_test, dm9000 debug Test);
#endif


int rt_hw_dm9000a_init(void)
{
    RCC_Configuration();
    NVIC_Configuration();
    GPIO_Configuration();
    FSMC_Configuration();

    rt_sem_init(&sem_ack, "tx_ack", 1, RT_IPC_FLAG_FIFO);
    rt_sem_init(&sem_lock, "eth_lock", 1, RT_IPC_FLAG_FIFO);

    dm9000_device.type             = TYPE_DM9000A;
    dm9000_device.mode             = DM9000_AUTO;
    dm9000_device.packet_cnt       = 0;
    dm9000_device.queue_packet_len = 0;

    /*
     * SRAM Tx/Rx pointer automatically return to start address,
     * Packet Transmitted, Packet Received
     */
    dm9000_device.imr_all = IMR_PAR | IMR_PTI | IMR_PRI;

    dm9000_device.dev_addr[0] = 0x00;
    dm9000_device.dev_addr[1] = 0x60;
    dm9000_device.dev_addr[2] = 0x6E;
    dm9000_device.dev_addr[3] = 0x11;
    dm9000_device.dev_addr[4] = 0x22;
    dm9000_device.dev_addr[5] = 0x33;

    dm9000_device.parent.parent.init       = rt_dm9000_init;
    dm9000_device.parent.parent.open       = rt_dm9000_open;
    dm9000_device.parent.parent.close      = rt_dm9000_close;
    dm9000_device.parent.parent.read       = rt_dm9000_read;
    dm9000_device.parent.parent.write      = rt_dm9000_write;
    dm9000_device.parent.parent.control    = rt_dm9000_control;
    dm9000_device.parent.parent.user_data  = RT_NULL;

    dm9000_device.parent.eth_rx     = rt_dm9000_rx;
    dm9000_device.parent.eth_tx     = rt_dm9000_tx;

    eth_device_init(&(dm9000_device.parent), "e0");

//    rt_dm9000_init();
    dm9000_debug_test();

    return 0;
}
INIT_DEVICE_EXPORT(rt_hw_dm9000a_init);


void dm9000(void)
{
    rt_kprintf("\n");
    rt_kprintf("NCR   (0x00): %02x\n", dm9000_reg_read(DM9000_REG_NCR));
    rt_kprintf("NSR   (0x01): %02x\n", dm9000_reg_read(DM9000_REG_NSR));
    rt_kprintf("TCR   (0x02): %02x\n", dm9000_reg_read(DM9000_REG_TCR));
    rt_kprintf("TSRI  (0x03): %02x\n", dm9000_reg_read(DM9000_REG_TSR1));
    rt_kprintf("TSRII (0x04): %02x\n", dm9000_reg_read(DM9000_REG_TSR2));
    rt_kprintf("RCR   (0x05): %02x\n", dm9000_reg_read(DM9000_REG_RCR));
    rt_kprintf("RSR   (0x06): %02x\n", dm9000_reg_read(DM9000_REG_RSR));
    rt_kprintf("ORCR  (0x07): %02x\n", dm9000_reg_read(DM9000_REG_ROCR));
    rt_kprintf("CRR   (0x2C): %02x\n", dm9000_reg_read(DM9000_REG_CHIPR));
    rt_kprintf("CSCR  (0x31): %02x\n", dm9000_reg_read(DM9000_REG_CSCR));
    rt_kprintf("RCSSR (0x32): %02x\n", dm9000_reg_read(DM9000_REG_RCSSR));
    rt_kprintf("ISR   (0xFE): %02x\n", dm9000_reg_read(DM9000_REG_ISR));
    rt_kprintf("IMR   (0xFF): %02x\n", dm9000_reg_read(DM9000_REG_IMR));
    rt_kprintf("\n");
}

#ifdef RT_USING_FINSH
#include <finsh.h>
FINSH_FUNCTION_EXPORT(dm9000, dm9000 register dump);
#endif
