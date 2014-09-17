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

#ifndef __DM9000_H__
#define __DM9000_H__

//#define DM9000_IO_BASE        0x6C100000
//#define DM9000_DATA_BASE  0x6C100008
#define DM9000_REG_ADDR_BASE    0x6C100000
#define DM9000_REG_DATA_BASE    0x6C100008

#define DM9000_REG_ADDR     (*((volatile rt_uint16_t *) DM9000_REG_ADDR_BASE)) /* CMD = 0寄存器地址端口*/
#define DM9000_REG_DATA     (*((volatile rt_uint16_t *) DM9000_REG_DATA_BASE)) /* CMD = 1寄存器数据端口*/

#define DM9000_inb(r)       (*(volatile rt_uint8_t *)r)
#define DM9000_outb(r, d)   (*(volatile rt_uint8_t *)r = d)
#define DM9000_inw(r)       (*(volatile rt_uint16_t *)r)
#define DM9000_outw(r, d)   (*(volatile rt_uint16_t *)r = d)

//#define    RST_1()   GPIO_SetBits(GPIOA,GPIO_Pin_1)
//#define    RST_0()   GPIO_ResetBits(GPIOA,GPIO_Pin_1)

#define DM9000_ID           0x90000A46  /* DM9000 ID */
#define DM9000_PKT_MAX      1536        /* Received packet max size */
#define DM9000_PKT_RDY      0x01        /* Packet ready to receive */

#define DM9000_REG_NCR          0x00
#define DM9000_REG_NSR          0x01
#define DM9000_REG_TCR          0x02
#define DM9000_REG_TSR1         0x03
#define DM9000_REG_TSR2         0x04
#define DM9000_REG_RCR          0x05
#define DM9000_REG_RSR          0x06
#define DM9000_REG_ROCR         0x07
#define DM9000_REG_BPTR         0x08
#define DM9000_REG_FCTR         0x09
#define DM9000_REG_FCR          0x0A
#define DM9000_REG_EPCR         0x0B
#define DM9000_REG_EPAR         0x0C
#define DM9000_REG_EPDRL        0x0D
#define DM9000_REG_EPDRH        0x0E
#define DM9000_REG_WCR          0x0F

#define DM9000_REG_PAR          0x10
#define DM9000_REG_MAR          0x16

#define DM9000_REG_GPCR         0x1e
#define DM9000_REG_GPR          0x1f
#define DM9000_REG_TRPAL        0x22
#define DM9000_REG_TRPAH        0x23
#define DM9000_REG_RWPAL        0x24
#define DM9000_REG_RWPAH        0x25

#define DM9000_REG_VIDL         0x28
#define DM9000_REG_VIDH         0x29
#define DM9000_REG_PIDL         0x2A
#define DM9000_REG_PIDH         0x2B

#define DM9000_REG_CHIPR        0x2C
#define DM9000_REG_TCR2         0x2D
#define DM9000_REG_OTCR         0x2E
#define DM9000_REG_SMCR         0x2F

#define DM9000_REG_ETCR         0x30    /* early transmit control/status register */
#define DM9000_REG_CSCR         0x31    /* check sum control register */
#define DM9000_REG_RCSSR        0x32    /* receive check sum status register */

#define DM9000_REG_MRCMDX       0xF0
#define DM9000_REG_MRCMD        0xF2
#define DM9000_REG_MRRL         0xF4
#define DM9000_REG_MRRH         0xF5
#define DM9000_REG_MWCMDX       0xF6
#define DM9000_REG_MWCMD        0xF8
#define DM9000_REG_MWRL         0xFA
#define DM9000_REG_MWRH         0xFB
#define DM9000_REG_TXPLL        0xFC
#define DM9000_REG_TXPLH        0xFD
#define DM9000_REG_ISR      0xFE
#define DM9000_REG_IMR      0xFF

#define CHIPR_DM9000A       0x19
#define CHIPR_DM9000B       0x1B

#define NCR_EXT_PHY         (1<<7)
#define NCR_WAKEEN          (1<<6)
#define NCR_FCOL            (1<<4)
#define NCR_FDX             (1<<3)
#define NCR_LBK             (3<<1)
#define NCR_RST             (1<<0)

#define NSR_SPEED           (1<<7)
#define NSR_LINKST          (1<<6)
#define NSR_WAKEST          (1<<5)
#define NSR_TX2END          (1<<3)
#define NSR_TX1END          (1<<2)
#define NSR_RXOV            (1<<1)

#define TCR_TJDIS           (1<<6)
#define TCR_EXCECM          (1<<5)
#define TCR_PAD_DIS2        (1<<4)
#define TCR_CRC_DIS2        (1<<3)
#define TCR_PAD_DIS1        (1<<2)
#define TCR_CRC_DIS1        (1<<1)
#define TCR_TXREQ           (1<<0)

#define TSR_TJTO            (1<<7)
#define TSR_LC              (1<<6)
#define TSR_NC              (1<<5)
#define TSR_LCOL            (1<<4)
#define TSR_COL             (1<<3)
#define TSR_EC              (1<<2)

#define RCR_WTDIS           (1<<6)
#define RCR_DIS_LONG        (1<<5)
#define RCR_DIS_CRC         (1<<4)
#define RCR_ALL             (1<<3)
#define RCR_RUNT            (1<<2)
#define RCR_PRMSC           (1<<1)
#define RCR_RXEN            (1<<0)

#define RSR_RF              (1<<7)
#define RSR_MF              (1<<6)
#define RSR_LCS             (1<<5)
#define RSR_RWTO            (1<<4)
#define RSR_PLE             (1<<3)
#define RSR_AE              (1<<2)
#define RSR_CE              (1<<1)
#define RSR_FOE             (1<<0)

#define FCTR_HWOT(ot)       (( ot & 0xf ) << 4 )
#define FCTR_LWOT(ot)       ( ot & 0xf )

#define IMR_PAR             (1<<7)
#define IMR_ROOM            (1<<3)
#define IMR_ROM             (1<<2)
#define IMR_PTI             (1<<1)
#define IMR_PRI             (1<<0)

#define ISR_ROOS            (1<<3)
#define ISR_ROS             (1<<2)
#define ISR_PTS             (1<<1)
#define ISR_PRS             (1<<0)
#define ISR_CLR_STATUS      (ISR_ROOS | ISR_ROS | ISR_PTS | ISR_PRS)

#define EPCR_REEP           (1<<5)
#define EPCR_WEP            (1<<4)
#define EPCR_EPOS           (1<<3)
#define EPCR_ERPRR          (1<<2)
#define EPCR_ERPRW          (1<<1)
#define EPCR_ERRE           (1<<0)

#define GPCR_GEP_CNTL       (1<<0)

#define PHY_BMCR         00 /*基本模式控制寄存器*/
#define PHY_BMSR         01 /*基本模式状态寄存器*/
#define PHY_PHYID1       02 /*PHY ID标识符寄存器#1*/
#define PHY_PHYID2       03 /*PHY ID标识符寄存器#2*/
#define PHY_ANAR         04 /*自动协商通知寄存器*/
#define PHY_ANLPAR       05 /*自动协商连接对象寄存器*/
#define PHY_ANER         06 /*自动协商扩展寄存器*/
#define PHY_DSCR         16 /*指定配置寄存器*/
#define PHY_DSCSR        17 /*指定配置和状态寄存器*/
#define PHY_10BTCSR      18 /*10BASE-T配置和状态寄存器*/
#define PHY_PWDOR        19 /*掉电控制寄存器*/
#define PHY_SCR          20 /*指定配置寄存器*/


#define DM9000_BYTE_MODE      0x01
#define DM9000_WORD_MODE      0x00
#define DM9000_PHY            0x40
#define DM9000_PKT_RDY        0x01
#define DM9000_PKT_NORDY      0x00
#define DM9000_REG_RESET      0x03

#define DM9000_RX_INTR        0x01                /* 接收中断判断 bit */
#define DM9000_TX_INTR        0x02                /* 传送中断判断 bit */
#define DM9000_OVERFLOW_INTR  0x04                /* 内存溢出中断判断 bit */
#define DM9000_LINK_CHANG     0x20                /* 连接变动中断判断 bit */

#define DM9000_PHY_ON         0x00                /* 设定 PHY 开启 */
#define DM9000_PHY_OFF        0x01                /* 设定 PHY 关闭 */
#define DM9000_RCR_SET        0x31                /* 设定 接收功能 (不收 CRC 及 超长包) */
#define DM9000_TCR_SET        0x01                /* 设定 传送功能 */
#define DM9000_RCR_OFF        0x00                /* 设定 接收功能关关闭设置 */
#define DM9000_BPTR_SET       0x37                /* 设定 Back Pressure 条件设置 */
#define DM9000_FCTR_SET       0x38                /* 设定 Flow Control 条件设置 */
#define DM9000_TCR2_SET       0x80                /* 设置 LED 显示模式 */
#define DM9000_OTCR_SET       0x80                /* 设置 DM9000 工作频率 0x80 = 100Mhz */
#define DM9000_ETXCSR_SET     0x83                /* 设置 Early Tramsmit 条件设置 */
#define DM9000_FCR_SET        0x28                /* 开启 网络流控功能设置 */
#define DM9000_TCSCR_SET      0x07                /* 设定 CHECKSUM 传送运算 设置 */
#define DM9000_RCSCSR_SET     0x03                /* 设定 CHECKSUM 接收检查 设置 */
#define DM9000_IMR_SET        0x81                /* 设定 启用中断使能 条件设置 */
#define DM9000_IMR_OFF        0x80                /* 设定 关闭中断使能 条件设置 */

#endif
