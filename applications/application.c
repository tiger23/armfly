/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2013-07-12     aozima       update for auto initial.
 */

/**
 * @addtogroup STM32
 */
/*@{*/
#include <stdio.h>
#include <string.h>
#include <board.h>
#include <rtthread.h>

#ifdef  RT_USING_COMPONENTS_INIT
#include <components.h>
#endif  /* RT_USING_COMPONENTS_INIT */

#ifdef RT_USING_DFS
/* dfs filesystem:ELM filesystem init */
#include <dfs_elm.h>
/* dfs Filesystem APIs */
#include <dfs_fs.h>
#endif

#ifdef RT_USING_RTGUI
#include <rtgui/rtgui.h>
#include <rtgui/rtgui_server.h>
#include <rtgui/rtgui_system.h>
#include <rtgui/driver.h>
#include <rtgui/calibration.h>
#endif

#include "led.h"
#include "spfd5420a.h"
#include "dm9000a.h"
#include "spi_flash.h"
#include "rs485.h"
#include "adc.h"
//#include "user_mb_app.h"
#include "delay_conf.h"
#include "bsp_rtc.h"
#include "key.h"

#ifdef RT_USING_MODBUS
USHORT  usModbusUserData[MB_PDU_SIZE_MAX];
UCHAR   ucModbusUserData[MB_PDU_SIZE_MAX];
//====================操作系统各线程优先级==================================
#define thread_ModbusSlavePoll_Prio         10
//#define thread_ModbusMasterPoll_Prio         9
ALIGN(RT_ALIGN_SIZE)
//====================操作系统各线程堆栈====================================
static rt_uint8_t thread_ModbusSlavePoll_stack[512];
//static rt_uint8_t thread_ModbusMasterPoll_stack[512];
struct rt_thread thread_ModbusSlavePoll;
//************************ Modbus从机轮训线程***************************
//函数定义: void thread_entry_ModbusSlavePoll(void* parameter)
//入口参数：无
//出口参数：无
//备    注：Editor：Armink   2013-08-02    Company: BXXJS
//******************************************************************
void thread_entry_ModbusSlavePoll(void* parameter)
{
    eMBInit(MB_RTU, 0x01, 1, 115200,  MB_PAR_NONE);
    eMBEnable();
    while (1)
    {
        eMBPoll();
        rt_thread_delay(DELAY_MB_SLAVE_POLL);
    }
}
#endif

//====================操作系统各线程优先级==================================
#define thread_RS485Poll_Prio         10

ALIGN(RT_ALIGN_SIZE)
//====================操作系统各线程堆栈====================================
static rt_uint8_t thread_RS485Poll_stack[512];

struct rt_thread thread_RS485Poll;
//************************ RS485轮训线程***************************
//函数定义: void thread_entry_RS485Poll(void* parameter)
//入口参数：无
//出口参数：无
//备    注：Editor：Astiger   2014-08-15
//******************************************************************
void thread_entry_RS485Poll(void *parameter)
{
    rt_hw_RS485_init();

    while (1)
    {
        RS485Poll();
        rt_thread_delay(DELAY_RS485_POLL);
    }
}

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t led_stack[ 512 ];
static struct rt_thread led_thread;
static void led_thread_entry(void *parameter)
{
    unsigned int count = 0;

    rt_hw_led_init();

    while (1)
    {
        /* led1 on */
#ifndef RT_USING_FINSH
        rt_kprintf("led on, count : %d\r\n", count);
#endif
        count++;
        rt_hw_led_on(1);
        rt_thread_delay(RT_TICK_PER_SECOND / 2); /* sleep 0.5 second and switch to other thread */

        /* led1 off */
#ifndef RT_USING_FINSH
        rt_kprintf("led off\r\n");
#endif
        rt_hw_led_off(1);
        rt_thread_delay(RT_TICK_PER_SECOND / 2);
    }
}

// ADC1转换的电压值通过MDA方式传到SRAM
extern __IO uint16_t ADC_ConvertedValue;

// 局部变量，用于保存转换计算后的电压值

float ADC_ConvertedValueLocal;
uint8_t buf[32];    /*字符缓冲区*/

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t adc_stack[ 512 ];
static struct rt_thread adc_thread;
static void adc_thread_entry(void *parameter)
{

    rt_hw_ADC1_init();

    while (1)
    {

#ifndef RT_USING_FINSH
        rt_kprintf("ADC is DMA mode, value is : 0x%04X \r\n", ADC_ConvertedValue);
#endif
        ADC_ConvertedValueLocal = (float) ADC_ConvertedValue / 4096 * 3.3; // 读取转换的AD值
        sprintf(buf, "%f", ADC_ConvertedValueLocal);

        rt_kprintf("\r\n The current AD value = 0x%04X \r\n", ADC_ConvertedValue);
        rt_kprintf("\r\n The current AD value = %s V \r\n", buf);

        rt_thread_delay(RT_TICK_PER_SECOND * 10);
    }
}


#ifdef RT_USING_LCD
ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t lcd_stack[ 512 ];
static struct rt_thread lcd_thread;
static void lcd_thread_entry(void *parameter)
{
    unsigned int count = 0;
    uint16_t ucBright;      /* 背光亮度(0-255) */
    uint8_t str_buf[50];

    rt_hw_lcd_init();

    while (1)
    {
        /* lcd on */
#ifndef RT_USING_FINSH
        rt_kprintf("lcd on, count : %d\r\n", count);
#endif
        count++;
        //str_buf="The current AD value =  "+buf;
        /* LCD背光设置为中间亮度,最大255 */
        ucBright = BRIGHT_DEFAULT;  /* 设置为缺省亮度 */
        LCD_SetBackLight(ucBright);
        //LCD_DispOn();
        lcd_clear(Blue);
        LCD_PutText(0, 10, (uint8_t *)buf, White, MASK);
        LCD_PutText(16, 26, "I am Tiger.", White, MASK);
        //LCD_PutChinese(32,10,"我",White,MASK);
        //LCD_PutChineseText(64,10,"我是王兵",White,MASK);
        rt_thread_delay(RT_TICK_PER_SECOND * 2); /* sleep 0.5 second and switch to other thread */

    }
}
#endif /* #ifdef RT_USING_LCD */

#ifdef RT_USING_RTGUI
rt_bool_t cali_setup(void)
{
    rt_kprintf("cali setup entered\n");
    return RT_FALSE;
}

void cali_store(struct calibration_data *data)
{
    rt_kprintf("cali finished (%d, %d), (%d, %d)\n",
               data->min_x,
               data->max_x,
               data->min_y,
               data->max_y);
}
#endif /* RT_USING_RTGUI */

void rt_init_thread_entry(void *parameter)
{
#ifdef RT_USING_COMPONENTS_INIT
    /* initialization RT-Thread Components */
    rt_components_init();
#endif

#ifdef  RT_USING_FINSH
    finsh_set_device(RT_CONSOLE_DEVICE_NAME);
#endif  /* RT_USING_FINSH */

    /* Filesystem Initialization */
#if defined(RT_USING_DFS) && defined(RT_USING_DFS_ELMFAT)
    /* mount sd card fat partition 1 as root directory */
    if (dfs_mount("sd0", "/", "elm", 0, 0) == 0)
    {
        rt_kprintf("File System initialized!\n");
    }
    else
        rt_kprintf("File System initialzation failed!\n");
#endif  /* RT_USING_DFS */

#ifdef RT_USING_RTGUI
    {
        extern void rt_hw_lcd_init();
        extern void rtgui_touch_hw_init(void);

        rt_device_t lcd;

        /* init lcd */
        rt_hw_lcd_init();

        /* init touch panel */
        rtgui_touch_hw_init();

        /* re-init device driver */
        rt_device_init_all();

        /* find lcd device */
        lcd = rt_device_find("lcd");

        /* set lcd device as rtgui graphic driver */
        rtgui_graphic_set_device(lcd);

#ifndef RT_USING_COMPONENTS_INIT
        /* init rtgui system server */
        rtgui_system_server_init();
#endif

        calibration_set_restore(cali_setup);
        calibration_set_after(cali_store);
        calibration_init();
    }
#endif /* #ifdef RT_USING_RTGUI */

#ifdef RT_USING_LWIP
    {
        extern void rt_hw_dm9000a_init();
        extern void lwip_sys_init(void);
        /* 初始化以太网线程 */
        eth_system_device_init();
        /* 注册以太网接口驱动 */
        rt_hw_dm9000a_init();
        //rt_hw_dm9000_init();
        /* 初始化LwIP系统 */
        lwip_sys_init();
        rt_kprintf("TCP/IP initialized!\n");

    }
#endif /* #ifdef RT_USING_LWIP */

#ifdef RT_USING_LCD
    {
        extern void rt_hw_lcd_init();

        rt_device_t lcd;

        /* init lcd */
        rt_hw_lcd_init();

        /* re-init device driver */
        //rt_device_init_all();

        /* find lcd device */
        lcd = rt_device_find("lcd");
        LCD_DispOn();
    }
#endif /* #ifdef RT_USING_LCD */

#ifdef RT_USING_RTC
    rt_hw_rtc_init();
#endif /* #ifdef RT_USING_I2C */

#ifdef RT_USING_I2C
    rt_i2c_core_init();
    rt_hw_I2C_init();
#endif /* #ifdef RT_USING_I2C */

#ifdef RT_USING_SPI
    rt_hw_sf_init();
#endif /* #ifdef RT_USING_SPI */

    /*on board KEY Configure Init*/
    bsp_InitButton();
    rt_hw_key_EXTI_PC13_Config();
}

int rt_application_init(void)
{
    rt_thread_t init_thread;

    rt_err_t result;

    /* init led thread */
    result = rt_thread_init(&led_thread,
                            "led",
                            led_thread_entry,
                            RT_NULL,
                            (rt_uint8_t *)&led_stack[0],
                            sizeof(led_stack),
                            20,
                            5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&led_thread);
    }

    /* init ADC thread */
    result = rt_thread_init(&adc_thread,
                            "adc",
                            adc_thread_entry,
                            RT_NULL,
                            (rt_uint8_t *)&adc_stack[0],
                            sizeof(adc_stack),
                            21,
                            5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&adc_thread);
    }

    /* init LCD thread */
    result = rt_thread_init(&lcd_thread,
                            "lcd",
                            lcd_thread_entry,
                            RT_NULL,
                            (rt_uint8_t *)&lcd_stack[0],
                            sizeof(lcd_stack),
                            22,
                            5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&lcd_thread);
    }

#ifdef RT_USING_MODBUS
    /* init MODBUS thread */
    result = rt_thread_init(&thread_ModbusSlavePoll,
                            "MBSlavePoll",
                            thread_entry_ModbusSlavePoll, 
                            RT_NULL, 
                            thread_ModbusSlavePoll_stack,
                            sizeof(thread_ModbusSlavePoll_stack), 
                            thread_ModbusSlavePoll_Prio,
                            5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&thread_ModbusSlavePoll);
    }
#endif

/* init rs485 thread */
    result = rt_thread_init(&thread_RS485Poll,
                            "485",
                            thread_entry_RS485Poll,
                            RT_NULL,
                            (rt_uint8_t *)&thread_RS485Poll_stack[0],
                            sizeof(thread_RS485Poll_stack),
                            23,
                            5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&thread_RS485Poll);
    }
    
#if (RT_THREAD_PRIORITY_MAX == 32)
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 8, 20);
#else
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 80, 20);
#endif

    if (init_thread != RT_NULL)
        rt_thread_startup(init_thread);

    return 0;
}

/*@}*/
