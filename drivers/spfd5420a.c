#include "spfd5420a.h"
#include <string.h>
#include <HzLib.h>
#include <AsciiLib.h>
// Compatible list:
// spfd5420

//������������,�����������
#ifdef __CC_ARM                          /* ARM Compiler    */
#define lcd_inline                  static __inline
#elif defined (__ICCARM__)              /* for IAR Compiler */
#define lcd_inline                  inline
#elif defined (__GNUC__)                /* GNU GCC Compiler */
#define lcd_inline                  static __inline
#else
#define lcd_inline                  static
#endif

#define rw_data_prepare()               write_cmd(34)


/********* control ***********/
#include "stm32f10x.h"
#include "board.h"

//����ض���.���������ض���ʱ.
#define printf               rt_kprintf //ʹ��rt_kprintf�����
//#define printf(...)                       //�����

/* LCD is connected to the FSMC_Bank1_NOR/SRAM2 and NE2 is used as ship select signal */
/* RS <==> A2 */
#define LCD_BASE             ((uint32_t)(0x60000000 | 0x0C000000))
#define LCD_REG              *(__IO uint16_t *)(LCD_BASE) /* RS = 0 */
#define LCD_RAM              *(__IO uint16_t *)(LCD_BASE + 2) /* RS = 1 */

static uint8_t s_RGBChgEn = 0;      /* RGBת��ʹ��, 4001��д�Դ������RGB��ʽ��д��Ĳ�ͬ */
static uint8_t s_AddrAutoInc = 0;   /* ����һ�����غ��Դ��ַ�Ƿ��Զ���1 */

static uint16_t LCD_BGR2RGB(uint16_t _usRGB);

/*
*********************************************************************************************************
*   �� �� ��: LCD_GPIOConfig
*   ����˵��: ����LCD���ƿ��ߣ�FSMC�ܽ�����Ϊ���ù���
*   ��    �Σ���
*   �� �� ֵ: ��
*********************************************************************************************************
*/
static void LCD_GPIOConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);

    /* ʹ�� FSMC, GPIOD, GPIOE, GPIOF, GPIOG �� AFIO ʱ�� */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE |
                           RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOG |
                           RCC_APB2Periph_AFIO, ENABLE);

    /* ���� PD.00(D2), PD.01(D3), PD.04(NOE), PD.05(NWE), PD.08(D13), PD.09(D14),
     PD.10(D15), PD.14(D0), PD.15(D1) Ϊ����������� */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 |
                                  GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_14 |
                                  GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* ���� PE.07(D4), PE.08(D5), PE.09(D6), PE.10(D7), PE.11(D8), PE.12(D9), PE.13(D10),
     PE.14(D11), PE.15(D12) Ϊ����������� */
    /* PE3,PE4 ����A19, A20, STM32F103ZE-EK(REV 1.0)����ʹ�� */
    /* PE5,PE6 ����A19, A20, STM32F103ZE-EK(REV 2.0)����ʹ�� */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
                                  GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |
                                  GPIO_Pin_15 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    /* ���� PF.00(A0 (RS))  Ϊ����������� */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

    /* ���� PG.12(NE4 (LCD/CS)) Ϊ����������� - CE3(LCD /CS) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_Init(GPIOG, &GPIO_InitStructure);

}

static void LCD_FSMCConfig(void)
{
    FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
    FSMC_NORSRAMTimingInitTypeDef  timing;

    /* FSMC GPIO configure */
    LCD_GPIOConfig();
    /* FSMC GPIO configure */

    /*-- FSMC Configuration -------------------------------------------------*/
    /* FSMC_Bank1_NORSRAM4 configuration */
    timing.FSMC_AddressSetupTime = 2;
    timing.FSMC_AddressHoldTime = 0;
    timing.FSMC_DataSetupTime = 4;
    timing.FSMC_BusTurnAroundDuration = 0;
    timing.FSMC_CLKDivision = 0;
    timing.FSMC_DataLatency = 0;
    timing.FSMC_AccessMode = FSMC_AccessMode_A;


    /* Color LCD configuration ------------------------------------
       LCD configured as follow:
          - Data/Address MUX = Disable
          - Memory Type = SRAM
          - Data Width = 16bit
          - Write Operation = Enable
          - Extended Mode = Enable
          - Asynchronous Wait = Disable */
    FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM4;
    FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
    FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
    FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
    FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
    FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
    FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
    FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;

    FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &timing;
    FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &timing;

    FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);

    /* - BANK 4 (of NOR/SRAM Bank 1~4) is enabled */
    FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);
}

static void delay(int cnt)
{
    volatile unsigned int dl;
    while (cnt--)
    {
        for (dl = 0; dl < 500; dl++);
    }
}


static void lcd_port_init(void)
{
    /* ����LCD���ƿ���GPIO */
    //LCD_GPIOConfig();
    LCD_FSMCConfig();
}

lcd_inline void write_cmd(unsigned short cmd)
{
    LCD_REG = cmd;
}

lcd_inline unsigned short read_data(void)
{
    return LCD_RAM;
}

lcd_inline void write_data(unsigned short data_code)
{
    LCD_RAM = data_code;
}

lcd_inline void write_reg(__IO uint16_t reg_addr, unsigned short reg_val)
{
    /* Write 16-bit Index, then Write Reg */
    LCD_REG = reg_addr;
    /* Write 16-bit Reg */
    LCD_RAM = reg_val;
}

lcd_inline unsigned short read_reg(__IO uint16_t reg_addr)
{
    unsigned short val = 0;
    /* Write 16-bit Index (then Read Reg) */
    LCD_REG = reg_addr;
    /* Read 16-bit Reg */
    val = LCD_RAM;
    return (val);
}

/********* control <ֻ��ֲ���Ϻ�������> ***********/

static unsigned short deviceid = 0; //����һ����̬������������LCD��ID

//static unsigned short BGR2RGB(unsigned short c)
//{
//    u16  r, g, b, rgb;
//
//    b = (c>>0)  & 0x1f;
//    g = (c>>5)  & 0x3f;
//    r = (c>>11) & 0x1f;
//
//    rgb =  (b<<11) + (g<<5) + (r<<0);
//
//    return( rgb );
//}

static void lcd_SetCursor(unsigned int x, unsigned int y)
{
    write_reg(0x0200, y);   /* 0-239 */
    write_reg(0x0201, 399 - x); /* 0-399 */
}

/* ��ȡָ����ַ��GRAM */
static unsigned short lcd_read_gram(unsigned int x, unsigned int y)
{
    unsigned short temp;
    //lcd_SetCursor(x,y);
    write_reg(0x0200, y);
    write_reg(0x0201, 399 - x);
    //rw_data_prepare();
    /* dummy read */
    LCD_REG = LR_GRAM;

    //temp = LCD_RAM;
    temp = LCD_RAM;

    /* �� 16-bit GRAM Reg */
    if (s_RGBChgEn == 1)
    {
        temp = LCD_BGR2RGB(temp);
    }
    return temp;
}

void lcd_clear(unsigned short Color)
{
    unsigned int index = 0;
    lcd_SetCursor(0, 0);
    LCD_REG = LR_GRAM;          /* ׼����д�Դ� */
    for (index = 0; index < (LCD_WIDTH * LCD_HEIGHT); index++)
    {
        LCD_RAM = Color;
    }
}

static void lcd_data_bus_test(void)
{
    unsigned short temp1;
    unsigned short temp2;
    //    /* [5:4]-ID~ID0 [3]-AM-1��ֱ-0ˮƽ */
    //    write_reg(0x0003,(1<<12)|(1<<5)|(1<<4) | (0<<3) );

    /* wirte */
    write_reg(0x0200, 0);
    write_reg(0x0201, 0);

    LCD_REG = 0x0202;
    LCD_RAM = 0x5555;       /* дһ������ */

    write_reg(0x0200, 0);
    write_reg(0x0201, 1);

    LCD_REG = 0x0202;
    LCD_RAM = 0xAAAA;       /* дһ������ */

    write_reg(0x0200, 0);
    write_reg(0x0201, 0);
    LCD_REG = 0x0202;
    temp1 = LCD_RAM;        /* ������ɫֵ */

    write_reg(0x0200, 0);
    write_reg(0x0201, 1);
    LCD_REG = 0x0202;
    temp2 = LCD_RAM;

    if ((temp1 == 0x5555) && (temp2 == 0xAAAA))
    {
        printf(" data bus test pass!\r\n");
    }
    else
    {
        printf(" data bus test error: %04X %04X\r\n", temp1, temp2);
    }
}

/*
*********************************************************************************************************
*   �� �� ��: LCD_Init_5420_4001
*   ����˵��: ��ʼ��5420��4001��
*   ��    �Σ���
*   �� �� ֵ: ��
*********************************************************************************************************
*/
static void LCD_Init_5420_4001(void)
{
    /* ��ʼ��LCD��дLCD�Ĵ����������� */
    write_reg(0x0000, 0x0000);
    write_reg(0x0001, 0x0100);
    write_reg(0x0002, 0x0100);

    /*
        R003H �Ĵ����ܹؼ��� Entry Mode ��������ɨ�跽��
        �μ���SPFD5420A.pdf ��15ҳ

        240x400��Ļ��������(px,py)����:
            R003 = 0x1018                  R003 = 0x1008
          -------------------          -------------------
         |(0,0)              |        |(0,0)              |
         |                   |        |                   |
         |  ^           ^    |        |   ^           ^   |
         |  |           |    |        |   |           |   |
         |  |           |    |        |   |           |   |
         |  |           |    |        |   |           |   |
         |  |  ------>  |    |        |   | <------   |   |
         |  |           |    |        |   |           |   |
         |  |           |    |        |   |           |   |
         |  |           |    |        |   |           |   |
         |  |           |    |        |   |           |   |
         |                   |        |                   |
         |                   |        |                   |
         |      (x=239,y=399)|        |      (x=239,y=399)|
         |-------------------|        |-------------------|
         |                   |        |                   |
          -------------------          -------------------

        ���հ�����������LCD�ķ����������������������ɨ�跽�����£�(����ͼ��1���Ǻ�)
         --------------------------------
        |  |(0,0)                        |
        |  |     --------->              |
        |  |         |                   |
        |  |         |                   |
        |  |         |                   |
        |  |         V                   |
        |  |     --------->              |
        |  |                    (399,239)|
         --------------------------------

        ��������(x,y) �����������ת����ϵ
        x = 399 - py;
        y = px;

        py = 399 - x;
        px = y;

    */
    write_reg(0x0003, 0x1018); /* 0x1018 1030 */

    write_reg(0x0008, 0x0808);
    write_reg(0x0009, 0x0001);
    write_reg(0x000B, 0x0010);
    write_reg(0x000C, 0x0000);
    write_reg(0x000F, 0x0000);
    write_reg(0x0007, 0x0001);
    write_reg(0x0010, 0x0013);
    write_reg(0x0011, 0x0501);
    write_reg(0x0012, 0x0300);
    write_reg(0x0020, 0x021E);
    write_reg(0x0021, 0x0202);
    write_reg(0x0090, 0x8000);
    write_reg(0x0100, 0x17B0);
    write_reg(0x0101, 0x0147);
    write_reg(0x0102, 0x0135);
    write_reg(0x0103, 0x0700);
    write_reg(0x0107, 0x0000);
    write_reg(0x0110, 0x0001);
    write_reg(0x0210, 0x0000);
    write_reg(0x0211, 0x00EF);
    write_reg(0x0212, 0x0000);
    write_reg(0x0213, 0x018F);
    write_reg(0x0280, 0x0000);
    write_reg(0x0281, 0x0004);
    write_reg(0x0282, 0x0000);
    write_reg(0x0300, 0x0101);
    write_reg(0x0301, 0x0B2C);
    write_reg(0x0302, 0x1030);
    write_reg(0x0303, 0x3010);
    write_reg(0x0304, 0x2C0B);
    write_reg(0x0305, 0x0101);
    write_reg(0x0306, 0x0807);
    write_reg(0x0307, 0x0708);
    write_reg(0x0308, 0x0107);
    write_reg(0x0309, 0x0105);
    write_reg(0x030A, 0x0F04);
    write_reg(0x030B, 0x0F00);
    write_reg(0x030C, 0x000F);
    write_reg(0x030D, 0x040F);
    write_reg(0x030E, 0x0300);
    write_reg(0x030F, 0x0701);
    write_reg(0x0400, 0x3500);
    write_reg(0x0401, 0x0001);
    write_reg(0x0404, 0x0000);
    write_reg(0x0500, 0x0000);
    write_reg(0x0501, 0x0000);
    write_reg(0x0502, 0x0000);
    write_reg(0x0503, 0x0000);
    write_reg(0x0504, 0x0000);
    write_reg(0x0505, 0x0000);
    write_reg(0x0600, 0x0000);
    write_reg(0x0606, 0x0000);
    write_reg(0x06F0, 0x0000);
    write_reg(0x07F0, 0x5420);
    write_reg(0x07DE, 0x0000);
    write_reg(0x07F2, 0x00DF);
    write_reg(0x07F3, 0x0810);
    write_reg(0x07F4, 0x0077);
    write_reg(0x07F5, 0x0021);
    write_reg(0x07F0, 0x0000);
    write_reg(0x0007, 0x0173);

    /* ������ʾ���� WINDOWS */
    write_reg(0x0210, 0);   /* ˮƽ��ʼ��ַ */
    write_reg(0x0211, 239); /* ˮƽ�������� */
    write_reg(0x0212, 0);   /* ��ֱ��ʼ��ַ */
    write_reg(0x0213, 399); /* ��ֱ������ַ */
}

void spfd5420_init(void)
{
    lcd_port_init();
    //delay(2000);
    rt_thread_delay(RT_TICK_PER_SECOND / 500); /* sleep 0.2 second  */
    deviceid = read_reg(0x0000);

    /* deviceid check */
    if (deviceid != 0x5420)
    {
        printf("Invalid LCD ID:%08X\r\n", deviceid);
        printf("Please check you hardware and configure.\r\n");
    }
    else
    {
        printf("\r\nLCD Device ID : %04X ", deviceid);
        LCD_Init_5420_4001(); /* ȱʡ��5420����,4001����5420��ͬ��4001�������Դ�RGBʱ����Ҫ����ת����5420����  */
    }



    /* ������δ�������ʶ����4001������5420�� */
    {
        uint16_t dummy;

        write_reg(0x0200, 0);
        write_reg(0x0201, 0);

        LCD_REG = 0x0202;
        LCD_RAM = 0x1234;       /* дһ������ */

        write_reg(0x0200, 0);
        write_reg(0x0201, 0);
        LCD_REG = 0x0202;
        dummy = LCD_RAM;        /* ������ɫֵ */
        if (dummy == 0x1234)
        {
            s_RGBChgEn = 0;
        }
        else
        {
            s_RGBChgEn = 1;     /* ������صĺ�д��Ĳ�ͬ������ҪRGBת��, ֻӰ���ȡ���صĺ��� */
        }

        if (deviceid == 0xB509)
        {
            s_AddrAutoInc = 0;  /* 61509����ַ�������� */
        }
        else
        {
            s_AddrAutoInc = 1;  /* 5420��4001����ַ���Զ����� */
        }
    }

    lcd_clear(Black);
    //�������߲���,���ڲ���Ӳ�������Ƿ�����.
    lcd_data_bus_test();
    //GRAM����,�˲��Կ��Բ���LCD�������ڲ�GRAM.����ͨ����֤Ӳ������
    //    lcd_gram_test();

    //����
    lcd_clear(Blue);
}

/*
*********************************************************************************************************
*   �� �� ��: LCD_DispOn
*   ����˵��: ����ʾ
*   ��    �Σ���
*   �� �� ֵ: ��
*********************************************************************************************************
*/
void LCD_DispOn(void)
{
    write_reg(7, 0x0173); /* ����262K��ɫ���Ҵ���ʾ */
}

/*
*********************************************************************************************************
*   �� �� ��: LCD_DispOff
*   ����˵��: �ر���ʾ
*   ��    �Σ���
*   �� �� ֵ: ��
*********************************************************************************************************
*/
void LCD_DispOff(void)
{
    write_reg(7, 0x0);  /* �ر���ʾ*/
}

/*
*********************************************************************************************************
*   �� �� ��: LCD_PutPixel
*   ����˵��: ��1������
*   ��    �Σ�
*           X,Y : ��������
*           Color  ��������ɫ
*   �� �� ֵ: ��
*********************************************************************************************************
*/
void LCD_PutPixel(uint16_t X, uint16_t Y, uint16_t Color)
{
    lcd_SetCursor(X, Y);    /* ���ù��λ�� */

    /* д�Դ� */
    LCD_REG = LR_GRAM;
    /* Write 16-bit GRAM Reg */
    LCD_RAM = Color;
}

/*
*********************************************************************************************************
*   �� �� ��: LCD_BGR2RGB
*   ����˵��: RRRRRGGGGGGBBBBB ��Ϊ BBBBBGGGGGGRRRRR ��ʽ
*   ��    �Σ�RGB��ɫ����
*   �� �� ֵ: ת�������ɫ����
*********************************************************************************************************
*/
static uint16_t LCD_BGR2RGB(uint16_t BGR)
{
    uint16_t  r, g, b, rgb;

    b = (BGR >> 0)  & 0x1F;
    g = (BGR >> 5)  & 0x3F;
    r = (BGR >> 11) & 0x1F;

    rgb = (b << 11) + (g << 5) + (r << 0);

    return (rgb);
}

/*
*********************************************************************************************************
*   �� �� ��: LCD_GetPixel
*   ����˵��: ��ȡ1������
*   ��    �Σ�
*           _usX,_usY : ��������
*           _usColor  ��������ɫ
*   �� �� ֵ: RGB��ɫֵ
*********************************************************************************************************
*/
uint16_t LCD_GetPixel(uint16_t X, uint16_t Y)
{
    uint16_t usRGB;

    lcd_SetCursor(X, Y);    /* ���ù��λ�� */

    /* ׼��д�Դ� */
    LCD_REG = LR_GRAM;

    usRGB = LCD_RAM;

    /* �� 16-bit GRAM Reg */
    if (s_RGBChgEn == 1)
    {
        usRGB = LCD_BGR2RGB(usRGB);
    }

    return usRGB;
}


/*  �������ص� ��ɫ,X,Y */
void spfd5420_lcd_set_pixel(const char *pixel, int x, int y)
{
    lcd_SetCursor(x, y);

    //rw_data_prepare();
    write_data(*(rt_uint16_t *)pixel);
}

/* ��ȡ���ص���ɫ */
void spfd5420_lcd_get_pixel(char *pixel, int x, int y)
{
    *(rt_uint16_t *)pixel = lcd_read_gram(x, y);
}

/* ��ˮƽ�� */
void spfd5420_lcd_draw_hline(const char *pixel, int x1, int x2, int y)
{
    /* [5:4]-ID~ID0 [3]-AM-1��ֱ-0ˮƽ */
    write_reg(0x0011, 0x6030 | (0 << 3)); // AM=0 hline

    lcd_SetCursor(x1, y);
    //rw_data_prepare(); /* Prepare to write GRAM */
    while (x1 < x2)
    {
        write_data(*(rt_uint16_t *)pixel);
        x1++;
    }
}

/* ��ֱ�� */
void spfd5420_lcd_draw_vline(const char *pixel, int x, int y1, int y2)
{
    /* [5:4]-ID~ID0 [3]-AM-1��ֱ-0ˮƽ */
    write_reg(0x0011, 0x6070 | (1 << 3)); // AM=0 vline

    lcd_SetCursor(x, y1);
    //rw_data_prepare(); /* Prepare to write GRAM */
    while (y1 < y2)
    {
        write_data(*(rt_uint16_t *)pixel);
        y1++;
    }
}

/* blit a line */
void spfd5420_lcd_blit_line(const char *pixels, int x, int y, rt_size_t size)
{
    rt_uint16_t *ptr;

    ptr = (rt_uint16_t *)pixels;

    /* [5:4]-ID~ID0 [3]-AM-1��ֱ-0ˮƽ */
    write_reg(0x0011, 0x6070 | (0 << 3)); // AM=0 hline

    lcd_SetCursor(x, y);
    //rw_data_prepare(); /* Prepare to write GRAM */
    while (size)
    {
        write_data(*ptr ++);
        size --;
    }
}

/******************************************************************************
* Function Name  : LCD_PutChar
* Description    : ��ָ��������ʾһ��8x16�����ascii�ַ�
* Input          : - x: ˮƽ����
*                  - y: ��ֱ����
*                  - c: ��ʾ���ַ�
*                  - charColor: �ַ���ɫ
*                  - bkColor: ������ɫ
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void LCD_PutChar(uint16_t x, uint16_t y, uint8_t c, uint16_t charColor, uint16_t bkColor)
{
    uint16_t i = 0;
    uint16_t j = 0;
    uint8_t tmp_char, buffer[16];
    GetASCIICode(buffer, c);   //ȡ��ģ����
    for (i = 0; i < 16; i++)   //����
    {
        tmp_char = buffer[i];
        for (j = 0; j < 8; j++) //����
        {
            if ((tmp_char >> 7 - j) & 0x01 == 0x01)
            {
                LCD_PutPixel(x + j, y + i, charColor); // �ַ���ɫ
            }
            else
            {
                LCD_PutPixel(x + j, y + i, bkColor); // ������ɫ
            }
        }
    }

}

/******************************************************************************
* Function Name  : LCD_PutText
* Description    : ��ָ��������ʾ�ַ���
* Input          : - Xpos: ������
*                  - Ypos: ������
*                  - str: �ַ���
*                  - charColor: �ַ���ɫ
*                  - bkColor: ������ɫ
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void LCD_PutText(uint16_t Xpos, uint16_t Ypos, uint8_t *str, uint16_t Color, uint16_t bkColor)
{
    uint8_t TempChar;
    do
    {
        TempChar = *str++;
        LCD_PutChar(Xpos, Ypos, TempChar, Color, bkColor);
        if (Xpos < MAX_X - 8)
        {
            Xpos += 8;
        }
        else if (Ypos < MAX_Y - 16)
        {
            Xpos = 0;           //���ˮƽλ���Ѿ����������ҵ�8λ�������»ص�����߿�ʼ
            Ypos += 16;
        }
        else
        {
            Xpos = 0;
            Ypos = 0;
        }
    }
    while (*str != 0);
}
/******************************************************************************
* Function Name  : LCD_PutChinese
* Description    : ��LCD��������λ����ʾһ��������
* Input          : - x: ˮƽ����
*                  - y: ��ֱ����
*                  - str: ��ʾ��������
*                  - Color: �ַ���ɫ
*                  - bkColor: ������ɫ
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void LCD_PutChinese(uint16_t x, uint16_t y, uint8_t *str, uint16_t Color, uint16_t bkColor)
{
    uint8_t i, j;
    uint8_t buffer[32];
    uint16_t temp_char;

    //GetGBKCode(buffer,str);  //ȡ��ģ����

    for (i = 0; i < 16; i++)       //�У�ȡģ��ʽ�Ǻ���
    {
        temp_char = buffer[i * 2];
        temp_char = (temp_char << 8);
        temp_char |= buffer[2 * i + 1];
        for (j = 0; j < 16; j++)   //��
        {
            if ((temp_char >> 15 - j) & 0x01 == 0x01)
            {
                LCD_PutPixel(x + j, y + i, Color); //�ַ���ɫ
            }
            else
            {
                LCD_PutPixel(x + j, y + i, bkColor); //������ɫ
            }
        }
    }
}

/******************************************************************************
* Function Name  : LCD_PutChineseText
* Description    : ��LCD������ʾ�����ַ���
* Input          : - Xpos: ������
*                  - Ypos: ������
*                  - str: �ַ���
*                  - charColor: �ַ���ɫ
*                  - bkColor: ������ɫ
* Output         : None
* Return         : None
* Attention      : None
*******************************************************************************/
void LCD_PutChineseText(uint16_t Xpos, uint16_t Ypos, uint8_t *str, uint16_t Color, uint16_t bkColor)
{
    do
    {
        LCD_PutChinese(Xpos, Ypos, str++, Color, bkColor);
        str++;     //һ�������������ֽ�
        if (Xpos < MAX_X - 16)
        {
            Xpos += 16;
        }
        else if (Ypos < MAX_Y - 16)
        {
            Xpos = 0;
            Ypos += 16;
        }
        else
        {
            Xpos = 0;
            Ypos = 0;
        }
    }
    while (*str != 0);
}

/*
*********************************************************************************************************
*   �� �� ��: LCD_SetBackLight
*   ����˵��: ��ʼ������LCD�������GPIO,����ΪPWMģʽ��
*           ���رձ���ʱ����CPU IO����Ϊ��������ģʽ���Ƽ�����Ϊ������������������͵�ƽ)����TIM3�ر� ʡ��
*   ��    �Σ�_bright ���ȣ�0����255������
*   �� �� ֵ: ��
*********************************************************************************************************
*/
void LCD_SetBackLight(uint8_t bright)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    /* ��1������GPIOB RCC_APB2Periph_AFIO ��ʱ��  */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    if (bright == 0)
    {
        /* ���ñ���GPIOΪ����ģʽ */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOB, &GPIO_InitStructure);

        /* �ر�TIM3 */
        TIM_Cmd(TIM3, DISABLE);
        return;
    }
    else if (bright == BRIGHT_MAX)  /* ������� */
    {
        /* ���ñ���GPIOΪ�������ģʽ */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOB, &GPIO_InitStructure);

        GPIO_SetBits(GPIOB, GPIO_Pin_1);

        /* �ر�TIM3 */
        TIM_Cmd(TIM3, DISABLE);
        return;
    }

    /* ���ñ���GPIOΪ�����������ģʽ */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* ʹ��TIM3��ʱ�� */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /*
        TIM3 ����: ����1·PWM�ź�;
        TIM3CLK = 72 MHz, Prescaler = 0(����Ƶ), TIM3 counter clock = 72 MHz
        ���㹫ʽ��
        PWM���Ƶ�� = TIM3 counter clock /(ARR + 1)

        ������������Ϊ100Hz

        �������TIM3CLKԤ��Ƶ����ô�����ܵõ�100Hz��Ƶ��
        �������÷�Ƶ�� = 1000�� ��ô  TIM3 counter clock = 72KHz
        TIM_Period = 720 - 1;
        Ƶ���²�����
     */
    TIM_TimeBaseStructure.TIM_Period = 720 - 1; /* TIM_Period = TIM3 ARR Register */
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    /*
        _bright = 1 ʱ, TIM_Pulse = 1
        _bright = 255 ʱ, TIM_Pulse = TIM_Period
    */
    TIM_OCInitStructure.TIM_Pulse = (TIM_TimeBaseStructure.TIM_Period * bright) / BRIGHT_MAX;   /* �ı�ռ�ձ� */

    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM3, ENABLE);

    /* ʹ�� TIM3 ��ʱ�� */
    TIM_Cmd(TIM3, ENABLE);
}


struct rt_device_graphic_ops spfd5420_ops =
{
    spfd5420_lcd_set_pixel,
    spfd5420_lcd_get_pixel,
    spfd5420_lcd_draw_hline,
    spfd5420_lcd_draw_vline,
    spfd5420_lcd_blit_line
};

struct rt_device _lcd_device;
static rt_err_t lcd_init(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t lcd_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

static rt_err_t lcd_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t lcd_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    switch (cmd)
    {
    case RTGRAPHIC_CTRL_GET_INFO:
    {
        struct rt_device_graphic_info *info;

        info = (struct rt_device_graphic_info *) args;
        RT_ASSERT(info != RT_NULL);

        info->bits_per_pixel = 16;
        info->pixel_format = RTGRAPHIC_PIXEL_FORMAT_RGB565P;
        info->framebuffer = RT_NULL;
        info->width = 240;
        info->height = 400;
    }
    break;

    case RTGRAPHIC_CTRL_RECT_UPDATE:
        /* nothong to be done */
        break;

    default:
        break;
    }

    return RT_EOK;
}

void rt_hw_lcd_init(void)
{

    /* register lcd device */
    _lcd_device.type    = RT_Device_Class_Graphic;
    _lcd_device.init    = lcd_init;
    _lcd_device.open    = lcd_open;
    _lcd_device.close   = lcd_close;
    _lcd_device.control = lcd_control;
    _lcd_device.read    = RT_NULL;
    _lcd_device.write   = RT_NULL;

    _lcd_device.user_data = &spfd5420_ops;
    spfd5420_init();

    /* register graphic device driver */
    rt_device_register(&_lcd_device, "lcd",
                       RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);
}

