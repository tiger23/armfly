#ifndef SPFD5420A_H_INCLUDED
#define SPFD5420A_H_INCLUDED

#include <rtthread.h>
#include <stm32f10x.h>

// Compatible list:
// spfd5420

/* LCD color */
#define White            0xFFFF
#define Black            0x0000
#define Grey             0xF7DE
#define Blue             0x001F
#define Blue2            0x051F
#define Red              0xF800
#define Magenta          0xF81F
#define Green            0x07E0
#define Cyan             0x7FFF
#define Yellow           0xFFE0

#define MASK			       0x9999	/* ��ɫ���룬�������ֱ���͸�� */

/*---------------------- Graphic LCD size definitions ------------------------*/
#define LCD_WIDTH       240                 /* Screen Width (in pixels)           */
#define LCD_HEIGHT      400                 /* Screen Hight (in pixels)           */
#define BPP             16                  /* Bits per pixel                     */
#define BYPP            ((BPP+7)/8)         /* Bytes per pixel                    */

#define MAX_X           239
#define MAX_Y           399

/* LCD �Ĵ�������, LR_ǰ׺��LCD Register�ļ�д */
#define LR_CTRL1		0x0007	/* ��д�Դ�ļĴ�����ַ */
#define LR_GRAM			0x0202	/* ��д�Դ�ļĴ�����ַ */
#define LR_GRAM_X		0x0200	/* �Դ�ˮƽ��ַ������X���꣩*/
#define LR_GRAM_Y		0x0201	/* �Դ洹ֱ��ַ������Y���꣩*/

/* ������� */
enum
{
	FC_ST_16X16 = 0,		/* ����15x16���� ����x�ߣ� */
	FC_ST_24X24 = 1			/* ����24x24���� ����x�ߣ� */
};

/* �������Խṹ, ����LCD_DispStr() */
typedef struct
{
	uint16_t usFontCode;	/* ������� 0 ��ʾ16���� */
	uint16_t usTextColor;	/* ������ɫ */
	uint16_t usBackColor;	/* ���ֱ�����ɫ��͸�� */
	uint16_t usSpace;		/* ���ּ�࣬��λ = ���� */
}FONT_T;

/* ��������� */
#define BRIGHT_MAX		  255
#define BRIGHT_MIN		  0
#define BRIGHT_DEFAULT	125
#define BRIGHT_STEP		  5

void spfd5420_init(void);
void LCD_DispOn(void);
void LCD_DispOff(void);
void lcd_clear(unsigned short Color);
void LCD_PutPixel(uint16_t _usX, uint16_t _usY, uint16_t _usColor);
uint16_t LCD_GetPixel(uint16_t _usX, uint16_t _usY);
void spfd5420_lcd_set_pixel(const char* pixel, int x, int y);
void spfd5420_lcd_get_pixel(char* pixel, int x, int y);
void spfd5420_lcd_draw_hline(const char* pixel, int x1, int x2, int y);
void spfd5420_lcd_draw_vline(const char* pixel, int x, int y1, int y2);
void spfd5420_lcd_blit_line(const char* pixels, int x, int y, rt_size_t size);
void LCD_PutChar(uint16_t x,uint16_t y,uint8_t c,uint16_t charColor,uint16_t bkColor);
void LCD_PutText(uint16_t Xpos, uint16_t Ypos, uint8_t *str,uint16_t Color, uint16_t bkColor);
void LCD_PutChinese(uint16_t x,uint16_t y,uint8_t *str,uint16_t Color,uint16_t bkColor);
void LCD_PutChineseText(uint16_t Xpos, uint16_t Ypos, uint8_t *str,uint16_t Color, uint16_t bkColor);
void LCD_SetBackLight(uint8_t _bright);
void rt_hw_lcd_init(void);

//#define _ILI_REVERSE_DIRECTION_

rt_size_t lcd_spfd5420_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size);
rt_size_t lcd_spfd5420_write (rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size);

#endif // SPFD5420A_H_INCLUDED
