/**
 * @file tftlcd.h
 * @brief This file contains all the function prototypes for
 *        the tftlcd.c file
 */

#ifndef __TFTLCD_H
#define __TFTLCD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp.h"

extern u16 POINT_COLOR; // 默认画笔颜色
extern u16 BACK_COLOR;  // 默认背景颜色

// 定义LCD的宽和高
#define USE_HORIZONTAL 2 // 设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏
#if USE_HORIZONTAL == 0 || USE_HORIZONTAL == 1
#define LCD_WIDTH 135
#define LCD_HEIGHT 240
#else
#define LCD_WIDTH 240
#define LCD_HEIGHT 135
#endif

#define LCD_PWR(n)                                                             \
  n ? LL_GPIO_SetOutputPin(LCD_PWR_GPIO_Port, LCD_PWR_Pin)                     \
    : LL_GPIO_ResetOutputPin(LCD_PWR_GPIO_Port, LCD_PWR_Pin)
#define LCD_RST(n)                                                             \
  n ? LL_GPIO_SetOutputPin(LCD_RST_GPIO_Port, LCD_RST_Pin)                     \
    : LL_GPIO_ResetOutputPin(LCD_RST_GPIO_Port, LCD_RST_Pin)
#define LCD_DC(n)                                                              \
  n ? LL_GPIO_SetOutputPin(LCD_DC_GPIO_Port, LCD_DC_Pin)                       \
    : LL_GPIO_ResetOutputPin(LCD_DC_GPIO_Port, LCD_DC_Pin)

enum Colors {
  WHITE = 0xFFFF,
  BLACK = 0x0000,
  BLUE = 0x001F,
  BRED = 0XF81F,
  GRED = 0XFFE0,
  GBLUE = 0X07FF,
  RED = 0xF800,
  MAGENTA = 0xF81F,
  GREEN = 0x07E0,
  CYAN = 0x7FFF,
  YELLOW = 0xFFE0,
  BROWN = 0XBC40,      // 棕色
  BRRED = 0XFC07,      // 棕红色
  GRAY = 0X8430,       // 灰色
  DARKBLUE = 0X01CF,   // 深蓝色
  LIGHTBLUE = 0X7D7C,  // 浅蓝色
  GRAYBLUE = 0X5458,   // 灰蓝色
  LIGHTGREEN = 0X841F, // 浅绿色
  LGRAY = 0XC618,      // 浅灰色(PANNEL),窗体背景色
  LGRAYBLUE = 0XA651,  // 浅灰蓝色(中间层颜色)
  LBBLUE = 0X2B12      // 浅棕蓝色(选择条目的反色)
};

void LCD_Init(void);                   // 初始化
void LCD_DisplayOn(void);              // 开显示
void LCD_DisplayOff(void);             // 关显示
void LCD_Write_HalfWord(const u16 da); // 写半个字节数据到LCD
void LCD_Address_Set(u16 x1, u16 y1, u16 x2, u16 y2); // 设置数据显示区域
void LCD_Clear(u16 color);                            // 清屏
void LCD_Fill(u16 x_start, u16 y_start, u16 x_end, u16 y_end,
              u16 color);                                 // 填充单色
void LCD_Draw_Point(u16 x, u16 y);                        // 画点
void LCD_Draw_ColorPoint(u16 x, u16 y, u16 color);        // 画带颜色点
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2);        // 画线
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2);   // 画矩形
void LCD_Draw_Circle(u16 x0, u16 y0, u8 r);               // 画圆
void LCD_ShowChar(u16 x, u16 y, char chr, u8 size);       // 显示一个字符
void LCD_ShowNum(u16 x, u16 y, u32 num, u8 len, u8 size); // 显示一个数字
void LCD_ShowxNum(u16 x, u16 y, u32 num, u8 len, u8 size, u8 mode); // 显示数字
void LCD_ShowString(u16 x, u16 y, u16 width, u16 height, u8 size,
                    char *p); // 显示字符串
void LCD_Show_Image(u16 x, u16 y, u16 width, u16 height,
                    const u8 *p); // 显示图片
void Display_ALIENTEK_LOGO();     // 显示ALIENTEK LOGO
void LCD_Draw_Point1(u16 x, u16 y, u8 t);

#ifdef __cplusplus
}
#endif

#endif /* __TFTLCD_H */
