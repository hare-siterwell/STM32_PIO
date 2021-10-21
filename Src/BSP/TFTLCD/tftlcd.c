/**
 * @file tftlcd.c
 * @brief TFTLCD驱动
 */

#include "tftlcd.h"
#include "font.h"
#include "jpeg.h"

#define LCD_SPI SPI6 // LCD底层SPI

// LCD缓存大小设置，修改此值时请注意!!
// 修改这两个值时可能会影响 LCD_Clear/LCD_Fill/LCD_DrawLine
#define LCD_TOTAL_BUF_SIZE (240 * 135 * 2)
#define LCD_BUF_SIZE 1152
static u8 lcd_buf[LCD_BUF_SIZE];

u16 POINT_COLOR = CYAN; // 画笔颜色
u16 BACK_COLOR = BLACK; // 背景颜色

/**
 * @brief LCD底层SPI发送数据函数
 * @param data 数据的起始地址
 * @param size 发送数据大小
 */
static void LCD_SPI_Send(u8 *data, u16 size) {
  LL_SPI_SetTransferSize(LCD_SPI, size);
  LL_SPI_Enable(LCD_SPI);
  LL_SPI_StartMasterTransfer(LCD_SPI);
  for (u16 i = 0; i < size; i++) {
    while (!LL_SPI_IsActiveFlag_TXP(LCD_SPI))
      ;
    LL_SPI_TransmitData8(LCD_SPI, data[i]);
  }
  while (!LL_SPI_IsActiveFlag_EOT(LCD_SPI))
    ;
  LL_SPI_ClearFlag_EOT(LCD_SPI);
  LL_SPI_ClearFlag_TXTF(LCD_SPI);
  LL_SPI_SuspendMasterTransfer(LCD_SPI);
  LL_SPI_Disable(LCD_SPI);
}

/**
 * @brief 写命令到LCD
 * @param cmd 需要发送的命令
 */
static void LCD_Write_Cmd(u8 cmd) {
  LCD_DC(0);
  LCD_SPI_Send(&cmd, 1);
}

/**
 * @brief 写数据到LCD
 * @param cmd 需要发送的数据
 */
static void LCD_Write_Data(u8 data) {
  LCD_DC(1);
  LCD_SPI_Send(&data, 1);
}

/**
 * @brief 写半个字的数据到LCD
 * @param cmd 需要发送的数据
 */
void LCD_Write_HalfWord(const u16 da) {
  u8 data[2] = {da >> 8, da};

  LCD_DC(1);
  LCD_SPI_Send(data, 2);
}

/**
 * @brief 设置数据写入LCD缓存区域
 * @param x1 起点x坐标
 * @param y1 起点y坐标
 * @param x2 终点x坐标
 * @param y2 终点y坐标
 */
void LCD_Address_Set(u16 x1, u16 y1, u16 x2, u16 y2) {
  if (USE_HORIZONTAL == 0) {
    LCD_Write_Cmd(0x2a); // 列地址设置
    LCD_Write_HalfWord(x1 + 52);
    LCD_Write_HalfWord(x2 + 52);
    LCD_Write_Cmd(0x2b); // 行地址设置
    LCD_Write_HalfWord(y1 + 40);
    LCD_Write_HalfWord(y2 + 40);
    LCD_Write_Cmd(0x2c); // 储存器写
  } else if (USE_HORIZONTAL == 1) {
    LCD_Write_Cmd(0x2a); // 列地址设置
    LCD_Write_HalfWord(x1 + 53);
    LCD_Write_HalfWord(x2 + 53);
    LCD_Write_Cmd(0x2b); // 行地址设置
    LCD_Write_HalfWord(y1 + 40);
    LCD_Write_HalfWord(y2 + 40);
    LCD_Write_Cmd(0x2c); // 储存器写
  } else if (USE_HORIZONTAL == 2) {
    LCD_Write_Cmd(0x2a); // 列地址设置
    LCD_Write_HalfWord(x1 + 40);
    LCD_Write_HalfWord(x2 + 40);
    LCD_Write_Cmd(0x2b); // 行地址设置
    LCD_Write_HalfWord(y1 + 53);
    LCD_Write_HalfWord(y2 + 53);
    LCD_Write_Cmd(0x2c); // 储存器写
  } else {
    LCD_Write_Cmd(0x2a); // 列地址设置
    LCD_Write_HalfWord(x1 + 40);
    LCD_Write_HalfWord(x2 + 40);
    LCD_Write_Cmd(0x2b); // 行地址设置
    LCD_Write_HalfWord(y1 + 52);
    LCD_Write_HalfWord(y2 + 52);
    LCD_Write_Cmd(0x2c); // 储存器写
  }
}

/**
 * @brief 打开LCD显示
 */
void LCD_DisplayOn(void) { LCD_PWR(0); }

/**
 * @brief 关闭LCD显示
 */
void LCD_DisplayOff(void) { LCD_PWR(1); }

/**
 * @brief 以一种颜色清空LCD屏
 * @param color 清屏颜色
 */
void LCD_Clear(u16 color) { LCD_Fill(0, 0, LCD_WIDTH, LCD_HEIGHT, color); }

/**
 * @brief 用一个颜色填充整个区域
 * @param x1 起点x坐标
 * @param y1 起点y坐标
 * @param x2 终点x坐标
 * @param y2 终点y坐标
 * @param color 填充颜色
 */
void LCD_Fill(u16 x_start, u16 y_start, u16 x_end, u16 y_end, u16 color) {
  u32 size_remain = 0;

  u32 size = (x_end - x_start + 1) * (y_end - y_start + 1) * 2;
  if (size > LCD_BUF_SIZE) {
    size_remain = size - LCD_BUF_SIZE;
    size = LCD_BUF_SIZE;
  }

  LCD_Address_Set(x_start, y_start, x_end, y_end);

  while (1) {
    for (u16 i = 0; i < size / 2; i++) {
      lcd_buf[2 * i] = color >> 8;
      lcd_buf[2 * i + 1] = color;
    }

    LCD_DC(1);
    LCD_SPI_Send(lcd_buf, size);

    if (size_remain == 0) {
      break;
    } else if (size_remain > LCD_BUF_SIZE) {
      size_remain = size_remain - LCD_BUF_SIZE;
    } else {
      size = size_remain;
      size_remain = 0;
    }
  }
}

/**
 * @brief 画点函数
 * @param x 画点x坐标
 * @param y 画点y坐标
 */
void LCD_Draw_Point(u16 x, u16 y) {
  LCD_Address_Set(x, y, x, y);
  LCD_Write_HalfWord(POINT_COLOR);
}

/**
 * @brief 画点函数
 * @param x 画点x坐标
 * @param y 画点y坐标
 * @param y 1:画笔颜色 0:背景颜色
 */
void LCD_Draw_Point1(u16 x, u16 y, u8 t) {
  LCD_Address_Set(x, y, x, y);
  if (t == 1) {
    LCD_Write_HalfWord(POINT_COLOR);
  } else if (t == 0) {
    LCD_Write_HalfWord(BACK_COLOR);
  }
}

/**
 * @brief 画点带颜色函数
 * @param x 画点x坐标
 * @param y 画点y坐标
 */
void LCD_Draw_ColorPoint(u16 x, u16 y, u16 color) {
  LCD_Address_Set(x, y, x, y);
  LCD_Write_HalfWord(color);
}

/**
 * @brief 画线函数(直线、斜线)
 * @param x1 起点x坐标
 * @param y1 起点y坐标
 * @param x2 终点x坐标
 * @param y2 终点y坐标
 */
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2) {
  int xerr = 0, yerr = 0, delta_x, delta_y, distance;
  int incx, incy, row, col;

  if (y1 == y2) {
    /* 快速画水平线 */
    LCD_Address_Set(x1, y1, x2, y2);

    for (u32 i = 0; i < x2 - x1; i++) {
      lcd_buf[2 * i] = POINT_COLOR >> 8;
      lcd_buf[2 * i + 1] = POINT_COLOR;
    }

    LCD_DC(1);
    LCD_SPI_Send(lcd_buf, (x2 - x1) * 2);
    return;
  }

  delta_x = x2 - x1;
  delta_y = y2 - y1;
  row = x1;
  col = y1;

  if (delta_x > 0) {
    incx = 1;
  } else if (delta_x == 0) {
    incx = 0;
  } else {
    incx = -1;
    delta_x = -delta_x;
  }

  if (delta_y > 0) {
    incy = 1;
  } else if (delta_y == 0) {
    incy = 0;
  } else {
    incy = -1;
    delta_y = -delta_y;
  }

  if (delta_x > delta_y) {
    distance = delta_x;
  } else {
    distance = delta_y;
  }

  for (u16 t = 0; t <= distance + 1; t++) {
    LCD_Draw_Point(row, col);
    xerr += delta_x;
    yerr += delta_y;

    if (xerr > distance) {
      xerr -= distance;
      row += incx;
    }

    if (yerr > distance) {
      yerr -= distance;
      col += incy;
    }
  }
}

/**
 * @brief 画一个矩形
 * @param x1 起点x坐标
 * @param y1 起点y坐标
 * @param x2 终点x坐标
 * @param y2 终点y坐标
 */
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2) {
  LCD_DrawLine(x1, y1, x2, y1);
  LCD_DrawLine(x1, y1, x1, y2);
  LCD_DrawLine(x1, y2, x2, y2);
  LCD_DrawLine(x2, y1, x2, y2);
}

/**
 * @brief 画一个圆
 * @param x0 圆心x坐标
 * @param y0 圆心y坐标
 * @param r 圆半径
 */
void LCD_Draw_Circle(u16 x0, u16 y0, u8 r) {
  int a = 0, b = r, di = 3 - (r << 1);

  while (a <= b) {
    LCD_Draw_Point(x0 - b, y0 - a);
    LCD_Draw_Point(x0 + b, y0 - a);
    LCD_Draw_Point(x0 - a, y0 + b);
    LCD_Draw_Point(x0 - b, y0 - a);
    LCD_Draw_Point(x0 - a, y0 - b);
    LCD_Draw_Point(x0 + b, y0 + a);
    LCD_Draw_Point(x0 + a, y0 - b);
    LCD_Draw_Point(x0 + a, y0 + b);
    LCD_Draw_Point(x0 - b, y0 + a);
    a++;

    if (di < 0) {
      di += 4 * a + 6;
    } else {
      di += 10 + 4 * (a - b);
      b--;
    }

    LCD_Draw_Point(x0 + a, y0 + b);
  }
}

/**
 * @brief 显示一个ASCII码字符
 * @param x,y 显示起始坐标
 * @param chr 需要显示的字符
 * @param size 字体大小(支持16/24/32号字体)
 */
void LCD_ShowChar(u16 x, u16 y, char chr, u8 size) {
  u8 temp, sta;
  u8 csize; // 得到字体一个字符对应点阵集所占的字节数
  u16 colortemp;

  chr -= ' '; // 得到偏移后的值（ASCII字库是从空格开始取模，
              // 所以-' '就是对应字符的字库）

  if ((x > (LCD_WIDTH - size / 2)) || (y > (LCD_HEIGHT - size)))
    return;

  LCD_Address_Set(x, y, x + size / 2 - 1, y + size - 1); //(x,y,x+8-1,y+16-1)

  if ((size == 16) || (size == 32)) { // 16和32号字体
    csize = (size / 8 + ((size % 8) ? 1 : 0)) * (size / 2);

    for (u8 t = 0; t < csize; t++) {
      if (size == 16) {
        temp = asc2_1608[(u8)chr][t]; // 调用1608字体
      } else if (size == 32) {
        temp = asc2_3216[(u8)chr][t]; // 调用3216字体
      } else {
        return; // 没有的字库
      }
      for (u8 t1 = 0; t1 < 8; t1++) {
        if (temp & 0x80) {
          colortemp = POINT_COLOR;
        } else {
          colortemp = BACK_COLOR;
        }

        LCD_Write_HalfWord(colortemp);
        temp <<= 1;
      }
    }
  } else if (size == 12) { // 12号字体
    csize = (size / 8 + ((size % 8) ? 1 : 0)) * (size / 2);

    for (u8 t = 0; t < csize; t++) {
      temp = asc2_1206[(u8)chr][t];

      for (u8 t1 = 0; t1 < 6; t1++) {
        if (temp & 0x80) {
          colortemp = POINT_COLOR;
        } else {
          colortemp = BACK_COLOR;
        }

        LCD_Write_HalfWord(colortemp);
        temp <<= 1;
      }
    }
  } else if (size == 24) { // 24号字体
    csize = (size * 16) / 8;

    for (u8 t = 0; t < csize; t++) {
      temp = asc2_2412[(u8)chr][t];

      if (t % 2 == 0) {
        sta = 8;
      } else {
        sta = 4;
      }

      for (u8 t1 = 0; t1 < sta; t1++) {
        if (temp & 0x80) {
          colortemp = POINT_COLOR;
        } else {
          colortemp = BACK_COLOR;
        }

        LCD_Write_HalfWord(colortemp);
        temp <<= 1;
      }
    }
  }
}

/**
 * @brief m^n函数
 * @param m 输入参数m
 * @param n 输入参数n
 * @return m^n次方
 */
static u32 LCD_Pow(u8 m, u8 n) {
  u32 result = 1;

  while (n--) {
    result *= m;
  }

  return result;
}

/**
 * @brief 显示数字,高位为0不显示
 * @param x 起点x坐标
 * @param y 起点y坐标
 * @param num 需要显示的数字,数字范围(0~4294967295)
 * @param len 需要显示的位数
 * @param size 字体大小
 */
void LCD_ShowNum(u16 x, u16 y, u32 num, u8 len, u8 size) {
  u8 temp, enshow = 0;

  for (u8 t = 0; t < len; t++) {
    temp = (num / LCD_Pow(10, len - t - 1)) % 10;

    if (enshow == 0 && t < (len - 1)) {
      if (temp == 0) {
        LCD_ShowChar(x + (size / 2) * t, y, ' ', size);
        continue;
      } else {
        enshow = 1;
      }
    }

    LCD_ShowChar(x + (size / 2) * t, y, temp + '0', size);
  }
}

/**
 * @brief 显示数字,高位为0,可以控制显示为0还是不显示
 * @param x 起点x坐标
 * @param y 起点y坐标
 * @param num 需要显示的数字,数字范围(0~999999999)
 * @param len 需要显示的位数
 * @param size 字体大小
 * @param mode 1:高位显示0 0:高位不显示
 */
void LCD_ShowxNum(u16 x, u16 y, u32 num, u8 len, u8 size, u8 mode) {
  u8 temp, enshow = 0;

  for (u8 t = 0; t < len; t++) {
    temp = (num / LCD_Pow(10, len - t - 1)) % 10;

    if (enshow == 0 && t < (len - 1)) {
      if (temp == 0) {
        if (mode) {
          LCD_ShowChar(x + (size / 2) * t, y, '0', size);
        } else {
          LCD_ShowChar(x + (size / 2) * t, y, ' ', size);
        }
        continue;
      } else {
        enshow = 1;
      }
    }

    LCD_ShowChar(x + (size / 2) * t, y, temp + '0', size);
  }
}

/**
 * @brief 显示字符串
 * @param x 起点x坐标
 * @param y 起点y坐标
 * @param width 字符显示区域宽度
 * @param height 字符显示区域高度
 * @param size 字体大小
 * @param p 字符串起始地址
 */
void LCD_ShowString(u16 x, u16 y, u16 width, u16 height, u8 size, char *p) {
  u8 x0 = x;
  width += x;
  height += y;

  while ((*p <= '~') && (*p >= ' ')) { // 判断非法字符!
    if (x >= width) {
      x = x0;
      y += size;
    }

    if (y >= height)
      break; // 退出

    LCD_ShowChar(x, y, *p, size);
    x += size / 2;
    p++;
  }
}

/**
 * @brief 显示图片
 * @remark Image2Lcd取模方式:
 *         C语言数据/水平扫描/16位真彩色(RGB565)/高位在前 其他的不要选
 * @param x 起点x坐标
 * @param y 起点y坐标
 * @param width 图片宽度
 * @param height 图片高度
 * @param p 图片缓存数据起始地址
 */
void LCD_Show_Image(u16 x, u16 y, u16 width, u16 height, const u8 *p) {
  if (x + width > LCD_WIDTH || y + height > LCD_HEIGHT)
    return;

  LCD_Address_Set(x, y, x + width - 1, y + height - 1);
  LCD_DC(1);
  LCD_SPI_Send((u8 *)p, width * height * 2);
}

/**
 * @brief 显示ALIENTEK LOGO
 * @param x 起点x坐标
 * @param y 起点y坐标
 */
void Display_ALIENTEK_LOGO() { LCD_Show_Image(0, 26, 240, 82, ALIENTEK_LOGO); }

/**
 * @brief LCD初始化
 */
void LCD_Init(void) {
  LCD_PWR(0);
  delay_ms(120);
  LCD_RST(0);
  delay_ms(120);
  LCD_RST(1);

  delay_ms(120);
  /* Sleep Out */
  LCD_Write_Cmd(0x11);
  /* wait for power stability */
  delay_ms(120);

  /* Memory Data Access Control */
  LCD_Write_Cmd(0x36);
  if (USE_HORIZONTAL == 0) {
    LCD_Write_Data(0x00);
  } else if (USE_HORIZONTAL == 1) {
    LCD_Write_Data(0xC0);
  } else if (USE_HORIZONTAL == 2) {
    LCD_Write_Data(0x70);
  } else {
    LCD_Write_Data(0xA0);
  }

  /* RGB 5-6-5-bit  */
  LCD_Write_Cmd(0x3A);
  LCD_Write_Data(0x05);

  /* Porch Setting */
  LCD_Write_Cmd(0xB2);
  LCD_Write_Data(0x0C);
  LCD_Write_Data(0x0C);
  LCD_Write_Data(0x00);
  LCD_Write_Data(0x33);
  LCD_Write_Data(0x33);

  /*  Gate Control */
  LCD_Write_Cmd(0xB7);
  LCD_Write_Data(0x35);

  /* VCOM Setting */
  LCD_Write_Cmd(0xBB);
  LCD_Write_Data(0x19); // Vcom=1.625V

  /* LCM Control */
  LCD_Write_Cmd(0xC0);
  LCD_Write_Data(0x2C);

  /* VDV and VRH Command Enable */
  LCD_Write_Cmd(0xC2);
  LCD_Write_Data(0x01);

  /* VRH Set */
  LCD_Write_Cmd(0xC3);
  LCD_Write_Data(0x12);

  /* VDV Set */
  LCD_Write_Cmd(0xC4);
  LCD_Write_Data(0x20);

  /* Frame Rate Control in Normal Mode */
  LCD_Write_Cmd(0xC6);
  LCD_Write_Data(0x0F); // 60MHZ

  /* Power Control 1 */
  LCD_Write_Cmd(0xD0);
  LCD_Write_Data(0xA4);
  LCD_Write_Data(0xA1);

  /* Positive Voltage Gamma Control */
  LCD_Write_Cmd(0xE0);
  LCD_Write_Data(0xD0);
  LCD_Write_Data(0x04);
  LCD_Write_Data(0x0D);
  LCD_Write_Data(0x11);
  LCD_Write_Data(0x13);
  LCD_Write_Data(0x2B);
  LCD_Write_Data(0x3F);
  LCD_Write_Data(0x54);
  LCD_Write_Data(0x4C);
  LCD_Write_Data(0x18);
  LCD_Write_Data(0x0D);
  LCD_Write_Data(0x0B);
  LCD_Write_Data(0x1F);
  LCD_Write_Data(0x23);

  /* Negative Voltage Gamma Control */
  LCD_Write_Cmd(0xE1);
  LCD_Write_Data(0xD0);
  LCD_Write_Data(0x04);
  LCD_Write_Data(0x0C);
  LCD_Write_Data(0x11);
  LCD_Write_Data(0x13);
  LCD_Write_Data(0x2C);
  LCD_Write_Data(0x3F);
  LCD_Write_Data(0x44);
  LCD_Write_Data(0x51);
  LCD_Write_Data(0x2F);
  LCD_Write_Data(0x1F);
  LCD_Write_Data(0x1F);
  LCD_Write_Data(0x20);
  LCD_Write_Data(0x23);

  /* Display Inversion On */
  LCD_Write_Cmd(0x21);

  LCD_Write_Cmd(0x29);

  LCD_Address_Set(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);

  /* 打开显示 */
  LCD_PWR(1);
}
