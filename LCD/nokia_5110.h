/*
Nokia5110 pin description
PIN1	RST
PIN2	CE
PIN3	DC
PIN4	DIN
PIN5	CLK
PIN6	VCC
PIN7	BL(backlight)
PIN8	GND    */


#ifndef _nokia_5110_h_
#define _nokia_5110_h_

#include "main.h"




void LCD_init(void);
void LCD_clear(void);
void LCD_write_english_string(unsigned char X,unsigned char Y,char *s);
void LCD_write_chinese_string(unsigned char X, unsigned char Y,
                   unsigned char ch_with,unsigned char num,
                   unsigned char line,unsigned char row);                 
void LCD_set_XY(unsigned char X, unsigned char Y);
void LCD_write_char(unsigned char c);
void LCD_draw_bmp_pixel(unsigned char X,unsigned char Y,const unsigned char *map,
                  unsigned char Pix_x,unsigned char Pix_y);
void LCD_write_byte(unsigned char dat, unsigned char dc);

#endif
