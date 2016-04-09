#include "nokia_5110.h"
#include "english_6x8_pixel.h"
#include "write_chinese_string_pixel.h"



/*-----------------------------------------------------------------------
LCD_init          : 3310LCD初始化

编写日期          ：2004-8-10 
最后修改日期      ：2004-8-10 
-----------------------------------------------------------------------*/
void LCD_init(void){
	unsigned char k;
	DIN_C = 1;
  CLK_C=1;
  CE_C=1;
  DC_C=1;
	CE=0;//关闭LCD
  for(k=0;k<0xff;k++);
  CE=1;// 使能LCD
  for(k=0;k<0xff;k++);

  LCD_write_byte(0x21, 0);	// 使用扩展命令设置LCD模式
  LCD_write_byte(0xca, 0);	// 设置偏置电压
  LCD_write_byte(0x06, 0);	// 温度校正
  LCD_write_byte(0x10, 0);	// 1:100
  LCD_write_byte(0x20, 0);	// 使用基本命令
  LCD_clear();	            // 清屏
  LCD_write_byte(0x0c, 0);	// 设定显示模式，正常显示
  CE=0;// 关闭LCD
}
/*-----------------------------------------------------------------------
LCD_clear         : LCD清屏函数

编写日期          ：2004-8-10 
最后修改日期      ：2004-8-10 
-----------------------------------------------------------------------*/
void LCD_clear(void){
    unsigned int i;
    LCD_write_byte(0x0c, 0);			
    LCD_write_byte(0x80, 0);			
    for (i=0; i<504; i++)LCD_write_byte(0, 1);		      	
}
/*-----------------------------------------------------------------------
LCD_set_XY        : 设置LCD坐标函数

输入参数：X       ：0－83   14个字(0~13)*6
          Y       ：0－5    6行
          
编写日期          ：2004-8-10 
最后修改日期      ：2004-8-10 
-----------------------------------------------------------------------*/
void LCD_set_XY(unsigned char X, unsigned char Y){
    LCD_write_byte(0x40 | Y, 0);		// column
    LCD_write_byte(0x80 | X, 0);        // row
}
/*-----------------------------------------------------------------------
LCD_write_char    : 显示英文字符

输入参数：c       ：显示的字符；

编写日期          ：2004-8-10 
最后修改日期      ：2004-8-10 
-----------------------------------------------------------------------*/
void LCD_write_char(unsigned char c){
    unsigned char line;
    c -= 32;
    for (line=0; line<6; line++)LCD_write_byte(font6x8[c][line], 1);   
}
/*-----------------------------------------------------------------------
LCD_write_english_String  : 英文字符串显示函数

输入参数：*s      ：英文字符串指针；
          X、Y    : 显示字符串的位置,x 0-83 ,y 0-5

编写日期          ：2004-8-10 
最后修改日期      ：2004-8-10 		
-----------------------------------------------------------------------*/
void LCD_write_english_string(unsigned char X,unsigned char Y,char *s){
    LCD_set_XY(X,Y);
    while (*s){
	    LCD_write_char(*s);
	 	s++;
    }
}
/*-----------------------------------------------------------------------
LCD_write_chinese_string: 在LCD上显示汉字

输入参数：X、Y    ：显示汉字的起始X、Y坐标；
          ch_with ：汉字点阵的宽度
          num     ：显示汉字的个数；  
          line    ：汉字点阵数组中的起始行数
          row     ：汉字显示的行间距
编写日期          ：2004-8-11 
最后修改日期      ：2004-8-12 
测试：
	LCD_write_chi(0,0,12,7,0,0);
	LCD_write_chi(0,2,12,7,0,0);
	LCD_write_chi(0,4,12,7,0,0);	
-----------------------------------------------------------------------*/                        
void LCD_write_chinese_string(unsigned char X, unsigned char Y, 
                   unsigned char ch_with,unsigned char num,
                   unsigned char line,unsigned char row)
{
    unsigned char i,n;
    LCD_set_XY(X,Y);                             //设置初始位置
    for (i=0;i<num;){
      	for (n=0; n<ch_with*2; n++){              //写一个汉字
      	    if (n==ch_with){                      //写汉字的下半部分
      	        if (i==0) LCD_set_XY(X,Y+1);
      	        else      LCD_set_XY((X+(ch_with+row)*i),Y+1);   
              }
      	    LCD_write_byte(write_chinese[line+i][n],1);
      	  }
      	i++;
      	LCD_set_XY((X+(ch_with+row)*i),Y);
     }
}
/*-----------------------------------------------------------------------
LCD_draw_map      : 位图绘制函数

输入参数：X、Y    ：位图绘制的起始X、Y坐标；
          *map    ：位图点阵数据；
          Pix_x   ：位图像素（长）
          Pix_y   ：位图像素（宽）

编写日期          ：2004-8-13
最后修改日期      ：2004-8-13 
-----------------------------------------------------------------------*/
void LCD_draw_bmp_pixel(unsigned char X,unsigned char Y,const unsigned char *map,
                  unsigned char Pix_x,unsigned char Pix_y)
{
    unsigned int i,n;
    unsigned char row;
    if (Pix_y%8==0) row=Pix_y/8;      //计算位图所占行数
    else			row=Pix_y/8+1;
    for (n=0;n<row;n++){
      	LCD_set_XY(X,Y);
        for(i=0; i<Pix_x; i++){
            LCD_write_byte(map[i+n*Pix_x], 1);
          }
        Y++;                         //换行
      }      
}
/*-----------------------------------------------------------------------
LCD_write_byte    : 写数据到LCD

输入参数：data    ：写入的数据；
          command ：写数据/命令选择；

编写日期          ：2004-8-10 
最后修改日期      ：2004-8-13 
-----------------------------------------------------------------------*/
void LCD_write_byte(unsigned char dat, unsigned char command){
  unsigned char i;
	CE=0;
  if(command == 0)DC=0;
  else		   		  DC=1;
	for(i=0;i<8;i++){
		if(dat&0x80)DIN=1;
		else	      DIN=0;
		CLK=0;
		dat = dat << 1;
	  CLK=1;
	}
	CE=1;
}


