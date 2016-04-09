#include "nokia_5110.h"
#include "english_6x8_pixel.h"
#include "write_chinese_string_pixel.h"



/*-----------------------------------------------------------------------
LCD_init          : 3310LCD��ʼ��

��д����          ��2004-8-10 
����޸�����      ��2004-8-10 
-----------------------------------------------------------------------*/
void LCD_init(void){
	unsigned char k;
	DIN_C = 1;
  CLK_C=1;
  CE_C=1;
  DC_C=1;
	CE=0;//�ر�LCD
  for(k=0;k<0xff;k++);
  CE=1;// ʹ��LCD
  for(k=0;k<0xff;k++);

  LCD_write_byte(0x21, 0);	// ʹ����չ��������LCDģʽ
  LCD_write_byte(0xca, 0);	// ����ƫ�õ�ѹ
  LCD_write_byte(0x06, 0);	// �¶�У��
  LCD_write_byte(0x10, 0);	// 1:100
  LCD_write_byte(0x20, 0);	// ʹ�û�������
  LCD_clear();	            // ����
  LCD_write_byte(0x0c, 0);	// �趨��ʾģʽ��������ʾ
  CE=0;// �ر�LCD
}
/*-----------------------------------------------------------------------
LCD_clear         : LCD��������

��д����          ��2004-8-10 
����޸�����      ��2004-8-10 
-----------------------------------------------------------------------*/
void LCD_clear(void){
    unsigned int i;
    LCD_write_byte(0x0c, 0);			
    LCD_write_byte(0x80, 0);			
    for (i=0; i<504; i++)LCD_write_byte(0, 1);		      	
}
/*-----------------------------------------------------------------------
LCD_set_XY        : ����LCD���꺯��

���������X       ��0��83   14����(0~13)*6
          Y       ��0��5    6��
          
��д����          ��2004-8-10 
����޸�����      ��2004-8-10 
-----------------------------------------------------------------------*/
void LCD_set_XY(unsigned char X, unsigned char Y){
    LCD_write_byte(0x40 | Y, 0);		// column
    LCD_write_byte(0x80 | X, 0);        // row
}
/*-----------------------------------------------------------------------
LCD_write_char    : ��ʾӢ���ַ�

���������c       ����ʾ���ַ���

��д����          ��2004-8-10 
����޸�����      ��2004-8-10 
-----------------------------------------------------------------------*/
void LCD_write_char(unsigned char c){
    unsigned char line;
    c -= 32;
    for (line=0; line<6; line++)LCD_write_byte(font6x8[c][line], 1);   
}
/*-----------------------------------------------------------------------
LCD_write_english_String  : Ӣ���ַ�����ʾ����

���������*s      ��Ӣ���ַ���ָ�룻
          X��Y    : ��ʾ�ַ�����λ��,x 0-83 ,y 0-5

��д����          ��2004-8-10 
����޸�����      ��2004-8-10 		
-----------------------------------------------------------------------*/
void LCD_write_english_string(unsigned char X,unsigned char Y,char *s){
    LCD_set_XY(X,Y);
    while (*s){
	    LCD_write_char(*s);
	 	s++;
    }
}
/*-----------------------------------------------------------------------
LCD_write_chinese_string: ��LCD����ʾ����

���������X��Y    ����ʾ���ֵ���ʼX��Y���ꣻ
          ch_with �����ֵ���Ŀ���
          num     ����ʾ���ֵĸ�����  
          line    �����ֵ��������е���ʼ����
          row     ��������ʾ���м��
��д����          ��2004-8-11 
����޸�����      ��2004-8-12 
���ԣ�
	LCD_write_chi(0,0,12,7,0,0);
	LCD_write_chi(0,2,12,7,0,0);
	LCD_write_chi(0,4,12,7,0,0);	
-----------------------------------------------------------------------*/                        
void LCD_write_chinese_string(unsigned char X, unsigned char Y, 
                   unsigned char ch_with,unsigned char num,
                   unsigned char line,unsigned char row)
{
    unsigned char i,n;
    LCD_set_XY(X,Y);                             //���ó�ʼλ��
    for (i=0;i<num;){
      	for (n=0; n<ch_with*2; n++){              //дһ������
      	    if (n==ch_with){                      //д���ֵ��°벿��
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
LCD_draw_map      : λͼ���ƺ���

���������X��Y    ��λͼ���Ƶ���ʼX��Y���ꣻ
          *map    ��λͼ�������ݣ�
          Pix_x   ��λͼ���أ�����
          Pix_y   ��λͼ���أ�����

��д����          ��2004-8-13
����޸�����      ��2004-8-13 
-----------------------------------------------------------------------*/
void LCD_draw_bmp_pixel(unsigned char X,unsigned char Y,const unsigned char *map,
                  unsigned char Pix_x,unsigned char Pix_y)
{
    unsigned int i,n;
    unsigned char row;
    if (Pix_y%8==0) row=Pix_y/8;      //����λͼ��ռ����
    else			row=Pix_y/8+1;
    for (n=0;n<row;n++){
      	LCD_set_XY(X,Y);
        for(i=0; i<Pix_x; i++){
            LCD_write_byte(map[i+n*Pix_x], 1);
          }
        Y++;                         //����
      }      
}
/*-----------------------------------------------------------------------
LCD_write_byte    : д���ݵ�LCD

���������data    ��д������ݣ�
          command ��д����/����ѡ��

��д����          ��2004-8-10 
����޸�����      ��2004-8-13 
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

