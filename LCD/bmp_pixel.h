/*------------------------------------------------------------------------------
;  Դ�ļ� / ���� : AVR
;  �����ߣ����أ�: 40��24
;  ��ģ��ʽ/��С : ��ɫ����Һ����ģ������ȡģ���ֽڵ���/120�ֽ�
;  ����ת������  : 2004-8-13
------------------------------------------------------------------------------*/ 
const unsigned char AVR_bmp[]=


  {
/*--  ������һ��ͼ��C:\Documents and Settings\armok\����\1.bmp  --*/
/*--  ����x�߶�=60x47  --*/
/*
0x00,0x00,0x00,0x00,0x80,0xE0,0xFC,0xFF,0xFF,0xFF,0x7F,0xFF,0xFE,0xFC,0xF0,0xC1,
0x0F,0x7F,0xFF,0xFF,0xFE,0xF0,0xC0,0x00,0x00,0x00,0xC0,0xF8,0xFE,0xFF,0xFF,0x3F,
0x07,0xC1,0xF0,0xFE,0xFF,0xFF,0xFF,0x1F,0x07,0x8F,0xCF,0xFF,0xFF,0xFF,0xFE,0xFC,
0x00,0x80,0xF0,0xFC,0xFF,0xFF,0xFF,0x7F,0x7F,0x78,0x78,0x79,0x7F,0x7F,0xFF,0xFF,
0xFC,0xF0,0xC1,0x07,0x1F,0xFF,0xFF,0xFE,0xFC,0xFF,0xFF,0xFF,0x1F,0x07,0xC1,0xF0,
0xFE,0xFF,0xFF,0x3F,0x0F,0x0F,0x7F,0xFF,0xFF,0xFF,0xFF,0xE7,0x07,0x03,0x01,0x00,
0x02,0x03,0x03,0x03,0x03,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,
0x03,0x03,0x03,0x03,0x00,0x00,0x03,0x1F,0x3F,0x1F,0x07,0x00,0x00,0x02,0x03,0x03,
0x03,0x03,0x01,0x00,0x00,0x00,0x00,0x03,0x03,0x03,0x03,0x03,0x03,0x00,0x00,0x00
*/
/*--  ������һ��ͼ��C:\Documents and Settings\Administrator\Desktop\untitled.bmp  --*/
/*--  ����x�߶�=60x47  --*/
/*--  ���Ȳ���8�ı������ֵ���Ϊ������x�߶�=60x48  --*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x48,0x48,0x5C,0x48,0xE8,0x48,0x5C,
0xC8,0x08,0x08,0x00,0x40,0x20,0xFC,0x88,0xA8,0xA8,0xFC,0xA8,0xA8,0xA8,0x88,0x00,
0x10,0x10,0x10,0xF0,0x14,0x18,0x10,0xF0,0x10,0x18,0x10,0x00,0xF0,0x10,0x18,0x14,
0xF0,0x40,0xB0,0x1C,0x10,0x10,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x12,0x11,0x08,0x06,0x11,0x10,0x18,0x07,0x01,0x06,0x00,
0x00,0x00,0x1F,0x00,0x00,0x00,0x1F,0x00,0x08,0x08,0x07,0x00,0x10,0x10,0x10,0x08,
0x0B,0x04,0x0B,0x08,0x10,0x10,0x10,0x00,0x1F,0x09,0x09,0x09,0x1F,0x00,0x00,0x13,
0x10,0x10,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x22,0x22,0x22,
0x22,0x00,0xF8,0x10,0x08,0x00,0xF0,0x48,0x48,0x48,0x48,0x70,0x00,0xF0,0x48,0x48,
0x48,0x48,0x70,0x00,0x30,0x48,0x48,0x88,0x00,0xF0,0x08,0x08,0x08,0x08,0x00,0x80,
0x48,0x48,0x48,0xF0,0x00,0xFF,0x00,0xF0,0x48,0x48,0x48,0x48,0x70,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0xC3,0x80,
0x80,0x80,0x01,0x82,0x82,0x82,0x82,0x01,0x00,0x81,0x42,0x02,0x82,0x02,0xC1,0x00,
0x02,0x82,0x02,0x01,0x80,0x81,0x82,0x82,0xC2,0x82,0x80,0x81,0x82,0x82,0x82,0x03,
0x00,0x03,0x00,0x01,0x02,0x02,0x02,0x02,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0x0B,0x06,0xF3,0x56,0x5A,0x50,0x57,
0xF4,0x04,0x07,0x00,0x01,0xFD,0x55,0x55,0xFD,0x00,0xF7,0x25,0x25,0x14,0xC7,0x00,
0x40,0x48,0x4C,0x4B,0x48,0xFE,0x48,0x48,0x48,0x48,0x40,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,
0x00,0x01,0x00,0x01,0x01,0x00,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,
0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00

  };

