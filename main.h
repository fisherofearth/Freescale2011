#ifndef _MAIN_H_
#define _MAIN_H_

#include <hidef.h>      /* common defines and macros */
#include "derivative.h"    /* derivative information */
#include "nokia_5110.h"
//#include <MCUinit.h>


#define   INT8U   unsigned char
#define   INT16U   unsigned int


#define VDx 176    //����ͷ����һ֡ͼ��Ŀ��
#define VDy 32    //����ͷ����һ֡ͼ��ĸ߶� (Һ���������Ǳ���Ϊ8�ı�����
#define UP_ign 1     //up ignor video data
#define scn_d  6     //scan video data_line  ��Чͼ������������ʶ��
#define D_C    2   //Direction_c ת��ϵ��

#define PIT0_C PITINTE_PINTE0 //PIT channel_0 ʹ�ܿ��� ��=1
//#define PIT1_C PITINTE_PINTE1 //PIT channel_1 ʹ�ܿ��� ��=1
#define TIE1_C TIE_C1I        //PT1 �ⲿ�ж� ʹ�ܿ��� ��=1

#define direction_mid 730


//PWM
#define PWM1 PWMDTY01 //0��10000 ���ƶ�� PP1
#define PWM2 PWMDTY23 //0��100  ���Ƶ�� PP3 
#define PWM4 PWMDTY67 //0��100  ���Ƶ�� PP7 


//����ֵ����
#define ABS(x) ((x)<0?-(x):(x))
/*------------LCD----------*/
#define	DIN_C DDRJ_DDRJ6
#define CLK_C DDRJ_DDRJ7
#define CE_C  DDRS_DDRS3
#define DC_C  DDRS_DDRS2

#define DIN   PTJ_PTJ6
#define CLK   PTJ_PTJ7 
#define CE    PTS_PTS3
#define DC    PTS_PTS2
/*------------LCD----------*/

/*-----------FIFO----------*/
#define RRST_C    DDRB_DDRB3
#define RCK_C     DDRB_DDRB4
#define RE1_C     DDRB_DDRB5
#define WE2_C     DDRB_DDRB6
#define WE1_C     DDRB_DDRB7

#define RRST    PORTB_PB3
#define RCK     PORTB_PB4
#define RE1     PORTB_PB5
#define WE2     PORTB_PB6
#define WE1     PORTB_PB7
#define C_DATA  PORTA
/*-----------FIFO----------*/

/*-----------I2C--------------*/
#define SCL PORTB_PB2
#define SDA PORTB_PB1

#define SCL_C DDRB_DDRB2    
#define SDA_C DDRB_DDRB1  
/*-----------I2C--------------*/

/*-----------KEYS-----------*/
#define KEY4_C  DDRT_DDRT6
#define KEY3_C  DDRT_DDRT5
#define KEY2_C  DDRT_DDRT4
#define KEY1_C  DDRT_DDRT3

#define KEY4  PTIT_PTIT6 
#define KEY3  PTIT_PTIT5 
#define KEY2  PTIT_PTIT4 
#define KEY1  PTIT_PTIT3
/*-----------KEYS-----------*/

#define motor_D PWMDTY23

/*----------CC1100---------*/

//***********
#define WRITE_BURST     0x40       //����д��
#define READ_SINGLE     0x80       //��
#define READ_BURST      0xC0       //������
#define BYTES_IN_RXFIFO     0x7F        //���ջ���������Ч�ֽ���
#define CRC_OK              0x80        //CRCУ��ͨ��λ��־
//**********


#define MISO  PTM_PTM0
#define MOSI  PTM_PTM1
#define CSN   PTM_PTM5
#define SCK   PTT_PTT0
#define GDO0  PTM_PTM2//��һ�ַ�ʽ����ͨ���ߣ���ͬSD��д��ʽ
#define RXEN  PTM_PTM4
#define TXEN  PTM_PTM3

#define DMISO DDRM_DDRM0
#define DMOSI DDRM_DDRM1
#define DCSN  DDRM_DDRM5
#define DSCK  DDRT_DDRT0
#define DGDO0 DDRM_DDRM2
#define DRXEN DDRM_DDRM4   
#define DTXEN DDRM_DDRM3
/*----------CC1100---------*/

/*--------ȫ�ֱ�������-----*/
extern char MBW;
extern unsigned char data_aroll [VDx][VDy];
extern unsigned char data_aroll_char [84][4];
extern unsigned char data_Vxy [15];
extern unsigned int data_line[3];   
extern unsigned char k;
extern unsigned char v_flag;     //���жϱ�־


extern signed int speed;     //�ٶ��趨����

extern signed int  speed_x;//��ʼ����
extern signed int  speed_p;//60;  //PID���ٵı���ϵ��
extern signed int  speed_i;
extern signed int  speed_now;
extern uchar flg_diudao;

extern uchar direction_speed;

/*--------ȫ�ֱ�������-----*/

/*----------��������-------*/
/*��ʼ��*/
void keys_init(void);
void camera_init(void);
/*����*/
void set_direction(signed int dir_s);
void setspeed(uchar asas,uchar dir);

void delay(unsigned long a);    //�ӳٺ���
void LCD_write_num(unsigned char n, unsigned char X, unsigned char Y,unsigned char c);
void PID_speed(void);//PID����

void getasimpleroll(void);
void VDtoLCD2(void);
void FIFO_Reset(void);     //FIFO_��ʱ������
void FIFO_Switch(unsigned char select);
void FIFO_noread(void);
unsigned char FIFO_read(void);


/*----------��������-------*/

#endif