#include "main.h"
/*-------键盘初始化--------*/
void keys_init(void){
 KEY1_C =0;
 KEY2_C =0;
 KEY3_C =0;
 KEY4_C =0;
}
/*-------键盘初始化--------*/

/*--------------延迟---------------*/
void delay(unsigned long a){
  for(;a>0;a--); 
}
/*--------------延迟---------------*/


/*-------LCD显示数字--------*/
/*-----------------------------------------------
 LCD_write_num(数字位数，X坐标，Y坐标，数据)
 例如CD_write_num(3，65，5，time=111)
-----------------------------------------------*/
void LCD_write_num(unsigned char n, unsigned char X, unsigned char Y,unsigned char c){
    switch(n){
      case 1:goto n_1;
      case 2:goto n_2;
      case 3:goto n_3;
      
    }
     n_3:
       LCD_set_XY(((n-3)*6+X),Y);
       LCD_write_char(((char)(c/100)+48));
     n_2:
       LCD_set_XY(((n-2)*6+X),Y);
       LCD_write_char(((char)((c/10)%10)+48));
     n_1:
       LCD_set_XY(((n-1)*6+X),Y);
       LCD_write_char(((char)(c%10)+48));
}
/*-------LCD显示数字--------*/

/*
功能：处理一行144个像素点，获取x坐标
输入：
输出：黑线中心点的x坐标(0~44)
参数：
备注：检测间隔像素=3
*/
uchar x_DQ;
unsigned char manage_aline(unsigned char x,unsigned char g,unsigned char h){     // (起始x坐标）
  unsigned char data_V [2];                             //x=起始坐标，g=最小检测间隔，h=最大检测间隔

  unsigned char i,j,l;
  char m;
  unsigned char x1=x,x2=x;
  unsigned char xx=0;
  data_V [0] = 255;
  for(j=0;j<x;j++){
       for(l=0;l<4;l++)FIFO_noread();        
  } 
  
  
  for(i=x;i<43;i++){
    FIFO_noread();
    FIFO_noread();
    FIFO_noread();
    data_V [1] = FIFO_read();
    m = data_V [0] - data_V [1];
    data_V [0] = data_V [1];
    if(m>25) x1 = i;
    if(m<(-25)){
       x2 = i;
       m = x2 - x1;
       if((m>g)&&(m<h)){
         xx =(char)((x1+x2)/2);
         i++;
         break;
       }   
    }
  } 
  for(j=i;j<44;j++){
     for(l=0;l<4;l++)FIFO_noread();      
     
  }
 // x_DQ = xx;
  return xx;
}

unsigned char manage_aline_far(unsigned char x,unsigned char g,unsigned char h){     // (起始x坐标）
  unsigned char data_V [2];                             //x=起始坐标，g=最小检测间隔，h=最大检测间隔

  unsigned char i,j,l;
  char m;
  unsigned char x1=x,x2=x;
  unsigned char xx=0;
  data_V [0] = 255;
  for(j=0;j<x;j++){
       for(l=0;l<3;l++)FIFO_noread();        
  } 
  
  
  for(i=x;i<57;i++){
    FIFO_noread();
    FIFO_noread();
    data_V [1] = FIFO_read();
    m = data_V [0] - data_V [1];
    data_V [0] = data_V [1];
    if(m>20) x1 = i;
    if(m<(-20)){
       x2 = i;
       m = x2 - x1;
       if((m>g)&&(m<h)){
         xx =(char)((x1+x2)/2);
         i++;
         break;
       }   
    }
  } 
  for(j=i;j<58;j++){
     for(l=0;l<3;l++)FIFO_noread();      
     
  }
  FIFO_noread();
  FIFO_noread(); //176/3余2个像素点，在这里处理掉
  
  x_DQ = xx;
  return xx;
}


void x_yuce (void){
  if(x_DQ<5)x_DQ=5;
  x_DQ=x_DQ-5;
}
/*-------获取并简化一帧图像--------*/
#define space_line1 8
void getasimpleroll(void){
  unsigned char i,j,l;
  //signed char m;
 
  if(v_flag==1){
      v_flag=0;
      flg_diudao=1;
      k=~k;
      FIFO_Switch(k);    //FIFO切换
      FIFO_Reset();           //FIFO复位     

      x_DQ=0;
      for(l=0;l<5;l++){  
        for(j=0;j<3;j++){   //略去3行
          for(i=0;i<176;i++){
            FIFO_noread();              
          }
        }
        data_Vxy [l] = manage_aline(x_DQ,2,9);
        if(x_DQ<5)x_DQ=5;
        x_DQ=x_DQ-5;
      }            
      for(j=0;j<10;j++){   //略去10行
        for(i=0;i<176;i++){
           FIFO_noread();
        }
      }      
      for(l=0;l<5;l++){  
        for(j=0;j<3;j++){   //略去3行
          for(i=0;i<176;i++){
            FIFO_noread();
          }
        }
        data_Vxy [(l+5)] = manage_aline(x_DQ,1,9);
        if(x_DQ<10)x_DQ=10;
        x_DQ=x_DQ-10;
      }
      for(j=0;j<10;j++){   //略去10行
        for(i=0;i<176;i++){
           FIFO_noread();
        }
      }       
      for(l=0;l<5;l++){  
        for(j=0;j<3;j++){   //略去3行
          for(i=0;i<176;i++){
            FIFO_noread();
          }
        }
        data_Vxy [(l+10)] = manage_aline(x_DQ,0,10);
        if(x_DQ<10)x_DQ=10;
        x_DQ=x_DQ-10;
      }
     // if((data_Vxy[3]!=0)&&(data_Vxy[8]!=0)&&(data_Vxy[10]!=0))flg_diudao=0;  
   
     flg_diudao=0;
  }  
  TIE_C1I=1;    
} 
/*-------获取并简化一帧图像--------*/

void VDtoLCD2(void){
  unsigned char i,j;
  
  for(j=0;j<4;j++){ 
     for(i=0;i<84;i++){
          data_aroll_char[i][j] = 0x00;
     }
  }
  
  for(j=0;j<4;j++){ 
     for(i=0;i<8;i++){
          data_aroll_char[45-data_Vxy[(j*8)+i]][3-j] |= 0b00000001 << (7-i);
     }
  }
  
  for(j=0;j<4;j++){ 
   for(i=0;i<84;i++){
     LCD_set_XY(i,j);
     LCD_write_byte(data_aroll_char[i][j], 1);     
   
   }
  }
}





/*------------设定速度-------------*/
void setspeed(uchar asas,uchar dir){ //asas=0~100
  switch(dir){
    case (0):
      PWM2 = 0; 
      PWM4 = 0;
      break;
    case (1):
      PWM2 = asas; 
      PWM4 = 0;
      break;
    case (2):
      PWM2 = 0;
      PWM4 = asas;
 
  }
}
/*------------设定速度-------------*/

/*-------PID调速-------*/
void PID_speed(void){ 
  
  speed_now=PACNT; //读入当前速度
  PACNT=0;//清空脉冲累加器
  speed_i=speed-speed_now; //比较当前速度和目标速度             
  speed_x+=(char)(speed_i/8);//积分

  if(speed_x<0)speed_x=0;
  if(speed_x>1000)speed_x=1000;
  
  setspeed((char)(speed_x/15),direction_speed);
   

}
/*-------PID调速-------*/

/*-------舵机转角-------*/

void set_direction(signed int dir_s){     // 舵机转角-140～140
  if(dir_s>140)dir_s=140;
  if(dir_s<(-140))dir_s=-140;
  PWM1=direction_mid-dir_s;
}

/*-------舵机转角-------*/

