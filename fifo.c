#include "main.h"

/*---------FIFO_发送读时钟---------*/ 
 void FIFO_noread(void){  //空读
  RCK=1;
  RCK=0; 
}
/*---------FIFO_发送读时钟---------*/ 


/*---------读取数据----------------*/ 
unsigned char FIFO_read(void){     //读取数据
  RCK=1;                                                                  
  RCK=0; 
  return C_DATA;
}
/*---------读取数据----------------*/ 


/*---------FIFO_读时针重置---------*/   
void FIFO_Reset(void){      //FIFO读指针复位
  RRST=0;
 // asm nop;
  FIFO_noread();
 // asm nop;
  FIFO_noread();
  //asm nop;
  //FIFO_noread();
 // asm nop;
  RRST=1;
}
/*---------FIFO_读时针重置---------*/


/*---------FIFO_读初始化 ----------*/
void camera_init(void){
  DDRA=0x00;
  RRST_C=1;
  RCK_C=1;
  RE1_C=1;
  WE2_C=1;
  WE1_C=1;
  
 //SDA=1;
 // SCL=1;
  RE1=1;
  WE1=1;   
  WE2=0;
  RRST=1;
  RCK=0;
}
/*---------FIFO_读初始化 ----------*/
 


/*---------切换两片FIFO------------*/
void FIFO_Switch(unsigned char select){   
  if(select==0){
    RE1=1;
    WE1=1;
    WE2=0;
  }else{
    RE1=0;                                                              
    WE1=0;
    WE2=1;
  }
}
/*---------切换两片FIFO------------*/

