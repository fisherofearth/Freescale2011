#include "main.h"

/*---------FIFO_���Ͷ�ʱ��---------*/ 
 void FIFO_noread(void){  //�ն�
  RCK=1;
  RCK=0; 
}
/*---------FIFO_���Ͷ�ʱ��---------*/ 


/*---------��ȡ����----------------*/ 
unsigned char FIFO_read(void){     //��ȡ����
  RCK=1;                                                                  
  RCK=0; 
  return C_DATA;
}
/*---------��ȡ����----------------*/ 


/*---------FIFO_��ʱ������---------*/   
void FIFO_Reset(void){      //FIFO��ָ�븴λ
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
/*---------FIFO_��ʱ������---------*/


/*---------FIFO_����ʼ�� ----------*/
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
/*---------FIFO_����ʼ�� ----------*/
 


/*---------�л���ƬFIFO------------*/
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
/*---------�л���ƬFIFO------------*/

