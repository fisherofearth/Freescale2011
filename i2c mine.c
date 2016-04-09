#include "main.h"

#define D1_TIME 100     //delay time
#define HD_TIME 100     //hode time
#define TRUE 1
#define FALSE 0

#define CAM_ADDRESS 0xC0

 void delay_i2c(unsigned int t){  //delay function
  unsigned int i;
  for(i=0;i<t;i++);        
 }
 void delay_long(long t){
long i;
  for(i=0;i<t;i++);
}

 void I2C_Start(void){    //I2C Start    开始前SDA\SCL要H，开始后SDA\SCL为L
    SDA = 1;
     delay_i2c(D1_TIME);  
    SCL = 1;
    delay_i2c(D1_TIME);    
    SDA=0;
    delay_i2c(D1_TIME);
    SCL=0;
    delay_i2c(D1_TIME);
 }
 void I2C_Stop(void){      //I2C Stop    结束前SDA\SCL要L，开始后SDA\SCL为H
        SCL=1;
        delay_i2c(D1_TIME);     
        SDA=1; 
        delay_i2c(D1_TIME);  
 }

void SEND_0(void){   // SEND 0  
        SDA=0;
        delay_i2c(HD_TIME); 
        SCL=1;
        delay_i2c(HD_TIME);   
        SCL=0;
        delay_i2c(HD_TIME);
}
void SEND_1(void){   // SEND 1 
        SDA=1;
        delay_i2c(HD_TIME);
        SCL=1;
        delay_i2c(HD_TIME); 
        SCL=0;
        delay_i2c(HD_TIME);
        SDA=0;
        delay_i2c(HD_TIME);
}

/*--------Write 1byte to i2c--------*/
void WriteI2CByte(char b){       //Write 1byte to i2c    
        char i;
        for(i=0;i<8;i++){
            if((b<<i)&0x80)
                SEND_1();
            else
                SEND_0();      
        }

}
/*--------Write 1byte to i2c--------*/

/*-------------配置I2C--------------*/
void Write_I2C(unsigned char addr,unsigned char datad){
        SCL_C=1;
        SDA_C=1; 
        
        SCL=0;
        SDA=0; 
        I2C_Stop(); 
         
         
         delay_i2c(10000);
          I2C_Start();
          WriteI2CByte(CAM_ADDRESS);
          
            delay_i2c(HD_TIME);
            SCL=1;
            delay_i2c(HD_TIME);   
            SCL=0;
            delay_i2c(HD_TIME);
       //  ACK_CHECK();       
          WriteI2CByte(addr);
           delay_i2c(HD_TIME);
            SCL=1;
            delay_i2c(HD_TIME);   
            SCL=0;
            delay_i2c(HD_TIME);
        // ACK_CHECK(); 
          WriteI2CByte(datad);
           delay_i2c(HD_TIME);
            SCL=1;
            delay_i2c(HD_TIME);   
            SCL=0;
            delay_i2c(HD_TIME);
        // ACK_CHECK();
          I2C_Stop();
  }        
/*-------------配置I2C--------------*/
