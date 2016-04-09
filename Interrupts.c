#include "main.h"

void set_speed_PID(void){
  speed_now=PACNT;          
  PID_speed(); 
  PACNT=0;
}
         

