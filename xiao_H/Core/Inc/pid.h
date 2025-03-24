#ifndef _PID_H
#define _PID_H
 
#include "stm32f4xx.h"

typedef struct {
    float Err;
    float last_err;
	  float sum;
    float pwm;
    float p;
    float i;
    float d;
} PID_Controller;




 
#endif