#ifndef _PID_H
#define _PID_H
 
#include "stm32f4xx.h"
#include "headfile.h"

typedef struct {
    float Err;
    float last_err;
    float sum;
    float pwm;
    float p;
    float i;
    float d;
} PID_Controller;

float pid1(PID_Controller *pid,int16_t speed1,float tar1);
float track_pid(PID_Controller *pid,int8_t  track_err);
float turn_PID_yaw(PID_Controller *pid,float yaw,float aim);
//void steering_ring(void);//转向环
//void Track_ring(void);//寻迹环
//void steering_ring(void);           //转向环
//void Track_ring(void);              //寻迹环
 
#endif