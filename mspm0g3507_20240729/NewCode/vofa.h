#ifndef __VOFA_H
#define __VOFA_H

#include "headfile.h"


float Get_Data(void);
void USART_PID_Adjust(uint8_t Motor_n);
void SendDataToVOFA(float target, int16_t speed,int16_t speed1);
void SendDataToVOFA_two(int16_t target,float yaw );
void SendDataToVOFA(float target, int16_t speed,int16_t speed1) ;

#endif
