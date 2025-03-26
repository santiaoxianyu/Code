#ifndef __VOFA_H
#define __VOFA_H

#include "stdint.h"
#include "stm32f4xx.h"                  // Device header
#include "pid.h"
#include "string.h"
#include "stdio.h"
#include "usart.h"


float Get_Data(void);
void SendDataToVOFA(float target, int16_t speed,int16_t speed1);
void SendDataToVOFA_two(int16_t target,float yaw );
void SendDataToVOFA_two(int16_t target,float yaw ) ;
void SendDataToVOFA(float target, int16_t speed,int16_t speed1) ;

#endif
