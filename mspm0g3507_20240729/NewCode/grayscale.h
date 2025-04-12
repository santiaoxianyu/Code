#ifndef __GRAYSCALE_H
#define __GRAYSCALE_H

#include "headfile.h"
#define H1   ((PORTA_PORT->DIN31_0 & PORTA_GRAY_BIT0_PIN ) ? 1 : 0)
#define H2   ((PORTA_PORT->DIN31_0 & PORTA_GRAY_BIT1_PIN ) ? 1 : 0)
#define H3   ((PORTA_PORT->DIN31_0 & PORTA_GRAY_BIT2_PIN ) ? 1 : 0)
#define H4   ((PORTA_PORT->DIN31_0 & PORTA_GRAY_BIT3_PIN ) ? 1 : 0)
#define H5   ((PORTA_PORT->DIN31_0 & PORTA_GRAY_BIT4_PIN ) ? 1 : 0)
#define H6   ((PORTA_PORT->DIN31_0 & PORTA_GRAY_BIT5_PIN ) ? 1 : 0)
#define H7   ((PORTB_PORT->DIN31_0 & PORTB_GRAY_BIT6_PIN ) ? 1 : 0)
#define H8   ((PORTB_PORT->DIN31_0 & PORTB_GRAY_BIT7_PIN ) ? 1 : 0)
#define H9   ((PORTB_PORT->DIN31_0 & PORTB_GRAY_BIT8_PIN ) ? 1 : 0)
#define H10  ((PORTB_PORT->DIN31_0 & PORTB_GRAY_BIT9_PIN ) ? 1 : 0)
#define H11  ((PORTA_PORT->DIN31_0 & PORTA_GRAY_BIT10_PIN) ? 1 : 0)
#define H12  ((PORTB_PORT->DIN31_0 & PORTB_GRAY_BIT11_PIN) ? 1 : 0)

#define All_Sensor1 H1==1&&H2==1&&H3==1&&H4==1&&H5==1&&H6==1&&H7==1&&H8==1&&H9==1&&H10==1&&H11==1&&H12==1
#define One_Sensor1 H1==0||H2==0||H3==0||H4==0||H5==0||H6==0||H7==0||H8==0||H9==0||H10==0||H11==0||H12==0
void Track_follow(void);

#endif
