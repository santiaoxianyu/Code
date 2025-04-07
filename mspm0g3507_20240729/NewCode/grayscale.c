#include "grayscale.h"

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

int8_t track_assignment[12];
int8_t track_sum;

double RetrunXunJiValue(void)
{
	track_assignment[0] = H12 ? 0 : 6;
	track_assignment[1] = H11 ? 0 : 5;
	track_assignment[2] = H10 ? 0 : 4;
	track_assignment[3] = H9 ? 0 : 3;
	track_assignment[4] = H8 ? 0 : 2;
	track_assignment[5] = H7 ? 0 : 1;
	track_assignment[6] = H6 ? 0 : -1;
	track_assignment[7] = H5 ? 0 : -2;
	track_assignment[8] = H4 ? 0 : -3;
	track_assignment[9] = H3 ? 0 : -4;
	track_assignment[10] = H2 ? 0 : -5;
	track_assignment[11] = H1 ? 0 : -6;
	
	track_sum=track_assignment[0]+track_assignment[1]+track_assignment[2]+track_assignment[3]+track_assignment[4]
	          +track_assignment[5]+track_assignment[6]+track_assignment[7]+track_assignment[8]+track_assignment[9]+track_assignment[10]+track_assignment[11];				
	
	return track_sum;
}

