#include "grayscale.h"



int8_t track_assignment[12];
int8_t track_sum;

uint8_t Track_follow(void)
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

int8_t B1_put,B2_put,B3_put,B4_put;

void button_scan(void)
{
	DL_GPIO_readPins(KEYB_PORT,PORTB_D3_1_PIN);
	DL_GPIO_readPins(KEYB_PORT,PORTB_D3_2_PIN);
	DL_GPIO_readPins(KEYB_PORT,PORTB_D3_3_PIN);
	DL_GPIO_readPins(KEYB_PORT,PORTB_D3_4_PIN);
  if( DL_GPIO_readPins(KEYB_PORT,PORTB_D3_1_PIN)>0)
	{
			B1_put=1;	
  }
  else
  {
      B1_put=0;  
	}
	if( DL_GPIO_readPins(KEYB_PORT,PORTB_D3_2_PIN)>0)
	{
			B2_put=1;	
  }
  else
  {
      B2_put=0;  
	}
	if( DL_GPIO_readPins(KEYB_PORT,PORTB_D3_3_PIN)>0)
	{
			B3_put=1;	
  }
  else
  {
      B3_put=0;  
	}
	if( DL_GPIO_readPins(KEYB_PORT,PORTB_D3_4_PIN)>0)
	{
			B4_put=1;	
  }
  else
  {
      B4_put=0;  
	}
}
