#include "key.h"

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

