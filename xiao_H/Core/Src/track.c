 #include "track.h"
#include "gpio.h"

int8_t track_read[8];
uint16_t track_Mode;
int8_t track_assignment[8];
int8_t track_sum;

void Track_follow(void)
{
	
	track_read[3]=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2);
	track_read[2]=HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_8);
	track_read[1]=HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_6);
	track_read[0]=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11);
	track_read[4]=HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_7);
	track_read[5]=HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_6);
	track_read[6]=HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_9);
	track_read[7]=HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_11);

	
	track_assignment[0] = track_read[3] ? 0 : 1;
	track_assignment[1] = track_read[2] ? 0 : 2;
	track_assignment[4] = track_read[1] ? 0 : 3;
	track_assignment[5] = track_read[0] ? 0 : 4;
	track_assignment[2] = track_read[4] ? 0 : -1;
	track_assignment[3] = track_read[5] ? 0 : -2;
	track_assignment[6] = track_read[6] ? 0 : -3;
	track_assignment[7] = track_read[7] ? 0 : -4;	
	
	track_sum=track_assignment[0]+track_assignment[1]+track_assignment[2]+track_assignment[3]+track_assignment[4]+track_assignment[5]+track_assignment[6]+track_assignment[7];				
}

