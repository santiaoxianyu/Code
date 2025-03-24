#include "motor.h"

#define PWM_MAX 7200
#define PWM_MIN -7200

extern TIM_HandleTypeDef htim1;

int abs(int p)
{
	if(p>0)
		return p;
	else
		return -p;
}

void Load(int moto1,int moto2)			//-7200~7200
{
	if(moto1>0)
	{
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_5,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_5,GPIO_PIN_SET);
	}	
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,abs(moto1));	
	if(moto2>0)
	{
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_3,GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_3,GPIO_PIN_RESET);
	}
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,abs(moto2));
}

void Limit(int *motoA,int *motoB)
{
	if(*motoA>PWM_MAX)*motoA=PWM_MAX;
	if(*motoA<PWM_MIN)*motoA=PWM_MIN;
	if(*motoB>PWM_MAX)*motoB=PWM_MAX;
	if(*motoB<PWM_MIN)*motoB=PWM_MIN;
}
