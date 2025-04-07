#include "BeepLedTips.h"


uint8_t Beep_Led_Flag=0;
uint8_t Beep_Led_Close_Flag=0;

void BeepLedRun(void)//操作变量Beep_Led_Flag进行声光提示控制，置1开始声光提示，需要将此函数放在周期较长的中断里，保证声光提示明显
{
	if(Beep_Led_Close_Flag==1)
	{
		Beep_Led_Flag=0;
		Beep_Led_Close_Flag=0;
	}
	if(Beep_Led_Flag==1)
	{
		DL_GPIO_setPins(PORTA_PORT,PORTA_BEEP_PIN);
		Beep_Led_Close_Flag=1;
	}	
}