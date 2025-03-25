#include "key.h"

uint8_t mode;

#define enter HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_4)
#define select HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_3)

int key(void)
{
    OLED_ShowString(1,1,"mode:");
    OLED_ShowNum(1,6,mode,2);
    if(select==0){HAL_Delay(100); mode++;}
    if(mode>5){mode=0;}
    if(enter==0){return mode;}
}
