#include "key.h"

uint8_t mode;

int key(void)
{
    OLED_ShowString(1,1,"mode:");
    OLED_ShowNum(1,6,mode,2);

    if (select==RESET)
    {
        HAL_Delay(100); 
        mode++;
    }
    if(mode>=5){mode=0;}
    if(enter==0){HAL_Delay(100);return mode;}
}
