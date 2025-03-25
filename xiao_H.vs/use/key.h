#ifndef __KEY_H_
#define __KEY_H_

#include "gpio.h"
#include "OLED.h"

#define enter HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_4)
#define select HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_3)

int key(void);

#endif 
