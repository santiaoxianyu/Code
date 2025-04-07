#include "ti_msp_dl_config.h"
#include "ngpio.h"


_laser_light beep;

/***************************************
函数名:	void GPIO_Init(void)
说明: GPIO初始化
入口:	无
出口:	无
备注:	无
作者:	无名创新
***************************************/
void nGPIO_Init(void)
{
  beep.port = PORTA_PORT;
  beep.pin = PORTA_BEEP_PIN;
  beep.period = 200; //200*5ms
  beep.light_on_percent = 0.5f;
	
  beep.reset = 1;
  beep.times = 0;
}

/***************************************
函数名:	laser_light_work(_laser_light *light)
说明: gpio驱动状态机
入口:	_laser_light *light-gpio控制结构体
出口:	无
备注:	无
作者:	无名创新
***************************************/
void laser_light_work(_laser_light *light)
{
	if(light->reset==1)
	{
		light->reset=0;
		light->cnt=0;
		light->times_cnt=0;//点亮次数计数器
		light->end=0;
	}
	
	if(light->times_cnt==light->times)
	{
		light->end=1;
		return;
	}

	light->cnt++;
	if(light->cnt<=light->period*light->light_on_percent)
	{
		DL_GPIO_setPins(light->port, light->pin);
	}
	else if(light->cnt<light->period)
	{
		DL_GPIO_clearPins(light->port, light->pin);
	}
	else//完成点亮一次
	{
		light->cnt=0;
		light->times_cnt++;
	}
}
//	uint16_t times;				 //预设闪烁总次数
//	uint8_t  reset;				 //闪烁进程复位标志
//	uint16_t cnt;					 //闪烁控制计数器
//	uint16_t times_cnt;		 //记录已闪烁次数
//	uint8_t  end;					 //闪烁完成标志位
//	GPIO_Regs *port;		   //闪烁所在的端口
//	uint32_t pin;					 //闪烁所在的GPIO
//	uint32_t period;			 //闪烁周期
//	float light_on_percent;//单个周期内点亮时间百分比


void buzzer_setup(uint32_t _period, float _light_on_percent, uint16_t _times)
{
  beep.period = _period / 5; //20*5ms
  beep.light_on_percent = _light_on_percent;
  beep.reset = 1;
  beep.times = _times;
}