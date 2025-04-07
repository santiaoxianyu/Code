#include "ti_msp_dl_config.h"
#include "datatype.h"
#include "ngpio.h"
#include "nadc.h"
#include "user.h"

uint16_t adc_value;

low_voltage vbat={
	.enable=no_voltage_enable_default,
	.value=0,
	.upper=no_voltage_upper_default,
	.lower=no_voltage_lower_default
};


void nADC_Init(void)
{
	NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);
}


/***************************************
������:	void get_battery_voltage(void)
˵��: ADC��������
���:	��
����:	��
��ע:	������ص�ѹ���ɿ�Ĭ�Ϸ�ѹ��Ϊ11���ʲ�����ѹ��Ҫ����
3.3V*11=36.6V����������ĵ�ѹ�����������Ϸ�ѹ������ֵ����
����:	��������
***************************************/
float get_battery_voltage(void)//ADC��ȡ   
{
	static float value_filter;
	value_filter=0.9f*value_filter+0.1f*adc_value*36.3f/4095.0f;	
	vbat.value=value_filter;
  return vbat.value;	
}

/***************************************
������:	void battery_voltage_detection(void)
˵��: ��ص�ѹ���
���:	��
����:	��
��ע:	��Ҫ�趨��ѹ��ֵ
����:	��������
***************************************/
void battery_voltage_detection(void)
{
	static uint16_t _cnt=0;
	_cnt++;
	if(_cnt>=200)//ÿ1S���һ��
	{
		_cnt=0;
		if(vbat.value<vbat.upper&&vbat.value>vbat.lower)	 vbat.low_vbat_cnt++;
		else vbat.low_vbat_cnt/=2;
		if(vbat.low_vbat_cnt>=10)//����10s����
		{
			if(vbat.enable==1)
			{
				beep.period=200;//200*5ms
				beep.light_on_percent=0.25f;
				beep.reset=1;	
				beep.times=5;//��˸5��
			}
			vbat.low_vbat_cnt=0;//��0����һ���ڼ��			
		}			
	}
}

volatile bool gCheckADC=false;
void adc_statemachine(void)
{
	DL_ADC12_startConversion(ADC12_0_INST);	//��ʼת��
	/* Wait until all data channels have been loaded. */
	while (gCheckADC == false);
	adc_value = DL_ADC12_getMemResult(ADC12_0_INST, ADC12_0_ADCMEM_1);//ͨ��PA25��ȡADֵ
	DL_ADC12_enableConversions(ADC12_0_INST);
}

void ADC12_0_INST_IRQHandler(void)
{
    switch (DL_ADC12_getPendingInterrupt(ADC12_0_INST)) {
        case DL_ADC12_IIDX_MEM1_RESULT_LOADED:
            gCheckADC = true;
            break;
        default:
            break;
    }
}
