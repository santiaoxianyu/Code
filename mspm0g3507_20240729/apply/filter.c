#include "Headfile.h"
#include "filter.h"

/***************************************************
������: float LPButterworth(float curr_input,lpf_buf *buf,lpf_param *params)
˵��:	���װ�����˹��ͨ�˲���
���:	float curr_input-��ǰ�˲�������
			lpf_buf *buf-�˲����м�״̬
			lpf_param *params-�˲�������
����:	float �˲������ֵ
��ע:	��
����:	��������
****************************************************/
float LPButterworth(float curr_input,lpf_buf *buf,lpf_param *params)
{
	if(buf->output[0]==0&&
		 buf->output[1]==0&&
		 buf->output[2]==0&&
		 buf->input[0]==0&&
		 buf->input[1]==0&&
		 buf->input[2]==0)
	{
		buf->output[0]=curr_input;
		buf->output[1]=curr_input;
		buf->output[2]=curr_input;
		buf->input[0]=curr_input;
		buf->input[1]=curr_input;
		buf->input[2]=curr_input;
		return curr_input;
	}
	
  /* ���ٶȼ�Butterworth�˲� */
  /* ��ȡ����x(n) */
  buf->input[2]=curr_input;
  /* Butterworth�˲� */
  buf->output[2]=params->b[0] * buf->input[2]
													+params->b[1] * buf->input[1]
													+params->b[2] * buf->input[0]
													-params->a[1] * buf->output[1]
													-params->a[2] * buf->output[0];
  /* x(n) ���б��� */
  buf->input[0]=buf->input[1];
  buf->input[1]=buf->input[2];
  /* y(n) ���б��� */
  buf->output[0]=buf->output[1];
  buf->output[1]=buf->output[2];
	
	for(uint16_t i=0;i<3;i++)
	{
	  if(isnan(buf->output[i])==1
			||isnan(buf->input[i])==1)		
			{		
				buf->output[0]=curr_input;
				buf->output[1]=curr_input;
				buf->output[2]=curr_input;
				buf->input[0]=curr_input;
				buf->input[1]=curr_input;
				buf->input[2]=curr_input;
				return curr_input;
			}
	}	
  return buf->output[2];
}

/***************************************************
������: void set_cutoff_frequency(float sample_frequent, float cutoff_frequent,lpf_param *LPF)
˵��:	���װ�����˹��ͨ�˲����������
���:	float sample_frequent-����Ƶ��
			float cutoff_frequent-��ֹƵ��
			lpf_param *LPF-�˲�������
����:	��
��ע:	��
����:	��������
****************************************************/
void set_cutoff_frequency(float sample_frequent, float cutoff_frequent,lpf_param *LPF)
{
  float fr = sample_frequent / cutoff_frequent;
  float ohm = tanf(M_PI_F / fr);
  float c = 1.0f + 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm;
  if (cutoff_frequent <= 0.0f) {
    // no filtering
    return;
  }
  LPF->b[0] = ohm * ohm / c;
  LPF->b[1] = 2.0f * LPF->b[0];
  LPF->b[2] = LPF->b[0];
  LPF->a[0]=1.0f;
  LPF->a[1] = 2.0f * (ohm * ohm - 1.0f) / c;
  LPF->a[2] = (1.0f - 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm) / c;
}

