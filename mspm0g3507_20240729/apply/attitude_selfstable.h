#ifndef __ATTITUDE_SELFSTABLE_H
#define __ATTITUDE_SELFSTABLE_H



#define  THR_MAX_OUTPUT      2000//����������ֵ
#define  THR_MIN_OUTPUT      1000//���������Сֵ
#define  THR_IDEL_OUTPUT     1150//���ŵ��٣�ȡ�ӽ���ת����ֵ����1150
#define  THR_CONTROL_WORK    1150//�����������ƫ����������ʼ�����õ������� 1150
#define  THR_STARTUP_MIN     1050//��ת��������������ǲ����ã�̫��ᵼ�¹�����


void attitude_ctrl_init(void);
void attitude_control(void);
void takeoff_ctrl_reset(void);
void ncontroller_output(void);

#endif

