#ifndef __SUBTASK_H
#define __SUBTASK_H


/**********************************************************************************************************************/
#define track_speed_cmps_default  20				//Ĭ�ϲ���켣Ѱ��ʱ��Ѳ���ٶ�10 
#define start_point_adjust1_default -2				//ʶ�𵽵�������Ӿ����������ǰ���������Ҫ�����ľ���-8.0f

#define forward_distance_cm_default 20				//��������һ������ִ�еľ��롪����̥���ǰ��
#define backward_distance1_cm_default 27			//�������ڶ�������ִ�еľ��롪����̥�Ҵ򲢺���
#define backward_distance2_cm_default 30      //����������������ִ�еľ��롪����̥����������

#define out_forward_distance1_cm_default 25		//������������һ����ִ�о��롪����̥������ǰ��
#define out_forward_distance2_cm_default 40   //�����������ڶ�����ִ�о��롪����̥�Ҵ�ǰ��

/**********************************************************************************************************************/
#define start_point_adjust2_default 25////ʶ�𵽲෽����Ӿ����������ǰ���������Ҫ�����ľ���30  25
#define parallel_backward_distance1_cm_default 22.0f//�෽����һ������ִ�еľ��롪��ǰ���Ҵ�ʱ�����˾���  20 25
#define parallel_backward_distance2_cm_default 17.5f//�෽���ڶ�������ִ�еľ��롪��ǰ�ֻ���ʱ�����˾���  20
#define parallel_backward_distance3_cm_default 15.0f//�෽������������ִ�еľ��롪��ǰ�����ʱ�����˾���	15
/**********************************************************************************************************************/


typedef struct
{
	float _track_speed_cmps;
	float _start_point_adjust1;
	float _forward_distance_cm;
	float _backward_distance1_cm;
	float _backward_distance2_cm;
	float _out_forward_distance1_cm;
	float _out_forward_distance2_cm;
	float _start_point_adjust2;
	float _parallel_backward_distance1_cm;
	float _parallel_backward_distance2_cm;
	float _parallel_backward_distance3_cm;
}_park_params;




void flight_subtask_reset(void);


void flight_subtask_1(void);
void flight_subtask_2(void);
void flight_subtask_3(void);
void flight_subtask_4(void);
void flight_subtask_5(void);
void auto_reverse_stall_park(void);
void auto_parallel_park(void);
	
extern _park_params park_params;


#endif
