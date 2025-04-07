#ifndef __RC_H
#define __RC_H

#define  RC_TOP_DEFAULT       2000
#define  RC_BUTTOM_DEFAULT    1000
#define  RC_MIDDLE_DEFAULT    1500
#define  RC_DEADBAND_DEFAULT  100
#define  RC_DEADBAND_PERCENT   		0.2f		//��λ����ռʵ���г̵İٷֱ�
#define  RC_THR_DEADBAND_PERCENT  0.2f		//������λ����ռʵ���г̵İٷֱ�
#define  RC_RESET_DEFAULT  1500




#define  Scale_Pecent_Max  0.75f   //��������ֵ��������
#define  Pit_Rol_Max 35.0f           //��������������
#define  Yaw_Max     300.0f          //���ƫ������
#define  Buttom_Safe_Deadband  50.0f //���ŵײ���ȫ����
/*****************ң�����г�����**********************/
#define  Climb_Up_Speed_Max    300//������������ٶȣ�cm/s  400  300
#define  Climb_Down_Speed_Max  150//��������½��ٶȣ�cm/s  200  150



#define  Front_Speed_Max 200//ǰ���ٶȣ�cm/s
#define  Back_Speed_Max  200//�����ٶȣ�cm/s

typedef enum 
{
  RC_ROLL=0,
	RC_PITCH,
	RC_YAW,
	RC_THR
}RC_RPYT;


typedef enum 
{
	RC_ROLL_CHANNEL=0,
	RC_PITCH_CHANNEL,
	RC_THR_CHANNEL,
	RC_YAW_CHANNEL,
	RC_AUX1_CHANNEL,
	RC_AUX2_CHANNEL,
	RC_AUX3_CHANNEL,
	RC_AUX4_CHANNEL,
	RC_CHL_MAX
}RC_CHL_MAP;

typedef enum 
{
  AUX1=0,
	AUX2,
	AUX3,
	AUX4
}AUX;

typedef enum 
{
	THR_BUTTOM=0,
	THR_MIDDLE=1,
	THR_UP=2,
}THR_POSITION;

typedef enum 
{
	FREE_STYLE=0,
	BUTTOM_TO_MIDDLE,
	MIDDLE_TO_BUTTOM,
	MIDDLE_TO_UP,
	UP_TO_MIDDLE,
}THR_PUSH_MODE;

typedef struct
{
  uint16_t max;					//���ֵ
	uint16_t min;					//��Сֵ
	uint16_t middle;			//�м�ֵ
	uint16_t deadband;    //��λ����ֵ
	bool reverse_flag;    //�����־λ
	float scale;
}rc_calibration;



typedef struct
{
  uint16_t rcdata[RC_CHL_MAX];
	rc_calibration cal[RC_CHL_MAX];
  float rc_rpyt[4];
  uint16_t thr;
  uint16_t aux[4];
	
	THR_PUSH_MODE thr_push_over_state;
	
	uint16_t unlock_makesure_cnt;
	uint16_t lock_makesure_cnt;
	LOCK_STATE lock_state;
	LOCK_STATE last_lock_state;
	int16_t auto_relock_cnt;
	uint8_t auto_relock_flag;
	uint8_t rc_return_flag;
	uint8_t unwanted_lock_flag;
	uint8_t fc_ready_flag;
}rc;





void rc_range_init(void);
void rc_data_input(void);
float remote_data_remap(rc *data,uint16_t ch,float max_down,float max_up,bool reverse_flag);
extern rc RC_Data,RC_Pilot;
#endif


