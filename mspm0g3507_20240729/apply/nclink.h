#ifndef __NCLINK_H__
#define __NCLINK_H__

#define RESERVED_PARAM_NUM 20
extern float param_value[RESERVED_PARAM_NUM];


#define BYTE0(dwTemp)  (*((char *)(&dwTemp)))
#define BYTE1(dwTemp)  (*((char *)(&dwTemp)+1))
#define BYTE2(dwTemp)  (*((char *)(&dwTemp)+2))
#define BYTE3(dwTemp)  (*((char *)(&dwTemp)+3))
	
#define NCLINK_STATUS         0x01//���ͷɿ�״̬���ݣ������ֽ�
#define NCLINK_SENSOR         0x02//���ʹ�����ԭʼ���ݣ������ֽ�
#define NCLINK_RCDATA         0x03//����ң�����������ݣ������ֽ�
#define NCLINK_GPS 		        0x04//����GPS��γ�ȡ��������ݣ������ֽ�
#define NCLINK_OBS_NE         0x05//����NE����۲����ݣ������ֽ�
#define NCLINK_OBS_UOP        0x06//����U����۲����ݣ������ֽ�
#define NCLINK_FUS_U          0x07//����U�����ں����ݣ������ֽ�
#define NCLINK_FUS_NE         0x08//����NE�����ں����ݣ������ֽ�
#define NCLINK_USER           0x09//�����û����ݣ������ֽ�

#define NCLINK_SEND_PID1_3    0x0A//PID01-03���ݣ������ֽ�+Ӧ���ֽ�
#define NCLINK_SEND_PID4_6    0x0B//PID04-06���ݣ������ֽ�+Ӧ���ֽ�
#define NCLINK_SEND_PID7_9    0x0C//PID07-09���ݣ������ֽ�+Ӧ���ֽ�
#define NCLINK_SEND_PID10_12  0x0D//PID10-12���ݣ������ֽ�+Ӧ���ֽ�
#define NCLINK_SEND_PID13_15  0x0E//PID13-15���ݣ������ֽ�+Ӧ���ֽ�
#define NCLINK_SEND_PID16_18  0x0F//PID16-18���ݣ������ֽ�+Ӧ���ֽ�

#define NCLINK_SEND_PARA      0x10//�ɿط������������������ֽ�+Ӧ���ֽ�
#define NCLINK_SEND_CAL_RAW1  0x11//����У׼ԭʼ����1�������ֽ�
#define NCLINK_SEND_CAL_RAW2  0x12//����У׼ԭʼ����2�������ֽ�
#define NCLINK_SEND_CAL_PARA1 0x13//����У׼�������1�������ֽ�
#define NCLINK_SEND_CAL_PARA2 0x14//����У׼�������2�������ֽ�
#define NCLINK_SEND_CAL_PARA3 0x15//����У׼�������3�������ֽ�

#define NCLINK_SEND_PARA_RESERVED		0x16//�ɿط���Ԥ�������������ֽ�+Ӧ���ֽ�
#define NCLINK_SEND_3D_TRACK				0x17//�ɿ�3Dλ�����ݷ��Ͳ���



#define NCLINK_SEND_CHECK     					0xF0//����Ӧ�����ݣ������ֽ�
//Ӧ������
#define NCLINK_SEND_RC        					0xF1//�ɿؽ���ң��������ָ�Ӧ���ֽ�
#define NCLINK_SEND_DIS       					0xF2//�ɿؽ���λ�ƿ���ָ�Ӧ���ֽ�
#define NCLINK_SEND_CAL       					0xF3//�ɿش�����У׼��ϣ�Ӧ���ֽ�
#define NCLINK_SEND_CAL_READ  					0xF4//��ȡУ׼�����ɹ���Ӧ���ֽ�
#define NCLINK_SEND_FACTORY   					0xF5//�ָ��������óɹ���Ӧ���ֽ�
#define NCLINK_SEND_NAV_CTRL  					0xF6//�ɿؽ�����������ָ�Ӧ���ֽ�
#define NCLINK_SEND_NAV_CTRL_FINISH     0xF7//�ɿص���������ϣ�Ӧ���ֽ�
#define NCLINK_SEND_SLAM_SYSTEM_RESET   0xF8//SLAM������λ��ϣ������ֽ�
#define NCLINK_SEND_SLAM_STOP_MOTOR     0xF9//SLAM���Ƶ��ֹͣ�������ֽ�
#define NCLINK_SEND_SLAM_START_MOTOR    0xFA//SLAM���Ƶ����ʼ�������ֽ�
//���ݽṹ����
typedef enum
{
	SDK_FRONT = 0x00,
	SDK_BEHIND,
	SDK_LEFT,
	SDK_RIGHT,
	SDK_UP,
	SDK_DOWN
}SDK_MODE;

typedef struct
{
  bool sdk_front_flag;
	bool sdk_behind_flag;
	bool sdk_left_flag;
	bool sdk_right_flag;
	bool sdk_up_flag;
	bool sdk_down_flag;
}sdk_mode_flag;


typedef struct
{
  uint8_t move_mode;//SDK�ƶ���־λ
  uint8_t mode_order;//SDK�ƶ�˳��
  uint16_t move_distance;//SDK�ƶ�����
	bool update_flag;
	sdk_mode_flag move_flag;
	float f_distance;
}ngs_sdk_control;




void NCLink_SEND_StateMachine(void);
void NCLink_Data_Prase_Prepare_Lite(uint8_t data);


void bluetooth_app_send(float user1,float user2,float user3,float user4,
												float user5,float user6,float user7,float user8);
void bluetooth_app_prase(uint8_t byte);

extern nav_ctrl ngs_nav_ctrl;
extern uint16_t NCLINK_PPM_Databuf[10];
extern uint8_t rc_update_flag;
extern uint8_t unlock_flag,takeoff_flag;
extern uint16_t bluetooth_ppm[8];
extern uint8_t bt_update_flag;

#endif

