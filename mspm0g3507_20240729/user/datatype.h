#ifndef __DATATYPE_H
#define __DATATYPE_H


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>





typedef   signed          char int8;
typedef unsigned          char _u8;
typedef unsigned          char uint8;
typedef unsigned          char byte;
typedef   signed short    int int16;
typedef unsigned short    int uint16;
typedef unsigned short    int _u16;
typedef unsigned long     int _u32; 


enum 
{
	_ROL=0,
	_PIT,
	_YAW
};

enum 
{
	ROL=0,
	PIT,
	YAW
};

typedef struct
{
  float x;
  float y;
  float z;
}vector3f;

typedef struct
{
  float x;
  float y;
}vector2f;


typedef struct
{
	uint8_t bit1	:1;
	uint8_t bit2	:1;
	uint8_t bit3	:1;
	uint8_t bit4	:1;
	uint8_t bit5	:1;
	uint8_t bit6	:1;
	uint8_t bit7	:1;
	uint8_t bit8	:1;
	uint8_t bit9	:1;
	uint8_t bit10	:1;
	uint8_t bit11	:1;
	uint8_t bit12	:1;
	uint8_t bit13	:1;
	uint8_t bit14	:1;
	uint8_t bit15	:1;
	uint8_t bit16	:1;
}gray_flags;


typedef struct
{
  volatile float last_time;
  volatile float current_time;
  volatile float period;
  volatile uint16_t period_int;//��λms
}systime;



typedef struct
{
	uint8_t update_flag;
  uint16_t buf[8];
	uint8_t online_flag;
}mppm;

typedef enum
{
	WHO_AM_I_MPU6050  =0x68,
	WHO_AM_I_ICM20689 =0x98,
	WHO_AM_I_ICM20608D=0xAE,
	WHO_AM_I_ICM20608G=0xAF,
	WHO_AM_I_ICM20602=0x12,
}IMU_ID_READ;


typedef struct
{
	int8_t left_encoder_dir_config,right_encoder_dir_config;//��������������
	int8_t left_motion_dir_config	,right_motion_dir_config; //����˶���������
	float wheel_radius_cm;				//��̥�뾶,��λΪcm
	uint16_t pulse_num_per_circle;//��̥ת��һȦ�ۼƵ���������
	uint16_t servo_median_value1,servo_median_value2;
}motor_config;

typedef struct
{
	int16_t left_motor_cnt,right_motor_cnt;//�������������ڵ���������
	int8_t left_motor_dir,right_motor_dir; //�˶�����
	float left_motor_speed_rpm,right_motor_speed_rpm;//ת�ٵ�λתÿ����
	float left_motor_gyro_rps,right_motor_gyro_rps;//ת�ٵ�λrad/s
	float left_motor_speed_cmps,right_motor_speed_cmps;//ת��c��λΪcm/s
	float left_motor_period_ms,right_motor_period_ms;
	
	int32_t left_motor_total_cnt,right_motor_total_cnt;
	int32_t left_motor_period_cnt,right_motor_period_cnt;
	uint8_t left_motor_cnt_clear,right_motor_cnt_clear;
	
}encoder;

typedef struct
{
	float value;
	uint8_t enable;
	float upper;
	float lower;
	uint16_t low_vbat_cnt;
}low_voltage;


typedef struct
{
	float speed;
	float azimuth;
	float w;
	vector2f pos;
	vector2f vel;
	float distance;
}two_wheel_model;



typedef struct
{
  vector3f _gyro_dps_raw,gyro_dps_raw;
  vector3f _accel_g_raw,accel_g_raw;
  vector3f mag_tesla_raw;
  vector3f last_mag_raw;
	float temperature_raw,last_temperature_raw;
  float temperature_filter;
	float vbat;
	//У׼�������
	vector3f gyro_dps;
	vector3f accel_g;
	vector3f mag_tesla;
	
	//
	vector3f gyro_offset;
	vector3f accel_scale,accel_offset;
	
	//
	float left_motor_speed_cmps;
	float right_motor_speed_cmps;
	float average_speed_cmps;
	
	
	uint8_t quaternion_init_ok;
	float quaternion_init[4];//��ʼ��Ԫ��
	float quaternion[4];//��Ԫ��
	float rpy_deg[3];
	float rpy_gyro_dps[3];
	float rpy_gyro_dps_enu[3];
	vector3f accel_earth_cmpss;
	vector2f accel_body_cmpss;
	float sin_rpy[3];
	float cos_rpy[3];
	float cb2n[9];
	float rpy_obs_deg[3];//�۲���̬�Ƕ�
	float rpy_kalman_deg[3];
	//
	float yaw_gyro_enu;
	//
	uint16_t imu_convergence_cnt;
	uint8_t imu_convergence_flag;
	uint8_t temperature_stable_flag;
	uint8_t imu_cal_flag;
	uint8_t imu_health;
	uint8_t lpf_init;
	two_wheel_model state_estimation;
}sensor;


typedef enum
{
	LOCK 		=0x00,
	UNLOCK  =0x01,
}LOCK_STATE;


#define ABS(X)  (((X)>0)?(X):-(X))
#define MAX(a,b)  ((a)>(b)?(a):(b))
#define MIN(a,b)  ((a)>(b)?(b):(a))


typedef enum 
{
  BODY_FRAME=0,//��������ϵ
	MAP_FRAME,	 //��������ϵ
}Navigation_Frame;

typedef enum 
{
  RELATIVE_MODE=0,//���ģʽ
	GLOBAL_MODE,	  //ȫ��ģʽ
	CMD_VEL_MODE,   //�ٶ�ģʽ
	TRANSITION_MODE //����ģʽ
}Navigation_Mode;


typedef struct{
	uint16_t number;								//������
	float x;												//x����λ��/�ٶ�����
	float y;												//y����λ��/�ٶ�����
	float z;												//z����λ��/ƫ�����ٶ�����
	uint8_t nav_mode;								//����ģʽ
	uint8_t frame_id;								//����ϵ����
	
	uint8_t update_flag;						//ָ����±�־λ
	uint8_t ctrl_finish_flag;				//������ϱ�־λ
	
	uint16_t cnt; 									//λ�ÿ�������жϼ�����
	float dis_cm;										//��λ��ƫ��
	const float dis_limit_cm;       //λ����ֵ
	const float cmd_vel_max;        //������ٶ�����
	const float cmd_angular_max;    //�����ٶ�����
	
	uint8_t cmd_vel_update;					//�ٶȿ���ָ����±�־λ
	float cmd_vel_x;								//����x�����ٶ�
	float cmd_vel_y;                //����y�����ٶ�
	float cmd_vel_angular_z;				//����ƫ��������ٶ�
	uint32_t cmd_vel_during_cnt;    //�ٶȿ��Ƽ�����
	uint8_t cmd_vel_suspend_flag;   //������ֹ��־λ
}nav_ctrl;


#endif


