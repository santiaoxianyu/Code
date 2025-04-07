#include "Headfile.h"
#include "rc.h"


////////////////ң�����г̲�����ʼ��





void unlock_state_check(void);


rc RC_Data={
	.fc_ready_flag=0
};

rc RC_Pilot={
	.fc_ready_flag=0
};

/***************************************************
������: void rc_range_init(void)
˵��:	ң�����г̲�����ʼ��
���:	��
����:	��
��ע:	��
����:	��������
****************************************************/
void rc_range_init(void)
{
	float _rc_deadband_percent=0;
	for(uint16_t i=0;i<8;i++)
	{		
		if(i==RC_THR_CHANNEL) _rc_deadband_percent=RC_THR_DEADBAND_PERCENT;
		else _rc_deadband_percent=RC_DEADBAND_PERCENT;
		
		RC_Data.cal[i].max=RC_TOP_DEFAULT;
		RC_Data.cal[i].min=RC_BUTTOM_DEFAULT;
		RC_Data.cal[i].middle=RC_MIDDLE_DEFAULT;
		
		RC_Data.cal[i].deadband=(float)((RC_Data.cal[i].max-RC_Data.cal[i].min)*_rc_deadband_percent/1.0f);;
		RC_Data.cal[i].reverse_flag=false;
		RC_Data.cal[i].scale=(RC_Data.cal[i].max-RC_Data.cal[i].min-RC_Data.cal[i].deadband)*0.5f;
	}
	
	RC_Pilot=RC_Data;
}

/***************************************************
������: float remote_data_remap(rc *data,uint16_t ch,float max_down,float max_up,bool reverse_flag)
˵��:	ң��������ӳ�亯��
���:	rc *data-ң�����ṹ��
			uint16_t ch-ͨ��
			float max_down-�������ֵ
			float max_up-�������ֵ
			bool reverse_flag-�������
����:	float ӳ�������ֵ
��ע:	��
����:	��������
****************************************************/
float remote_data_remap(rc *data,uint16_t ch,float max_down,float max_up,bool reverse_flag)
{
	if(ppm_rc.online_flag==0) return 0;
  float value=0;
  if(data->rcdata[ch]<=data->cal[ch].middle-0.5f*data->cal[ch].deadband)
		value=(data->cal[ch].middle-0.5f*data->cal[ch].deadband-data->rcdata[ch])*max_down
														/data->cal[ch].scale;
	else if(data->rcdata[ch]>=data->cal[ch].middle+0.5f*data->cal[ch].deadband)
		value=(data->cal[ch].middle+0.5f*data->cal[ch].deadband-data->rcdata[ch])*max_up
														/data->cal[ch].scale;	
  else value=0;
	
	if(reverse_flag)  value*=(-1);
  return 	value;
}


uint8_t rc_read_switch(uint16_t ch)
{
	uint16_t pulsewidth = ch;
	if (pulsewidth <= 900 || pulsewidth >= 2200) return 255;// This is an error condition
	if (pulsewidth <= 1210) return 0;//��λ
	if (pulsewidth <= 1710) return 1;//��λ
	return 2;//��λ
}


/***************************************************
������: float rc_data_input(void)
˵��:	ң�������ݲɼ�
���:	��
����:	��
��ע:	��
����:	��������
****************************************************/
void rc_data_input(void)
{
	if(ppm_rc.update_flag==1)//����ѡ���ⲿ���ջ������PPM�ź�
	{
		memcpy(RC_Data.rcdata,ppm_rc.buf,RC_CHL_MAX*sizeof(uint16_t));//����ÿ��ͨ��ң��������
		memcpy(RC_Pilot.rcdata,ppm_rc.buf,RC_CHL_MAX*sizeof(uint16_t));//����ÿ��ͨ��ң��������
	}
	else if(rc_update_flag==1)//������������ң����
	{
		NCLINK_PPM_Databuf[1]=3000-NCLINK_PPM_Databuf[1];//�ڶ�ͨ��������
	  memcpy(RC_Data.rcdata,NCLINK_PPM_Databuf,RC_CHL_MAX*sizeof(uint16));
		memcpy(RC_Pilot.rcdata,NCLINK_PPM_Databuf,RC_CHL_MAX*sizeof(uint16_t));
	}
	else if(bt_update_flag==1)//����appң��������
	{
	  bluetooth_ppm[1]=3000-bluetooth_ppm[1];//�ڶ�ͨ��������
	  memcpy(RC_Data.rcdata,bluetooth_ppm,RC_CHL_MAX*sizeof(uint16));	
		memcpy(RC_Pilot.rcdata,bluetooth_ppm,RC_CHL_MAX*sizeof(uint16_t));
	}
	
	if(ppm_rc.update_flag==1||rc_update_flag==1||bt_update_flag==1)
	{
		//��ձ��θ��±�־λ
		ppm_rc.update_flag=0;
		rc_update_flag=0;
		bt_update_flag=0;
		
		ppm_rc.online_flag=1;//����ң���������ź�
		
		RC_Data.rc_rpyt[RC_ROLL]	=remote_data_remap(&RC_Data ,RC_ROLL_CHANNEL	,Yaw_Max,Yaw_Max,false);
		RC_Data.rc_rpyt[RC_PITCH] =remote_data_remap(&RC_Data ,RC_PITCH_CHANNEL ,Back_Speed_Max,Front_Speed_Max,false);
		RC_Data.rc_rpyt[RC_YAW]		=remote_data_remap(&RC_Data ,RC_YAW_CHANNEL	  ,Yaw_Max		,Yaw_Max    ,false);
		RC_Data.rc_rpyt[RC_THR]		=remote_data_remap(&RC_Data ,RC_THR_CHANNEL	  ,Back_Speed_Max,Front_Speed_Max,true);
		
		RC_Data.thr=RC_Data.rcdata[RC_THR_CHANNEL];
		RC_Data.aux[AUX1]=RC_Data.rcdata[RC_AUX1_CHANNEL];
		RC_Data.aux[AUX2]=RC_Data.rcdata[RC_AUX2_CHANNEL];
		RC_Data.aux[AUX3]=RC_Data.rcdata[RC_AUX3_CHANNEL];
		RC_Data.aux[AUX4]=RC_Data.rcdata[RC_AUX4_CHANNEL];
		
		/********************************************************************************************************************/
		RC_Pilot.rc_rpyt[RC_ROLL]	=remote_data_remap(&RC_Pilot ,RC_ROLL_CHANNEL	,Pit_Rol_Max,Pit_Rol_Max,true);
		RC_Pilot.rc_rpyt[RC_PITCH]=remote_data_remap(&RC_Pilot ,RC_PITCH_CHANNEL,Pit_Rol_Max,Pit_Rol_Max,true);
		RC_Pilot.rc_rpyt[RC_YAW]	=remote_data_remap(&RC_Pilot ,RC_YAW_CHANNEL	,Yaw_Max		,Yaw_Max    ,false);
		RC_Pilot.rc_rpyt[RC_THR]	=remote_data_remap(&RC_Pilot ,RC_THR_CHANNEL	,Climb_Down_Speed_Max,Climb_Up_Speed_Max,true);
	
		RC_Pilot.thr=RC_Pilot.rcdata[RC_THR_CHANNEL];
		RC_Pilot.aux[AUX1]=RC_Pilot.rcdata[RC_AUX1_CHANNEL];
		RC_Pilot.aux[AUX2]=RC_Pilot.rcdata[RC_AUX2_CHANNEL];
		RC_Pilot.aux[AUX3]=RC_Pilot.rcdata[RC_AUX3_CHANNEL];
		RC_Pilot.aux[AUX4]=RC_Pilot.rcdata[RC_AUX4_CHANNEL];
		/********************************************************************************************************************/
		unlock_state_check();
	}
}


uint16_t imu_cal_cnt=0;
void unlock_state_check(void)
{
	RC_Pilot.fc_ready_flag=smartcar_imu.quaternion_init_ok;

	RC_Pilot.last_lock_state=RC_Pilot.lock_state;
	//�����߼�
  if(RC_Pilot.thr<RC_Pilot.cal[RC_THR_CHANNEL].min+RC_Pilot.cal[RC_THR_CHANNEL].scale*0.05f
		 &&RC_Pilot.rc_rpyt[RC_ROLL]==0
		 &&RC_Pilot.rc_rpyt[RC_PITCH]==0
	   &&RC_Pilot.rc_rpyt[RC_YAW]<=-Yaw_Max*Scale_Pecent_Max
		 &&RC_Pilot.fc_ready_flag==1)//�¿ؾ�λ
	RC_Pilot.unlock_makesure_cnt++;
	else RC_Pilot.unlock_makesure_cnt/=5;
  
	if(RC_Pilot.unlock_makesure_cnt>=100)//100*20ms=2000ms����2Sʱ��
	{
	  RC_Pilot.lock_state=UNLOCK;
		RC_Pilot.unlock_makesure_cnt=0;
		bling_set(&light_green,5000,200,0.5,0,0);//��ɫ
	}

	//�����߼�
	if(RC_Pilot.thr<RC_Pilot.cal[RC_THR_CHANNEL].min+RC_Pilot.cal[RC_THR_CHANNEL].scale*0.05f
	 &&RC_Pilot.rc_rpyt[RC_ROLL]==0
	 &&RC_Pilot.rc_rpyt[RC_PITCH]==0
	 &&RC_Pilot.rc_rpyt[RC_YAW]>=Yaw_Max*Scale_Pecent_Max)
	RC_Pilot.lock_makesure_cnt++;
	else RC_Pilot.lock_makesure_cnt/=5;
		
	if(RC_Pilot.lock_makesure_cnt>=50)//50*20ms=1000ms����1Sʱ��
	{
	  RC_Pilot.lock_state=LOCK;
		RC_Pilot.lock_makesure_cnt=0;
		RC_Pilot.auto_relock_flag=0;//����Զ�������־λ
		RC_Pilot.rc_return_flag=0;
		RC_Pilot.auto_relock_cnt=0;
		bling_set(&light_green,3000,500,0.5,0,0);//��ɫ
	}
		
	//�ɿ��Զ������ж�
	if(RC_Pilot.last_lock_state==LOCK&&RC_Pilot.lock_state==UNLOCK)//�������ʱ��
	{ 
		//rc_data.auto_relock_flag=0;//������ģʽ�£���ֹ�������Զ�����
		RC_Pilot.auto_relock_flag=1;  //�����Զ�������־λ
		RC_Pilot.auto_relock_cnt=300; //���ý������κβ�����6����Զ�����
		
		RC_Pilot.unwanted_lock_flag=1;//��ֹ�������Զ�������־λ	
		RC_Pilot.thr_push_over_state=FREE_STYLE;
		//reset_landon_state();
	}
	
	
	//�������Զ������߼��ж�
	if(RC_Pilot.auto_relock_flag==1)
	{
		 if(RC_Pilot.thr<RC_Pilot.cal[RC_THR_CHANNEL].min+RC_Pilot.cal[RC_THR_CHANNEL].scale*0.05f
		  &&RC_Pilot.rc_rpyt[RC_ROLL]==0
		  &&RC_Pilot.rc_rpyt[RC_PITCH]==0
		  &&RC_Pilot.rc_rpyt[RC_YAW]==0)//ң����������6S�����κβ�������λ
		 {	
				RC_Pilot.auto_relock_cnt--;
				if(RC_Pilot.auto_relock_cnt<=0)  RC_Pilot.auto_relock_cnt=0;
				if(RC_Pilot.auto_relock_cnt==0)
				{
					RC_Pilot.lock_state=LOCK;
					RC_Pilot.auto_relock_flag=0;//����Զ�������־λ
					RC_Pilot.rc_return_flag=0;
					bling_set(&light_green,3000,500,0.5,0,0);//��ɫ
				}
				RC_Pilot.rc_return_flag=1;//���ж�����־λ
		 }
			
		 if((RC_Pilot.thr>RC_Pilot.cal[RC_THR_CHANNEL].min+RC_Pilot.cal[RC_THR_CHANNEL].scale*0.05f
		   ||ABS(RC_Pilot.rc_rpyt[RC_ROLL])>=0.1f*Pit_Rol_Max
		   ||ABS(RC_Pilot.rc_rpyt[RC_PITCH])>=0.1f*Pit_Rol_Max
		   ||ABS(RC_Pilot.rc_rpyt[RC_YAW])>=0.5f*Yaw_Max)
		   &&RC_Pilot.rc_return_flag==1&&RC_Pilot.auto_relock_cnt>0)//ң�����������к�6S�ڴ��ڲ�������λ
		 {	
			 RC_Pilot.auto_relock_flag=0;//����Զ�������־λ
			 RC_Pilot.rc_return_flag=0;
			 RC_Pilot.auto_relock_cnt=0;			 
		 }		 
	}
	
	
	if(RC_Pilot.thr<RC_Pilot.cal[RC_THR_CHANNEL].min+RC_Pilot.cal[RC_THR_CHANNEL].scale*0.05f
	 &&RC_Pilot.rc_rpyt[RC_ROLL] >=Pit_Rol_Max*Scale_Pecent_Max
	 &&RC_Pilot.rc_rpyt[RC_PITCH]>=Pit_Rol_Max*Scale_Pecent_Max
	 &&RC_Pilot.rc_rpyt[RC_YAW]  >=Yaw_Max*Scale_Pecent_Max
	 &&RC_Pilot.last_lock_state==LOCK
	 &&RC_Pilot.fc_ready_flag==1)//�¿ؾ�λ
	imu_cal_cnt++;
	else imu_cal_cnt/=2;

	if(imu_cal_cnt>=200)//200*20ms=4000ms����4Sʱ��
	{
		smartcar_imu.imu_cal_flag=0;//����У׼���ٶȡ�������
		imu_cal_cnt=0;
		bling_set(&light_green,5000,200,0.5,0,0);//��ɫ
	}
}

