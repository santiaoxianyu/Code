#include "headfile.h"
#include "sensor.h"
#include "Fusion.h"

#define temperature_ctrl_enable 0//IMU���¿���ʹ��
																 //1��ʹ��MSPM0ջ����չ��-���¿�
																 //0�����MPU6050-�����¿�
#define dt_1 0.080


extern uint8_t ID;//??????ID????
extern  int16_t AX, AY, AZ, GX, GY, GZ;//?????????????
//////////////////////////mpu??
extern double Gyro_z, Gyro_x, Gyro_y;
extern float Acc_x,Acc_y;
extern double fil_Acc_x,fil_Acc_y,fil_Gyro_z,fil_Gyro_x,fil_Gyro_y;
extern double Angle_Z, Angle_Y, Angle_X;
extern float coe_Gyro_z;
extern float ICM20602_FIFO_Z[11], ICM20602_FIFO_Y[11], ICM20602_FIFO_X[11];
extern float imu660ra_acc[11];
extern int moto_flag;
extern int gyro_i;



extern float anglez_mpu;
extern float anglez_mpu2;
extern float anglez_mpu3;

sensor angle_imu;

lpf_param accel_lpf_param,gyro_lpf_param;
lpf_buf gyro_filter_buf[3],accel_filter_buf[3];

sensor smartcar_imu;
FusionAhrs ahrs;
FusionOffset offset;
#define sampling_frequent 200

/***************************************************
������: vector3f_sub(vector3f a,vector3f b,vector3f *c)
˵��:	��������
���:	vector3f a-������
			vector3f b-����
			vector3f *c-��
����:	��
��ע:	��
����:	��������
****************************************************/
void vector3f_sub(vector3f a,vector3f b,vector3f *c)
{
	c->x=a.x-b.x;
  c->y=a.y-b.y;
  c->z=a.z-b.z;
}

/***************************************************
������: void euler_to_quaternion(float *rpy,float *q)
˵��:	ŷ����ת��Ԫ��
���:	float *rpy-ŷ����
			float *q-��Ԫ��
����:	��
��ע:	��
����:	��������
****************************************************/
void euler_to_quaternion(float *rpy,float *q)
{
	float sPitch2, cPitch2; // sin(phi/2) and cos(phi/2)
	float sRoll2 , cRoll2;  // sin(theta/2) and cos(theta/2)
	float sYaw2  , cYaw2;   // sin(psi/2) and cos(psi/2)
	//calculate sines and cosines
	
	FastSinCos(0.5f * rpy[0]*DEG2RAD, &sRoll2, &cRoll2);//roll
	FastSinCos(0.5f * rpy[1]*DEG2RAD, &sPitch2,&cPitch2);//pitch
	FastSinCos(0.5f * rpy[2]*DEG2RAD, &sYaw2,  &cYaw2);//yaw
	
	// compute the quaternion elements
	q[0] = cPitch2*cRoll2*cYaw2+sPitch2*sRoll2*sYaw2;
	q[1] = sPitch2*cRoll2*cYaw2-cPitch2*sRoll2*sYaw2;
	q[2] = cPitch2*sRoll2*cYaw2+sPitch2*cRoll2*sYaw2;
	q[3] = cPitch2*cRoll2*sYaw2-sPitch2*sRoll2*cYaw2;

  // Normalise quaternion
  float recipNorm = invSqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
  q[0] *= recipNorm;
  q[1] *= recipNorm;
  q[2] *= recipNorm;
  q[3] *= recipNorm;
}

/***************************************************
������: void quaternion_to_cb2n(float *q,float *cb2n)
˵��: ͨ����Ԫ�����㷽�����Ҿ���
���:	float *q-��Ԫ��
			float *cb2n-�������Ҿ���
����:	��
��ע:	��
����:	��������
****************************************************/
void quaternion_to_cb2n(float *q,float *cb2n)
{
	float a=q[0];
	float b=q[1];
	float c=q[2];
	float d=q[3];
	float bc=b*c;
	float ad=a*d;
	float bd=b*d;
	float ac=a*c;
	float cd=c*d;
  float ab=a*b;
	float a2=a*a;
	float b2=b*b;
	float c2=c*c;
	float d2=d*d;
  cb2n[0]=a2+b2-c2-d2;
  cb2n[1]=2*(bc-ad);
  cb2n[2]=2*(bd+ac);  
  cb2n[3]=2*(bc+ad);
  cb2n[4]=a2-b2+c2-d2;
  cb2n[5]=2*(cd-ab);
  cb2n[6]=2*(bd-ac);
  cb2n[7]=2*(cd+ab);
  cb2n[8]=a2-b2-c2+d2;	
}

/***************************************************
������: void calculate_quaternion_init(vector3f a,vector3f m,float *q)
˵��: ͨ�����ٶȼơ����������ݼ����ʼ��Ԫ��
���:	vector3f a-������ٶȼ�
			vector3f m-���������
			float *q-��Ԫ��
����:	��
��ע:	��
����:	��������
****************************************************/
void calculate_quaternion_init(vector3f a,vector3f m,float *q)
{
	float ax,ay,az,mx,my,mz;
	float rpy_obs_deg[3],_sin_rpy[2],_cos_rpy[2];
	
	ax= a.x;	ay= a.y;	az= a.z;
	mx= m.x;	my= m.y;	mz= m.z;	
	float norm = invSqrt(mx * mx + my * my + az * az);
	mx *=	norm;
	my *=	norm;
	mz *=	norm;
	
  rpy_obs_deg[0]=-57.3f*atan(ax*invSqrt(ay*ay+az*az)); //�����
  rpy_obs_deg[1]= 57.3f*atan(ay*invSqrt(ax*ax+az*az)); //������
	_sin_rpy[_PIT] =	sinf(rpy_obs_deg[1]*DEG2RAD);
	_cos_rpy[_PIT] =	cosf(rpy_obs_deg[1]*DEG2RAD);
	_sin_rpy[_ROL] =	sinf(rpy_obs_deg[0]*DEG2RAD);
	_cos_rpy[_ROL] =	cosf(rpy_obs_deg[0]*DEG2RAD);

  /************��������ǲ���*****************/
	vector2f magn;	
	magn.y=  my * _cos_rpy[_PIT]- mz * _sin_rpy[_PIT];
	magn.x=  mx * _cos_rpy[_ROL]
					+my * _sin_rpy[_ROL] * _cos_rpy[_PIT]
					+mz * _sin_rpy[_ROL] * _cos_rpy[_PIT];
  /***********�����еõ������ƹ۲�Ƕ�*********/
  rpy_obs_deg[2] = atan2f(magn.x,magn.y)*57.296f;
	if(rpy_obs_deg[2]<0) rpy_obs_deg[2] = rpy_obs_deg[2]+360;
	rpy_obs_deg[2] = constrain_float(rpy_obs_deg[2],0.0f,360);
	//
	euler_to_quaternion(rpy_obs_deg,q);//����۲���Ԫ��
}

/***************************************************
������: float kalman_filter(float angle,float gyro)
˵��:	�Ǿ��󿨶����˲�
���:	float angle-�۲�ŷ����
			float gyro-���ٶ�
����:	��
��ע:	��
����:	��������
****************************************************/
float kalman_filter(float angle,float gyro)
{
	static uint8_t init;
	static float x=0; 
	static float P=0.000001; 
	static float Q=0.000001; 
	static float R=0.35;//0.35 
	static float k=0; //*************
	if(init==0)
	{
		x=angle;
		init=1;
	}
	x=x+gyro*0.005f; 
	P=P+Q; 
	k=P/(P+R); 
	x=x+k*(angle-x);
	P=(1-k)*P;
	return x; 
} 


/***************************************************
������: void imu_data_sampling(void)
˵��:	IMU���ݲ���/У׼/�˲�
���:	��
����:	��
��ע:	��
����:	��������
****************************************************/
void imu_data_sampling(void)
{
	if(smartcar_imu.lpf_init==0)
	{
		set_cutoff_frequency(200, 50,&gyro_lpf_param); //��̬���ٶȷ����˲����� 
		set_cutoff_frequency(200, 30,&accel_lpf_param);//��̬����Ӽ������˲�ֵ
	  smartcar_imu.lpf_init=1;
	}
	smartcar_imu.last_temperature_raw=smartcar_imu.temperature_raw;
	//������/���������ݲɼ�
	ICM206xx_Read_Data(&smartcar_imu._gyro_dps_raw,&smartcar_imu._accel_g_raw,&smartcar_imu.temperature_raw);
	//���������ݵ�ͨ�˲�
	smartcar_imu.gyro_dps_raw.x=LPButterworth(smartcar_imu._gyro_dps_raw.x,&gyro_filter_buf[0],&gyro_lpf_param);
  smartcar_imu.gyro_dps_raw.y=LPButterworth(smartcar_imu._gyro_dps_raw.y,&gyro_filter_buf[1],&gyro_lpf_param);
  smartcar_imu.gyro_dps_raw.z=LPButterworth(smartcar_imu._gyro_dps_raw.z,&gyro_filter_buf[2],&gyro_lpf_param);		
	//���ٶ����ݵ�ͨ�˲�
	smartcar_imu.accel_g_raw.x=LPButterworth(smartcar_imu._accel_g_raw.x,&accel_filter_buf[0],&accel_lpf_param);
	smartcar_imu.accel_g_raw.y=LPButterworth(smartcar_imu._accel_g_raw.y,&accel_filter_buf[1],&accel_lpf_param);
	smartcar_imu.accel_g_raw.z=LPButterworth(smartcar_imu._accel_g_raw.z,&accel_filter_buf[2],&accel_lpf_param);	////////
	//�¶ȴ���������һ�׵�ͨ�˲�
	smartcar_imu.temperature_filter=0.75f*smartcar_imu.temperature_raw+0.25f*smartcar_imu.temperature_filter;
  //�õ�У׼��Ľ��ٶȡ����ٶȡ�����������
	vector3f_sub(smartcar_imu.gyro_dps_raw,smartcar_imu.gyro_offset,&smartcar_imu.gyro_dps);
  
	smartcar_imu.accel_g.x=smartcar_imu.accel_scale.x*smartcar_imu.accel_g_raw.x-smartcar_imu.accel_offset.x;
  smartcar_imu.accel_g.y=smartcar_imu.accel_scale.y*smartcar_imu.accel_g_raw.y-smartcar_imu.accel_offset.y;
  smartcar_imu.accel_g.z=smartcar_imu.accel_scale.z*smartcar_imu.accel_g_raw.z-smartcar_imu.accel_offset.z;  	
	//���ٶȼ�/������У׼���
	imu_calibration(&smartcar_imu.gyro_dps_raw,&smartcar_imu.accel_g_raw);
	
	//ͨ��������ٶȼ�,����ˮƽ�۲�Ƕ�
	float ax,ay,az;
	ax=smartcar_imu.accel_g.x;
	ay=smartcar_imu.accel_g.y;
	az=smartcar_imu.accel_g.z;//9.8/9.8=1
	
  smartcar_imu.rpy_obs_deg[0]=-57.3f*atan(ax*invSqrt(ay*ay+az*az)); //�����
  smartcar_imu.rpy_obs_deg[1]= 57.3f*atan(ay*invSqrt(ax*ax+az*az)); //������ 
//	smartcar_imu.rpy_obs_deg[2]= 57.3f*atan(az*invSqrt(ax*ax+ay*ay));//������

	//��������̬�ǿ������˲�
	smartcar_imu.rpy_kalman_deg[1]=kalman_filter(smartcar_imu.rpy_obs_deg[1],smartcar_imu.gyro_dps.x);
	smartcar_imu.rpy_kalman_deg[0]=kalman_filter(smartcar_imu.rpy_obs_deg[0],smartcar_imu.gyro_dps.y);
	
	
}








void ICM20602_newValues()
{
     float sum_x=0,sum_y=0,sum_z=0;
     static float gyroz[100],sum_gyro_z,gyroy[100],sum_gyro_y,gyrox[100],sum_gyro_x;
     static int gyro_flag=0,Gyro_flag;
	ICM206xx_Read_Data(&angle_imu._gyro_dps_raw,&angle_imu._accel_g_raw,&angle_imu.temperature_raw);
	//	GZ=angle_imu._gyro_dps_raw.z;
							GZ=		smartcar_imu.gyro_dps_raw.z;
    GY=angle_imu._gyro_dps_raw.y;
	  GX=angle_imu._gyro_dps_raw.x;
  //   MPU6050_getData(&AX, &AY, &AZ, &GX, &GY, &GZ);
     //????,??????,??????????,????
        if(gyro_flag==0)
     {
          gyroz[gyro_i]=GZ;
          gyroy[gyro_i]=GY;
          gyrox[gyro_i]=GX;

          fil_Gyro_z=0.0;
          fil_Gyro_y=0.0;
          fil_Gyro_x=0.0;

          gyro_i++;
         if(gyro_i==99)
         {
             moto_flag=1;
             for(gyro_i=0;gyro_i<100;gyro_i++)
             {
                 sum_gyro_z+=gyroz[gyro_i];
                 sum_gyro_y+=gyroy[gyro_i];
                 sum_gyro_x+=gyrox[gyro_i];
             }
             gyro_flag=1;
         }
     }
     if(gyro_flag==1)
     {
    Gyro_z = (float)(GZ-sum_gyro_z/100)* 2000.0f / 32767.0f;
        Gyro_x = (float)(GX-sum_gyro_x/100)* 2000.0f / 32767.0f;
        Gyro_y = (float)(GY-sum_gyro_y/100)* 2000.0f / 32767.0f;
      for(Gyro_flag=1;Gyro_flag<10;Gyro_flag++)
        {
          ICM20602_FIFO_Z[Gyro_flag-1]=ICM20602_FIFO_Z[Gyro_flag];//FIFO ??
            ICM20602_FIFO_Y[Gyro_flag-1]=ICM20602_FIFO_Y[Gyro_flag];//FIFO ??
            ICM20602_FIFO_X[Gyro_flag-1]=ICM20602_FIFO_X[Gyro_flag];//FIFO ??
        }
      ICM20602_FIFO_Z[9]=Gyro_z;
        ICM20602_FIFO_Y[9]=Gyro_y;
        ICM20602_FIFO_X[9]=Gyro_x;
      for(Gyro_flag=0;Gyro_flag<10;Gyro_flag++)
        {
            sum_z+=ICM20602_FIFO_Z[Gyro_flag];//???????,?????
            sum_y+=ICM20602_FIFO_Y[Gyro_flag];//???????,?????
            sum_x+=ICM20602_FIFO_X[Gyro_flag];//???????,?????
        }
      fil_Gyro_z=sum_z/10;
        fil_Gyro_y=sum_y/10;
        fil_Gyro_x=sum_x/10;
        Angle_Z-=fil_Gyro_z*dt_1;
        Angle_Y-=fil_Gyro_y*dt_1;
        Angle_X-=fil_Gyro_x*dt_1;
        if(Angle_Z>=360)Angle_Z-=360;
        else if(Angle_Z<=-360)Angle_Z+=360;

    }
}














/***************************************************
������: void trackless_ahrs_update(void)
˵��:	��̬����
���:	��
����:	��
��ע:	��
����:	��������
****************************************************/
void trackless_ahrs_update(void)
{
	/****************************************************/
	FusionVector gyroscope={0.0f, 0.0f, 0.0f};
	FusionVector accelerometer = {0.0f, 0.0f, 1.0f};
	FusionVector earthacceleration = {0.0f, 0.0f, 0.0f}; 
	
	gyroscope.axis.x=smartcar_imu.gyro_dps.x;
	gyroscope.axis.y=smartcar_imu.gyro_dps.y;
	gyroscope.axis.z=smartcar_imu.gyro_dps.z;
	
	accelerometer.axis.x=smartcar_imu.accel_g.x;
	accelerometer.axis.y=smartcar_imu.accel_g.y;
	accelerometer.axis.z=smartcar_imu.accel_g.z;
	if(smartcar_imu.quaternion_init_ok==0)
	{
		if(smartcar_imu.temperature_stable_flag==1)//�¶��ȶ�
		{
			calculate_quaternion_init(smartcar_imu.accel_g,smartcar_imu.mag_tesla,smartcar_imu.quaternion_init);
			smartcar_imu.quaternion_init_ok	=	1;
			//AHRS��ʼ��
			FusionOffsetInitialise(&offset, sampling_frequent);
			FusionAhrsInitialise(&ahrs);
			//Set AHRS algorithm settings
			const FusionAhrsSettings settings = {
							.gain = 0.5f,
							.accelerationRejection = 10.0f,
							.magneticRejection = 20.0f,
							.rejectionTimeout = 5 * sampling_frequent, /* 5 seconds */
			};
			FusionAhrsSetSettings(&ahrs, &settings);
		}		
	}

	if(smartcar_imu.quaternion_init_ok==1)
	{
		gyroscope = FusionOffsetUpdate(&offset, gyroscope);
		FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer,0.005f);//FusionAhrsUpdateExternalHeading(&ahrs, gyroscope, accelerometer,uav_ahrs.rpy_obs_deg[2],0.005f);
		FusionEuler euler=FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
		earthacceleration=FusionAhrsGetEarthAcceleration(&ahrs);
		smartcar_imu.rpy_deg[_ROL]=euler.angle.pitch;
		smartcar_imu.rpy_deg[_PIT]=euler.angle.roll;
		smartcar_imu.rpy_deg[_YAW]=euler.angle.yaw;
		//��ȡ����ϵϵͳ�˶����ٶ�
		smartcar_imu.accel_earth_cmpss.x=earthacceleration.axis.x*GRAVITY_MSS*100.0f;
		smartcar_imu.accel_earth_cmpss.y=earthacceleration.axis.y*GRAVITY_MSS*100.0f;
		smartcar_imu.accel_earth_cmpss.z=earthacceleration.axis.z*GRAVITY_MSS*100.0f;
		
		if(smartcar_imu.imu_convergence_cnt<200*5)  smartcar_imu.imu_convergence_cnt++;
		else if(smartcar_imu.imu_convergence_cnt==200*5)
		{
			smartcar_imu.imu_convergence_cnt++;
//			beep.period=100;//100*5ms
//			beep.light_on_percent=0.5f;
//			beep.reset=1;	
//			beep.times=2;//��˸2��		
		}
		else 
		{
			smartcar_imu.imu_convergence_flag=1;
		}
	}

  smartcar_imu.rpy_gyro_dps[_PIT]=smartcar_imu.gyro_dps.x;
  smartcar_imu.rpy_gyro_dps[_ROL]=smartcar_imu.gyro_dps.y;
  smartcar_imu.rpy_gyro_dps[_YAW]=smartcar_imu.gyro_dps.z;
	
  //������̬������Ǻ���
  smartcar_imu.sin_rpy[_PIT]=FastSin(smartcar_imu.rpy_deg[_PIT]*DEG2RAD);
  smartcar_imu.cos_rpy[_PIT]=FastCos(smartcar_imu.rpy_deg[_PIT]*DEG2RAD);
  smartcar_imu.sin_rpy[_ROL]=FastSin(smartcar_imu.rpy_deg[_ROL]*DEG2RAD);
  smartcar_imu.cos_rpy[_ROL]=FastCos(smartcar_imu.rpy_deg[_ROL]*DEG2RAD);
  smartcar_imu.sin_rpy[_YAW]=FastSin(smartcar_imu.rpy_deg[_YAW]*DEG2RAD);
  smartcar_imu.cos_rpy[_YAW]=FastCos(smartcar_imu.rpy_deg[_YAW]*DEG2RAD);
	//��ȡ��̬��Ԫ��
  smartcar_imu.quaternion[0]=ahrs.quaternion.element.w;
	smartcar_imu.quaternion[1]=ahrs.quaternion.element.x;
  smartcar_imu.quaternion[2]=ahrs.quaternion.element.y;
	smartcar_imu.quaternion[3]=ahrs.quaternion.element.z;
	//��������ϵ������ϵ��ת����
	quaternion_to_cb2n(smartcar_imu.quaternion,smartcar_imu.cb2n);						  //ͨ����Ԫ����ȡ��ת����	
	//�����˻��ڵ�������ϵ�µ���������������������˶����ٶ���ת����ǰ������˶����ٶ�:��ͷ(����)+��� 
  smartcar_imu.accel_body_cmpss.x= smartcar_imu.accel_earth_cmpss.x*smartcar_imu.cos_rpy[_YAW]+smartcar_imu.accel_earth_cmpss.y*smartcar_imu.sin_rpy[_YAW];  //��������˶����ٶ�  X������
  smartcar_imu.accel_body_cmpss.y=-smartcar_imu.accel_earth_cmpss.x*smartcar_imu.sin_rpy[_YAW]+smartcar_imu.accel_earth_cmpss.y*smartcar_imu.cos_rpy[_YAW];  //��ͷ�����˶����ٶ�  Y������
	
	//{-sin��          cos��sin ��                          cos��cos��                   }
  smartcar_imu.yaw_gyro_enu=-smartcar_imu.sin_rpy[_ROL]*smartcar_imu.gyro_dps.x
														+smartcar_imu.cos_rpy[_ROL]*smartcar_imu.sin_rpy[_PIT]*smartcar_imu.gyro_dps.y
														+smartcar_imu.cos_rpy[_PIT]*smartcar_imu.cos_rpy[_ROL]*smartcar_imu.gyro_dps.z;	
	smartcar_imu.vbat=get_battery_voltage();
//	anglez_mpu2=atan(invSqrt(smartcar_imu.sin_rpy[_YAW]/smartcar_imu.cos_rpy[_YAW]));
	
}



float temp_kp=8.0f,temp_ki=0.75f,temp_kd=125.0f;
float temp_error=0,temp_expect=50.0f,temp_feedback=0,temp_last_error=0;
float	temp_integral=0,temp_output=0;
/***************************************************
������: void imu_temperature_ctrl(void)
˵��:	IMU���¿���
���:	��
����:	��
��ע:	��
����:	��������
****************************************************/
void imu_temperature_ctrl(void)
{
	temperature_state_check();
	static uint16_t tmp_period_cnt=0;
	tmp_period_cnt++;
	if(tmp_period_cnt<20) return;
	tmp_period_cnt=0;
	
	float temp_dis_error=0;
	temp_last_error=temp_error;
	temp_feedback=smartcar_imu.temperature_filter;
	temp_error=temp_expect-temp_feedback;
	temp_error=constrain_float(temp_error,-50,50);
	if(ABS(temp_error)<10)	temp_integral+=temp_ki*temp_error*0.1f;
	temp_integral=constrain_float(temp_integral,-80,80);
	temp_dis_error=temp_error-temp_last_error;
	temp_output=temp_kp*temp_error+temp_integral+temp_kd*temp_dis_error;
	temp_output=constrain_float(temp_output,-100,100);
}

/***************************************************
������: void simulation_pwm_init(void)
˵��:	ģ��pwm��ʼ��
���:	��
����:	��
��ע:	��
����:	��������
****************************************************/
void simulation_pwm_init(void)
{	
	DL_GPIO_clearPins(heater_port, heater_pin);
}

#define Simulation_PWM_Period_MAX  100//100*1ms=0.1S
/***************************************************
������: void simulation_pwm_output(void)
˵��:	ģ��pwm���
���:	��
����:	��
��ע:	��
����:	��������
****************************************************/
void simulation_pwm_output(void)
{
#if temperature_ctrl_enable
	int16_t width=temp_output;
	static uint16_t cnt=0;	cnt++;
	if(cnt>=Simulation_PWM_Period_MAX)  cnt=0;
  if(cnt<=width) DL_GPIO_setPins(heater_port, heater_pin);
	else DL_GPIO_clearPins(heater_port, heater_pin);
#else
	DL_GPIO_clearPins(heater_port, heater_pin);
#endif
}

/***************************************************
������: uint8_t temperature_state_get(void)
˵��:	�¶Ƚӽ�Ŀ��ֵ���
���:	��
����:	uint8_t ��λ��־
��ע:	��
����:	��������
****************************************************/
uint8_t temperature_state_get(void)
{
#if temperature_ctrl_enable
	return (ABS(temp_error))<=3.0f?1:0;
#else
	return 1;
#endif
  
	
}

/***************************************************
������: void temperature_state_check(void)
˵��:	�¶Ⱥ㶨���
���:	��
����:	��
��ע:	��
����:	��������
****************************************************/
void temperature_state_check(void)
{
	static uint16_t _cnt=0;
	static uint16_t temperature_crash_cnt=0;
	if(temperature_state_get()==1){
		_cnt++;
		if(_cnt>=400) smartcar_imu.temperature_stable_flag=1;
	}
	else{
		_cnt/=2;
	}
	
	if(temperature_crash_cnt<400)
	{
		if(smartcar_imu.last_temperature_raw==smartcar_imu.temperature_raw)	temperature_crash_cnt++;
		else temperature_crash_cnt/=2;
		smartcar_imu.imu_health=1;		
	}
	else
	{
		smartcar_imu.imu_health=0;
	}	
}	


/***************************************************
������: void get_wheel_speed(void)
˵��:	������̼�ϵͳ
���:	��
����:	��
��ע:	���¶Ⱥ㶨֮��,��̬�����λ��ſ�ʼ����
����:	��������
****************************************************/
void get_wheel_speed(void)
{
	static uint32_t cnt=0;
	if(cnt<200)  { cnt++; return; }
	smartcar_imu.left_motor_speed_cmps =get_left_motor_speed();
	smartcar_imu.right_motor_speed_cmps=get_right_motor_speed();
	//��������ƽ���ٶ�
	smartcar_imu.state_estimation.speed=0.5f*(smartcar_imu.left_motor_speed_cmps+smartcar_imu.right_motor_speed_cmps);
	smartcar_imu.state_estimation.azimuth=smartcar_imu.rpy_deg[_YAW];
	smartcar_imu.state_estimation.w=smartcar_imu.yaw_gyro_enu;
	//����x,y�����ϵ��ٶȷ���
	smartcar_imu.state_estimation.vel.x=smartcar_imu.state_estimation.speed*cosf(DEG2RAD*smartcar_imu.state_estimation.azimuth);
	smartcar_imu.state_estimation.vel.y=smartcar_imu.state_estimation.speed*sinf(DEG2RAD*smartcar_imu.state_estimation.azimuth);
	//�ٶ�ֱ�ӻ��ֵõ����������λ��
	smartcar_imu.state_estimation.pos.x+=smartcar_imu.state_estimation.vel.x*0.005f;
	smartcar_imu.state_estimation.pos.y+=smartcar_imu.state_estimation.vel.y*0.005f;
	//ƽ���ٶ�ֱ�ӻ����ܾ���
	smartcar_imu.state_estimation.distance+=smartcar_imu.state_estimation.speed*0.005f;
}


