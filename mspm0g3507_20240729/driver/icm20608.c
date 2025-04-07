#include "ti_msp_dl_config.h"
#include "datatype.h"
#include "system.h"
#include "wp_math.h"
#include "ni2c.h"
#include "sensor.h"
#include "neeprom.h"
#include "ngpio.h"
#include "icm20608.h"



IMU_ID_READ IMU_ID=WHO_AM_I_ICM20608D;
static uint8_t imu_address=ICM20689_ADRESS;
uint8_t icm_read_register[5]={0x00,0x02,0x08,0x08,0x03};
uint8_t icm_read_check[5]={0};
/***************************************
������:	void ICM206xx_Init(void)
˵��: ���ٶȼ�/�����ǳ�ʼ��
���:	��
����:	uint8_t ����ʧ�ܱ�־
��ע:	��
����:	��������
***************************************/
uint8_t ICM206xx_Init(void)//ICM20689��ʼ��
{
	uint8_t fault=0;
	single_writei2c(imu_address,PWR_MGMT_1, 0x81);//���ǿ�Ƹ�λ81
	delay_ms(100);	
	IMU_ID=(IMU_ID_READ)(single_readi2c(imu_address,WHO_AM_I));
	switch(IMU_ID)
	{
		case WHO_AM_I_MPU6050:
		{
			single_writei2c(imu_address,PWR_MGMT_1  , 0x80);//�����λ
			delay_ms(200);
			single_writei2c(imu_address,PWR_MGMT_1  , 0x00);//�������
			delay_ms(10);
			single_writei2c(imu_address,SMPLRT_DIV  , 0x00); // sample rate.  Fsample= 1Khz/(<this value>+1) = 1000Hz
			single_writei2c(imu_address,PWR_MGMT_1  , 0x03); //ʱ��Դ PLL with Z Gyro reference
			single_writei2c(imu_address,MPU_CONFIG  , 0x02); //�ڲ���ͨ�˲�Ƶ�ʣ����ٶȼ�184hz��������188hz  //Ĭ��0x03	  
			single_writei2c(imu_address,GYRO_CONFIG , 0x08); //500deg/s
			single_writei2c(imu_address,ACCEL_CONFIG, 0x08); // Accel scale 4g (8192 LSB/g)			
		}
		break;
		case WHO_AM_I_ICM20689:
		{
			single_writei2c(imu_address,PWR_MGMT_1  , 0x00);//�ر������ж�,�������
			single_writei2c(imu_address,SMPLRT_DIV  , 0x00); // sample rate.  Fsample= 1Khz/(<this value>+1) = 1000Hz	
			delay_ms(100);			
			single_writei2c(imu_address,MPU_CONFIG  , 0x02);//0x00���������ǡ��¶��ڲ���ͨ�˲�Ƶ�ʷ�Χ��������250hz����������306.6hz���¶�4000hz
																														//0x01���������ǡ��¶��ڲ���ͨ�˲�Ƶ�ʷ�Χ��������176hz����������177hz���¶�188hz
																														//0x02���������ǡ��¶��ڲ���ͨ�˲�Ƶ�ʷ�Χ��������92hz����������108.6hz���¶�98hz
																														//0x03���������ǡ��¶��ڲ���ͨ�˲�Ƶ�ʷ�Χ��������41hz����������59hz���¶�42hz		
																														//0x04���������ǡ��¶��ڲ���ͨ�˲�Ƶ�ʷ�Χ��������20hz����������30.5hz���¶�20hz
																														//0x05���������ǡ��¶��ڲ���ͨ�˲�Ƶ�ʷ�Χ��������10hz����������15.6hz���¶�10hz
			single_writei2c(imu_address,GYRO_CONFIG , 0x08);//�������������̣�500deg/s
			single_writei2c(imu_address,ACCEL_CONFIG, 0x08);// Accel scale 4g (8192 LSB/g)	
			single_writei2c(imu_address,ACCEL_CONFIG2,0x03);
																										 //0x00���ü��ٶȼ��ڲ���ͨ�˲�Ƶ�ʷ�Χ�����ٶ�218.1hz����������235hz		
																										 //0x01���ü��ٶȼ��ڲ���ͨ�˲�Ƶ�ʷ�Χ�����ٶ�218.1hz����������235hz
																										 //0x02���ü��ٶȼ��ڲ���ͨ�˲�Ƶ�ʷ�Χ�����ٶ�99.0hz����������121.3hz		
																										 //0x03���ü��ٶȼ��ڲ���ͨ�˲�Ƶ�ʷ�Χ�����ٶ�44.8hz����������61.5hz
																										 //0x04���ü��ٶȼ��ڲ���ͨ�˲�Ƶ�ʷ�Χ�����ٶ�21.2hz����������31.0hz
																										 //0x05���ü��ٶȼ��ڲ���ͨ�˲�Ƶ�ʷ�Χ�����ٶ�10.2hz����������15.5hz	
			//Single_WriteI2C(imu_address,ACCEL_CONFIG2,0xC3);//���ü��ٶȼ��ڲ���ͨ�˲�Ƶ�ʷ�Χ�����ٶ�1046.0hz����������1100.0hz
		}
		break;	
		case WHO_AM_I_ICM20608D:
		case WHO_AM_I_ICM20608G:
		case WHO_AM_I_ICM20602:
		{
			single_writei2c(imu_address,PWR_MGMT_1,0X80);	//��λICM20608
			delay_ms(200);
			single_writei2c(imu_address,PWR_MGMT_1, 0X01);	//����ICM20608
			delay_ms(10);
			single_writei2c(imu_address,0x19, 0x00);   /* ����������ڲ������� */
			single_writei2c(imu_address,0x1A, 0x02);   /* �����ǵ�ͨ�˲�BW=92Hz */
			single_writei2c(imu_address,0x1B, 0x08);   /* �����ǡ�500dps���� */
			single_writei2c(imu_address,0x1C, 0x08);   /* ���ٶȼơ�16G���� */
			single_writei2c(imu_address,0x1D, 0x03);   /* ���ٶȼƵ�ͨ�˲�BW=44.8Hz */
			
			single_writei2c(imu_address,0x6C, 0x00);   /* �򿪼��ٶȼƺ������������� */
			single_writei2c(imu_address,0x1E, 0x00);   /* �رյ͹��� */
			single_writei2c(imu_address,0x23, 0x00);   /* �ر�FIFO */ 
			
			delay_ms(200);	
			icm_read_check[0]=single_readi2c(imu_address,0x19);
			icm_read_check[1]=single_readi2c(imu_address,0x1A);
			icm_read_check[2]=single_readi2c(imu_address,0x1B);
			icm_read_check[3]=single_readi2c(imu_address,0x1C);
			icm_read_check[4]=single_readi2c(imu_address,0x1D);
			for(uint8_t i=0;i<5;i++)
			{
				if(icm_read_check[i]!=icm_read_register[i]) fault=1;
			}	
		}
		break;
		default:
		{
			fault=1;
		}			
	}
	delay_ms(500);
	imu_calibration_params_init();	
	return fault;
}

/***************************************
������:	void ICM206xx_Read_Data(vector3f *gyro,vector3f *accel,float *temperature)
˵��: ��ȡ���������ٶȼ�/������/�¶�����
���:	vector3f *gyro-��ȡ��������������ָ��
			vector3f *accelgyro-��ȡ������ٶ�����ָ��
			float *temperaturegyro-��ȡ�¶�����ָ��
����:	��
��ע:	�����ǵ�λdeg/s,���ٶȼƵ�λg,�¶ȵ�λ��
����:	��������
***************************************/
void ICM206xx_Read_Data(vector3f *gyro,vector3f *accel,float *temperature)
{
	uint8_t buf[14];
	int16_t temp;
	i2creadnbyte(imu_address,ACCEL_XOUT_H,buf,14);
	switch(IMU_ID)
	{
		case WHO_AM_I_MPU6050:
		{
			accel->x=-(int16_t)((buf[0]<<8)|buf[1]);
			accel->y=-(int16_t)((buf[2]<<8)|buf[3]);
			accel->z= (int16_t)((buf[4]<<8)|buf[5]);	
			temp		=(int16_t)((buf[6]<<8)|buf[7]);
			gyro->x	=-(int16_t)((buf[8]<<8)|buf[9]);
			gyro->y	=-(int16_t)((buf[10]<<8)|buf[11]);
			gyro->z	= (int16_t)((buf[12]<<8)|buf[13]);	
			*temperature=36.53f+(float)(temp/340.0f);	
		}
		break;
		case WHO_AM_I_ICM20689:
		{
			accel->x=(int16_t)((buf[0]<<8)|buf[1]);
			accel->y=-(int16_t)((buf[2]<<8)|buf[3]);
			accel->z=-(int16_t)((buf[4]<<8)|buf[5]);	
			temp		=(int16_t)((buf[6]<<8)|buf[7]);
			gyro->x	=(int16_t)((buf[8]<<8)|buf[9]);
			gyro->y	=-(int16_t)((buf[10]<<8)|buf[11]);
			gyro->z	=-(int16_t)((buf[12]<<8)|buf[13]);	
			*temperature=25.0f+(double)((temp-25.0f)/326.8f);	
		}
		break;	
		case WHO_AM_I_ICM20608D:
		case WHO_AM_I_ICM20608G:
		case WHO_AM_I_ICM20602:				
		{
			accel->x=-(int16_t)((buf[0]<<8)|buf[1]);
			accel->y= (int16_t)((buf[2]<<8)|buf[3]);
			accel->z=-(int16_t)((buf[4]<<8)|buf[5]);	
			temp		=(int16_t)((buf[6]<<8)|buf[7]);
			gyro->x	=-(int16_t)((buf[8]<<8)|buf[9]);
			gyro->y	= (int16_t)((buf[10]<<8)|buf[11]);
			gyro->z	=-(int16_t)((buf[12]<<8)|buf[13]);	
			
			*temperature=25.0f+(double)((temp-25.0f)/326.8f);		
		}
		break;
		default:
		{
			accel->x=-(int16_t)((buf[0]<<8)|buf[1]);
			accel->y=-(int16_t)((buf[2]<<8)|buf[3]);
			accel->z= (int16_t)((buf[4]<<8)|buf[5]);	
			temp		=(int16_t)((buf[6]<<8)|buf[7]);
			gyro->x	=-(int16_t)((buf[8]<<8)|buf[9]);
			gyro->y	=-(int16_t)((buf[10]<<8)|buf[11]);
			gyro->z	= (int16_t)((buf[12]<<8)|buf[13]);	
			*temperature=36.53f+(float)(temp/340.0f);				
		}			
	}
	gyro->x*=GYRO_CALIBRATION_COFF;
	gyro->y*=GYRO_CALIBRATION_COFF;
	gyro->z*=GYRO_CALIBRATION_COFF;
	
  accel->x/=GRAVITY_RAW;
	accel->y/=GRAVITY_RAW;
	accel->z/=GRAVITY_RAW;
}


/***************************************
������:	void imu_calibration_params_init(void)
˵��: ���ٶȼ�/�����Ǳ궨���ݳ�ʼ��
���:	��
����:	��
��ע:	�ȴ�eeprom��ȡ����,�����ݲ����ڣ��ȴ��¶Ⱦ�λ���ٱ궨
����:	��������
***************************************/
void imu_calibration_params_init(void)
{
	vector3f gyro_offset_temp;
	ReadFlashParameterOne(GYRO_X_OFFSET,&gyro_offset_temp.x);
	ReadFlashParameterOne(GYRO_Y_OFFSET,&gyro_offset_temp.y);
	ReadFlashParameterOne(GYRO_Z_OFFSET,&gyro_offset_temp.z);	
	if(isnan(gyro_offset_temp.x)==0
		&&isnan(gyro_offset_temp.y)==0
		 &&isnan(gyro_offset_temp.z)==0)//���֮ǰ�Ѿ��¶�У׼��������ʱֱ����֮ǰУ׼������ 
	{
		smartcar_imu.gyro_offset.x=gyro_offset_temp.x;
		smartcar_imu.gyro_offset.y=gyro_offset_temp.y;
		smartcar_imu.gyro_offset.z=gyro_offset_temp.z;
		smartcar_imu.imu_cal_flag=1;
	}
	else
	{
		smartcar_imu.gyro_offset.x=0;
		smartcar_imu.gyro_offset.y=0;
		smartcar_imu.gyro_offset.z=0;
	}
	//���¶��ȶ����Զ�У׼��������ƫ
	smartcar_imu.accel_scale.x=1.0f;
	smartcar_imu.accel_scale.y=1.0f;
	smartcar_imu.accel_scale.z=1.0f;
	vector3f accel_offset_temp;
	ReadFlashParameterOne(ACCEL_X_OFFSET,&accel_offset_temp.x);
	ReadFlashParameterOne(ACCEL_Y_OFFSET,&accel_offset_temp.y);
	ReadFlashParameterOne(ACCEL_Z_OFFSET,&accel_offset_temp.z);	
	if(isnan(accel_offset_temp.x)==0
		&&isnan(accel_offset_temp.y)==0
		 &&isnan(accel_offset_temp.z)==0)//���֮ǰ�Ѿ��¶�У׼��������ʱֱ����֮ǰУ׼������ 
	{
		smartcar_imu.accel_offset.x=accel_offset_temp.x;
		smartcar_imu.accel_offset.y=accel_offset_temp.y;
		smartcar_imu.accel_offset.z=accel_offset_temp.z;
		smartcar_imu.imu_cal_flag=1;
	}
	else
	{
		smartcar_imu.accel_offset.x=0;
		smartcar_imu.accel_offset.y=0;
		smartcar_imu.accel_offset.z=0;
	}	
} 


#define gyro_delta_dps  3.0f
/***************************************
������:	void imu_calibration(vector3f *gyro,vector3f *accel)
˵��: ���ٶȼ�/�����Ǳ궨
���:	��
����:	��
��ע:	��
����:	��������
***************************************/
void imu_calibration(vector3f *gyro,vector3f *accel)
{
	if(smartcar_imu.imu_cal_flag==1)  return;
	
	static uint16_t cnt=0;
	static vector3f last_gyro;
	static vector3f accel_sum,gyro_sum;
	if(ABS(gyro->x-last_gyro.x)<=gyro_delta_dps
	 &&ABS(gyro->y-last_gyro.y)<=gyro_delta_dps
	 &&ABS(gyro->z-last_gyro.z)<=gyro_delta_dps
	 &&smartcar_imu.temperature_stable_flag==1)
	{
		gyro_sum.x+=gyro->x;
		gyro_sum.y+=gyro->y;
		gyro_sum.z+=gyro->z;
		accel_sum.x+=accel->x;
		accel_sum.y+=accel->y;
		accel_sum.z+=accel->z;
		cnt++;
	}
  else
	{
		gyro_sum.x=0;
		gyro_sum.y=0;
		gyro_sum.z=0;
		accel_sum.x=0;
		accel_sum.y=0;
		accel_sum.z=0;
		cnt=0;
	}
  last_gyro.x=gyro->x;
	last_gyro.y=gyro->y;
	last_gyro.z=gyro->z;

	if(cnt>=400)//��������2s
	{
		smartcar_imu.gyro_offset.x =(gyro_sum.x/cnt);//�õ��궨ƫ��
		smartcar_imu.gyro_offset.y =(gyro_sum.y/cnt);
		smartcar_imu.gyro_offset.z =(gyro_sum.z/cnt);
		
		smartcar_imu.accel_offset.x =(accel_sum.x/cnt);//�õ��궨ƫ��
		smartcar_imu.accel_offset.y =(accel_sum.y/cnt);
		smartcar_imu.accel_offset.z =(accel_sum.z/cnt)-safe_sqrt(1-sq2(smartcar_imu.accel_offset.x)-sq2(smartcar_imu.accel_offset.y));
			
		WriteFlashParameter_Three(GYRO_X_OFFSET,
															smartcar_imu.gyro_offset.x,
															smartcar_imu.gyro_offset.y,
															smartcar_imu.gyro_offset.z,
															&Trackless_Params);		
		WriteFlashParameter_Three(ACCEL_X_OFFSET,
															smartcar_imu.accel_offset.x,
															smartcar_imu.accel_offset.y,
															smartcar_imu.accel_offset.z,
															&Trackless_Params);	
		
		gyro_sum.x=0;
		gyro_sum.y=0;
		gyro_sum.z=0;
		accel_sum.x=0;
		accel_sum.y=0;
		accel_sum.z=0;		
		cnt=0;
			
//		beep.period=100;//100*5ms
//		beep.light_on_percent=0.25f;
//		beep.reset=1;	
//		beep.times=3;//��˸3��
		
		smartcar_imu.imu_cal_flag=1;
	}
}

