#include "stm32f1xx_hal.h"
//#include "softiic.h"
//#include "ICM42688.h"
#include "mpu6050.h"
#include "mpu6050_reg.h"
#include "math.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ���廥���˲�����
#define ALPHA 0.98   // ���ٶȼ�Ȩ��
#define BETA 0.02    // ������Ȩ��

float GyroXOffset = 0.0;
float GyroYOffset = 0.0;
float GyroZOffset = 0.0;
extern float Roll;
extern float Pitch;
extern float Yaw;
int16_t AccX,AccY,AccZ,GyroX,GyroY,GyroZ;
//extern TIM_HandleTypeDef htim2;

/**
  * ��    ����MPU6050��ʼ��
  * ��    ������
  * �� �� ֵ����
  */
//void MPU6050_Init(void)
//{
//	MyI2C_Init();									//�ȳ�ʼ���ײ��I2C
//	
//	/*MPU6050�Ĵ�����ʼ������Ҫ����MPU6050�ֲ�ļĴ����������ã��˴��������˲�����Ҫ�ļĴ���*/
//	MPU6050_Write(MPU6050_PWR_MGMT_1, 0x01);		//��Դ����Ĵ���1��ȡ��˯��ģʽ��ѡ��ʱ��ԴΪX��������
//	MPU6050_Write(MPU6050_PWR_MGMT_2, 0x00);		//��Դ����Ĵ���2������Ĭ��ֵ0���������������
//	MPU6050_Write(MPU6050_SMPLRT_DIV, 0x09);		//�����ʷ�Ƶ�Ĵ��������ò�����
//	MPU6050_Write(MPU6050_CONFIG, 0x06);			//���üĴ���������DLPF
//	MPU6050_Write(MPU6050_GYRO_CONFIG, 0x18);	//���������üĴ�����ѡ��������Ϊ��2000��/s
//	MPU6050_Write(MPU6050_ACCEL_CONFIG, 0x18);	//���ٶȼ����üĴ�����ѡ��������Ϊ��16g
//	
//	CalibrateGyro(100);  // У׼������
//}

///**
//  * ��    ����MPU6050��ȡID��
//  * ��    ������
//  * �� �� ֵ��MPU6050��ID��
//  */
//uint8_t MPU6050_GetID(void)
//{
//	return MPU6050_Read(MPU6050_WHO_AM_I);		//����WHO_AM_I�Ĵ�����ֵ
//}

///**
//  * ��    ����MPU6050��ȡ����
//  * ��    ����AccX AccY AccZ ���ٶȼ�X��Y��Z������ݣ�ʹ�������������ʽ���أ���Χ��-32768~32767
//  * ��    ����GyroX GyroY GyroZ ������X��Y��Z������ݣ�ʹ�������������ʽ���أ���Χ��-32768~32767
//  * �� �� ֵ����
//  */
//void MPU6050_GetData(int16_t *AccX,  int16_t *AccY,  int16_t *AccZ, 
//										 int16_t *GyroX, int16_t *GyroY)
//{
//	uint8_t DataH, DataL;								//�������ݸ�8λ�͵�8λ�ı���
//	
//	DataH = MPU6050_Read(MPU6050_ACCEL_XOUT_H);		//��ȡ���ٶȼ�X��ĸ�8λ����
//	DataL = MPU6050_Read(MPU6050_ACCEL_XOUT_L);		//��ȡ���ٶȼ�X��ĵ�8λ����
//	*AccX = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
//	
//	DataH = MPU6050_Read(MPU6050_ACCEL_YOUT_H);		//��ȡ���ٶȼ�Y��ĸ�8λ����
//	DataL = MPU6050_Read(MPU6050_ACCEL_YOUT_L);		//��ȡ���ٶȼ�Y��ĵ�8λ����
//	*AccY = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
//	
//	DataH = MPU6050_Read(MPU6050_ACCEL_ZOUT_H);		//��ȡ���ٶȼ�Z��ĸ�8λ����
//	DataL = MPU6050_Read(MPU6050_ACCEL_ZOUT_L);		//��ȡ���ٶȼ�Z��ĵ�8λ����
//	*AccZ = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
//	
//	DataH = MPU6050_Read(MPU6050_GYRO_XOUT_H);		//��ȡ������X��ĸ�8λ����
//	DataL = MPU6050_Read(MPU6050_GYRO_XOUT_L);		//��ȡ������X��ĵ�8λ����
//	*GyroX = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
//	
//	DataH = MPU6050_Read(MPU6050_GYRO_YOUT_H);		//��ȡ������Y��ĸ�8λ����
//	DataL = MPU6050_Read(MPU6050_GYRO_YOUT_L);		//��ȡ������Y��ĵ�8λ����
//	*GyroY = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
////	
////	DataH = MPU6050_Read(MPU6050_GYRO_ZOUT_H);		//��ȡ������Z��ĸ�8λ����
////	DataL = MPU6050_Read(MPU6050_GYRO_ZOUT_L);		//��ȡ������Z��ĵ�8λ����
////	*GyroZ = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
//}


//void MPU6050_GetYaw(int16_t *GyroZ)
//{
//	uint8_t DataH, DataL;								//�������ݸ�8λ�͵�8λ�ı���
//	
//	DataH = MPU6050_Read(MPU6050_GYRO_ZOUT_H);		//��ȡ������Z��ĸ�8λ����
//	DataL = MPU6050_Read(MPU6050_GYRO_ZOUT_L);		//��ȡ������Z��ĵ�8λ����
//	*GyroZ = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
//}
///*
//	����1��Ҫд��ļĴ���
//	����2��Ҫд�������
//*/
//void MPU6050_Write(uint8_t Regaddr, uint8_t data)
//{
//	MyI2C_Start();
//	MyI2C_SendByte(MPU6050_ADDR);
//	MyI2C_ReceiveAck();
//	MyI2C_SendByte(Regaddr);
//	MyI2C_ReceiveAck();
//	MyI2C_SendByte(data);
//	MyI2C_ReceiveAck();
//	MyI2C_Stop();
//}


///*
//	����1��Ҫ��ȡ�ļĴ���
//*/
//uint8_t MPU6050_Read(uint8_t Regaddr)
//{
//	uint8_t data;
//	
//	MyI2C_Start();
//	MyI2C_SendByte(MPU6050_ADDR);
//	MyI2C_ReceiveAck();
//	MyI2C_SendByte(Regaddr);
//	MyI2C_ReceiveAck();
//	
//	MyI2C_Start();
//	MyI2C_SendByte(MPU6050_ADDR | 0x01);
//	MyI2C_ReceiveAck();
//	data = MyI2C_ReceiveByte();
//	MyI2C_SendAck(1);
//	MyI2C_Stop();
//	return data;
//}

//double MPU6050_TEMP(void)  //MPU6050��������ȡ�¶����ݵĺ���
//{
//	uint8_t DataH, DataL;								//�������ݸ�8λ�͵�8λ�ı���
//	int16_t temp;
//	
//	DataH = MPU6050_Read(MPU6050_TEMP_OUT_H);		//��ȡ���ٶȼ�X��ĸ�8λ����
//	DataL = MPU6050_Read(MPU6050_TEMP_OUT_L);		//��ȡ���ٶȼ�X��ĵ�8λ����
//	temp = (DataH << 8) | DataL;						//����ƴ�ӣ�ͨ�������������
//	return (36.53+temp/340.0f);
//}
//int8_t bsp_GetRawData(int16_t *AccX,  int16_t *AccY,  int16_t *AccZ, 
//										 int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
//{
//	  bsp_Icm42688RegCfg();
//    uint8_t buffer[12] = {0};

//    icm42688_read_regs(ICM42688_ACCEL_DATA_X1, buffer, 12);

//    *AccX= ((uint16_t)buffer[0] << 8)  | buffer[1];
//    *AccY= ((uint16_t)buffer[2] << 8)  | buffer[3];
//    *AccZ= ((uint16_t)buffer[4] << 8)  | buffer[5];
//    *GyroX= ((uint16_t)buffer[6] << 8)  | buffer[7];
//    *GyroY= ((uint16_t)buffer[8] << 8)  | buffer[9];
//    *GyroZ=((uint16_t)buffer[10] << 8) | buffer[11];
//		return 0;
//}
//void mpu6050_Init(void)
//{
//	bsp_GetRawData(&AccX, &AccY, &AccZ, &GyroX, &GyroY, &GyroZ);									
//}

/*******MPU6050��̬����********/
// �Ƕȷ�Χ���ƺ���
void LimitAngle(float *angle)
{
    if (*angle > 180.0f)
    {
        *angle -= 360.0f;
    }
    else if (*angle < -180.0f)
    {
        *angle += 360.0f;
    }
}

//�궨
void CalibrateGyro(int samples)
{
    int32_t sumX = 0, sumY = 0, sumZ = 0;
    for (int i = 0; i < samples; i++)
    {
        int16_t GyroX, GyroY, GyroZ;
        bsp_GetRawData(NULL, NULL, NULL, &GyroX, &GyroY,&GyroZ);
        sumX += GyroX;
        sumY += GyroY;
        sumZ += GyroZ;
        HAL_Delay(10);  // �ȴ�10ms
    }
    GyroXOffset = sumX / samples;
    GyroYOffset = sumY / samples;
    GyroZOffset = sumZ / samples;
}

//�����˲�
void ComplementaryFilter(float AccX, float AccY, float AccZ, float GyroX, float GyroY, float dt)
	
{
   	GyroX -= GyroXOffset;
    GyroY -= GyroYOffset;

    // ���ٶȼ�����ת��Ϊ�Ƕ�
    Roll = atan2f((float)AccY, (float)AccZ) * 57.3;//arctan2�������ص��ǻ���ֵ��������Ҫ����ת��Ϊ�Ƕȣ�
    Pitch = atan2f(-(float)AccX, sqrtf((float)AccY * (float)AccY + (float)AccZ * (float)AccZ)) * 57.3;
		
    // �������������ݽ��н��ٶȻ��ָ���
    Roll += GyroX * dt;
    Pitch += GyroY * dt;
		//Yaw+=GyroZ* dt;
    // Ӧ�û����˲�����ʾ����Ϊ�򵥵Ļ����˲���ʵ�ʿɸ�����Ҫ������
    // �����˲�ϵ��
    float alpha = 0.98f;

    // �����˲�����
    Roll = alpha * Roll + (1.0f - alpha) * (atan2f((float)AccY, (float)AccZ) * 57.3);
    Pitch = alpha * Pitch + (1.0f - alpha) * (atan2f(-(float)AccX, sqrtf((float)AccY * (float)AccY + (float)AccZ * (float)AccZ)) * 57.3);
			
		 // �Ƕȷ�Χ���ƺ���
    LimitAngle(&Roll);
    LimitAngle(&Pitch);
}


void ComplementaryFilter_YAW(float GyroZ, float dt)
{

	//float yaw_gyro=Yaw+GyroZ*dt;
	//float alpha = 0.98f;
	
	//Yaw=alpha *yaw_gyro+(1-alpha)*Yaw;
//	Yaw=(Yaw+GyroZ/16.4*dt)-0.973*dt;
//	LimitAngle(&Yaw);
	  Yaw += GyroZ*0.03051 * dt;

    // ����ƫ������-180�㵽180��֮��
    if (Yaw > 180.0) Yaw -= 360.0;
    if (Yaw < -180.0) Yaw += 360.0;

}































