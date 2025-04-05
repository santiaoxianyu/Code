#include "stm32f1xx_hal.h"
//#include "softiic.h"
//#include "ICM42688.h"
#include "mpu6050.h"
#include "mpu6050_reg.h"
#include "math.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 定义互补滤波参数
#define ALPHA 0.98   // 加速度计权重
#define BETA 0.02    // 陀螺仪权重

float GyroXOffset = 0.0;
float GyroYOffset = 0.0;
float GyroZOffset = 0.0;
extern float Roll;
extern float Pitch;
extern float Yaw;
int16_t AccX,AccY,AccZ,GyroX,GyroY,GyroZ;
//extern TIM_HandleTypeDef htim2;

/**
  * 函    数：MPU6050初始化
  * 参    数：无
  * 返 回 值：无
  */
//void MPU6050_Init(void)
//{
//	MyI2C_Init();									//先初始化底层的I2C
//	
//	/*MPU6050寄存器初始化，需要对照MPU6050手册的寄存器描述配置，此处仅配置了部分重要的寄存器*/
//	MPU6050_Write(MPU6050_PWR_MGMT_1, 0x01);		//电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
//	MPU6050_Write(MPU6050_PWR_MGMT_2, 0x00);		//电源管理寄存器2，保持默认值0，所有轴均不待机
//	MPU6050_Write(MPU6050_SMPLRT_DIV, 0x09);		//采样率分频寄存器，配置采样率
//	MPU6050_Write(MPU6050_CONFIG, 0x06);			//配置寄存器，配置DLPF
//	MPU6050_Write(MPU6050_GYRO_CONFIG, 0x18);	//陀螺仪配置寄存器，选择满量程为±2000°/s
//	MPU6050_Write(MPU6050_ACCEL_CONFIG, 0x18);	//加速度计配置寄存器，选择满量程为±16g
//	
//	CalibrateGyro(100);  // 校准陀螺仪
//}

///**
//  * 函    数：MPU6050获取ID号
//  * 参    数：无
//  * 返 回 值：MPU6050的ID号
//  */
//uint8_t MPU6050_GetID(void)
//{
//	return MPU6050_Read(MPU6050_WHO_AM_I);		//返回WHO_AM_I寄存器的值
//}

///**
//  * 函    数：MPU6050获取数据
//  * 参    数：AccX AccY AccZ 加速度计X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
//  * 参    数：GyroX GyroY GyroZ 陀螺仪X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
//  * 返 回 值：无
//  */
//void MPU6050_GetData(int16_t *AccX,  int16_t *AccY,  int16_t *AccZ, 
//										 int16_t *GyroX, int16_t *GyroY)
//{
//	uint8_t DataH, DataL;								//定义数据高8位和低8位的变量
//	
//	DataH = MPU6050_Read(MPU6050_ACCEL_XOUT_H);		//读取加速度计X轴的高8位数据
//	DataL = MPU6050_Read(MPU6050_ACCEL_XOUT_L);		//读取加速度计X轴的低8位数据
//	*AccX = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
//	
//	DataH = MPU6050_Read(MPU6050_ACCEL_YOUT_H);		//读取加速度计Y轴的高8位数据
//	DataL = MPU6050_Read(MPU6050_ACCEL_YOUT_L);		//读取加速度计Y轴的低8位数据
//	*AccY = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
//	
//	DataH = MPU6050_Read(MPU6050_ACCEL_ZOUT_H);		//读取加速度计Z轴的高8位数据
//	DataL = MPU6050_Read(MPU6050_ACCEL_ZOUT_L);		//读取加速度计Z轴的低8位数据
//	*AccZ = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
//	
//	DataH = MPU6050_Read(MPU6050_GYRO_XOUT_H);		//读取陀螺仪X轴的高8位数据
//	DataL = MPU6050_Read(MPU6050_GYRO_XOUT_L);		//读取陀螺仪X轴的低8位数据
//	*GyroX = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
//	
//	DataH = MPU6050_Read(MPU6050_GYRO_YOUT_H);		//读取陀螺仪Y轴的高8位数据
//	DataL = MPU6050_Read(MPU6050_GYRO_YOUT_L);		//读取陀螺仪Y轴的低8位数据
//	*GyroY = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
////	
////	DataH = MPU6050_Read(MPU6050_GYRO_ZOUT_H);		//读取陀螺仪Z轴的高8位数据
////	DataL = MPU6050_Read(MPU6050_GYRO_ZOUT_L);		//读取陀螺仪Z轴的低8位数据
////	*GyroZ = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
//}


//void MPU6050_GetYaw(int16_t *GyroZ)
//{
//	uint8_t DataH, DataL;								//定义数据高8位和低8位的变量
//	
//	DataH = MPU6050_Read(MPU6050_GYRO_ZOUT_H);		//读取陀螺仪Z轴的高8位数据
//	DataL = MPU6050_Read(MPU6050_GYRO_ZOUT_L);		//读取陀螺仪Z轴的低8位数据
//	*GyroZ = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
//}
///*
//	参数1：要写入的寄存器
//	参数2：要写入的数据
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
//	参数1：要读取的寄存器
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

//double MPU6050_TEMP(void)  //MPU6050传感器读取温度数据的函数
//{
//	uint8_t DataH, DataL;								//定义数据高8位和低8位的变量
//	int16_t temp;
//	
//	DataH = MPU6050_Read(MPU6050_TEMP_OUT_H);		//读取加速度计X轴的高8位数据
//	DataL = MPU6050_Read(MPU6050_TEMP_OUT_L);		//读取加速度计X轴的低8位数据
//	temp = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
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

/*******MPU6050姿态解算********/
// 角度范围限制函数
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

//标定
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
        HAL_Delay(10);  // 等待10ms
    }
    GyroXOffset = sumX / samples;
    GyroYOffset = sumY / samples;
    GyroZOffset = sumZ / samples;
}

//互补滤波
void ComplementaryFilter(float AccX, float AccY, float AccZ, float GyroX, float GyroY, float dt)
	
{
   	GyroX -= GyroXOffset;
    GyroY -= GyroYOffset;

    // 加速度计数据转换为角度
    Roll = atan2f((float)AccY, (float)AccZ) * 57.3;//arctan2函数返回的是弧度值，我们需要将其转换为角度：
    Pitch = atan2f(-(float)AccX, sqrtf((float)AccY * (float)AccY + (float)AccZ * (float)AccZ)) * 57.3;
		
    // 根据陀螺仪数据进行角速度积分更新
    Roll += GyroX * dt;
    Pitch += GyroY * dt;
		//Yaw+=GyroZ* dt;
    // 应用互补滤波器（示例中为简单的互补滤波，实际可根据需要调整）
    // 互补滤波系数
    float alpha = 0.98f;

    // 互补滤波计算
    Roll = alpha * Roll + (1.0f - alpha) * (atan2f((float)AccY, (float)AccZ) * 57.3);
    Pitch = alpha * Pitch + (1.0f - alpha) * (atan2f(-(float)AccX, sqrtf((float)AccY * (float)AccY + (float)AccZ * (float)AccZ)) * 57.3);
			
		 // 角度范围限制函数
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

    // 限制偏航角在-180°到180°之间
    if (Yaw > 180.0) Yaw -= 360.0;
    if (Yaw < -180.0) Yaw += 360.0;

}































