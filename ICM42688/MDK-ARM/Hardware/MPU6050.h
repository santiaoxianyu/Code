#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>
#include "ICM42688.h"
#define ALPHA 0.98   // 加速度计权重
#define BETA 0.02    // 陀螺仪权重

#define M_PI 3.14159265358979323846  // Define M_PI if not defined

extern float Roll;
extern float Pitch;
extern float Yaw;


//void MPU6050_Write(uint8_t Regaddr, uint8_t data);
//uint8_t MPU6050_Read(uint8_t Regaddr);
//void MPU6050_Init(void);
//void MPU6050_GetData(int16_t *AccX,  int16_t *AccY,  int16_t *AccZ, 
//										 int16_t *GyroX, int16_t *GyroY);
void MPU6050_GetYaw(int16_t *GyroZ);
uint8_t MPU6050_GetID(void);
double MPU6050_TEMP(void);

void ComplementaryFilter_YAW(float GyroZ, float dt);
void ComplementaryFilter(float AccX, float AccY, float AccZ, float GyroX, float GyroY, float dt);
void CalibrateGyro(int samples);
void LimitAngle(float *angle);
	
#endif
