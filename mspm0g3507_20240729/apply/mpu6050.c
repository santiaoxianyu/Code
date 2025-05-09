#include "mpu6050.h"
  

//MPU IIC 延时函数
static void MPU_delay(void)
{
    delay_us(2);
}

//产生IIC起始信号
static void MPU_IIC_Start(void)
{
	MPU_IIC_SDA(1);
	MPU_IIC_SCL(1);
    MPU_delay();
	MPU_IIC_SDA(0);
    MPU_delay();
	MPU_IIC_SCL(0);
    MPU_delay();
}      
//产生IIC停止信号
void MPU_IIC_Stop(void)
{
	MPU_IIC_SDA(0);
    MPU_delay();
	MPU_IIC_SCL(1);
    MPU_delay();
	MPU_IIC_SDA(1);
    MPU_delay();                               
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t MPU_IIC_Wait_Ack(void)
{
	uint8_t AckBit;
	MPU_IIC_SDA(1);
	MPU_IIC_SCL(1);
	AckBit = MPU_READ_SDA();
	MPU_IIC_SCL(0);
	return AckBit;
} 
//ACK应答
void MPU_IIC_Ack(uint8_t ack)
{
	MPU_IIC_SDA(ack);
	MPU_IIC_SCL(1);
	MPU_IIC_SCL(0);
}
                                
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答              
void MPU_IIC_Send_Byte(uint8_t txd)
{                        
	uint8_t i;
	for (i = 0; i < 8; i ++)
	{
		MPU_IIC_SDA(txd & (0x80 >> i));
		MPU_IIC_SCL(1);
		MPU_IIC_SCL(0);
	}
}         
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t MPU_IIC_Read_Byte(unsigned char ack)
{
	uint8_t i, Byte = 0x00;
    MPU_SDA_IN();
	MPU_IIC_SDA(1);
	for (i = 0; i < 8; i ++)
	{
		MPU_IIC_SCL(1);
		if (MPU_READ_SDA() == 1){Byte |= (0x80 >> i);}
		MPU_IIC_SCL(0);
	}
	return Byte;
}



//初始化IIC
void MPU_IIC_Init(void)
{            
//    RCC->APB2ENR |= 1 << 3;  /* SCL引脚时钟使能 */
//    RCC->APB2ENR |= 1 << 3;  /* SDA引脚时钟使能 */

//    /* SCL引脚模式设置,推挽输出,上拉 */
//    sys_gpio_set(GPIOB, SYS_GPIO_PIN8,
//                 SYS_GPIO_MODE_OUT, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_MID, SYS_GPIO_PUPD_PU);

//    /* SDA引脚模式设置,开漏输出,上拉, 这样就不用再设置IO方向了, 开漏输出的时候(=1), 也可以读取外部信号的高低电平 */
//    sys_gpio_set(GPIOB, SYS_GPIO_PIN9,
//                 SYS_GPIO_MODE_OUT, SYS_GPIO_OTYPE_OD, SYS_GPIO_SPEED_MID, SYS_GPIO_PUPD_PU);
}

//初始化MPU6050
//返回值:0,成功
//其他,错误代码
uint8_t MPU_Init(void)
{ 
    uint8_t res; 
    MPU_IIC_Init();//初始化IIC总线
    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);    //复位MPU6050
    delay_ms(100);
    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);    //唤醒MPU6050 
    MPU_Set_Gyro_Fsr(3);                    //陀螺仪传感器,±2000dps
    MPU_Set_Accel_Fsr(0);                    //加速度传感器,±2g
    MPU_Set_Rate(200);                        //设置采样率50Hz
    MPU_Write_Byte(MPU_INT_EN_REG,0X00);    //关闭所有中断
    MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);    //I2C主模式关闭
    MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);    //关闭FIFO
    MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);    //INT引脚低电平有效
    res=MPU_Read_Byte(MPU_DEVICE_ID_REG); 
    if(res==MPU_ADDR)//器件ID正确
    {
        MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);    //设置CLKSEL,PLL X轴为参考
        MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);    //加速度与陀螺仪都工作
        MPU_Set_Rate(100);                        //设置采样率为50Hz
     }else return 1;
    return 0;
}
//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_LPF(uint16_t lpf)
{
    uint8_t data=0;
    if(lpf>=188)data=1;
    else if(lpf>=98)data=2;
    else if(lpf>=42)data=3;
    else if(lpf>=20)data=4;
    else if(lpf>=10)data=5;
    else data=6; 
    return MPU_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器  
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Rate(uint16_t rate)
{
    uint8_t data;
    if(rate>1000)rate=1000;
    if(rate<4)rate=4;
    data=1000/rate-1;
    data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);    //设置数字低通滤波器
     return MPU_Set_LPF(rate/2);    //自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
short MPU_Get_Temperature(void)
{
    uint8_t buf[2]; 
    short raw;
    float temp;
    MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((uint16_t)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    uint8_t buf[6],res;  
    res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
    if(res==0)
    {
        *gx=((uint16_t)buf[0]<<8)|buf[1];  
        *gy=((uint16_t)buf[2]<<8)|buf[3];  
        *gz=((uint16_t)buf[4]<<8)|buf[5];
    }     
    return res;;
}
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    uint8_t buf[6],res;  
    res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
    if(res==0)
    {
        *ax=((uint16_t)buf[0]<<8)|buf[1];  
        *ay=((uint16_t)buf[2]<<8)|buf[3];  
        *az=((uint16_t)buf[4]<<8)|buf[5];
    }     
    return res;;
}
//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    uint8_t i; 
    MPU_IIC_Start(); 
    MPU_IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令    
    if(MPU_IIC_Wait_Ack())    //等待应答
    {
        MPU_IIC_Stop();         
        return 1;        
    }
    MPU_IIC_Send_Byte(reg);    //写寄存器地址
    MPU_IIC_Wait_Ack();        //等待应答
    for(i=0;i<len;i++)
    {
        MPU_IIC_Send_Byte(buf[i]);    //发送数据
        if(MPU_IIC_Wait_Ack())        //等待ACK
        {
            MPU_IIC_Stop();     
            return 1;         
        }        
    }    
    MPU_IIC_Stop();     
    return 0;    
} 
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
     MPU_IIC_Start(); 
    MPU_IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令    
    if(MPU_IIC_Wait_Ack())    //等待应答
    {
        MPU_IIC_Stop();         
        return 1;        
    }
    MPU_IIC_Send_Byte(reg);    //写寄存器地址
    MPU_IIC_Wait_Ack();        //等待应答
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令    
    MPU_IIC_Wait_Ack();        //等待应答 
    while(len)
    {
        if(len==1)*buf=MPU_IIC_Read_Byte(0);//读数据,发送nACK 
        else *buf=MPU_IIC_Read_Byte(1);        //读数据,发送ACK  
        len--;
        buf++; 
    }    
    MPU_IIC_Stop();    //产生一个停止条件 
    return 0;    
}
//IIC写一个字节 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data)                  
{ 
    MPU_IIC_Start(); 
    MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令    
    if(MPU_IIC_Wait_Ack())    //等待应答
    {
        MPU_IIC_Stop();         
        return 1;        
    }
    MPU_IIC_Send_Byte(reg);    //写寄存器地址
    MPU_IIC_Wait_Ack();        //等待应答 
    MPU_IIC_Send_Byte(data);//发送数据
    if(MPU_IIC_Wait_Ack())    //等待ACK
    {
        MPU_IIC_Stop();     
        return 1;         
    }         
    MPU_IIC_Stop();     
    return 0;
}
//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
uint8_t MPU_Read_Byte(uint8_t reg)
{
    uint8_t res;
    MPU_IIC_Start(); 
    MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令    
    MPU_IIC_Wait_Ack();        //等待应答 
    MPU_IIC_Send_Byte(reg);    //写寄存器地址
    MPU_IIC_Wait_Ack();        //等待应答
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU_ADDR<<1)|1);//发送器件地址+读命令    
    MPU_IIC_Wait_Ack();        //等待应答 
    res=MPU_IIC_Read_Byte(0);//读取数据,发送nACK 
    MPU_IIC_Stop();            //产生一个停止条件 
    return res;        
}


