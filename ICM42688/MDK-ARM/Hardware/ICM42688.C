#include "ICM42688.h"
#if defined(ICM_USE_HARD_SPI)
#include "spi.h"
#include "math.h"
//#include "IO_SPI.h"
#elif defined(ICM_USE_HARD_I2C)
//#include "bsp_cpu_i2c2.h"
#endif
//#include "DWT.h"
#include "stdio.h"

static float accSensitivity   = 0.244f;   //加速度的最小分辨率 mg/LSB 8g量程
static float gyroSensitivity  ;    //陀螺仪的最小分辨率


/*ICM42688使用的ms级延时函数，须由用户提供。*/
#define ICM42688DelayMs(_nms)  HAL_Delay(_nms)

#if defined(ICM_USE_HARD_SPI)
#define ICM_RCC_SPIX_CS()    __HAL_RCC_GPIOB_CLK_ENABLE()
#define ICM_PORT_SPIX_CS		 GPIOA
#define ICM_PIN_SPIX_CS	     GPIO_PIN_4
#define ICM_SPI_CS_LOW()     HAL_GPIO_WritePin(ICM_PORT_SPIX_CS, ICM_PIN_SPIX_CS, GPIO_PIN_RESET)
#define ICM_SPI_CS_HIGH()    HAL_GPIO_WritePin(ICM_PORT_SPIX_CS, ICM_PIN_SPIX_CS, GPIO_PIN_SET)

#define SPIx_SCK_PIN GPIO_PIN_5
#define SPIx_SCK_GPIO_PORT GPIOA
#define SPIx_MOSI_PIN GPIO_PIN_7
#define SPIx_MOSI_GPIO_PORT GPIOA
#define SPIx_MISO_PIN GPIO_PIN_6
#define SPIx_MISO_GPIO_PORT GPIOA


#define SPI_CLK(level) 	HAL_GPIO_WritePin(SPIx_SCK_GPIO_PORT, SPIx_SCK_PIN, level?GPIO_PIN_SET:GPIO_PIN_RESET)
#define SPI_MOSI(level) HAL_GPIO_WritePin(SPIx_MOSI_GPIO_PORT, SPIx_MOSI_PIN, level?GPIO_PIN_SET:GPIO_PIN_RESET)
#define SPI_MISO() 			HAL_GPIO_ReadPin(SPIx_MISO_GPIO_PORT, SPIx_MISO_PIN)
#define CS(level) 			HAL_GPIO_WritePin(ICM_PORT_SPIX_CS,ICM_PIN_SPIX_CS, level?GPIO_PIN_SET:GPIO_PIN_RESET)


uint8_t hal_Spi1_ReadWriteByte(uint8_t txdata)
{
    uint8_t rxdata = 0;
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rxdata, 1, HAL_MAX_DELAY); //??5ms    	
    return rxdata;
}

/*******************************************************************************
* 名    称： bsp_IcmSpixCsInit
* 功    能： Icm SPI的CS控制引脚初始化
* 入口参数： 无
* 出口参数： 无
* 作　　者： Baxiange
* 创建日期： 2022-07-25
* 修    改：
* 修改日期：
* 备    注：
*******************************************************************************/
//void bsp_IcmSpixCsInit(void)
//{
//    GPIO_InitTypeDef GPIO_InitStruct = {0};

//    /* GPIO Ports Clock Enable */
//    ICM_RCC_SPIX_CS();

//    /*Configure GPIO pins */
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    GPIO_InitStruct.Pin = ICM_PIN_SPIX_CS;
//    HAL_GPIO_Init(ICM_PORT_SPIX_CS, &amp;GPIO_InitStruct);
//    HAL_GPIO_WritePin(ICM_PORT_SPIX_CS, ICM_PIN_SPIX_CS, GPIO_PIN_SET);
//}

/*******************************************************************************
* 名    称： Icm_Spi_ReadWriteNbytes
* 功    能： 使用SPI读写n个字节
* 入口参数： pBuffer: 写入的数组  len:写入数组的长度
* 出口参数： 无
* 作　　者： Baxiange
* 创建日期： 2022-07-25
* 修    改：
* 修改日期：
* 备    注：
*******************************************************************************/

 static void Icm_Spi_ReadWriteNbytes(uint8_t* pBuffer, uint8_t len)
 {
     uint8_t i = 0;
 #if defined(ICM_USE_HARD_SPI)
     for(i = 0; i < len; i ++)
     {
			  *pBuffer = hal_Spi1_ReadWriteByte(*pBuffer);
         pBuffer++;
     }
 #endif

 }




#endif

/*******************************************************************************
* 名    称： icm42688_read_reg
* 功    能： 读取单个寄存器的值
* 入口参数： reg: 寄存器地址 
* 出口参数： 当前寄存器地址的值
* 作　　者： Baxiange
* 创建日期： 2022-07-25
* 修    改：
* 修改日期：
* 备    注： 使用SPI读取寄存器时要注意:最高位为读写位，详见datasheet page51.
*******************************************************************************/
uint8_t icm42688_read_reg(uint8_t reg)
{
    uint8_t regval = 0;

#if defined(ICM_USE_HARD_SPI)
    ICM_SPI_CS_LOW();
//		printf("ICM_SPI_CS_LOW\n");
    reg |= 0x80;
    /* 写入要读的寄存器地址 */
    Icm_Spi_ReadWriteNbytes(&reg,1);
    /* 读取寄存器数据 */
    Icm_Spi_ReadWriteNbytes(&regval,1);
    ICM_SPI_CS_HIGH();
//		printf("ICM_SPI_CS_HIGH\n");
#elif defined(ICM_USE_HARD_I2C)

#endif

    return regval;
}

/*******************************************************************************
* 名    称： icm42688_read_regs
* 功    能： 连续读取多个寄存器的值
* 入口参数： reg: 起始寄存器地址 *buf数据指针,uint16_t len长度
* 出口参数： 无
* 作　　者： Baxiange
* 创建日期： 2022-07-25
* 修    改：
* 修改日期：
* 备    注： 使用SPI读取寄存器时要注意:最高位为读写位，详见datasheet page50.
*******************************************************************************/
static void icm42688_read_regs(uint8_t reg, uint8_t* buf, uint16_t len)
{
#if defined(ICM_USE_HARD_SPI)
    reg |= 0x80;
    ICM_SPI_CS_LOW();
    /* 写入要读的寄存器地址 */
    Icm_Spi_ReadWriteNbytes(&reg, 1);
    /* 读取寄存器数据 */
    Icm_Spi_ReadWriteNbytes(buf, len);
    ICM_SPI_CS_HIGH();
#elif defined(ICM_USE_HARD_I2C)
#endif
}


/*******************************************************************************
* 名    称： icm42688_write_reg
* 功    能： 向单个寄存器写数据
* 入口参数： reg: 寄存器地址 value:数据
* 出口参数： 0
* 作　　者： Baxiange
* 创建日期： 2022-07-25
* 修    改：
* 修改日期：
* 备    注： 使用SPI读取寄存器时要注意:最高位为读写位，详见datasheet page50.
*******************************************************************************/
static uint8_t icm42688_write_reg(uint8_t reg, uint8_t value)
{
#if defined(ICM_USE_HARD_SPI)
    ICM_SPI_CS_LOW();
    /* 写入要读的寄存器地址 */
    Icm_Spi_ReadWriteNbytes(&reg, 1);
    /* 读取寄存器数据 */
    Icm_Spi_ReadWriteNbytes(&value, 1);
    ICM_SPI_CS_HIGH();
#elif defined(ICM_USE_HARD_I2C)
#endif
    return 0;
}



float bsp_Icm42688GetAres(uint8_t Ascale)
{
    switch(Ascale)
    {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (11), 4 Gs (10), 8 Gs (01), and 16 Gs  (00).
    case AFS_2G:
        accSensitivity = 2000 / 32768.0f;
        break;
    case AFS_4G:
        accSensitivity = 4000 / 32768.0f;
        break;
    case AFS_8G:
        accSensitivity = 8000 / 32768.0f;
        break;
    case AFS_16G:
        accSensitivity = 16000 / 32768.0f;
        break;
    }

    return accSensitivity;
}

float bsp_Icm42688GetGres(uint8_t Gscale)
{
    switch(Gscale)
    {
    case GFS_15_125DPS:
        gyroSensitivity = 15.125f / 32768.0f;
        break;
    case GFS_31_25DPS:
        gyroSensitivity = 31.25f / 32768.0f;
        break;
    case GFS_62_5DPS:
        gyroSensitivity = 62.5f / 32768.0f;
        break;
    case GFS_125DPS:
        gyroSensitivity = 125.0f / 32768.0f;
        break;
    case GFS_250DPS:
        gyroSensitivity = 250.0f / 32768.0f;
        break;
    case GFS_500DPS:
        gyroSensitivity = 500.0f / 32768.0f;
        break;
    case GFS_1000DPS:
        gyroSensitivity = 1000.0f / 32768.0f;
        break;
    case GFS_2000DPS:
        gyroSensitivity = 2000.0f / 32768.0f;
        break;
    }
    return gyroSensitivity;
}

/*******************************************************************************
* 名    称： bsp_Icm42688RegCfg
* 功    能： Icm42688 寄存器配置
* 入口参数： 无
* 出口参数： 无
* 作　　者： Baxiange
* 创建日期： 2022-07-25
* 修    改：
* 修改日期：
* 备    注：
*******************************************************************************/
int8_t bsp_Icm42688RegCfg(void)
{
    uint8_t reg_val = 0;
    /* 读取 who am i 寄存器 */
    reg_val = icm42688_read_reg(ICM42688_WHO_AM_I);
		printf("reg_val:%d\n",reg_val);
    icm42688_write_reg(ICM42688_REG_BANK_SEL, 0); //设置bank 0区域寄存器
    icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x01); //软复位传感器
    ICM42688DelayMs(100);


    if(reg_val == ICM42688_ID)
    {
        icm42688_write_reg(ICM42688_REG_BANK_SEL, 1); //设置bank 1区域寄存器
        icm42688_write_reg(ICM42688_INTF_CONFIG4, 0x02); //设置为4线SPI通信

        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0); //设置bank 0区域寄存器
        icm42688_write_reg(ICM42688_FIFO_CONFIG, 0x40); //Stream-to-FIFO Mode(page63)


        reg_val = icm42688_read_reg(ICM42688_INT_SOURCE0);
        icm42688_write_reg(ICM42688_INT_SOURCE0, 0x00);
        icm42688_write_reg(ICM42688_FIFO_CONFIG2, 0x00); // watermark
        icm42688_write_reg(ICM42688_FIFO_CONFIG3, 0x02); // watermark
        icm42688_write_reg(ICM42688_INT_SOURCE0, reg_val);
        icm42688_write_reg(ICM42688_FIFO_CONFIG1, 0x63); // Enable the accel and gyro to the FIFO

        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
        icm42688_write_reg(ICM42688_INT_CONFIG, 0x36);

        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
        reg_val = icm42688_read_reg(ICM42688_INT_SOURCE0);
        reg_val |= (1 << 2); //FIFO_THS_INT1_ENABLE
        icm42688_write_reg(ICM42688_INT_SOURCE0, reg_val);

        bsp_Icm42688GetAres(AFS_8G);
        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
        reg_val = icm42688_read_reg(ICM42688_ACCEL_CONFIG0);//page74
        reg_val |= (AFS_8G << 5);   //量程 ±8g
        reg_val |= (AODR_200Hz);     //输出速率 50HZ
        icm42688_write_reg(ICM42688_ACCEL_CONFIG0, reg_val);

        bsp_Icm42688GetGres(GFS_1000DPS);
        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
        reg_val = icm42688_read_reg(ICM42688_GYRO_CONFIG0);//page73
        reg_val |= (GFS_1000DPS << 5);   //量程 ±1000dps
        reg_val |= (GODR_200Hz);     //输出速率 50HZ
        icm42688_write_reg(ICM42688_GYRO_CONFIG0, reg_val);

        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
        reg_val = icm42688_read_reg(ICM42688_PWR_MGMT0); //读取PWR—MGMT0当前寄存器的值(page72)
        reg_val &= ~(1 << 5);//使能温度测量
        reg_val |= ((3) << 2);//设置GYRO_MODE  0:关闭 1:待机 2:预留 3:低噪声
        reg_val |= (3);//设置ACCEL_MODE 0:关闭 1:关闭 2:低功耗 3:低噪声
        icm42688_write_reg(ICM42688_PWR_MGMT0, reg_val);
        ICM42688DelayMs(1); //操作完PWR—MGMT0寄存器后 200us内不能有任何读写寄存器的操作

        return 0;
    }
    return -1;
}
/*******************************************************************************
* 名    称： bsp_Icm42688Init
* 功    能： Icm42688 传感器初始化
* 入口参数： 无
* 出口参数： 0: 初始化成功  其他值: 初始化失败
* 作　　者： Baxiange
* 创建日期： 2022-07-25
* 修    改：
* 修改日期：
* 备    注：
*******************************************************************************/
int8_t bsp_Icm42688Init(void)
{
//    bsp_IcmSpixCsInit();
		 //SPI_Init();

    return(bsp_Icm42688RegCfg());

}

/*******************************************************************************
* 名    称： bsp_IcmGetTemperature
* 功    能： 读取Icm42688 内部传感器温度
* 入口参数： 无
* 出口参数： 无
* 作　　者： Baxiange
* 创建日期： 2022-07-25
* 修    改：
* 修改日期：
* 备    注： datasheet page62
*******************************************************************************/
int8_t bsp_IcmGetTemperature(int16_t* pTemp)
{
    uint8_t buffer[2] = {0};

    icm42688_read_regs(ICM42688_TEMP_DATA1, buffer, 2);

    *pTemp = (int16_t)(((int16_t)((buffer[0] << 8) | buffer[1])) / 132.48 + 25);
    return 0;
}

/*******************************************************************************
* 名    称： bsp_IcmGetAccelerometer
* 功    能： 读取Icm42688 加速度的值
* 入口参数： 三轴加速度的值
* 出口参数： 无
* 作　　者： Baxiange
* 创建日期： 2022-07-25
* 修    改：
* 修改日期：
* 备    注： datasheet page62
*******************************************************************************/
int8_t bsp_IcmGetAccelerometer(icm42688RawData_t* accData)
{
    uint8_t buffer[6] = {0};

    icm42688_read_regs(ICM42688_ACCEL_DATA_X1, buffer, 6);

    accData->x = ((uint16_t)buffer[0] << 8) | buffer[1];
    accData->y = ((uint16_t)buffer[2] << 8) | buffer[3];
    accData->z = ((uint16_t)buffer[4] << 8) | buffer[5];

    accData->x = (int16_t)(accData->x * accSensitivity);//将原始的16位数值转换为实际的加速度值
    accData->y = (int16_t)(accData->y * accSensitivity);
    accData->z = (int16_t)(accData->z * accSensitivity);

    return 0;
}
void icm42688_GetDataAcc(int16_t *AccX,int16_t *AccY,int16_t *AccZ)
{
	uint8_t buffer[6] = {0};
	icm42688_read_regs(ICM42688_ACCEL_DATA_X1, buffer, 6);
	*AccX=(buffer[0]<<8) | buffer[1];
	*AccY=(buffer[2]<<8) | buffer[3];
	*AccZ=(buffer[4]<<8) | buffer[5];
	
	*AccX=*AccX*accSensitivity;
	*AccY=*AccY*accSensitivity;	
	*AccZ=*AccZ*accSensitivity;	
}

/*******************************************************************************
* 名    称： bsp_IcmGetGyroscope
* 功    能： 读取Icm42688 陀螺仪的值
* 入口参数： 三轴陀螺仪的值
* 出口参数： 无
* 作　　者： Baxiange
* 创建日期： 2022-07-25
* 修    改：
* 修改日期：
* 备    注： datasheet page63
*******************************************************************************/
int8_t bsp_IcmGetGyroscope(icm42688RawData_t* GyroData)
{
    uint8_t buffer[6] = {0};

    icm42688_read_regs(ICM42688_GYRO_DATA_X1, buffer, 6);

    GyroData->x = ((uint16_t)buffer[0] << 8) | buffer[1];
    GyroData->y = ((uint16_t)buffer[2] << 8) | buffer[3];
    GyroData->z = ((uint16_t)buffer[4] << 8) | buffer[5];

    GyroData->x = (int16_t)(GyroData->x * gyroSensitivity);
    GyroData->y = (int16_t)(GyroData->y * gyroSensitivity);
    GyroData->z = (int16_t)(GyroData->z * gyroSensitivity);
    return 0;
}
void icm42688_GetDataGyro(int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t buffer[2] = {0};
	icm42688_read_regs(ICM42688_ACCEL_DATA_X1, buffer, 2);
	*GyroX=(buffer[0]<<8) | buffer[1];
	*GyroX=*GyroX*gyroSensitivity;
  buffer[0] =0;buffer[1] =0;
	icm42688_read_regs(ICM42688_ACCEL_DATA_Y1, buffer, 2);
	*GyroY=(buffer[0]<<8) | buffer[1];
	*GyroY=*GyroY*gyroSensitivity;
  buffer[0] =0;buffer[1] =0;
	icm42688_read_regs(ICM42688_ACCEL_DATA_Z1, buffer, 2);
	*GyroZ=(buffer[0]<<8) | buffer[1];
	*GyroZ=*GyroZ*gyroSensitivity;	
}


/*******************************************************************************
* 名    称： bsp_IcmGetRawData
* 功    能： 读取Icm42688加速度陀螺仪数据
* 入口参数： 六轴
* 出口参数： 无
* 作　　者： Baxiange
* 创建日期： 2022-07-25
* 修    改：
* 修改日期：
* 备    注： datasheet page62,63
*******************************************************************************/
int8_t bsp_IcmGetRawData(icm42688RawData_t* accData, icm42688RawData_t* GyroData)
{
    uint8_t buffer[12] = {0};

    icm42688_read_regs(ICM42688_ACCEL_DATA_X1, buffer, 12);

    accData->x  = ((uint16_t)buffer[0] << 8)  | buffer[1];
    accData->y  = ((uint16_t)buffer[2] << 8)  | buffer[3];
    accData->z  = ((uint16_t)buffer[4] << 8)  | buffer[5];
    GyroData->x = ((uint16_t)buffer[6] << 8)  | buffer[7];
    GyroData->y = ((uint16_t)buffer[8] << 8)  | buffer[9];
    GyroData->z = ((uint16_t)buffer[10] << 8) | buffer[11];


    accData->x = (int16_t)(accData->x * accSensitivity);
    accData->y = (int16_t)(accData->y * accSensitivity);
    accData->z = (int16_t)(accData->z * accSensitivity);

    GyroData->x = (int16_t)(GyroData->x * gyroSensitivity);
    GyroData->y = (int16_t)(GyroData->y * gyroSensitivity);
    GyroData->z = (int16_t)(GyroData->z * gyroSensitivity);

    return 0;
}
void bsp_GetRawData(int16_t *AccX,  int16_t *AccY,  int16_t *AccZ, 
										 int16_t *GyroX, int16_t *GyroY,int16_t *GyroZ)

{
//	  bsp_Icm42688RegCfg();
    uint8_t buffer[12] = {0};

    icm42688_read_regs(ICM42688_ACCEL_DATA_X1, buffer, 12);

    *AccX= ((int16_t)buffer[0] << 8)  | buffer[1];
    *AccY= ((int16_t)buffer[2] << 8)  | buffer[3];
    *AccZ= ((int16_t)buffer[4] << 8)  | buffer[5];
    *GyroX= ((int16_t)buffer[6] << 8)  | buffer[7];
    *GyroY= ((int16_t)buffer[8] << 8)  | buffer[9];
    *GyroZ=((int16_t)buffer[10] << 8) | buffer[11];
		
//		*AccX=(int16_t)(*AccX*accSensitivity);
//		*AccY=(int16_t)(*AccY*accSensitivity);	
//		*AccZ=(int16_t)(*AccZ*accSensitivity);	
//		
//	  *GyroX=(int16_t)(*GyroX*gyroSensitivity);
//    *GyroY=(int16_t)(*GyroY*gyroSensitivity);
//    *GyroZ=(int16_t)(*GyroZ*gyroSensitivity);
		
		
}
#define dt 0.01
#define WINDOW_SIZE 10


//static float filtered_gyro_z = 0.0f;     // 滤波后的 Z 轴角速度
//static const float alpha = 0.3f;// 低通滤波系数（0.1~0.3
static float gyro_z_dispose[10];
static uint8_t write_index = 0;
static float yaw_total;
static float add = 0.0f;	
static uint8_t flag;

/**************************************************************************解算yaw**********************************************************************************/

float get_gyro_z(void)
{
	int16_t gyro_z_l;
	float gyro_angle;
	uint8_t buffer[2];
  icm42688_read_regs(ICM42688_GYRO_DATA_Z1, buffer, 2);
	gyro_z_l= (buffer[0]<<8) | buffer[1];
	
	gyro_angle=gyro_z_l*gyroSensitivity;
	
//  filtered_gyro_z = alpha * gyro_z_l + (1 - alpha) * filtered_gyro_z;
	

	return gyro_angle;

}

void get_yaw(YawProcessor_t *yaw_solation ,float fil_gyro_z)
{
//	uint16_t gyro_z_l;
//	uint8_t buffer[2];
//  icm42688_read_regs(ICM42688_GYRO_DATA_Z1, buffer, 2);
//	gyro_z_l= (buffer[0]<<8) | buffer[1];
//	gyro_z_l=gyro_z_l*gyroSensitivity;
//	
//  filtered_gyro_z = alpha * gyro_z_l + (1 - alpha) * filtered_gyro_z;
	
		

	if(!flag)
	{
		gyro_z_dispose[write_index]=fil_gyro_z;
		write_index++;
		yaw_total+=fil_gyro_z;
		
		if(write_index>9)
		{
			write_index=0;
			flag=1;
		}
	}
	else
	{
		yaw_total-=gyro_z_dispose[write_index];
		yaw_total+=fil_gyro_z;
		gyro_z_dispose[write_index]=fil_gyro_z;
		write_index = (write_index + 1) % WINDOW_SIZE;
	}

  yaw_solation->gyro = flag? (yaw_total / WINDOW_SIZE) : (yaw_total / write_index);
	
	
	

	yaw_solation->yaw_integral+=yaw_solation->gyro*dt;
	
	yaw_solation->err=yaw_solation->yaw_integral - yaw_solation->yaw_integral_last;



	yaw_solation->yaw_integral_last=yaw_solation->yaw_integral;
	
	if(fabs(yaw_solation->err)<0.008)
	{
		add+=yaw_solation->err;
	}
	yaw_solation->yaw=yaw_solation->yaw_integral-add;
		
	if(yaw_solation->yaw>180)yaw_solation->yaw-=360;
	if(yaw_solation->yaw<-180)yaw_solation->yaw+=360;



}







