//#include "ICM42688.h"
//#include "stdio.h"


//static float accSensitivity   = 0.244f;   //加速度的最小分辨率 mg/LSB
//static float gyroSensitivity  = 0.03052;    //陀螺仪的最小分辨率


//#define ICM_PORT_SPIX_CS		 GPIOC
//#define ICM_PIN_SPIX_CS	     GPIO_PIN_4
//#define ICM_SPI_CS_LOW()     HAL_GPIO_WritePin(ICM_PORT_SPIX_CS, ICM_PIN_SPIX_CS, GPIO_PIN_RESET)
//#define ICM_SPI_CS_HIGH()    HAL_GPIO_WritePin(ICM_PORT_SPIX_CS, ICM_PIN_SPIX_CS, GPIO_PIN_SET)




///*******************************************************************************
//* 名    称： icm42688_read_regs
//* 功    能： 连续读取多个寄存器的值
//* 入口参数： 
//            *dev:           设备接口
//            reg_address:    起始寄存器地址 
//            *data:          数据指针
//            len:            读取寄存器的个数
//* 出口参数： 无
//* 作　　者： DP
//* 创建日期： 2025-01-07
//* 修    改：
//* 修改日期：
//* 备    注： 使用SPI读取寄存器时要注意:最高位为读写位，详见datasheet page50.
//*******************************************************************************/
//void icm42688_read_regs(const bsp_ICM42688_t *dev,uint8_t reg_address,uint8_t *data,uint16_t len)
//{
//    //读写位设置为1
//    reg_address |= 0x80;
//    ICM_SPI_CS_LOW();
//    dev->read_reg(reg_address,data,len);
//    ICM_SPI_CS_HIGH();

//}


///*******************************************************************************
//* 名    称： icm42688_write_regs
//* 功    能： 连续写寄存器
//* 入口参数： 
//            *dev:           设备接口
//            reg_address:    起始寄存器地址 
//            data:           数据
//            len:            写入寄存器的个数
//* 出口参数： 0
//* 作　　者： DP
//* 创建日期： 2025-01-07
//* 修    改：
//* 修改日期：
//* 备    注： 使用SPI读取寄存器时要注意:最高位为读写位，详见datasheet page50.
//*******************************************************************************/
//static void icm42688_write_regs(const bsp_ICM42688_t *dev,uint8_t reg_address, uint8_t data,uint16_t len)
//{

//    ICM_SPI_CS_LOW();
//    dev->write_reg(reg_address,data,len);
//    ICM_SPI_CS_HIGH();

//}

///*******************************************************************************
//* 名    称： getGyroAafConfig
//* 功    能： 获取对应AFF配置
//* 入口参数： config：配置数率
//* 出口参数： 无
//* 作　　者： DP
//* 创建日期： 2025-01-07
//* 修    改：
//* 修改日期：
//* 备    注： 
//*******************************************************************************/
//static aafConfig_t getGyroAafConfig(const aafConfig_e);

//static aafConfig_t getGyroAafConfig(const aafConfig_e config)
//{
//    
//        switch (config) {
//        case AAF_CONFIG_258HZ:
//            return aafLUT42688[AAF_CONFIG_258HZ];
//        case AAF_CONFIG_536HZ:
//            return aafLUT42688[AAF_CONFIG_536HZ];
//        case AAF_CONFIG_997HZ:
//            return aafLUT42688[AAF_CONFIG_997HZ];

//        case AAF_CONFIG_1962HZ:
//            return aafLUT42688[AAF_CONFIG_1962HZ];

//        default:
//            return aafLUT42688[AAF_CONFIG_258HZ];
//        }
//    
//}
///*******************************************************************************
//* 名    称： turnGyroAccOff
//* 功    能： 关闭陀螺仪和加速度计
//* 入口参数： *dev:           设备接口
//* 出口参数： 无
//* 作　　者： DP
//* 创建日期： 2025-01-07
//* 修    改：
//* 修改日期：
//* 备    注： 
//*******************************************************************************/
//static void turnGyroAccOff(const bsp_ICM42688_t *dev)
//{
//    icm42688_write_regs(dev,ICM42688_PWR_MGMT0, ICM426XX_PWR_MGMT0_GYRO_ACCEL_MODE_OFF,1);
//}

///*******************************************************************************
//* 名    称： turnGyroAccOn
//* 功    能： 开启陀螺仪和加速度计
//* 入口参数： *dev:           设备接口
//* 出口参数： 无
//* 作　　者： DP
//* 创建日期： 2025-01-07
//* 修    改：
//* 修改日期：
//* 备    注： 
//*******************************************************************************/
//static void turnGyroAccOn(const bsp_ICM42688_t *dev)
//{
//    icm42688_write_regs(dev,ICM42688_PWR_MGMT0, ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF | ICM426XX_PWR_MGMT0_ACCEL_MODE_LN | ICM426XX_PWR_MGMT0_GYRO_MODE_LN,1);
//    dev->msdelay(1);
//}

///*******************************************************************************
//* 名    称： setUserBank
//* 功    能： 选择要操作的寄存器区块
//* 入口参数： 
//            *dev:           设备接口
//            user_bank：     要操作的区块
//* 出口参数： 无
//* 作　　者： DP
//* 创建日期： 2025-01-07
//* 修    改：
//* 修改日期：
//* 备    注： datasheet page62,63
//*******************************************************************************/
//static void setUserBank(const bsp_ICM42688_t *dev, const uint8_t user_bank)
//{
//    icm42688_write_regs(dev,ICM42688_REG_BANK_SEL, user_bank & 7,1);
//}





///*******************************************************************************
//* 名    称： bsp_Icm42688GetAres
//* 功    能： 获取加速度量程配置
//* 入口参数： 六轴
//* 出口参数： 无
//* 作　　者： DP
//* 创建日期： 2025-01-07
//* 修    改：
//* 修改日期：
//* 备    注： datasheet page62,63
//*******************************************************************************/
//float bsp_Icm42688GetAres(uint8_t Ascale)
//{
//    switch(Ascale)
//    {
//    // Possible accelerometer scales (and their register bit settings) are:
//    // 2 Gs (11), 4 Gs (10), 8 Gs (01), and 16 Gs  (00).
//    case AFS_2G:
//        accSensitivity = 2000 / 32768.0f;
//        break;
//    case AFS_4G:
//        accSensitivity = 4000 / 32768.0f;
//        break;
//    case AFS_8G:
//        accSensitivity = 8000 / 32768.0f;
//        break;
//    case AFS_16G:
//        accSensitivity = 16000 / 32768.0f;
//        break;
//    }

//    return accSensitivity;
//}
///*******************************************************************************
//* 名    称： bsp_IcmGetRawData
//* 功    能： 读取Icm42688加速度陀螺仪数据
//* 入口参数： 六轴
//* 出口参数： 无
//* 作　　者： DP
//* 创建日期： 2025-01-07
//* 修    改：
//* 修改日期：
//* 备    注：
//*******************************************************************************/
//float bsp_Icm42688GetGres(uint8_t Gscale)
//{
//    switch(Gscale)
//    {
//    case GFS_15_125DPS:
//        gyroSensitivity = 15.125f / 32768.0f;
//        break;
//    case GFS_31_25DPS:
//        gyroSensitivity = 31.25f / 32768.0f;
//        break;
//    case GFS_62_5DPS:
//        gyroSensitivity = 62.5f / 32768.0f;
//        break;
//    case GFS_125DPS:
//        gyroSensitivity = 125.0f / 32768.0f;
//        break;
//    case GFS_250DPS:
//        gyroSensitivity = 250.0f / 32768.0f;
//        break;
//    case GFS_500DPS:
//        gyroSensitivity = 500.0f / 32768.0f;
//        break;
//    case GFS_1000DPS:
//        gyroSensitivity = 1000.0f / 32768.0f;
//        break;
//    case GFS_2000DPS:
//        gyroSensitivity = 2000.0f / 32768.0f;
//        break;
//    }
//    return gyroSensitivity;
//}

///*******************************************************************************
//* 名    称： icm42688GyroInit
//* 功    能： ICM42688陀螺仪初始化
//* 入口参数： 
//            *dev:           设备接口
//* 出口参数： 无
//* 作　　者： DP
//* 创建日期： 2025-01-07
//* 修    改：
//* 修改日期：
//* 备    注： datasheet page62,63
//*******************************************************************************/
//void icm42688GyroInit(const bsp_ICM42688_t *dev)
//{
//    //选择Bank0
//    setUserBank(dev, ICM426XX_BANK_SELECT0);
//    //关闭陀螺仪和加速度计（关闭后才能操作其他寄存器）
//    turnGyroAccOff(dev);
//    dev->msdelay(100);
//    //获取陀螺仪滤波器配置
//    aafConfig_t aafConfig = getGyroAafConfig(AAF_CONFIG_258HZ);

//    //选择Bank1
//    setUserBank(dev,ICM426XX_BANK_SELECT1);
//    icm42688_write_regs(dev,ICM42688_GYRO_CONFIG_STATIC3, aafConfig.delt,1);
//    icm42688_write_regs(dev,ICM42688_GYRO_CONFIG_STATIC4, aafConfig.deltSqr & 0xFF,1);
//    icm42688_write_regs(dev,ICM42688_GYRO_CONFIG_STATIC5, (aafConfig.deltSqr >> 8) | (aafConfig.bitshift << 4),1);
//    //获取加速度计滤波器配置
//    aafConfig = getGyroAafConfig(AAF_CONFIG_258HZ);
//    //选择Bank2
//    setUserBank(dev, ICM426XX_BANK_SELECT2);
//    icm42688_write_regs(dev,ICM42688_ACCEL_CONFIG_STATIC2, aafConfig.delt << 1,1);
//    icm42688_write_regs(dev,ICM42688_ACCEL_CONFIG_STATIC3, aafConfig.deltSqr & 0xFF,1);
//    icm42688_write_regs(dev,ICM42688_ACCEL_CONFIG_STATIC4, (aafConfig.deltSqr >> 8) | (aafConfig.bitshift << 4),1);
//    //选择Bank0
//    setUserBank(dev, ICM426XX_BANK_SELECT0);
//    //使能陀螺仪和加速度计滤波器
//    icm42688_write_regs(dev,ICM42688_GYRO_ACCEL_CONFIG0, ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY | ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY,1);

//   // Configure interrupt pin
//   icm42688_write_regs(dev,ICM42688_INT_CONFIG, ICM426XX_INT1_MODE_PULSED | ICM426XX_INT1_DRIVE_CIRCUIT_PP | ICM426XX_INT1_POLARITY_ACTIVE_HIGH,1);
//   icm42688_write_regs(dev,ICM42688_INT_CONFIG0, 0x00,1);

//   icm42688_write_regs(dev,ICM42688_INT_SOURCE0, ICM426XX_UI_DRDY_INT1_EN_ENABLED,1);

//    uint8_t intConfig1Value = 0;
//    icm42688_read_regs(dev,ICM42688_INT_CONFIG1,&intConfig1Value,1);
//    // Datasheet says: "User should change setting to 0 from default setting of 1, for proper INT1 and INT2 pin operation"
//    intConfig1Value &= ~(1 << ICM426XX_INT_ASYNC_RESET_BIT);
//    intConfig1Value |= (ICM426XX_INT_TPULSE_DURATION_8 | ICM426XX_INT_TDEASSERT_DISABLED);

//    icm42688_write_regs(dev,ICM42688_INT_CONFIG1, intConfig1Value,1);
//    //关闭AFSR功能以防止数据异常
//    uint8_t intfConfig1Value =0;
//    icm42688_read_regs(dev,ICM42688_INTF_CONFIG1,&intfConfig1Value,1);
//    intfConfig1Value &= ~ICM426XX_INTF_CONFIG1_AFSR_MASK;
//    intfConfig1Value |= ICM426XX_INTF_CONFIG1_AFSR_DISABLE;
//    icm42688_write_regs(dev,ICM42688_INTF_CONFIG1, intfConfig1Value,1);

//    //开启陀螺仪和加速度计
//    turnGyroAccOn(dev);
//    dev->msdelay(5);

//    //设置加速度计量程和速率
//    bsp_Icm42688GetAres(AFS_8G);
//    setUserBank(dev, ICM426XX_BANK_SELECT0);
//    uint8_t reg_val = 0;//page74
//    icm42688_read_regs(dev,ICM42688_ACCEL_CONFIG0,&reg_val,1);
//    reg_val |= (AFS_8G << 5);   //量程 ±8g
//    reg_val |= (AODR_200Hz);     //输出速率 50HZ
//    icm42688_write_regs(dev,ICM42688_ACCEL_CONFIG0, reg_val,1);
//    dev->msdelay(15);

//    //设置陀螺仪量程和速率
//    bsp_Icm42688GetGres(GFS_1000DPS);
//    setUserBank(dev, ICM426XX_BANK_SELECT0);
//    icm42688_read_regs(dev,ICM42688_GYRO_CONFIG0,&reg_val,1);//page73
//    reg_val |= (GFS_1000DPS << 5);   //量程 ±2000dps
//    reg_val |= (GODR_1000Hz);     //输出速率 50HZ
//    icm42688_write_regs(dev,ICM42688_GYRO_CONFIG0, reg_val,1);
//    dev->msdelay(15);

//}


///*******************************************************************************
//* 名    称： bsp_IcmGetTemperature
//* 功    能： 读取Icm42688 内部传感器温度
//* 入口参数： 无
//* 出口参数： 无
//* 作　　者： dp
//* 创建日期： 2025-01-07
//* 修    改：
//* 修改日期：
//* 备    注： datasheet page62
//*******************************************************************************/
//void bsp_IcmGetTemperature(const bsp_ICM42688_t *dev,int16_t* pTemp)
//{
//    uint8_t buffer[2] = {0};

//    icm42688_read_regs(dev,ICM42688_TEMP_DATA1, buffer, 2);

//    *pTemp = (int16_t)(((int16_t)((buffer[0] << 8) | buffer[1])) / 132.48 + 25);
//    
//}

///*******************************************************************************
//* 名    称： bsp_IcmGetAccelerometer
//* 功    能： 读取Icm42688 加速度的值
//* 入口参数： 三轴加速度的值
//* 出口参数： 无
//* 作　　者： DP
//* 创建日期： 2025-01-07
//* 修    改：
//* 修改日期：
//* 备    注：
//*******************************************************************************/
//void bsp_IcmGetAccelerometer(const bsp_ICM42688_t *dev,icm42688RawData_t* accData)
//{
//    uint8_t buffer[6] = {0};

//    
//    icm42688_read_regs(dev,ICM42688_ACCEL_DATA_X1, buffer, 6);

//    accData->x = ((uint16_t)buffer[0] << 8) | buffer[1];
//    accData->y = ((uint16_t)buffer[2] << 8) | buffer[3];
//    accData->z = ((uint16_t)buffer[4] << 8) | buffer[5];

//    accData->x = (int16_t)(accData->x * accSensitivity);
//    accData->y = (int16_t)(accData->y * accSensitivity);
//    accData->z = (int16_t)(accData->z * accSensitivity);

//  
//}

///*******************************************************************************
//* 名    称： bsp_IcmGetGyroscope
//* 功    能： 读取Icm42688 陀螺仪的值
//* 入口参数： 三轴陀螺仪的值
//* 出口参数： 无
//* 作　　者： dp
//* 创建日期： 2025-01-07
//* 修    改：
//* 修改日期：
//* 备    注： datasheet page63
//*******************************************************************************/
//void bsp_IcmGetGyroscope(const bsp_ICM42688_t *dev,icm42688RawData_t* GyroData)
//{
//    uint8_t buffer[6] = {0};

//    icm42688_read_regs(dev,ICM42688_GYRO_DATA_X1, buffer, 6);

//    GyroData->x = ((uint16_t)buffer[0] << 8) | buffer[1];
//    GyroData->y = ((uint16_t)buffer[2] << 8) | buffer[3];
//    GyroData->z = ((uint16_t)buffer[4] << 8) | buffer[5];

//    GyroData->x = (int16_t)(GyroData->x * gyroSensitivity);
//    GyroData->y = (int16_t)(GyroData->y * gyroSensitivity);
//    GyroData->z = (int16_t)(GyroData->z * gyroSensitivity);
//    
//}

///*******************************************************************************
//* 名    称： bsp_IcmGetGyroscopeZ
//* 功    能： 读取Icm42688 陀螺仪Z轴的值
//* 入口参数： 三轴陀螺仪的值
//* 出口参数： 无
//* 作　　者： dp
//* 创建日期： 2025-01-07
//* 修    改：
//* 修改日期：
//* 备    注： datasheet page63
//*******************************************************************************/
//void bsp_IcmGetGyroscopeZ(const bsp_ICM42688_t *dev,icm42688RawData_t* GyroData)
//{
//    uint8_t buffer[2] = {0};

//    icm42688_read_regs(dev,ICM42688_GYRO_DATA_Z1, buffer, 2);

//    GyroData->z = ((uint16_t)buffer[0] << 8) | buffer[1];

//    
//    
//    
//}
///*******************************************************************************
//* 名    称： bsp_IcmGetRawData
//* 功    能： 读取Icm42688加速度陀螺仪数据
//* 入口参数： 六轴
//* 出口参数： 无
//* 作　　者： dp
//* 创建日期： 2025-01-07
//* 修    改：
//* 修改日期：
//* 备    注： datasheet page62,63
//*******************************************************************************/
//void bsp_IcmGetRawData(const bsp_ICM42688_t *dev,icm42688RawData_t* accData, icm42688RawData_t* GyroData)
//{
//    uint8_t buffer[12] = {0};

//    icm42688_read_regs(dev,ICM42688_ACCEL_DATA_X1, buffer, 12);

//    accData->x  = ((uint16_t)buffer[0] << 8)  | buffer[1];
//    accData->y  = ((uint16_t)buffer[2] << 8)  | buffer[3];
//    accData->z  = ((uint16_t)buffer[4] << 8)  | buffer[5];
//    GyroData->x = ((uint16_t)buffer[6] << 8)  | buffer[7];
//    GyroData->y = ((uint16_t)buffer[8] << 8)  | buffer[9];
//    GyroData->z = ((uint16_t)buffer[10] << 8) | buffer[11];


//    accData->x = (int16_t)(accData->x * accSensitivity);
//    accData->y = (int16_t)(accData->y * accSensitivity);
//    accData->z = (int16_t)(accData->z * accSensitivity);

//    GyroData->x = (int16_t)(GyroData->x * gyroSensitivity);
//    GyroData->y = (int16_t)(GyroData->y * gyroSensitivity);
//		GyroData->z = (int16_t)(GyroData->z * gyroSensitivity);

//}
////示例代码
//// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//// {
////   if(htim->Instance==TIM3)
////   {
////     uint32_t current_tick = HAL_GetTick();
//// 		dt = (current_tick - last_tick) / 1000.0; //
//// 		last_tick = current_tick;
////     //icm42688RawData_t gyroz;
////     bsp_IcmGetGyroscopeZ(&icm42688,&gyro);
////     gyro.z = (int16_t)((gyro.z-3)*0.122);//里边减三去零偏，下面0.122和0.25乘起来就是0.0305即分辨率
////     anglez=anglez+gyro.z*dt*0.25;
//    
////   }

//// }
