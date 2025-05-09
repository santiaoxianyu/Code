//#ifndef __ICM42688_H__
//#define __ICM42688_H__

//#include <stdint.h>
//#include "main.h"
//#define ICM_USE_HARD_SPI
////#define ICM_USE_HARD_I2C

//typedef struct {
//    void (*write_reg)(uint8_t register_address,uint8_t data,uint16_t len);
//    void(*read_reg)(uint8_t register_address,uint8_t *data,uint16_t len);
//    void (*msdelay)(uint32_t us);
//    void *handle;  // 用于存储特定于平台的上下文信息
//} bsp_ICM42688_t;


//#define ICM426XX_BANK_SELECT0                       0x00
//#define ICM426XX_BANK_SELECT1                       0x01
//#define ICM426XX_BANK_SELECT2                       0x02
//#define ICM426XX_BANK_SELECT3                       0x03
//#define ICM426XX_BANK_SELECT4                       0x04

//#define ICM426XX_PWR_MGMT0_ACCEL_MODE_LN            (3 << 0)
//#define ICM426XX_PWR_MGMT0_GYRO_MODE_LN             (3 << 2)
//#define ICM426XX_PWR_MGMT0_GYRO_ACCEL_MODE_OFF      ((0 << 0) | (0 << 2))
//#define ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF         (0 << 5)
//#define ICM426XX_PWR_MGMT0_TEMP_DISABLE_ON          (1 << 5)

//#define ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY       (15 << 4) 
//#define ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY        (15 << 0)

//#define ICM426XX_INTF_CONFIG1_AFSR_MASK             0xC0
//#define ICM426XX_INTF_CONFIG1_AFSR_DISABLE          0x40

//#define ICM426XX_INT1_MODE_PULSED                   (0 << 2)
//#define ICM426XX_INT1_MODE_LATCHED                  (1 << 2)
//#define ICM426XX_INT1_DRIVE_CIRCUIT_OD              (0 << 1)
//#define ICM426XX_INT1_DRIVE_CIRCUIT_PP              (1 << 1)
//#define ICM426XX_INT1_POLARITY_ACTIVE_LOW           (0 << 0)
//#define ICM426XX_INT1_POLARITY_ACTIVE_HIGH          (1 << 0)

//#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR           ((0 << 5) || (0 << 4))
//#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR_DUPLICATE ((0 << 5) || (0 << 4)) // duplicate settings in datasheet, Rev 1.2.
//#define ICM426XX_UI_DRDY_INT_CLEAR_ON_F1BR          ((1 << 5) || (0 << 4))
//#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR_AND_F1BR  ((1 << 5) || (1 << 4))

//#define ICM426XX_UI_DRDY_INT1_EN_DISABLED           (0 << 3)
//#define ICM426XX_UI_DRDY_INT1_EN_ENABLED            (1 << 3)

//#define ICM426XX_INT_ASYNC_RESET_BIT                4
//#define ICM426XX_INT_TDEASSERT_DISABLE_BIT          5
//#define ICM426XX_INT_TDEASSERT_ENABLED              (0 << ICM426XX_INT_TDEASSERT_DISABLE_BIT)
//#define ICM426XX_INT_TDEASSERT_DISABLED             (1 << ICM426XX_INT_TDEASSERT_DISABLE_BIT)
//#define ICM426XX_INT_TPULSE_DURATION_BIT            6
//#define ICM426XX_INT_TPULSE_DURATION_100            (0 << ICM426XX_INT_TPULSE_DURATION_BIT)
//#define ICM426XX_INT_TPULSE_DURATION_8              (1 << ICM426XX_INT_TPULSE_DURATION_BIT)

//// Bank 0
//#define ICM42688_DEVICE_CONFIG             0x11
//#define ICM42688_DRIVE_CONFIG              0x13
//#define ICM42688_INT_CONFIG                0x14
//#define ICM42688_FIFO_CONFIG               0x16
//#define ICM42688_TEMP_DATA1                0x1D
//#define ICM42688_TEMP_DATA0                0x1E
//#define ICM42688_ACCEL_DATA_X1             0x1F
//#define ICM42688_ACCEL_DATA_X0             0x20
//#define ICM42688_ACCEL_DATA_Y1             0x21
//#define ICM42688_ACCEL_DATA_Y0             0x22
//#define ICM42688_ACCEL_DATA_Z1             0x23
//#define ICM42688_ACCEL_DATA_Z0             0x24
//#define ICM42688_GYRO_DATA_X1              0x25
//#define ICM42688_GYRO_DATA_X0              0x26
//#define ICM42688_GYRO_DATA_Y1              0x27
//#define ICM42688_GYRO_DATA_Y0              0x28
//#define ICM42688_GYRO_DATA_Z1              0x29
//#define ICM42688_GYRO_DATA_Z0              0x2A
//#define ICM42688_TMST_FSYNCH               0x2B
//#define ICM42688_TMST_FSYNCL               0x2C
//#define ICM42688_INT_STATUS                0x2D
//#define ICM42688_FIFO_COUNTH               0x2E
//#define ICM42688_FIFO_COUNTL               0x2F
//#define ICM42688_FIFO_DATA                 0x30
//#define ICM42688_APEX_DATA0                0x31
//#define ICM42688_APEX_DATA1                0x32
//#define ICM42688_APEX_DATA2                0x33
//#define ICM42688_APEX_DATA3                0x34
//#define ICM42688_APEX_DATA4                0x35
//#define ICM42688_APEX_DATA5                0x36
//#define ICM42688_INT_STATUS2               0x37
//#define ICM42688_INT_STATUS3               0x38
//#define ICM42688_SIGNAL_PATH_RESET         0x4B
//#define ICM42688_INTF_CONFIG0              0x4C
//#define ICM42688_INTF_CONFIG1              0x4D
//#define ICM42688_PWR_MGMT0                 0x4E
//#define ICM42688_GYRO_CONFIG0              0x4F
//#define ICM42688_ACCEL_CONFIG0             0x50
//#define ICM42688_GYRO_CONFIG1              0x51
//#define ICM42688_GYRO_ACCEL_CONFIG0        0x52
//#define ICM42688_ACCEL_CONFIG1             0x53
//#define ICM42688_TMST_CONFIG               0x54
//#define ICM42688_APEX_CONFIG0              0x56
//#define ICM42688_SMD_CONFIG                0x57
//#define ICM42688_FIFO_CONFIG1              0x5F
//#define ICM42688_FIFO_CONFIG2              0x60
//#define ICM42688_FIFO_CONFIG3              0x61
//#define ICM42688_FSYNC_CONFIG              0x62
//#define ICM42688_INT_CONFIG0               0x63
//#define ICM42688_INT_CONFIG1               0x64
//#define ICM42688_INT_SOURCE0               0x65
//#define ICM42688_INT_SOURCE1               0x66
//#define ICM42688_INT_SOURCE3               0x68
//#define ICM42688_INT_SOURCE4               0x69
//#define ICM42688_FIFO_LOST_PKT0            0x6C
//#define ICM42688_FIFO_LOST_PKT1            0x6D
//#define ICM42688_SELF_TEST_CONFIG          0x70
//#define ICM42688_WHO_AM_I                  0x75
//#define ICM42688_REG_BANK_SEL              0x76

//// Bank 1
//#define ICM42688_SENSOR_CONFIG0            0x03
//#define ICM42688_GYRO_CONFIG_STATIC2       0x0B
//#define ICM42688_GYRO_CONFIG_STATIC3       0x0C
//#define ICM42688_GYRO_CONFIG_STATIC4       0x0D
//#define ICM42688_GYRO_CONFIG_STATIC5       0x0E
//#define ICM42688_GYRO_CONFIG_STATIC6       0x0F
//#define ICM42688_GYRO_CONFIG_STATIC7       0x10
//#define ICM42688_GYRO_CONFIG_STATIC8       0x11
//#define ICM42688_GYRO_CONFIG_STATIC9       0x12
//#define ICM42688_GYRO_CONFIG_STATIC10      0x13
//#define ICM42688_XG_ST_DATA                0x5F
//#define ICM42688_YG_ST_DATA                0x60
//#define ICM42688_ZG_ST_DATA                0x61
//#define ICM42688_TMSTVAL0                  0x62
//#define ICM42688_TMSTVAL1                  0x63
//#define ICM42688_TMSTVAL2                  0x64
//#define ICM42688_INTF_CONFIG4              0x7A
//#define ICM42688_INTF_CONFIG5              0x7B
//#define ICM42688_INTF_CONFIG6              0x7C

//// Bank 2
//#define ICM42688_ACCEL_CONFIG_STATIC2      0x03
//#define ICM42688_ACCEL_CONFIG_STATIC3      0x04
//#define ICM42688_ACCEL_CONFIG_STATIC4      0x05
//#define ICM42688_XA_ST_DATA                0x3B
//#define ICM42688_YA_ST_DATA                0x3C
//#define ICM42688_ZA_ST_DATA                0x3D

//// Bank 4
//#define ICM42688_GYRO_ON_OFF_CONFIG        0x0E
//#define ICM42688_APEX_CONFIG1              0x40
//#define ICM42688_APEX_CONFIG2              0x41
//#define ICM42688_APEX_CONFIG3              0x42
//#define ICM42688_APEX_CONFIG4              0x43
//#define ICM42688_APEX_CONFIG5              0x44
//#define ICM42688_APEX_CONFIG6              0x45
//#define ICM42688_APEX_CONFIG7              0x46
//#define ICM42688_APEX_CONFIG8              0x47
//#define ICM42688_APEX_CONFIG9              0x48
//#define ICM42688_ACCEL_WOM_X_THR           0x4A
//#define ICM42688_ACCEL_WOM_Y_THR           0x4B
//#define ICM42688_ACCEL_WOM_Z_THR           0x4C
//#define ICM42688_INT_SOURCE6               0x4D
//#define ICM42688_INT_SOURCE7               0x4E
//#define ICM42688_INT_SOURCE8               0x4F
//#define ICM42688_INT_SOURCE9               0x50
//#define ICM42688_INT_SOURCE10              0x51
//#define ICM42688_OFFSET_USER0              0x77
//#define ICM42688_OFFSET_USER1              0x78
//#define ICM42688_OFFSET_USER2              0x79
//#define ICM42688_OFFSET_USER3              0x7A
//#define ICM42688_OFFSET_USER4              0x7B
//#define ICM42688_OFFSET_USER5              0x7C
//#define ICM42688_OFFSET_USER6              0x7D
//#define ICM42688_OFFSET_USER7              0x7E
//#define ICM42688_OFFSET_USER8              0x7F

//#define ICM42688_ADDRESS                   0x69   // Address of ICM42688 accel/gyro when ADO = HIGH


//#define AFS_2G  0x03
//#define AFS_4G  0x02
//#define AFS_8G  0x01
//#define AFS_16G 0x00  // default

//#define GFS_2000DPS   0x00 // default
//#define GFS_1000DPS   0x01
//#define GFS_500DPS    0x02
//#define GFS_250DPS    0x03
//#define GFS_125DPS    0x04
//#define GFS_62_5DPS   0x05
//#define GFS_31_25DPS  0x06
//#define GFS_15_125DPS 0x07

//#define AODR_8000Hz   0x03
//#define AODR_4000Hz   0x04
//#define AODR_2000Hz   0x05
//#define AODR_1000Hz   0x06 // default
//#define AODR_200Hz    0x07
//#define AODR_100Hz    0x08
//#define AODR_50Hz     0x09
//#define AODR_25Hz     0x0A
//#define AODR_12_5Hz   0x0B
//#define AODR_6_25Hz   0x0C
//#define AODR_3_125Hz  0x0D
//#define AODR_1_5625Hz 0x0E
//#define AODR_500Hz    0x0F

//#define GODR_8000Hz  0x03
//#define GODR_4000Hz  0x04
//#define GODR_2000Hz  0x05
//#define GODR_1000Hz  0x06 // default
//#define GODR_200Hz   0x07
//#define GODR_100Hz   0x08
//#define GODR_50Hz    0x09
//#define GODR_25Hz    0x0A
//#define GODR_12_5Hz  0x0B
//#define GODR_500Hz   0x0F

//typedef enum {
//    ODR_CONFIG_8K = 0,
//    ODR_CONFIG_4K,
//    ODR_CONFIG_2K,
//    ODR_CONFIG_1K,
//    ODR_CONFIG_COUNT
//} odrConfig_e;

//typedef enum {
//    AAF_CONFIG_258HZ = 0,
//    AAF_CONFIG_536HZ,
//    AAF_CONFIG_997HZ,
//    AAF_CONFIG_1962HZ,
//    AAF_CONFIG_COUNT
//} aafConfig_e;

//typedef struct aafConfig_s {
//    uint8_t delt;
//    uint16_t deltSqr;
//    uint8_t bitshift;
//} aafConfig_t;

//static aafConfig_t aafLUT42688[AAF_CONFIG_COUNT] = {  // see table in section 5.3
//    [AAF_CONFIG_258HZ]  = {  6,   36, 10 },
//    [AAF_CONFIG_536HZ]  = { 12,  144,  8 },
//    [AAF_CONFIG_997HZ]  = { 21,  440,  6 },
//    [AAF_CONFIG_1962HZ] = { 37, 1376,  4 },
//};

//#define ICM42688_ID	             0x47	


//typedef struct {
//  int16_t x; /**< Raw int16_t value from the x axis */
//  int16_t y; /**< Raw int16_t value from the y axis */
//  int16_t z; /**< Raw int16_t value from the z axis */
//} icm42688RawData_t;

////--------------------------------------------------------//
//int8_t bsp_Icm42688RegCfg(const bsp_ICM42688_t *dev);
//void icm42688GyroInit(const bsp_ICM42688_t *dev);
//void icm42688_read_regs(const bsp_ICM42688_t *dev,uint8_t reg_address,uint8_t *data,uint16_t len);
//void bsp_IcmGetTemperature(const bsp_ICM42688_t *dev,int16_t* pTemp);
//void bsp_IcmGetAccelerometer(const bsp_ICM42688_t *dev,icm42688RawData_t* accData);
//void bsp_IcmGetGyroscope(const bsp_ICM42688_t *dev,icm42688RawData_t* GyroData);
//void bsp_IcmGetRawData(const bsp_ICM42688_t *dev,icm42688RawData_t* accData, icm42688RawData_t* GyroData);
//void bsp_IcmGetGyroscopeZ(const bsp_ICM42688_t *dev,icm42688RawData_t* GyroData);
//void CalibrateGyroZ(const bsp_ICM42688_t *dev,float GyroZOffset);
//void bsp_icmGetYAW(const bsp_ICM42688_t *dev,icm42688RawData_t* GyroData,float yaw);
//#endif
