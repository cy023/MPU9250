#ifndef _MPU9250_H_
#define _MPU9250_H_

#include <stdint.h>
#include <math.h>
#include "i2c.h"

#define MPU9250_I2C         hi2c1
#define MPU9250_ADDR        (0x68 << 1)
#define AK8963_ADDR         (0x0C << 1)
#define WHOAMI_MPU9250      0x71
#define WHOAMI_MPU9255      0x73
#define WHOAMI_AK8963       0x48

/* MPU-9250 Register Map and Descriptions Revision 1.6 */
/* Table 1 MPU-9250 mode register map for Gyroscope and Accelerometer */
#define SELF_TEST_X_GYRO    0x00
#define SELF_TEST_Y_GYRO    0x01
#define SELF_TEST_Z_GYRO    0x02

#define SELF_TEST_X_ACCEL   0x0D
#define SELF_TEST_Y_ACCEL   0x0E
#define SELF_TEST_Z_ACCEL   0x0F

#define XG_OFFSET_H         0x13
#define XG_OFFSET_L         0x14
#define YG_OFFSET_H         0x15
#define YG_OFFSET_L         0x16
#define ZG_OFFSET_H         0x17
#define ZG_OFFSET_L         0x18

#define SMPLRT_DIV          0x19

#define CONFIG              0x1A
#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C
#define ACCEL_CONFIG2       0x1D
#define LP_ACCEL_ODR        0x1E

#define WOM_THR             0x1F
#define FIFO_EN             0x23

#define I2C_MST_CTRL        0x24

#define I2C_SLV0_ADDR       0x25
#define I2C_SLV0_REG        0x26
#define I2C_SLV0_CTRL       0x27

#define I2C_SLV1_ADDR       0x28
#define I2C_SLV1_REG        0x29
#define I2C_SLV1_CTRL       0x2A

#define I2C_SLV2_ADDR       0x2B
#define I2C_SLV2_REG        0x2C
#define I2C_SLV2_CTRL       0x2D

#define I2C_SLV3_ADDR       0x2E
#define I2C_SLV3_REG        0x2F
#define I2C_SLV3_CTRL       0x30

#define I2C_SLV4_ADDR       0x31
#define I2C_SLV4_REG        0x32
#define I2C_SLV4_DO         0x33
#define I2C_SLV4_CTRL       0x34
#define I2C_SLV4_DI         0x35

#define I2C_MST_STATUS      0x36

#define INT_PIN_CFG         0x37
#define INT_ENABLE          0x38
#define INT_STATUS          0x3A

#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40

#define TEMP_OUT_H          0x41
#define TEMP_OUT_L          0x42

#define GYRO_XOUT_H         0x43
#define GYRO_XOUT_L         0x44
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48

#define EXT_SENS_DATA_00    0x49
#define EXT_SENS_DATA_01    0x4A
#define EXT_SENS_DATA_02    0x4B
#define EXT_SENS_DATA_03    0x4C
#define EXT_SENS_DATA_04    0x4D
#define EXT_SENS_DATA_05    0x4E
#define EXT_SENS_DATA_06    0x4F
#define EXT_SENS_DATA_07    0x50
#define EXT_SENS_DATA_08    0x51
#define EXT_SENS_DATA_09    0x52
#define EXT_SENS_DATA_10    0x53
#define EXT_SENS_DATA_11    0x54
#define EXT_SENS_DATA_12    0x55
#define EXT_SENS_DATA_13    0x56
#define EXT_SENS_DATA_14    0x57
#define EXT_SENS_DATA_15    0x58
#define EXT_SENS_DATA_16    0x59
#define EXT_SENS_DATA_17    0x5A
#define EXT_SENS_DATA_18    0x5B
#define EXT_SENS_DATA_19    0x5C
#define EXT_SENS_DATA_20    0x5D
#define EXT_SENS_DATA_21    0x5E
#define EXT_SENS_DATA_22    0x5F
#define EXT_SENS_DATA_23    0x60

#define I2C_SLV0_DO         0x63
#define I2C_SLV1_DO         0x64
#define I2C_SLV2_DO         0x65
#define I2C_SLV3_DO         0x66

#define I2C_MST_DELAY_CTRL  0x67
#define SIGNAL_PATH_RESET   0x68
#define MOT_DETECT_CTRL     0x69

#define USER_CTRL           0x6A

#define PWR_MGMT_1          0x6B
#define PWR_MGMT_2          0x6C

#define FIFO_COUNTH         0x72
#define FIFO_COUNTL         0x73
#define FIFO_R_W            0x74

#define WHO_AM_I            0x75

#define XA_OFFSET_H         0x77
#define XA_OFFSET_L         0x78
#define YA_OFFSET_H         0x7A
#define YA_OFFSET_L         0x7B
#define ZA_OFFSET_H         0x7D
#define ZA_OFFSET_L         0x7E

/* MPU-9250 Register Map and Descriptions Revision 1.6 */
/* Register Map for Magnetometer */
#define AK8963_WIA          0x00
#define AK8963_INFO         0x01
#define AK8963_ST1          0x02

#define AK8963_HXL          0x03
#define AK8963_HXH          0x04
#define AK8963_HYL          0x05
#define AK8963_HYH          0x06
#define AK8963_HZL          0x07
#define AK8963_HZH          0x08

#define AK8963_ST2          0x09
#define AK8963_CNTL1        0x0A
#define AK8963_CNTL2        0x0B
#define AK8963_ASTC         0x0C
#define AK8963_TS1          0x0D
#define AK8963_TS2          0x0E
#define AK8963_I2CDIS       0x0F
#define AK8963_ASAX         0x10
#define AK8963_ASAY         0x11
#define AK8963_ASAZ         0x12

typedef enum {
    GYRO_FS_250DPS = 0,
    GYRO_FS_500DPS = 8,
    GYRO_FS_1000DPS = 16,
    GYRO_FS_2000DPS = 48
} GYRO_FS_t;

typedef enum {
    ACCEL_FS_2G = 0,
    ACCEL_FS_4G = 8,
    ACCEL_FS_8G = 16,
    ACCEL_FS_16G = 32
} ACCEL_FS_t;

/* 4.5 Register 26 – Configuration Table */
typedef enum {
    DLPF_GYRO_BANDWIDTH_184HZ = 1,
    DLPF_GYRO_BANDWIDTH_92HZ,
    DLPF_GYRO_BANDWIDTH_41HZ,
    DLPF_GYRO_BANDWIDTH_20HZ,
    DLPF_GYRO_BANDWIDTH_10HZ,
    DLPF_GYRO_BANDWIDTH_5HZ
} DLPF_GYRO_BANDWIDTH_t;

/* 4.5 Register 26 – Configuration Table */
typedef enum {
    DLPF_ACCE_BANDWIDTH_218_1HZ = 1,
    DLPF_ACCE_BANDWIDTH_99HZ,
    DLPF_ACCE_BANDWIDTH_44_8HZ,
    DLPF_ACCE_BANDWIDTH_21_2HZ,
    DLPF_ACCE_BANDWIDTH_10_2HZ,
    DLPF_ACCE_BANDWIDTH_5_05HZ
} DLPF_ACCE_BANDWIDTH_t;

typedef enum {
    AK8963_16BIT_100HZ = 22,
    AK8963_16BIT_8HZ = 18,
    AK8963_14BIT_100HZ = 6,
    AK8963_14BIT_8HZ = 4
} AK8963_RES_SAMPLERATE_t;

typedef struct {
    float Q_angle;   // Process noise variance for the accelerometer
    float Q_bias;    // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias;  // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate;  // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2]; // Error covariance matrix
} Kalman_t;

typedef struct {
    float GyroX, GyroY, GyroZ;
    float AcceX, AcceY, AcceZ;
    float MagnX, MagnY, MagnZ;
    float Temp;
    float gyroScale;
    float acceScale;
    int16_t AcceX_bias, AcceY_bias, AcceZ_bias;
    float magnScale[3];
    float roll, pitch, yaw;
    Kalman_t kalman[3];
} IMU_t;

uint8_t MPU9250_Init(IMU_t *IMUx,
                     ACCEL_FS_t acceFS,
                     GYRO_FS_t gyroFS,
                     DLPF_GYRO_BANDWIDTH_t gBW,
                     DLPF_ACCE_BANDWIDTH_t aBW,
                     uint8_t smplrt_div);

void MPU9250_CalibrateGyroscopeAccelerator(IMU_t *IMUx, 
                                           ACCEL_FS_t acceFS, 
                                           GYRO_FS_t gyroFS, 
                                           DLPF_GYRO_BANDWIDTH_t gBW, 
                                           DLPF_ACCE_BANDWIDTH_t aBW, 
                                           uint8_t smplrt_div);

void AK8963_CalibrateMagnetometer(IMU_t *IMUx);
uint8_t MPU9250_Read_IMU(IMU_t *IMUx);

void Calculate_Attitude_ComplementaryFilter(IMU_t *IMUx);
void Calculate_Attitude_KalmanFilter(IMU_t *IMUx);

#endif /* _MPU9250_H_ */
