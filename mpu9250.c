#include "mpu9250.h"
// #include "usart.h"

#define RAD2DEG     57.2957795131

uint8_t ACCEL_FS_SEL_2G = 0x00;
uint8_t ACCEL_FS_SEL_4G = 0x08;
uint8_t ACCEL_FS_SEL_8G = 0x10;
uint8_t ACCEL_FS_SEL_16G = 0x18;

uint8_t GYRO_FS_SEL_250DPS = 0x00;
uint8_t GYRO_FS_SEL_500DPS = 0x08;
uint8_t GYRO_FS_SEL_1000DPS = 0x10;
uint8_t GYRO_FS_SEL_2000DPS = 0x18;

uint8_t PWR_MGMT_1_H_RESET = 0x80;
uint8_t PWR_MGMT_1_WAKEUP = 0x00;
uint8_t PWR_MGMT_1_CLKSEL_PLL = 0x01;

uint8_t PWR_MGMT_2_9DOF = 0x00;
uint8_t INT_STATUS_RAW_DATA_RDY_INT = 0x01;

uint8_t INT_PIN_CFG_BYPASS_LATCHINT = 0x22;
uint8_t INT_ENABLE_RAW_RDY_EN = 0x01;

uint8_t AK8963_ST1_DRDY = 0x01;
uint8_t AK8963_ST2_HOFL = 0x08;

uint8_t AK8963_CNTL1_PWR_DOWN = 0x00;
uint8_t AK8963_CNTL1_FUSE_ROM = 0xFF;
uint8_t AK8963_CNTL2_RESET = 0x01;

////////////////////////////////////////////////////////////////////////////////
//  Low Level Driver
////////////////////////////////////////////////////////////////////////////////
static void MPU9250_I2C_Read(uint8_t regAddr, uint8_t bytes, uint8_t *dest)
{
    HAL_I2C_Mem_Read(&MPU9250_I2C, MPU9250_ADDR, regAddr, I2C_MEMADD_SIZE_8BIT, dest, bytes, HAL_MAX_DELAY);
}

static void MPU9250_I2C_Write(uint8_t regAddr, uint8_t bytes, uint8_t *data)
{
    HAL_I2C_Mem_Write(&MPU9250_I2C, MPU9250_ADDR, regAddr, I2C_MEMADD_SIZE_8BIT, data, bytes, HAL_MAX_DELAY);
}

static void AK8963_I2C_Read(uint8_t regAddr, uint8_t bytes, uint8_t *dest)
{
    HAL_I2C_Mem_Read(&MPU9250_I2C, AK8963_ADDR, regAddr, I2C_MEMADD_SIZE_8BIT, dest, bytes, HAL_MAX_DELAY);
}

static void AK8963_I2C_Write(uint8_t regAddr, uint8_t bytes, uint8_t *data)
{
    HAL_I2C_Mem_Write(&MPU9250_I2C, AK8963_ADDR, regAddr, I2C_MEMADD_SIZE_8BIT, data, bytes, HAL_MAX_DELAY);
}

////////////////////////////////////////////////////////////////////////////////
//  MPU9250 utilities
////////////////////////////////////////////////////////////////////////////////
uint8_t MPU9250_WHOAMI(void)
{
    uint8_t who;
    MPU9250_I2C_Read(WHO_AM_I, 1, &who);
    if (who != WHOAMI_MPU9250)
        return HAL_ERROR;
    return HAL_OK;
}

uint8_t AK8963_WHOAMI(void)
{
    uint8_t who;
    AK8963_I2C_Read(AK8963_WIA, 1, &who);
    if (who != WHOAMI_AK8963)
        return HAL_ERROR;
    return HAL_OK;
}

void MPU9250_SetGyroRange(IMU_t *IMUx, ACCEL_FS_t fullscale)
{
    switch ((int)fullscale)
    {
    case GYRO_FS_250DPS:
        IMUx->gyroScale = 131.0;
        MPU9250_I2C_Write(GYRO_CONFIG, 1, &fullscale);
        break;
    case GYRO_FS_500DPS:
        IMUx->gyroScale = 65.5;
        MPU9250_I2C_Write(GYRO_CONFIG, 1, &fullscale);
        break;
    case GYRO_FS_1000DPS:
        IMUx->gyroScale = 32.8;
        MPU9250_I2C_Write(GYRO_CONFIG, 1, &fullscale);
        break;
    case GYRO_FS_2000DPS:
        IMUx->gyroScale = 16.4;
        MPU9250_I2C_Write(GYRO_CONFIG, 1, &fullscale);
        break;
    default:
        fullscale = 0;
        IMUx->gyroScale = 131.0;
        MPU9250_I2C_Write(GYRO_CONFIG, 1, &fullscale);
        break;
    }
}

void MPU9250_SetAcceRange(IMU_t *IMUx, GYRO_FS_t fullscale)
{
    switch ((int)fullscale)
    {
    case ACCEL_FS_2G:
        IMUx->acceScale = 16384.0;
        MPU9250_I2C_Write(ACCEL_CONFIG, 1, &fullscale);
        break;
    case ACCEL_FS_4G:
        IMUx->acceScale = 8192.0;
        MPU9250_I2C_Write(ACCEL_CONFIG, 1, &fullscale);
        break;
    case ACCEL_FS_8G:
        IMUx->acceScale = 4096.0;
        MPU9250_I2C_Write(ACCEL_CONFIG, 1, &fullscale);
        break;
    case ACCEL_FS_16G:
        IMUx->acceScale = 2048.0;
        MPU9250_I2C_Write(ACCEL_CONFIG, 1, &fullscale);
        break;
    default:
        fullscale = 0;
        IMUx->acceScale = 16384.0;
        MPU9250_I2C_Write(ACCEL_CONFIG, 1, &fullscale);
        break;
    }
}

/* Set Gyroscope low pass filter bandwirth (Temperature be setting simultaneously) */
/* Reference: 4.5 Register 26 – Configuration */
/* ACCEL_FCHOICE bit have to set 0b1 */
/* Gyroscope sample rate: 1kHz */
void MPU9250_SetGyroDLPFBandWidth(DLPF_GYRO_BANDWIDTH_t bandwidth)
{
    switch ((int)bandwidth)
    {
    case DLPF_GYRO_BANDWIDTH_184HZ:
        MPU9250_I2C_Write(CONFIG, 1, &bandwidth);
        break;
    case DLPF_GYRO_BANDWIDTH_92HZ:
        MPU9250_I2C_Write(CONFIG, 1, &bandwidth);
        break;
    case DLPF_GYRO_BANDWIDTH_41HZ:
        MPU9250_I2C_Write(CONFIG, 1, &bandwidth);
        break;
    case DLPF_GYRO_BANDWIDTH_20HZ:
        MPU9250_I2C_Write(CONFIG, 1, &bandwidth);
        break;
    case DLPF_GYRO_BANDWIDTH_10HZ:
        MPU9250_I2C_Write(CONFIG, 1, &bandwidth);
        break;
    case DLPF_GYRO_BANDWIDTH_5HZ:
        MPU9250_I2C_Write(CONFIG, 1, &bandwidth);
        break;
    default:
        bandwidth = DLPF_GYRO_BANDWIDTH_184HZ;
        MPU9250_I2C_Write(CONFIG, 1, &bandwidth);
        break;
    }
}

/* Set Accelerator low pass filter bandwirth. */
/* Reference: Register 29 – Accelerometer Configuration 2 */
/* FCHOICE bit have to set 0b11 */
/* Accelerator sample rate: 1kHz */
void MPU9250_SetAcceDLPFBandWidth(DLPF_ACCE_BANDWIDTH_t bandwidth)
{
    switch ((int)bandwidth)
    {
    case DLPF_ACCE_BANDWIDTH_218_1HZ:
        MPU9250_I2C_Write(ACCEL_CONFIG2, 1, &bandwidth);
        break;
    case DLPF_ACCE_BANDWIDTH_99HZ:
        MPU9250_I2C_Write(ACCEL_CONFIG2, 1, &bandwidth);
        break;
    case DLPF_ACCE_BANDWIDTH_44_8HZ:
        MPU9250_I2C_Write(ACCEL_CONFIG2, 1, &bandwidth);
        break;
    case DLPF_ACCE_BANDWIDTH_21_2HZ:
        MPU9250_I2C_Write(ACCEL_CONFIG2, 1, &bandwidth);
        break;
    case DLPF_ACCE_BANDWIDTH_10_2HZ:
        MPU9250_I2C_Write(ACCEL_CONFIG2, 1, &bandwidth);
        break;
    case DLPF_ACCE_BANDWIDTH_5_05HZ:
        MPU9250_I2C_Write(ACCEL_CONFIG2, 1, &bandwidth);
        break;
    default:
        bandwidth = DLPF_ACCE_BANDWIDTH_218_1HZ;
        MPU9250_I2C_Write(ACCEL_CONFIG2, 1, &bandwidth);
        break;
    }
}

/* SAMPLE_RATE = Internal_Sample_Rate / (1 + SMPLRT_DIV) */
/* Internal_Sample_Rate usually use 1kHz */
void MPU9250_SetSampleRateDIV(uint8_t smplrt_div)
{
    /* Set the IMU sample rate */
    MPU9250_I2C_Write(SMPLRT_DIV, 1, &smplrt_div);
}

void AK8963_SetSampleRateResolution(AK8963_RES_SAMPLERATE_t resSR)
{
    switch ((int)resSR)
    {
    case AK8963_16BIT_100HZ:
        AK8963_I2C_Write(AK8963_CNTL1, 1, &resSR);
        break;
    case AK8963_16BIT_8HZ:
        AK8963_I2C_Write(AK8963_CNTL1, 1, &resSR);
        break;
    case AK8963_14BIT_100HZ:
        AK8963_I2C_Write(AK8963_CNTL1, 1, &resSR);
        break;
    case AK8963_14BIT_8HZ:
        AK8963_I2C_Write(AK8963_CNTL1, 1, &resSR);
        break;
    default:
        resSR = AK8963_16BIT_100HZ;
        AK8963_I2C_Write(AK8963_CNTL1, 1, &resSR);
        break;
    }
}

////////////////////////////////////////////////////////////////////////////////
//  Kalman filter utilities
////////////////////////////////////////////////////////////////////////////////
void Kalman_INIT(IMU_t *IMUx)
{
    for (int i = 0; i < 3; i++) {
        /* We will set the variables like so, these can also be tuned by the user */
        IMUx->kalman[i].Q_angle = 0.001f;
        IMUx->kalman[i].Q_bias = 0.003f;
        IMUx->kalman[i].R_measure = 0.03f;

        IMUx->kalman[i].angle = 0.0f; // Reset the angle
        IMUx->kalman[i].bias = 0.0f; // Reset bias
        // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so
        // - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
        IMUx->kalman[i].P[0][0] = 0.0f;
        IMUx->kalman[i].P[0][1] = 0.0f;
        IMUx->kalman[i].P[1][0] = 0.0f;
        IMUx->kalman[i].P[1][1] = 0.0f;
    }
}

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float Kalman_update(Kalman_t *k, float newAngle, float newRate, float dt)
{
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    k->rate = newRate - k->bias;
    k->angle += dt * k->rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    k->P[0][0] += dt * (dt * k->P[1][1] - k->P[0][1] - k->P[1][0] + k->Q_angle);
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += k->Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = k->P[0][0] + k->R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = k->P[0][0] / S;
    K[1] = k->P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - k->angle; // Angle difference
    /* Step 6 */
    k->angle += K[0] * y;
    k->bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = k->P[0][0];
    float P01_temp = k->P[0][1];

    k->P[0][0] -= K[0] * P00_temp;
    k->P[0][1] -= K[0] * P01_temp;
    k->P[1][0] -= K[1] * P00_temp;
    k->P[1][1] -= K[1] * P01_temp;

    return k->angle;
}

////////////////////////////////////////////////////////////////////////////////
//  HAL
////////////////////////////////////////////////////////////////////////////////

// TODO: Calibration Accelerator
uint8_t MPU9250_Init(IMU_t *IMUx, 
                     ACCEL_FS_t acceFS, 
                     GYRO_FS_t gyroFS, 
                     DLPF_GYRO_BANDWIDTH_t gBW, 
                     DLPF_ACCE_BANDWIDTH_t aBW, 
                     uint8_t smplrt_div)
{
    /* MPU9250 reset */
    MPU9250_I2C_Write(PWR_MGMT_1, 1, &PWR_MGMT_1_H_RESET);
    HAL_Delay(10);

    if (MPU9250_WHOAMI() == HAL_ERROR)
        return HAL_ERROR;

    MPU9250_CalibrateGyroscopeAccelerator(IMUx, ACCEL_FS_2G, GYRO_FS_250DPS, DLPF_GYRO_BANDWIDTH_184HZ, DLPF_ACCE_BANDWIDTH_218_1HZ, 0);
    /* select clock source */
    MPU9250_I2C_Write(PWR_MGMT_1, 1, &PWR_MGMT_1_CLKSEL_PLL);

    MPU9250_SetGyroRange(IMUx, gyroFS);
    MPU9250_SetAcceRange(IMUx, acceFS);

    MPU9250_SetGyroDLPFBandWidth(gBW);
    MPU9250_SetAcceDLPFBandWidth(aBW);
    MPU9250_SetSampleRateDIV(smplrt_div);

    /* set BYPASS_EN, LATCH_INT_EN */
    MPU9250_I2C_Write(INT_PIN_CFG, 1, &INT_PIN_CFG_BYPASS_LATCHINT);
    /* set RAW_RDY_EN */
    MPU9250_I2C_Write(INT_ENABLE, 1, &INT_ENABLE_RAW_RDY_EN);
    HAL_Delay(100);

    if (AK8963_WHOAMI() == HAL_ERROR)
        return HAL_ERROR;

    AK8963_CalibrateMagnetometer(IMUx);
    AK8963_SetSampleRateResolution(AK8963_16BIT_100HZ);
    HAL_Delay(100);

    Kalman_INIT(IMUx);
    return HAL_OK;
}

/* Reference: MPU Hardware Offset Registers App Note */
void MPU9250_CalibrateGyroscopeAccelerator(IMU_t *IMUx, 
                                           ACCEL_FS_t acceFS, 
                                           GYRO_FS_t gyroFS, 
                                           DLPF_GYRO_BANDWIDTH_t gBW, 
                                           DLPF_ACCE_BANDWIDTH_t aBW, 
                                           uint8_t smplrt_div)
{
    MPU9250_I2C_Write(PWR_MGMT_1, 1, &PWR_MGMT_1_CLKSEL_PLL);
    MPU9250_I2C_Write(PWR_MGMT_2, 1, &PWR_MGMT_2_9DOF);
    HAL_Delay(200);

    uint8_t data = 0;
    /* Disable all interrupt, Disable FIFO, Use Internal clock source, Disable I2C master,  */
    MPU9250_I2C_Write(  INT_ENABLE, 1, &data);
    MPU9250_I2C_Write(     FIFO_EN, 1, &data);
    MPU9250_I2C_Write(  PWR_MGMT_1, 1, &data);
    MPU9250_I2C_Write(I2C_MST_CTRL, 1, &data);
    data = 0x07;
    // Disable FIFO and I2C master modes, Reset FIFO and DMP
    MPU9250_I2C_Write(   USER_CTRL, 1, &data);

    MPU9250_SetGyroRange(IMUx, gyroFS);
    MPU9250_SetAcceRange(IMUx, acceFS);
    MPU9250_SetGyroDLPFBandWidth(gBW);
    MPU9250_SetAcceDLPFBandWidth(aBW);
    MPU9250_SetSampleRateDIV(smplrt_div);

    HAL_Delay(100);

    float _AcceX_bias = 0;
    float _AcceY_bias = 0;
    float _AcceZ_bias = 0;
    
    float _GyroX_bias = 0;
    float _GyroY_bias = 0;
    float _GyroZ_bias = 0;

    uint8_t accum = 64;

    for (uint8_t i = 0; i < accum;) {
        uint8_t buffer[15];
        MPU9250_I2C_Read(INT_STATUS, 15, buffer);
        /* check whether data is ready */
        if (!(buffer[0] & INT_STATUS_RAW_DATA_RDY_INT))
            continue;
        else
            i++;
            
        _AcceX_bias += (float)((int16_t)((buffer[1]  << 8) | buffer[2]));
        _AcceY_bias += (float)((int16_t)((buffer[3]  << 8) | buffer[4]));
        _AcceZ_bias += (float)((int16_t)((buffer[5]  << 8) | buffer[6]));
        _GyroX_bias += (float)((int16_t)((buffer[9]  << 8) | buffer[10]));
        _GyroY_bias += (float)((int16_t)((buffer[11] << 8) | buffer[12]));
        _GyroZ_bias += (float)((int16_t)((buffer[13] << 8) | buffer[14]));

        // HAL_Delay(10);
    }

    int16_t AcceX_bias = (int16_t)(_AcceX_bias / accum);
    int16_t AcceY_bias = (int16_t)(_AcceY_bias / accum);
    int16_t AcceZ_bias = (int16_t)(_AcceZ_bias / accum);
    int16_t GyroX_bias = (int16_t)(_GyroX_bias / accum);
    int16_t GyroY_bias = (int16_t)(_GyroY_bias / accum);
    int16_t GyroZ_bias = (int16_t)(_GyroZ_bias / accum);

    // sprintf((char *)buf, "\r\na bias cal %f, %f, %f\r\n\r\n", AcceX_bias / IMUx->acceScale, AcceY_bias / IMUx->acceScale, AcceZ_bias / IMUx->acceScale);
    // HAL_UART_Transmit(&huart4, buf, strlen(buf), 100);
    // sprintf((char *)buf, "\r\na bias cal %d, %d, %d\r\n\r\n", AcceX_bias, AcceY_bias, AcceZ_bias);
    // HAL_UART_Transmit(&huart4, buf, strlen(buf), 100);

    // sprintf((char *)buf, "g bias cal %f, %f, %f\r\n", GyroX_bias / IMUx->gyroScale, GyroY_bias / IMUx->gyroScale, GyroZ_bias / IMUx->gyroScale);
    // HAL_UART_Transmit(&huart4, buf, strlen(buf), 100);

    // Remove gravity from the z-axis accelerometer bias calculation
    if (AcceZ_bias > 0L)
        AcceZ_bias -= IMUx->acceScale;
    else
        AcceZ_bias += IMUx->acceScale;

    IMUx->AcceX_bias = AcceX_bias;
    IMUx->AcceY_bias = AcceY_bias;
    IMUx->AcceZ_bias = AcceZ_bias;

    // uint8_t readData[6];
    // int16_t readAX_bias, readAY_bias, readAZ_bias;
    // int16_t mask[3] = {0};

    // MPU9250_I2C_Read(XA_OFFSET_H, 6, readData);
    // readAX_bias = ((int16_t)readData[0] << 8) | readData[1];
    // readAY_bias = ((int16_t)readData[2] << 8) | readData[3];
    // readAZ_bias = ((int16_t)readData[4] << 8) | readData[5];

    // sprintf((char *)buf, "a mask read bias %d, %d, %d\r\n", readAX_bias & 1, readAY_bias & 1, readAZ_bias & 1);
    // HAL_UART_Transmit(&huart4, buf, strlen(buf), 100);
    // sprintf((char *)buf, "as read bias %d, %d, %d\r\n\r\n", readAX_bias, readAY_bias, readAZ_bias);
    // HAL_UART_Transmit(&huart4, buf, strlen(buf), 100);


    // mask[0] = readAX_bias & 0x0001;
    // mask[1] = readAY_bias & 0x0001;
    // mask[2] = readAZ_bias & 0x0001;

    // readAX_bias -= (AcceX_bias >> 3);
    // readAY_bias -= (AcceY_bias >> 3);
    // readAZ_bias -= (AcceZ_bias >> 3);

    // // Preserve bit 0 of factory value (for temperature compensation)
    // readAX_bias = mask[0] ? (readAX_bias | 0x0001) : (readAX_bias & 0xFFFE);
    // readAY_bias = mask[1] ? (readAY_bias | 0x0001) : (readAY_bias & 0xFFFE);
    // readAZ_bias = mask[2] ? (readAZ_bias | 0x0001) : (readAZ_bias & 0xFFFE);

    // sprintf((char *)buf, "a mask write bias %d, %d, %d\r\n", readAX_bias & 1, readAY_bias & 1, readAZ_bias & 1);
    // HAL_UART_Transmit(&huart4, buf, strlen(buf), 100);
    // sprintf((char *)buf, "as bias write %d, %d, %d\r\n\r\n", readAX_bias, readAY_bias, readAZ_bias);
    // HAL_UART_Transmit(&huart4, buf, strlen(buf), 100);

    // uint8_t Acce_offset[6];
    // Acce_offset[0] = ((int16_t)readAX_bias >> 8) & 0xFF;  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    // Acce_offset[1] = ((int16_t)readAX_bias) & 0xFF;       // Biases are additive, so change sign on calculated average Acce biases
    // Acce_offset[2] = ((int16_t)readAY_bias >> 8) & 0xFF;
    // Acce_offset[3] = ((int16_t)readAY_bias) & 0xFF;
    // Acce_offset[4] = ((int16_t)readAZ_bias >> 8) & 0xFF;
    // Acce_offset[5] = ((int16_t)readAZ_bias) & 0xFF;
    // MPU9250_I2C_Write(XA_OFFSET_H, 6, Acce_offset);
    // HAL_Delay(500);

    // sprintf((char *)buf, "g bias write %f, %f, %f\r\n", GyroX_bias / IMUx->gyroScale, GyroY_bias / IMUx->gyroScale, GyroZ_bias / IMUx->gyroScale);
    // HAL_UART_Transmit(&huart4, buf, strlen(buf), 100);

    // Refserence: MPU Hardware Offset Registers App Note
    //              Example Code from Motion Driver 5.1.2
    // Bias inputs are LSB in +-1000dps format.
    // we set the fsr +-250dps, so divide 4 here.
    uint8_t Gyro_offset[6];
    Gyro_offset[0] = (-(int16_t)GyroX_bias / 4 >> 8) & 0xFF;
    Gyro_offset[1] = (-(int16_t)GyroX_bias / 4) & 0xFF;
    Gyro_offset[2] = (-(int16_t)GyroY_bias / 4 >> 8) & 0xFF;
    Gyro_offset[3] = (-(int16_t)GyroY_bias / 4) & 0xFF;
    Gyro_offset[4] = (-(int16_t)GyroZ_bias / 4 >> 8) & 0xFF;
    Gyro_offset[5] = (-(int16_t)GyroZ_bias / 4) & 0xFF;
    MPU9250_I2C_Write(XG_OFFSET_H, 6, Gyro_offset);
    HAL_Delay(100);
}

void AK8963_CalibrateMagnetometer(IMU_t *IMUx)
{
    /* AK8963 Reset */
    AK8963_I2C_Write(AK8963_CNTL1, 1, &AK8963_CNTL2_RESET);
    HAL_Delay(10);
    /* Set AK8963 to FUSE ROM access */
    AK8963_I2C_Write(AK8963_CNTL1, 1, &AK8963_CNTL1_FUSE_ROM);
    HAL_Delay(10);

    uint8_t asaBuf[3];
    AK8963_I2C_Read(AK8963_ASAX, 3, asaBuf);

    /* 16-bits output */
    /* AK8963 datasheet - Table 8.3 Measurement data format */
    IMUx->magnScale[0] = ((float)(asaBuf[0] - 128.0f) / 256.0f + 1.0f) * 4912.0f / 32760.0f;
    IMUx->magnScale[1] = ((float)(asaBuf[1] - 128.0f) / 256.0f + 1.0f) * 4912.0f / 32760.0f;
    IMUx->magnScale[2] = ((float)(asaBuf[2] - 128.0f) / 256.0f + 1.0f) * 4912.0f / 32760.0f;
}

uint8_t MPU9250_Read_IMU(IMU_t *IMUx)
{
    uint8_t buffer[23];
    MPU9250_I2C_Read(INT_STATUS, 15, buffer);
    if (!(buffer[0] & INT_STATUS_RAW_DATA_RDY_INT))
        return HAL_ERROR;

    AK8963_I2C_Read(AK8963_ST1, 8, buffer + 15);

    int16_t accex = ((int16_t)(buffer[1]  << 8) | (int16_t)buffer[2]) - IMUx->AcceX_bias;
    int16_t accey = ((int16_t)(buffer[3]  << 8) | (int16_t)buffer[4]) - IMUx->AcceY_bias;
    int16_t accez = ((int16_t)(buffer[5]  << 8) | (int16_t)buffer[6]) - IMUx->AcceZ_bias;
    int16_t  temp = ((int16_t)(buffer[7]  << 8) | (int16_t)buffer[8]);
    int16_t gyrox = ((int16_t)(buffer[9]  << 8) | (int16_t)buffer[10]);
    int16_t gyroy = ((int16_t)(buffer[11] << 8) | (int16_t)buffer[12]);
    int16_t gyroz = ((int16_t)(buffer[13] << 8) | (int16_t)buffer[14]);
    int16_t magnx = ((int16_t)(buffer[17] << 8) | (int16_t)buffer[16]);
    int16_t magny = ((int16_t)(buffer[19] << 8) | (int16_t)buffer[18]);
    int16_t magnz = ((int16_t)(buffer[21] << 8) | (int16_t)buffer[20]);

    // uint8_t buff[100];
    // sprintf((char *)buff, "=== %d, %d, %d ===\r\n", accex, accey, accez);
    // HAL_UART_Transmit(&huart4, buff, strlen(buff), 100);

    IMUx->AcceX = (float)accex / IMUx->acceScale;
    IMUx->AcceY = (float)accey / IMUx->acceScale;
    IMUx->AcceZ = (float)accez / IMUx->acceScale;
    IMUx->Temp  = (float)(temp - 21.0f) / 333.87f + 21.0f;
    IMUx->GyroX = (float)gyrox / IMUx->gyroScale;
    IMUx->GyroY = (float)gyroy / IMUx->gyroScale;
    IMUx->GyroZ = (float)gyroz / IMUx->gyroScale;

    if (!(buffer[15] & AK8963_ST1_DRDY) || (buffer[22] & AK8963_ST2_HOFL))
        return HAL_ERROR;
    
    IMUx->MagnX = (float)magnx * IMUx->magnScale[0];
    IMUx->MagnY = (float)magny * IMUx->magnScale[1];
    IMUx->MagnZ = (float)magnz * IMUx->magnScale[2];
    
    return HAL_OK;
}

void Calculate_Attitude_ComplementaryFilter(IMU_t *IMUx)
{
    MPU9250_Read_IMU(IMUx);

    float alpha = 0.98;
    float dt = 0.01;

    float acceRoll  = atan2( IMUx->AcceY, sqrt(IMUx->AcceX * IMUx->AcceX + IMUx->AcceZ * IMUx->AcceZ)) * RAD2DEG;
    float accePitch = atan2( IMUx->AcceX, sqrt(IMUx->AcceY * IMUx->AcceY + IMUx->AcceZ * IMUx->AcceZ)) * RAD2DEG;
    // float magnYaw   = atan2(-IMUx->MagnY, IMUx->MagnX) * RAD2DEG;

    // Complementary filter
    IMUx->roll  = alpha * (IMUx->roll  + IMUx->GyroY * dt) + (1 - alpha) * acceRoll;
    IMUx->pitch = alpha * (IMUx->pitch + IMUx->GyroX * dt) + (1 - alpha) * accePitch;
    IMUx->yaw  += IMUx->GyroZ * dt;
    // IMUx->yaw   = alpha * (IMUx->yaw   + IMUx->GyroZ * dt) + (1 - alpha) * magnYaw;
}

void Calculate_Attitude_KalmanFilter(IMU_t *IMUx)
{
    MPU9250_Read_IMU(IMUx);

    float dt = 0.01;

    float acceRoll  = atan2(IMUx->AcceX, IMUx->AcceZ) * RAD2DEG;
    float accePitch = atan2(IMUx->AcceY, IMUx->AcceZ) * RAD2DEG;
    float magnYaw   = atan2(IMUx->MagnY, IMUx->MagnX) * RAD2DEG;

    // Kalman filter
    IMUx->roll  = Kalman_update(&(IMUx->kalman[0]),  acceRoll, IMUx->GyroX, dt);
    IMUx->pitch = Kalman_update(&(IMUx->kalman[1]), accePitch, IMUx->GyroY, dt);
    IMUx->yaw   = Kalman_update(&(IMUx->kalman[2]),   magnYaw, IMUx->GyroZ, dt);
}
