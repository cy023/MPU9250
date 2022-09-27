# MPU9250

### Introduction
- Implement the basic operation of IMU IC - MPU9250, including 3-axis accelerator, 3-axis gyroscope, and 3-axis magnetometer.
- Derive the attitude angle (Euler Angle Representation - Roll, Pitch, Yaw) by the following Sensor Fusion Algorithm
  - Complementary Filter
  - Kalman Filter

### Kalman Filter Algorithm
The Kalman Filter algorithm ported from https://github.com/TKJElectronics/KalmanFilter.

### Documents
MPU9250 related files can refer to https://github.com/kriswiner/MPU9250/tree/master/Documents.

### Instructions
You can use the library by the following steps.

- In `mpu9250.h`. Modify the I2C peripheral instance, and include the header file of the i2c driver.
  ```c
  #include "i2c.h"
  #define MPU9250_I2C         hi2c1
  ```
- In `mpu9250.c`. Implement the low-level driver interface.
  ```c
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
  ```
