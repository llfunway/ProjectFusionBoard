/**
  ******************************************************************************
  * @file    mpu6050.h
  * @author  Embedded System Team
  * @brief   MPU6050 6-Axis IMU Driver Header
  *          Provides accelerometer and gyroscope interface with Mahony fusion
  ******************************************************************************
  */

#ifndef __MPU6050_H
#define __MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "i2c_hal_wrapper.h"
#include <math.h>
#include <stdbool.h>

/* Exported Macros -----------------------------------------------------------*/
#define MPU6050_ADDR_AD0_LOW            ((uint8_t)0x68)
#define MPU6050_ADDR_AD0_HIGH           ((uint8_t)0x69)
#define MPU6050_DEFAULT_ADDR            MPU6050_ADDR_AD0_LOW

#define MPU6050_REG_SMPLRT_DIV          ((uint8_t)0x19)
#define MPU6050_REG_CONFIG              ((uint8_t)0x1A)
#define MPU6050_REG_GYRO_CONFIG         ((uint8_t)0x1B)
#define MPU6050_REG_ACCEL_CONFIG        ((uint8_t)0x1C)
#define MPU6050_REG_PWR_MGMT_1          ((uint8_t)0x6B)
#define MPU6050_REG_ACCEL_XOUT_H        ((uint8_t)0x3B)
#define MPU6050_REG_TEMP_OUT_H          ((uint8_t)0x41)
#define MPU6050_REG_GYRO_XOUT_H         ((uint8_t)0x43)
#define MPU6050_REG_WHO_AM_I            ((uint8_t)0x75)

#define MPU6050_WHO_AM_I_VALUE          ((uint8_t)0x68)

#define MPU6050_ACCEL_RANGE_2G          ((uint8_t)0x00)
#define MPU6050_ACCEL_RANGE_4G          ((uint8_t)0x08)
#define MPU6050_ACCEL_RANGE_8G          ((uint8_t)0x10)
#define MPU6050_ACCEL_RANGE_16G         ((uint8_t)0x18)

#define MPU6050_GYRO_RANGE_250DPS       ((uint8_t)0x00)
#define MPU6050_GYRO_RANGE_500DPS       ((uint8_t)0x08)
#define MPU6050_GYRO_RANGE_1000DPS      ((uint8_t)0x10)
#define MPU6050_GYRO_RANGE_2000DPS      ((uint8_t)0x18)

#define MPU6050_DLPF_BW_184HZ           ((uint8_t)0x01)
#define MPU6050_DLPF_BW_98HZ            ((uint8_t)0x02)
#define MPU6050_DLPF_BW_42HZ            ((uint8_t)0x03)
#define MPU6050_DLPF_BW_20HZ            ((uint8_t)0x04)
#define MPU6050_DLPF_BW_10HZ            ((uint8_t)0x05)
#define MPU6050_DLPF_BW_5HZ             ((uint8_t)0x06)

#define MPU6050_DEFAULT_ACCEL_RANGE     MPU6050_ACCEL_RANGE_2G
#define MPU6050_DEFAULT_GYRO_RANGE      MPU6050_GYRO_RANGE_500DPS
#define MPU6050_DEFAULT_DLPF            MPU6050_DLPF_BW_42HZ
#define MPU6050_DEFAULT_SAMPLE_RATE     ((uint8_t)0x04)

/* Exported Types ------------------------------------------------------------*/
typedef enum
{
  MPU6050_OK = 0,
  MPU6050_ERROR = -1,
  MPU6050_ERROR_NOT_INIT = -2,
  MPU6050_ERROR_ID = -3,
  MPU6050_ERROR_I2C = -4,
  MPU6050_ERROR_PARAM = -5,
  MPU6050_ERROR_TIMEOUT = -6
} MPU6050_Status_t;

typedef struct
{
  int16_t AccelX;
  int16_t AccelY;
  int16_t AccelZ;
  int16_t Temp;
  int16_t GyroX;
  int16_t GyroY;
  int16_t GyroZ;
} MPU6050_RawData_t;

typedef struct
{
  float AccelX;
  float AccelY;
  float AccelZ;
  float Temp;
  float GyroX;
  float GyroY;
  float GyroZ;
} MPU6050_Data_t;

typedef struct
{
  float Roll;
  float Pitch;
  uint32_t Timestamp;
} MPU6050_Attitude_t;

typedef struct
{
  int16_t AccelOffsetX;
  int16_t AccelOffsetY;
  int16_t AccelOffsetZ;
  int16_t GyroOffsetX;
  int16_t GyroOffsetY;
  int16_t GyroOffsetZ;
} MPU6050_Calibration_t;

typedef struct
{
  I2C_HandleTypeDef *hi2c;
  uint8_t I2cAddress;
  uint8_t AccelRange;
  uint8_t GyroRange;
  uint8_t DlpfBandwidth;
  uint8_t SampleRateDiv;
  uint32_t Timeout;
} MPU6050_Config_t;

typedef struct
{
  MPU6050_Config_t Config;
  I2C_Device_TypeDef I2cDevice;
  MPU6050_RawData_t RawData;
  MPU6050_Data_t Data;
  MPU6050_Calibration_t Calibration;
  MPU6050_Attitude_t Attitude;
  uint8_t IsInitialized;
  uint8_t DeviceID;
  
  float q0;
  float q1;
  float q2;
  float q3;
  float twoKp;
  float twoKi;
  float integralFBx;
  float integralFBy;
  float integralFBz;
  float dt;
  uint32_t lastTime;
  float nominalTwoKp;
  float nominalTwoKi;
  uint32_t startupBoostUntil;
  float runtimeGyroBiasX;
  float runtimeGyroBiasY;
  float runtimeGyroBiasZ;
} MPU6050_Handle_t;

/* Exported Functions --------------------------------------------------------*/

MPU6050_Status_t MPU6050_Init(MPU6050_Handle_t *hmpu, MPU6050_Config_t *pConfig);

MPU6050_Status_t MPU6050_ReadRawData(MPU6050_Handle_t *hmpu);

MPU6050_Status_t MPU6050_UpdateAttitude(MPU6050_Handle_t *hmpu);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H */
