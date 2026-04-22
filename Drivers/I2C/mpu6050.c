/**
  ******************************************************************************
  * @file    mpu6050.c
  * @author  Embedded System Team
  * @brief   MPU6050 6-Axis IMU Driver Implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "mpu6050.h"

/* Private Macros ------------------------------------------------------------*/
#ifndef M_PI
#define M_PI                          (3.14159265358979323846f)
#endif

#define ACCEL_SENSITIVITY_2G          (16384.0f)
#define ACCEL_SENSITIVITY_4G          (8192.0f)
#define ACCEL_SENSITIVITY_8G          (4096.0f)
#define ACCEL_SENSITIVITY_16G         (2048.0f)

#define GYRO_SENSITIVITY_250DPS       (131.0f)
#define GYRO_SENSITIVITY_500DPS       (65.5f)
#define GYRO_SENSITIVITY_1000DPS      (32.8f)
#define GYRO_SENSITIVITY_2000DPS      (16.4f)

#define TEMP_SENSITIVITY              (340.0f)
#define TEMP_OFFSET                   (36.53f)

/* Private Function Prototypes -----------------------------------------------*/
static float MPU6050_GetAccelSensitivity(uint8_t range);
static float MPU6050_GetGyroSensitivity(uint8_t range);
static void MPU6050_MahonyInit(MPU6050_Handle_t *hmpu);
static float MPU6050_InvSqrt(float x);

/* Public Functions ----------------------------------------------------------*/

MPU6050_Status_t MPU6050_Init(MPU6050_Handle_t *hmpu, MPU6050_Config_t *pConfig)
{
  int8_t i2cRet;
  uint8_t deviceId = 0;

  if (hmpu == NULL || pConfig == NULL)
  {
    return MPU6050_ERROR_PARAM;
  }

  if (pConfig->hi2c == NULL)
  {
    return MPU6050_ERROR_PARAM;
  }

  hmpu->Config = *pConfig;

  i2cRet = I2C_Device_Init(&hmpu->I2cDevice,
                           pConfig->hi2c,
                           pConfig->I2cAddress,
                           pConfig->Timeout);

  if (i2cRet != I2C_OK)
  {
    return MPU6050_ERROR_I2C;
  }

  i2cRet = I2C_ReadByte(&hmpu->I2cDevice, MPU6050_REG_WHO_AM_I, &deviceId);
  if (i2cRet != I2C_OK)
  {
    return MPU6050_ERROR_I2C;
  }

  if (deviceId != MPU6050_WHO_AM_I_VALUE)
  {
    hmpu->DeviceID = deviceId;
    return MPU6050_ERROR_ID;
  }
  hmpu->DeviceID = deviceId;

  i2cRet = I2C_WriteByte(&hmpu->I2cDevice, MPU6050_REG_PWR_MGMT_1, 0x00);
  if (i2cRet != I2C_OK)
  {
    return MPU6050_ERROR_I2C;
  }

  i2cRet = I2C_WriteByte(&hmpu->I2cDevice, MPU6050_REG_SMPLRT_DIV, pConfig->SampleRateDiv);
  if (i2cRet != I2C_OK)
  {
    return MPU6050_ERROR_I2C;
  }

  i2cRet = I2C_WriteByte(&hmpu->I2cDevice, MPU6050_REG_CONFIG, pConfig->DlpfBandwidth);
  if (i2cRet != I2C_OK)
  {
    return MPU6050_ERROR_I2C;
  }

  i2cRet = I2C_WriteByte(&hmpu->I2cDevice, MPU6050_REG_GYRO_CONFIG, pConfig->GyroRange);
  if (i2cRet != I2C_OK)
  {
    return MPU6050_ERROR_I2C;
  }

  i2cRet = I2C_WriteByte(&hmpu->I2cDevice, MPU6050_REG_ACCEL_CONFIG, pConfig->AccelRange);
  if (i2cRet != I2C_OK)
  {
    return MPU6050_ERROR_I2C;
  }

  hmpu->Calibration.AccelOffsetX = 0;
  hmpu->Calibration.AccelOffsetY = 0;
  hmpu->Calibration.AccelOffsetZ = 0;
  hmpu->Calibration.GyroOffsetX = 0;
  hmpu->Calibration.GyroOffsetY = 0;
  hmpu->Calibration.GyroOffsetZ = 0;

  MPU6050_MahonyInit(hmpu);
  hmpu->IsInitialized = 1;

  return MPU6050_OK;
}

MPU6050_Status_t MPU6050_ReadRawData(MPU6050_Handle_t *hmpu)
{
  uint8_t buffer[14];
  int8_t i2cRet;
  float accelScale;
  float gyroScale;

  if (hmpu == NULL || !hmpu->IsInitialized)
  {
    return MPU6050_ERROR_NOT_INIT;
  }

  i2cRet = I2C_ReadBytes(&hmpu->I2cDevice, MPU6050_REG_ACCEL_XOUT_H, buffer, 14);
  if (i2cRet != I2C_OK)
  {
    return MPU6050_ERROR_I2C;
  }

  hmpu->RawData.AccelX = (int16_t)((buffer[0] << 8) | buffer[1]);
  hmpu->RawData.AccelY = (int16_t)((buffer[2] << 8) | buffer[3]);
  hmpu->RawData.AccelZ = (int16_t)((buffer[4] << 8) | buffer[5]);
  hmpu->RawData.Temp = (int16_t)((buffer[6] << 8) | buffer[7]);
  hmpu->RawData.GyroX = (int16_t)((buffer[8] << 8) | buffer[9]);
  hmpu->RawData.GyroY = (int16_t)((buffer[10] << 8) | buffer[11]);
  hmpu->RawData.GyroZ = (int16_t)((buffer[12] << 8) | buffer[13]);

  accelScale = MPU6050_GetAccelSensitivity(hmpu->Config.AccelRange);
  gyroScale = MPU6050_GetGyroSensitivity(hmpu->Config.GyroRange);

  hmpu->Data.AccelX = (float)hmpu->RawData.AccelX / accelScale;
  hmpu->Data.AccelY = (float)hmpu->RawData.AccelY / accelScale;
  hmpu->Data.AccelZ = (float)hmpu->RawData.AccelZ / accelScale;
  hmpu->Data.Temp = (float)hmpu->RawData.Temp / TEMP_SENSITIVITY + TEMP_OFFSET;
  hmpu->Data.GyroX = (float)hmpu->RawData.GyroX / gyroScale;
  hmpu->Data.GyroY = (float)hmpu->RawData.GyroY / gyroScale;
  hmpu->Data.GyroZ = (float)hmpu->RawData.GyroZ / gyroScale;

  return MPU6050_OK;
}

MPU6050_Status_t MPU6050_UpdateAttitude(MPU6050_Handle_t *hmpu)
{
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float recipNorm;
  float halfvx;
  float halfvy;
  float halfvz;
  float halfex;
  float halfey;
  float halfez;
  float qa;
  float qb;
  float qc;
  float sinp;
  uint32_t currentTime;

  if (hmpu == NULL || !hmpu->IsInitialized)
  {
    return MPU6050_ERROR_NOT_INIT;
  }

  currentTime = HAL_GetTick();
  hmpu->dt = (float)(currentTime - hmpu->lastTime) / 1000.0f;
  hmpu->lastTime = currentTime;

  if (hmpu->dt <= 0.0f || hmpu->dt > 0.1f)
  {
    hmpu->dt = 0.01f;
  }

  ax = hmpu->Data.AccelX;
  ay = hmpu->Data.AccelY;
  az = hmpu->Data.AccelZ;
  gx = hmpu->Data.GyroX * ((float)M_PI / 180.0f);
  gy = hmpu->Data.GyroY * ((float)M_PI / 180.0f);
  gz = hmpu->Data.GyroZ * ((float)M_PI / 180.0f);

  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {
    recipNorm = MPU6050_InvSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    halfvx = hmpu->q1 * hmpu->q3 - hmpu->q0 * hmpu->q2;
    halfvy = hmpu->q0 * hmpu->q1 + hmpu->q2 * hmpu->q3;
    halfvz = hmpu->q0 * hmpu->q0 - 0.5f + hmpu->q3 * hmpu->q3;

    halfex = (ay * halfvz) - (az * halfvy);
    halfey = (az * halfvx) - (ax * halfvz);
    halfez = (ax * halfvy) - (ay * halfvx);

    if (hmpu->twoKi > 0.0f)
    {
      hmpu->integralFBx += hmpu->twoKi * halfex * hmpu->dt;
      hmpu->integralFBy += hmpu->twoKi * halfey * hmpu->dt;
      hmpu->integralFBz += hmpu->twoKi * halfez * hmpu->dt;
      gx += hmpu->integralFBx;
      gy += hmpu->integralFBy;
      gz += hmpu->integralFBz;
    }
    else
    {
      hmpu->integralFBx = 0.0f;
      hmpu->integralFBy = 0.0f;
      hmpu->integralFBz = 0.0f;
    }

    gx += hmpu->twoKp * halfex;
    gy += hmpu->twoKp * halfey;
    gz += hmpu->twoKp * halfez;
  }

  gx *= (0.5f * hmpu->dt);
  gy *= (0.5f * hmpu->dt);
  gz *= (0.5f * hmpu->dt);

  qa = hmpu->q0;
  qb = hmpu->q1;
  qc = hmpu->q2;

  hmpu->q0 += (-qb * gx - qc * gy - hmpu->q3 * gz);
  hmpu->q1 += (qa * gx + qc * gz - hmpu->q3 * gy);
  hmpu->q2 += (qa * gy - qb * gz + hmpu->q3 * gx);
  hmpu->q3 += (qa * gz + qb * gy - qc * gx);

  recipNorm = MPU6050_InvSqrt(hmpu->q0 * hmpu->q0 +
                              hmpu->q1 * hmpu->q1 +
                              hmpu->q2 * hmpu->q2 +
                              hmpu->q3 * hmpu->q3);
  hmpu->q0 *= recipNorm;
  hmpu->q1 *= recipNorm;
  hmpu->q2 *= recipNorm;
  hmpu->q3 *= recipNorm;

  hmpu->Attitude.Roll = atan2f(2.0f * (hmpu->q0 * hmpu->q1 + hmpu->q2 * hmpu->q3),
                               1.0f - 2.0f * (hmpu->q1 * hmpu->q1 + hmpu->q2 * hmpu->q2))
                        * (180.0f / (float)M_PI);

  sinp = 2.0f * (hmpu->q0 * hmpu->q2 - hmpu->q3 * hmpu->q1);
  if (sinp > 1.0f)
  {
    sinp = 1.0f;
  }
  else if (sinp < -1.0f)
  {
    sinp = -1.0f;
  }

  hmpu->Attitude.Pitch = asinf(sinp) * (180.0f / (float)M_PI);

  hmpu->Attitude.Timestamp = currentTime;

  return MPU6050_OK;
}

/* Private Functions ---------------------------------------------------------*/

static float MPU6050_GetAccelSensitivity(uint8_t range)
{
  switch (range)
  {
    case MPU6050_ACCEL_RANGE_4G:
      return ACCEL_SENSITIVITY_4G;
    case MPU6050_ACCEL_RANGE_8G:
      return ACCEL_SENSITIVITY_8G;
    case MPU6050_ACCEL_RANGE_16G:
      return ACCEL_SENSITIVITY_16G;
    case MPU6050_ACCEL_RANGE_2G:
    default:
      return ACCEL_SENSITIVITY_2G;
  }
}

static float MPU6050_GetGyroSensitivity(uint8_t range)
{
  switch (range)
  {
    case MPU6050_GYRO_RANGE_250DPS:
      return GYRO_SENSITIVITY_250DPS;
    case MPU6050_GYRO_RANGE_1000DPS:
      return GYRO_SENSITIVITY_1000DPS;
    case MPU6050_GYRO_RANGE_2000DPS:
      return GYRO_SENSITIVITY_2000DPS;
    case MPU6050_GYRO_RANGE_500DPS:
    default:
      return GYRO_SENSITIVITY_500DPS;
  }
}

static void MPU6050_MahonyInit(MPU6050_Handle_t *hmpu)
{
  hmpu->q0 = 1.0f;
  hmpu->q1 = 0.0f;
  hmpu->q2 = 0.0f;
  hmpu->q3 = 0.0f;
  hmpu->twoKp = 2.0f * 0.5f;
  hmpu->twoKi = 2.0f * 0.05f;
  hmpu->integralFBx = 0.0f;
  hmpu->integralFBy = 0.0f;
  hmpu->integralFBz = 0.0f;
  hmpu->dt = 0.0f;
  hmpu->lastTime = HAL_GetTick();
  hmpu->Attitude.Roll = 0.0f;
  hmpu->Attitude.Pitch = 0.0f;
  hmpu->Attitude.Timestamp = 0;
}

static float MPU6050_InvSqrt(float x)
{
  return 1.0f / sqrtf(x);
}
