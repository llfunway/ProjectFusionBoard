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

#define GYRO_SENSITIVITY_250DPS        (131.0f)
#define GYRO_SENSITIVITY_500DPS       (65.5f)
#define GYRO_SENSITIVITY_1000DPS      (32.8f)
#define GYRO_SENSITIVITY_2000DPS      (16.4f)

#define TEMP_SENSITIVITY              (340.0f)
#define TEMP_OFFSET                   (36.53f)
#define MPU6050_ALIGN_STARTUP_DELAY_MS    (200U)
#define MPU6050_ALIGN_DISCARD_COUNT       (32U)
#define MPU6050_ALIGN_SAMPLE_COUNT        (128U)
#define MPU6050_ALIGN_MIN_VALID_SAMPLES   (24U)
#define MPU6050_ALIGN_MAX_ATTEMPTS        (320U)
#define MPU6050_ALIGN_SAMPLE_DELAY_MS     (2U)
#define MPU6050_ALIGN_ACCEL_TOLERANCE_G   (0.12f)
#define MPU6050_ALIGN_GYRO_TOLERANCE_DPS  (3.0f)
#define MPU6050_ACCEL_GATE_MIN_G          (0.85f)
#define MPU6050_ACCEL_GATE_MAX_G          (1.15f)
#define MPU6050_ACCEL_FULL_TRUST_DELTA_G  (0.03f)
#define MPU6050_ACCEL_ZERO_TRUST_DELTA_G  (0.20f)
#define MPU6050_STARTUP_BOOST_MS          (800U)
#define MPU6050_NOMINAL_TWO_KP            (1.0f)
#define MPU6050_NOMINAL_TWO_KI            (0.02f)
#define MPU6050_BOOST_TWO_KP              (6.0f)
#define MPU6050_BOOST_TWO_KI              (0.0f)
#define MPU6050_FALLBACK_SAMPLE_COUNT     (16U)
#define MPU6050_STATIC_ACCEL_TOLERANCE_G  (0.05f)
#define MPU6050_STATIC_GYRO_TOLERANCE_DPS (0.30f)
#define MPU6050_SETTLE_ACCEL_TOLERANCE_G  (0.08f)
#define MPU6050_SETTLE_GYRO_TOLERANCE_DPS (2.0f)
#define MPU6050_SETTLE_TWO_KP             (4.0f)
#define MPU6050_INTEGRAL_DECAY            (0.98f)
#define MPU6050_GYRO_BIAS_ADAPT_ALPHA     (0.001f)
#define MPU6050_ACCEL_NORM_REF_ALPHA      (0.002f)

/* Private Function Prototypes -----------------------------------------------*/
static float MPU6050_GetAccelSensitivity(uint8_t range);
static float MPU6050_GetGyroSensitivity(uint8_t range);
static void MPU6050_MahonyInit(MPU6050_Handle_t *hmpu);
static MPU6050_Status_t MPU6050_InitAttitudeFromAccel(MPU6050_Handle_t *hmpu);
static void MPU6050_SetQuaternionFromRollPitch(MPU6050_Handle_t *hmpu, float roll_rad, float pitch_rad);
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

  HAL_Delay(MPU6050_ALIGN_STARTUP_DELAY_MS);
  if (MPU6050_InitAttitudeFromAccel(hmpu) != MPU6050_OK)
  {
    hmpu->IsInitialized = 0;
    return MPU6050_ERROR_TIMEOUT;
  }

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

  hmpu->RawData.AccelX = (int16_t)(hmpu->RawData.AccelX - hmpu->Calibration.AccelOffsetX);
  hmpu->RawData.AccelY = (int16_t)(hmpu->RawData.AccelY - hmpu->Calibration.AccelOffsetY);
  hmpu->RawData.AccelZ = (int16_t)(hmpu->RawData.AccelZ - hmpu->Calibration.AccelOffsetZ);
  hmpu->RawData.GyroX = (int16_t)(hmpu->RawData.GyroX - hmpu->Calibration.GyroOffsetX);
  hmpu->RawData.GyroY = (int16_t)(hmpu->RawData.GyroY - hmpu->Calibration.GyroOffsetY);
  hmpu->RawData.GyroZ = (int16_t)(hmpu->RawData.GyroZ - hmpu->Calibration.GyroOffsetZ);

  accelScale = MPU6050_GetAccelSensitivity(hmpu->Config.AccelRange);
  gyroScale = MPU6050_GetGyroSensitivity(hmpu->Config.GyroRange);

  hmpu->Data.AccelX = (float)hmpu->RawData.AccelX / accelScale;
  hmpu->Data.AccelY = (float)hmpu->RawData.AccelY / accelScale;
  hmpu->Data.AccelZ = (float)hmpu->RawData.AccelZ / accelScale;
  hmpu->Data.Temp = (float)hmpu->RawData.Temp / TEMP_SENSITIVITY + TEMP_OFFSET;
  hmpu->Data.GyroX = (float)hmpu->RawData.GyroX / gyroScale;
  hmpu->Data.GyroY = (float)hmpu->RawData.GyroY / gyroScale;
  hmpu->Data.GyroZ = (float)hmpu->RawData.GyroZ / gyroScale;

  if (isnan(hmpu->Data.AccelX) || isnan(hmpu->Data.AccelY) || isnan(hmpu->Data.AccelZ) ||
      isnan(hmpu->Data.GyroX) || isnan(hmpu->Data.GyroY) || isnan(hmpu->Data.GyroZ))
  {
    return MPU6050_ERROR_I2C;
  }

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
  float accelWeight = 0.0f;
  float accelDelta;
  float accelNormRatio;
  float correctionTwoKp;
  float qa;
  float qb;
  float qc;
  float sinp;
  uint32_t currentTime;
  float accelNormSq;
  float accelNorm;
  float gyroMagDps;

  if (hmpu == NULL || !hmpu->IsInitialized)
  {
    return MPU6050_ERROR_NOT_INIT;
  }

  /* Detect and recover from NaN/Inf in quaternion (e.g. from bad sensor data).
   * Reset to identity quaternion so attitude can converge again from accelerometer. */
  if (isnan(hmpu->q0) || isinf(hmpu->q0) ||
      isnan(hmpu->q1) || isinf(hmpu->q1) ||
      isnan(hmpu->q2) || isinf(hmpu->q2) ||
      isnan(hmpu->q3) || isinf(hmpu->q3))
  {
    hmpu->q0 = 1.0f;
    hmpu->q1 = 0.0f;
    hmpu->q2 = 0.0f;
    hmpu->q3 = 0.0f;
    hmpu->integralFBx = 0.0f;
    hmpu->integralFBy = 0.0f;
    hmpu->integralFBz = 0.0f;
    hmpu->twoKp = hmpu->nominalTwoKp;
    hmpu->twoKi = hmpu->nominalTwoKi;
    hmpu->startupBoostUntil = 0U;
  }

  currentTime = HAL_GetTick();
  if ((hmpu->startupBoostUntil != 0U) && ((int32_t)(currentTime - hmpu->startupBoostUntil) >= 0))
  {
    hmpu->twoKp = hmpu->nominalTwoKp;
    hmpu->twoKi = hmpu->nominalTwoKi;
    hmpu->startupBoostUntil = 0U;
  }

  hmpu->dt = (float)(currentTime - hmpu->lastTime) / 1000.0f;
  hmpu->lastTime = currentTime;

  if (hmpu->dt <= 0.0f || hmpu->dt > 0.1f)
  {
    hmpu->dt = 0.01f;
  }

  ax = hmpu->Data.AccelX;
  ay = hmpu->Data.AccelY;
  az = hmpu->Data.AccelZ;
  gx = (hmpu->Data.GyroX - hmpu->runtimeGyroBiasX) * ((float)M_PI / 180.0f);
  gy = (hmpu->Data.GyroY - hmpu->runtimeGyroBiasY) * ((float)M_PI / 180.0f);
  gz = (hmpu->Data.GyroZ - hmpu->runtimeGyroBiasZ) * ((float)M_PI / 180.0f);

  /* Guard against NaN/Inf from corrupted I2C reads */
  if (isnan(ax) || isinf(ax)) ax = 0.0f;
  if (isnan(ay) || isinf(ay)) ay = 0.0f;
  if (isnan(az) || isinf(az)) az = 1.0f;
  if (isnan(gx) || isinf(gx)) gx = 0.0f;
  if (isnan(gy) || isinf(gy)) gy = 0.0f;
  if (isnan(gz) || isinf(gz)) gz = 0.0f;

  accelNormSq = (ax * ax) + (ay * ay) + (az * az);
  accelNorm = sqrtf(accelNormSq);
  if (hmpu->accelNormRef <= 0.05f)
  {
    hmpu->accelNormRef = accelNorm;
  }

  accelNormRatio = (hmpu->accelNormRef > 0.05f) ? (accelNorm / hmpu->accelNormRef) : accelNorm;
  accelDelta = fabsf(accelNormRatio - 1.0f);
  gyroMagDps = sqrtf((hmpu->Data.GyroX * hmpu->Data.GyroX) +
                     (hmpu->Data.GyroY * hmpu->Data.GyroY) +
                     (hmpu->Data.GyroZ * hmpu->Data.GyroZ));

  if (accelDelta <= MPU6050_ACCEL_FULL_TRUST_DELTA_G)
  {
    accelWeight = 1.0f;
  }
  else if (accelDelta < MPU6050_ACCEL_ZERO_TRUST_DELTA_G)
  {
    accelWeight = 1.0f - ((accelDelta - MPU6050_ACCEL_FULL_TRUST_DELTA_G) /
                          (MPU6050_ACCEL_ZERO_TRUST_DELTA_G - MPU6050_ACCEL_FULL_TRUST_DELTA_G));
  }

  if ((accelDelta <= MPU6050_STATIC_ACCEL_TOLERANCE_G) &&
      (gyroMagDps <= MPU6050_STATIC_GYRO_TOLERANCE_DPS))
  {
    hmpu->accelNormRef += MPU6050_ACCEL_NORM_REF_ALPHA * (accelNorm - hmpu->accelNormRef);
    hmpu->runtimeGyroBiasX += MPU6050_GYRO_BIAS_ADAPT_ALPHA *
                               (hmpu->Data.GyroX - hmpu->runtimeGyroBiasX);
    hmpu->runtimeGyroBiasY += MPU6050_GYRO_BIAS_ADAPT_ALPHA *
                               (hmpu->Data.GyroY - hmpu->runtimeGyroBiasY);
    hmpu->runtimeGyroBiasZ += MPU6050_GYRO_BIAS_ADAPT_ALPHA *
                               (hmpu->Data.GyroZ - hmpu->runtimeGyroBiasZ);
    gx = (hmpu->Data.GyroX - hmpu->runtimeGyroBiasX) * ((float)M_PI / 180.0f);
    gy = (hmpu->Data.GyroY - hmpu->runtimeGyroBiasY) * ((float)M_PI / 180.0f);
    gz = (hmpu->Data.GyroZ - hmpu->runtimeGyroBiasZ) * ((float)M_PI / 180.0f);
  }

  if ((accelNormSq > 0.0f) &&
      (accelNormRatio >= MPU6050_ACCEL_GATE_MIN_G) &&
      (accelNormRatio <= MPU6050_ACCEL_GATE_MAX_G) &&
      (accelWeight > 0.0f))
  {
    recipNorm = MPU6050_InvSqrt(accelNormSq);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    halfvx = hmpu->q1 * hmpu->q3 - hmpu->q0 * hmpu->q2;
    halfvy = hmpu->q0 * hmpu->q1 + hmpu->q2 * hmpu->q3;
    halfvz = hmpu->q0 * hmpu->q0 - 0.5f + hmpu->q3 * hmpu->q3;

    halfex = (ay * halfvz) - (az * halfvy);
    halfey = (az * halfvx) - (ax * halfvz);
    halfez = (ax * halfvy) - (ay * halfvx);
    halfex *= accelWeight;
    halfey *= accelWeight;
    halfez *= accelWeight;

    if ((hmpu->twoKi > 0.0f) &&
        (accelWeight >= 0.9f) &&
        (accelDelta <= MPU6050_STATIC_ACCEL_TOLERANCE_G) &&
        (gyroMagDps <= MPU6050_STATIC_GYRO_TOLERANCE_DPS))
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
      hmpu->integralFBx *= MPU6050_INTEGRAL_DECAY;
      hmpu->integralFBy *= MPU6050_INTEGRAL_DECAY;
      hmpu->integralFBz *= MPU6050_INTEGRAL_DECAY;
    }

    correctionTwoKp = hmpu->twoKp;
    if ((accelDelta <= MPU6050_SETTLE_ACCEL_TOLERANCE_G) &&
        (gyroMagDps <= MPU6050_SETTLE_GYRO_TOLERANCE_DPS) &&
        (correctionTwoKp < MPU6050_SETTLE_TWO_KP))
    {
      correctionTwoKp = MPU6050_SETTLE_TWO_KP;
    }

    gx += correctionTwoKp * halfex;
    gy += correctionTwoKp * halfey;
    gz += correctionTwoKp * halfez;
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
  hmpu->nominalTwoKp = MPU6050_NOMINAL_TWO_KP;
  hmpu->nominalTwoKi = MPU6050_NOMINAL_TWO_KI;
  hmpu->twoKp = hmpu->nominalTwoKp;
  hmpu->twoKi = hmpu->nominalTwoKi;
  hmpu->integralFBx = 0.0f;
  hmpu->integralFBy = 0.0f;
  hmpu->integralFBz = 0.0f;
  hmpu->dt = 0.0f;
  hmpu->lastTime = HAL_GetTick();
  hmpu->startupBoostUntil = 0U;
  hmpu->runtimeGyroBiasX = 0.0f;
  hmpu->runtimeGyroBiasY = 0.0f;
  hmpu->runtimeGyroBiasZ = 0.0f;
  hmpu->accelNormRef = 1.0f;
  hmpu->Attitude.Roll = 0.0f;
  hmpu->Attitude.Pitch = 0.0f;
  hmpu->Attitude.Timestamp = 0;
}

static MPU6050_Status_t MPU6050_InitAttitudeFromAccel(MPU6050_Handle_t *hmpu)
{
  float ax_sum = 0.0f;
  float ay_sum = 0.0f;
  float az_sum = 0.0f;
  int32_t gx_sum = 0;
  int32_t gy_sum = 0;
  int32_t gz_sum = 0;
  float ax;
  float ay;
  float az;
  float norm;
  float roll_rad;
  float pitch_rad;
  float gyro_mag;
  uint16_t valid_count = 0;
  uint16_t sample_count = 0;

  for (uint16_t i = 0; i < MPU6050_ALIGN_DISCARD_COUNT; i++)
  {
    if (MPU6050_ReadRawData(hmpu) != MPU6050_OK)
    {
      return MPU6050_ERROR_I2C;
    }
    HAL_Delay(MPU6050_ALIGN_SAMPLE_DELAY_MS);
  }

  for (uint16_t attempt = 0; attempt < MPU6050_ALIGN_MAX_ATTEMPTS; attempt++)
  {
    if (MPU6050_ReadRawData(hmpu) != MPU6050_OK)
    {
      return MPU6050_ERROR_I2C;
    }

    norm = sqrtf((hmpu->Data.AccelX * hmpu->Data.AccelX) +
                 (hmpu->Data.AccelY * hmpu->Data.AccelY) +
                 (hmpu->Data.AccelZ * hmpu->Data.AccelZ));
    gyro_mag = sqrtf((hmpu->Data.GyroX * hmpu->Data.GyroX) +
                     (hmpu->Data.GyroY * hmpu->Data.GyroY) +
                     (hmpu->Data.GyroZ * hmpu->Data.GyroZ));

    if ((fabsf(norm - 1.0f) <= MPU6050_ALIGN_ACCEL_TOLERANCE_G) &&
        (gyro_mag <= MPU6050_ALIGN_GYRO_TOLERANCE_DPS))
    {
      ax_sum += hmpu->Data.AccelX;
      ay_sum += hmpu->Data.AccelY;
      az_sum += hmpu->Data.AccelZ;
      gx_sum += hmpu->RawData.GyroX;
      gy_sum += hmpu->RawData.GyroY;
      gz_sum += hmpu->RawData.GyroZ;
      valid_count++;

      if (valid_count >= MPU6050_ALIGN_SAMPLE_COUNT)
      {
        break;
      }
    }
    else
    {
      ax_sum = 0.0f;
      ay_sum = 0.0f;
      az_sum = 0.0f;
      gx_sum = 0;
      gy_sum = 0;
      gz_sum = 0;
      valid_count = 0;
    }

    HAL_Delay(MPU6050_ALIGN_SAMPLE_DELAY_MS);
  }

  if (valid_count >= MPU6050_ALIGN_MIN_VALID_SAMPLES)
  {
    sample_count = valid_count;
  }
  else
  {
    ax_sum = 0.0f;
    ay_sum = 0.0f;
    az_sum = 0.0f;
    gx_sum = 0;
    gy_sum = 0;
    gz_sum = 0;

    for (uint16_t i = 0; i < MPU6050_FALLBACK_SAMPLE_COUNT; i++)
    {
      if (MPU6050_ReadRawData(hmpu) != MPU6050_OK)
      {
        return MPU6050_ERROR_I2C;
      }

      ax_sum += hmpu->Data.AccelX;
      ay_sum += hmpu->Data.AccelY;
      az_sum += hmpu->Data.AccelZ;
      gx_sum += hmpu->RawData.GyroX;
      gy_sum += hmpu->RawData.GyroY;
      gz_sum += hmpu->RawData.GyroZ;
      HAL_Delay(MPU6050_ALIGN_SAMPLE_DELAY_MS);
    }

    sample_count = MPU6050_FALLBACK_SAMPLE_COUNT;
  }

  ax = ax_sum / (float)sample_count;
  ay = ay_sum / (float)sample_count;
  az = az_sum / (float)sample_count;

  norm = sqrtf((ax * ax) + (ay * ay) + (az * az));
  if (norm <= 0.0f)
  {
    return MPU6050_ERROR_PARAM;
  }

  ax /= norm;
  ay /= norm;
  az /= norm;

  roll_rad = atan2f(ay, az);
  pitch_rad = atan2f(-ax, sqrtf((ay * ay) + (az * az)));

  MPU6050_SetQuaternionFromRollPitch(hmpu, roll_rad, pitch_rad);

  hmpu->Attitude.Roll = roll_rad * (180.0f / (float)M_PI);
  hmpu->Attitude.Pitch = pitch_rad * (180.0f / (float)M_PI);
  hmpu->accelNormRef = norm;
  hmpu->Attitude.Timestamp = HAL_GetTick();
  hmpu->Calibration.GyroOffsetX = (int16_t)(gx_sum / (int32_t)sample_count);
  hmpu->Calibration.GyroOffsetY = (int16_t)(gy_sum / (int32_t)sample_count);
  hmpu->Calibration.GyroOffsetZ = (int16_t)(gz_sum / (int32_t)sample_count);
  hmpu->integralFBx = 0.0f;
  hmpu->integralFBy = 0.0f;
  hmpu->integralFBz = 0.0f;
  hmpu->lastTime = HAL_GetTick();
  hmpu->twoKp = MPU6050_BOOST_TWO_KP;
  hmpu->twoKi = MPU6050_BOOST_TWO_KI;
  hmpu->startupBoostUntil = hmpu->lastTime + MPU6050_STARTUP_BOOST_MS;

  return MPU6050_OK;
}

static void MPU6050_SetQuaternionFromRollPitch(MPU6050_Handle_t *hmpu, float roll_rad, float pitch_rad)
{
  const float half_roll = 0.5f * roll_rad;
  const float half_pitch = 0.5f * pitch_rad;
  const float cr = cosf(half_roll);
  const float sr = sinf(half_roll);
  const float cp = cosf(half_pitch);
  const float sp = sinf(half_pitch);

  hmpu->q0 = cr * cp;
  hmpu->q1 = sr * cp;
  hmpu->q2 = cr * sp;
  hmpu->q3 = -sr * sp;
}

static float MPU6050_InvSqrt(float x)
{
  return 1.0f / sqrtf(x);
}
