#include "mag_residual_filter.h"

#include <math.h>
#include <stddef.h>

#define MAG_FILTER_SAMPLE_DT_S          (0.0200f)
#define MAG_FEATURE_SCALE_NT            (50000.0f)
#define MAG_TREND_CUTOFF_HZ             (0.2000f)
#define MAG_RLS_LAMBDA                  (0.9990f)
#define MAG_RLS_INITIAL_P               (20.0f)
#define MAG_RLS_MAX_WEIGHT_NT           (2500.0f)
#define MAG_RATE_PROCESS_ACCEL_NT_S2    (900.0f)
#define MAG_RATE_MEAS_NOISE_NT          (180.0f)
#define MAG_RATE_INITIAL_RATE_VAR       (1000000.0f)
#define MAG_RATE_LEAD_S                 (0.1200f)

#ifndef M_PI
#define M_PI                            (3.14159265358979323846)
#endif

static float MagResidualFilter_Clamp(float value, float min_value, float max_value)
{
  if (value > max_value)
  {
    return max_value;
  }

  if (value < min_value)
  {
    return min_value;
  }

  return value;
}

static float MagResidualFilter_LowpassAlpha(float cutoff_hz)
{
  const float tau = 1.0f / (2.0f * (float)M_PI * cutoff_hz);
  return MAG_FILTER_SAMPLE_DT_S / (tau + MAG_FILTER_SAMPLE_DT_S);
}

static void MagResidualFilter_ResetMatrix3(float matrix[3][3], float diagonal)
{
  unsigned char row;
  unsigned char col;

  for (row = 0U; row < 3U; row++)
  {
    for (col = 0U; col < 3U; col++)
    {
      matrix[row][col] = (row == col) ? diagonal : 0.0f;
    }
  }
}

static void MagResidualFilter_InitRateState(MagResidualFilter_t *filter,
                                            float initial_nT)
{
  filter->rate_state[0] = initial_nT;
  filter->rate_state[1] = 0.0f;
  filter->rate_p[0][0] = MAG_RATE_MEAS_NOISE_NT * MAG_RATE_MEAS_NOISE_NT;
  filter->rate_p[0][1] = 0.0f;
  filter->rate_p[1][0] = 0.0f;
  filter->rate_p[1][1] = MAG_RATE_INITIAL_RATE_VAR;
}

void MagResidualFilter_Init(MagResidualFilter_t *filter)
{
  if (filter == NULL)
  {
    return;
  }

  filter->trend_nT = 0.0f;
  filter->weights[0] = 0.0f;
  filter->weights[1] = 0.0f;
  filter->weights[2] = 0.0f;
  MagResidualFilter_ResetMatrix3(filter->rls_p, MAG_RLS_INITIAL_P);
  filter->rate_state[0] = 0.0f;
  filter->rate_state[1] = 0.0f;
  filter->rate_p[0][0] = 0.0f;
  filter->rate_p[0][1] = 0.0f;
  filter->rate_p[1][0] = 0.0f;
  filter->rate_p[1][1] = 0.0f;
  filter->corrected_nT = 0.0f;
  filter->correction_nT = 0.0f;
  filter->residual_nT = 0.0f;
  filter->rate_nT_per_s = 0.0f;
  filter->initialized = 0U;
}

static void MagResidualFilter_UpdateRls(MagResidualFilter_t *filter,
                                        const float feature[3],
                                        float target_residual_nT)
{
  float px[3];
  float gain[3];
  float predicted;
  float error;
  float denom;
  float xp[3];
  unsigned char row;
  unsigned char col;

  px[0] = (filter->rls_p[0][0] * feature[0]) +
          (filter->rls_p[0][1] * feature[1]) +
          (filter->rls_p[0][2] * feature[2]);
  px[1] = (filter->rls_p[1][0] * feature[0]) +
          (filter->rls_p[1][1] * feature[1]) +
          (filter->rls_p[1][2] * feature[2]);
  px[2] = (filter->rls_p[2][0] * feature[0]) +
          (filter->rls_p[2][1] * feature[1]) +
          (filter->rls_p[2][2] * feature[2]);

  denom = MAG_RLS_LAMBDA +
          (feature[0] * px[0]) +
          (feature[1] * px[1]) +
          (feature[2] * px[2]);

  if (denom < 0.0001f)
  {
    return;
  }

  predicted = (filter->weights[0] * feature[0]) +
              (filter->weights[1] * feature[1]) +
              (filter->weights[2] * feature[2]);
  error = target_residual_nT - predicted;

  for (row = 0U; row < 3U; row++)
  {
    gain[row] = px[row] / denom;
    filter->weights[row] += gain[row] * error;
    filter->weights[row] = MagResidualFilter_Clamp(filter->weights[row],
                                                   -MAG_RLS_MAX_WEIGHT_NT,
                                                   MAG_RLS_MAX_WEIGHT_NT);
  }

  for (col = 0U; col < 3U; col++)
  {
    xp[col] = (feature[0] * filter->rls_p[0][col]) +
              (feature[1] * filter->rls_p[1][col]) +
              (feature[2] * filter->rls_p[2][col]);
  }

  for (row = 0U; row < 3U; row++)
  {
    for (col = 0U; col < 3U; col++)
    {
      filter->rls_p[row][col] =
          (filter->rls_p[row][col] - (gain[row] * xp[col])) / MAG_RLS_LAMBDA;
    }
  }
}

static float MagResidualFilter_UpdateRateKalman(MagResidualFilter_t *filter,
                                                float measurement_nT)
{
  const float dt = MAG_FILTER_SAMPLE_DT_S;
  const float q = MAG_RATE_PROCESS_ACCEL_NT_S2 * MAG_RATE_PROCESS_ACCEL_NT_S2;
  const float r = MAG_RATE_MEAS_NOISE_NT * MAG_RATE_MEAS_NOISE_NT;
  const float q00 = 0.25f * q * dt * dt * dt * dt;
  const float q01 = 0.5f * q * dt * dt * dt;
  const float q11 = q * dt * dt;
  float x0;
  float x1;
  float p00;
  float p01;
  float p10;
  float p11;
  float innovation;
  float s;
  float k0;
  float k1;

  x0 = filter->rate_state[0] + (dt * filter->rate_state[1]);
  x1 = filter->rate_state[1];

  p00 = filter->rate_p[0][0] +
        (dt * (filter->rate_p[1][0] + filter->rate_p[0][1])) +
        (dt * dt * filter->rate_p[1][1]) + q00;
  p01 = filter->rate_p[0][1] + (dt * filter->rate_p[1][1]) + q01;
  p10 = filter->rate_p[1][0] + (dt * filter->rate_p[1][1]) + q01;
  p11 = filter->rate_p[1][1] + q11;

  innovation = measurement_nT - x0;
  s = p00 + r;
  if (s < 1.0f)
  {
    s = 1.0f;
  }

  k0 = p00 / s;
  k1 = p10 / s;

  filter->rate_state[0] = x0 + (k0 * innovation);
  filter->rate_state[1] = x1 + (k1 * innovation);

  filter->rate_p[0][0] = (1.0f - k0) * p00;
  filter->rate_p[0][1] = (1.0f - k0) * p01;
  filter->rate_p[1][0] = p10 - (k1 * p00);
  filter->rate_p[1][1] = p11 - (k1 * p01);
  filter->rate_nT_per_s = filter->rate_state[1];

  return filter->rate_state[0] + (MAG_RATE_LEAD_S * filter->rate_state[1]);
}

float MagResidualFilter_Update(MagResidualFilter_t *filter,
                               const float mag_nT[3],
                               const float vertical_unit[3],
                               float vertical_raw_nT)
{
  const float trend_alpha = MagResidualFilter_LowpassAlpha(MAG_TREND_CUTOFF_HZ);
  float horizontal[3];
  float feature[3];
  float target_residual;
  float rls_output;
  float rate_output;
  unsigned char i;

  if ((filter == NULL) || (mag_nT == NULL) || (vertical_unit == NULL))
  {
    return vertical_raw_nT;
  }

  if (isnan(vertical_raw_nT) || isinf(vertical_raw_nT))
  {
    vertical_raw_nT = 0.0f;
  }

  if (filter->initialized == 0U)
  {
    filter->trend_nT = vertical_raw_nT;
    filter->corrected_nT = vertical_raw_nT;
    filter->correction_nT = 0.0f;
    filter->residual_nT = 0.0f;
    MagResidualFilter_InitRateState(filter, vertical_raw_nT);
    filter->initialized = 1U;
    return vertical_raw_nT;
  }

  for (i = 0U; i < 3U; i++)
  {
    horizontal[i] = mag_nT[i] - (vertical_raw_nT * vertical_unit[i]);
    feature[i] = horizontal[i] / MAG_FEATURE_SCALE_NT;
  }

  filter->trend_nT += trend_alpha * (vertical_raw_nT - filter->trend_nT);
  target_residual = vertical_raw_nT - filter->trend_nT;

  MagResidualFilter_UpdateRls(filter, feature, target_residual);

  filter->correction_nT = (filter->weights[0] * feature[0]) +
                          (filter->weights[1] * feature[1]) +
                          (filter->weights[2] * feature[2]);
  rls_output = vertical_raw_nT - filter->correction_nT;
  rate_output = MagResidualFilter_UpdateRateKalman(filter, rls_output);
  filter->corrected_nT = rate_output;
  filter->residual_nT = vertical_raw_nT - filter->corrected_nT;

  if (isnan(filter->corrected_nT) || isinf(filter->corrected_nT))
  {
    MagResidualFilter_Init(filter);
    return vertical_raw_nT;
  }

  return filter->corrected_nT;
}
