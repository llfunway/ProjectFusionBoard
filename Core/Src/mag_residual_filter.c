#include "mag_residual_filter.h"

#include <math.h>
#include <stddef.h>

#define MAG_RESIDUAL_BASELINE_ALPHA       (0.0020f)
#define MAG_RESIDUAL_ADAPT_MU             (0.0300f)
#define MAG_RESIDUAL_WEIGHT_DECAY         (0.0002f)
#define MAG_RESIDUAL_MIN_HORIZONTAL_NT    (1000.0f)
#define MAG_RESIDUAL_MAX_ERROR_NT         (20000.0f)
#define MAG_RESIDUAL_MAX_WEIGHT           (0.2500f)
#define MAG_RESIDUAL_NORM_FLOOR           (1000000.0f)

static float MagResidualFilter_Clamp(float value, float limit)
{
  if (value > limit)
  {
    return limit;
  }

  if (value < -limit)
  {
    return -limit;
  }

  return value;
}

void MagResidualFilter_Init(MagResidualFilter_t *filter)
{
  if (filter == NULL)
  {
    return;
  }

  filter->baseline_nT = 0.0f;
  filter->weights[0] = 0.0f;
  filter->weights[1] = 0.0f;
  filter->weights[2] = 0.0f;
  filter->corrected_nT = 0.0f;
  filter->correction_nT = 0.0f;
  filter->residual_nT = 0.0f;
  filter->initialized = 0U;
}

float MagResidualFilter_Update(MagResidualFilter_t *filter,
                               const float mag_nT[3],
                               const float vertical_unit[3],
                               float vertical_raw_nT)
{
  float horizontal[3];
  float horizontal_norm_sq;
  float error;
  float adapt_gain;
  unsigned char i;

  if ((filter == NULL) || (mag_nT == NULL) || (vertical_unit == NULL))
  {
    return vertical_raw_nT;
  }

  if (filter->initialized == 0U)
  {
    filter->baseline_nT = vertical_raw_nT;
    filter->corrected_nT = vertical_raw_nT;
    filter->correction_nT = 0.0f;
    filter->residual_nT = 0.0f;
    filter->initialized = 1U;
    return vertical_raw_nT;
  }

  for (i = 0U; i < 3U; i++)
  {
    horizontal[i] = mag_nT[i] - (vertical_raw_nT * vertical_unit[i]);
  }

  horizontal_norm_sq = (horizontal[0] * horizontal[0]) +
                       (horizontal[1] * horizontal[1]) +
                       (horizontal[2] * horizontal[2]);

  filter->correction_nT = (filter->weights[0] * horizontal[0]) +
                          (filter->weights[1] * horizontal[1]) +
                          (filter->weights[2] * horizontal[2]);
  filter->corrected_nT = vertical_raw_nT - filter->correction_nT;
  error = filter->corrected_nT - filter->baseline_nT;
  filter->residual_nT = error;

  if ((horizontal_norm_sq > (MAG_RESIDUAL_MIN_HORIZONTAL_NT * MAG_RESIDUAL_MIN_HORIZONTAL_NT)) &&
      (fabsf(error) < MAG_RESIDUAL_MAX_ERROR_NT))
  {
    adapt_gain = MAG_RESIDUAL_ADAPT_MU * error /
                 (horizontal_norm_sq + MAG_RESIDUAL_NORM_FLOOR);

    for (i = 0U; i < 3U; i++)
    {
      filter->weights[i] *= (1.0f - MAG_RESIDUAL_WEIGHT_DECAY);
      filter->weights[i] += adapt_gain * horizontal[i];
      filter->weights[i] = MagResidualFilter_Clamp(filter->weights[i],
                                                   MAG_RESIDUAL_MAX_WEIGHT);
    }
  }

  filter->baseline_nT += MAG_RESIDUAL_BASELINE_ALPHA *
                         (filter->corrected_nT - filter->baseline_nT);

  return filter->corrected_nT;
}
