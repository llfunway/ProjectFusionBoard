#include "mag_residual_filter.h"

#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int test_turning_residual_is_reduced(void)
{
  MagResidualFilter_t filter;
  const float vertical_unit[3] = {0.0f, 0.0f, 1.0f};
  float raw_min = 1000000.0f;
  float raw_max = -1000000.0f;
  float corr_min = 1000000.0f;
  float corr_max = -1000000.0f;

  MagResidualFilter_Init(&filter);

  for (int i = 0; i < 2500; i++)
  {
    const float phase = (float)i * 0.035f;
    float mag[3];
    float residual;
    float raw_vertical;
    float corrected;

    mag[0] = 30000.0f * cosf(phase);
    mag[1] = 30000.0f * sinf(phase);
    residual = (0.040f * mag[0]) - (0.020f * mag[1]);
    mag[2] = 48000.0f + residual;
    raw_vertical = mag[2];

    corrected = MagResidualFilter_Update(&filter, mag, vertical_unit, raw_vertical);

    if (i > 1200)
    {
      if (raw_vertical < raw_min)
      {
        raw_min = raw_vertical;
      }
      if (raw_vertical > raw_max)
      {
        raw_max = raw_vertical;
      }
      if (corrected < corr_min)
      {
        corr_min = corrected;
      }
      if (corrected > corr_max)
      {
        corr_max = corrected;
      }
    }
  }

  return ((raw_max - raw_min) > 2500.0f) &&
         ((corr_max - corr_min) < ((raw_max - raw_min) * 0.35f));
}

int main(void)
{
  if (!test_turning_residual_is_reduced())
  {
    printf("FAIL: turning residual was not reduced enough\n");
    return 1;
  }

  printf("PASS: turning residual reduced\n");
  return 0;
}
