#ifndef MAG_RESIDUAL_FILTER_H
#define MAG_RESIDUAL_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float trend_nT;
  float weights[3];
  float rls_p[3][3];
  float rate_state[2];
  float rate_p[2][2];
  float corrected_nT;
  float correction_nT;
  float residual_nT;
  float rate_nT_per_s;
  unsigned char initialized;
} MagResidualFilter_t;

void MagResidualFilter_Init(MagResidualFilter_t *filter);
float MagResidualFilter_Update(MagResidualFilter_t *filter,
                               const float mag_nT[3],
                               const float vertical_unit[3],
                               float vertical_raw_nT);

#ifdef __cplusplus
}
#endif

#endif /* MAG_RESIDUAL_FILTER_H */
