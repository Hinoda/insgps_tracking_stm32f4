/*
 * File: diag.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 12-Nov-2017 15:05:52
 */

/* Include Files */
#include "Cbn_31.h"
#include "insgps_v6_0.h"
#include "normC.h"
#include "skew_mat3.h"
#include "diag.h"

/* Function Definitions */

/*
 * Arguments    : const float v[2]
 *                float d[4]
 * Return Type  : void
 */
void b_diag(float* v, float* d)
{
  int j;
  for (j = 0; j < 4; j++) {
    *(d+j) = 0.0;
  }

  for (j = 0; j < 2; j++) {
    *(d+j+(j<<1)) = *(v+j);
  }
}

/*
 * Arguments    : const float v[6]
 *                float d[36]
 * Return Type  : void
 */
void diag(float* v, float* d)
{
  int j;
  memset(d, 0, 36U * sizeof(float));
  for (j = 0; j < 6; j++) {
    *(d+j+6*j) = *(v+j);
  }
}

/*
 * File trailer for diag.c
 *
 * [EOF]
 */
