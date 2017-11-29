/*
 * File: power.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 12-Nov-2017 15:05:52
 */

/* Include Files */
#include "Cbn_31.h"
#include "insgps_v6_0.h"
#include "normC.h"
#include "skew_mat3.h"
#include "power.h"

/* Function Definitions */

/*
 * Arguments    : const float a[3]
 *                float y[3]
 * Return Type  : void
 */
void b_power(float* a, float* y)
{
  int k;
  for (k = 0; k < 3; k++) {
    *(y+k) = *(a+k) * (*(a+k));
  }
}

/*
 * Arguments    : const float a[36]
 *                float y[36]
 * Return Type  : void
 */
void power(float* a, float* y)
{
  int k;
  for (k = 0; k < 36; k++) {
    *(y+k) = *(a+k) * (*(a+k));
  }
}

/*
 * File trailer for power.c
 *
 * [EOF]
 */
