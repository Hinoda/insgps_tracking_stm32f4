/*
 * File: power.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 16-Oct-2017 08:31:21
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "Cbn_31.h"
#include "insgps_v1_0.h"
#include "normC.h"
#include "skew_mat3.h"
#include "power.h"

/* Function Definitions */

/*
 * Arguments    : const double a[3]
 *                double y[3]
 * Return Type  : void
 */
void power(const double a[3], double y[3])
{
  int k;
  for (k = 0; k < 3; k++) {
    y[k] = a[k] * a[k];
  }
}

/*
 * File trailer for power.c
 *
 * [EOF]
 */
