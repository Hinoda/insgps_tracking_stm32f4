/*
 * File: power.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 07-Nov-2017 15:42:26
 */

/* Include Files */
#include "Cbn_31.h"
#include "insgps_v4_0.h"
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
