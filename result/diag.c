/*
 * File: diag.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 09-Nov-2017 10:12:01
 */

/* Include Files */
#include "Cbn_31.h"
#include "insgps_v5_1.h"
#include "normC.h"
#include "skew_mat3.h"
#include "diag.h"

/* Function Definitions */

/*
 * Arguments    : const double v[3]
 *                double d[9]
 * Return Type  : void
 */
void b_diag(const double v[3], double d[9])
{
  int j;
  memset(&d[0], 0, 9U * sizeof(double));
  for (j = 0; j < 3; j++) {
    d[j + 3 * j] = v[j];
  }
}

/*
 * Arguments    : const double v[6]
 *                double d[36]
 * Return Type  : void
 */
void diag(const double v[6], double d[36])
{
  int j;
  memset(&d[0], 0, 36U * sizeof(double));
  for (j = 0; j < 6; j++) {
    d[j + 6 * j] = v[j];
  }
}

/*
 * File trailer for diag.c
 *
 * [EOF]
 */
