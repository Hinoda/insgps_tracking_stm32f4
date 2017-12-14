/*
 * File: diag.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 13-Dec-2017 10:47:33
 */

/* Include Files */
#include "Cbn_31.h"
#include "GramSchmidt.h"
#include "deg2utm.h"
#include "insgps_v11.h"
#include "skew_mat3.h"
#include "diag.h"

/* Function Definitions */

/*
 * Arguments    : const double v[2]
 *                double d[4]
 * Return Type  : void
 */
void b_diag(const double v[2], double d[4])
{
  int j;
  for (j = 0; j < 4; j++) {
    d[j] = 0.0;
  }

  for (j = 0; j < 2; j++) {
    d[j + (j << 1)] = v[j];
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
