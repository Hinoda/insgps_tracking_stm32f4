/*
 * File: skew_mat3.c
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

/* Function Definitions */

/*
 * Arguments    : const double A[3]
 *                double ret[9]
 * Return Type  : void
 */
void skew_mat3(const double A[3], double ret[9])
{
  ret[0] = 0.0;
  ret[3] = -A[2];
  ret[6] = A[1];
  ret[1] = A[2];
  ret[4] = 0.0;
  ret[7] = -A[0];
  ret[2] = -A[1];
  ret[5] = A[0];
  ret[8] = 0.0;
}

/*
 * File trailer for skew_mat3.c
 *
 * [EOF]
 */
