/*
 * File: skew_mat3.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 12-Nov-2017 15:05:52
 */

/* Include Files */
#include "Cbn_31.h"
#include "insgps_v6_0.h"
#include "normC.h"
#include "skew_mat3.h"

/* Function Definitions */

/*
 * Arguments    : const float A[3]
 *                float ret[9]
 * Return Type  : void
 */
void skew_mat3(const float A[3], float ret[9])
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
