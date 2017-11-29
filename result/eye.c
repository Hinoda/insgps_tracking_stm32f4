/*
 * File: eye.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 12-Nov-2017 15:05:52
 */

/* Include Files */
#include "Cbn_31.h"
#include "insgps_v6_0.h"
#include "normC.h"
#include "skew_mat3.h"
#include "eye.h"

/* Function Definitions */

/*
 * Arguments    : float I[225]
 * Return Type  : void
 */
void b_eye(float* I)
{
  int k;
  memset(I, 0, 225U * sizeof(float));
  for (k = 0; k < 15; k++) {
    *(I+k+15*k) = 1.0;
  }
}

/*
 * Arguments    : float I[9]
 * Return Type  : void
 */
void eye(float* I)
{
  int k;
  memset(I, 0, 9U * sizeof(float));
  for (k = 0; k < 3; k++) {
    *(I+k+3*k) = 1.0;
  }
}

/*
 * File trailer for eye.c
 *
 * [EOF]
 */
