/*
 * File: eye.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 07-Nov-2017 15:42:26
 */

/* Include Files */
#include "Cbn_31.h"
#include "insgps_v4_0.h"
#include "normC.h"
#include "skew_mat3.h"
#include "eye.h"

/* Function Definitions */

/*
 * Arguments    : double I[225]
 * Return Type  : void
 */
void b_eye(double I[225])
{
  int k;
  memset(&I[0], 0, 225U * sizeof(double));
  for (k = 0; k < 15; k++) {
    I[k + 15 * k] = 1.0;
  }
}

/*
 * Arguments    : double I[9]
 * Return Type  : void
 */
void eye(double I[9])
{
  int k;
  memset(&I[0], 0, 9U * sizeof(double));
  for (k = 0; k < 3; k++) {
    I[k + 3 * k] = 1.0;
  }
}

/*
 * File trailer for eye.c
 *
 * [EOF]
 */
