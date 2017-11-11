/*
 * File: normC.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 09-Nov-2017 10:12:01
 */

/* Include Files */
#include "Cbn_31.h"
#include "insgps_v5_1.h"
#include "normC.h"
#include "skew_mat3.h"
#include "power.h"

/* Function Definitions */

/*
 * % Desciption
 *
 * @param  :[in] Cbn
 * @return :     ortho-nomalized Cbn
 * @bug    :     make angle unsteady everytime GPS
 * TODO    :     quaternion or ortho-normalized
 * Arguments    : float C[9]
 *                float Cbn[9]
 * Return Type  : void
 */
void normC(float C[9], float Cbn[9])
{
  float dv0[9];
  int k;
  float x[3];
  float y;
  int i0;

  /* % METHOD web */
  skew_mat3(*(float (*)[3])&C[3], dv0);
  for (k = 0; k < 3; k++) {
    C[k] = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      C[k] += dv0[k + 3 * i0] * C[6 + i0];
    }
  }

  power(*(float (*)[3])&C[0], x);
  y = x[0];
  for (k = 0; k < 2; k++) {
    y += x[k + 1];
  }

  y = sqrt(y);
  for (k = 0; k < 3; k++) {
    C[k] /= y;
  }

  skew_mat3(*(float (*)[3])&C[6], dv0);
  for (k = 0; k < 3; k++) {
    C[3 + k] = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      C[3 + k] += dv0[k + 3 * i0] * C[i0];
    }
  }

  power(*(float (*)[3])&C[3], x);
  y = x[0];
  for (k = 0; k < 2; k++) {
    y += x[k + 1];
  }

  y = sqrt(y);
  for (k = 0; k < 3; k++) {
    C[3 + k] /= y;
  }

  power(*(float (*)[3])&C[6], x);
  y = x[0];
  for (k = 0; k < 2; k++) {
    y += x[k + 1];
  }

  y = sqrt(y);
  for (k = 0; k < 3; k++) {
    C[6 + k] /= y;
  }

  memcpy(&Cbn[0], &C[0], 9U * sizeof(float));
}

/*
 * File trailer for normC.c
 *
 * [EOF]
 */
