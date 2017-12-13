/*
 * File: normC.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 07-Dec-2017 23:15:41
 */

/* Include Files */
#include "Cbn_31.h"
#include "deg2utm.h"
#include "insgps_v8_0.h"
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
 * Arguments    : double C[9]
 *                double Cbn[9]
 * Return Type  : void
 */
void normC(double C[9], double Cbn[9])
{
  double dv0[9];
  int k;
  double x[3];
  double y;
  int i0;

  /* % METHOD web */
  skew_mat3(*(double (*)[3])&C[3], dv0);
  for (k = 0; k < 3; k++) {
    C[k] = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      C[k] += dv0[k + 3 * i0] * C[6 + i0];
    }
  }

  power(*(double (*)[3])&C[0], x);
  y = x[0];
  for (k = 0; k < 2; k++) {
    y += x[k + 1];
  }

  y = sqrt(y);
  for (k = 0; k < 3; k++) {
    C[k] /= y;
  }

  skew_mat3(*(double (*)[3])&C[6], dv0);
  for (k = 0; k < 3; k++) {
    C[3 + k] = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      C[3 + k] += dv0[k + 3 * i0] * C[i0];
    }
  }

  power(*(double (*)[3])&C[3], x);
  y = x[0];
  for (k = 0; k < 2; k++) {
    y += x[k + 1];
  }

  y = sqrt(y);
  for (k = 0; k < 3; k++) {
    C[3 + k] /= y;
  }

  power(*(double (*)[3])&C[6], x);
  y = x[0];
  for (k = 0; k < 2; k++) {
    y += x[k + 1];
  }

  y = sqrt(y);
  for (k = 0; k < 3; k++) {
    C[6 + k] /= y;
  }

  /*  C(3,:)=C(3,:)/(sqrt(sum(C(3,:).^2))); */
  memcpy(&Cbn[0], &C[0], 9U * sizeof(double));
}

/*
 * File trailer for normC.c
 *
 * [EOF]
 */
