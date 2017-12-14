/*
 * File: Cbn_31.c
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
 * matran xoay DCM Cbn
 *  euler=3*1;
 * Arguments    : const double b_euler[3]
 *                double Cbn[9]
 * Return Type  : void
 */
void Cbn_31(const double b_euler[3], double Cbn[9])
{
  /* Ci1 */
  /* Ci2 */
  /* Ci3 */
  Cbn[0] = cos(b_euler[1]) * cos(b_euler[2]);
  Cbn[1] = cos(b_euler[1]) * sin(b_euler[2]);
  Cbn[2] = -sin(b_euler[1]);
  Cbn[3] = -cos(b_euler[0]) * sin(b_euler[2]) + sin(b_euler[0]) * sin(b_euler[1])
    * cos(b_euler[2]);
  Cbn[4] = cos(b_euler[0]) * cos(b_euler[2]) + sin(b_euler[0]) * sin(b_euler[1])
    * sin(b_euler[2]);
  Cbn[5] = sin(b_euler[0]) * cos(b_euler[1]);
  Cbn[6] = sin(b_euler[0]) * sin(b_euler[2]) + cos(b_euler[0]) * sin(b_euler[1])
    * cos(b_euler[2]);
  Cbn[7] = -sin(b_euler[0]) * cos(b_euler[2]) + cos(b_euler[0]) * sin(b_euler[1])
    * sin(b_euler[2]);
  Cbn[8] = cos(b_euler[0]) * cos(b_euler[1]);
}

/*
 * File trailer for Cbn_31.c
 *
 * [EOF]
 */
