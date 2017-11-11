/*
 * File: Cbn_31.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 09-Nov-2017 10:12:01
 */

/* Include Files */
#include "Cbn_31.h"
#include "insgps_v5_1.h"
#include "normC.h"
#include "skew_mat3.h"

/* Function Definitions */

/*
 * matran xoay DCM Cbn
 *  euler=3*1;
 * Arguments    : const float euler[3]
 *                float Cbn[9]
 * Return Type  : void
 */
void Cbn_31(const float euler[3], float Cbn[9])
{
  /* Ci1 */
  /* Ci2 */
  /* Ci3 */
  Cbn[0] = cos(euler[1]) * cos(euler[2]);
  Cbn[1] = cos(euler[1]) * sin(euler[2]);
  Cbn[2] = -sin(euler[1]);
  Cbn[3] = -cos(euler[0]) * sin(euler[2]) + sin(euler[0]) * sin(euler[1]) * cos
    (euler[2]);
  Cbn[4] = cos(euler[0]) * cos(euler[2]) + sin(euler[0]) * sin(euler[1]) * sin
    (euler[2]);
  Cbn[5] = sin(euler[0]) * cos(euler[1]);
  Cbn[6] = sin(euler[0]) * sin(euler[2]) + cos(euler[0]) * sin(euler[1]) * cos
    (euler[2]);
  Cbn[7] = -sin(euler[0]) * cos(euler[2]) + cos(euler[0]) * sin(euler[1]) * sin
    (euler[2]);
  Cbn[8] = cos(euler[0]) * cos(euler[1]);
}

/*
 * File trailer for Cbn_31.c
 *
 * [EOF]
 */
