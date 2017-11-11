/*
 * File: insgps_v5_1.h
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 09-Nov-2017 10:12:01
 */

#ifndef ARM_MATH_CM4
#define ARM_MATH_CM4
#endif

#ifndef INSGPS_V5_1_H
#define INSGPS_V5_1_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "Cbn_31_types.h"
//#include "arm_math.h"

/* Function Declarations */
void insgps_v5_1(const float zI[10], const float zG[7], int gpsflag, float
                 dt, float g0, float a, float e, float we, const float Q
                 [144], const float R[36], float PVA[10], float bias[6],
                 float Pk_1[225], float xk_1[15]);

#endif

/*
 * File trailer for insgps_v5_1.h
 *
 * [EOF]
 */
