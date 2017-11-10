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
#include "arm_math.h"

/* Function Declarations */
void insgps_v5_1(const double zI[10], const double zG[7], int gpsflag, double
                 dt, double g0, double a, double e, double we, const double Q
                 [144], const double R[36], double PVA[10], double bias[6],
                 double Pk_1[225], double xk_1[15]);

#endif

/*
 * File trailer for insgps_v5_1.h
 *
 * [EOF]
 */
