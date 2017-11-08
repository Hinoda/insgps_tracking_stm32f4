/*
 * File: insgps_v4_0.h
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 07-Nov-2017 15:42:26
 */

#ifndef INSGPS_V4_0_H
#define INSGPS_V4_0_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "Cbn_31_types.h"

/* Function Declarations */
void insgps_v4_0(const float zI[10], const float zG[7], int gpsflag, float
                 dt, float g0, float a, float e, float we, const float Q
                 [144], const float R[36], float PVA[10], float bias[6],
                 float Pk_1[225], float xk_1[15]);

#endif

/*
 * File trailer for insgps_v4_0.h
 *
 * [EOF]
 */
