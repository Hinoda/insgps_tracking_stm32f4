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
void insgps_v4_0(const double zI[10], const double zG[7], double gpsflag, double
                 dt, double g0, double a, double e, double we, const double Q
                 [144], const double R[36], double PVA[10], double bias[6],
                 double Pk_1[225], double xk_1[15]);

#endif

/*
 * File trailer for insgps_v4_0.h
 *
 * [EOF]
 */
