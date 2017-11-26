/*
 * File: insgps_v6_0.h
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 12-Nov-2017 15:05:52
 */

#ifndef INSGPS_V6_0_H
#define INSGPS_V6_0_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "Cbn_31_types.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/* Function Declarations */
//void insgps_v6_0(float zI[10], float zG[7], bool gpsflag, float dt,\
//					float g0, float a, float e, float we, float Q[144], float R[36],\
//					float PVA[10], float bias[6], float Pk_1[225], float xk_1[15]);
void insgps_v6_0(float* zI, float* zG, bool gpsflag, float dt,\
					float g0, float a, float e, float we, float* Q, float* R,\
					float* PVA, float* bias, float* Pk_1, float* xk_1);

#endif

/*
 * File trailer for insgps_v6_0.h
 *
 * [EOF]
 */
