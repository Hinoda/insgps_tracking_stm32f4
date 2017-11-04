/*
 * File: insgps_v1_0.h
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 16-Oct-2017 08:31:21
 */

#ifndef INSGPS_V1_0_H
#define INSGPS_V1_0_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "Cbn_31_types.h"


extern double zI[10];
extern double zG[7];
extern enum gpsflag_t{NOGPS,YESGPS}gpsflag;
extern double dt, g0;
extern double a, e;				//ban kinh TD va tam sai WGS84
extern double we;					//toc do quay TD
extern double PVA_[10];
extern double PVAout[10];
extern double biasout[6];
extern double bias[6];
extern double Q[144], sgmR[6];
extern double Pk_1[225], xk_1[15]; /*ko setup them Pk, xk vi 
														chi muon delay, ko xuat ra*/
/* Function Declarations */
extern void insgps_v1_0(void);

#endif

/*
 * File trailer for insgps_v1_0.h
 *
 * [EOF]
 */
