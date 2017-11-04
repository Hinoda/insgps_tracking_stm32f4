/*
 * File: insgps_v1_0.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 16-Oct-2017 08:31:21
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "Cbn_31.h"
#include "insgps_v1_0.h"
#include "normC.h"
#include "skew_mat3.h"
#include "mrdivide.h"

/* Function Declarations */
static double rt_atan2d_snf(double u0, double u1);
static double rt_powd_snf(double u0, double u1);

/* Function Definitions */

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2(b_u0, b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_powd_snf(double u0, double u1)
{
  double y;
  double d0;
  double d1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d0 = fabs(u0);
    d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d0 == 1.0) {
        y = rtNaN;
      } else if (d0 > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

/*
 * ************************************
 *    imufix
 *  zI(5:7)=zI(5:7)-bias(4:6);%bg
 *  zI(8:10)=zI(8:10)-bias(1:3);%ba
 * ************************************
 *  mecha
 * Arguments    : const double zI[10]
 *                const double zG[7]
 *                double gpsflag
 *                double dt
 *                double g0
 *                double a
 *                double e
 *                double we
 *                const double PVA_[10]
 *                double bias[6]
 *                const double Q[144]
 *                const double sgmR[6]
 *                const double Pk_1[225]
 *                const double xk_1[15]
 *                double PVAout[10]
 *                double biasout[6]
 * Return Type  : void
 */
void insgps_v1_0(void)
{
  double vn_[3];
  double b_PVA_[3];
  double Cbn_[9];
  double x;
  double N_;
  double M_;
  double R;
  double y;
  double wen_n[3];
  double wie_n[3];
  signed char I[9];
  int k;
  double b_zI[3];
  double dv0[9];
  double dv1[9];
  double dv2[9];
  double b_I[9];
  double Cbn[9];
  int i0;
  double b_Cbn[9];
  double b_a;
  int i1;
  double c_zI[3];
  double dv3[3];
  double h;
  double lat;
  double N;
  double M;
  double lon;
  double v[3];
  double Hk[90];
  static const signed char iv0[45] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0 };

  double b_y[6];
  double d[36];
  double b_v[6];
  double b_d[36];
  double b_Hk[90];
  double c_Hk[36];
  double c_d[36];
  double K[90];
  double d_Hk[36];
  double b_M[6];
  double xk[15];
  double b_xk[3];
  double c_xk[3];
  double d_xk[3];
  double c_Cbn[9];
  (void)Q;

  /*  */
  vn_[0] = PVA_[4];
  vn_[1] = PVA_[5];
  vn_[2] = PVA_[6];

  /*  */
  /* rad */
  b_PVA_[0] = PVA_[7];
  b_PVA_[1] = PVA_[8];
  b_PVA_[2] = PVA_[9];
  Cbn_31(b_PVA_, Cbn_);

  /*  Cbn_=normC(Cbn_); */
  /* params */
  x = sin(PVA_[1]);
  N_ = a / sqrt(1.0 - e * e * (x * x));
  x = sin(PVA_[1]);
  M_ = a * (1.0 - e * e) / rt_powd_snf(1.0 - e * e * (x * x), 1.5);
  R = sqrt(N_ * M_);

  /* % */
  y = R / (R + PVA_[3]);

  /* gn */
  wen_n[0] = PVA_[5] / (N_ + PVA_[3]);
  wen_n[1] = -PVA_[4] / (M_ + PVA_[3]);
  wen_n[2] = -PVA_[5] * tan(PVA_[1]) / (N_ + PVA_[3]);
  wie_n[0] = we * cos(PVA_[1]);
  wie_n[1] = 0.0;
  wie_n[2] = -we * sin(PVA_[1]);
  for (k = 0; k < 9; k++) {
    I[k] = 0;
  }

  for (k = 0; k < 3; k++) {
    I[k + 3 * k] = 1;
  }

  b_zI[0] = zI[4] - bias[3];
  b_zI[1] = zI[5] - bias[4];
  b_zI[2] = zI[6] - bias[5];
  skew_mat3(b_zI, dv0);
  skew_mat3(wie_n, dv1);
  skew_mat3(wen_n, dv2);
  for (k = 0; k < 3; k++) {
    for (i0 = 0; i0 < 3; i0++) {
      b_I[i0 + 3 * k] = (double)I[i0 + 3 * k] + dv0[i0 + 3 * k] * dt;
      Cbn[i0 + 3 * k] = dv1[i0 + 3 * k] + dv2[i0 + 3 * k];
    }
  }

  for (k = 0; k < 3; k++) {
    for (i0 = 0; i0 < 3; i0++) {
      dv0[k + 3 * i0] = 0.0;
      b_a = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        dv0[k + 3 * i0] += Cbn[k + 3 * i1] * Cbn_[i1 + 3 * i0];
        b_a += Cbn_[k + 3 * i1] * b_I[i1 + 3 * i0];
      }

      b_Cbn[k + 3 * i0] = b_a - dv0[k + 3 * i0] * dt;
    }
  }

  /*  Cbn = normC(Cbn); */
  /* ** PVA result */
  /* % Velo Accel */
  /* ** vn */
  skew_mat3(wen_n, dv0);
  skew_mat3(wie_n, dv1);
  c_zI[0] = zI[7];
  c_zI[1] = zI[8];
  c_zI[2] = zI[9];
  for (k = 0; k < 3; k++) {
    for (i0 = 0; i0 < 3; i0++) {
      Cbn[i0 + 3 * k] = (b_Cbn[i0 + 3 * k] + Cbn_[i0 + 3 * k]) / 2.0;
    }

    b_zI[k] = c_zI[k] - bias[k];
  }

  dv3[0] = 0.0;
  dv3[1] = 0.0;
  dv3[2] = g0 * (y * y);
  for (k = 0; k < 3; k++) {
    c_zI[k] = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      dv2[i0 + 3 * k] = dv0[i0 + 3 * k] + 2.0 * dv1[i0 + 3 * k];
      c_zI[k] += Cbn[k + 3 * i0] * b_zI[i0];
    }
  }

  for (k = 0; k < 3; k++) {
    b_a = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      b_a += dv2[k + 3 * i0] * vn_[i0];
    }

    b_PVA_[k] = (c_zI[k] + dv3[k]) - b_a;
    wie_n[k] = vn_[k] + b_PVA_[k] * dt;
  }

  /* % Position */
  h = PVA_[3] - dt / 2.0 * (PVA_[6] + wie_n[2]);
  lat = PVA_[1] + dt / 2.0 * (PVA_[4] / (M_ + PVA_[3]) + wie_n[0] / (M_ + h));
  x = sin(lat);
  N = a / sqrt(1.0 - e * e * (x * x));
  x = sin(lat);
  M = a * (1.0 - e * e) / rt_powd_snf(1.0 - e * e * (x * x), 1.5);
  lon = PVA_[2] + dt / 2.0 * (PVA_[5] / ((N_ + PVA_[3]) * cos(PVA_[1])) + wie_n
    [1] / ((N + h) * cos(lat)));

  /* ** rn */
  wen_n[0] = lat;
  wen_n[1] = lon;
  wen_n[2] = h;

  /* ********************************************************************************************** */
  /* 	Kalman */
  /* --------- */
  /*  */
  /*  */
  /*  */
  /*  */
  /*  */
  /*  */
  /*  */
  /* --------- */
  /* 270 */
  /* 350 */
  /* ------------------------------------------------ */
  /* dlat_edit */
  /* dlon_edit */
  /* dh */
  /* dVN */
  /* dVE */
  /* dVD */
  v[0] = M + h;
  v[1] = (N + h) * cos(lat);
  v[2] = 1.0;
  memset(&Cbn_[0], 0, 9U * sizeof(double));
  for (k = 0; k < 3; k++) {
    Cbn_[k + 3 * k] = v[k];
  }

  for (k = 0; k < 3; k++) {
    for (i0 = 0; i0 < 3; i0++) {
      Hk[i0 + 6 * k] = Cbn_[i0 + 3 * k];
      Hk[i0 + 6 * (k + 3)] = 0.0;
    }
  }

  for (k = 0; k < 9; k++) {
    for (i0 = 0; i0 < 3; i0++) {
      Hk[i0 + 6 * (k + 6)] = 0.0;
    }
  }

  for (k = 0; k < 15; k++) {
    for (i0 = 0; i0 < 3; i0++) {
      Hk[(i0 + 6 * k) + 3] = iv0[i0 + 3 * k];
    }
  }

  /* 6x15 */
  /* ---------------------------------------------- */
  /*  R~z  R~6 */
  for (k = 0; k < 6; k++) {
    b_y[k] = sgmR[k] * sgmR[k];
  }

  memset(&d[0], 0, 36U * sizeof(double));
  for (k = 0; k < 6; k++) {
    d[k + 6 * k] = b_y[k];
  }

  y = M + h;
  b_a = N + h;
  x = cos(lat);
  b_v[0] = y * y;
  b_v[1] = b_a * b_a * (x * x);
  b_v[2] = 1.0;
  b_v[3] = 1.0;
  b_v[4] = 1.0;
  b_v[5] = 1.0;
  memset(&b_d[0], 0, 36U * sizeof(double));
  for (k = 0; k < 6; k++) {
    b_d[k + 6 * k] = b_v[k];
  }

  /*  Q:P~x  Q~12=>GQG'~15=>Qk~15 */
  /* 		15x15*15x12*12x12*12x15 */
  if (gpsflag != 0.0) {
    /* co GPS moi, ko co INS */
    /*     %% ----UPDATE FIRST------------------------------------------------------- */
    /* ************************************************** */
    /* STEP1: measure_Update***************************** */
    /* ************************************************** */
    for (k = 0; k < 15; k++) {
      for (i0 = 0; i0 < 6; i0++) {
        K[k + 15 * i0] = 0.0;
        for (i1 = 0; i1 < 15; i1++) {
          K[k + 15 * i0] += Pk_1[k + 15 * i1] * Hk[i0 + 6 * i1];
        }
      }
    }

    for (k = 0; k < 6; k++) {
      for (i0 = 0; i0 < 15; i0++) {
        b_Hk[k + 6 * i0] = 0.0;
        for (i1 = 0; i1 < 15; i1++) {
          b_Hk[k + 6 * i0] += Hk[k + 6 * i1] * Pk_1[i1 + 15 * i0];
        }
      }

      for (i0 = 0; i0 < 6; i0++) {
        c_Hk[k + 6 * i0] = 0.0;
        for (i1 = 0; i1 < 15; i1++) {
          c_Hk[k + 6 * i0] += b_Hk[k + 6 * i1] * Hk[i0 + 6 * i1];
        }

        c_d[k + 6 * i0] = 0.0;
        for (i1 = 0; i1 < 6; i1++) {
          c_d[k + 6 * i0] += d[k + 6 * i1] * b_d[i1 + 6 * i0];
        }
      }
    }

    for (k = 0; k < 6; k++) {
      for (i0 = 0; i0 < 6; i0++) {
        d_Hk[i0 + 6 * k] = c_Hk[i0 + 6 * k] + c_d[i0 + 6 * k];
      }
    }

    mrdivide(K, d_Hk);

    /* 15x15*15x6*(6x15*15x15*15x6+6*6)^-1 */
    b_M[0] = (M + h) * (lat - zG[1]);
    b_M[1] = (N + h) * cos(lat) * (lon - zG[2]);
    b_M[2] = h - zG[3];
    b_M[3] = wie_n[0] - zG[4];
    b_M[4] = wie_n[1] - zG[5];
    b_M[5] = wie_n[2] - zG[6];
    for (k = 0; k < 6; k++) {
      b_y[k] = 0.0;
      for (i0 = 0; i0 < 15; i0++) {
        b_y[k] += Hk[k + 6 * i0] * xk_1[i0];
      }

      b_v[k] = b_M[k] - b_y[k];
    }

    for (k = 0; k < 15; k++) {
      b_a = 0.0;
      for (i0 = 0; i0 < 6; i0++) {
        b_a += K[k + 15 * i0] * b_v[i0];
      }

      xk[k] = xk_1[k] + b_a;
    }

    /*  Pk = Pk_ - K*(H*Pk_*H'+ Rk)*K'; */
    /* -------------------------------------------------- */
    /* ----------Cap nhat ngo ra x voi dx---------------- */
    /* -------------------------------------------------- */
    b_xk[0] = xk[0];
    b_xk[1] = xk[1];
    b_xk[2] = xk[2];
    c_xk[0] = xk[3];
    c_xk[1] = xk[4];
    c_xk[2] = xk[5];
    for (k = 0; k < 3; k++) {
      wen_n[k] -= b_xk[k];
      wie_n[k] -= c_xk[k];
    }

    for (k = 0; k < 9; k++) {
      I[k] = 0;
    }

    for (k = 0; k < 3; k++) {
      I[k + 3 * k] = 1;
    }

    d_xk[0] = xk[6];
    d_xk[1] = xk[7];
    d_xk[2] = xk[8];
    skew_mat3(d_xk, dv0);
    for (k = 0; k < 3; k++) {
      for (i0 = 0; i0 < 3; i0++) {
        b_I[i0 + 3 * k] = (double)I[i0 + 3 * k] + dv0[i0 + 3 * k];
      }
    }

    for (k = 0; k < 3; k++) {
      for (i0 = 0; i0 < 3; i0++) {
        Cbn[k + 3 * i0] = 0.0;
        for (i1 = 0; i1 < 3; i1++) {
          Cbn[k + 3 * i0] += b_I[k + 3 * i1] * b_Cbn[i1 + 3 * i0];
        }
      }
    }

    for (k = 0; k < 3; k++) {
      for (i0 = 0; i0 < 3; i0++) {
        b_Cbn[i0 + 3 * k] = Cbn[i0 + 3 * k];
      }
    }

    memcpy(&c_Cbn[0], &b_Cbn[0], 9U * sizeof(double));
    normC(c_Cbn, b_Cbn);
    PVAout[0] = zI[0];
    for (k = 0; k < 3; k++) {
      PVAout[k + 1] = wen_n[k];
      PVAout[k + 4] = wie_n[k];
    }

    PVAout[7] = rt_atan2d_snf(b_Cbn[5], b_Cbn[8]);
    PVAout[8] = -atan(b_Cbn[2] / sqrt(1.0 - b_Cbn[2] * b_Cbn[2]));
    PVAout[9] = rt_atan2d_snf(b_Cbn[1], b_Cbn[0]);

    /* 10x1 */
    bias[0] = xk[9];
    bias[1] = xk[10];
    bias[2] = xk[11];
    bias[3] = xk[12];
    bias[4] = xk[13];
    bias[5] = xk[14];

    /* -------------------------------------------------- */
    /* ----------sau khi update xong, reset state dx----- */
    /* -------------------------------------------------- */
    /* ************************************************** */
    /* STEP2: time_Prediction**************************** */
    /* ************************************************** */
    /* xk_1(1:9)=0; */
    /* Pk_ = (Pk_ + Pk_')/2; */
  } else {
    /*  ko co GPS, thi loc bias!!! */
    /* ************************************************** */
    /* STEP: time_Prediction***************************** */
    /* ************************************************** */
    /* Pk = (Pk + Pk')/2; */
    PVAout[0] = zI[0];
    for (k = 0; k < 3; k++) {
      PVAout[k + 1] = wen_n[k];
      PVAout[k + 4] = wie_n[k];
    }

    PVAout[7] = rt_atan2d_snf(b_Cbn[5], b_Cbn[8]);
    PVAout[8] = -atan(b_Cbn[2] / sqrt(1.0 - b_Cbn[2] * b_Cbn[2]));
    PVAout[9] = rt_atan2d_snf(b_Cbn[1], b_Cbn[0]);

    /* 10x1 */
    bias[0] = xk_1[9];
    bias[1] = xk_1[10];
    bias[2] = xk_1[11];
    bias[3] = xk_1[12];
    bias[4] = xk_1[13];
    bias[5] = xk_1[14];

    /* xk(1:9)=0; */
  }

  for (k = 0; k < 6; k++) {
    biasout[k] = bias[k];
  }

  /* ***************************************************************************************************** */
  /*  doi don vi */
  /* [PVA(3),PVA(2)]=deg2utm_new(PVA(2)*180/pi,PVA(3)*180/pi); */
  /*  PVA(8:10)=PVA(8:10)*180/pi; */
  /*  tao output */
  /* end */
  /* ************************************ */
  /*  save output */
  /*  save 'result\output.mat' */
  /*  plot */
  /* 	plot quy dao */
  /* 	plot bias */
}

/*
 * File trailer for insgps_v1_0.c
 *
 * [EOF]
 */
