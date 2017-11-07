/*
 * File: insgps_v4_0.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 07-Nov-2017 15:42:26
 */

/* Include Files */
#include "Cbn_31.h"
#include "insgps_v4_0.h"
#include "normC.h"
#include "skew_mat3.h"
#include "eye.h"
#include "mrdivide.h"
#include "diag.h"
#include "initialize.h"
#include "genDataGetProcess.h"

/* Function Definitions */

/*
 * mecha
 * Arguments    : const double zI[10]
 *                const double zG[7]
 *                double gpsflag
 *                double dt
 *                double g0
 *                double a
 *                double e
 *                double we
 *                const double Q[144]
 *                const double R[36]
 *                double PVA[10]
 *                double bias[6]
 *                double Pk_1[225]
 *                double xk_1[15]
 * Return Type  : void
 */
void insgps_v4_0(const double zI[10], const double zG[7], double gpsflag, double
                 dt, double g0, double a, double e, double we, const double Q
                 [144], const double R[36], double PVA[10], double bias[6],
                 double Pk_1[225], double xk_1[15])
{
  double vn_[3];
  double b_PVA[3];
  double Cbn_[9];
  double x;
  double N_;
  double M_;
  double RMN;
  double y;
  double g;
  double wen_n[3];
  double wie_n[3];
  double b[9];
  double b_zI[3];
  double dv4[9];
  int i1;
  double dv5[9];
  double dv6[9];
  int i2;
  double Cbn[9];
  int i3;
  double c_zI[3];
  double dv7[3];
  double fn[3];
  double h;
  double lat;
  double N;
  double M;
  double lon;
  double G[180];
  static const signed char iv0[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  double b_a;
  static const signed char iv1[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  double b_x;
  double c_a;
  double d_a;
  double c_x;
  double e_a;
  double f_a;
  double g_a;
  double h_a;
  double Fvv12;
  double Fvv13;
  double Fvv23;
  double i_a;
  double j_a;
  double d_x;
  double k_a;
  double Pk[225];
  double b_we[3];
  double b_G[225];
  double PHIk_hat[225];
  static const signed char iv2[3] = { 0, 0, -1 };

  static const double dv8[45] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -5.5555555555555558E-5, -0.0, -0.0, -0.0,
    -5.5555555555555558E-5, -0.0, -0.0, -0.0, -5.5555555555555558E-5, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static const double dv9[45] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -5.5555555555555558E-5, -0.0, -0.0, -0.0, -5.5555555555555558E-5, -0.0, -0.0,
    -0.0, -5.5555555555555558E-5 };

  double b_M[3];
  double Hk[90];
  double l_a[6];
  double B[36];
  double b_PHIk_hat[180];
  double c_PHIk_hat[180];
  double c_G[180];
  double d_PHIk_hat[225];
  double d_G[225];
  double Qk[225];
  static const signed char iv3[45] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0 };

  double b_Hk[90];
  double c_Hk[36];
  double b_R[36];
  double K[90];
  double e_PHIk_hat[15];
  double c_M[6];
  double d_M[6];
  double xk[15];
  double b_xk[3];
  double c_xk[3];
  double m_a[225];
  double d_xk[3];
  double b_Cbn[9];

  /*  */
  vn_[0] = PVA[4];
  vn_[1] = PVA[5];
  vn_[2] = PVA[6];

  /*  */
  /* rad */
  b_PVA[0] = PVA[7];
  b_PVA[1] = PVA[8];
  b_PVA[2] = PVA[9];
  Cbn_31(b_PVA, Cbn_);

  /*  Cbn_=normC(Cbn_); */
  /* params */
  x = sin(PVA[1]);
  N_ = a / sqrt(1.0 - e * e * (x * x));
  x = sin(PVA[1]);
  M_ = a * (1.0 - e * e) / pow(1.0 - e * e * (x * x), 1.5);
  RMN = sqrt(N_ * M_);
  y = RMN / (RMN + PVA[3]);
  g = g0 * (y * y);

  /* gn */
  wen_n[0] = PVA[5] / (N_ + PVA[3]);
  wen_n[1] = -PVA[4] / (M_ + PVA[3]);
  wen_n[2] = -PVA[5] * tan(PVA[1]) / (N_ + PVA[3]);
  wie_n[0] = we * cos(PVA[1]);
  wie_n[1] = 0.0;
  wie_n[2] = -we * sin(PVA[1]);
  eye(b);
  b_zI[0] = zI[4] - bias[3];
  b_zI[1] = zI[5] - bias[4];
  b_zI[2] = zI[6] - bias[5];
  skew_mat3(b_zI, dv4);
  for (i1 = 0; i1 < 9; i1++) {
    b[i1] += dv4[i1] * dt;
  }

  skew_mat3(wie_n, dv4);
  skew_mat3(wen_n, dv5);
  for (i1 = 0; i1 < 3; i1++) {
    for (i2 = 0; i2 < 3; i2++) {
      dv6[i2 + 3 * i1] = dv4[i2 + 3 * i1] + dv5[i2 + 3 * i1];
    }
  }

  for (i1 = 0; i1 < 3; i1++) {
    for (i2 = 0; i2 < 3; i2++) {
      dv4[i1 + 3 * i2] = 0.0;
      y = 0.0;
      for (i3 = 0; i3 < 3; i3++) {
        dv4[i1 + 3 * i2] += dv6[i1 + 3 * i3] * Cbn_[i3 + 3 * i2];
        y += Cbn_[i1 + 3 * i3] * b[i3 + 3 * i2];
      }

      Cbn[i1 + 3 * i2] = y - dv4[i1 + 3 * i2] * dt;
    }
  }

  /* ** result */
  /* % Velo Accel */
  for (i1 = 0; i1 < 3; i1++) {
    for (i2 = 0; i2 < 3; i2++) {
      b[i2 + 3 * i1] = (Cbn[i2 + 3 * i1] + Cbn_[i2 + 3 * i1]) / 2.0;
    }
  }

  c_zI[0] = zI[7] - bias[0];
  c_zI[1] = zI[8] - bias[1];
  c_zI[2] = zI[9] - bias[2];

  /* ** vn */
  skew_mat3(wen_n, dv4);
  skew_mat3(wie_n, dv5);
  dv7[0] = 0.0;
  dv7[1] = 0.0;
  dv7[2] = g;
  for (i1 = 0; i1 < 3; i1++) {
    fn[i1] = 0.0;
    for (i2 = 0; i2 < 3; i2++) {
      dv6[i2 + 3 * i1] = dv4[i2 + 3 * i1] + 2.0 * dv5[i2 + 3 * i1];
      fn[i1] += b[i1 + 3 * i2] * c_zI[i2];
    }
  }

  for (i1 = 0; i1 < 3; i1++) {
    y = 0.0;
    for (i2 = 0; i2 < 3; i2++) {
      y += dv6[i1 + 3 * i2] * vn_[i2];
    }

    b_PVA[i1] = (fn[i1] + dv7[i1]) - y;
    wie_n[i1] = vn_[i1] + b_PVA[i1] * dt;
  }

  /* % Position */
  h = PVA[3] - dt / 2.0 * (PVA[6] + wie_n[2]);
  lat = PVA[1] + dt / 2.0 * (PVA[4] / (M_ + PVA[3]) + wie_n[0] / (M_ + h));
  x = sin(lat);
  N = a / sqrt(1.0 - e * e * (x * x));
  x = sin(lat);
  M = a * (1.0 - e * e) / pow(1.0 - e * e * (x * x), 1.5);
  lon = PVA[2] + dt / 2.0 * (PVA[5] / ((N_ + PVA[3]) * cos(PVA[1])) + wie_n[1] /
    ((N + h) * cos(lat)));

  /* ** rn */
  wen_n[0] = lat;
  wen_n[1] = lon;
  wen_n[2] = h;

  /* ****************************************************************** */
  /* 	Kalman */
  for (i1 = 0; i1 < 12; i1++) {
    for (i2 = 0; i2 < 3; i2++) {
      G[i2 + 15 * i1] = 0.0;
    }
  }

  for (i1 = 0; i1 < 3; i1++) {
    for (i2 = 0; i2 < 3; i2++) {
      G[(i2 + 15 * i1) + 3] = Cbn[i2 + 3 * i1];
      G[(i2 + 15 * (i1 + 3)) + 3] = 0.0;
      G[(i2 + 15 * (i1 + 6)) + 3] = 0.0;
      G[(i2 + 15 * (i1 + 9)) + 3] = 0.0;
      G[(i2 + 15 * i1) + 6] = 0.0;
      G[(i2 + 15 * (i1 + 3)) + 6] = -Cbn[i2 + 3 * i1];
      G[(i2 + 15 * (i1 + 6)) + 6] = 0.0;
      G[(i2 + 15 * (i1 + 9)) + 6] = 0.0;
    }
  }

  for (i1 = 0; i1 < 12; i1++) {
    for (i2 = 0; i2 < 3; i2++) {
      G[(i2 + 15 * i1) + 9] = iv0[i2 + 3 * i1];
      G[(i2 + 15 * i1) + 12] = iv1[i2 + 3 * i1];
    }
  }

  /* --------- */
  y = M + h;
  x = cos(lat);
  b_a = N + h;

  /*  */
  /*  */
  /*  */
  b_x = cos(lat);
  c_a = M + h;
  d_a = N + h;
  c_x = cos(lat);
  e_a = N + h;
  f_a = N + h;
  g_a = N + h;
  h_a = M + h;

  /*  */
  Fvv12 = -2.0 * we * sin(lat) - 2.0 * wie_n[1] * tan(lat) / (N + h);
  Fvv13 = wie_n[0] / (M + h);
  Fvv23 = 2.0 * we * cos(lat) + wie_n[1] / (N + h);

  /*  */
  /*  */
  i_a = N + h;
  j_a = M + h;
  d_x = cos(lat);
  k_a = N + h;

  /*  */
  /* --------- */
  /* 270 */
  /* 350 */
  b_eye(Pk);
  skew_mat3(fn, dv4);
  b_we[0] = we * cos(lat) + wie_n[1] / (N + h);
  b_we[1] = -wie_n[0] / (M + h);
  b_we[2] = -we * sin(lat) - wie_n[1] * tan(lat) / (N + h);
  skew_mat3(b_we, dv5);
  b_G[0] = 0.0;
  b_G[15] = 0.0;
  b_G[30] = -wie_n[0] / (y * y);
  b_G[1] = wie_n[1] * sin(lat) / ((N + h) * (x * x));
  b_G[16] = 0.0;
  b_G[31] = -wie_n[1] / (b_a * b_a * cos(lat));
  b_G[45] = 1.0 / (M + h);
  b_G[60] = 0.0;
  b_G[75] = 0.0;
  b_G[46] = 0.0;
  b_G[61] = 1.0 / ((N + h) * cos(lat));
  b_G[76] = 0.0;
  b_G[3] = -2.0 * wie_n[1] * we * cos(lat) - wie_n[1] * wie_n[1] / ((N + h) *
    (b_x * b_x));
  b_G[18] = 0.0;
  b_G[33] = -wie_n[0] * wie_n[2] / (c_a * c_a) + wie_n[1] * wie_n[1] * tan(lat) /
    (d_a * d_a);
  b_G[4] = 2.0 * we * (wie_n[0] * cos(lat) - wie_n[2] * sin(lat)) + wie_n[1] *
    wie_n[0] / ((N + h) * (c_x * c_x));
  b_G[19] = 0.0;
  b_G[34] = -wie_n[1] * wie_n[2] / (e_a * e_a) - wie_n[0] * wie_n[1] * tan(lat) /
    (f_a * f_a);
  b_G[5] = 2.0 * wie_n[1] * we * sin(lat);
  b_G[20] = 0.0;
  b_G[35] = (wie_n[1] * wie_n[1] / (g_a * g_a) + wie_n[0] * wie_n[0] / (h_a *
              h_a)) - 2.0 * g / (sqrt(M * N) + h);
  b_G[48] = wie_n[2] / (M + h);
  b_G[63] = Fvv12;
  b_G[78] = Fvv13;
  b_G[49] = -Fvv12 - wie_n[1] * tan(lat) / (N + h);
  b_G[64] = (wie_n[2] + wie_n[0] * tan(lat)) / (N + h);
  b_G[79] = Fvv23;
  b_G[50] = -2.0 * Fvv13;
  b_G[65] = -Fvv23 - wie_n[1] / (N + h);
  b_G[80] = 0.0;
  b_G[6] = -we * sin(lat);
  b_G[21] = 0.0;
  b_G[36] = -wie_n[1] / (i_a * i_a);
  b_G[7] = 0.0;
  b_G[22] = 0.0;
  b_G[37] = wie_n[0] / (j_a * j_a);
  b_G[8] = -we * cos(lat) - wie_n[1] / ((N + h) * (d_x * d_x));
  b_G[23] = 0.0;
  b_G[38] = wie_n[1] * tan(lat) / (k_a * k_a);
  b_G[51] = 0.0;
  b_G[66] = 1.0 / (N + h);
  b_G[81] = 0.0;
  b_G[52] = -1.0 / (M + h);
  b_G[67] = 0.0;
  b_G[82] = 0.0;
  b_G[53] = 0.0;
  b_G[68] = -tan(lat) / (N + h);
  b_G[83] = 0.0;
  for (i1 = 0; i1 < 3; i1++) {
    b_G[2 + 15 * i1] = 0.0;
    b_G[2 + 15 * (i1 + 3)] = iv2[i1];
    for (i2 = 0; i2 < 3; i2++) {
      b_G[i2 + 15 * (i1 + 6)] = 0.0;
      b_G[i2 + 15 * (i1 + 9)] = 0.0;
      b_G[i2 + 15 * (i1 + 12)] = 0.0;
      b_G[(i2 + 15 * (i1 + 6)) + 3] = dv4[i2 + 3 * i1];
      b_G[(i2 + 15 * (i1 + 9)) + 3] = Cbn[i2 + 3 * i1];
      b_G[(i2 + 15 * (i1 + 12)) + 3] = 0.0;
      b_G[(i2 + 15 * (i1 + 6)) + 6] = -dv5[i2 + 3 * i1];
      b_G[(i2 + 15 * (i1 + 9)) + 6] = 0.0;
      b_G[(i2 + 15 * (i1 + 12)) + 6] = -Cbn[i2 + 3 * i1];
    }
  }

  for (i1 = 0; i1 < 15; i1++) {
    for (i2 = 0; i2 < 3; i2++) {
      b_G[(i2 + 15 * i1) + 9] = dv8[i2 + 3 * i1];
      b_G[(i2 + 15 * i1) + 12] = dv9[i2 + 3 * i1];
    }

    for (i2 = 0; i2 < 15; i2++) {
      PHIk_hat[i2 + 15 * i1] = Pk[i2 + 15 * i1] + b_G[i2 + 15 * i1] * dt;
    }
  }

  /* ------------------------------------------------ */
  /* dlat_edit */
  /* dlon_edit */
  /* dh */
  /* dVN */
  /* dVE */
  /* dVD */
  b_M[0] = M + h;
  b_M[1] = (N + h) * cos(lat);
  b_M[2] = 1.0;
  diag(b_M, Cbn_);
  for (i1 = 0; i1 < 3; i1++) {
    for (i2 = 0; i2 < 3; i2++) {
      Hk[i2 + 6 * i1] = Cbn_[i2 + 3 * i1];
      Hk[i2 + 6 * (i1 + 3)] = 0.0;
    }
  }

  for (i1 = 0; i1 < 9; i1++) {
    for (i2 = 0; i2 < 3; i2++) {
      Hk[i2 + 6 * (i1 + 6)] = 0.0;
    }
  }

  /* 6x15 */
  /* ---------------------------------------------- */
  /*  R~z  R~6 */
  y = M + h;
  b_a = N + h;
  x = cos(lat);
  l_a[0] = y * y;
  l_a[1] = b_a * b_a * (x * x);
  l_a[2] = 1.0;
  l_a[3] = 1.0;
  l_a[4] = 1.0;
  l_a[5] = 1.0;
  b_diag(l_a, B);

  /*  Q:P~x  Q~12=>GQG'~15=>Qk~15 */
  /* 		15x15*15x12*12x12*12x15 */
  for (i1 = 0; i1 < 15; i1++) {
    for (i2 = 0; i2 < 3; i2++) {
      Hk[(i2 + 6 * i1) + 3] = iv3[i2 + 3 * i1];
    }

    for (i2 = 0; i2 < 12; i2++) {
      b_PHIk_hat[i1 + 15 * i2] = 0.0;
      for (i3 = 0; i3 < 15; i3++) {
        b_PHIk_hat[i1 + 15 * i2] += PHIk_hat[i1 + 15 * i3] * G[i3 + 15 * i2];
      }
    }

    for (i2 = 0; i2 < 12; i2++) {
      c_PHIk_hat[i1 + 15 * i2] = 0.0;
      c_G[i1 + 15 * i2] = 0.0;
      for (i3 = 0; i3 < 12; i3++) {
        c_PHIk_hat[i1 + 15 * i2] += b_PHIk_hat[i1 + 15 * i3] * Q[i3 + 12 * i2];
        c_G[i1 + 15 * i2] += G[i1 + 15 * i3] * Q[i3 + 12 * i2];
      }
    }

    for (i2 = 0; i2 < 15; i2++) {
      b_G[i1 + 15 * i2] = 0.0;
      d_PHIk_hat[i1 + 15 * i2] = 0.0;
      for (i3 = 0; i3 < 12; i3++) {
        b_G[i1 + 15 * i2] += c_G[i1 + 15 * i3] * G[i2 + 15 * i3];
        d_PHIk_hat[i1 + 15 * i2] += c_PHIk_hat[i1 + 15 * i3] * G[i2 + 15 * i3];
      }
    }

    for (i2 = 0; i2 < 15; i2++) {
      d_G[i1 + 15 * i2] = 0.0;
      for (i3 = 0; i3 < 15; i3++) {
        d_G[i1 + 15 * i2] += b_G[i1 + 15 * i3] * PHIk_hat[i2 + 15 * i3];
      }
    }
  }

  for (i1 = 0; i1 < 15; i1++) {
    for (i2 = 0; i2 < 15; i2++) {
      Qk[i2 + 15 * i1] = 0.5 * (d_PHIk_hat[i2 + 15 * i1] + d_G[i2 + 15 * i1]) *
        dt;
    }
  }

  if (gpsflag != 0.0) {
    /* co GPS moi, ko co INS */
    /* ********************************************************* */
    /* STEP1: measure_Update */
    for (i1 = 0; i1 < 15; i1++) {
      for (i2 = 0; i2 < 6; i2++) {
        K[i1 + 15 * i2] = 0.0;
        for (i3 = 0; i3 < 15; i3++) {
          K[i1 + 15 * i2] += Pk_1[i1 + 15 * i3] * Hk[i2 + 6 * i3];
        }
      }
    }

    for (i1 = 0; i1 < 6; i1++) {
      for (i2 = 0; i2 < 15; i2++) {
        b_Hk[i1 + 6 * i2] = 0.0;
        for (i3 = 0; i3 < 15; i3++) {
          b_Hk[i1 + 6 * i2] += Hk[i1 + 6 * i3] * Pk_1[i3 + 15 * i2];
        }
      }

      for (i2 = 0; i2 < 6; i2++) {
        c_Hk[i1 + 6 * i2] = 0.0;
        for (i3 = 0; i3 < 15; i3++) {
          c_Hk[i1 + 6 * i2] += b_Hk[i1 + 6 * i3] * Hk[i2 + 6 * i3];
        }

        b_R[i1 + 6 * i2] = 0.0;
        for (i3 = 0; i3 < 6; i3++) {
          b_R[i1 + 6 * i2] += R[i1 + 6 * i3] * B[i3 + 6 * i2];
        }
      }
    }

    for (i1 = 0; i1 < 6; i1++) {
      for (i2 = 0; i2 < 6; i2++) {
        B[i2 + 6 * i1] = c_Hk[i2 + 6 * i1] + b_R[i2 + 6 * i1];
      }
    }

    mrdivide(K, B);

    /* 15x15*15x6*(6x15*15x15*15x6+6*6)^-1 */
    c_M[0] = (M + h) * (lat - zG[1]);
    c_M[1] = (N + h) * cos(lat) * (lon - zG[2]);
    c_M[2] = h - zG[3];
    c_M[3] = wie_n[0] - zG[4];
    c_M[4] = wie_n[1] - zG[5];
    c_M[5] = wie_n[2] - zG[6];
    for (i1 = 0; i1 < 6; i1++) {
      l_a[i1] = 0.0;
      for (i2 = 0; i2 < 15; i2++) {
        l_a[i1] += Hk[i1 + 6 * i2] * xk_1[i2];
      }

      d_M[i1] = c_M[i1] - l_a[i1];
    }

    b_eye(Pk);
    for (i1 = 0; i1 < 15; i1++) {
      y = 0.0;
      for (i2 = 0; i2 < 6; i2++) {
        y += K[i1 + 15 * i2] * d_M[i2];
      }

      xk[i1] = xk_1[i1] + y;
      for (i2 = 0; i2 < 15; i2++) {
        y = 0.0;
        for (i3 = 0; i3 < 6; i3++) {
          y += K[i1 + 15 * i3] * Hk[i3 + 6 * i2];
        }

        m_a[i1 + 15 * i2] = Pk[i1 + 15 * i2] - y;
      }

      for (i2 = 0; i2 < 15; i2++) {
        Pk[i1 + 15 * i2] = 0.0;
        for (i3 = 0; i3 < 15; i3++) {
          Pk[i1 + 15 * i2] += m_a[i1 + 15 * i3] * Pk_1[i3 + 15 * i2];
        }
      }
    }

    /* update xk with d_xk */
    b_xk[0] = xk[0];
    b_xk[1] = xk[1];
    b_xk[2] = xk[2];
    c_xk[0] = xk[3];
    c_xk[1] = xk[4];
    c_xk[2] = xk[5];
    for (i1 = 0; i1 < 3; i1++) {
      wen_n[i1] -= b_xk[i1];
      wie_n[i1] -= c_xk[i1];
    }

    eye(Cbn_);
    d_xk[0] = xk[6];
    d_xk[1] = xk[7];
    d_xk[2] = xk[8];
    skew_mat3(d_xk, dv4);
    for (i1 = 0; i1 < 9; i1++) {
      Cbn_[i1] += dv4[i1];
    }

    for (i1 = 0; i1 < 3; i1++) {
      for (i2 = 0; i2 < 3; i2++) {
        b[i1 + 3 * i2] = 0.0;
        for (i3 = 0; i3 < 3; i3++) {
          b[i1 + 3 * i2] += Cbn_[i1 + 3 * i3] * Cbn[i3 + 3 * i2];
        }
      }
    }

    for (i1 = 0; i1 < 3; i1++) {
      for (i2 = 0; i2 < 3; i2++) {
        Cbn[i2 + 3 * i1] = b[i2 + 3 * i1];
      }
    }

    memcpy(&b_Cbn[0], &Cbn[0], 9U * sizeof(double));
    normC(b_Cbn, Cbn);
    PVA[0] = zI[0];
    for (i1 = 0; i1 < 3; i1++) {
      PVA[i1 + 1] = wen_n[i1];
      PVA[i1 + 4] = wie_n[i1];
    }

    PVA[7] = atan2(Cbn[5], Cbn[8]);
    PVA[8] = -atan(Cbn[2] / sqrt(1.0 - Cbn[2] * Cbn[2]));
    PVA[9] = atan2(Cbn[1], Cbn[0]);

    /* 10x1 */
    bias[0] = xk[9];
    bias[1] = xk[10];
    bias[2] = xk[11];
    bias[3] = xk[12];
    bias[4] = xk[13];
    bias[5] = xk[14];

    /* reset state d_xk after update xk */
    memset(&xk[0], 0, 9U * sizeof(double));

    /* ********************************************************* */
    /* STEP2: time_Prediction */
    for (i1 = 0; i1 < 15; i1++) {
      xk_1[i1] = 0.0;
      for (i2 = 0; i2 < 15; i2++) {
        xk_1[i1] += PHIk_hat[i1 + 15 * i2] * xk[i2];
        b_G[i2 + 15 * i1] = (Pk[i2 + 15 * i1] + Pk[i1 + 15 * i2]) / 2.0;
      }
    }

    for (i1 = 0; i1 < 15; i1++) {
      for (i2 = 0; i2 < 15; i2++) {
        d_PHIk_hat[i1 + 15 * i2] = 0.0;
        for (i3 = 0; i3 < 15; i3++) {
          d_PHIk_hat[i1 + 15 * i2] += PHIk_hat[i1 + 15 * i3] * b_G[i3 + 15 * i2];
        }
      }

      for (i2 = 0; i2 < 15; i2++) {
        y = 0.0;
        for (i3 = 0; i3 < 15; i3++) {
          y += d_PHIk_hat[i1 + 15 * i3] * PHIk_hat[i2 + 15 * i3];
        }

        Pk_1[i1 + 15 * i2] = y + Qk[i1 + 15 * i2];
      }
    }
  } else {
    /* STEP: time_Prediction */
    PVA[0] = zI[0];
    for (i1 = 0; i1 < 3; i1++) {
      PVA[i1 + 1] = wen_n[i1];
      PVA[i1 + 4] = wie_n[i1];
    }

    PVA[7] = atan2(Cbn[5], Cbn[8]);
    PVA[8] = -atan(Cbn[2] / sqrt(1.0 - Cbn[2] * Cbn[2]));
    PVA[9] = atan2(Cbn[1], Cbn[0]);

    /* 10x1 */
    bias[0] = xk_1[9];
    bias[1] = xk_1[10];
    bias[2] = xk_1[11];
    bias[3] = xk_1[12];
    bias[4] = xk_1[13];
    bias[5] = xk_1[14];
    for (i1 = 0; i1 < 15; i1++) {
      e_PHIk_hat[i1] = 0.0;
      for (i2 = 0; i2 < 15; i2++) {
        e_PHIk_hat[i1] += PHIk_hat[i1 + 15 * i2] * xk_1[i2];
      }
    }

    for (i1 = 0; i1 < 15; i1++) {
      xk_1[i1] = e_PHIk_hat[i1];
      for (i2 = 0; i2 < 15; i2++) {
        d_PHIk_hat[i1 + 15 * i2] = 0.0;
        for (i3 = 0; i3 < 15; i3++) {
          d_PHIk_hat[i1 + 15 * i2] += PHIk_hat[i1 + 15 * i3] * Pk_1[i3 + 15 * i2];
        }
      }
    }

    for (i1 = 0; i1 < 15; i1++) {
      for (i2 = 0; i2 < 15; i2++) {
        y = 0.0;
        for (i3 = 0; i3 < 15; i3++) {
          y += d_PHIk_hat[i1 + 15 * i3] * PHIk_hat[i2 + 15 * i3];
        }

        Pk_1[i1 + 15 * i2] = y + Qk[i1 + 15 * i2];
      }
    }
  }
}

/*
 * File trailer for insgps_v4_0.c
 *
 * [EOF]
 */
