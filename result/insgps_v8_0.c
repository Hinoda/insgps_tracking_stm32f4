/*
 * File: insgps_v8_0.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 07-Dec-2017 23:15:41
 */

/* Include Files */
#include "Cbn_31.h"
#include "deg2utm.h"
#include "insgps_v8_0.h"
#include "normC.h"
#include "skew_mat3.h"
#include "eye.h"
#include "mrdivide.h"
#include "diag.h"
#include <stdbool.h>

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
void insgps_v8_0(const double zI[10], const double zG[7], bool gpsflag, double
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
  double g;
  double wen_n[3];
  double wie_n[3];
  double b[9];
  int b_i;
  double dv4[9];
  double dv5[9];
  double dv6[9];
  int i1;
  double Cbn[9];
  int i2;
  double dv7[3];
  double fn[3];
  double vn[3];
  double h;
  double lat;
  double sin_lat;
  double cos_lat;
  double tan_lat;
  double N;
  double M;
  double invNplushcos_lat2;
  double lon;
  double b_a;
  double c_a;
  double d_a;
  double e_a;
  double f_a;
  double h_a;
  double i_a;
  double j_a;
  double k_a;
  double l_a;
  double PHIk_hat[225];
  double matCbn[54];
  double dv8[225];
  static const signed char iv0[9] = { 0, 0, 0, 0, 0, -1, 0, 0, 0 };

  double b_PHIk_hat[225];
  static const double dv9[90] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0037037037037037038, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0037037037037037038,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0037037037037037038, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.0028571428571428571, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.0028571428571428571, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0028571428571428571
  };

  double G[180];
  double c_PHIk_hat[180];
  static const signed char iv1[72] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1 };

  double d_PHIk_hat[180];
  double b_G[180];
  double e_PHIk_hat[225];
  double c_G[225];
  double Qk[225];
  double b_M[6];
  double Hk[30];
  double Hksub[36];
  double b_Hk[2];
  double c_Hk[90];
  double Rk[4];
  double b_Pk_1[30];
  double d_Hk[36];
  double e_Hk[90];
  double K[90];
  double f_Hk[4];
  double g_Hk[30];
  double b_lat[6];
  double b_K[30];
  double b_wen_n[2];
  double c_wen_n[2];
  double h_Hk[6];
  double b_Hksub[6];
  double m_a[225];
  double n_a[225];
  double c_Pk_1[225];
  double d_Pk_1[225];
  double o_a[225];
  double b_xk_1[3];
  double c_xk_1[3];
  double d_xk_1[3];
  double b_Cbn[9];
  double c_Cbn[9];
  double f_PHIk_hat[15];

  /* { */
  /* lat_ = PVA(2); */
  /* lon_ = PVA(3); */
  /* h_   = PVA(4); */
  /* rn_  = [PVA(2);PVA(3);PVA(4)]; */
  /*  */
  /* VN_  = PVA(5); */
  /* VE_  = PVA(6); */
  /* VD_  = PVA(7); */
  /* } */
  vn_[0] = PVA[4];
  vn_[1] = PVA[5];
  vn_[2] = PVA[6];

  /*  rad */
  b_PVA[0] = PVA[7];
  b_PVA[1] = PVA[8];
  b_PVA[2] = PVA[9];
  Cbn_31(b_PVA, Cbn_);

  /*  Cbn_=normC(Cbn_); */
  /* fbhat=fbswigg-bias */
  /* params */
  x = sin(PVA[1]);
  N_ = a / sqrt(1.0 - e * e * (x * x));
  x = sin(PVA[1]);
  M_ = a * (1.0 - e * e) / pow(1.0 - e * e * (x * x), 1.5);
  RMN = sqrt(N_ * M_);
  x = RMN / (RMN + PVA[3]);
  g = g0 * (x * x);

  /* gn */
  wen_n[0] = PVA[5] / (N_ + PVA[3]);
  wen_n[1] = -PVA[4] / (M_ + PVA[3]);
  wen_n[2] = -PVA[5] * tan(PVA[1]) / (N_ + PVA[3]);
  wie_n[0] = we * cos(PVA[1]);
  wie_n[1] = 0.0;
  wie_n[2] = -we * sin(PVA[1]);
  eye(b);
  for (b_i = 0; b_i < 3; b_i++) {
    b_PVA[b_i] = zI[b_i + 4] - bias[b_i + 3];
  }

  skew_mat3(b_PVA, dv4);
  for (b_i = 0; b_i < 9; b_i++) {
    b[b_i] += dv4[b_i] * dt;
  }

  skew_mat3(wie_n, dv4);
  skew_mat3(wen_n, dv5);
  for (b_i = 0; b_i < 3; b_i++) {
    for (i1 = 0; i1 < 3; i1++) {
      dv6[i1 + 3 * b_i] = dv4[i1 + 3 * b_i] + dv5[i1 + 3 * b_i];
    }
  }

  for (b_i = 0; b_i < 3; b_i++) {
    for (i1 = 0; i1 < 3; i1++) {
      dv4[b_i + 3 * i1] = 0.0;
      x = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        dv4[b_i + 3 * i1] += dv6[b_i + 3 * i2] * Cbn_[i2 + 3 * i1];
        x += Cbn_[b_i + 3 * i2] * b[i2 + 3 * i1];
      }

      Cbn[b_i + 3 * i1] = x - dv4[b_i + 3 * i1] * dt;
    }
  }

  /* ** result */
  /* % Velo Accel */
  for (b_i = 0; b_i < 3; b_i++) {
    for (i1 = 0; i1 < 3; i1++) {
      b[i1 + 3 * b_i] = (Cbn[i1 + 3 * b_i] + Cbn_[i1 + 3 * b_i]) / 2.0;
    }

    b_PVA[b_i] = zI[7 + b_i] - bias[b_i];
  }

  /* ** vn */
  skew_mat3(wen_n, dv4);
  skew_mat3(wie_n, dv5);
  dv7[0] = 0.0;
  dv7[1] = 0.0;
  dv7[2] = g;
  for (b_i = 0; b_i < 3; b_i++) {
    fn[b_i] = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      dv6[i1 + 3 * b_i] = dv4[i1 + 3 * b_i] + 2.0 * dv5[i1 + 3 * b_i];
      fn[b_i] += b[b_i + 3 * i1] * b_PVA[i1];
    }
  }

  for (b_i = 0; b_i < 3; b_i++) {
    x = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      x += dv6[b_i + 3 * i1] * vn_[i1];
    }

    b_PVA[b_i] = (fn[b_i] + dv7[b_i]) - x;
    vn[b_i] = vn_[b_i] + b_PVA[b_i] * dt;
  }

  /* % Position */
  h = PVA[3] - dt / 2.0 * (PVA[6] + vn[2]);
  lat = PVA[1] + dt / 2.0 * (PVA[4] / (M_ + PVA[3]) + vn[0] / (M_ + h));
  sin_lat = sin(lat);
  cos_lat = cos(lat);
  tan_lat = sin_lat / cos_lat;
  N = a / sqrt(1.0 - e * e * (sin_lat * sin_lat));
  M = a * (1.0 - e * e) / pow(1.0 - e * e * (sin_lat * sin_lat), 1.5);
  invNplushcos_lat2 = 1.0 / ((N + h) * (cos_lat * cos_lat));
  lon = PVA[2] + dt / 2.0 * (PVA[5] / ((N_ + PVA[3]) * cos(PVA[1])) + vn[1] /
    ((N + h) * cos_lat));

  /* ** rn */
  wie_n[0] = lat;
  wie_n[1] = lon;
  wie_n[2] = h;
  wen_n[0] = we * cos_lat + vn[1] / (N + h);
  wen_n[1] = -vn[0] / (M + h);
  wen_n[2] = -we * sin_lat - vn[1] * tan_lat / (N + h);

  /* ****************************************************************** */
  /* 	Extended Kalman */
  /* ------------------------------------------------ */
  /*  linearize */
  /* --------- */
  x = M + h;
  b_a = N + h;

  /* --------- */
  /* --------- */
  c_a = M + h;
  d_a = N + h;
  e_a = N + h;
  f_a = N + h;
  h_a = N + h;
  i_a = M + h;

  /* --------- */
  /* --------- */
  j_a = N + h;
  k_a = M + h;
  l_a = N + h;

  /* --------- */
  /* ------------------------------------------------ */
  for (b_i = 0; b_i < 6; b_i++) {
    for (i1 = 0; i1 < 3; i1++) {
      matCbn[i1 + 9 * b_i] = 0.0;
    }
  }

  for (b_i = 0; b_i < 3; b_i++) {
    for (i1 = 0; i1 < 3; i1++) {
      matCbn[(i1 + 9 * b_i) + 3] = Cbn[i1 + 3 * b_i];
      matCbn[(i1 + 9 * (b_i + 3)) + 3] = 0.0;
      matCbn[(i1 + 9 * b_i) + 6] = 0.0;
      matCbn[(i1 + 9 * (b_i + 3)) + 6] = -Cbn[i1 + 3 * b_i];
    }
  }

  b_eye(PHIk_hat);
  dv8[0] = 0.0;
  dv8[15] = 0.0;
  dv8[30] = -vn[0] / (x * x);
  dv8[45] = 1.0 / (M + h);
  dv8[60] = 0.0;
  dv8[75] = 0.0;
  dv8[90] = 0.0;
  dv8[105] = 0.0;
  dv8[120] = 0.0;
  dv8[1] = vn[1] * sin_lat * invNplushcos_lat2;
  dv8[16] = 0.0;
  dv8[31] = -vn[1] / (b_a * b_a * cos_lat);
  dv8[46] = 0.0;
  dv8[61] = 1.0 / ((N + h) * cos_lat);
  dv8[76] = 0.0;
  dv8[91] = 0.0;
  dv8[106] = 0.0;
  dv8[121] = 0.0;
  for (b_i = 0; b_i < 9; b_i++) {
    dv8[2 + 15 * b_i] = iv0[b_i];
  }

  dv8[3] = -2.0 * vn[1] * we * cos_lat - vn[1] * vn[1] * invNplushcos_lat2;
  dv8[18] = 0.0;
  dv8[33] = -vn[0] * vn[2] / (c_a * c_a) + vn[1] * vn[1] * tan_lat / (d_a * d_a);
  dv8[48] = vn[2] / (M + h);
  dv8[63] = -2.0 * we * sin_lat - 2.0 * vn[1] * tan_lat / (N + h);
  dv8[78] = vn[0] / (M + h);
  dv8[93] = 0.0;
  dv8[108] = -fn[2];
  dv8[123] = fn[1];
  dv8[4] = 2.0 * we * (vn[0] * cos_lat - vn[2] * sin_lat) + vn[1] * vn[0] *
    invNplushcos_lat2;
  dv8[19] = 0.0;
  dv8[34] = -vn[1] * vn[2] / (e_a * e_a) - vn[0] * vn[1] * tan_lat / (f_a * f_a);
  dv8[49] = 2.0 * we * sin_lat + vn[1] * tan_lat / (N + h);
  dv8[64] = (vn[2] + vn[0] * tan_lat) / (N + h);
  dv8[79] = 2.0 * we * cos_lat + vn[1] / (N + h);
  dv8[94] = fn[2];
  dv8[109] = 0.0;
  dv8[124] = -fn[0];
  dv8[5] = 2.0 * vn[1] * we * sin_lat;
  dv8[20] = 0.0;
  dv8[35] = (vn[1] * vn[1] / (h_a * h_a) + vn[0] * vn[0] / (i_a * i_a)) - 2.0 *
    g / (sqrt(M * N) + h);
  dv8[50] = -2.0 * vn[0] / (M + h);
  dv8[65] = -2.0 * we * cos_lat - 2.0 * vn[1] / (N + h);
  dv8[80] = 0.0;
  dv8[95] = -fn[1];
  dv8[110] = fn[0];
  dv8[125] = 0.0;
  dv8[6] = -we * sin_lat;
  dv8[21] = 0.0;
  dv8[36] = -vn[1] / (j_a * j_a);
  dv8[51] = 0.0;
  dv8[66] = 1.0 / (N + h);
  dv8[81] = 0.0;
  dv8[96] = 0.0;
  dv8[111] = wen_n[2];
  dv8[126] = -wen_n[1];
  dv8[7] = 0.0;
  dv8[22] = 0.0;
  dv8[37] = vn[0] / (k_a * k_a);
  dv8[52] = -1.0 / (M + h);
  dv8[67] = 0.0;
  dv8[82] = 0.0;
  dv8[97] = -wen_n[2];
  dv8[112] = 0.0;
  dv8[127] = wen_n[0];
  dv8[8] = -we * cos_lat - vn[1] * invNplushcos_lat2;
  dv8[23] = 0.0;
  dv8[38] = vn[1] * tan_lat / (l_a * l_a);
  dv8[53] = 0.0;
  dv8[68] = -tan_lat / (N + h);
  dv8[83] = 0.0;
  dv8[98] = wen_n[1];
  dv8[113] = -wen_n[0];
  dv8[128] = 0.0;
  for (b_i = 0; b_i < 6; b_i++) {
    memcpy(&dv8[b_i * 15 + 135], &matCbn[b_i * 9], 9U * sizeof(double));
  }

  for (b_i = 0; b_i < 15; b_i++) {
    for (i1 = 0; i1 < 6; i1++) {
      dv8[(i1 + 15 * b_i) + 9] = dv9[i1 + 6 * b_i];
    }

    for (i1 = 0; i1 < 15; i1++) {
      b_PHIk_hat[i1 + 15 * b_i] = PHIk_hat[i1 + 15 * b_i] + dv8[i1 + 15 * b_i] *
        dt;
    }
  }

  for (b_i = 0; b_i < 6; b_i++) {
    for (i1 = 0; i1 < 9; i1++) {
      G[i1 + 15 * b_i] = matCbn[i1 + 9 * b_i];
      G[i1 + 15 * (b_i + 6)] = 0.0;
    }
  }

  for (b_i = 0; b_i < 12; b_i++) {
    for (i1 = 0; i1 < 6; i1++) {
      G[(i1 + 15 * b_i) + 9] = iv1[i1 + 6 * b_i];
    }
  }

  /*  Q:P~x  Q~12=>GQG'~15=>Qk~15 */
  /* 		15x15*15x12*12x12*12x15 */
  for (b_i = 0; b_i < 15; b_i++) {
    for (i1 = 0; i1 < 12; i1++) {
      c_PHIk_hat[b_i + 15 * i1] = 0.0;
      for (i2 = 0; i2 < 15; i2++) {
        c_PHIk_hat[b_i + 15 * i1] += b_PHIk_hat[b_i + 15 * i2] * G[i2 + 15 * i1];
      }
    }

    for (i1 = 0; i1 < 12; i1++) {
      d_PHIk_hat[b_i + 15 * i1] = 0.0;
      for (i2 = 0; i2 < 12; i2++) {
        d_PHIk_hat[b_i + 15 * i1] += c_PHIk_hat[b_i + 15 * i2] * Q[i2 + 12 * i1];
      }
    }

    for (i1 = 0; i1 < 15; i1++) {
      PHIk_hat[b_i + 15 * i1] = 0.0;
      for (i2 = 0; i2 < 12; i2++) {
        PHIk_hat[b_i + 15 * i1] += d_PHIk_hat[b_i + 15 * i2] * G[i1 + 15 * i2];
      }
    }

    for (i1 = 0; i1 < 12; i1++) {
      b_G[b_i + 15 * i1] = 0.0;
      for (i2 = 0; i2 < 12; i2++) {
        b_G[b_i + 15 * i1] += G[b_i + 15 * i2] * Q[i2 + 12 * i1];
      }
    }

    for (i1 = 0; i1 < 15; i1++) {
      e_PHIk_hat[b_i + 15 * i1] = 0.0;
      for (i2 = 0; i2 < 15; i2++) {
        e_PHIk_hat[b_i + 15 * i1] += PHIk_hat[b_i + 15 * i2] * b_PHIk_hat[i1 +
          15 * i2];
      }

      c_G[b_i + 15 * i1] = 0.0;
      for (i2 = 0; i2 < 12; i2++) {
        c_G[b_i + 15 * i1] += b_G[b_i + 15 * i2] * G[i1 + 15 * i2];
      }
    }
  }

  for (b_i = 0; b_i < 15; b_i++) {
    for (i1 = 0; i1 < 15; i1++) {
      Qk[i1 + 15 * b_i] = 0.5 * (e_PHIk_hat[i1 + 15 * b_i] + c_G[i1 + 15 * b_i])
        * dt;
    }
  }

  if (gpsflag) {
    /* co GPS moi, ko co INS */
    /*  measurements */
    /* ------------- */
    b_M[0] = M + h;
    b_M[1] = (N + h) * cos_lat;
    b_M[2] = 1.0;
    b_M[3] = 1.0;
    b_M[4] = 1.0;
    b_M[5] = 1.0;
    diag(b_M, Hksub);

    /* dlat_edit */
    /* dlon_edit */
    /* dh */
    /* dVN */
    /* dVE */
    /* dVD */
    for (b_i = 0; b_i < 6; b_i++) {
      for (i1 = 0; i1 < 6; i1++) {
        c_Hk[i1 + 6 * b_i] = Hksub[i1 + 6 * b_i];
      }
    }

    for (b_i = 0; b_i < 9; b_i++) {
      for (i1 = 0; i1 < 6; i1++) {
        c_Hk[i1 + 6 * (b_i + 6)] = 0.0;
      }
    }

    /* 6x15 */
    /*  R~z  R~6 */
    /* ********************* */
    /* STEP1: measure_Update */
    /* ********************* */
    for (b_i = 0; b_i < 15; b_i++) {
      for (i1 = 0; i1 < 6; i1++) {
        K[b_i + 15 * i1] = 0.0;
        for (i2 = 0; i2 < 15; i2++) {
          K[b_i + 15 * i1] += Pk_1[b_i + 15 * i2] * c_Hk[i1 + 6 * i2];
        }
      }
    }

    for (b_i = 0; b_i < 6; b_i++) {
      for (i1 = 0; i1 < 15; i1++) {
        e_Hk[b_i + 6 * i1] = 0.0;
        for (i2 = 0; i2 < 15; i2++) {
          e_Hk[b_i + 6 * i1] += c_Hk[b_i + 6 * i2] * Pk_1[i2 + 15 * i1];
        }
      }

      for (i1 = 0; i1 < 6; i1++) {
        x = 0.0;
        for (i2 = 0; i2 < 15; i2++) {
          x += e_Hk[b_i + 6 * i2] * c_Hk[i1 + 6 * i2];
        }

        d_Hk[b_i + 6 * i1] = x + R[b_i + 6 * i1];
      }
    }

    b_mrdivide(K, d_Hk);

    /* 15x15*15x6*(6x15*15x15*15x6+6*6)^-1 */
    b_lat[0] = lat - zG[1];
    b_lat[1] = lon - zG[2];
    b_lat[2] = h - zG[3];
    b_lat[3] = vn[0] - zG[4];
    b_lat[4] = vn[1] - zG[5];
    b_lat[5] = vn[2] - zG[6];
    for (b_i = 0; b_i < 6; b_i++) {
      b_M[b_i] = 0.0;
      for (i1 = 0; i1 < 6; i1++) {
        b_M[b_i] += Hksub[b_i + 6 * i1] * b_lat[i1];
      }

      h_Hk[b_i] = 0.0;
      for (i1 = 0; i1 < 15; i1++) {
        h_Hk[b_i] += c_Hk[b_i + 6 * i1] * xk_1[i1];
      }

      b_Hksub[b_i] = b_M[b_i] - h_Hk[b_i];
    }

    b_eye(PHIk_hat);
    for (b_i = 0; b_i < 15; b_i++) {
      x = 0.0;
      for (i1 = 0; i1 < 6; i1++) {
        x += K[b_i + 15 * i1] * b_Hksub[i1];
      }

      xk_1[b_i] += x;
      for (i1 = 0; i1 < 15; i1++) {
        x = 0.0;
        for (i2 = 0; i2 < 6; i2++) {
          x += K[b_i + 15 * i2] * c_Hk[i2 + 6 * i1];
        }

        o_a[b_i + 15 * i1] = PHIk_hat[b_i + 15 * i1] - x;
      }

      for (i1 = 0; i1 < 15; i1++) {
        n_a[b_i + 15 * i1] = 0.0;
        for (i2 = 0; i2 < 15; i2++) {
          n_a[b_i + 15 * i1] += o_a[b_i + 15 * i2] * Pk_1[i2 + 15 * i1];
        }
      }
    }

    for (b_i = 0; b_i < 15; b_i++) {
      memcpy(&Pk_1[b_i * 15], &n_a[b_i * 15], 15U * sizeof(double));
    }

    for (b_i = 0; b_i < 15; b_i++) {
      for (i1 = 0; i1 < 15; i1++) {
        d_Pk_1[i1 + 15 * b_i] = (Pk_1[i1 + 15 * b_i] + Pk_1[b_i + 15 * i1]) /
          2.0;
      }
    }

    for (b_i = 0; b_i < 15; b_i++) {
      memcpy(&Pk_1[b_i * 15], &d_Pk_1[b_i * 15], 15U * sizeof(double));
    }

    /* update xk with d_xk */
    /* 	%{ */
    /* 	dr = [xk_1(1);xk_1(2);xk_1(3)]; */
    /* 	dv = [xk_1(4);xk_1(5);xk_1(6)]; */
    /* 	de = [xk_1(7);xk_1(8);xk_1(9)]; */
    /* 	%} */
    b_xk_1[0] = xk_1[0];
    b_xk_1[1] = xk_1[1];
    b_xk_1[2] = xk_1[2];
    c_xk_1[0] = xk_1[3];
    c_xk_1[1] = xk_1[4];
    c_xk_1[2] = xk_1[5];
    for (b_i = 0; b_i < 3; b_i++) {
      wie_n[b_i] -= b_xk_1[b_i];
      vn[b_i] -= c_xk_1[b_i];
    }

    eye(Cbn_);
    d_xk_1[0] = xk_1[6];
    d_xk_1[1] = xk_1[7];
    d_xk_1[2] = xk_1[8];
    skew_mat3(d_xk_1, dv4);
    for (b_i = 0; b_i < 9; b_i++) {
      Cbn_[b_i] += dv4[b_i];
    }

    for (b_i = 0; b_i < 3; b_i++) {
      for (i1 = 0; i1 < 3; i1++) {
        b[b_i + 3 * i1] = 0.0;
        for (i2 = 0; i2 < 3; i2++) {
          b[b_i + 3 * i1] += Cbn_[b_i + 3 * i2] * Cbn[i2 + 3 * i1];
        }
      }
    }

    for (b_i = 0; b_i < 3; b_i++) {
      for (i1 = 0; i1 < 3; i1++) {
        Cbn[i1 + 3 * b_i] = b[i1 + 3 * b_i];
      }
    }

    memcpy(&c_Cbn[0], &Cbn[0], 9U * sizeof(double));
    normC(c_Cbn, Cbn);
    PVA[0] = zI[0];
    for (b_i = 0; b_i < 3; b_i++) {
      PVA[b_i + 1] = wie_n[b_i];
      PVA[b_i + 4] = vn[b_i];
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

    /* reset state d_xk after update xk */
    memset(&xk_1[0], 0, 9U * sizeof(double));

    /* ********************** */
    /* STEP2: time_Prediction */
    /* ********************** */
    for (b_i = 0; b_i < 15; b_i++) {
      f_PHIk_hat[b_i] = 0.0;
      for (i1 = 0; i1 < 15; i1++) {
        f_PHIk_hat[b_i] += b_PHIk_hat[b_i + 15 * i1] * xk_1[i1];
      }
    }

    for (b_i = 0; b_i < 15; b_i++) {
      xk_1[b_i] = f_PHIk_hat[b_i];
      for (i1 = 0; i1 < 15; i1++) {
        PHIk_hat[b_i + 15 * i1] = 0.0;
        for (i2 = 0; i2 < 15; i2++) {
          PHIk_hat[b_i + 15 * i1] += b_PHIk_hat[b_i + 15 * i2] * Pk_1[i2 + 15 *
            i1];
        }
      }
    }

    for (b_i = 0; b_i < 15; b_i++) {
      for (i1 = 0; i1 < 15; i1++) {
        x = 0.0;
        for (i2 = 0; i2 < 15; i2++) {
          x += PHIk_hat[b_i + 15 * i2] * b_PHIk_hat[i1 + 15 * i2];
        }

        Pk_1[b_i + 15 * i1] = x + Qk[b_i + 15 * i1];
      }
    }
  } else {
    /*  measurements */
    /* ------------- */
    /* z=dVy, dVz */
    for (b_i = 0; b_i < 3; b_i++) {
      wen_n[b_i] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        wen_n[b_i] += Cbn[i1 + 3 * b_i] * vn[i1];
      }

      for (i1 = 0; i1 < 2; i1++) {
        Hk[i1 + (b_i << 1)] = 0.0;
      }
    }

    Hk[6] = Cbn[3];
    Hk[8] = Cbn[4];
    Hk[10] = Cbn[5];
    Hk[12] = -vn[2] * Cbn[4] + vn[1] * Cbn[5];
    Hk[14] = vn[2] * Cbn[3] - vn[0] * Cbn[5];
    Hk[16] = -vn[1] * Cbn[3] + vn[0] * Cbn[4];
    Hk[7] = Cbn[6];
    Hk[9] = Cbn[7];
    Hk[11] = Cbn[8];
    Hk[13] = -vn[2] * Cbn[7] + vn[1] * Cbn[8];
    Hk[15] = vn[2] * Cbn[6] - vn[0] * Cbn[8];
    Hk[17] = -vn[1] * Cbn[6] + vn[0] * Cbn[7];
    for (b_i = 0; b_i < 6; b_i++) {
      for (i1 = 0; i1 < 2; i1++) {
        Hk[i1 + ((b_i + 9) << 1)] = 0.0;
      }
    }

    /*  R~z  R~2 */
    for (b_i = 0; b_i < 2; b_i++) {
      b_Hk[b_i] = 0.5;
    }

    b_diag(b_Hk, Rk);

    /* ********************* */
    /* STEP1: measure_Update */
    /* ********************* */
    for (b_i = 0; b_i < 15; b_i++) {
      for (i1 = 0; i1 < 2; i1++) {
        b_Pk_1[b_i + 15 * i1] = 0.0;
        for (i2 = 0; i2 < 15; i2++) {
          b_Pk_1[b_i + 15 * i1] += Pk_1[b_i + 15 * i2] * Hk[i1 + (i2 << 1)];
        }
      }
    }

    for (b_i = 0; b_i < 2; b_i++) {
      for (i1 = 0; i1 < 15; i1++) {
        g_Hk[b_i + (i1 << 1)] = 0.0;
        for (i2 = 0; i2 < 15; i2++) {
          g_Hk[b_i + (i1 << 1)] += Hk[b_i + (i2 << 1)] * Pk_1[i2 + 15 * i1];
        }
      }

      for (i1 = 0; i1 < 2; i1++) {
        x = 0.0;
        for (i2 = 0; i2 < 15; i2++) {
          x += g_Hk[b_i + (i2 << 1)] * Hk[i1 + (i2 << 1)];
        }

        f_Hk[b_i + (i1 << 1)] = x + Rk[b_i + (i1 << 1)];
      }
    }

    mrdivide(b_Pk_1, f_Hk, b_K);

    /* 15x15*15x6*(6x15*15x15*15x6+6*6)^-1 */
    b_wen_n[0] = wen_n[1];
    b_wen_n[1] = wen_n[2];
    for (b_i = 0; b_i < 2; b_i++) {
      b_Hk[b_i] = 0.0;
      for (i1 = 0; i1 < 15; i1++) {
        b_Hk[b_i] += Hk[b_i + (i1 << 1)] * xk_1[i1];
      }

      c_wen_n[b_i] = b_wen_n[b_i] - b_Hk[b_i];
    }

    b_eye(PHIk_hat);
    for (b_i = 0; b_i < 15; b_i++) {
      x = 0.0;
      for (i1 = 0; i1 < 2; i1++) {
        x += b_K[b_i + 15 * i1] * c_wen_n[i1];
      }

      xk_1[b_i] += x;
      for (i1 = 0; i1 < 15; i1++) {
        x = 0.0;
        for (i2 = 0; i2 < 2; i2++) {
          x += b_K[b_i + 15 * i2] * Hk[i2 + (i1 << 1)];
        }

        o_a[b_i + 15 * i1] = PHIk_hat[b_i + 15 * i1] - x;
      }

      for (i1 = 0; i1 < 15; i1++) {
        m_a[b_i + 15 * i1] = 0.0;
        for (i2 = 0; i2 < 15; i2++) {
          m_a[b_i + 15 * i1] += o_a[b_i + 15 * i2] * Pk_1[i2 + 15 * i1];
        }
      }
    }

    for (b_i = 0; b_i < 15; b_i++) {
      memcpy(&Pk_1[b_i * 15], &m_a[b_i * 15], 15U * sizeof(double));
    }

    for (b_i = 0; b_i < 15; b_i++) {
      for (i1 = 0; i1 < 15; i1++) {
        c_Pk_1[i1 + 15 * b_i] = (Pk_1[i1 + 15 * b_i] + Pk_1[b_i + 15 * i1]) /
          2.0;
      }
    }

    for (b_i = 0; b_i < 15; b_i++) {
      memcpy(&Pk_1[b_i * 15], &c_Pk_1[b_i * 15], 15U * sizeof(double));
    }

    /* update xk with d_xk */
    /* 	%{ */
    /* 	dr = [xk_1(1);xk_1(2);xk_1(3)]; */
    /* 	dv = [xk_1(4);xk_1(5);xk_1(6)]; */
    /* 	de = [xk_1(7);xk_1(8);xk_1(9)]; */
    /* 	%} */
    b_xk_1[0] = xk_1[0];
    b_xk_1[1] = xk_1[1];
    b_xk_1[2] = xk_1[2];
    c_xk_1[0] = xk_1[3];
    c_xk_1[1] = xk_1[4];
    c_xk_1[2] = xk_1[5];
    for (b_i = 0; b_i < 3; b_i++) {
      wie_n[b_i] -= b_xk_1[b_i];
      vn[b_i] -= c_xk_1[b_i];
    }

    eye(Cbn_);
    d_xk_1[0] = xk_1[6];
    d_xk_1[1] = xk_1[7];
    d_xk_1[2] = xk_1[8];
    skew_mat3(d_xk_1, dv4);
    for (b_i = 0; b_i < 9; b_i++) {
      Cbn_[b_i] += dv4[b_i];
    }

    for (b_i = 0; b_i < 3; b_i++) {
      for (i1 = 0; i1 < 3; i1++) {
        b[b_i + 3 * i1] = 0.0;
        for (i2 = 0; i2 < 3; i2++) {
          b[b_i + 3 * i1] += Cbn_[b_i + 3 * i2] * Cbn[i2 + 3 * i1];
        }
      }
    }

    for (b_i = 0; b_i < 3; b_i++) {
      for (i1 = 0; i1 < 3; i1++) {
        Cbn[i1 + 3 * b_i] = b[i1 + 3 * b_i];
      }
    }

    memcpy(&b_Cbn[0], &Cbn[0], 9U * sizeof(double));
    normC(b_Cbn, Cbn);
    PVA[0] = zI[0];
    for (b_i = 0; b_i < 3; b_i++) {
      PVA[b_i + 1] = wie_n[b_i];
      PVA[b_i + 4] = vn[b_i];
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

    /* reset state d_xk after update xk */
    memset(&xk_1[0], 0, 9U * sizeof(double));

    /* ********************** */
    /* STEP2: time_Prediction */
    /* ********************** */
    for (b_i = 0; b_i < 15; b_i++) {
      f_PHIk_hat[b_i] = 0.0;
      for (i1 = 0; i1 < 15; i1++) {
        f_PHIk_hat[b_i] += b_PHIk_hat[b_i + 15 * i1] * xk_1[i1];
      }
    }

    for (b_i = 0; b_i < 15; b_i++) {
      xk_1[b_i] = f_PHIk_hat[b_i];
      for (i1 = 0; i1 < 15; i1++) {
        PHIk_hat[b_i + 15 * i1] = 0.0;
        for (i2 = 0; i2 < 15; i2++) {
          PHIk_hat[b_i + 15 * i1] += b_PHIk_hat[b_i + 15 * i2] * Pk_1[i2 + 15 *
            i1];
        }
      }
    }

    for (b_i = 0; b_i < 15; b_i++) {
      for (i1 = 0; i1 < 15; i1++) {
        x = 0.0;
        for (i2 = 0; i2 < 15; i2++) {
          x += PHIk_hat[b_i + 15 * i2] * b_PHIk_hat[i1 + 15 * i2];
        }

        Pk_1[b_i + 15 * i1] = x + Qk[b_i + 15 * i1];
      }
    }
  }
}

/*
 * File trailer for insgps_v8_0.c
 *
 * [EOF]
 */
