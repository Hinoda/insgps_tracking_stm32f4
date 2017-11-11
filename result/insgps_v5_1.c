/*
 * File: insgps_v5_1.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 09-Nov-2017 10:12:01
 */

/* Include Files */
#include "initialize.h"
#include "genDataGetProcess.h"
#include "Cbn_31.h"
#include "insgps_v5_1.h"
#include "normC.h"
#include "skew_mat3.h"
#include "eye.h"
#include "mrdivide.h"
#include "diag.h"


/* Function Definitions */

/*
 * mecha
 * Arguments    : const float zI[10]
 *                const float zG[7]
 *                float gpsflag
 *                float dt
 *                float g0
 *                float a
 *                float e
 *                float we
 *                const float Q[144]
 *                const float R[36]
 *                float PVA[10]
 *                float bias[6]
 *                float Pk_1[225]
 *                float xk_1[15]
 * Return Type  : void
 */
void insgps_v5_1(const float zI[10], const float zG[7], int gpsflag, float
                 dt, float g0, float a, float e, float we, const float Q
                 [144], const float R[36], float PVA[10], float bias[6],
                 float Pk_1[225], float xk_1[15])
{
  float vn_[3];
  float b_PVA[3];
  float Cbn_[9];
  float x;
  float N_;
  float M_;
  float RMN;
  float y;
  float g;
  float wen_n[3];
  float wie_n[3];
  float b[9];
  float b_zI[3];
  float dv4[9];
  int i1;
  float dv5[9];
  float dv6[9];
  int i2;
  float Cbn[9];
  int i3;
  float c_zI[3];
  float dv7[3];
  float fn[3];
  float vn[3];
  float h;
  float lat;
  float N;
  float M;
  float lon;
  float b_a;
  float b_x;
  float c_a;
  float d_a;
  float c_x;
  float e_a;
  float f_a;
  float g_a;
  float h_a;
  float i_a;
  float j_a;
  float d_x;
  float k_a;
  float G[225];
  float matCbn[54];
  float dv8[225];
  static const signed char iv0[9] = { 0, 0, 0, 0, 0, -1, 0, 0, 0 };

  float PHIk_hat[225];
  static const float dv9[90] = { -0, -0, -0, -0, -0, -0, -0, -0,
    -0, -0, -0, -0, -0, -0, -0, -0, -0, -0, -0, -0, -0,
    -0, -0, -0, -0, -0, -0, -0, -0, -0, -0, -0, -0, -0,
    -0, -0, -0, -0, -0, -0, -0, -0, -0, -0, -0, -0, -0,
    -0, -0, -0, -0, -0, -0, -0, -5.5555555555555558E-5, -0, -0,
    -0, -0, -0, -0, -5.5555555555555558E-5, -0, -0, -0, -0, -0,
    -0, -5.5555555555555558E-5, -0, -0, -0, -0, -0, -0,
    -5.5555555555555558E-5, -0, -0, -0, -0, -0, -0,
    -5.5555555555555558E-5, -0, -0, -0, -0, -0, -0,
    -5.5555555555555558E-5 };

  float b_G[180];
  static const signed char iv1[72] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1 };

  float b_M[3];
  float Hk[90];
  float l_a[6];
  float B[36];
  float b_PHIk_hat[180];
  float c_PHIk_hat[180];
  float c_G[180];
  float d_PHIk_hat[225];
  float d_G[225];
  float Qk[225];
  static const signed char iv2[45] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0 };

  float e_PHIk_hat[15];
  float b_Hk[90];
  float c_Hk[36];
  float b_R[36];
  float K[90];
  float c_M[6];
  float d_M[6];
  float m_a[225];
  float b_Pk_1[225];
  float n_a[225];
  float b_xk_1[3];
  float c_xk_1[3];
  float d_xk_1[3];
  float b_Cbn[9];

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
  /* params */
  x = sin(PVA[1]);
  N_ = a / sqrt(1 - e * e * (x * x));
  x = sin(PVA[1]);
  M_ = a * (1 - e * e) / pow(1 - e * e * (x * x), 1.5);
  RMN = sqrt(N_ * M_);
  y = RMN / (RMN + PVA[3]);
  g = g0 * (y * y);

  /* gn */
  wen_n[0] = PVA[5] / (N_ + PVA[3]);
  wen_n[1] = -PVA[4] / (M_ + PVA[3]);
  wen_n[2] = -PVA[5] * tan(PVA[1]) / (N_ + PVA[3]);
  wie_n[0] = we * cos(PVA[1]);
  wie_n[1] = 0;
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
      dv4[i1 + 3 * i2] = 0;
      y = 0;
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
      b[i2 + 3 * i1] = (Cbn[i2 + 3 * i1] + Cbn_[i2 + 3 * i1]) / 2;
    }
  }

  c_zI[0] = zI[7] - bias[0];
  c_zI[1] = zI[8] - bias[1];
  c_zI[2] = zI[9] - bias[2];

  /* ** vn */
  skew_mat3(wen_n, dv4);
  skew_mat3(wie_n, dv5);
  dv7[0] = 0;
  dv7[1] = 0;
  dv7[2] = g;
  for (i1 = 0; i1 < 3; i1++) {
    fn[i1] = 0;
    for (i2 = 0; i2 < 3; i2++) {
      dv6[i2 + 3 * i1] = dv4[i2 + 3 * i1] + 2 * dv5[i2 + 3 * i1];
      fn[i1] += b[i1 + 3 * i2] * c_zI[i2];
    }
  }

  for (i1 = 0; i1 < 3; i1++) {
    y = 0;
    for (i2 = 0; i2 < 3; i2++) {
      y += dv6[i1 + 3 * i2] * vn_[i2];
    }

    b_PVA[i1] = (fn[i1] + dv7[i1]) - y;
    vn[i1] = vn_[i1] + b_PVA[i1] * dt;
  }

  /* % Position */
  h = PVA[3] - dt / 2 * (PVA[6] + vn[2]);
  lat = PVA[1] + dt / 2 * (PVA[4] / (M_ + PVA[3]) + vn[0] / (M_ + h));
  x = sin(lat);
  N = a / sqrt(1 - e * e * (x * x));
  x = sin(lat);
  M = a * (1 - e * e) / pow(1 - e * e * (x * x), 1.5);
  lon = PVA[2] + dt / 2 * (PVA[5] / ((N_ + PVA[3]) * cos(PVA[1])) + vn[1] /
    ((N + h) * cos(lat)));

  /* ** rn */
  wen_n[0] = lat;
  wen_n[1] = lon;
  wen_n[2] = h;

  /* ****************************************************************** */
  /* 	Extended Kalman */
  /* ------------------------------------------------ */
  wie_n[0] = we * cos(lat) + vn[1] / (N + h);
  wie_n[1] = -vn[0] / (M + h);
  wie_n[2] = -we * sin(lat) - vn[1] * tan(lat) / (N + h);

  /*  linearize */
  /* --------- */
  y = M + h;
  x = cos(lat);
  b_a = N + h;

  /* --------- */
  /* --------- */
  b_x = cos(lat);
  c_a = M + h;
  d_a = N + h;
  c_x = cos(lat);
  e_a = N + h;
  f_a = N + h;
  g_a = N + h;
  h_a = M + h;

  /* --------- */
  /* --------- */
  i_a = N + h;
  j_a = M + h;
  d_x = cos(lat);
  k_a = N + h;

  /* --------- */
  /* ------------------------------------------------ */
  for (i1 = 0; i1 < 6; i1++) {
    for (i2 = 0; i2 < 3; i2++) {
      matCbn[i2 + 9 * i1] = 0;
    }
  }

  for (i1 = 0; i1 < 3; i1++) {
    for (i2 = 0; i2 < 3; i2++) {
      matCbn[(i2 + 9 * i1) + 3] = Cbn[i2 + 3 * i1];
      matCbn[(i2 + 9 * (i1 + 3)) + 3] = 0;
      matCbn[(i2 + 9 * i1) + 6] = 0;
      matCbn[(i2 + 9 * (i1 + 3)) + 6] = -Cbn[i2 + 3 * i1];
    }
  }

  b_eye(G);
  dv8[0] = 0;
  dv8[15] = 0;
  dv8[30] = -vn[0] / (y * y);
  dv8[45] = 1 / (M + h);
  dv8[60] = 0;
  dv8[75] = 0;
  dv8[90] = 0;
  dv8[105] = 0;
  dv8[120] = 0;
  dv8[1] = vn[1] * sin(lat) / ((N + h) * (x * x));
  dv8[16] = 0;
  dv8[31] = -vn[1] / (b_a * b_a * cos(lat));
  dv8[46] = 0;
  dv8[61] = 1 / ((N + h) * cos(lat));
  dv8[76] = 0;
  dv8[91] = 0;
  dv8[106] = 0;
  dv8[121] = 0;
  for (i1 = 0; i1 < 9; i1++) {
    dv8[2 + 15 * i1] = iv0[i1];
  }

  dv8[3] = -2 * vn[1] * we * cos(lat) - vn[1] * vn[1] / ((N + h) * (b_x * b_x));
  dv8[18] = 0;
  dv8[33] = -vn[0] * vn[2] / (c_a * c_a) + vn[1] * vn[1] * tan(lat) / (d_a * d_a);
  dv8[48] = vn[2] / (M + h);
  dv8[63] = -2 * we * sin(lat) - 2 * vn[1] * tan(lat) / (N + h);
  dv8[78] = vn[0] / (M + h);
  dv8[93] = 0;
  dv8[108] = -fn[2];
  dv8[123] = fn[1];
  dv8[4] = 2 * we * (vn[0] * cos(lat) - vn[2] * sin(lat)) + vn[1] * vn[0] /
    ((N + h) * (c_x * c_x));
  dv8[19] = 0;
  dv8[34] = -vn[1] * vn[2] / (e_a * e_a) - vn[0] * vn[1] * tan(lat) / (f_a * f_a);
  dv8[49] = 2 * we * sin(lat) + vn[1] * tan(lat) / (N + h);
  dv8[64] = (vn[2] + vn[0] * tan(lat)) / (N + h);
  dv8[79] = 2 * we * cos(lat) + vn[1] / (N + h);
  dv8[94] = fn[2];
  dv8[109] = 0;
  dv8[124] = -fn[0];
  dv8[5] = 2 * vn[1] * we * sin(lat);
  dv8[20] = 0;
  dv8[35] = (vn[1] * vn[1] / (g_a * g_a) + vn[0] * vn[0] / (h_a * h_a)) - 2 *
    g / (sqrt(M * N) + h);
  dv8[50] = -2 * vn[0] / (M + h);
  dv8[65] = -2 * we * cos(lat) - 2 * vn[1] / (N + h);
  dv8[80] = 0;
  dv8[95] = -fn[1];
  dv8[110] = fn[0];
  dv8[125] = 0;
  dv8[6] = -we * sin(lat);
  dv8[21] = 0;
  dv8[36] = -vn[1] / (i_a * i_a);
  dv8[51] = 0;
  dv8[66] = 1 / (N + h);
  dv8[81] = 0;
  dv8[96] = 0;
  dv8[111] = wie_n[2];
  dv8[126] = -wie_n[1];
  dv8[7] = 0;
  dv8[22] = 0;
  dv8[37] = vn[0] / (j_a * j_a);
  dv8[52] = -1 / (M + h);
  dv8[67] = 0;
  dv8[82] = 0;
  dv8[97] = -wie_n[2];
  dv8[112] = 0;
  dv8[127] = wie_n[0];
  dv8[8] = -we * cos(lat) - vn[1] / ((N + h) * (d_x * d_x));
  dv8[23] = 0;
  dv8[38] = vn[1] * tan(lat) / (k_a * k_a);
  dv8[53] = 0;
  dv8[68] = -tan(lat) / (N + h);
  dv8[83] = 0;
  dv8[98] = wie_n[1];
  dv8[113] = -wie_n[0];
  dv8[128] = 0;
  for (i1 = 0; i1 < 6; i1++) {
    memcpy(&dv8[i1 * 15 + 135], &matCbn[i1 * 9], 9U * sizeof(float));
  }

  for (i1 = 0; i1 < 15; i1++) {
    for (i2 = 0; i2 < 6; i2++) {
      dv8[(i2 + 15 * i1) + 9] = dv9[i2 + 6 * i1];
    }

    for (i2 = 0; i2 < 15; i2++) {
      PHIk_hat[i2 + 15 * i1] = G[i2 + 15 * i1] + dv8[i2 + 15 * i1] * dt;
    }
  }

  for (i1 = 0; i1 < 6; i1++) {
    for (i2 = 0; i2 < 9; i2++) {
      b_G[i2 + 15 * i1] = matCbn[i2 + 9 * i1];
      b_G[i2 + 15 * (i1 + 6)] = 0;
    }
  }

  for (i1 = 0; i1 < 12; i1++) {
    for (i2 = 0; i2 < 6; i2++) {
      b_G[(i2 + 15 * i1) + 9] = iv1[i2 + 6 * i1];
    }
  }

  /*  measurements */
  /* ------------- */
  /* dlat_edit */
  /* dlon_edit */
  /* dh */
  /* dVN */
  /* dVE */
  /* dVD */
  b_M[0] = M + h;
  b_M[1] = (N + h) * cos(lat);
  b_M[2] = 1;
  b_diag(b_M, Cbn_);
  for (i1 = 0; i1 < 3; i1++) {
    for (i2 = 0; i2 < 3; i2++) {
      Hk[i2 + 6 * i1] = Cbn_[i2 + 3 * i1];
      Hk[i2 + 6 * (i1 + 3)] = 0;
    }
  }

  for (i1 = 0; i1 < 9; i1++) {
    for (i2 = 0; i2 < 3; i2++) {
      Hk[i2 + 6 * (i1 + 6)] = 0;
    }
  }

  /* 6x15 */
  /*  covariance filter params */
  /* ----------- */
  /*  R~z  R~6 */
  y = M + h;
  b_a = N + h;
  x = cos(lat);
  l_a[0] = y * y;
  l_a[1] = b_a * b_a * (x * x);
  l_a[2] = 1;
  l_a[3] = 1;
  l_a[4] = 1;
  l_a[5] = 1;
  diag(l_a, B);

  /*  Q:P~x  Q~12=>GQG'~15=>Qk~15 */
  /* 		15x15*15x12*12x12*12x15 */
  for (i1 = 0; i1 < 15; i1++) {
    for (i2 = 0; i2 < 3; i2++) {
      Hk[(i2 + 6 * i1) + 3] = iv2[i2 + 3 * i1];
    }

    for (i2 = 0; i2 < 12; i2++) {
      b_PHIk_hat[i1 + 15 * i2] = 0;
      for (i3 = 0; i3 < 15; i3++) {
        b_PHIk_hat[i1 + 15 * i2] += PHIk_hat[i1 + 15 * i3] * b_G[i3 + 15 * i2];
      }
    }

    for (i2 = 0; i2 < 12; i2++) {
      c_PHIk_hat[i1 + 15 * i2] = 0;
      c_G[i1 + 15 * i2] = 0;
      for (i3 = 0; i3 < 12; i3++) {
        c_PHIk_hat[i1 + 15 * i2] += b_PHIk_hat[i1 + 15 * i3] * Q[i3 + 12 * i2];
        c_G[i1 + 15 * i2] += b_G[i1 + 15 * i3] * Q[i3 + 12 * i2];
      }
    }

    for (i2 = 0; i2 < 15; i2++) {
      G[i1 + 15 * i2] = 0;
      d_PHIk_hat[i1 + 15 * i2] = 0;
      for (i3 = 0; i3 < 12; i3++) {
        G[i1 + 15 * i2] += c_G[i1 + 15 * i3] * b_G[i2 + 15 * i3];
        d_PHIk_hat[i1 + 15 * i2] += c_PHIk_hat[i1 + 15 * i3] * b_G[i2 + 15 * i3];
      }
    }

    for (i2 = 0; i2 < 15; i2++) {
      d_G[i1 + 15 * i2] = 0;
      for (i3 = 0; i3 < 15; i3++) {
        d_G[i1 + 15 * i2] += G[i1 + 15 * i3] * PHIk_hat[i2 + 15 * i3];
      }
    }
  }

  for (i1 = 0; i1 < 15; i1++) {
    for (i2 = 0; i2 < 15; i2++) {
      Qk[i2 + 15 * i1] = 0.5 * (d_PHIk_hat[i2 + 15 * i1] + d_G[i2 + 15 * i1]) *
        dt;
    }
  }

  if (gpsflag == 1) {
    /* co GPS moi, ko co INS */
    /* STEP1: measure_Update */
    /* ********************* */
    for (i1 = 0; i1 < 15; i1++) {
      for (i2 = 0; i2 < 6; i2++) {
        K[i1 + 15 * i2] = 0;
        for (i3 = 0; i3 < 15; i3++) {
          K[i1 + 15 * i2] += Pk_1[i1 + 15 * i3] * Hk[i2 + 6 * i3];
        }
      }
    }

    for (i1 = 0; i1 < 6; i1++) {
      for (i2 = 0; i2 < 15; i2++) {
        b_Hk[i1 + 6 * i2] = 0;
        for (i3 = 0; i3 < 15; i3++) {
          b_Hk[i1 + 6 * i2] += Hk[i1 + 6 * i3] * Pk_1[i3 + 15 * i2];
        }
      }

      for (i2 = 0; i2 < 6; i2++) {
        c_Hk[i1 + 6 * i2] = 0;
        for (i3 = 0; i3 < 15; i3++) {
          c_Hk[i1 + 6 * i2] += b_Hk[i1 + 6 * i3] * Hk[i2 + 6 * i3];
        }

        b_R[i1 + 6 * i2] = 0;
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
    c_M[3] = vn[0] - zG[4];
    c_M[4] = vn[1] - zG[5];
    c_M[5] = vn[2] - zG[6];
    for (i1 = 0; i1 < 6; i1++) {
      l_a[i1] = 0;
      for (i2 = 0; i2 < 15; i2++) {
        l_a[i1] += Hk[i1 + 6 * i2] * xk_1[i2];
      }

      d_M[i1] = c_M[i1] - l_a[i1];
    }

    b_eye(G);
    for (i1 = 0; i1 < 15; i1++) {
      y = 0;
      for (i2 = 0; i2 < 6; i2++) {
        y += K[i1 + 15 * i2] * d_M[i2];
      }

      xk_1[i1] += y;
      for (i2 = 0; i2 < 15; i2++) {
        y = 0;
        for (i3 = 0; i3 < 6; i3++) {
          y += K[i1 + 15 * i3] * Hk[i3 + 6 * i2];
        }

        n_a[i1 + 15 * i2] = G[i1 + 15 * i2] - y;
      }

      for (i2 = 0; i2 < 15; i2++) {
        m_a[i1 + 15 * i2] = 0;
        for (i3 = 0; i3 < 15; i3++) {
          m_a[i1 + 15 * i2] += n_a[i1 + 15 * i3] * Pk_1[i3 + 15 * i2];
        }
      }
    }

    for (i1 = 0; i1 < 15; i1++) {
      memcpy(&Pk_1[i1 * 15], &m_a[i1 * 15], 15U * sizeof(float));
    }

    for (i1 = 0; i1 < 15; i1++) {
      for (i2 = 0; i2 < 15; i2++) {
        b_Pk_1[i2 + 15 * i1] = (Pk_1[i2 + 15 * i1] + Pk_1[i1 + 15 * i2]) / 2;
      }
    }

    for (i1 = 0; i1 < 15; i1++) {
      memcpy(&Pk_1[i1 * 15], &b_Pk_1[i1 * 15], 15U * sizeof(float));
    }

    /* update xk with d_xk */
    /*     %{ */
    /* 	dr = [xk_1(1);xk_1(2);xk_1(3)]; */
    /*     dv = [xk_1(4);xk_1(5);xk_1(6)]; */
    /*     de = [xk_1(7);xk_1(8);xk_1(9)]; */
    /* 	%} */
    b_xk_1[0] = xk_1[0];
    b_xk_1[1] = xk_1[1];
    b_xk_1[2] = xk_1[2];
    c_xk_1[0] = xk_1[3];
    c_xk_1[1] = xk_1[4];
    c_xk_1[2] = xk_1[5];
    for (i1 = 0; i1 < 3; i1++) {
      wen_n[i1] -= b_xk_1[i1];
      vn[i1] -= c_xk_1[i1];
    }

    eye(Cbn_);
    d_xk_1[0] = xk_1[6];
    d_xk_1[1] = xk_1[7];
    d_xk_1[2] = xk_1[8];
    skew_mat3(d_xk_1, dv4);
    for (i1 = 0; i1 < 9; i1++) {
      Cbn_[i1] += dv4[i1];
    }

    for (i1 = 0; i1 < 3; i1++) {
      for (i2 = 0; i2 < 3; i2++) {
        b[i1 + 3 * i2] = 0;
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

    memcpy(&b_Cbn[0], &Cbn[0], 9U * sizeof(float));
    normC(b_Cbn, Cbn);
    PVA[0] = zI[0];
    for (i1 = 0; i1 < 3; i1++) {
      PVA[i1 + 1] = wen_n[i1];
      PVA[i1 + 4] = vn[i1];
    }

    PVA[7] = atan2(Cbn[5], Cbn[8]);
    PVA[8] = -atan(Cbn[2] / sqrt(1 - Cbn[2] * Cbn[2]));
    PVA[9] = atan2(Cbn[1], Cbn[0]);

    /* 10x1 */
    bias[0] = xk_1[9];
    bias[1] = xk_1[10];
    bias[2] = xk_1[11];
    bias[3] = xk_1[12];
    bias[4] = xk_1[13];
    bias[5] = xk_1[14];

    /* reset state d_xk after update xk */
    memset(&xk_1[0], 0, 9U * sizeof(float));

    /* STEP2: time_Prediction */
    /* ********************** */
    for (i1 = 0; i1 < 15; i1++) {
      e_PHIk_hat[i1] = 0;
      for (i2 = 0; i2 < 15; i2++) {
        e_PHIk_hat[i1] += PHIk_hat[i1 + 15 * i2] * xk_1[i2];
      }
    }

    for (i1 = 0; i1 < 15; i1++) {
      xk_1[i1] = e_PHIk_hat[i1];
      for (i2 = 0; i2 < 15; i2++) {
        d_PHIk_hat[i1 + 15 * i2] = 0;
        for (i3 = 0; i3 < 15; i3++) {
          d_PHIk_hat[i1 + 15 * i2] += PHIk_hat[i1 + 15 * i3] * Pk_1[i3 + 15 * i2];
        }
      }
    }

    for (i1 = 0; i1 < 15; i1++) {
      for (i2 = 0; i2 < 15; i2++) {
        y = 0;
        for (i3 = 0; i3 < 15; i3++) {
          y += d_PHIk_hat[i1 + 15 * i3] * PHIk_hat[i2 + 15 * i3];
        }

        Pk_1[i1 + 15 * i2] = y + Qk[i1 + 15 * i2];
      }
    }
  } else {
    /* STEP: time_Prediction */
    /* ********************* */
    for (i1 = 0; i1 < 15; i1++) {
      e_PHIk_hat[i1] = 0;
      for (i2 = 0; i2 < 15; i2++) {
        e_PHIk_hat[i1] += PHIk_hat[i1 + 15 * i2] * xk_1[i2];
      }
    }

    for (i1 = 0; i1 < 15; i1++) {
      xk_1[i1] = e_PHIk_hat[i1];
      for (i2 = 0; i2 < 15; i2++) {
        d_PHIk_hat[i1 + 15 * i2] = 0;
        for (i3 = 0; i3 < 15; i3++) {
          d_PHIk_hat[i1 + 15 * i2] += PHIk_hat[i1 + 15 * i3] * Pk_1[i3 + 15 * i2];
        }
      }
    }

    for (i1 = 0; i1 < 15; i1++) {
      for (i2 = 0; i2 < 15; i2++) {
        y = 0;
        for (i3 = 0; i3 < 15; i3++) {
          y += d_PHIk_hat[i1 + 15 * i3] * PHIk_hat[i2 + 15 * i3];
        }

        Pk_1[i1 + 15 * i2] = y + Qk[i1 + 15 * i2];
      }
    }

    PVA[0] = zI[0];
    for (i1 = 0; i1 < 3; i1++) {
      PVA[i1 + 1] = wen_n[i1];
      PVA[i1 + 4] = vn[i1];
    }

    PVA[7] = atan2(Cbn[5], Cbn[8]);
    PVA[8] = -atan(Cbn[2] / sqrt(1 - Cbn[2] * Cbn[2]));
    PVA[9] = atan2(Cbn[1], Cbn[0]);

    /* 10x1 */
    bias[0] = xk_1[9];
    bias[1] = xk_1[10];
    bias[2] = xk_1[11];
    bias[3] = xk_1[12];
    bias[4] = xk_1[13];
    bias[5] = xk_1[14];
  }
}

/*
 * File trailer for insgps_v5_1.c
 *
 * [EOF]
 */
