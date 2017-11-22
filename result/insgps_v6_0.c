/*
 * File: insgps_v6_0.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 12-Nov-2017 15:05:52
 */

/* Include Files */
#include "Cbn_31.h"
#include "insgps_v6_0.h"
#include "normC.h"
#include "skew_mat3.h"
#include "eye.h"
#include "mrdivide.h"
#include "power.h"
#include "diag.h"
#include "initialize.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
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
void insgps_v6_0(const float zI[10], const float zG[7], bool gpsflag, float
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
  float sin_lat;
  float cos_lat;
  float tan_lat;
  float N;
  float M;
  float invNplushcos_lat2;
  float lon;
  float b_a;
  float c_a;
  float d_a;
  float e_a;
  float f_a;
  float g_a;
  float h_a;
  float i_a;
  float j_a;
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
  float b_PHIk_hat[180];
  static const signed char iv1[72] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1 };

  float c_PHIk_hat[180];
  float c_G[180];
  float d_PHIk_hat[225];
  float d_G[225];
  float Qk[225];
  float b_M[6];
  float Hk[30];
  float Hksub[36];
  float b_Hk[2];
  float c_Hk[90];
  float B[36];
  float Rk[4];
  float b_Pk_1[30];
  float d_Hk[90];
  float e_Hk[36];
  float f_Hk[4];
  float b_R[36];
  float K[90];
  float g_Hk[30];
  float b_K[30];
  float b_wen_n[2];
  float b_lat[6];
  float c_wen_n[2];
  float l_a[225];
  float h_Hk[6];
  float b_Hksub[6];
  float c_Pk_1[225];
  float m_a[225];
  float n_a[225];
  float d_Pk_1[225];
  float b_xk_1[3];
  float c_xk_1[3];
  float d_xk_1[3];
  float b_Cbn[9];
  float c_Cbn[9];
  float e_PHIk_hat[15];

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
  x = RMN / (RMN + PVA[3]);
  g = g0 * (x * x);

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
      x = 0;
      for (i3 = 0; i3 < 3; i3++) {
        dv4[i1 + 3 * i2] += dv6[i1 + 3 * i3] * Cbn_[i3 + 3 * i2];
        x += Cbn_[i1 + 3 * i3] * b[i3 + 3 * i2];
      }

      Cbn[i1 + 3 * i2] = x - dv4[i1 + 3 * i2] * dt;
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
    x = 0;
    for (i2 = 0; i2 < 3; i2++) {
      x += dv6[i1 + 3 * i2] * vn_[i2];
    }

    b_PVA[i1] = (fn[i1] + dv7[i1]) - x;
    vn[i1] = vn_[i1] + b_PVA[i1] * dt;
  }

  /* % Position */
  h = PVA[3] - dt / 2 * (PVA[6] + vn[2]);
  lat = PVA[1] + dt / 2 * (PVA[4] / (M_ + PVA[3]) + vn[0] / (M_ + h));
  sin_lat = sin(lat);
  cos_lat = cos(lat);
  tan_lat = sin_lat / cos_lat;
  N = a / sqrt(1 - e * e * (sin_lat * sin_lat));
  M = a * (1 - e * e) / pow(1 - e * e * (sin_lat * sin_lat), 1.5);
  invNplushcos_lat2 = 1 / ((N + h) * (cos_lat * cos_lat));
  lon = PVA[2] + dt / 2 * (PVA[5] / ((N_ + PVA[3]) * cos(PVA[1])) + vn[1] /
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
  g_a = N + h;
  h_a = M + h;

  /* --------- */
  /* --------- */
  i_a = N + h;
  j_a = M + h;
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
  dv8[30] = -vn[0] / (x * x);
  dv8[45] = 1 / (M + h);
  dv8[60] = 0;
  dv8[75] = 0;
  dv8[90] = 0;
  dv8[105] = 0;
  dv8[120] = 0;
  dv8[1] = vn[1] * sin_lat * invNplushcos_lat2;
  dv8[16] = 0;
  dv8[31] = -vn[1] / (b_a * b_a * cos_lat);
  dv8[46] = 0;
  dv8[61] = 1 / ((N + h) * cos_lat);
  dv8[76] = 0;
  dv8[91] = 0;
  dv8[106] = 0;
  dv8[121] = 0;
  for (i1 = 0; i1 < 9; i1++) {
    dv8[2 + 15 * i1] = iv0[i1];
  }

  dv8[3] = -2 * vn[1] * we * cos_lat - vn[1] * vn[1] * invNplushcos_lat2;
  dv8[18] = 0;
  dv8[33] = -vn[0] * vn[2] / (c_a * c_a) + vn[1] * vn[1] * tan_lat / (d_a * d_a);
  dv8[48] = vn[2] / (M + h);
  dv8[63] = -2 * we * sin_lat - 2 * vn[1] * tan_lat / (N + h);
  dv8[78] = vn[0] / (M + h);
  dv8[93] = 0;
  dv8[108] = -fn[2];
  dv8[123] = fn[1];
  dv8[4] = 2 * we * (vn[0] * cos_lat - vn[2] * sin_lat) + vn[1] * vn[0] *
    invNplushcos_lat2;
  dv8[19] = 0;
  dv8[34] = -vn[1] * vn[2] / (e_a * e_a) - vn[0] * vn[1] * tan_lat / (f_a * f_a);
  dv8[49] = 2 * we * sin_lat + vn[1] * tan_lat / (N + h);
  dv8[64] = (vn[2] + vn[0] * tan_lat) / (N + h);
  dv8[79] = 2 * we * cos_lat + vn[1] / (N + h);
  dv8[94] = fn[2];
  dv8[109] = 0;
  dv8[124] = -fn[0];
  dv8[5] = 2 * vn[1] * we * sin_lat;
  dv8[20] = 0;
  dv8[35] = (vn[1] * vn[1] / (g_a * g_a) + vn[0] * vn[0] / (h_a * h_a)) - 2 *
    g / (sqrt(M * N) + h);
  dv8[50] = -2 * vn[0] / (M + h);
  dv8[65] = -2 * we * cos_lat - 2 * vn[1] / (N + h);
  dv8[80] = 0;
  dv8[95] = -fn[1];
  dv8[110] = fn[0];
  dv8[125] = 0;
  dv8[6] = -we * sin_lat;
  dv8[21] = 0;
  dv8[36] = -vn[1] / (i_a * i_a);
  dv8[51] = 0;
  dv8[66] = 1 / (N + h);
  dv8[81] = 0;
  dv8[96] = 0;
  dv8[111] = wen_n[2];
  dv8[126] = -wen_n[1];
  dv8[7] = 0;
  dv8[22] = 0;
  dv8[37] = vn[0] / (j_a * j_a);
  dv8[52] = -1 / (M + h);
  dv8[67] = 0;
  dv8[82] = 0;
  dv8[97] = -wen_n[2];
  dv8[112] = 0;
  dv8[127] = wen_n[0];
  dv8[8] = -we * cos_lat - vn[1] * invNplushcos_lat2;
  dv8[23] = 0;
  dv8[38] = vn[1] * tan_lat / (k_a * k_a);
  dv8[53] = 0;
  dv8[68] = -tan_lat / (N + h);
  dv8[83] = 0;
  dv8[98] = wen_n[1];
  dv8[113] = -wen_n[0];
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

  /*  Q:P~x  Q~12=>GQG'~15=>Qk~15 */
  /* 		15x15*15x12*12x12*12x15 */
  for (i1 = 0; i1 < 15; i1++) {
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

  if (gpsflag) {
    /* co GPS moi, ko co INS */
    /*  measurements */
    /* ------------- */
    b_M[0] = M + h;
    b_M[1] = (N + h) * cos_lat;
    b_M[2] = 1;
    b_M[3] = 1;
    b_M[4] = 1;
    b_M[5] = 1;
    diag(b_M, Hksub);

    /* dlat_edit */
    /* dlon_edit */
    /* dh */
    /* dVN */
    /* dVE */
    /* dVD */
    for (i1 = 0; i1 < 6; i1++) {
      for (i2 = 0; i2 < 6; i2++) {
        c_Hk[i2 + 6 * i1] = Hksub[i2 + 6 * i1];
      }
    }

    for (i1 = 0; i1 < 9; i1++) {
      for (i2 = 0; i2 < 6; i2++) {
        c_Hk[i2 + 6 * (i1 + 6)] = 0;
      }
    }

    /* 6x15 */
    /*  R~z  R~6 */
    power(Hksub, B);

    /* ********************* */
    /* STEP1: measure_Update */
    /* ********************* */
    for (i1 = 0; i1 < 15; i1++) {
      for (i2 = 0; i2 < 6; i2++) {
        K[i1 + 15 * i2] = 0;
        for (i3 = 0; i3 < 15; i3++) {
          K[i1 + 15 * i2] += Pk_1[i1 + 15 * i3] * c_Hk[i2 + 6 * i3];
        }
      }
    }

    for (i1 = 0; i1 < 6; i1++) {
      for (i2 = 0; i2 < 15; i2++) {
        d_Hk[i1 + 6 * i2] = 0;
        for (i3 = 0; i3 < 15; i3++) {
          d_Hk[i1 + 6 * i2] += c_Hk[i1 + 6 * i3] * Pk_1[i3 + 15 * i2];
        }
      }

      for (i2 = 0; i2 < 6; i2++) {
        e_Hk[i1 + 6 * i2] = 0;
        for (i3 = 0; i3 < 15; i3++) {
          e_Hk[i1 + 6 * i2] += d_Hk[i1 + 6 * i3] * c_Hk[i2 + 6 * i3];
        }

        b_R[i1 + 6 * i2] = 0;
        for (i3 = 0; i3 < 6; i3++) {
          b_R[i1 + 6 * i2] += R[i1 + 6 * i3] * B[i3 + 6 * i2];
        }
      }
    }

    for (i1 = 0; i1 < 6; i1++) {
      for (i2 = 0; i2 < 6; i2++) {
        B[i2 + 6 * i1] = e_Hk[i2 + 6 * i1] + b_R[i2 + 6 * i1];
      }
    }

    b_mrdivide(K, B);

    /* 15x15*15x6*(6x15*15x15*15x6+6*6)^-1 */
    b_lat[0] = lat - zG[1];
    b_lat[1] = lon - zG[2];
    b_lat[2] = h - zG[3];
    b_lat[3] = vn[0] - zG[4];
    b_lat[4] = vn[1] - zG[5];
    b_lat[5] = vn[2] - zG[6];
    for (i1 = 0; i1 < 6; i1++) {
      b_M[i1] = 0;
      for (i2 = 0; i2 < 6; i2++) {
        b_M[i1] += Hksub[i1 + 6 * i2] * b_lat[i2];
      }

      h_Hk[i1] = 0;
      for (i2 = 0; i2 < 15; i2++) {
        h_Hk[i1] += c_Hk[i1 + 6 * i2] * xk_1[i2];
      }

      b_Hksub[i1] = b_M[i1] - h_Hk[i1];
    }

    b_eye(G);
    for (i1 = 0; i1 < 15; i1++) {
      x = 0;
      for (i2 = 0; i2 < 6; i2++) {
        x += K[i1 + 15 * i2] * b_Hksub[i2];
      }

      xk_1[i1] += x;
      for (i2 = 0; i2 < 15; i2++) {
        x = 0;
        for (i3 = 0; i3 < 6; i3++) {
          x += K[i1 + 15 * i3] * c_Hk[i3 + 6 * i2];
        }

        n_a[i1 + 15 * i2] = G[i1 + 15 * i2] - x;
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
        d_Pk_1[i2 + 15 * i1] = (Pk_1[i2 + 15 * i1] + Pk_1[i1 + 15 * i2]) / 2;
      }
    }

    for (i1 = 0; i1 < 15; i1++) {
      memcpy(&Pk_1[i1 * 15], &d_Pk_1[i1 * 15], 15U * sizeof(float));
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
    for (i1 = 0; i1 < 3; i1++) {
      wie_n[i1] -= b_xk_1[i1];
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

    memcpy(&c_Cbn[0], &Cbn[0], 9U * sizeof(float));
    normC(c_Cbn, Cbn);
    PVA[0] = zI[0];
    for (i1 = 0; i1 < 3; i1++) {
      PVA[i1 + 1] = wie_n[i1];
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

    /* ********************** */
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
        x = 0;
        for (i3 = 0; i3 < 15; i3++) {
          x += d_PHIk_hat[i1 + 15 * i3] * PHIk_hat[i2 + 15 * i3];
        }

        Pk_1[i1 + 15 * i2] = x + Qk[i1 + 15 * i2];
      }
    }
  } else {
    /*  measurements */
    /* ------------- */
    /* z=dVy, dVz */
    for (i1 = 0; i1 < 3; i1++) {
      wen_n[i1] = 0;
      for (i2 = 0; i2 < 3; i2++) {
        wen_n[i1] += Cbn[i2 + 3 * i1] * vn[i2];
      }

      for (i2 = 0; i2 < 2; i2++) {
        Hk[i2 + (i1 << 1)] = 0;
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
    for (i1 = 0; i1 < 6; i1++) {
      for (i2 = 0; i2 < 2; i2++) {
        Hk[i2 + ((i1 + 9) << 1)] = 0;
      }
    }

    /*  R~z  R~2 */
    for (i1 = 0; i1 < 2; i1++) {
      b_Hk[i1] = 25;
    }

    b_diag(b_Hk, Rk);

    /* ********************* */
    /* STEP1: measure_Update */
    /* ********************* */
    for (i1 = 0; i1 < 15; i1++) {
      for (i2 = 0; i2 < 2; i2++) {
        b_Pk_1[i1 + 15 * i2] = 0;
        for (i3 = 0; i3 < 15; i3++) {
          b_Pk_1[i1 + 15 * i2] += Pk_1[i1 + 15 * i3] * Hk[i2 + (i3 << 1)];
        }
      }
    }

    for (i1 = 0; i1 < 2; i1++) {
      for (i2 = 0; i2 < 15; i2++) {
        g_Hk[i1 + (i2 << 1)] = 0;
        for (i3 = 0; i3 < 15; i3++) {
          g_Hk[i1 + (i2 << 1)] += Hk[i1 + (i3 << 1)] * Pk_1[i3 + 15 * i2];
        }
      }

      for (i2 = 0; i2 < 2; i2++) {
        x = 0;
        for (i3 = 0; i3 < 15; i3++) {
          x += g_Hk[i1 + (i3 << 1)] * Hk[i2 + (i3 << 1)];
        }

        f_Hk[i1 + (i2 << 1)] = x + Rk[i1 + (i2 << 1)];
      }
    }

    mrdivide(b_Pk_1, f_Hk, b_K);

    /* 15x15*15x6*(6x15*15x15*15x6+6*6)^-1 */
    b_wen_n[0] = -wen_n[1];
    b_wen_n[1] = -wen_n[2];
    for (i1 = 0; i1 < 2; i1++) {
      b_Hk[i1] = 0;
      for (i2 = 0; i2 < 15; i2++) {
        b_Hk[i1] += Hk[i1 + (i2 << 1)] * xk_1[i2];
      }

      c_wen_n[i1] = b_wen_n[i1] - b_Hk[i1];
    }

    b_eye(G);
    for (i1 = 0; i1 < 15; i1++) {
      x = 0;
      for (i2 = 0; i2 < 2; i2++) {
        x += b_K[i1 + 15 * i2] * c_wen_n[i2];
      }

      xk_1[i1] += x;
      for (i2 = 0; i2 < 15; i2++) {
        x = 0;
        for (i3 = 0; i3 < 2; i3++) {
          x += b_K[i1 + 15 * i3] * Hk[i3 + (i2 << 1)];
        }

        n_a[i1 + 15 * i2] = G[i1 + 15 * i2] - x;
      }

      for (i2 = 0; i2 < 15; i2++) {
        l_a[i1 + 15 * i2] = 0;
        for (i3 = 0; i3 < 15; i3++) {
          l_a[i1 + 15 * i2] += n_a[i1 + 15 * i3] * Pk_1[i3 + 15 * i2];
        }
      }
    }

    for (i1 = 0; i1 < 15; i1++) {
      memcpy(&Pk_1[i1 * 15], &l_a[i1 * 15], 15U * sizeof(float));
    }

    for (i1 = 0; i1 < 15; i1++) {
      for (i2 = 0; i2 < 15; i2++) {
        c_Pk_1[i2 + 15 * i1] = (Pk_1[i2 + 15 * i1] + Pk_1[i1 + 15 * i2]) / 2;
      }
    }

    for (i1 = 0; i1 < 15; i1++) {
      memcpy(&Pk_1[i1 * 15], &c_Pk_1[i1 * 15], 15U * sizeof(float));
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
    for (i1 = 0; i1 < 3; i1++) {
      wie_n[i1] -= b_xk_1[i1];
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
      PVA[i1 + 1] = wie_n[i1];
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

    /* ********************** */
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
        x = 0;
        for (i3 = 0; i3 < 15; i3++) {
          x += d_PHIk_hat[i1 + 15 * i3] * PHIk_hat[i2 + 15 * i3];
        }

        Pk_1[i1 + 15 * i2] = x + Qk[i1 + 15 * i2];
      }
    }
  }
}

/*
 * File trailer for insgps_v6_0.c
 *
 * [EOF]
 */
