/*
 * File: mrdivide.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 12-Nov-2017 15:05:52
 */

/* Include Files */
#include "Cbn_31.h"
#include "insgps_v6_0.h"
#include "normC.h"
#include "skew_mat3.h"
#include "mrdivide.h"

/* Function Definitions */

/*
 * Arguments    : float A[90]
 *                const float B[36]
 * Return Type  : void
 */
void b_mrdivide(float A[90], const float B[36])
{
  float b_A[36];
  signed char ipiv[6];
  int k;
  int j;
  int c;
  int kBcol;
  int jp;
  int ix;
  int jAcol;
  float temp;
  int i;
  float s;
  memcpy(&b_A[0], &B[0], 36U * sizeof(float));
  for (k = 0; k < 6; k++) {
    ipiv[k] = (signed char)(1 + k);
  }

  for (j = 0; j < 5; j++) {
    c = j * 7;
    kBcol = 0;
    ix = c;
    temp = fabs(b_A[c]);
    for (k = 2; k <= 6 - j; k++) {
      ix++;
      s = fabs(b_A[ix]);
      if (s > temp) {
        kBcol = k - 1;
        temp = s;
      }
    }

    if (b_A[c + kBcol] != 0.0) {
      if (kBcol != 0) {
        ipiv[j] = (signed char)((j + kBcol) + 1);
        ix = j;
        kBcol += j;
        for (k = 0; k < 6; k++) {
          temp = b_A[ix];
          b_A[ix] = b_A[kBcol];
          b_A[kBcol] = temp;
          ix += 6;
          kBcol += 6;
        }
      }

      k = (c - j) + 6;
      for (i = c + 1; i + 1 <= k; i++) {
        b_A[i] /= b_A[c];
      }
    }

    jp = c;
    jAcol = c + 6;
    for (kBcol = 1; kBcol <= 5 - j; kBcol++) {
      temp = b_A[jAcol];
      if (b_A[jAcol] != 0.0) {
        ix = c + 1;
        k = (jp - j) + 12;
        for (i = 7 + jp; i + 1 <= k; i++) {
          b_A[i] += b_A[ix] * -temp;
          ix++;
        }
      }

      jAcol += 6;
      jp += 6;
    }
  }

  for (j = 0; j < 6; j++) {
    jp = 15 * j;
    jAcol = 6 * j;
    for (k = 1; k <= j; k++) {
      kBcol = 15 * (k - 1);
      if (b_A[(k + jAcol) - 1] != 0.0) {
        for (i = 0; i < 15; i++) {
          A[i + jp] -= b_A[(k + jAcol) - 1] * A[i + kBcol];
        }
      }
    }

    temp = 1.0 / b_A[j + jAcol];
    for (i = 0; i < 15; i++) {
      A[i + jp] *= temp;
    }
  }

  for (j = 5; j >= 0; j += -1) {
    jp = 15 * j;
    jAcol = 6 * j - 1;
    for (k = j + 2; k < 7; k++) {
      kBcol = 15 * (k - 1);
      if (b_A[k + jAcol] != 0.0) {
        for (i = 0; i < 15; i++) {
          A[i + jp] -= b_A[k + jAcol] * A[i + kBcol];
        }
      }
    }
  }

  for (kBcol = 4; kBcol >= 0; kBcol += -1) {
    if (ipiv[kBcol] != kBcol + 1) {
      jp = ipiv[kBcol] - 1;
      for (jAcol = 0; jAcol < 15; jAcol++) {
        temp = A[jAcol + 15 * kBcol];
        A[jAcol + 15 * kBcol] = A[jAcol + 15 * jp];
        A[jAcol + 15 * jp] = temp;
      }
    }
  }
}

/*
 * Arguments    : const float A[30]
 *                const float B[4]
 *                float y[30]
 * Return Type  : void
 */
void mrdivide(const float A[30], const float B[4], float y[30])
{
  int r1;
  int r2;
  float a21;
  float a22;
  int k;
  if (fabs(B[1]) > fabs(B[0])) {
    r1 = 1;
    r2 = 0;
  } else {
    r1 = 0;
    r2 = 1;
  }

  a21 = B[r2] / B[r1];
  a22 = B[2 + r2] - a21 * B[2 + r1];
  for (k = 0; k < 15; k++) {
    y[k + 15 * r1] = A[k] / B[r1];
    y[k + 15 * r2] = (A[15 + k] - y[k + 15 * r1] * B[2 + r1]) / a22;
    y[k + 15 * r1] -= y[k + 15 * r2] * a21;
  }
}

/*
 * File trailer for mrdivide.c
 *
 * [EOF]
 */
