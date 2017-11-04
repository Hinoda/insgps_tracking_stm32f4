/*
 * File: normC.c
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
#include "power.h"

/* Function Definitions */

/*
 * % Desciption
 *
 * @param  :[in] Cbn
 * @return :     ortho-nomalized Cbn
 * @bug    :     make angle unsteady everytime GPS
 * TODO    :     quaternion or ortho-normalized
 * Arguments    : double C[9]
 *                double Cbn[9]
 * Return Type  : void
 */
void normC(double C[9], double Cbn[9])
{
  double dv4[9];
  int k;
  double x[3];
  double y;
  int i2;

  /* % BUG */
  /*  when GPS is available (update), angle seems to be robust */
  /*  my data is absolutely trusted to be steady (data get from real flight */
  /*  (Hanoi to Saigon) by Mr. Vinh Hao) */
  /* % */
  /*   */
  /*  <<angle_sample_broken.PNG>> */
  /*   */
  /* % Idea suggested */
  /*  <https://www.gamedev.net/forums/topic/278410-rotation-matrix-ortho-normalization/> */
  /*   */
  /*  "" */
  /*  As others have said, use quaternions. Anyway, it can be done as */
  /*  */
  /*    Column1=Normalized(CrossProduct(Column2,Column3)); */
  /*    Column2=Normalized(CrossProduct(Column3,Column1)); */
  /*    // and then you don't really need to Column3=Normalized(CrossProduct(Column1,Column2)); */
  /*    Column3=Normalized(Column3); */
  /*   */
  /*  Where ColumnN contains nth column of 3x3 rotation matrix as vector.  */
  /*  If you use 4x4 matrix, it's submatrix in top-left corner. */
  /*  You can do the same with rows if you want. */
  /*  Note that one column/row may be selected to be distorted the least. */
  /*   */
  /*  Also, don't do it when matrix is unchanged, or your things might start slowly turn.  */
  /*  "" */
  /* % */
  /*  >> *So, with this function*  */
  /*   */
  /*    function ret=skew_mat3(A) */
  /*        if (size(A,1)==3)%row(A)=3 */
  /*            ret=[0      -A(3,1)   A(2,1); */
  /*                 A(3,1)      0   -A(1,1); */
  /*                -A(2,1)  A(1,1)       0]; */
  /*        end */
  /*    end */
  /*   */
  /*  *should I use...* */
  /* % METHOD 1 */
  /*  normalized by collumns */
  /*  C(:,1)=C(:,1)/(sqrt(sum(C(:,1).^2))); */
  /*  C(:,2)=C(:,2)/(sqrt(sum(C(:,2).^2))); */
  /*  C(:,3)=C(:,3)/(sqrt(sum(C(:,3).^2)));     */
  /*  Cbn=C;    */
  /* % METHOD 2 */
  /*  normalized by rows */
  /*  */
  /*    C(1,:)=C(1,:)/(sqrt(sum(C(1,:).^2))); */
  /*    C(2,:)=C(2,:)/(sqrt(sum(C(2,:).^2))); */
  /*    C(3,:)=C(3,:)/(sqrt(sum(C(3,:).^2))); */
  /*    Cbn=C; */
  /* % METHOD 3 */
  /*  ortho */
  /*  */
  /*    C(1,:)=(skew_mat3(C(2,:)')*C(3,:)')'; */
  /*    C(2,:)=(skew_mat3(C(3,:)')*C(1,:)')'; */
  /*    C(3,:)=(skew_mat3(C(1,:)')*C(2,:)')'; */
  /*    C(:,1)=(skew_mat3(C(:,2))*C(:,3)); */
  /*    C(:,2)=(skew_mat3(C(:,3))*C(:,1)); */
  /*    C(:,3)=(skew_mat3(C(:,1))*C(:,2)); */
  /*    Cbn=C; */
  /*  %% METHOD web */
  skew_mat3(*(double (*)[3])&C[3], dv4);
  for (k = 0; k < 3; k++) {
    C[k] = 0.0;
    for (i2 = 0; i2 < 3; i2++) {
      C[k] += dv4[k + 3 * i2] * C[6 + i2];
    }
  }

  power(*(double (*)[3])&C[0], x);
  y = x[0];
  for (k = 0; k < 2; k++) {
    y += x[k + 1];
  }

  y = sqrt(y);
  for (k = 0; k < 3; k++) {
    C[k] /= y;
  }

  skew_mat3(*(double (*)[3])&C[6], dv4);
  for (k = 0; k < 3; k++) {
    C[3 + k] = 0.0;
    for (i2 = 0; i2 < 3; i2++) {
      C[3 + k] += dv4[k + 3 * i2] * C[i2];
    }
  }

  power(*(double (*)[3])&C[3], x);
  y = x[0];
  for (k = 0; k < 2; k++) {
    y += x[k + 1];
  }

  y = sqrt(y);
  for (k = 0; k < 3; k++) {
    C[3 + k] /= y;
  }

  power(*(double (*)[3])&C[6], x);
  y = x[0];
  for (k = 0; k < 2; k++) {
    y += x[k + 1];
  }

  y = sqrt(y);
  for (k = 0; k < 3; k++) {
    C[6 + k] /= y;
  }

  memcpy(&Cbn[0], &C[0], 9U * sizeof(double));
}

/*
 * File trailer for normC.c
 *
 * [EOF]
 */
