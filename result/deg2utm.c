/*
 * File: deg2utm.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 13-Dec-2017 10:47:33
 */

/* Include Files */
#include "Cbn_31.h"
#include "GramSchmidt.h"
#include "deg2utm.h"
#include "insgps_v11.h"
#include "skew_mat3.h"

/* Function Definitions */

/*
 * -------------------------------------------------------------------------
 *  [x,y,utmzone] = deg2utm(Lat,Lon)
 *
 *  Description: Function to convert lat/lon vectors into UTM coordinates (WGS84).
 *  Some code has been extracted from UTM.m function by Gabriel Ruiz Martinez.
 *
 *  Inputs:
 *     Lat: Latitude vector.   Degrees.  +ddd.ddddd  WGS84
 *     Lon: Longitude vector.  Degrees.  +ddd.ddddd  WGS84
 *
 *  Outputs:
 *     x, y , utmzone.   See example
 *
 *  Example 1:
 *     Lat=[40.3154333; 46.283900; 37.577833; 28.645650; 38.855550; 25.061783];
 *     Lon=[-3.4857166; 7.8012333; -119.95525; -17.759533; -94.7990166; 121.640266];
 *     [x,y,utmzone] = deg2utm(Lat,Lon);
 *     fprintf('%7.0f ',x)
 *        458731  407653  239027  230253  343898  362850
 *     fprintf('%7.0f ',y)
 *       4462881 5126290 4163083 3171843 4302285 2772478
 *     utmzone =
 *        30 T
 *        32 T
 *        11 S
 *        28 R
 *        15 S
 *        51 R
 *
 *  Example 2: If you have Lat/Lon coordinates in Degrees, Minutes and Seconds
 *     LatDMS=[40 18 55.56; 46 17 2.04];
 *     LonDMS=[-3 29  8.58;  7 48 4.44];
 *     Lat=dms2deg(mat2dms(LatDMS)); %convert into degrees
 *     Lon=dms2deg(mat2dms(LonDMS)); %convert into degrees
 *     [x,y,utmzone] = deg2utm(Lat,Lon)
 *
 *  Author:
 *    Rafael Palacios
 *    Universidad Pontificia Comillas
 *    Madrid, Spain
 *  Version: Apr/06, Jun/06, Aug/06, Aug/06
 *  Aug/06: fixed a problem (found by Rodolphe Dewarrat) related to southern
 *     hemisphere coordinates.
 *  Aug/06: corrected m-Lint warnings
 * -------------------------------------------------------------------------
 * Arguments    : double Lat
 *                double Lon
 *                double *x
 *                double *y
 * Return Type  : void
 */
void deg2utm(double Lat, double Lon, double *x, double *y)
{
  double lat;
  double b_y;
  double deltaS;
  double a;
  double epsilon;
  double v;
  double ta;
  double a1;
  double a2;
  double j2;
  double j4;
  double yy;

  /*  Argument checking */
  /*  */
  /* 2 arguments required */
  /*  Memory pre-allocation */
  /*  */
  /* edit 30/03/2017 */
  /*  comment utmzone, not use */
  /* utmzone(n1,:)='60 X'; */
  /* utmzone = '60 X'; */
  /*  Main Loop */
  /*  */
  /* e = ( ( ( sa ^ 2 ) - ( sb ^ 2 ) ) ^ 0.5 ) / sa; */
  /* alpha = ( sa - sb ) / sa;             %f */
  /* ablandamiento = 1 / alpha;   % 1/f */
  lat = Lat * 0.017453292519943295;
  *y = Lon / 6.0;
  if (*y + 31.0 < 0.0) {
    b_y = ceil(*y + 31.0);
  } else {
    b_y = floor(*y + 31.0);
  }

  deltaS = Lon * 0.017453292519943295 - (b_y * 6.0 - 183.0) *
    0.017453292519943295;
  a = cos(lat) * sin(deltaS);
  epsilon = 0.5 * log((1.0 + a) / (1.0 - a));
  *x = cos(lat);
  v = 6.399593625758674E+6 / sqrt(1.0 + 0.006739496742333464 * (*x * *x)) *
    0.9996;
  *x = cos(lat);
  ta = 0.003369748371166732 * (epsilon * epsilon) * (*x * *x);
  a1 = sin(2.0 * lat);
  *x = cos(lat);
  a2 = a1 * (*x * *x);
  j2 = lat + a1 / 2.0;
  j4 = (3.0 * j2 + a2) / 4.0;
  *x = cos(lat);
  yy = (atan(tan(lat) / cos(deltaS)) - lat) * v * (1.0 + ta) +
    6.3970337883083709E+6 * (((lat - 0.0050546225567500982 * j2) +
    4.258201531867817E-5 * j4) - 1.2962962962962963 * pow(0.0050546225567500982,
    3.0) * ((5.0 * j4 + a2 * (*x * *x)) / 3.0));
  if (yy < 0.0) {
    yy += 9.999999E+6;
  }

  *x = epsilon * v * (1.0 + ta / 3.0) + 500000.0;
  *y = yy;

  /* edit 30/03/2017 */
  /* uncomment utmzone */
  /* utmzone(i,:)=sprintf('%02d %c',Huso,Letra); */
}

/*
 * File trailer for deg2utm.c
 *
 * [EOF]
 */
