#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>


//extern double dt, g0;
//extern double a, e;				//ban kinh TD va tam sai WGS84
//extern double we;					//toc do quay TD
//extern double PVA[10];
//extern double bias[6];
//extern double Q[144], R[36];
//extern double Pk_1[225], xk_1[15]; /*ko setup them Pk, xk vi 
//														chi muon delay, ko xuat ra*/
extern double xPI_180;
extern double x180_PI;	

//void initialize(double *dt,\
//				double *g0, double *a, double *e, double *we, double Q[144], double R[36],\
//				double PVA[10], double bias[6], double Pk_1[225], double xk_1[15]);
void initialize(double *dt,\
				double *g0, double *a, double *e, double *we, double* Q, double* R,\
				double* PVA, double* bias, double* Pk_1, double* xk_1);
