#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>


extern float dt, g0;
extern float a, e;				//ban kinh TD va tam sai WGS84
extern float we;					//toc do quay TD
extern float PVA[10];
extern float bias[6];
extern float Q[144], R[36];
extern float Pk_1[225], xk_1[15]; /*ko setup them Pk, xk vi 
														chi muon delay, ko xuat ra*/
extern float xPI_180;
extern float x180_PI;	

void initialize(void);
