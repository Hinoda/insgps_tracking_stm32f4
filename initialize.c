//init file
//params
#include "initialize.h"
#include "genDataGetProcess.h"
//double dt, g0;
//double a, e;
//double we;
//double PVA[10];
//double bias[6];
//double Q[144], R[36];
//double Pk_1[225], xk_1[15]; /*ko setup them Pk, xk vi chi muon delay, ko xuat ra*/
double xPI_180 = 0.017453292519943;
double x180_PI = 57.295779513082323;

//void initialize(double *dt,\
//				double *g0, double *a, double *e, double *we, double Q[144], double R[36],\
//				double PVA[10], double bias[6], double Pk_1[225], double xk_1[15])
void initialize(double *dt,\
				double *g0, double *a, double *e, double *we, double* Q, double* R,\
				double* PVA, double* bias, double* Pk_1, double* xk_1)
{

//	dt = 0.1/5;
	*g0 = 9.80665;
	*a = 6378137;
	*e = 0.08181919;
	*we = 7.2921155*pow(10,-5);
	int j;
	/***********************************************************/
	/*initialize PVA_,params,bias_*/
	/***********************************************************/
	// PVA_ = [zI(1); rn_; vn_; euler_];
	*PVA = *g_zI;
	*(PVA+1) = *(g_zG+1);
	*(PVA+2) = *(g_zG+2);
	*(PVA+3) = *(g_zG+3);
	*(PVA+4) = *(g_zG+4);
	*(PVA+5) = *(g_zG+5);
	*(PVA+6) = *(g_zG+6);
	*(PVA+7) = *(g_zI+1);
	*(PVA+8) = *(g_zI+2);
	*(PVA+9) = *(g_zI+3);
	// bias = [ba_, bg_];
	for (j = 0; j < 6; j++) {
		*(bias + j) = 0;			//bai=0.49033 bgi=0.05235
	}
	/***********************************************************/
	// P: Pk_1 = diag([sgmPr_; sgmPv_; sgmPe_; sgmPba_; sgmPbg_};
	double sgmPk_1[15];
	//sgmPr_[3]
	sgmPk_1[0] = 5e-6*xPI_180;
	sgmPk_1[1] = 5e-6*xPI_180;
	sgmPk_1[2] = 2e0;
	//sgmPv_[3]
	sgmPk_1[3] = 2e0;
	sgmPk_1[4] = 2e0;
	sgmPk_1[5] = 2e0;
	//sgmPe_[3]
	sgmPk_1[6] = 2e0*xPI_180;
	sgmPk_1[7] = 2e0*xPI_180;
	sgmPk_1[8] = 5e0*xPI_180;
	//sgmPba_[3]					
	sgmPk_1[9] = 6e-1;
	sgmPk_1[10] = 6e-1;
	sgmPk_1[11] = 6e-1;
	//sgmPbg_[3]
	sgmPk_1[12] = 4e0*xPI_180;
	sgmPk_1[13] = 4e0*xPI_180;
	sgmPk_1[14] = 4e0*xPI_180;
	for (j = 0; j < 225; j++) {
		*(Pk_1 + j) = 0;
	}
	for (j = 0; j < 15; j++) {
		*(Pk_1 + j + 15 * j) = sgmPk_1[j] * sgmPk_1[j];
	}
	// x: xk_1 = zeros(15,1);
	for (j = 0; j < 15; j++) {
		*(xk_1 + j) = 0;
	}
	
	/***********************************************************/
	/* Kalman params */
	/***********************************************************/
	//Q: do bat dinh cua he thong
	double sgmQ[12];
	//sgmQa_[3]
	sgmQ[0] = (5.56e-5)*1;
	sgmQ[1] = (5.56e-5)*1;
	sgmQ[2] = (5.56e-5)*1;
	//sgmQg_[3]
	sgmQ[3] = (5.56e-4)*1*xPI_180;
	sgmQ[4] = (5.56e-4)*1*xPI_180;
	sgmQ[5] = (5.56e-4)*1*xPI_180;
	//sgmQba_[3]
	sgmQ[6] = (8.72e-4)*1;
	sgmQ[7] = (8.72e-4)*1;
	sgmQ[8] = (8.72e-4)*1;
	//sgmQbg_[3]
	sgmQ[9] = 0.0065*1*xPI_180;
	sgmQ[10] = 0.0065*1*xPI_180;
	sgmQ[11] = 0.0065*1*xPI_180;					
	for (j = 0; j < 144; j++) {
		*(Q + j) = 0;
	}
	for (j = 0; j < 12; j++) {
		*(Q + j + 12 * j) = sgmQ[j] * sgmQ[j];
	}
	/***********************************************************/
	// R: do sai lech cua phep do
	double sgmR[6];
	//RP
	sgmR[0] = 5e-2;
	sgmR[1] = 5e-2;
	sgmR[2] = 5e-1;
	//RV
	sgmR[3] = 1e-2;
	sgmR[4] = 1e-2;
	sgmR[5] = 2e-2;
	for (j = 0; j < 36; j++) {
		*(R + j) = 0;
	}
	for (j = 0; j < 6; j++) {
		*(R + j + 6 * j) = sgmR[j] * sgmR[j];
	}
}
