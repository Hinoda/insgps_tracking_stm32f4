//init file
//params
#include "initialize.h"
#include "genDataGetProcess.h"
//float dt, g0;
//float a, e;
//float we;
//float PVA[10];
//float bias[6];
//float Q[144], R[36];
//float Pk_1[225], xk_1[15]; /*ko setup them Pk, xk vi chi muon delay, ko xuat ra*/
float xPI_180 = 0.017453292519943;
float x180_PI = 57.295779513082323;

//void initialize(float *dt,\
//				float *g0, float *a, float *e, float *we, float Q[144], float R[36],\
//				float PVA[10], float bias[6], float Pk_1[225], float xk_1[15])
void initialize(float *dt,\
				float *g0, float *a, float *e, float *we, float* Q, float* R,\
				float* PVA, float* bias, float* Pk_1, float* xk_1)
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
	PVA[0] = zI[0];
	PVA[1] = zG[1];
	PVA[2] = zG[2];
	PVA[3] = zG[3];
	PVA[4] = zG[4];
	PVA[5] = zG[5];
	PVA[6] = zG[6];
	PVA[7] = zI[1];
	PVA[8] = zI[2];
	PVA[9] = zI[3];
	// bias = [ba_, bg_];
	for (j = 0; j < 6; j++) {
		bias[j] = 0;			//bai=0.49033 bgi=0.05235
	}
	/***********************************************************/
	// P: Pk_1 = diag([sgmPr_; sgmPv_; sgmPe_; sgmPba_; sgmPbg_};
	float sgmPk_1[15];
	//sgmPr_[3]
	sgmPk_1[0] = 1e2*xPI_180;
	sgmPk_1[1] = 1e2*xPI_180;
	sgmPk_1[2] = 1e1;
	//sgmPv_[3]
	sgmPk_1[3] = 5e0;
	sgmPk_1[4] = 5e0;
	sgmPk_1[5] = 2e0;
	//sgmPe_[3]
	sgmPk_1[6] = 1e1*xPI_180;
	sgmPk_1[7] = 1e1*xPI_180;
	sgmPk_1[8] = 1e2*xPI_180;
	//sgmPba_[3]					
	sgmPk_1[9] = 1e1;
	sgmPk_1[10] = 1e1;
	sgmPk_1[11] = 1e1;
	//sgmPbg_[3]
	sgmPk_1[12] = 1e1*xPI_180;
	sgmPk_1[13] = 1e1*xPI_180;
	sgmPk_1[14] = 1e1*xPI_180;
	for (j = 0; j < 225; j++) {
		Pk_1[j] = 0;
	}
	for (j = 0; j < 15; j++) {
		Pk_1[j + 15 * j] = sgmPk_1[j] * sgmPk_1[j];
	}
	// x: xk_1 = zeros(15,1);
	for (j = 0; j < 15; j++) {
		xk_1[j] = 0;
	}
	
	/***********************************************************/
	/* Kalman params */
	/***********************************************************/
	//Q: do bat dinh cua he thong
	float sgmQ[12];
	//sgmQa_[3]
	sgmQ[0] = 5e0;
	sgmQ[1] = 5e0;
	sgmQ[2] = 1e0;
	//sgmQg_[3]
	sgmQ[3] = 5e0*xPI_180;
	sgmQ[4] = 5e0*xPI_180;
	sgmQ[5] = 5e0*xPI_180;
	//sgmQba_[3]
	sgmQ[6] = 1e-1;
	sgmQ[7] = 1e-1;
	sgmQ[8] = 1e-1;
	//sgmQbg_[3]
	sgmQ[9] = 7e-2*xPI_180;
	sgmQ[10] = 7e-2*xPI_180;
	sgmQ[11] = 7e-2*xPI_180;					
	for (j = 0; j < 144; j++) {
		Q[j] = 0;
	}
	for (j = 0; j < 12; j++) {
		Q[j + 12 * j] = sgmQ[j] * sgmQ[j];
	}
	/***********************************************************/
	// R: do sai lech cua phep do
	float sgmR[6];
	//RP
	sgmR[0] = 1e-6;
	sgmR[1] = 1e-6;
	sgmR[2] = 1e-4;
	//RV
	sgmR[3] = 1e-2;
	sgmR[4] = 1e-2;
	sgmR[5] = 1e-2;
	for (j = 0; j < 36; j++) {
		R[j] = 0;
	}
	for (j = 0; j < 6; j++) {
		R[j + 6 * j] = sgmR[j] * sgmR[j];
	}
}
