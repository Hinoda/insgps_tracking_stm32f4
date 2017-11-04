//init file
//params
#include "initialize.h"

double dt, g0;
double a, e;				//ban kinh TD va tam sai WGS84
double we;					//toc do quay TD
double PVA_[10];
double PVAout[10];
double biasout[6];
double bias[6];
double Q[144], sgmR[6];
double Pk_1[225], xk_1[15]; /*ko setup them Pk, xk vi 
														chi muon delay, ko xuat ra*/
double xPI_180 = 0.017453292519943;
double x180_PI = 57.295779513082323;												
void initialize(void){
	/*const*/

	dt = 0.1/10;
	g0 = 9.80665;
	a = 6378137;
	e = 0.08181919;
	we = 7.2921155*pow(10,-5);
	int j;
	
	/*initialize PVA_,params,bias_*/
	// PVA_ = [zI(1); rn_; vn_; euler_];
	PVA_[0] = zI[0];
	PVA_[1] = zG[1];
	PVA_[2] = zG[2];
	PVA_[3] = zG[3];
	PVA_[4] = zG[4];
	PVA_[5] = zG[5];
	PVA_[6] = zG[6];
	PVA_[7] = zI[1];
	PVA_[8] = zI[2];
	PVA_[9] = zI[3];
	// bias = [ba_, bg_];
	for (j = 0; j < 6; j++) {
		bias[j] = 0;			//bai=0.49033 bgi=0.05235
	}
	// Pk_1 = diag([sgmPr_; sgmPv_; sgmPe_; sgmPba_; sgmPbg_};
	double sgmPk_1[15] = {1e2*xPI_180, 1e2*xPI_180, 1e1, \
						2e1, 2e1, 1e1, \
						3e1*xPI_180, 3e1*xPI_180, 5e2*xPI_180, \
						5e-1, 5e-1, 5e-1, \
						2e-1*xPI_180, 2e-1*xPI_180, 2e-1*xPI_180 \
						};
						//sgmPr_[3]
						//sgmPv_[3]
						//sgmPe_[3]
						//sgmPba_[3]
						//sgmPbg_[3]
	for (j = 0; j < 225; j++) {
		Pk_1[j] = 0;
	}
	for (j = 0; j < 15; j++) {
		Pk_1[j + 15 * j] = sgmPk_1[j];
	}
	// xk_1 = zeros(15,1);
	for (j = 0; j < 15; j++) {
		xk_1[j] = 0;
	}
	// output = zeros(10,1);
	for (j = 0; j < 10; j++) {
		PVAout[j] = 0;
	}
	/* Kalman params */
	//do sai lech cua phep do
	sgmR[0] = 5e-7;//RP
	sgmR[1] = 5e-7;
	sgmR[2] = 5e-7;
	sgmR[3] = 5e-3;//RV
	sgmR[4] = 5e-3;
	sgmR[5] = 5e-3;
	
	//do bat dinh cua he thong
	double sgmQ[12] = {5e0, 5e0, 5e0, \
						5e0*xPI_180, 5e0*xPI_180, 5e0*xPI_180, \
						3e-2, 3e-2, 3e-2, \
						3e-2*xPI_180, 3e-2*xPI_180, 3e-2*xPI_180, \
						};
						//sgmQa_[3]
						//sgmQg_[3]
						//sgmQba_[3]
						//sgmQbg_[3]
	for (j = 0; j < 144; j++) {
		Q[j] = 0;
	}
	for (j = 0; j < 12; j++) {
		Q[j + 12 * j] = sgmQ[j];
	}
}
