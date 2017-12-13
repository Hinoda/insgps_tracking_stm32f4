
/********** Include section ***************************************************/

#include "stm32f4xx.h"
#include "misc.h"
#include "system_timetick.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "driver.h"
#include "insgps_v8_0.h"
#include "initialize.h"
#include "genDataGetProcess.h"
#include "IMU_Quest.h"                 /* Model's header file */

//#include "rtwtypes.h"                  /* MathWorks types */

#include "Cbn_31.h"
#include "normC.h"
#include "skew_mat3.h"
#include "Cbn_31_terminate.h"
#include "Cbn_31_initialize.h"
#include "deg2utm.h"
/********** Local (static) variable definition ********************************/

static boolean_T OverrunFlag = 0;
uint16_t mycount = 0;
uint16_t ID[3];
uint16_t i = 0;
uint16_t timesRun = 0;
//------- main variable ----------------
bool gpsAvail = false;
bool firstTime = false;
bool sendGPS = false;
uint8_t g_commaIndex[23];
double g_dt, g_g0;
double g_a, g_e;
double g_we;
double g_PVA[10];
double g_bias[6];
double g_Q[144], g_R[36];
double g_Pk_1[225], g_xk_1[15];
//------- debug variable ----------------
//double Eroll=0,Epitch=0,Eyaw=0;
//double PVAroll=0,PVApitch=0,PVAyaw=0;

//double G_latUTM=0,G_lonUTM=0,G_hUTM=0;
//double PVA_latUTM=0,PVA_lonUTM=0,PVA_hUTM=0;
/********** Local function definition section *********************************/

void rt_OneStep(void)
{
	/* Disable interrupts here */
	/* Check for overrun */
	if (OverrunFlag++) {
		rtmSetErrorStatus(IMU_Quest_M, "Overrun");
		return;
	}

	/* Save FPU context here (if necessary) */
	/* Re-enable timer or interrupt here */
	/* Set model inputs here */

	/* Step the model */
	IMU_Quest_step();

	/* Get model outputs here */

	/* Indicate task complete */
	OverrunFlag--;

	/* Disable interrupts here */
	/* Restore FPU context here (if necessary) */
	/* Enable interrupts here */
}


int main(void)
{
	//sua system time
	
	SystemCoreClockUpdate();
	/* Enable SysTick at 5ms interrupt */
	//SysTick_Config(SystemCoreClock/100);//10ms
	SysTick_Config(SystemCoreClock/200);//5ms
	delay_01ms(20000);
	init_board();
	ElapsedDef_001ms(10000);
	enable_gps();
	IMU_Quest_initialize();
	while(1)
	{
		if(tick_flag)
		{
			tick_flag = 0;
			if (mycount < 2000) //waiting 1s to settle euler
			{
				mycount++;
				rt_OneStep();
			}
			else //1s
			{
				////////////////////////////////////////////////////////////////////////////
				//ElapsedRestart();
				/* zI*/
				INSDataProcess();			/* Process & Store new INS data */
				receive_data();				/* Get GPS data */
				if(rxflag == 1)
				{
					AssignGPSComma(g_commaIndex);
					gpsAvail = CheckGPSflag(g_commaIndex);
					sendGPS = gpsAvail;
					GPSDataProcess(gpsAvail, firstTime, g_commaIndex);
					rxflag = 0;
				}
				else
				{
					gpsAvail = false;
					sendGPS = false;
				}
				if (firstTime==false)
				{
					if (gpsAvail==true)
					{
						initialize(&g_dt, &g_g0, &g_a, &g_e, &g_we, g_Q, g_R, g_PVA, g_bias, g_Pk_1, g_xk_1);
						//g_dt = elapsedTime1*pow(10,-5);
						g_dt = 0.025;//guess
						ElapsedRestart();
						insgps_v8_0(g_zI, g_zG, gpsAvail, g_dt, g_g0, g_a, g_e, g_we, g_Q, g_R, g_PVA, g_bias, g_Pk_1, g_xk_1);
						firstTime = true;
						gpsAvail = false;
						//sendMode("YAY!!!\r\nCube is starting now.........");
					}
					/*else
					{
						//sendMode("Trying to start...");
					}*/
				}
				else //second third ...
				{
					ElapsedGet(&elapsedTime1);
					g_dt = elapsedTime1*pow(10,-5);
					insgps_v8_0(g_zI, g_zG, gpsAvail, g_dt, g_g0, g_a, g_e, g_we, g_Q, g_R, g_PVA, g_bias, g_Pk_1, g_xk_1);
					gpsAvail = false;
				}
				/*
				PVAroll=g_PVA[7]*x180_PI;
				PVApitch=g_PVA[8]*x180_PI;
				PVAyaw=g_PVA[9]*x180_PI;
				
				Eroll=euler[0]*0.1;
				Epitch=euler[1]*0.1;
				Eyaw=euler[2]*0.1;
				
				deg2utm(g_zG[1]*x180_PI, g_zG[2]*x180_PI, &G_lonUTM, &G_latUTM);
				deg2utm(g_PVA[1]*x180_PI, g_PVA[2]*x180_PI, &PVA_lonUTM, &PVA_latUTM);
				G_hUTM=g_zG[3];
				PVA_hUTM=g_PVA[3];
				*/
				
				//sendMode("Sending PVA ==>");
				send_PVA(g_PVA,g_zG,sendGPS);					/* Transmit PVA to UART5 */
				sendGPS = false;	
				for (uint8_t i=0;i<7;i++)	g_zG[i]=0;
				for (uint8_t i=0;i<10;i++)	g_zI[i]=0;
				//ElapsedGet(&elapsedTime1);
				//sendElapsed(elapsedTime1);
				/*
				G_lonUTM=0;			
				G_latUTM=0;
				G_hUTM=0;
				
				PVA_lonUTM=0;
				PVA_latUTM=0;
				PVA_hUTM=0;
				
				PVAroll=0;
				PVApitch=0;
				PVAyaw=0;
				
				Eroll=0;
				Epitch=0;
				Eyaw=0;
				*/
			}
		}
	}
}
