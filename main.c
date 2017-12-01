
/********** Include section ***************************************************/

#include "stm32f4xx.h"
#include "misc.h"
#include "system_timetick.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "driver.h"
#include "insgps_v6_0.h"
#include "initialize.h"
#include "genDataGetProcess.h"
#include "IMU_Quest.h"                 /* Model's header file */

//#include "rtwtypes.h"                  /* MathWorks types */

#include "Cbn_31.h"
#include "normC.h"
#include "skew_mat3.h"
#include "Cbn_31_terminate.h"
#include "Cbn_31_initialize.h"

/********** Local (static) variable definition ********************************/

static boolean_T OverrunFlag = 0;

uint16_t ID[3];
uint16_t i = 0;
uint16_t timesRun = 0;

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
	bool gpsAvail = false;
	bool firstTime = false;
	bool sendGPS = false;
	uint8_t g_commaIndex[23];
	float g_dt, g_g0;
	float g_a, g_e;
	float g_we;
	float g_PVA[10];
	float g_bias[6];
	float g_Q[144], g_R[36];
	float g_Pk_1[225], g_xk_1[15];
	
	SystemCoreClockUpdate();
	/* Enable SysTick at 5ms interrupt */
	//SysTick_Config(SystemCoreClock/100);//10ms
	SysTick_Config(SystemCoreClock/50);//20ms
	delay_01ms(50000);
	
	init_board();
	ElapsedDef_001ms(10000);
	//reset_adis();
	enable_gps();
	IMU_Quest_initialize();
	while(1)
	{
		if(tick_flag)
		{
			tick_flag = 0;
			//ElapsedRestart();
			/* Calcutalte zI+zG data */
			rt_OneStep();				/* IMU_Quest: Get IMU data. Find Euler angles */
			INSDataProcess();			/* Process & Store new INS data */
			
			/* Get dt from elapsed time between samples*/
            ElapsedGet(&elapsedTime1);
			ElapsedRestart();
			/* ------------------ */
			
			receive_data();				/* Get GPS data */
			if(rxflag == 1)
            {
                AssignGPSComma(g_commaIndex);
				//gpsAvail = rxflag&(CheckGPSflag(commaIndex));
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
            if (!firstTime)
            {
                if (gpsAvail)
                {
                    initialize(&g_dt, &g_g0, &g_a, &g_e, &g_we, g_Q, g_R, g_PVA, g_bias, g_Pk_1, g_xk_1);
					g_dt = elapsedTime1*pow(10,-5);
					//g_dt = 0.02;
					//zG[6]=0.1;//VD =/=0.1
					insgps_v6_0(g_zI, g_zG, gpsAvail, g_dt, g_g0, g_a, g_e, g_we, g_Q, g_R, g_PVA, g_bias, g_Pk_1, g_xk_1);
                    firstTime = true;
                    gpsAvail = false;
                    delay_01ms(100);
                    //sendMode("YAY!!!\r\nCube is starting now.........");
                }
                /*else
                {
                    //sendMode("Trying to start...");
                }*/
            }
			else //second third ...
            {
                g_dt = elapsedTime1*pow(10,-5);
				//g_dt = 0.02;
				insgps_v6_0(g_zI, g_zG, gpsAvail, g_dt, g_g0, g_a, g_e, g_we, g_Q, g_R, g_PVA, g_bias, g_Pk_1, g_xk_1);
				gpsAvail = false;
            }
			
            //sendMode("Sending PVA ==>");
            send_PVA(g_PVA,g_zG,sendGPS);					/* Transmit PVA to UART5 */
			sendGPS = false;	
			for (uint8_t i=0;i<7;i++)	g_zG[i]=0;
			for (uint8_t i=0;i<10;i++)	g_zI[i]=0;
			//i=0;
			//ElapsedGet(&elapsedTime1);
		}
	}
}
