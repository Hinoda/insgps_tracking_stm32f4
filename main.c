
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
bool gpsAvail = false;
bool firstTime = false;
uint16_t ID[3];
uint16_t i = 0;
uint8_t commaIndex[23];
uint16_t timesRun = 0;
float dt, g0;
float a, e;
float we;
float PVA[10];
float bias[6];
float Q[144], R[36];
float Pk_1[225], xk_1[15]; /*ko setup them Pk, xk vi chi muon delay, ko xuat ra*/
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

			receive_data();				/* Get GPS data */
			if(rxflag == 1)
            {
                AssignGPSComma(commaIndex);
				//gpsAvail = rxflag&(CheckGPSflag(commaIndex));
				gpsAvail = CheckGPSflag(commaIndex);
				GPSDataProcess(firstTime, commaIndex);
				rxflag = 0;
            }
			else
			{
				gpsAvail = false;
			}
            ElapsedGet(&elapsedTime1);
			ElapsedRestart();
            if (!firstTime)
            {
                if (gpsAvail)
                {
                    initialize(&dt, &g0, &a, &e, &we, PVA, bias, Q, R, Pk_1, xk_1);
                    firstTime = true;
                    gpsAvail = false;
                    delay_01ms(100);
                    sendMode("YAY!!!\r\nCube is starting now.........");
                }
                else
                {
                    sendMode("Trying to start...");
                }
            }
			else //second third ...
            {
                dt = elapsedTime1*pow(10,-5);
				//dt = 0.01;
				insgps_v6_0(zI, zG, gpsAvail, dt, g0, a, e, we, Q, R, PVA, bias, Pk_1, xk_1);
                sendMode("Cube is sending PVA. Please check your save mode!");
				send_PVA(PVA,zG,gpsAvail);					/* Transmit PVA to UART5 */	
				gpsAvail = false;
            }
			memset(zI,0,10);
			memset(zG,0,7);
		}
	}
}
