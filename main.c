#include "stm32f4xx.h"
#include "misc.h"
#include "system_timetick.h"

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


static boolean_T OverrunFlag = 0;

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

uint16_t ID[3];
int main(void)
{
  //uint16_t i;
  //uint16_t cmdbuff[16];
  //SystemCoreClockUpdate();
  /* Enable SysTick at 5ms interrupt */
  SysTick_Config(SystemCoreClock/50);//10ms
  
  delay_01ms(20000);
  init_board();
	ElapseDef_001ms(10000);

  //reset_adis();
	enable_gps();
  IMU_Quest_initialize();
  while(1){
 		if(tick_flag){
			tick_flag = 0;
			ElapseRestart();
			/* Calcutalte zI+zG data */

			rt_OneStep();							/* IMU_Quest: Get IMU data. Find Euler angles */
			INSDataProcess();					/* Process & Store new INS data */
			
			receive_data();						/* Get GPS data */
			if (rxflag){
				GPSDataProcess();				/* Process & Store new GPS data */
				rxflag = 0;
			}
			
			if (started){
				/* Mechanize and EKF */
				insgps_v6_0(zI, zG, gpsflag, dt, g0, a, e, we, 
										Q, R, PVA, bias, Pk_1, xk_1);
				send_PVA(PVA,zG);				/* Transmit PVA to UART5 */
			}
			else{
				if (gpsflag == 1){
					initialize();					/* Init const params (a,e,P0,Q0)*/
					started = 1;
				}
			}
			gpsflag = 0;
			ElapseGet(&elapsedTime1);
		}
  }
}
