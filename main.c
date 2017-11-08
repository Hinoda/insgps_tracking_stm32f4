#include "stm32f4xx.h"
#include "misc.h"
#include "system_timetick.h"

#include "driver.h"
#include "insgps_v4_0.h"
#include "initialize.h"
#include "genDataGetProcess.h"
#include "IMU_Quest.h"                 /* Model's header file */

//#include "rtwtypes.h"                  /* MathWorks types */


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
uint16_t elapsedTime;
int main(void)
{
  //uint16_t i;
  //uint16_t cmdbuff[16];
  //SystemCoreClockUpdate();
  /* Enable SysTick at 5ms interrupt */
  SysTick_Config(SystemCoreClock/100);//10ms
  
  delay_01ms(20000);
  init_board();
	ElapseDef_001ms(10000);

  //reset_adis();
	enable_gps();
	
  IMU_Quest_initialize();
  delay_01ms(20000);
  while(1){
 		if(tick_flag){
			tick_flag = 0;
			/* Reset Elapse Timer*/
			TIM7->SR  = 0;		// clear overflow flag
			TIM7->CR1 = 1;		// enable Timer7
			TIM7->CNT = 0;
			/* Calcutalte zI+zG data */
			
			/* Get IMU data and Find Euler angles */
			rt_OneStep();
			/* Get GPS data */
			receive_data();
			
			/* Process & Store new INS data */
			INSDataProcess();
			/* Process & Store new GPS data */
			GPSDataProcess();
			
			/* Check if is started */
			if (ISstarted){
				
				
				/* Mechanize and EKF */
				insgps_v4_0(zI, zG, gpsflag, dt, g0, a, e, we, 
										Q, R, PVA, bias, Pk_1, xk_1);

				/* Transmit PVA to UART5 */
				send_PVA();
				//send_data();
			}
			else{
				/* Check if GPS has been received */ //xem CRC
				if (gpsflag == YESGPS){
					
					/* Init const params (a,e,P0,Q0)*/
					initialize();

					ISstarted = 1;
				}
			}
			gpsflag = NOGPS;
			elapsedTime=TIM7->CNT;
			TIM7->CR1 = 0;		// stop Timer7
		}
  }
}
