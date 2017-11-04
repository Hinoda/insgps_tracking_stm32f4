#include "stm32f4xx.h"
#include "misc.h"
#include "system_timetick.h"
#include "driver.h"
#include "insgps_v1_0.h"
#include "IMU_Quest.h"                 /* Model's header file */
#include "rtwtypes.h"                  /* MathWorks types */
#include "initialize.h"
#include "genDataGetProcess.h"

static boolean_T OverrunFlag = 0;
static _Bool started = 0;
extern double zG[7];
extern uint8_t comma_counter;
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
  
  /* Enable SysTick at 5ms interrupt */
  SysTick_Config(SystemCoreClock/100);//10ms
  
  delay_01ms(20000);
  init_board();
	ElapseDef_01ms(10000);

  //reset_adis();
	enable_gps();
  
  IMU_Quest_initialize();
  
  while(1){
 		if(tick_flag){
			tick_flag = 0;
			/* Reset Elapse Timer*/
			TIM7->SR  = 0;		// clear overflow flag
			TIM7->CR1 = 1;		// enable Timer6
			TIM7->CNT = 0;
			/* Calcutalte zI+zG data */
			/* Find Euler angles */
			rt_OneStep();
			/* Process & Store new INS data */
			INSDataProcess();
			/* Request GPS data */
			receive_data();
			/* Process & Store new GPS data */
			GPSDataProcess();
			/* Check if is started */
			if (started){
				/* Mechanize and EKF */
				insgps_v1_0();
				/* Transmit PVA to UART5 */
				send_PVA();
				//send_data();
			}
			else{
				/* Check if GPS has been received */ //xem CRC
				if (gpsflag == YESGPS){
					/* Init const params (a,e,P0,Q0)*/
					initialize();
					started = 1;
				}
			}
			gpsflag = NOGPS;
			elapsedTime=TIM7->CNT;
			TIM7->CR1 = 0;		// stop Timer7
		}
  }
}


