#include "stm32f4xx.h"
#include "misc.h"
#include "system_timetick.h"
#include "driver.h"

#include "IMU_Quest.h"                 /* Model's header file */
#include "rtwtypes.h"                  /* MathWorks types */
#include "genDataGetProcess.h"
static boolean_T OverrunFlag = 0;

extern uint16_t rxflag;

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
double a;
int main(void)
{
  //uint16_t i;
  //uint16_t cmdbuff[16];
  
  /* Enable SysTick at 5ms interrupt */
  SysTick_Config(SystemCoreClock/200);
  
  delay_01ms(20000);
  init_board();
  //reset_adis();
	enable_gps();
  
  IMU_Quest_initialize();
  
  while(1){
		if(tick_flag){
			tick_flag = 0;
			
			/* Find Euler angles */
			rt_OneStep();
			
			/* Request GPS data */
			receive_data();
			
			/* Process IMU-GPS data */
			DataProcess();//can't use rxflag
			
			/* Mechanize and EKF */
			
			
			/* Transmit PVA to laptop */
			
			/**
			not yet defined
			HINT: tham khao send_data, dung UART5 DMA7
			*/
			//send_PVA();
		}
  }   
}


