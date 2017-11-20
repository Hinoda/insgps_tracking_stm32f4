
/* Includes ------------------------------------------------------------------*/
#include "system_timetick.h"

uint32_t	tick_count;
uint32_t	tick_flag;


void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
	sendMode("HardFault");
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
	sendMode("MemManage");
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
	sendMode("BusFault");
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
	sendMode("UsageFault");
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}
	  
void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
	//sendMode("Systick");
  tick_flag = 1;
  tick_count++;
}

