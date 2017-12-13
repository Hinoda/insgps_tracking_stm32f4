#define TXBUFF_SIZE	300		//74 
#define RXBUFF_SIZE	300 
#define XSBUFF_SIZE	RXBUFF_SIZE 
#define GPS_SENTENCES	2
#define RTK_SENTENCES	2
#define brPC 460800
#define brIMU 112500
#include "stm32f4xx.h"
#include "misc.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#define UARTprintf(...) {\
							uint8_t buffer[128];\
							uint8_t len = sprintf(buffer,__VA_ARGS__);\
                            sendMode(buffer);\
												}
extern int16_t elapsedTime1;
extern double   marg[15];
extern double   euler[3];
extern uint8_t  rxbuff[RXBUFF_SIZE];
extern uint8_t  txbuff[TXBUFF_SIZE];
extern uint8_t  xsbuff[XSBUFF_SIZE];
extern uint16_t rxflag;
extern uint16_t rxlen;
//---FUNCTION------------------------
void init_board(void);
//-----------------------------------
void delay_us(uint16_t period);
void delay_01ms(uint16_t period);
//-----------------------------------
/* Define TIM7 for calculating elapsed time */
void ElapsedDef_001ms(uint16_t period);
void ElapsedGet(int16_t* elapsedTime);
void ElapsedRestart(void);
//-----------------------------------
void reset_adis(void);
void read_adis(void);
//-----------------------------------
void cmdr_adis(uint16_t *buff, uint16_t addr, uint16_t N);
void cmdw_adis(uint16_t data);
//-----------------------------------
/*
 * using UART5, transmit PVA/zG/data results to laptop
 * transmit buffer is txbuff[300]
 */
void send_zG(double zG[7], int16_t moreInfo);
void send_PVA(double* PVA, double* zG, bool gpsflag);
void sendElapsed(uint16_t elapsedTime);
void sendMode(uint8_t* myinfo);
void send_data(void);
//-----------------------------------
/* receive GPS/RTK, store in xsbuff
 * check rxflag when (last char of gpsbuff == 0x0A) && (NMEA received == 2)
 */
void receive_data(void);
//-----------------------------------
void enable_gps(void);
void DMA1_Stream2_IRQHandler(void);
