#define TXBUFF_SIZE	300		//74 
#define RXBUFF_SIZE	300 
#define XSBUFF_SIZE	RXBUFF_SIZE 
#define GPS_SENTENCES	2
#define RTK_SENTENCES	2
//#include "genDataGetProcess.h"
#include "stm32f4xx.h"
#include "misc.h"
extern double   marg[15];
extern double   euler[3];
extern uint8_t  rxbuff[RXBUFF_SIZE];
extern uint8_t  txbuff[TXBUFF_SIZE];
extern uint8_t  xsbuff[XSBUFF_SIZE];
extern uint16_t rxflag;
/**
	
  */
void init_board(void);
//-----------------------------------

/**
	
  */
void delay_us(uint16_t period);

/**
	
  */
void delay_01ms(uint16_t period);
//-----------------------------------

/**
	
  */
void reset_adis(void);

/**
	
  */
void read_adis(void);
//-----------------------------------

/**
	cmd read adis
  */
void cmdr_adis(uint16_t *buff, uint16_t addr, uint16_t N);

/**
	cmd write adis
  */
void cmdw_adis(uint16_t data);
//-----------------------------------




/**
	
  */
void send_data(void);

/**
	receive GPS
  */
void receive_data(void);
//-----------------------------------


/**
	
  */
void enable_gps(void);

/**
	
  */
void DMA1_Stream2_IRQHandler(void);
