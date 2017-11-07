#define TXBUFF_SIZE	300		//74 
#define RXBUFF_SIZE	300 
#define XSBUFF_SIZE	RXBUFF_SIZE 
#define GPS_SENTENCES	2
#define RTK_SENTENCES	2
#define brPC 460800
#define brIMU 112500
#include "stm32f4xx.h"
#include "misc.h"
extern double   marg[15];
extern double   euler[3];
extern uint8_t  rxbuff[RXBUFF_SIZE];
extern uint8_t  txbuff[TXBUFF_SIZE];
extern uint8_t  xsbuff[XSBUFF_SIZE];
extern uint16_t rxflag;
extern uint16_t rxlen;
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
	Define TIM7 for calculating elapsed time
  */
void ElapseDef_001ms(uint16_t period);
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
	using UART5 transmit PVA results to laptop
	transmit buffer is
  */
void send_PVA(void);


/**
	using UART5 transmit INS+GPS string to laptop
	transmit buffer is txbuff
  */
void send_data(void);

/**
	receive GPS/RTK, store in xsbuff
  */
void receive_data(void);
//-----------------------------------


/**
	
  */
void enable_gps(void);

/**
	
  */
void DMA1_Stream2_IRQHandler(void);
