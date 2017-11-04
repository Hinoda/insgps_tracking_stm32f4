#define TXBUFF_SIZE	300		//74 
#define RXBUFF_SIZE	300 
#define XSBUFF_SIZE	RXBUFF_SIZE 
#define GPS_SENTENCES	2
#define RTK_SENTENCES	2
#define PI 3.14159265

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "initialize.h"

extern double xPI_180;
extern double x180_PI;
extern double zI[10];
extern double zG[7];
extern double  marg[15];
extern double  euler[3];
extern uint8_t xsbuff[XSBUFF_SIZE];
//extern uint8_t gpsbuff[RXBUFF_SIZE];
extern void pos2googAddr(double* dest, uint8_t* from, uint8_t length, uint8_t _cPos);
extern void ToDoubleAddr(double* dest, uint8_t* from, uint8_t length);
extern void GPSDataProcess(void);
extern void INSDataProcess(void);
