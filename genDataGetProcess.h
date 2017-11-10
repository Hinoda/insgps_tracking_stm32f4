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

extern double zI[10];
extern double zG[7];
extern double gpstime;
extern double head;
extern double speed;
extern int gpsflag;
extern _Bool started;
//extern uint8_t gpsbuff[RXBUFF_SIZE];
void pos2googAddr(double* dest, uint8_t* from, uint8_t length, uint8_t _cPos);
void ToDoubleAddr(double* dest, uint8_t* from, uint8_t length);
void GPSDataProcess(void);
void INSDataProcess(void);
