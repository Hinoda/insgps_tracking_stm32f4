#define TXBUFF_SIZE	300		//74 
#define RXBUFF_SIZE	300 
#define XSBUFF_SIZE	RXBUFF_SIZE 
#define GPS_SENTENCES	2
#define RTK_SENTENCES	2
#define PI 3.141592654

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

extern float g_zI[10];
extern float g_zG[7];
extern float gpstimeOld;
extern float head;
extern float speed;
extern float heightOld;
extern int numOfSat;


//extern uint8_t gpsbuff[RXBUFF_SIZE];
void posToGoog(float* dest, uint8_t* from, uint8_t length, uint8_t _cPos);
void pos2googAddr(float* dest, uint8_t* from, uint8_t length, uint8_t _cPos);
void ToFloat(float* dest, uint8_t* from, uint8_t length);
void ToInt(int* dest, uint8_t* from, uint8_t length);
void AssignGPSComma(uint8_t* commaIndex);
bool CheckGPSflag(uint8_t* commaIndex);
void GPSDataProcess(bool my_gpsAvail, bool my_firstTime, uint8_t* commaIndex);
void INSDataProcess(void);
