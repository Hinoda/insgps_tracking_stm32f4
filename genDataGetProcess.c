#include "genDataGetProcess.h"
#include "initialize.h"
#include "driver.h"
#include <math.h>
/**
 * @params   IN: gpsbuff[300]
 *               euler[3]=[roll pitch yaw]
 *               marg[15]=[vol; wx wy wz; ax ay az; mx my mz]
 *           OUT: PVA[10x1]
 * @brief    collect gpsbuff, euler, marg then calculate kth PVA
 * @format
	% zI   =   [tt  phi   thta   psi   , wx      wy      wz,  ax   ay   az];
	% zG   =   [tt  lat   lon    h     , Vn      Ve      Vd];
 */
int ind;
float zI[10];
float zG[7];
float gpstimeOld=0;
float head=0;
float speed=0;
float heightOld=0;
int numofsat=0;


void AssignGPSComma(uint8_t* commaIndex)
{
	uint8_t pGPS=0, commaCounter = 0;
	memset(commaIndex,0,23*sizeof(uint8_t));
	while(pGPS<rxlen)
	{
		if(xsbuff[pGPS]==',')
		{
			*(commaIndex+commaCounter)=pGPS;
			commaCounter++;
		}
		pGPS++;
	}
}

bool CheckGPSflag(uint8_t* commaIndex)
{
	ToInt(&numofsat,xsbuff+*(commaIndex+15)+1,*(commaIndex+16)-*(commaIndex+15)-1);
	if ((numofsat>=4)){
		numofsat=0;
		return true;
	}
	else return false;
}

void GPSDataProcess(bool my_gpsAvail, bool my_firstTime, uint8_t* commaIndex)
{
	for (uint8_t i=0;i<7;i++)	zG[i]=0;
	if (my_gpsAvail){
		//TIME
		ToFloat(&zG[0],xsbuff+*(commaIndex+9)+1,*(commaIndex+10)-*(commaIndex+9)-1);
		
		//HEIGHT
		ToFloat(&zG[3],xsbuff+*(commaIndex+17)+1,*(commaIndex+18)-*(commaIndex+17)-1);
		
		//VD
		if (my_firstTime){
			zG[6]=0;
		}
		else{
			zG[6]=(heightOld-zG[3])/(zG[0]-gpstimeOld);
		}
		heightOld=zG[3];
		gpstimeOld=zG[0];
		
		//LAT-LON
		posToGoog(&zG[1],xsbuff+*(commaIndex+10)+1,*(commaIndex+11)-*(commaIndex+10)-1,xsbuff[*(commaIndex+11)+1]);
		posToGoog(&zG[2],xsbuff+*(commaIndex+12)+1,*(commaIndex+13)-*(commaIndex+12)-1,xsbuff[*(commaIndex+13)+1]);
		zG[1]=zG[1]*xPI_180;
		zG[2]=zG[2]*xPI_180;
		
		//SPEED
		ToFloat(&speed,xsbuff+*(commaIndex+4)+1,*(commaIndex+5)-*(commaIndex+4)-1);
		speed=speed*0.51444444;
		
		//HEAD
		if ((*(commaIndex+1)-*commaIndex-1)>0){
			ToFloat(&head,xsbuff+*(commaIndex+0)+1,*(commaIndex+1)-*(commaIndex+0)-1);
			head=head*xPI_180;
		}
		else{
			head=euler[2]*0.1*xPI_180;
		}
		
		//VN-VE
		zG[4]=speed*cos(head);
		zG[5]=speed*sin(head);
		speed=0;
		head=0;
	}
}

void INSDataProcess(void)
{
	for (uint8_t i=0;i<10;i++)	zI[i]=0;
	zI[0] = ind++;
	zI[1] = euler[0]*0.1*xPI_180;          // euler (unit rad)
	zI[2] = euler[1]*0.1*xPI_180;
	zI[3] = euler[2]*0.1*xPI_180;
	zI[4] = marg[1]*0.001;           // gyro (unit rad/s)
	zI[5] = marg[2]*0.001;
	zI[6] = marg[3]*0.001;
	zI[7] = -marg[4]*9.80665*0.001;   // accel (unit m/s^2)
	zI[8] = -marg[5]*9.80665*0.001;
	zI[9] = -marg[6]*9.80665*0.001;
}

//void ToFloat(float* dest, uint8_t* from, uint8_t length)
//{
//  char strdest[30];//khong dat chieu dai thay doi, nhu malloc, calloc no se luu vao heap
//	memset(strdest, 0, 30);//nen reset bien truoc khi chay
//	memcpy(strdest,from,length);
//	strdest[length]='\0';
//	double temp;
//	
//	*dest = atof(strdest);//ko nen dung atof atoi (thu vien standard) co the dung sscanf
//}
void ToFloat(float* dest, uint8_t* from, uint8_t length)
{
	int phanNguyen = 0;
	int phanThapPhan = 0;
	int viTriDauCham = 0;
	uint8_t i;
	uint32_t soChia = 1;
	for ( i = 0; i < length; i++)
	{
		if((from[i] <= '9')&&(from[i] >= '0')&&(from[i] != '.'))
		{
			phanNguyen = (from[i] - 0x30 ) + phanNguyen * 10;	
		}
		else if (from[i] == '.')
		{
			viTriDauCham = i;
			break;
		}
		else
		{
			*dest=0;
			return;
		}
	}
	for ( i = viTriDauCham + 1; i < length; i++)
	{
		phanThapPhan = (from[i] - 0x30 ) + phanThapPhan * 10;
		soChia *= 10;
	}

	*dest = (float)phanNguyen + (float)(phanThapPhan) / soChia;
}

//void ToInt(int* dest, uint8_t* from, uint8_t length)
//{
//  char strdest[30];
//	memset(strdest, 0, 30);
//	memcpy(strdest,from,length);
//	strdest[length]='\0';
//	*dest = atoi(strdest);
//}

void ToInt(int* dest, uint8_t* from, uint8_t length)
{
	int temp = 0;
	uint8_t i;
	for ( i = 0; i < length; i++)
	{
		if ((from[i] <= '9')&&(from[i] >= '0')){
			temp = (from[i] - 0x30 ) + temp * 10;
		}
		else{
			*dest=0;
			return;
		}
	}
	*dest = temp;
}



//void pos2googAddr(float* dest, uint8_t* from, uint8_t length, uint8_t _cPos)
//{
//	float mnt,out;
//	int deg;
//	char strMnt[8], strDeg[3], strtemp[30];
//	memcpy(strtemp,from,length);
//	strtemp[length]='\0';
//	for(uint8_t pGPS=0;pGPS<length;pGPS++){
//		if(strtemp[pGPS]=='.'){
//			memcpy(strMnt,strtemp+pGPS-2,length-pGPS+2);
//			strMnt[length-pGPS+2]='\0';
//			//mnt = atof(strMnt);
//			ToFloat(&mnt, strMnt, length-pGPS+2);
//			//----------------------------
//			memcpy(strDeg,strtemp,pGPS-2);
//			strDeg[pGPS-2]='\0';
//			//deg = atoi(strDeg);
//			ToInt(&deg, strDeg, pGPS-1);
//			break;
//		}
//	}
//	out=deg+mnt/60;
//	if (_cPos=='W'||_cPos=='S')		*dest=-out;
//	else if (_cPos=='E'||_cPos=='N') *dest=out;
//	else *dest=0;
//}

void posToGoog(float* dest, uint8_t* from, uint8_t length, uint8_t _cPos)
{
	float mnt = 0, out = 0;
	int deg = 0;
	uint8_t* extractMntCursor;
	uint8_t mntLength, degLength;
	for(uint8_t pGPS=0;pGPS<length;pGPS++){
		if(*(from+pGPS)=='.'){//
			extractMntCursor = from + pGPS - 2;
			mntLength = length - pGPS + 2;
			degLength = pGPS-2;
			break;
		}
	}
	ToFloat(&mnt, extractMntCursor, mntLength);
	ToInt(&deg, from, degLength);
	out = (float)deg+mnt/60;
	switch (_cPos)
	{
		case 'N' :
		case 'E' :
			*dest=out;
			break;
		case 'W' :
		case 'S' :
			*dest=-out;
			break;
		default:
			*dest=0;
			break;
	}
}
