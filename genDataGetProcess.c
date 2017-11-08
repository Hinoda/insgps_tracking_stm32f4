#include "genDataGetProcess.h"
#include "initialize.h"
#include "driver.h"
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
double zI[10];
double zG[7]={0,0,0,0,0,0,0};
double gpstime;
double head;
double speed;
gpsflag_t gpsflag = NOGPS;
static _Bool ISstarted = 0;

//uint8_t pxsbuff[XSBUFF_SIZE];
double height_=0;
void GPSDataProcess(void)
{
	//memcpy(pxsbuff,xsbuff,XSBUFF_SIZE);
	for (uint8_t i=0;i<7;i++)	zG[i]=0;
	uint8_t comma_idx[23];
	uint8_t comma_counter=0;
	
	for(uint8_t pGPS=0;pGPS<130;pGPS++){
		if(xsbuff[pGPS]==','){
			comma_idx[comma_counter++]=pGPS;
		}
	}
	
	//gpsflag=NOGPS;//check them CRC
	//if ((zG[1]!=0)&(zG[2]!=0)&(zG[3]!=0)&(comma_counter==23)){//zG
	//	gpsflag = YESGPS;
	//}
	
//	if (comma_counter == 23){//check them CRC
		/**************************************************************************************/

		//---------SPEED------------
		ToDoubleAddr(&speed,xsbuff+comma_idx[4]+1,comma_idx[5]-comma_idx[4]-1);
		speed=speed*0.51444444;
		//---------HEAD------------	
		if (ISstarted){		
			ToDoubleAddr(&head,xsbuff+comma_idx[0]+1,comma_idx[1]-comma_idx[0]-1);
			head=head*xPI_180;
		}
		else{
			head=euler[2]*0.1*xPI_180;
		}
		zG[4]=speed*cos(head);
		zG[5]=speed*sin(head);
		//---------TIME-------------
		ToDoubleAddr(&zG[0],xsbuff+comma_idx[9]+1,comma_idx[10]-comma_idx[9]-1);
		//---------HEIGHT-----------
		ToDoubleAddr(&zG[3],xsbuff+comma_idx[17]+1,comma_idx[18]-comma_idx[17]-1);
		//---------LATLON-----------
		pos2googAddr(&zG[1],xsbuff+comma_idx[10]+1,comma_idx[11]-comma_idx[10]-1,xsbuff[comma_idx[11]+1]);
		pos2googAddr(&zG[2],xsbuff+comma_idx[12]+1,comma_idx[13]-comma_idx[12]-1,xsbuff[comma_idx[13]+1]);
		zG[1]=zG[1]*xPI_180;
		zG[2]=zG[2]*xPI_180;
		/**************************************************************************************/
		/** TODO (HaiDang1#1#): watch turns of previous fb */
		zG[6]=(height_-zG[3])/(zG[0]-gpstime);
		height_=zG[3];
		gpstime=zG[0];
		/** xem height -- neu mat 2 lan gps thi sao */
		//gpsflag=NOGPS;//check them CRC
		if ((zG[1]!=0)&(zG[2]!=0)&(zG[3]!=0)){
			gpsflag = YESGPS;
		}
//	}
}

void INSDataProcess(void)
{
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
void ToDoubleAddr(double* dest, uint8_t* from, uint8_t length)
{
  char strdest[length];
	memcpy(strdest,from,length);
	strdest[length]='\0';
	*dest = atof(strdest);
}

void pos2googAddr(double* dest, uint8_t* from, uint8_t length, uint8_t _cPos)
{
	double mnt,out;
	uint8_t deg;
	char strMnt[8], strDeg[3], strtemp[length];
	memcpy(strtemp,from,length);
	strtemp[length]='\0';
	for(uint8_t pGPS=0;pGPS<length;pGPS++){
		if(strtemp[pGPS]=='.'){
			memcpy(strMnt,strtemp+pGPS-2,length-pGPS+2);
			strMnt[length-pGPS+2]='\0';
			mnt = atof(strMnt);
			//----------------------------
			memcpy(strDeg,strtemp,pGPS-2);
			strDeg[pGPS-2]='\0';
			deg = atoi(strDeg);
			break;
		}
	}
	out=deg+mnt/60;
	if (_cPos=='W'|_cPos=='S')		*dest=-out;
	else if (_cPos=='E'|_cPos=='N') *dest=out;
	else *dest=0;
}
