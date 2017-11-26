#include "driver.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
double  marg[15];
double  euler[3];
int16_t elapsedTime1;
uint8_t txbuff[TXBUFF_SIZE];
uint8_t gpsbuff[RXBUFF_SIZE];
uint8_t rtkbuff[RXBUFF_SIZE];
uint8_t xsbuff[XSBUFF_SIZE];
uint16_t rxlen  = 0;
uint16_t rxflag = 0;
uint16_t rxfull = 0;



void init_board(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	DMA_InitTypeDef  DMA_InitStructure;
	SPI_InitTypeDef SPI_InitStructure; 
	USART_InitTypeDef USART_InitStructure; 
	NVIC_InitTypeDef  NVIC_InitStructure;  
	
	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	/* Enable DMA1 clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	/* Enable SPI clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	/* Enable UART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
	
	
	/* DMA1 Stream5 Channel4 for USART2 Rx configuration */			
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rtkbuff;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = RXBUFF_SIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);
	DMA_Cmd(DMA1_Stream5, ENABLE);
	
	/* DMA1 Stream2 Channel4 for USART4 Rx configuration */			
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)gpsbuff;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = RXBUFF_SIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream2, &DMA_InitStructure);
	DMA_Cmd(DMA1_Stream2, ENABLE);
	
	/* DMA1 Stream7 Channel4 for UART5 Tx configuration */			
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART5->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)txbuff;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = TXBUFF_SIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream7, &DMA_InitStructure);
	DMA_Cmd(DMA1_Stream7, ENABLE);
	
	/* Enable DMA1 Stream2 Interrupt to the highest priority */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	/* Transfer complete interrupt mask */
	DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);
	
	/* Enable DMA1 Stream5 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	/* Transfer complete interrupt mask */
	DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);
	
	/* Configure PA10 (ADIS_RST) in pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_10);
	delay_01ms(10000);
	GPIO_SetBits(GPIOA,GPIO_Pin_10);
	delay_01ms(10000);
	
	
	/* Configure PB1 (PULSE) in pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* Configure PB12 SPI_SS in pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
	
	/* Connect SPI2 pins to AF */  
	//GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2); 
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
	
	/* GPIO Configuration for SPI2 (PB12, PB13, PB14, PB15) */
	//GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* SPI2 configuration for ADIS16405 interface*/
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; //clk = 168/4/64
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	
	SPI_Init(SPI2, &SPI_InitStructure);
	
	/* Enable SPI NSS*/
	SPI_SSOutputCmd(SPI2, ENABLE);
	SPI_Cmd(SPI2, ENABLE);
	
	/* Connect UART4, UART5 and USART2 pins to AF */  
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4); 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
	//GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
	
	/* GPIO Configuration for USART2 (PA2,PA3), UART4 (PA0,PA1), UART5 (PC12,PD2) */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	
	/* USARTx configuration */
	USART_InitStructure.USART_BaudRate = brPC;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_Init(UART5, &USART_InitStructure);
	
	USART_InitStructure.USART_BaudRate = brIMU;
	USART_Init(USART2, &USART_InitStructure);
	USART_Init(UART4, &USART_InitStructure);
	
	/* Enable USART2 UART4 UART5 DMA */
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);  
	USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
	USART_DMACmd(UART5, USART_DMAReq_Tx, ENABLE);
	
	/* Enable UART 5*/
	USART_Cmd(UART5, ENABLE);
}

void DMA1_Stream2_IRQHandler(void){
	/* Clear the DMA1_Stream2 TCIF2 pending bit */
	DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);

	/* set rxfull = 1 to inform that the rxbuff is full */
	rxfull = 1;

	/* reload new buff size for next reception */
	DMA1_Stream2->NDTR = RXBUFF_SIZE;
	DMA_Cmd(DMA1_Stream2, ENABLE);
}

void DMA1_Stream5_IRQHandler(void){
	/* Clear the DMA1_Stream5 TCIF2 pending bit */
	DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);

	/* set rxfull = 1 to inform that the rxbuff is full */
	rxfull = 1;

	/* reload new buff size for next reception */
	DMA1_Stream5->NDTR = RXBUFF_SIZE;
	DMA_Cmd(DMA1_Stream5, ENABLE);
}

void enable_gps(void){
	USART_Cmd(UART4, ENABLE);
	USART_Cmd(USART2, ENABLE);
}

void receive_data(void){
	uint16_t i, k, k1, k2, count;

	k1 = DMA1_Stream2->NDTR;		 // gps UART4					/* giam con 0 la nhan xong */
	k1 = RXBUFF_SIZE - k1;											/* size-so byte con phai gui*/

	k2 = DMA1_Stream5->NDTR;		 // rtk USART2
	k2 = RXBUFF_SIZE - k2;

	/* check if GPS and RTK data are unavailable then skip */
	if ((k1==0)&&(k2==0))		return;								/* so byte da nhan>0 */
	
	if (k1) {	 			// check GPS data
		k = k1;
		count = 0;
		for (i=0; i<k; i++)
			if (gpsbuff[i]==0x0A)	count++;

		/* check if receiving enough GPS data */
		if ((gpsbuff[k-1]==0x0A) && (count>=GPS_SENTENCES)){
			for (i=0; i<k; i++)	xsbuff[i] = gpsbuff[i];
			rxlen  = k;
			rxflag = 1;
		
			/* reload new buff size for next reception */
			DMA_Cmd(DMA1_Stream2, DISABLE);
			DMA1_Stream2->NDTR = RXBUFF_SIZE;
			DMA_Cmd(DMA1_Stream2, ENABLE);
		}
	}
	else {					// check RTK data
		k = k2;
		count = 0;
		for (i=0; i<k; i++)
			if (rtkbuff[i]==0x0A)	count++;

		/* check if receiving enough GPS data */
		if ((rtkbuff[k-1]==0x0A) && (count>=RTK_SENTENCES)){
			for (i=0; i<k; i++)	xsbuff[i] = rtkbuff[i];
			rxlen  = k;
			rxflag = 1;
		
			/* reload new buff size for next reception */
			DMA_Cmd(DMA1_Stream5, DISABLE);
			DMA1_Stream5->NDTR = RXBUFF_SIZE;
			DMA_Cmd(DMA1_Stream5, ENABLE);
		}
	}
}
	

void reset_adis(void){
  	GPIO_ResetBits(GPIOA, GPIO_Pin_10);
  	delay_01ms(1000);
  	GPIO_SetBits(GPIOA, GPIO_Pin_10);
  	delay_01ms(1000);
}


/*
//void read_adis(void){
//  uint16_t i, j, tbuff;
//  double temp;

//  GPIO_ResetBits(GPIOB,GPIO_Pin_12);		// ADIS_STE = 0
//  for(j=0; j<500; j++);
//  
//  SPI_I2S_SendData(SPI2, 0x3E00);				// burst mode
//  
//	for(i=0; i<11; i++){ 
//		while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
//    tbuff = SPI_I2S_ReceiveData(SPI2);
//		SPI_I2S_SendData(SPI2, 0x0);

//		tbuff &= 0x3FFF;
//		if (tbuff >= 0x2000)
//	  		temp = (double)(tbuff - 0x4000);
//		else
//	  		temp = (double)tbuff;

//		if (i==1)
//	  		marg[i-1] = temp * 2.418;				// SUPPLY_OUT (unit mV)
//		else if ((i==2)||(i==3)||(i==4))
//	  		marg[i-1] = temp * 0.8726646;		// GYRO_OUT (unit mrad/s)
//		else if ((i==5)||(i==6)||(i==7))
//	  		marg[i-1] = temp * 3.333;				// ACC_OUT (unit mg)
//		else if ((i==8)||(i==9)||(i==10))
//	  		marg[i-1] = temp * 0.5;					// MAG_OUT (unit mgauss)
//  }

//  while (!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
//  tbuff = SPI_I2S_ReceiveData(SPI2);

//  tbuff &= 0x0FFF;
//  if (tbuff >= 0x800)
//	  	temp = (double)(tbuff - 0x1000);
//  else
//	  	temp = (double)tbuff;

//  marg[i-1] = temp * 13.6 + 2500.0;			// TEM_OUT (don vi 0.01oC)

//  while (!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
//  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY));

//  for(j=0; j<500; j++);
//	GPIO_SetBits(GPIOB,GPIO_Pin_12);			// ADIS_STE = 1
// 
//	// Axis transformation -x,y,-z
//	marg[1] = -marg[1];
//	marg[3] = -marg[3];
//	marg[4] = -marg[4];
//	marg[6] = -marg[6];
//	marg[7] = -marg[7];
//	marg[9] = -marg[9];

////	marg[4] = -marg[4];
////	marg[5] = -marg[5];
////	marg[6] = -marg[6];

//	// Magnetic offset calibration
//	marg[7] = marg[7] - 10;
//   //marg[8] = marg[8] + 0;
//   marg[9] = marg[9] - 10;
//	
//}
*/
void read_adis(void)
{
	uint16_t i, j, tbuff;
	double temp;

	GPIO_ResetBits(GPIOB,GPIO_Pin_12);		// ADIS_STE = 0
	for(j=0; j<1000; j++);

	SPI_I2S_SendData(SPI2, 0x3E00);				// burst mode

	for(i=0; i<11; i++)
	{
		while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
		tbuff = SPI_I2S_ReceiveData(SPI2);
		SPI_I2S_SendData(SPI2, 0x0);

		tbuff &= 0x3FFF;
		if (tbuff >= 0x2000)
	  		temp = (double)(tbuff - 0x4000);
		else
	  		temp = (double)tbuff;

		if (i==1)
	  		marg[i-1] = temp * 2.418;			// SUPPLY_OUT (unit mV)
		else if ((i==2)||(i==3)||(i==4))
	  		marg[i-1] = temp * 0.87266;		// GYRO_OUT (unit mrad/s)
		else if ((i==5)||(i==6)||(i==7))
	  		marg[i-1] = temp * 3.33;			// ACC_OUT (unit mg)
		else if ((i==8)||(i==9)||(i==10))
	  		marg[i-1] = temp * 0.5;				// MAG_OUT (unit mgauss)
	}

	while (!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
	tbuff = SPI_I2S_ReceiveData(SPI2);

	tbuff &= 0x0FFF;
	if (tbuff >= 0x800)
	  	temp = (double)(tbuff - 0x1000);
	else
	  	temp = (double)tbuff;

	marg[i-1] = temp * 14.0 + 2500.0;		// TEM_OUT (don vi 0.01oC)

	while (!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY));

	for(j=0; j<1000; j++)
	;
	GPIO_SetBits(GPIOB,GPIO_Pin_12);		// ADIS_STE = 1

	// Axis transformation -x,y,-z
	
	marg[1] = -marg[1];
	marg[3] = -marg[3];
	marg[4] = -marg[4];
	marg[6] = -marg[6];
	marg[7] = -marg[7];
	marg[9] = -marg[9];
	

	//marg[4] = -marg[4] + 10.0;
	//marg[5] = -marg[5] + 5.0;
	//marg[6] = -marg[6] + 12.0;

//	marg[4] = -marg[4];
//	marg[5] = -marg[5];
//	marg[6] = -marg[6];
	
}


void cmdr_adis(uint16_t *buff, uint16_t addr, uint16_t N){
  	uint16_t i, j, addr1, temp;

  	GPIO_ResetBits(GPIOB,GPIO_Pin_12);		// ADIS_STE = 0
    for(j=0; j<500; j++)
	;

	addr1 = addr; 
	temp  = (addr1<<8);

  	SPI_I2S_SendData(SPI2, (uint16_t)temp);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
	temp = SPI_I2S_ReceiveData(SPI2);

	for(j=0; j<500; j++)
	;
	GPIO_SetBits(GPIOB,GPIO_Pin_12);		// ADIS_STE = 1			

	delay_us(10);
	
  	for(i=0; i<N; i++){
						
		GPIO_ResetBits(GPIOB,GPIO_Pin_12);		// ADIS_STE = 0
		for(j=0; j<500; j++)
		;
		
    	addr1 = addr1 + 2; 
		temp = (addr1<<8);

  		SPI_I2S_SendData(SPI2, (uint16_t)temp);
		while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
		buff[i] = SPI_I2S_ReceiveData(SPI2);
	
		for(j=0; j<500; j++)
		;
		GPIO_SetBits(GPIOB,GPIO_Pin_12);		// ADIS_STE = 1	
			
		delay_us(10);
  	}
}

void cmdw_adis(uint16_t data){
  	uint16_t temp, j;
	(void)temp;
  	GPIO_ResetBits(GPIOB,GPIO_Pin_12);		// ADIS_STE = 0
	for(j=0; j<500; j++)
	;

  	SPI_I2S_SendData(SPI2, (uint16_t)data);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
	temp = SPI_I2S_ReceiveData(SPI2);
	
	for(j=0; j<500; j++);
	GPIO_SetBits(GPIOB,GPIO_Pin_12);		// ADIS_STE = 1	
	
	delay_us(10);	
}

void delay_us(uint16_t period){

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  	TIM6->PSC = 83;		// clk = SystemCoreClock / 4 / (PSC+1) *2 = 1MHz
  	TIM6->ARR = period-1;
  	TIM6->CNT = 0;
  	TIM6->EGR = 1;		// update registers;

  	TIM6->SR  = 0;		// clear overflow flag
  	TIM6->CR1 = 1;		// enable Timer6

  	while (!TIM6->SR);
    
  	TIM6->CR1 = 0;		// stop Timer6
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, DISABLE);
}

void delay_01ms(uint16_t period){

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  	TIM6->PSC = 8399;		// clk = SystemCoreClock / 4 / (PSC+1) *2 = 10KHz
  	TIM6->ARR = period-1;
  	TIM6->CNT = 0;
  	TIM6->EGR = 1;		// update registers;

  	TIM6->SR  = 0;		// clear overflow flag
  	TIM6->CR1 = 1;		// enable Timer6

  	while (!TIM6->SR);
    
  	TIM6->CR1 = 0;		// stop Timer6
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, DISABLE);
}

void ElapsedDef_001ms(uint16_t period){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	TIM7->PSC = 839;		// clk = SystemCoreClock / 4 / (PSC+1) *2 = 10KHz
	TIM7->ARR = period-1;
	TIM7->CNT = 0;
	TIM7->EGR = 1;		/* Generate an update event to reload the Prescaler value immediately */
}

void ElapsedGet(int16_t* elapsedTime){
	*elapsedTime=TIM7->CNT;
	TIM7->CR1 = 0;		// stop Timer7
}

void ElapsedRestart(void){
	TIM7->SR  = 0;		// clear overflow flag
	TIM7->CNT = 0;
	TIM7->CR1 = 1;		// enable Timer7
}
void IntToStr5(int16_t u, uint8_t *y){
	int16_t a;
	
	a = u;
	if (a<0){ 
		a = -a; 
		y[0] = '-';
	}
	else y[0] = ' ';
	
	y[4] = a % 10 + 0x30; 
	a = a / 10;
	y[3] = a % 10 + 0x30; 
	a = a / 10;
	y[2] = a % 10 + 0x30; 
	a = a / 10;
	y[1] = a + 0x30;
}

void IntToStr6(int32_t u, uint8_t *y){
	int32_t a;
     
	a = u;
	if (a<0){ 
		a = -a; 
		y[0] = '-';
	}
	else y[0] = ' ';

	y[5] = a % 10 + 0x30; 
	a = a / 10;	
	y[4] = a % 10 + 0x30; 
	a = a / 10;
	y[3] = a % 10 + 0x30; 
	a = a / 10;
	y[2] = a % 10 + 0x30; 
	a = a / 10;
	y[1] = a + 0x30;
}
void IntToStr8(int32_t u, uint8_t *y){
	int32_t a;
	a = u;
	if (a<0){ 
		a = -a; 
		y[0] = '-';
	}
	else y[0] = ' ';

	y[7] = a % 10 + 0x30; 
	a = a / 10;	
	y[6] = a % 10 + 0x30; 
	a = a / 10;
	y[5] = a % 10 + 0x30; 
	a = a / 10;
	y[4] = a % 10 + 0x30; 
	a = a / 10;
	y[3] = a % 10 + 0x30; 
	a = a / 10;
	y[2] = a % 10 + 0x30; 
	a = a / 10;
	y[1] = a + 0x30;
	
}

void send_data(void)
{
	uint16_t 	i, k;
	int16_t  	temp;
	double 		dtemp;//hold data

	txbuff[0] = 10;//start line with LF
	k = 1;
	
	for (i=0; i<3; i++){
		dtemp = euler[i]*10; 
		temp  = dtemp;
		IntToStr6(temp, &txbuff[k]);	
		k = k + 6;
		txbuff[k++] = ' ';
	}
	
//		for (i=1; i<4; i++){
//	  dtemp = marg[i]*10;
//		temp = dtemp;
//		IntToStr6(temp, &txbuff[k]);	
//		k = k + 6;
//		txbuff[k++] = ' ';
//	}

	for (i=1; i<11; i++){
	  dtemp = marg[i];
		temp = dtemp;
		IntToStr5(temp, &txbuff[k]);	
		k = k + 5;
		txbuff[k++] = ' ';
	}

/*	
	temp = marg[0];
	IntToStr5(temp, &txbuff[k]);	
	k = k + 5;
	txbuff[k++] = ' ';
*/	
	txbuff[k++] = 13;//CR 0x0d

	receive_data();//data GPS
	if(rxflag){
		txbuff[k++] = 10;//LF 0x0a
		for (i=0; i<(rxlen-1); i++)
			txbuff[k++] = xsbuff[i];
		rxflag = 0;
	}

	DMA_ClearFlag(DMA1_Stream7, DMA_FLAG_TCIF7);
	DMA1_Stream7->NDTR = k;
	DMA_Cmd(DMA1_Stream7, ENABLE);
}
/**
* lan1: 100 100 10 10 10 10 10 10 10 10 10
* lan2: 10000 10000 10000 10000 10000 10000 10000 10000 10000 10000
*
*/
void send_PVA(float PVA[10], float zG[7], bool gpsflag){
	while(DMA_GetCmdStatus(DMA1_Stream7)==ENABLE);
	uint16_t 	i, k;
	int32_t 	temp32;
	float 		ftemp;//hold data

	txbuff[0] = 10;//start line with LF
	k=1;
	/* index */
	ftemp = PVA[0]; 
	temp32  = (int32_t)ftemp;
	IntToStr8(temp32, &txbuff[k]);
	k = k + 8;
	txbuff[k++] = ' ';
	//k++;
	/* lat lon => [rad]*10^6 */
	for (i=1; i<3; i++){
		ftemp = PVA[i]*1000000;
		temp32  = (int32_t)ftemp;
		IntToStr8(temp32, &txbuff[k]);
		k = k + 8;
		txbuff[k++] = ' ';
	}
	/* height VN VE VD => [m m/s]*10^3 */
	for (i=3; i<7; i++){
		ftemp = PVA[i]*1000;
		temp32  = (int32_t)ftemp;
		IntToStr6(temp32, &txbuff[k]);
		k = k + 6;
		txbuff[k++] = ' ';
	}
	/* roll pitch yaw => [rad]*10^6 */
	for (i=7; i<10; i++){
		ftemp = PVA[i]*1000000;
		temp32  = (int32_t)ftemp;
		IntToStr8(temp32, &txbuff[k]);
		k = k + 8;
		txbuff[k++] = ' ';
	}
	txbuff[k++] = 13;//CR 0x0d

	if(gpsflag){
		txbuff[k++] = 10;//LF 0x0a
		txbuff[k++] = 'G';
		txbuff[k++] = ' ';
		/* time => [s]*10 */
		ftemp = zG[0]*10; 
		temp32  = (int32_t)ftemp;
		IntToStr8(temp32, &txbuff[k]);
		k = k + 8;
		txbuff[k++] = ' ';
		
		/* lat lon => [rad]*10^6 */
		for (i=1; i<3; i++){
			ftemp = zG[i]*1000000; 
			temp32  = (int32_t)ftemp;
			IntToStr8(temp32, &txbuff[k]);	
			k = k + 8;
			txbuff[k++] = ' ';
		}
		/* height VN VE VD => [m m/s]*10^3 */
		for (i=3; i<7; i++){
			ftemp = zG[i]*1000; 
			temp32  = (int32_t)ftemp;
			IntToStr6(temp32, &txbuff[k]);
			k = k + 6;
			txbuff[k++] = ' ';
		}
		txbuff[k++] = 13;//CR 0x0d
	}
	k++;
	DMA_ClearFlag(DMA1_Stream7, DMA_FLAG_TCIF7);
	DMA1_Stream7->NDTR = k;
	DMA_Cmd(DMA1_Stream7, ENABLE);
}

void send_zG(float zG[7], int16_t moreInfo){
	while(DMA_GetCmdStatus(DMA1_Stream7)==ENABLE);
	//while(DMA_GetFlagStatus(DMA1_Stream7, DMA_FLAG_TCIF7) == 0);
	//while(DMA1_Stream7->NDTR!= 0);
	//UART5->DR=0;
	uint16_t 	i=0, k=1;
	int32_t 	temp32;
	int16_t 	temp16;
	float 		ftemp;//hold data

	txbuff[0] = 10;//start line with LF
	/* index */
	temp16  = (int16_t)moreInfo;
	IntToStr5(temp16, &txbuff[k]);
	k = k + 5;
	txbuff[k++] = ' ';
	
	/* time => [s]*10 */
	ftemp = zG[0]*10; 
	temp32  = (int32_t)ftemp;
	IntToStr8(temp32, &txbuff[k]);
	k = k + 8;
	txbuff[k++] = ' ';
	
	/* lat lon => [rad]*10^6 */
	for (i=1; i<3; i++){
		ftemp = zG[i]*1000000; 
		temp32  = (int32_t)ftemp;
		IntToStr8(temp32, &txbuff[k]);	
		k = k + 8;
		txbuff[k++] = ' ';
	}
	
	/* height VN VE VD => [m m/s]*10^3 */
	for (i=3; i<7; i++){
		ftemp = zG[i]*1000; 
		temp32 = (int32_t)ftemp;
		IntToStr6(temp32, &txbuff[k]);
		k = k + 6;
		txbuff[k++] = ' ';
	}
	txbuff[k++] = 13;
	k++;
	DMA_ClearFlag(DMA1_Stream7, DMA_FLAG_TCIF7);
	DMA1_Stream7->NDTR = k;
	DMA_Cmd(DMA1_Stream7, ENABLE);
}
void sendMode(uint8_t* myinfo){
	while(DMA_GetCmdStatus(DMA1_Stream7)==ENABLE);
	//while(DMA_GetFlagStatus(DMA1_Stream7, DMA_FLAG_TCIF7) == 0);
	//while(DMA1_Stream7->NDTR!= 0);
	//UART5->DR=0;
	uint16_t 	i = 0, k = 1;
	while(myinfo[i]!='\0') i++;
	txbuff[0] = 10;//start line with LF
	txbuff[k++] = 42;
	txbuff[k++] = ' ';
	memcpy(&txbuff[k],myinfo,i);
	k+=i;
	txbuff[k++] = 13;
	DMA_ClearFlag(DMA1_Stream7, DMA_FLAG_TCIF7);
	DMA1_Stream7->NDTR = k;
	DMA_Cmd(DMA1_Stream7, ENABLE);
}

void sendElapsed(uint16_t elapsedTime){
	while(DMA_GetCmdStatus(DMA1_Stream7)==ENABLE);
	uint16_t 	k = 1;
	int16_t  	temp16;
	
	txbuff[0] = 10;//start line with LF
	txbuff[k++] = 42;
	txbuff[k++] = ' ';
	temp16  = (int16_t)elapsedTime;
	IntToStr5(temp16, &txbuff[k]);
	k = k + 5;
	//txbuff[k++] = ' ';
	
	txbuff[k++] = 13;
	//k++;
	DMA_ClearFlag(DMA1_Stream7, DMA_FLAG_TCIF7);
	DMA1_Stream7->NDTR = k;
	DMA_Cmd(DMA1_Stream7, ENABLE);
}