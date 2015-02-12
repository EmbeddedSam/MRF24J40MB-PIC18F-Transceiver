#ifndef _HARDWARE_PROFILE_H
#define _HARDWARE_PROFILE_H
#if !defined(__C18__)
	#define __C18__
#endif

#include "p18f8722.h"
#include "spi.h"
#include "stdio.h"
#include "delays.h"
#include "string.h"		// memset()

#define HARDWARE_SPI							// vs. software bit-bang (slower)

#define BYTEPTR(x)			((UINT8*)&(x))		// converts x to a UINT8* for bytewise access ala x[foo]

#define FOSC				(10000000)			// PIC18 cpu clock speed, Hz
//BAUD RATES REMOVED AS USART NOT USED

//NEXT 3 LINES REMOVED AS READCORETIMER() CANNOT BE USED
//#define ONE_SECOND ((FOSC*5)/2)						// 1s of PIC32 core timer ticks (== Hz)
//#define MS_TO_CORE_TICKS(x) ((UINT64)(x)*ONE_SECOND/1000)
//#define CT_TICKS_SINCE(tick) (ReadCoreTimer() - (tick))			// number of core timer ticks since "tick"

//typedef enum _BOOL { FALSE = 0, TRUE } BOOL;
// Transceiver Configuration

#define MAX_SPI_CLK_FREQ           	(10e6)		// Seems to match Table 5-5 of MRF24J20 datasheet

//TO BE EDITED - EDITED 27/6/2012
//updated to INT0 on 2/7/2012 as Zena analyzer requires INT0 and SPI1
#define RFIF            	INTCONbits.INT0IF		// interrupt input to PIC32 - RB0(EXT. INT) IN PIC18
#define RFIE            	INTCONbits.INT0IE		// ENABLE PIC18 RB0 INTERRUPT
#define RF_INT_PIN      	PORTBbits.RB0  		//RB0	// INT pin on RF module - RB1 IN PIC
#define RF_INT_TRIS     	TRISBbits.TRISB0	//RB0	// SET RB0 AS OUTPUT

// Transceiver Information
//TO BE EDITED - EDITED 27/6/2012
#define RADIO_CS            LATDbits.LATD0			
#define RADIO_CS_TRIS       TRISDbits.TRISD0
#define RADIO_RESETn        LATDbits.LATD2			// Not needed; leave floating - USE THIS PORT JUST IN CASE USED
#define RADIO_RESETn_TRIS   TRISDbits.TRISD2
#define RADIO_WAKE          LATDbits.LATD1			// Not needed; leave floating - USE THIS PORT JUST IN CASE USED
#define RADIO_WAKE_TRIS     TRISDbits.TRISD1	

//DEFINITION RENAMED AS THEY DON'T WANT TO CHANGE OTHER DEFINITION, TO BE REVISED
// EDITED 27/6/2012

//TO BE EDITED - EDITED 27/6/2012
//updated to SPI1 on 2/7/2012
#define SPI_SDI             PORTDbits.RD5			//THIS IS SDI FOR THE MRF NOT FOR THE PIC USE RD4
#define SDI_TRIS            TRISDbits.TRISD5	
#define SPI_SDO             LATDbits.LATD4			//THIS IS SDO FOR THE MRF NOT FOR THE PIC USE RD5
#define SDO_TRIS            TRISDbits.TRISD4	
#define SPI_SCK             LATDbits.LATD6			//SPI1 SCK		
#define SCK_TRIS            TRISDbits.TRISD6		

//NEXT 3 DEFINITIONS USED FOR TIME COUNTING IN RESET
#define RADIO_CLK		SPI_SCK		//
#define RADIO_TX		SPI_SDO		//
#define RADIO_RX		SPI_SDI		//

void BoardInit(void);
void Delay1(void);
void Delay2(void);

#endif	// _HARDWARE_PROFILE_H



