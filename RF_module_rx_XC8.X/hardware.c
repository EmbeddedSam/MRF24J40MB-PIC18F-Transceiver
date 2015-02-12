#include "hardware.h"
#include "p18f8722.h"

signed int SPI_Brg;

void BoardInit(void)
{
	ADCON1 = 0x0F;			       // all analog pins set to digital mode, CAN BE CHANGED, DOESNT AFFECT SPI

        //********CHANGE THIS BIT TO SUIT YOUR ROBOT JUST KEEP THE PORTD AND PORTB STUFF INTACT SO RADIO WORKS**************
	LATA  = 0x00;
	LATB  = 0x00;			       // clear all output bits
	LATC  = 0x00;			       // clear all output bits
	LATD  = 0x00;			       // clear all output bits
	LATE  = 0x00;			       // clear all output bits
	LATF  = 0x00;			       // clear all output bits
	LATG  = 0x00;			       // clear all output bits

	TRISA = 0x00;
	TRISB = 0xFF;				// RB0 MUST BE SET AS INPUT FOR INTERRUPT
	TRISC = 0xFF; 				// 
	TRISD = 0x20;				// RD6-SCK2, RD5-SDI2, RD4-SDO2, RD2-RESET, RD1-WAKE, RD0-CS    
	TRISE = 0xFF;				// NOT USED
	TRISF = 0x00;				// SET AS OUTPUT FOR LED DISPLAY ON I/O BOARD
	TRISG = 0x00;				// NOT USED

	RCONbits.IPEN = 0;	

	// setup I/O for RF module

	RADIO_CS_TRIS = 0;					//SET CS PIN AS OUTPUT
	RADIO_CS = 1;						// deselect chip for now
	RADIO_RESETn_TRIS = 0;				        //SET RESET PIN AS OUTPUT
	RADIO_RESETn = 1;					// can be just tied high

	RF_INT_TRIS = 1;					// RF_INT_TRIS definition changed by KP SET RB0 AS INPUT FOR INTERRUPT
	SDI_TRIS = 1;
	SDO_TRIS = 0;
	SCK_TRIS = 0;
	SPI_SDO = 0;
	SPI_SCK = 0;

	RADIO_WAKE_TRIS = 0;
	RADIO_WAKE = 1;						// can be just tied high

	#ifdef  HARDWARE_SPI		
		OpenSPI2(SPI_FOSC_64, MODE_00, SMPMID);	  	//CONFIGURE SPI, TO BE EDITED - EDITED 27/6/2012, updated 2/7/2012
		Nop();
	#endif  // HARDWARE_SPI

	RFIF = 0;						//CLEAR INTERRUPT FLAG OF RB0
	//DO INTERRUPT SETUP HERE
	INTCONbits.PEIE = 0;			//PERIPHERAL DISABLED BY NI 13/06/14 (NOT NEEDED)
	INTCONbits.GIE = 1;

	if( RF_INT_PIN == 0 )			//WHY SET INTERRUPT FLAG WHEN NO INPUT?
		RFIF = 1;

}
