/*******************************************************************************
*              ______          _ _         _     _       _                     *
*              | ___ \        | (_)       | |   (_)     | |                    *
*              | |_/ /__ _  __| |_  ___   | |    _ _ __ | | __                 *
*              |    // _` |/ _` | |/ _ \  | |   | | '_ \| |/ /                 *
*              | |\ \ (_| | (_| | | (_) | | |___| | | | |   <                  *
*              \_| \_\__,_|\__,_|_|\___/  \_____/_|_| |_|_|\_\                 *
*                                                                              *
*******************************************************************************/

// PIC18F8722 - MRF24J40MB Radio Link code
//  2015 - University of Manchester
//  Hardware used - 2x PIC18F8722 School board and IO board (one receive, one trasmit)
//  Software used - MPLABX IDE v2.30
//  Compiler used - XC8 v1.33

//  To use this code you will need to connect up the MRF board with the following
//  connections
//
//  +5V  - 5V ON PIC
//  GND  - GND ON PIC
//  RST  - RD2
//  WAKE - RD1
//  SDI  - RD4 (SDO2)
//  SCK  - RD6 (SCK2)
//  CS   - RD0
//  INT  - RB0
//  SDO  - RD5 (SDI2)
//
//  START WITH THE IO BOARD SWITCHES IN THE OFF POSITION (0V ON PIC)
//
//  This code can act as a receiver or a transmitter if you look at main below
//  it should be clear how it swaps between the two modes, you can swap to transmit
//  or receive mode at any point just by swapping the switches around
//
//  Transmit Mode
//  On boot up set RC3 (Right most switch on IO board to 1)
//  a "t" should appear on the IO Board 7 segment display
//  The LED's (LATF) Should display 0x0F
//  If no receiver is found on channel 22 then the LED's will flash red
//  If a receiver is found then it will transmit a single byte a time counting
//  up from 0 to 255, the reciver and transmitter should display both these numbers
//  on the LED's and they should be in sync.
//
//  Receive Mode
//  On boot up set RC3 (Right most switch on IO board to 0)
//  an "r" should appear on the IO Board 7 segment display
//  The LED's (LATF) Should display 0x9E
//  If no transmitter is found nothing should happen, once a transmitter is found
//  the recieiver will display anything it receives automatically on the LED's (LATF)
//
//  Sleep mode
//  If you ever set the second rightmost switch (RC3) to one then the radio will
//  get trapped in a while loop and will cease transmitting or receiving.
//  Two dashes will be displayed on the 7 segment LED's while it is in sleep mode
//******************************************************************************

#include "p18f8722.h"
#include "hardware.h"
#include "MRF24J40.h"
#include "radioAddress.h"
#include "stdio.h"
#include "config_settings.h"
#include "xc.h"

//******************************************************************************
// Global Variables
//******************************************************************************

UINT8 txPayload[TX_PAYLOAD_SIZE]; // TX payload buffer
UINT8 payLoad = 0;
int i;

//******************************************************************************
// Function Prototypes
//******************************************************************************
void RadioInitP2P(void);


//******************************************************************************
// Main Program
//******************************************************************************
void main(void) {

    static UINT8 lastFrameNumber;
    static UINT8 ReadData;

    TRISCbits.RC2 = 1; //Set transmit/receive switch as input
    TRISCbits.RC3 = 1; //Set radio sleep switch as input
    TRISH = 0xF0;      //Enable 7 segment display transistors

    BoardInit(); // setup hardware
    RadioInit(); // cold start MRF24J40 radio
    RadioInitP2P(); // setup for simple peer-to-peer communication

    // Check code is running by flashing LED's
    for (i = 0; i < 3; i++) {

        LATF = 0xFF;
        Delay10KTCYx(30); 
        LATF = 0x00;
        Delay10KTCYx(30);
    }
    //Main loop
    while (1)
    {
        if (PORTCbits.RC3) {
            RadioSetSleep(1);
            LATH = 0b00000000; //7 segment disabled
            while (PORTCbits.RC3) {
                LATF = 0x7F; //deliberate kill loop, gets stuck in this loop
            }
            LATH = 0b00000011;
            RadioSetSleep(0);
        } 
        else if (!PORTCbits.RC2)
        {
            if (PORTCbits.RC3)
            {
                break; //sleep
            }

            for (i = 0; i < 3; i++)
            {
                LATH = 0b00000010; //7 segment disabled
                LATF = 0x9E;
                Delay10KTCYx(250);
                LATH = 0b00000011;
                Delay10KTCYx(50);
            }

            //If rightmost switch is set to 0, go into receive mode
            while (!PORTCbits.RC2)
            {
               // main program loop
               // process any received packets
                if (PORTCbits.RC3) {
                    break;
                }
                ReadData = RadioRXPacket();

                if (Rx.frameNumber != lastFrameNumber)
                { 
                    lastFrameNumber = Rx.frameNumber;
                    LATF = (UINT8) * Rx.payload;
                    RadioDiscardPacket();
                 }
              }
        }//If rightmost switch is set to 1, go into transmit mode
        else if (PORTCbits.RC2) {

            if (PORTCbits.RC3) {
                break;
            }

            for (i = 0; i < 3; i++) {
                LATH = 0b00000010;
                LATF = 0x0F;
                Delay10KTCYx(250);
                LATH = 0b00000011;
                Delay10KTCYx(50);
            }

            while (PORTCbits.RC2) {

                if (PORTCbits.RC3) {
                    break;
                }
                //tx goes here
                Tx.payload = (unsigned char*) &payLoad; // pointer to UINT8
                Tx.payloadLength = sizeof (unsigned char); // 1 byte to send
                RadioTXPacket();
                LATF = payLoad; //display payload on LED's
                payLoad++;      
                Delay10KTCYx(10);
                Delay10KTCYx(250);

                if (payLoad > 0x0F) {
                    payLoad = 0;
                }
                while (RadioStatus.TX_FAIL) {
                    if (PORTCbits.RC3) {
                        break;
                    }

                    RadioTXPacket();

                    for (i = 0; i < 4; i++) {
                        LATF = 0x00;
                        Delay10KTCYx(100);
                        LATF = 0xFF;
                        Delay10KTCYx(100);
                    }
                }
            }
        }
    }
}

void RadioInitP2P(void) {
    // inits Tx structure for simple point-to-point connection between a single pair of devices who both use the same address
    // after calling this, you can send packets by just filling out:
    // txPayload[] with payload and
    // Tx.payloadLength,
    // then calling RadioTXPacket()
    Tx.frameType = PACKET_TYPE_DATA;
    Tx.securityEnabled = 0;
    Tx.framePending = 0;
    Tx.ackRequest = 1;
    Tx.panIDcomp = 1;
    Tx.dstAddrMode = SHORT_ADDR_FIELD;
    Tx.frameVersion = 0;
    Tx.srcAddrMode = NO_ADDR_FIELD;
    Tx.dstPANID = RadioStatus.MyPANID;
    Tx.dstAddr1 = RadioStatus.MyShortAddress;
    Tx.payload = txPayload;
}