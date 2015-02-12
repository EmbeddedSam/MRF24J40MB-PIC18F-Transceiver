// Driver for Microchip MRF24J40 802.15.4 radio hardware
// New parts (c) 2010-2012 nerdfever.com
// Originally based on Microchip MiWi DE v.3.1.3, 5/28/2010 (c) Microchip

#if !defined(MCHP_C18)
#define MCHP_C18
#endif

//#include "debug.h"			// debug status
#include "hardware.h"
#include "MRF24J40.h"
#include "radioAddress.h"	// addr for radio
#include "p18f8722.h"
#include "timers.h"
#include "xc.h"


// globals
UINT8 Tempbyte;

MRF24J40_STATUS volatile RadioStatus; // radio state

//#pragma udata udata1=0x100	// JMA allocate to large ram section
UINT8 volatile RXBuffer1[RX_BUFFER_SIZE] @ 0x100; // JMA rx packet buffer 1
//#pragma udata udata2=0x300
UINT8 volatile RXBuffer2[RX_BUFFER_SIZE] @ 0x300; // JMA rx packet buffer 2
//#pragma udata
PACKET Tx, Rx; // structures describing transmitted and received packets

// this combines memcpy with incrementing the source point.  It copies bytewise, allowing it to copy to/from unaligned addresses

unsigned char* readBytes(unsigned char* dstPtr, unsigned char* srcPtr, unsigned int count) {
    while (count--)
        *dstPtr++ = *srcPtr++;
    return srcPtr;
}

/* The key to understanding SPI is that there is only 1 clock line, and so all transfers
   are always bidirectional - you send one bit for each you receive and vice-versa.  And the CLK
   only runs (in Master mode) when you transmit.
   So to transmit, you store to the TX buffer, wait for it to clock out, flush away the bogus received byte.
   And to receive, you send, wait for the byte to clock in, then read it.
 */

void spiPut(unsigned char v) // write 1 byte to SPI
{
    RADIO_CS = 0; //enable radio if disabled
    Nop();

    WriteSPI2(v);
    while (!SSP2STATbits.BF);
}

unsigned char spiGet(void) // read 1 byte from SPI
{
    if (WriteSPI2(0) != -1) { //to determine whether collision occurs,
        // write to TX buffer (force CLK to run for TX transfer)
        Nop(); //200ns delay
        while (!DataRdySPI2());
        return (ReadSPI2());
    }
    else
    {
        return 'X';
    }
}

unsigned char SpiTake(unsigned char v) {
#ifdef HARDWARE_SPI				//USE DEDICATED HARDWARE

    unsigned char val;

    RADIO_CS = 0;
    Nop();
    WriteSPI2(v); //to determine whether collision occurs,
    while (!SSP2STATbits.BF); // write to TX buffer (force CLK to run for TX transfer)
    //to be edited for optimization
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    while (!DataRdySPI2());
    val = ReadSPI2();
    RADIO_CS = 1;
    Nop();
    return (val);

#endif
}

// reads byte from radio at long "address"

UINT8 highRead(UINT16 address) {
    UINT8 toReturn, tmpRFIE;
    tmpRFIE = 0;
    if (INTCONbits.INT0IE)
        tmpRFIE = 1;

    RFIE = 0;
    RADIO_CS = 0;

    spiPut(((address >> 3)&0x7F) | 0x80);
    toReturn = SpiTake(((address << 5)&0xE0));
    RADIO_CS = 1;
    RFIE = tmpRFIE;

    return toReturn;
}

// writes "value" to radio at long "address"

void highWrite(UINT16 address, UINT8 value) {
    UINT8 tmpRFIE = RFIE;
    RFIE = 0; // disable radio ints during communication
    RADIO_CS = 0; // select radio SPI bus

    spiPut((((UINT8) (address >> 3))&0x7F) | 0x80);
    spiPut((((UINT8) (address << 5))&0xE0) | 0x10);
    spiPut(value);

    RADIO_CS = 1; // de-select radio SPI bus
    RFIE = tmpRFIE; // restore interrupt state

    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
}

// reads byte from radio at short "address"

UINT8 lowRead(UINT8 address) {
    UINT8 toReturn;

    UINT8 tmpRFIE = RFIE;
    RFIE = 0; // disable radio ints during communication
    RADIO_CS = 0; // select radio SPI bus

    toReturn = SpiTake(address);

    RADIO_CS = 1; // de-select radio SPI bus
    RFIE = tmpRFIE; // restore interrupt state
    return toReturn;
}

// writes "value" to radio at short "address"

void lowWrite(UINT8 address, UINT8 value) {
    UINT8 tmpRFIE = RFIE;
    RFIE = 0;
    RADIO_CS = 0;

    spiPut(address);
    spiPut(value);

    RADIO_CS = 1;
    RFIE = tmpRFIE;
}

// writes count consecutive bytes from source into consecutive FIFO slots starting at "register".  Returns next empty register #.

UINT8 toTXfifo(UINT16 reg, UINT8* source, UINT8 count) {
    while (count--)
        highWrite(reg++, *source++);

    return reg;
}


// warm start radio hardware, tunes to Channel.  Takes about 0.37 ms on PIC32 at 20 MHz, 10 MHz SPI hardware clock
// on return, 0=no radio hardare, 1=radio is reset

UINT8 initMRF24J40(void) {
    UINT8 i;

    //	UINT32 radioReset = ReadCoreTimer();	// record time we started the reset procedure

    RadioStatus.ResetCount++;

    RadioStatus.TX_BUSY = 0; // tx is not busy after reset
    RadioStatus.TX_FAIL = 1; // if we had to reset, consider last packet (if any) as failed
    RadioStatus.TX_PENDING_ACK = 0; // not pending an ack after reset
    RadioStatus.SLEEPING = 0; // radio is not sleeping
    RadioStatus.RXReadBuffer = 0;
    //RadioStatus.RXWriteBuffer = 0;

    /* do a soft reset */

    lowWrite(WRITE_SOFTRST, 0x07);
    // reset everything (power, baseband, MAC) (also does wakeup if in sleep)
    do {
        i = lowRead(READ_SOFTRST);
        //		if (CT_TICKS_SINCE(radioReset) > MS_TO_CORE_TICKS(50))		// if no reset in a reasonable time
        //			return 0;												// then there is no radio hardware
    } while ((i & 0x07) != (UINT8) 0x00); // wait for hardware to clear reset bits

    lowWrite(WRITE_RXFLUSH, 0x01); // flush the RX fifo, leave WAKE pin disabled

    RadioSetAddress(RadioStatus.MyShortAddress, RadioStatus.MyLongAddress1, RadioStatus.MyLongAddress2, RadioStatus.MyPANID);

    highWrite(RFCTRL0, 0x03); // RFOPT=0x03
    highWrite(RFCTRL1, 0x02); // VCOOPT=0x02, per datasheet
    highWrite(RFCTRL2, 0x80); // PLL enable
    highWrite(RFCTRL3, TX_POWER); // set transmit power
    highWrite(RFCTRL6, 0x90); // TXFILter on, 20MRECVR set to < 3 mS
    highWrite(RFCTRL7, 0x80); // sleep clock 100 kHz internal
    highWrite(RFCTRL8, 0x10); // RFVCO to 1
    highWrite(SCLKDIV, 0x21); // CLKOUT disabled, sleep clock divisor is 2

    lowWrite(WRITE_BBREG2, 0x80); // CCA energy threshold mode
    lowWrite(WRITE_BBREG6, 0x40); // RSSI on every packet
    lowWrite(WRITE_RSSITHCCA, 0x60); // CCA threshold ~ -69 dBm

#if defined(ENABLE_PA_LNA)
    highWrite(TESTMODE, 0x0F); // setup for PA_LNA mode control
#endif

    lowWrite(WRITE_FFOEN, 0x98); // PACON2, per datasheet init
    lowWrite(WRITE_TXPEMISP, 0x95); // TXSTBL; RFSTBL=9, MSIFS-5

    while ((highRead(RFSTATE)&0xA0) != 0xA0); // wait till RF state machine in RX mode

    lowWrite(WRITE_INTMSK, 0b11110110); // INTCON, enabled=0. RXIE and TXNIE only enabled.
    //WHY WRITTEN AT THIS ADDRESS? cause <<1 & +1 for write bit
    // Make RF communication stable under extreme temperatures
    highWrite(RFCTRL0, 0x03); // this was previously done above
    highWrite(RFCTRL1, 0x02); // VCCOPT - whatever that does...

    RadioSetChannel(RadioStatus.Channel); // tune to current radio channel

#ifdef TURBO_MODE					// propriatary TURBO_MODE runs at 625 kbps (vs. 802.15.4 compliant 250 kbps)
    lowWrite(WRITE_BBREG0, 0x01); // TURBO mode enable
    lowWrite(WRITE_BBREG3, 0x38); // PREVALIDTH to turbo optimized setting
    lowWrite(WRITE_BBREG4, 0x5C); // CSTH carrier sense threshold to turbo optimal
#endif

    lowWrite(WRITE_RFCTL, 0x04); // reset RF state machine

    lowWrite(WRITE_RFCTL, 0x00); // back to normal operation
    //	lowWrite(WRITE_RXMCR,1); 			// bit 0=1 => Don't filter packets on PAN/address. ADDED BY KCP 5/12

    // now delay at least 192 uS per datasheet init
    Nop();

    return 1;
}

// on return, 1=radio is setup, 0=there is no radio




BOOL RadioInit(void) // cold start radio init
{
    BOOL radio;

    memset((void*) &RadioStatus, 0, sizeof (RadioStatus));

    RadioStatus.MyPANID = MY_PAN_ID;
    RadioStatus.MyShortAddress = MY_SHORT_ADDRESS;
    RadioStatus.MyLongAddress1 = MY_LONG_ADDRESS1;
    RadioStatus.MyLongAddress2 = MY_LONG_ADDRESS2;

    RadioStatus.Channel = Channel_ID; // start at channel 22   modified by mchssar6 10/07/12 due to interference on channel 12

    radio = initMRF24J40(); // init radio hardware, tune to RadioStatus.Channel

    RFIE = 1; // INTCONbits.INT0IE = 1 enables radio interrupts
    INTCON2bits.INTEDG0 = 0; // interrupt on falling edge OF INT0 (RB0)
    INTCONbits.GIE = 1; // enable global interrupts WITH NO PRIORITY

    return radio;
}

// set short address and PANID
//void RadioSetAddress(UINT16 shortAddress, UINT64 longAddress, UINT16 panID)

void RadioSetAddress(UINT16 shortAddress, UINT32 longAddress1, UINT32 longAddress2, UINT16 panID) {
    UINT8 i;

    lowWrite(WRITE_SADRL, BYTEPTR(shortAddress)[0]);
    lowWrite(WRITE_SADRH, BYTEPTR(shortAddress)[1]);

    lowWrite(WRITE_PANIDL, BYTEPTR(panID)[0]);
    lowWrite(WRITE_PANIDH, BYTEPTR(panID)[1]);

    for (i = 0; i<sizeof (longAddress1); i++) // program long MAC address
        lowWrite(WRITE_EADR0 + i * 2, BYTEPTR(longAddress1)[i]);

    for (i = 0; i<sizeof (longAddress2); i++) // program long MAC address
        lowWrite(WRITE_EADR4 + i * 2, BYTEPTR(longAddress2)[i]);

    RadioStatus.MyPANID = panID;
    RadioStatus.MyShortAddress = shortAddress;
    RadioStatus.MyLongAddress1 = longAddress1;
    RadioStatus.MyLongAddress2 = longAddress2;
}

// Set radio channel.  Returns with success/fail flag.

BOOL RadioSetChannel(UINT8 channel) {
    if (channel < 11 || channel > 26) //if wrong channels entered
        return FALSE;

#if defined(ENABLE_PA_LNA)	// Permitted band is 2400 to 2483.5 MHz.
    if (channel == 26) // Center Freq. is 2405+5(k-11) MHz, for k=channel 11 to 26
        return FALSE; // max output is 100mW (USA)
#endif						// rolloff is not steep enough to avoid 2483.5 from channel 26 center of 2480 MHz at full MB power

    RadioStatus.Channel = channel;
    highWrite(RFCTRL0, ((channel - 11) << 4) | 0x03); //RFCON0 general equation to convert as per datasheet

    lowWrite(WRITE_RFCTL, 0x04); // reset RF state machine
    lowWrite(WRITE_RFCTL, 0x00); // back to normal

    return 1;
}

//sets radio sleep state

void RadioSetSleep(UINT8 powerState) {
    if (powerState) {
#if defined(ENABLE_PA_LNA)
        highWrite(TESTMODE, 0x08);
        lowWrite(WRITE_GPIODIR, 0x0F);
        lowWrite(WRITE_GPIO, 0x00);
#endif
        lowWrite(WRITE_SOFTRST, 0x04);
        lowWrite(WRITE_WAKECON, 0x80);
        lowWrite(WRITE_SLPACK, 0x80);

        RadioStatus.SLEEPING = 1;
    } else {
        initMRF24J40();
    }
}

// TX side - what goes in the TX FIFO (MRF24J40 datahseet figure 3-11):
//
// Size Offset	Descr
// 1		0		Header length (m)
// 1		1		Frame length (m+n)
// 1		2		LSB of Frame Control (bits/i)
// 1		3		MSB of Frame Control (type)
// 1		4		Sequence number
// 20		24		Addressing fields, worst case (PANIDx2 = 4, LONGx2=16 total =20)
// 103		127		Payload (from TxBuffer)

// sends raw packet per already setup Tx structure.  No error checking here.

void RadioTXRaw(void) {
    UINT8 wReg;

    wReg = toTXfifo(2, BYTEPTR(Tx) + 1, 2 + 1); // frame control (2) + sequence number (1)

    if (Tx.dstAddrMode == SHORT_ADDR_FIELD) // if a short dest addr is present
    {
        wReg = toTXfifo(wReg, BYTEPTR(Tx.dstPANID), 2); // write dstPANID
        wReg = toTXfifo(wReg, BYTEPTR(Tx.dstAddr1), 2); // write short address
    } else if (Tx.dstAddrMode == LONG_ADDR_FIELD) // if a long dest addr is present
    {
        wReg = toTXfifo(wReg, BYTEPTR(Tx.dstPANID), 2); // write dstPANID
        wReg = toTXfifo(wReg, BYTEPTR(Tx.dstAddr1), 4); // long addr
        wReg = toTXfifo(wReg, BYTEPTR(Tx.dstAddr2), 4); // long addr
    }

    // now wReg is at start of source PANID (if present) - 0x05

    if (Tx.srcAddrMode != NO_ADDR_FIELD && // if source present
            Tx.dstAddrMode != NO_ADDR_FIELD && // and dest present
            !Tx.panIDcomp) { // and no PANID compression
        wReg = toTXfifo(wReg, BYTEPTR(Tx.srcPANID), 2); // then write src PANID
    }

    if (Tx.srcAddrMode == SHORT_ADDR_FIELD) // if a short src addr is present
        wReg = toTXfifo(wReg, BYTEPTR(Tx.srcAddr1), 2);
    else if (Tx.srcAddrMode == LONG_ADDR_FIELD) // if a long src addr is present
    {
        wReg = toTXfifo(wReg, BYTEPTR(Tx.srcAddr1), 4);
        wReg = toTXfifo(wReg, BYTEPTR(Tx.srcAddr2), 4);
    }
    // now wReg is pointing to first wReg after header (m)
    // wReg IS POINTING AT 9

    highWrite(0, wReg - 2); // header length, m (-2 for header & frame lengths)
    wReg = toTXfifo(wReg, Tx.payload, Tx.payloadLength);
    highWrite(1, wReg - 2); // frame length (m+n)

    RadioStatus.TX_BUSY = 1; // mark TX as busy TXing
    RadioStatus.TX_PENDING_ACK = Tx.ackRequest;
    lowWrite(WRITE_TXNMTRIG, Tx.ackRequest << 2 | 1); // kick off transmit with above parameters, acknowledgement requested, pg 37

    //	RadioStatus.LastTXTriggerTick = ReadCoreTimer();			// record time (used to check for locked-up radio or PLL loss)
}

// Sends next packet from Tx.  Blocks for up to MRF24J40_TIMEOUT_TICKS if transmitter is
// not ready (RadioStatus.TX_BUSY).  If you don't want to be blocked, don't call
// this until RadioStatus.TX_BUSY == 0.  
//
// This automatically sets frame number and source address for you

void RadioTXPacket(void) {
    unsigned int i = 0;
    if (Tx.srcAddrMode == SHORT_ADDR_FIELD)
        Tx.srcAddr1 = RadioStatus.MyShortAddress;
    else if (Tx.srcAddrMode == LONG_ADDR_FIELD) {
        Tx.srcAddr1 = RadioStatus.MyLongAddress1;
        Tx.srcAddr2 = RadioStatus.MyLongAddress2;
    }

    Tx.frameNumber = RadioStatus.IEEESeqNum++;

    // Don't change here....enhance the next line of code later....05/2012
    while (RadioStatus.TX_BUSY) {

        i = i + 1;
        if (i >= 0xC000)
        {
            Nop();
            Nop();
            Nop();
            Nop();
            Nop();
            initMRF24J40(); // If TX is busy, wait for it to clear (for a resaonable time)
            break;
        }
    }

    //		if ( CT_TICKS_SINCE(RadioStatus.LastTXTriggerTick) > MRF24J40_TIMEOUT_TICKS )	// if not ready in a resonable time
    //			initMRF24J40();										// reset radio hardware (stay on same channel)

    RadioTXRaw();
}


// returns status of last transmitted packet: TX_SUCCESS (1), TX_FAILED (2), or 0 = no result yet because TX busy

UINT8 RadioTXResult(void) {
    if (RadioStatus.TX_BUSY) // if TX not done yet
        return TX_RESULT_BUSY;

    return TX_RESULT_SUCCESS + RadioStatus.TX_FAIL; // 1=success, 2=fail
}

// returns TX_RESULT_SUCCESS or TX_RESULT_FAILED.  Waits up to MRF24J40_TIMEOUT_TICKS.

UINT8 RadioWaitTXResult(void) {
    unsigned int i = 0;
    // Enhance next line of code later....05/2012

    while (RadioStatus.TX_BUSY) {
        i = i + 1;
        if (i >= 0xc000) {
            Nop();
            Nop();
            Nop();
            Nop();
            initMRF24J40(); // If TX is busy, wait for it to clear (for a resaonable time)
            break;
        }
    } // If TX is busy, wait for it to clear (for a resaonable time)

    //		if ( CT_TICKS_SINCE(RadioStatus.LastTXTriggerTick) > MRF24J40_TIMEOUT_TICKS )		// if not ready in a resonable time
    //			initMRF24J40();										// reset radio hardware (stay on same channel)

    return TX_RESULT_SUCCESS + RadioStatus.TX_FAIL; // 1=success, 2=fail
}


//	RX side - what goes in RXBuffer (from MRF24J40 datasheet figure 3-9)
//
//	Size	Offset
//	1		0		Frame length (m+n+2 = header + 102 + FCS)
//	1		1		LSB of Frame Control (bits)
//	1		2		MSB of Frame Control (type)
//	1		3		Sequence number
//	20		23		Addressing fields, worst case (PANIDx2 = 4, MACx2=16 total =20)
//	103		126		Payload
//	2		128		FCS
//	1		129		LQI
//	1		130		RSSI

// Returns count of received packets waiting to be processed & discarded.  Next packet to process is in "Rx".
// Note this gives you ALL received packets (not just ones addressed to you).   Check the addressing yourself if you care.
// Also be aware that sucessive identical packets (same frame number) will be received if the far-end misses your ACK (it
// will re-transmit).  Check for that if you care.

UINT8 RadioRXPacket(void)
{
    unsigned int i = 0;
    UINT8* readPoint;

    if (RadioStatus.RXReadBuffer) {
        readPoint = RXBuffer2;
    }// JMA received packet read point
    else {
        readPoint = RXBuffer1; // JMA received packet read point
    }
    if (RadioStatus.RXPacketCount == 0)
        return 0; // no packets to process

    //if(RadioStatus.TX_BUSY)

    while (RadioStatus.TX_BUSY) { //1
        i = i + 1;
        if (!(RadioStatus.TX_BUSY))
            break;
        else if (i > 0x7000) {
            initMRF24J40();
            break;
        }
    }
    // time out and reset radio if we missed interrupts for a long time
    Nop();
    Nop();
    Nop();
    Nop();
    //	if ( CT_TICKS_SINCE(RadioStatus.LastTXTriggerTick) > MRF24J40_TIMEOUT_TICKS )
    //			initMRF24J40();											// reset radio hardware (stay on same channel)

    readPoint = readBytes(BYTEPTR(Rx), readPoint, 1 + 2 + 1 + 2); // copy frame length (1), frame control (2), frame number (1), PANID (2)

    if (Rx.securityEnabled) // if security enabled, toss it (not supported)
    {
        RadioStatus.RXSecurityEnabled++; // log error
        RadioDiscardPacket();
        //return RadioRXPacket(); // yes I know it's a little recursive, but the RXBuffer is small enough that the stack is unlikely to overflow
    }


    if (Rx.frameType == PACKET_TYPE_ACK) // no PANID present on ACK frames [802.15.4 weakness: No way to know if this ACK is really for you]
        readPoint -= 2;

    // readPoint now just after first PANID field

    if (Rx.dstAddrMode == SHORT_ADDR_FIELD) // if a short dest addr is present
        readPoint = readBytes(BYTEPTR(Rx.dstAddr1), readPoint, 2);
    else if (Rx.dstAddrMode == LONG_ADDR_FIELD) // if long dest addr is present
    { // if a long dest addr is present
        readPoint = readBytes(BYTEPTR(Rx.dstAddr1), readPoint, 4);
        readPoint = readBytes(BYTEPTR(Rx.dstAddr2), readPoint, 4);

    }
    Rx.srcPANID = Rx.dstPANID; // copy first PANID because we don't know if it's src or dst yet
    Rx.srcAddr1 = Rx.dstAddr1;
    Rx.srcAddr2 = Rx.dstAddr2;
    // ditto for address
    // now readPoint is at start of source PANID (if present)

    if (Rx.srcAddrMode != NO_ADDR_FIELD && // if source present
            Rx.dstAddrMode != NO_ADDR_FIELD && // and dest present
            !Rx.panIDcomp) // and no PANID compression
        readPoint = readBytes(BYTEPTR(Rx.srcPANID), readPoint, 2); // then read src PANID

    if (Rx.srcAddrMode == SHORT_ADDR_FIELD) // if a short src addr is present
        readPoint = readBytes(BYTEPTR(Rx.srcAddr1), readPoint, 2);
    else if (Rx.srcAddrMode == LONG_ADDR_FIELD) // if a long src addr is present
    {
        readPoint = readBytes(BYTEPTR(Rx.srcAddr1), readPoint, 4);
        readPoint = readBytes(BYTEPTR(Rx.srcAddr2), readPoint, 4);
    }
    Rx.payload = readPoint;

    if (RadioStatus.RXReadBuffer) {
        Rx.payloadLength = Rx.frameLength + (RXBuffer2 - readPoint) + 1;
        Rx.lqi = RXBuffer2[RXBuffer2[0] + 3];
        Rx.rssi = RXBuffer2[RXBuffer2[0] + 4];
    } else {
        Rx.payloadLength = Rx.frameLength + (RXBuffer1 - readPoint) + 1;
        Rx.lqi = RXBuffer1[RXBuffer1[0] + 3];
        Rx.rssi = RXBuffer1[RXBuffer1[0] + 4];
    }

    return RadioStatus.RXPacketCount;
}

void RadioDiscardPacket(void) {
    if (RadioStatus.RXPacketCount) // just in case we get called more than we ought
    {
        RadioStatus.RXPacketCount--;
        RadioStatus.RXReadBuffer = (RadioStatus.RXReadBuffer + 1) & (1); //Toggle read buffers
    } else
        RadioStatus.RadioExtraDiscard++;
}


// Interrupt handler for the MRF24J40 and P2P stack (PIC32 only, no security)
//void __ISR(_EXTERNAL_4_VECTOR, ipl4) _INT4Interrupt(void)				// from INT pin on MRF24J40 radio
//******************************************************************************
// ISR
//******************************************************************************
//#if defined(MCHP_C18)
//#pragma code lowVector=0x08

//void LowVector(void) {
//    _asm goto ISR _endasm // Commented out by TXV
//}
//#pragma code 						/* return to default code section */
//#endif
//#pragma interrupt ISR

void interrupt ISR(void) {
    MRF24J40_IFREG iflags;

    //Nop();
    //Nop();
    //Nop();
    //Nop();

    iflags.Val = lowRead(READ_ISRSTS); // read ISR to see what caused the interrupt
    Tempbyte = iflags.Val; // TEMPORARY....remove at once!
    RFIF = 0; //clear interrupt flag bit														// clear IF immediately to allow next interrupt

    if (iflags.bits.RXIF) // RX int?
    {
        UINT8 i, bytes;

        lowWrite(WRITE_BBREG1, 0x04); // set RXDECINV to disable hw RX while we're reading the FIFO

        bytes = highRead(0x300) + 2; // get the size of the packet w/FCS, + 2 more bytes for RSSI and LQI

        if (bytes > RX_BUFFER_SIZE) // if too big for the RX buffer
        {
            RadioStatus.RXPacketTooBig++;
            bytes = RX_BUFFER_SIZE; // truncate to fit
        }


        if (RadioStatus.RXReadBuffer)
            RXBuffer2[0] = bytes - 4; // store length of packet (not counting length byte, FCS, LQI and RSSI)
        else
            RXBuffer1[0] = bytes - 4; // store length of packet (not counting length byte, FCS, LQI and RSSI)



        for (i = 1; i <= bytes; i++) { // copy data from the FIFO into the RX buffer, plus RSSI and LQI
            if (RadioStatus.RXReadBuffer) //changed from write
                RXBuffer2[i] = highRead(0x300 + i);
            else
                RXBuffer1[i] = highRead(0x300 + i);
        }

        RadioStatus.RXPacketCount++;
        //RadioStatus.RXWriteBuffer = (RadioStatus.RXWriteBuffer+1) & (1);	// JMA mod PACKET_BUFFERS

        if ((RadioStatus.RXPacketCount > 2) /*|| (RadioStatus.RXWriteBuffer == RadioStatus.RXReadBuffer)*/)
            RadioStatus.RXBufferOverruns++;

        lowWrite(WRITE_RXFLUSH, 0x01); // flush RX hw FIFO manually (workaround for silicon errata #1)
        lowWrite(WRITE_BBREG1, 0x00); // reset RXDECINV to enable radio to receive next packet
    }

    if (iflags.bits.TXIF) // TX int?  If so, this means TX is no longer busy, and the result (if any) of the ACK request is in
    {
        RadioStatus.TX_BUSY = 0; // clear busy flag (TX is complete now)

        if (RadioStatus.TX_PENDING_ACK) // if we were waiting for an ACK
        {
            UINT8 TXSTAT = lowRead(READ_TXSR); // read TXSTAT, transmit status register
            RadioStatus.TX_FAIL = TXSTAT & 1; // read TXNSTAT (TX failure status)
            RadioStatus.TX_RETRIES = TXSTAT >> 6; // read TXNRETRY, number of retries of last sent packet (0..3)
            RadioStatus.TX_CCAFAIL = TXSTAT & 0b00100000; // read CCAFAIL

            RadioStatus.TX_PENDING_ACK = 0; // TX finished, clear that I am pending an ACK, already got it (if I was gonna get it)
        }
    }

}






