// (c) 2010-2012 nerdfever.com

// Address for IEEE 802.15.4 radio

#if !defined(_RADIOADDRESS_H_)
#define _RADIOADDRESS_H_

//EITHER ONE LONG ADDRESS IS NOT NEEDED, TO BE REVISED
#define MY_LONG_ADDRESS 	(0xaa55aa55aa55aa55)		// device MAC address (8 bytes, 64 bit)
#define MY_LONG_ADDRESS1	(0xaa55aa55)				//
#define MY_LONG_ADDRESS2   	(0xaa55aa55)					//
#define MY_SHORT_ADDRESS	(0xaa11)					// short (2 byte) 802.15.4 address
#define MY_PAN_ID			(0x0015)					// PAN identifier
#define Channel_ID			22							//

#endif // _RADIOADDRESS_H_