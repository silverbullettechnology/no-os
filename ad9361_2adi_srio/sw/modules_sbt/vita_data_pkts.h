
#ifndef VITA_DATA_PKTS_H
#define VITA_DATA_PKTS_H

#include "xbasic_types.h"

u32 vita_pkt_a[8] = { 
	0x10000008,   	// hdr
  	0xbeef0000,		// stream id
	0xdadada01,
	0xdadada02,
	0xdadada03,
	0xdadada04,
	0xdadada05,
	0xdadada06
};	

u32 vita_pkt_b[8] = {
	0x10d10008,   	// hdr
  	0xbeef0000,		// stream id
	0xffffffff,		// tsi
	0x00000000,		// tsf
	0x12345678,		// tsf
	0xdadada04,
	0xdadada05,
	0xdadada06
};	

u32 vita_pkt_c[12] = {
	0x18d2000c,   	// hdr
  	0xbeef0000,		// stream id
	0xace0ace0,		// class id
	0x911b911c,		// class id
	0xffffffff,		// tsi
	0x00000011,		// tsf
	0x12345678,		// tsf
	0xdadada04,
	0xdadada05,
	0xdadada06,
	0xdadada07,
	0xdadada08	
};	

u32 vita_pkt_d[12] = {
	0x18da000c,   	// hdr
  	0xbeef0000,		// stream id
	0xace0ace0,		// class id
	0x911b911c,		// class id
	0xffffffff,		// tsi
	0x00000011,		// tsf
	0x12345678,		// tsf
	0xfafada04,
	0xfafada05,
	0xfafada06,
	0xfafada07,
	0xfafada08	
};	


#endif 
