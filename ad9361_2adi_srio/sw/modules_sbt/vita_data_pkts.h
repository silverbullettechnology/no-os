
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



//1
// pkt0
u32 vita_pkt_1[12] = {
0x18f0000c,
0xfacebeef,
0xc1a55001,
0xc1a55002,
0x00000001,
0x00000000,
0x0000000a,
0x01010101,
0x02020202,
0x03030303,
0x04040404,
0x05050505
};

//2
//pkt1
u32 vita_pkt_2[12] = {
0x18f1000c,
0xfacebeef,
0xc1a55001,
0xc1a55002,
0x00000001,
0x00000000,
0x000000a0,
0x01010101,
0x02020202,
0x03030303,
0x04040404,
0x05050505
};

//3
// bad stream id
u32 vita_pkt_3[12] = {
0x18f2000c,
0xbad0bad1,
0xc1a55001,
0xc1a55002,
0x00000001,
0x00000000,
0x0000000a,
0x01010101,
0x02020202,
0x03030303,
0x04040404,
0x05050505
};

//4
// pkt2
// no class id
// ts order error
u32 vita_pkt_4[12] = {
0x10f2000c,
0xfacebeef,
0x00000001,
0x00000000,
0x0000000a,
0x01010101,
0x02020202,
0x03030303,
0x04040404,
0x05050505,
0x06060606,
0x07070707
};

//5
//pkt3
// bad packet type
u32 vita_pkt_5[12] = {
0x08f3000c,
0xbad0bad1,
0xc1a55001,
0xc1a55002,
0x00000001,
0x00000000,
0x0000000a,
0x01010101,
0x02020202,
0x03030303,
0x04040404,
0x05050505
};

//6
// pkt3
u32 vita_pkt_6[12] = {
0x18f3000c,
0xfacebeef,
0xc1a55001,
0xc1a55002,
0x00000001,
0x00000000,
0x00000a00,
0x01010101,
0x02020202,
0x03030303,
0x04040404,
0x05050505
};

//7
//pkt4
u32 vita_pkt_7[12] = {
0x18f4000c,
0xfacebeef,
0xc1a55001,
0xc1a55002,
0x00000001,
0x00000000,
0x0000a000,
0x01010101,
0x02020202,
0x03030303,
0x04040404,
0x05050505
};

//8
// pkt: A
// bad pkt number error
u32 vita_pkt_8[12] = {
0x18fa000c,
0xfacebeef,
0xc1a55001,
0xc1a55002,
0x00000001,
0x00000000,
0x0000a000,
0x01010101,
0x02020202,
0x03030303,
0x04040404,
0x05050505
};

//9
// pkt:B
// bad trailer
u32 vita_pkt_9[12] = {
0x14fb000c,
0xfacebeef,
0x00000001,
0x00000000,
0x0000b000,
0x01010101,
0x02020202,
0x03030303,
0x04040404,
0x05050505,
0x06060606,
0x0bad0567
};

//10
// pkt: C
// packet size error (tlast late)
u32 vita_pkt_10[18] = {
0x10fc000c,
0xfacebeef,
0x00000001,
0x00000000,
0x0000c000,
0x01010101,
0x02020202,
0x03030303,
0x04040404,
0x05050505,
0x07070707,
0x08080808,
0x09090909,
0x0a0a0a0a,
0x0b0b0b0b,
0x0c0c0c0c,
0x0d0d0d0d,
0x0e0e0e0e
};

//11
// pkt:D
// packet size error (tlast early)
u32 vita_pkt_11[8] = {
0x10fd000c,
0xfacebeef,
0x00000001,
0x00000000,
0x0000c000,
0x01010101,
0x02020202,
0x03030303
};


//12
// pkt:D
// packet size error (tlast early)
u32 vita_pkt_12[8] = {
0x10fd000c,
0xfacebeef,
0x00000001,
0x00000000,
0x0000c000,
0x01010101,
0x02020202,
0x03030303
};




#endif 
