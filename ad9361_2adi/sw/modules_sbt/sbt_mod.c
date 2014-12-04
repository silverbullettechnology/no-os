
#include <stdio.h>

#include "xparameters.h"
#include "xil_exception.h"
#include "xil_cache.h"
#include "xstatus.h"
#include "platform.h"
#include "axilite_test.h"

#include "sbt_mod.h"


void reset_dsnk(u32 base_addr)
{
	int Status;
	int read_loop_index;

    // INITIALIZE SINK MODULE
    AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x02);
    xil_printf("writing reg[%x] <- %x (reset dsnk counters) \n\r",0, 0x02);
    for (read_loop_index = 0 ; read_loop_index < 5; read_loop_index++) {
        xil_printf ("reading reg[%x] -> %x \n\r",read_loop_index, AXILITE_TEST_mReadSlaveReg0 (base_addr, read_loop_index*4));
    }

    // ENABLE SINK MODULE
    xil_printf("* enabling data sink module\n\r");
    AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x01);
    xil_printf("writing reg[%x] <- %x (enable module) \n\r",0, 0x01);
    for (read_loop_index = 0 ; read_loop_index < 5; read_loop_index++) {
//        xil_printf ("reading reg[%x] -> %x \n\r",read_loop_index, AXILITE_TEST_mReadSlaveReg0 (base_addr, read_loop_index*4));
    }
}

void enable_adi2axis(u32 base_addr, u32 cnt)
{
    // Reset ADI2AXIS module
    AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x00);

	// Program bytes to counter (should be multiple of 8)
	AXILITE_TEST_mWriteSlaveReg1 (base_addr, 0, cnt);

	// Enable block
    AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x01);
}


void reset_dsrc (u32 base_addr)
{
	int Status;
	int read_loop_index;

	int MAX_PKT_LEN = 0x100;
	int NUM_PKT = 1;

	// INITIALIZE SRC MODULE
	 for (read_loop_index = 0 ; read_loop_index < 7; read_loop_index++) {
	//          xil_printf ("reading reg[%x] -> %x \n\r",read_loop_index, AXILITE_TEST_mReadSlaveReg0 (base_addr, read_loop_index*4));
	 }
	 AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x02);
	 xil_printf("writing reg[%x] <- %x (reset counters) \n\r",0, 0x02);
	 for (read_loop_index = 0 ; read_loop_index < 5; read_loop_index++) {
		 xil_printf ("reading reg[%x] -> %x \n\r",read_loop_index, AXILITE_TEST_mReadSlaveReg0 (base_addr, read_loop_index*4));
	 }

	 print("* setting bytes to send\n\r");
	 // SET SRC MODULE PARAMETERS
	 AXILITE_TEST_mWriteSlaveReg2 (base_addr, 0, MAX_PKT_LEN);
	 xil_printf("writing reg[%x] <- %x (bytes to send) \n\r",2, MAX_PKT_LEN);
	 for (read_loop_index = 0 ; read_loop_index < 7; read_loop_index++) {
	//          xil_printf ("reading reg[%x] -> %x \n\r",read_loop_index, AXILITE_TEST_mReadSlaveReg0 (base_addr, read_loop_index*4));
	 }

	 print("* setting pkts to send\n\r");
	 // SET SRC MODULE PARAMETERS
	 AXILITE_TEST_mWriteSlaveReg5 (base_addr, 0, NUM_PKT);
	 xil_printf("writing reg[%x] <- %x (pkts to send) \n\r",5, NUM_PKT);
	 for (read_loop_index = 0 ; read_loop_index < 7; read_loop_index++) {
	//          xil_printf ("reading reg[%x] -> %x \n\r",read_loop_index, AXILITE_TEST_mReadSlaveReg0 (base_addr, read_loop_index*4));
	 }

	 print("* setting test pattern\n\r");
	 // SET SRC MODULE PARAMETERS
	 AXILITE_TEST_mWriteSlaveReg4 (base_addr, 0, 0x00);
	 xil_printf("writing reg[%x] <- %x (setting test pattern) \n\r",4, 0x00);
	 for (read_loop_index = 0 ; read_loop_index < 7; read_loop_index++) {
	//          xil_printf ("reading reg[%x] -> %x \n\r",read_loop_index, AXILITE_TEST_mReadSlaveReg0 (base_addr, read_loop_index*4));
	 }

	print("* enable source module\n\r");
	// ENABLE SRC MODULE
	AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x01);
	xil_printf("writing reg[%x] <- %x (enable source module) \n\r",0,0x01);
	for (read_loop_index = 0 ; read_loop_index < 5; read_loop_index++) {
	//	    xil_printf ("reading reg[%x] -> %x \n\r",read_loop_index, AXILITE_TEST_mReadSlaveReg0 (base_addr, read_loop_index*4));
	}

}
