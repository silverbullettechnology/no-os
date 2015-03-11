
#include <stdio.h>

#include "xparameters.h"
#include "xil_exception.h"
#include "xil_cache.h"
#include "xstatus.h"
#include "platform.h"
#include "axilite_test.h"
#include "parameters.h"

#include "console.h"
#include "vita_data_pkts.h"
#include "dac_core.h"
#include "sbt_mod.h"
#include "srio_mod.h"


void reset_dsnk(u32 base_addr)
{
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

void reset_vita_modules()
{
	#ifdef XPAR_VITA49_CLK_BASEADDR
	reset_vita_clk(XPAR_VITA49_CLK_BASEADDR);
	#endif

	#ifdef XPAR_AXIS_VITA49_PACK_0_BASEADDR
	reset_vita_pack(XPAR_AXIS_VITA49_PACK_0_BASEADDR);
	#endif
	
	#ifdef XPAR_AXIS_VITA49_UNPACK_0_BASEADDR
	reset_vita_unpack(XPAR_AXIS_VITA49_UNPACK_0_BASEADDR);
	#endif

	#ifdef XPAR_VITA49_TRIG_DAC_0_BASEADDR
	reset_vita_trig(XPAR_VITA49_TRIG_DAC_0_BASEADDR);
	#endif
	
	#ifdef XPAR_VITA49_TRIG_ADC_0_BASEADDR
	reset_vita_trig(XPAR_VITA49_TRIG_ADC_0_BASEADDR);
	#endif

	#ifdef XPAR_AXIS_VITA49_PACK_1_BASEADDR
	reset_vita_pack(XPAR_AXIS_VITA49_PACK_1_BASEADDR);
	#endif

	#ifdef XPAR_AXIS_VITA49_UNPACK_1_BASEADDR
	reset_vita_unpack(XPAR_AXIS_VITA49_UNPACK_1_BASEADDR);
	#endif

	#ifdef XPAR_VITA49_TRIG_DAC_1_BASEADDR
	reset_vita_trig(XPAR_VITA49_TRIG_DAC_1_BASEADDR);
	#endif

	#ifdef XPAR_VITA49_TRIG_ADC_1_BASEADDR
	reset_vita_trig(XPAR_VITA49_TRIG_ADC_1_BASEADDR);
	#endif

	#ifdef XPAR_VITA49_ASSEM_0_BASEADDR
	reset_vita_assem (XPAR_VITA49_ASSEM_0_BASEADDR);
	#endif

	#ifdef XPAR_VITA49_ASSEM_1_BASEADDR
	reset_vita_assem (XPAR_VITA49_ASSEM_1_BASEADDR);
	#endif

}	


void pass_vita_modules()
{
	#ifdef XPAR_AXIS_VITA49_PACK_0_BASEADDR
	pass_vita_pack(XPAR_AXIS_VITA49_PACK_0_BASEADDR);
	#endif
	
	#ifdef XPAR_AXIS_VITA49_UNPACK_0_BASEADDR
	pass_vita_unpack(XPAR_AXIS_VITA49_UNPACK_0_BASEADDR);
	#endif

	#ifdef XPAR_VITA49_TRIG_DAC_0_BASEADDR
	pass_vita_trig(XPAR_VITA49_TRIG_DAC_0_BASEADDR);
	#endif
	
	#ifdef XPAR_VITA49_TRIG_ADC_0_BASEADDR
	pass_vita_trig(XPAR_VITA49_TRIG_ADC_0_BASEADDR);
	#endif	

	#ifdef XPAR_AXIS_VITA49_PACK_1_BASEADDR
	pass_vita_pack(XPAR_AXIS_VITA49_PACK_1_BASEADDR);
	#endif

	#ifdef XPAR_AXIS_VITA49_UNPACK_1_BASEADDR
	pass_vita_unpack(XPAR_AXIS_VITA49_UNPACK_1_BASEADDR);
	#endif

	#ifdef XPAR_VITA49_TRIG_DAC_1_BASEADDR
	pass_vita_trig(XPAR_VITA49_TRIG_DAC_1_BASEADDR);
	#endif

	#ifdef XPAR_VITA49_TRIG_ADC_1_BASEADDR
	pass_vita_trig(XPAR_VITA49_TRIG_ADC_1_BASEADDR);
	#endif

	#ifdef XPAR_VITA49_ASSEM_0_BASEADDR
	pass_vita_assem (XPAR_VITA49_ASSEM_0_BASEADDR);
	#endif

	#ifdef XPAR_VITA49_ASSEM_1_BASEADDR
	pass_vita_assem (XPAR_VITA49_ASSEM_1_BASEADDR);
	#endif
}	

void reset_vita_trig(u32 base_addr)
{
    AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x02);
    AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x00);
}

void reset_vita_pack(u32 base_addr)
{
    AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x02);
    AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x00);
}
void reset_vita_unpack(u32 base_addr)
{
    AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x02);
    AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x00);
}
void reset_vita_clk(u32 base_addr)
{
    AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x02);
    AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x00);
}
void reset_vita_assem (u32 base_addr)
{
    AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x02);
    AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x00);
}




void enable_vita_trig(u32 base_addr)
{
    AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x01);
}
void enable_vita_pack(u32 base_addr)
{
    AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x01);
}
void enable_vita_unpack(u32 base_addr)
{
    AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x01);
}
void enable_vita_clk(u32 base_addr)
{
    AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x01);
}
void enable_vita_assem (u32 base_addr)
{
    AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x01);
}




void pass_vita_trig(u32 base_addr)
{
    AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x08);
}
void pass_vita_pack(u32 base_addr)
{
    AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x04);
}
void pass_vita_unpack(u32 base_addr)
{
    AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x04);
}

void pass_vita_assem (u32 base_addr)
{
    AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x04);
}




void set_vita_trig (u32 base_addr, u32 tsi, u32 tsf_hi, u32 tsf_lo)
{
    AXILITE_TEST_mWriteSlaveReg2 (base_addr, 0, tsi);
    AXILITE_TEST_mWriteSlaveReg3 (base_addr, 0, tsf_hi);
    AXILITE_TEST_mWriteSlaveReg4 (base_addr, 0, tsf_lo);

    AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x04); // set trigger
    AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x04);
    AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x01); // enable module
}

u32 get_vita_assem_err (int adinum)
{
	u32 base_addr;
	u32 err = -1;

	if (adinum==0) {
		#ifdef XPAR_VITA49_ASSEM_0_BASEADDR
		base_addr = XPAR_VITA49_ASSEM_0_BASEADDR;
		err = AXILITE_TEST_mReadSlaveReg1 (base_addr, 0);
		#endif
	}
	if (adinum==1){
		#ifdef XPAR_VITA49_ASSEM_1_BASEADDR
		base_addr = XPAR_VITA49_ASSEM_1_BASEADDR;
		err = AXILITE_TEST_mReadSlaveReg1 (base_addr, 0);
		#endif
}
	return err;
}

#ifdef XPAR_VITA49_CLK_BASEADDR
u32 get_vita_clk ()
{
	u32 tsi_0, tsf_hi_0, tsf_lo_0;
	u32 tsi_1, tsf_hi_1, tsf_lo_1;
	u32 base_addr = XPAR_VITA49_CLK_BASEADDR;

	tsi_0    = AXILITE_TEST_mReadSlaveReg3 (base_addr, 0);
    tsf_hi_0 = AXILITE_TEST_mReadSlaveReg4 (base_addr, 0);
    tsf_lo_0 = AXILITE_TEST_mReadSlaveReg5 (base_addr, 0);

    tsi_1    = AXILITE_TEST_mReadSlaveReg6 (base_addr, 0);
    tsf_hi_1 = AXILITE_TEST_mReadSlaveReg7 (base_addr, 0);
    tsf_lo_1 = AXILITE_TEST_mReadSlaveReg8 (base_addr, 0);

    xil_printf ("current clk0 time: %x . %x %x\n\r", tsi_0, tsf_hi_0, tsf_lo_0);
//    xil_printf ("current clk1 time: %x . %x %x\n\r", tsi_1, tsf_hi_1, tsf_lo_1);
    return (tsi_0);
}


void set_vita_clk (u32 tsi)
{
	u32 base_addr = XPAR_VITA49_CLK_BASEADDR;

    AXILITE_TEST_mWriteSlaveReg2 (base_addr, 0, tsi);
    AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x04); // set tsi counter
    AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x04);
    AXILITE_TEST_mWriteSlaveReg0 (base_addr, 0, 0x01); // enable module

    get_vita_clk();

    sleep(3);
    get_vita_clk();

}

#endif //XPAR_VITA49_CLK_BASEADDR


#ifdef XPAR_AXIS_VITA49_PACK_1_BASEADDR
void vita_pack_test (u32 adi_num, u32 stream_id, u32 pkt_size, u32 words_to_pack)
{
//	int adi_num = 0;
	int temp;
	int timeout = 10;
	int vita_pack_base_addr;
	int vita_adc_trig_base_addr;
	int vita_clk_base_addr = XPAR_VITA49_CLK_BASEADDR;

	set_adi_adc_snk (ADI_TO_DDR);

	switch (adi_num)
	{
		case 0:
			vita_pack_base_addr = XPAR_AXIS_VITA49_PACK_0_BASEADDR;
			vita_adc_trig_base_addr = XPAR_VITA49_TRIG_ADC_0_BASEADDR;
			break;
		case 1:
			vita_pack_base_addr = XPAR_AXIS_VITA49_PACK_1_BASEADDR;
			vita_adc_trig_base_addr = XPAR_VITA49_TRIG_ADC_1_BASEADDR;
			break;
		default:
			vita_pack_base_addr = XPAR_AXIS_VITA49_PACK_0_BASEADDR;
			vita_adc_trig_base_addr = XPAR_VITA49_TRIG_ADC_0_BASEADDR;
	}

	// set trigger to be 10 seconds into the future
    temp = get_vita_clk(vita_clk_base_addr);
    set_vita_trig (vita_adc_trig_base_addr, temp + timeout, 0, 0);

    AXILITE_TEST_mWriteSlaveReg2 (vita_pack_base_addr, 0, stream_id);
    AXILITE_TEST_mWriteSlaveReg3 (vita_pack_base_addr, 0, pkt_size);
    AXILITE_TEST_mWriteSlaveReg4 (vita_pack_base_addr, 0, words_to_pack);
    AXILITE_TEST_mWriteSlaveReg0 (vita_pack_base_addr, 0, 0x01);

	if (adc_capture(words_to_pack, ADC_DDR_BASEADDR, timeout*2, adi_num) == -1) {
		xil_printf("rx dma timeout error\n\r");
	};

	ShowRxData();
}
#endif //XPAR_AXIS_VITA49_PACK_1_BASEADDR



#ifdef XPAR_AXIS_VITA49_UNPACK_1_BASEADDR
void vita_unpack_test (u32 adi_num, u32 stream_id, int length, struct ad9361_rf_phy *phy)
{
	int  vita_unpack_base_addr;
	int  vita_dac_trig_base_addr;
	int  temp;
	char received_cmd[30] ;

	switch (adi_num)
	{
		case 0:
			vita_unpack_base_addr = XPAR_AXIS_VITA49_UNPACK_0_BASEADDR;
			vita_dac_trig_base_addr = XPAR_VITA49_TRIG_DAC_0_BASEADDR;
			break;
		case 1:
			vita_unpack_base_addr = XPAR_AXIS_VITA49_UNPACK_1_BASEADDR;
			vita_dac_trig_base_addr = XPAR_VITA49_TRIG_DAC_1_BASEADDR;
			break;
		default:
			vita_unpack_base_addr = XPAR_AXIS_VITA49_UNPACK_0_BASEADDR;
			vita_dac_trig_base_addr = XPAR_VITA49_TRIG_DAC_0_BASEADDR;
	}

	xil_printf("number of 32bit words to transmit: %x \n\r", length);

    AXILITE_TEST_mWriteSlaveReg2 (vita_unpack_base_addr, 0, stream_id);
    AXILITE_TEST_mWriteSlaveReg3 (vita_unpack_base_addr, 0, length);
    AXILITE_TEST_mWriteSlaveReg0 (vita_unpack_base_addr, 0, 0x01);

    // transmit VITA packets
    // use chipscope to verify that VITA headers are correctly removed before data transmission
    dac_user_axidma (phy, vita_pkt_a, sizeof(vita_pkt_a));

	temp = console_get_num(received_cmd);
    dac_user_axidma (phy, vita_pkt_b, sizeof(vita_pkt_b));

    temp = console_get_num(received_cmd);
    dac_user_axidma (phy, vita_pkt_c, sizeof(vita_pkt_c));

    temp = console_get_num(received_cmd);
    dac_user_axidma (phy, vita_pkt_d, sizeof(vita_pkt_d));
}
#endif //XPAR_AXIS_VITA49_UNPACK_1_BASEADDR


#ifdef XPAR_AXIS_VITA49_PACK_1_BASEADDR
void vita_endtoend_test (u32 adi_num, u32 srio_addr, u32 stream_id, u32 pkt_size, u32 words_to_pack, struct ad9361_rf_phy *phy)
{
//	int adi_num = 0;
	int temp;
	int timeout = 10;

	int vita_unpack_base_addr;
	int vita_pack_base_addr;
	int vita_adc_trig_base_addr;
	int vita_dac_trig_base_addr;
	int vita_clk_base_addr = XPAR_VITA49_CLK_BASEADDR;

	reset_dmatx(0);
	ad9361_spi_write(phy->spi, REG_BIST_CONFIG, 0x0B);  // 0x09 for PRBS, 0x0B for tone, 0x00 no bist
	ad9361_spi_write(phy->spi, REG_OBSERVE_CONFIG, 0x00);  // 0x01 enable loopback of tx to rx

	xil_printf("set_adi_adc_snk (ADI_TO_SRIO); \n\r"); getchar();
	set_adi_adc_snk (ADI_TO_SRIO);

	xil_printf("set_swrite_bypass (0); \n\r"); getchar();
	set_swrite_bypass (0);   //send swrite packets to adi (not zynq fifo)

	xil_printf("enable_swrite_unpack(); \n\r"); getchar();
	reset_swrite_mod();
	enable_swrite_unpack();
	enable_swrite_pack();



	switch (adi_num)
	{
		case 0:
			vita_pack_base_addr     = XPAR_AXIS_VITA49_PACK_0_BASEADDR;
			vita_unpack_base_addr   = XPAR_AXIS_VITA49_UNPACK_0_BASEADDR;
			vita_adc_trig_base_addr = XPAR_VITA49_TRIG_ADC_0_BASEADDR;
			vita_dac_trig_base_addr = XPAR_VITA49_TRIG_DAC_0_BASEADDR;
			// set up swrite pack addresses
			set_swrite_addr (0, 0, srio_addr);
			set_swrite_addr (1, 0, 0xbeef0120);
			// set up swrite unpack addresses
			set_swrite_addr (0, 1, srio_addr);
			set_swrite_addr (1, 1, 0xbeef0120);
			break;
		case 1:
			vita_pack_base_addr     = XPAR_AXIS_VITA49_PACK_1_BASEADDR;
			vita_unpack_base_addr   = XPAR_AXIS_VITA49_UNPACK_1_BASEADDR;
			vita_adc_trig_base_addr = XPAR_VITA49_TRIG_ADC_1_BASEADDR;
			vita_dac_trig_base_addr = XPAR_VITA49_TRIG_DAC_1_BASEADDR;
			// set up swrite pack addresses
			set_swrite_addr (1, 0, srio_addr);
			set_swrite_addr (0, 0, 0xbeef0120);
			// set up swrite unpack addresses
			set_swrite_addr (1, 1, srio_addr);
			set_swrite_addr (0, 1, 0xbeef0120);
			break;
		default:
			vita_pack_base_addr     = XPAR_AXIS_VITA49_PACK_0_BASEADDR;
			vita_unpack_base_addr   = XPAR_AXIS_VITA49_UNPACK_0_BASEADDR;
			vita_adc_trig_base_addr = XPAR_VITA49_TRIG_ADC_0_BASEADDR;
			vita_dac_trig_base_addr = XPAR_VITA49_TRIG_DAC_0_BASEADDR;
			// set up swrite pack addresses
			set_swrite_addr (0, 0, srio_addr);
			set_swrite_addr (1, 0, 0xbeef0120);
			// set up swrite unpack addresses
			set_swrite_addr (0, 1, srio_addr);
			set_swrite_addr (1, 1, 0xbeef0120);
	}

	xil_printf("setting trigger \n\r");
    temp = get_vita_clk();

//VITA TRIG
    // set adc_trigger to be 10 seconds into the future
    set_vita_trig (vita_adc_trig_base_addr, temp + timeout, 0, 0);
    // set dac trigger to be 20 seconds into the future
    set_vita_trig (vita_dac_trig_base_addr, temp + timeout+2, 0, 0);

//VITA PACK
    AXILITE_TEST_mWriteSlaveReg2 (vita_pack_base_addr, 0, stream_id);
    AXILITE_TEST_mWriteSlaveReg3 (vita_pack_base_addr, 0, pkt_size);
    AXILITE_TEST_mWriteSlaveReg4 (vita_pack_base_addr, 0, words_to_pack);
    AXILITE_TEST_mWriteSlaveReg0 (vita_pack_base_addr, 0, 0x01);  //enable

//VITA UNPACK
    AXILITE_TEST_mWriteSlaveReg0 (vita_unpack_base_addr, 0, 0x00);
    AXILITE_TEST_mWriteSlaveReg2 (vita_unpack_base_addr, 0, stream_id);
    AXILITE_TEST_mWriteSlaveReg3 (vita_unpack_base_addr, 0, words_to_pack);
//    AXILITE_TEST_mWriteSlaveReg0 (vita_unpack_base_addr, 0, 0x04);  //passthrough
    AXILITE_TEST_mWriteSlaveReg0 (vita_unpack_base_addr, 0, 0x01);  //enable

    temp    = AXILITE_TEST_mReadSlaveReg0 (vita_unpack_base_addr, 0);
	xil_printf("VITA UNPACK (%x): %x \n\r", vita_unpack_base_addr, temp);


    // why is this necessary?
	if (adc_capture(words_to_pack*2, ADC_DDR_BASEADDR, timeout*2, adi_num) == -1) {
		xil_printf("rx dma timeout error\n\r");
	};

	xil_printf("wait \n\r");
	while(1) {
		getchar();
		get_vita_clk();
	}
}
#endif //XPAR_AXIS_VITA49_PACK_1_BASEADDR



