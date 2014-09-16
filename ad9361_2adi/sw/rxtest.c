
#include "xil_cache.h"
#include "parameters.h"
#include "ad9361_api.h"
#include "adc_core.h"
#include "dac_core.h"
#include "console.h"
#include "command.h"
#include "rxtest.h"
#include "ad9361.h"

#define WordsToRx 0x4000   //16384

static uint32_t rshift_tx (uint32_t x)
{
	uint32_t output = x >> 4;
	if (x>>15 & 1) output = output | 0x0000f000;
	if (x>>31 & 1) output = output | 0xf0000000;
	return output;
}


void rxtest_main(struct ad9361_rf_phy *phy)
{
	u8 			rx_clk_delay;
	u8 			adi_num;
	uint32_t 	rx_data_clk;
	char		received_cmd[30] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
									0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
									0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	rx_clk_delay = 0x0A;
	adi_num = phy->pcore_id;
while(1){
	for (rx_clk_delay = 0x00; rx_clk_delay < 0x10; rx_clk_delay = rx_clk_delay + 1)
//	for (rx_clk_delay = 0x60; rx_clk_delay < 0xff; rx_clk_delay = rx_clk_delay + 0x10)
	{
		xil_printf("************ RXTEST START *********************\n\r");
		console_get_command(received_cmd);
		memset((void *)ADC_DDR_BASEADDR, 0, WordsToRx);
		Xil_DCacheFlush();
		//	CheckData();

		ad9361_spi_write(phy->spi, REG_BIST_CONFIG, 0X09);            // 0x09 for PRBS, 0x0B for tone
		//	ad9361_spi_write(REG_BIST_CONFIG, 0X0B);
		//ad9361_spi_write(REG_BIST_CONFIG, 0XFB);

		//ad9361_spi_write(REG_RX_CLOCK_DATA_DELAY, 0X0f);
		ad9361_spi_write(phy->spi, REG_RX_CLOCK_DATA_DELAY, rx_clk_delay);

		// If running software on SDRDC cut2 AD2 chip
		//ad9361_spi_write( REG_LVDS_INVERT_CTRL1, 0X10);
		xil_printf("%03x RX_CLOCK_DATA_DELAY  : %02x \n\r", REG_RX_CLOCK_DATA_DELAY, ad9361_spi_read (phy->spi, REG_RX_CLOCK_DATA_DELAY));

		if (adc_capture(WordsToRx, ADC_DDR_BASEADDR, adi_num) == -1) {continue;};

		xil_printf("************ RXTEST DONE *********************\n\r");
		CheckRxData_PRBS();
//		ShowRxData();

//		main_xadcps();
//		xil_printf ("ADI temperature: Raw: %d Corrected: %d Centigrade \n\r",ad9361_spi_read (phy->spi, REG_TEMPERATURE), ad9361_get_temp(phy));

		xil_printf("%03x RX_CLOCK_DATA_DELAY  : %02x \n\r", REG_RX_CLOCK_DATA_DELAY, ad9361_spi_read (phy->spi, REG_RX_CLOCK_DATA_DELAY));

		xil_printf("%03x PARALLEL_PORT_CONFIG1: %02x \n\r", REG_PARALLEL_PORT_CONF_1, ad9361_spi_read (phy->spi, REG_PARALLEL_PORT_CONF_1));
		xil_printf("%03x PARALLEL_PORT_CONFIG2: %02x \n\r", REG_PARALLEL_PORT_CONF_2, ad9361_spi_read (phy->spi, REG_PARALLEL_PORT_CONF_2));
		xil_printf("%03x PARALLEL_PORT_CONFIG3: %02x \n\r", REG_PARALLEL_PORT_CONF_3, ad9361_spi_read (phy->spi, REG_PARALLEL_PORT_CONF_3));

		xil_printf("%03x ENSM_MODE            : %02x \n\r", REG_ENSM_MODE, ad9361_spi_read (phy->spi, REG_ENSM_MODE));
		xil_printf("%03x ENSM_CONFIG_1        : %02x \n\r", REG_ENSM_CONFIG_1, ad9361_spi_read (phy->spi, REG_ENSM_CONFIG_1));
		xil_printf("%03x ENSM_CONFIG_2        : %02x \n\r", REG_ENSM_CONFIG_2, ad9361_spi_read (phy->spi, REG_ENSM_CONFIG_2));
		xil_printf("%03x CALIBRATION_CONTROL  : %02x \n\r", REG_CALIBRATION_CTRL, ad9361_spi_read (phy->spi, REG_CALIBRATION_CTRL));
		xil_printf("%03x STATE                : %02x \n\r", REG_STATE, ad9361_spi_read (phy->spi, REG_STATE));

		xil_printf("%03x DIGITAL_IO_CONTROL   : %02x \n\r", REG_DIGITAL_IO_CTRL, ad9361_spi_read (phy->spi, REG_DIGITAL_IO_CTRL));
		xil_printf("%03x LVDS_BIAS_CONTROL    : %02x \n\r", REG_LVDS_BIAS_CTRL, ad9361_spi_read (phy->spi, REG_LVDS_BIAS_CTRL));
		xil_printf("%03x LVDS_INVERT_CONTROL_1: %02x \n\r", REG_LVDS_INVERT_CTRL1, ad9361_spi_read (phy->spi, REG_LVDS_INVERT_CTRL1));
		xil_printf("%03x LVDS_INVERT_CONTROL_2: %02x \n\r", REG_LVDS_INVERT_CTRL2, ad9361_spi_read (phy->spi, REG_LVDS_INVERT_CTRL2));

		xil_printf("%03x REG_BIST_CONFIG                   : %02x \n\r", REG_BIST_CONFIG, ad9361_spi_read (phy->spi, REG_BIST_CONFIG));
		xil_printf("%03x REG_OBSERVE_CONFIG                : %02x \n\r", REG_OBSERVE_CONFIG, ad9361_spi_read (phy->spi, REG_OBSERVE_CONFIG));
		xil_printf("%03x REG_BIST_AND_DATA_PORT_TEST_CONFIG: %02x \n\r", REG_BIST_AND_DATA_PORT_TEST_CONFIG, ad9361_spi_read (phy->spi, REG_BIST_AND_DATA_PORT_TEST_CONFIG));
		ad9361_get_rx_sampling_freq (phy, &rx_data_clk);
		xil_printf("DATA CLK RATE: %d \n\r", rx_data_clk);
	}
}//while(1)
}



void txrxtest_main(struct ad9361_rf_phy *phy)
{
	u8 			rx_clk_delay;
	u8 			tx_clk_delay;
	u8 			adi_num;
	int 		errors;

	uint32_t 	rx_data_clk;
	char		received_cmd[30] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
									0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
									0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	uint32_t status;

	//tx_clk_delay = 0x20;
	adi_num = phy->pcore_id;

	xil_printf("************ TXRXTEST START *********************\n\r");

//	for (tx_clk_delay = 0; tx_clk_delay < 0x10; tx_clk_delay = tx_clk_delay + 1)
//	for (tx_clk_delay = 0x10; tx_clk_delay < 0xFF; tx_clk_delay = tx_clk_delay + 0x10)
	while(1)
	{
		memset((void *)ADC_DDR_BASEADDR, 0xff, WordsToRx*2);
		Xil_DCacheFlush();
		//	CheckData();

		ad9361_spi_write(phy->spi, REG_BIST_CONFIG, 0X00);  // 0x09 for PRBS, 0x0B for tone, 0x00 no bist
		//ad9361_spi_write(REG_TX_CLOCK_DATA_DELAY, tx_clk_delay);

		ad9361_spi_write(phy->spi, REG_OBSERVE_CONFIG, 0x01);  // 0x01 enable loopback of tx to rx

		// If running software on SDRDC cut2 AD2 chip
		//ad9361_spi_write( REG_LVDS_INVERT_CTRL1, 0X10);

		//while(1)
		dac_init(phy, DATA_SEL_DMA);

		sleep(1);

		//dac_init(DATA_SEL_DDS);
		adc_capture(WordsToRx*4, ADC_DDR_BASEADDR, adi_num);

//		adc_read(0x0400, &status, adi_num);  xil_printf("REG_CHAN_CNTRL        : %02x \n\r", status);
//		adc_read(0x0410, &status, adi_num);  xil_printf("REG_CHAN_CNTRL_1      : %02x \n\r", status);
//		adc_read(0x0414, &status, adi_num);  xil_printf("REG_CHAN_CNTRL_2      : %02x \n\r", status);
//		adc_read(0x0420, &status, adi_num);  xil_printf("REG_CHAN_USR_CNTRL_1  : %02x \n\r", status);
//		adc_read(0x0424, &status, adi_num);  xil_printf("REG_CHAN_USR_CNTRL_2  : %02x \n\r", status);
//
//		ShowTxData();

//		xil_printf("************ TXRXTEST DONE *********************\n\r");
		//ShowRxData();
		errors = CheckRxData_DMA(1);

		xil_printf ("Byte ERROR: %d / %d  \n\r", errors, WordsToRx/4 );

//		main_xadcps();
//		xil_printf ("ADI temperature: Raw: %d Corrected: %d Centigrade \n\r",ad9361_spi_read (phy->spi, REG_TEMPERATURE), ad9361_get_temp(phy));
		xil_printf (" \n\r" );

		xil_printf("%03x REG_TEMP_OFFSET           : %02x \n\r", REG_TEMP_OFFSET, ad9361_spi_read (phy->spi, REG_TEMP_OFFSET));
		xil_printf("%03x REG_START_TEMP_READING    : %02x \n\r", REG_START_TEMP_READING, ad9361_spi_read (phy->spi, REG_START_TEMP_READING));
		xil_printf("%03x REG_TEMP_SENSE2           : %02x \n\r", REG_TEMP_SENSE2, ad9361_spi_read (phy->spi, REG_TEMP_SENSE2));
		xil_printf("%03x REG_TEMPERATURE           : %02x \n\r", REG_TEMPERATURE, ad9361_spi_read (phy->spi, REG_TEMPERATURE));
		xil_printf("%03x REG_TEMP_SENSOR_CONFIG    : %02x \n\r", REG_TEMP_SENSOR_CONFIG, ad9361_spi_read (phy->spi, REG_TEMP_SENSOR_CONFIG));
		xil_printf("%03x REG_AUXADC_CONFIG         : %02x \n\r", REG_AUXADC_CONFIG, ad9361_spi_read (phy->spi, REG_AUXADC_CONFIG));
		xil_printf ("\n\r");

		xil_printf("%03x TX_CLOCK_DATA_DELAY  : %02x \n\r", REG_TX_CLOCK_DATA_DELAY, ad9361_spi_read (phy->spi, REG_TX_CLOCK_DATA_DELAY));
		xil_printf("%03x RX_CLOCK_DATA_DELAY  : %02x \n\r", REG_RX_CLOCK_DATA_DELAY, ad9361_spi_read (phy->spi, REG_RX_CLOCK_DATA_DELAY));

		xil_printf("%03x PARALLEL_PORT_CONFIG1: %02x \n\r", REG_PARALLEL_PORT_CONF_1, ad9361_spi_read (phy->spi, REG_PARALLEL_PORT_CONF_1));
		xil_printf("%03x PARALLEL_PORT_CONFIG2: %02x \n\r", REG_PARALLEL_PORT_CONF_2, ad9361_spi_read (phy->spi, REG_PARALLEL_PORT_CONF_2));
		xil_printf("%03x PARALLEL_PORT_CONFIG3: %02x \n\r", REG_PARALLEL_PORT_CONF_3, ad9361_spi_read (phy->spi, REG_PARALLEL_PORT_CONF_3));

		xil_printf("%03x ENSM_MODE            : %02x \n\r", REG_ENSM_MODE, ad9361_spi_read (phy->spi, REG_ENSM_MODE));
		xil_printf("%03x ENSM_CONFIG_1        : %02x \n\r", REG_ENSM_CONFIG_1, ad9361_spi_read (phy->spi, REG_ENSM_CONFIG_1));
		xil_printf("%03x ENSM_CONFIG_2        : %02x \n\r", REG_ENSM_CONFIG_2, ad9361_spi_read (phy->spi, REG_ENSM_CONFIG_2));
		xil_printf("%03x CALIBRATION_CONTROL  : %02x \n\r", REG_CALIBRATION_CTRL, ad9361_spi_read (phy->spi, REG_CALIBRATION_CTRL));
		xil_printf("%03x STATE                : %02x \n\r", REG_STATE, ad9361_spi_read (phy->spi, REG_STATE));

		xil_printf("%03x DIGITAL_IO_CONTROL   : %02x \n\r", REG_DIGITAL_IO_CTRL, ad9361_spi_read (phy->spi, REG_DIGITAL_IO_CTRL));
		xil_printf("%03x LVDS_BIAS_CONTROL    : %02x \n\r", REG_LVDS_BIAS_CTRL, ad9361_spi_read (phy->spi, REG_LVDS_BIAS_CTRL));
		xil_printf("%03x LVDS_INVERT_CONTROL_1: %02x \n\r", REG_LVDS_INVERT_CTRL1, ad9361_spi_read (phy->spi, REG_LVDS_INVERT_CTRL1));
		xil_printf("%03x LVDS_INVERT_CONTROL_2: %02x \n\r", REG_LVDS_INVERT_CTRL2, ad9361_spi_read (phy->spi, REG_LVDS_INVERT_CTRL2));

		xil_printf("%03x REG_BIST_CONFIG                   : %02x \n\r", REG_BIST_CONFIG, ad9361_spi_read (phy->spi, REG_BIST_CONFIG));
		xil_printf("%03x REG_OBSERVE_CONFIG                : %02x \n\r", REG_OBSERVE_CONFIG, ad9361_spi_read (phy->spi, REG_OBSERVE_CONFIG));
		xil_printf("%03x REG_BIST_AND_DATA_PORT_TEST_CONFIG: %02x \n\r", REG_BIST_AND_DATA_PORT_TEST_CONFIG, ad9361_spi_read (phy->spi, REG_BIST_AND_DATA_PORT_TEST_CONFIG));

		ad9361_get_rx_sampling_freq (phy, &rx_data_clk);
		xil_printf("RX DATA CLK RATE: %d \n\r", rx_data_clk);

		ad9361_get_tx_sampling_freq (phy, &rx_data_clk);
		xil_printf("TX DATA CLK RATE: %d \n\r", rx_data_clk);

		xil_printf("***********  DONE  *******************\n\r");
		console_get_command(received_cmd);

	}

}

u8 reverse (u8 in)
{
	u8 out;

	out = 0;
	out = out | ( (in & 0x80) >> 7);
	out = out | ( (in & 0x40) >> 5);
	out = out | ( (in & 0x20) >> 3);
	out = out | ( (in & 0x10) >> 1);
	out = out | ( (in & 0x08) << 1);
	out = out | ( (in & 0x04) << 3);
	out = out | ( (in & 0x02) << 5);
	out = out | ( (in & 0x01) << 7);

	return out;
}

int CheckRxData_PRBS(void)
{
	u8 *RxPacket;
	u8 ilsb;
	u8 ri_lsb, ri_msb;
	int err_cnt[16];
	int wrd_err_cnt = 0;
	int Index = 0;

	int BYTES_TO_RX = WordsToRx*8;// 0x4000;//0x4000;//16384;

	RxPacket = (u8 *) ADC_DDR_BASEADDR;
	for(Index = 0; Index <16; Index++) {
		err_cnt[Index] = 0;
	}

	// Invalidate the DestBuffer before receiving the data, in case the
	// Data Cache is enabled
	//
	Xil_DCacheInvalidateRange((u32)RxPacket, BYTES_TO_RX);

//	print("* print DDR rx buffer");
	for(Index = 0; Index < BYTES_TO_RX; Index++) {
//	  if (Index%8==0)
//	  {
//  	    xil_printf("\r\nValue[%4d]: %02x ", Index, (unsigned int)RxPacket[Index]);
//  	    ilsb = RxPacket[Index];
//	  }
//	  else
//	  	xil_printf("%02x ",  (unsigned int)RxPacket[Index]);

	  if (Index%8 == 0)
		  ri_lsb = reverse( RxPacket[Index]);
	  if (Index%8 == 1)
		  ri_msb = reverse( RxPacket[Index]);
	  if (Index%8 == 2){
		  if ((ri_lsb ^ RxPacket[Index]) & 0x01) {err_cnt[0] = err_cnt[0] + 1;}// xil_printf("*");}
		  if ((ri_lsb ^ RxPacket[Index]) & 0x02) {err_cnt[1] = err_cnt[1] + 1;}// xil_printf("*");}
		  if ((ri_lsb ^ RxPacket[Index]) & 0x04) {err_cnt[2] = err_cnt[2] + 1;}// xil_printf("");}
		  if ((ri_lsb ^ RxPacket[Index]) & 0x08) {err_cnt[3] = err_cnt[3] + 1;}// xil_printf("*");}
		  if ((ri_lsb ^ RxPacket[Index]) & 0x10) {err_cnt[4] = err_cnt[4] + 1;}// xil_printf("*");}
		  if ((ri_lsb ^ RxPacket[Index]) & 0x20) {err_cnt[5] = err_cnt[5] + 1;}// xil_printf("");}
		  if ((ri_lsb ^ RxPacket[Index]) & 0x40) {err_cnt[6] = err_cnt[6] + 1;}// xil_printf("*");}
		  if ((ri_lsb ^ RxPacket[Index]) & 0x80) {err_cnt[7] = err_cnt[7] + 1;}// xil_printf("*");}
		  if ((ri_lsb ^ RxPacket[Index])       ) wrd_err_cnt = wrd_err_cnt + 1;
	  }
	}
//	xil_printf("\r\n ");
//	for(Index = 0; Index <8; Index++) {
//		xil_printf("ERRORS[%d]: %d\r\n ", Index, err_cnt[Index]);
//	}
//	xil_printf("WRD ERRORS: %d\r\n ", wrd_err_cnt);

	if (wrd_err_cnt != 0)
		return (-1);
	else
		return 0;
}

int ShowRxData(void)
{
	u8 *RxPacket;
	int Index = 0;

	int BYTES_TO_RX = 0x100;//0x4000;//16384;

	RxPacket = (u8 *) ADC_DDR_BASEADDR;

	/* Invalidate the DestBuffer before receiving the data, in case the
	 * Data Cache is enabled
	 */
	Xil_DCacheInvalidateRange((u32)RxPacket, BYTES_TO_RX);

	print("* print DDR rx buffer");
	for(Index = 0; Index < BYTES_TO_RX; Index++) {
	  if (Index%8==0)
	  {
  	    xil_printf("\r\nValue[%4d]: %02x ", Index, (unsigned int)RxPacket[Index]);
	  }
	  else
	  	xil_printf("%02x ",  (unsigned int)RxPacket[Index]);
	}
	xil_printf("\r\n");
	return 0;
}


int ShowTxData(void)
{
	volatile u8 *TxPacket8;
	volatile s16 temp16;
	int Index = 0;

	int BYTES_TO_TX = 0x100;//0x4000;//16384;

	TxPacket8  = (u8 *) DAC_DDR_BASEADDR;

	// shift data down 4 bit for easier comparison to rx data
	for(Index = 0; Index < BYTES_TO_TX; Index++) {
		temp16 = (s16) Xil_In16(DAC_DDR_BASEADDR + 2*Index);
		Xil_Out16(DAC_DDR_BASEADDR + 2*Index, temp16>>4);
	}

	Xil_DCacheFlush();

	print("* print DDR tx buffer");
	for(Index = 0; Index < BYTES_TO_TX; Index++) {
	  if (Index%8==0)
	  {
  	    xil_printf("\r\nValue[%4d]: %02x ", Index, (unsigned int)TxPacket8[Index]);
	  }
	  else
	  	xil_printf("%02x ",  (unsigned int)TxPacket8[Index]);
	}
	xil_printf("\r\n");
	return 0;
}


int CheckRxData_DMA(int p)
{
	u32 rx_temp, tx_temp;
	u32 rx_temp1, rx_temp2;
	u32 tx_temp1, tx_temp2;
	int Index = 0;
    int index_start;
    int error = 0;

    int offset = 16;   // comparision offset to ignore first few bytes of receive packet

	int BYTES_TO_RX = 0x400; //WordsToRx;// 0x400;//0x4000;//16384;

	rx_temp1 = Xil_In32(ADC_DDR_BASEADDR + offset);
	rx_temp2 = Xil_In32(ADC_DDR_BASEADDR + 4 + offset);

	// find
	index_start = -1;
	for(Index = 0; Index < 254; Index = Index+4) {
		tx_temp1 = Xil_In32(DAC_DDR_BASEADDR + Index);
		tx_temp2 = Xil_In32(DAC_DDR_BASEADDR + Index + 4);
		tx_temp1 = rshift_tx( tx_temp1);
		tx_temp2 = rshift_tx( tx_temp2);
//  	    xil_printf("\r\nValue[%4d]: %08x . %08x . %08x", Index, rx_temp1, tx_temp1);
		if ((tx_temp1 == rx_temp1) && (tx_temp2 == rx_temp2)) index_start = Index;
	}

    if (index_start == -1) {
    	xil_printf("* ");
 //   	xil_printf("Error: No match");
    	for(Index = 0; Index < 254; Index = Index+4) {
    		tx_temp1 = Xil_In32(DAC_DDR_BASEADDR + Index);
    		tx_temp2 = Xil_In32(DAC_DDR_BASEADDR + Index + 4);
    		tx_temp1 = rshift_tx( tx_temp1);
    		tx_temp2 = rshift_tx( tx_temp2);
//      	    xil_printf("\r\nValue[%4d]: %08x . %08x ", Index, rx_temp1, tx_temp1);
    	}
    	return (-1);
    }
	if (p) xil_printf("\r\nIndex: %d \n", index_start);


	for (Index =0; Index < BYTES_TO_RX; Index = Index+4) {
		rx_temp = Xil_In32(ADC_DDR_BASEADDR + Index);
		tx_temp = Xil_In32(DAC_DDR_BASEADDR + (index_start + Index - offset) % 256);
		tx_temp = rshift_tx( tx_temp);
  	    if (p) xil_printf("\r\nValue[%4d]: %08x . %08x", Index, rx_temp, tx_temp);

		if (rx_temp != tx_temp){
	  	    if (p) xil_printf("\r\nValue[%4d]: %08x . %08x", Index, rx_temp, tx_temp);
			if (p) xil_printf("*");
			error = error + 1;
		}
	}

	xil_printf("\r\n ");
	xil_printf("ERRORS: %d / %d\r\n ", error, BYTES_TO_RX/4);

	if (error != 0)
		return error;
	else
		return 0;
}



int get_eye_rx(struct ad9361_rf_phy *phy, u8 *delay_vec)
{
	u8		rx_clk_delay;
	u8		adi_num;
	uint32_t 	rx_data_clk;
	int     eye_found = 0;
	char		received_cmd[30] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	for (rx_clk_delay = 0x00; rx_clk_delay < 0x10; rx_clk_delay = rx_clk_delay + 1)
		delay_vec[rx_clk_delay] = 0;

	adi_num = phy->pcore_id;

	ad9361_spi_write(phy->spi, REG_BIST_CONFIG, 0X09);            // 0x09 for PRBS, 0x0B for tone

//	ad9361_get_rx_sampling_freq (phy, &rx_data_clk);
//	xil_printf("DATA CLK RATE: %d \n\r", rx_data_clk);

//	main_xadcps();
//	xil_printf ("ADI temperature: Raw: %d Corrected: %d Centigrade \n\r",ad9361_spi_read (REG_TEMPERATURE), ad9361_get_temp(phy));

	xil_printf ("Valid delays: ");

	for (rx_clk_delay = 0x00; rx_clk_delay < 0x10; rx_clk_delay = rx_clk_delay + 1)
	{
		ad9361_spi_write(phy->spi, REG_RX_CLOCK_DATA_DELAY, rx_clk_delay);

		if (adc_capture(WordsToRx, ADC_DDR_BASEADDR, adi_num) == -1)
		{
			delay_vec[rx_clk_delay] = 1;
			xil_printf ("*  ");
			continue;
		};

		if (CheckRxData_PRBS() == -1)
		{
			delay_vec[rx_clk_delay] = 1;
			xil_printf ("*  ");
		}
		else
		{
			delay_vec[rx_clk_delay] = 2;
			xil_printf ("%02x  ", rx_clk_delay);
			eye_found = 1;
		}
	}

	xil_printf ("\n\r");
//	main_xadcps();
//	xil_printf ("ADI temperature: Raw: %d Corrected: %d Centigrade \n\r",ad9361_spi_read (REG_TEMPERATURE), ad9361_get_temp(phy));
	return (eye_found);
}

void set_eye_rx(struct ad9361_rf_phy *phy, u8 *delay_vec)
{
	u8		rx_clk_delay;
	u8		min_delay = 0;
	u8		max_delay = 0;
	u8		sel_delay = 0;
	u8		adi_num;

	adi_num = phy->pcore_id;

	for (rx_clk_delay = 0x00; rx_clk_delay < 0x10; rx_clk_delay = rx_clk_delay + 1)
	{
		if (delay_vec[rx_clk_delay] == 2) {
			min_delay = rx_clk_delay;
			break;
		}
	}

	for (rx_clk_delay = 0x0F; rx_clk_delay > 0x00; rx_clk_delay = rx_clk_delay - 1)
	{
		if (delay_vec[rx_clk_delay] == 2) {
			max_delay = rx_clk_delay;
			break;
		}
	}

	sel_delay = (min_delay + max_delay) / 2;
	xil_printf ("Select Delay:  %02x \n\r", sel_delay);
	ad9361_spi_write(phy->spi, REG_RX_CLOCK_DATA_DELAY, sel_delay);
}



int get_eye_tx(struct ad9361_rf_phy *phy, u8 *delay_vec)
{
	u8 			tx_clk_delay;
	u8 			adi_num;
	int 		eye_found = 0;

	char		received_cmd[30] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
									0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
									0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	tx_clk_delay = 0x20;
	adi_num = phy->pcore_id;

	for (tx_clk_delay = 0x00; tx_clk_delay < 0x10; tx_clk_delay = tx_clk_delay + 1)
		delay_vec[tx_clk_delay] = 0;

//	main_xadcps();
//	xil_printf ("ADI temperature: Raw: %d Corrected: %d Centigrade \n\r",ad9361_spi_read (REG_TEMPERATURE), ad9361_get_temp(phy));

	ad9361_spi_write(phy->spi, REG_BIST_CONFIG, 0X00);  // 0x09 for PRBS, 0x0B for tone, 0x00 no bist
	ad9361_spi_write(phy->spi, REG_OBSERVE_CONFIG, 0x01);  // 0x01 enable loopback of tx to rx

	xil_printf ("Valid delays: ");

	for (tx_clk_delay = 0x00; tx_clk_delay < 0x10; tx_clk_delay = tx_clk_delay + 1)
	{
		memset((void *)ADC_DDR_BASEADDR, 0xff, WordsToRx*2);
		Xil_DCacheFlush();


		ad9361_spi_write(phy->spi, REG_TX_CLOCK_DATA_DELAY, tx_clk_delay<<4);

		dac_init(phy, DATA_SEL_DMA);

//ShowTxData();
//xil_printf ("TX DELAY %02x \n\r", tx_clk_delay);
sleep(1);  // wait for tx data to reach rx side
//continue;
//while(1){
//adc_capture(WordsToRx*4, ADC_DDR_BASEADDR, adi_num);
//}

		if (adc_capture(WordsToRx*4, ADC_DDR_BASEADDR, adi_num) == -1)
		{
			delay_vec[tx_clk_delay] = 1;
			continue;
		};

//xil_printf ("TX DELAY %02x \n\r", tx_clk_delay);
//ShowRxData();
//continue;

		if (CheckRxData_DMA(1) != 0)
		{
			delay_vec[tx_clk_delay] = 1;
		}
		else
		{
			delay_vec[tx_clk_delay] = 2;
			xil_printf ("%02x  ", tx_clk_delay<<4);
			eye_found = 1;
		}
	}
	xil_printf ("\n\r");
//	main_xadcps();
//	xil_printf ("ADI temperature: Raw: %d Corrected: %d Centigrade \n\r",ad9361_spi_read (REG_TEMPERATURE), ad9361_get_temp(phy));
	return (eye_found);
}

void set_eye_tx(struct ad9361_rf_phy *phy, u8 *delay_vec)
{
	u8		tx_clk_delay;
	u8		min_delay = 0;
	u8		max_delay = 0;
	u8		sel_delay = 0;
	u8		adi_num;

	adi_num = phy->pcore_id;

//	xil_printf ("[");
//	for (tx_clk_delay = 0x00; tx_clk_delay < 0x10; tx_clk_delay = tx_clk_delay + 1)
//	{
//		xil_printf ("%d ", delay_vec[tx_clk_delay]);
//	}
//	xil_printf ("]\r\n");

	for (tx_clk_delay = 0x00; tx_clk_delay < 0x10; tx_clk_delay = tx_clk_delay + 1)
	{
		if (delay_vec[tx_clk_delay] == 2) {
			min_delay = tx_clk_delay;
			break;
		}
	}

	for (tx_clk_delay = 0x0F; tx_clk_delay > 0x00; tx_clk_delay = tx_clk_delay - 1)
	{

		if (delay_vec[tx_clk_delay] == 2) {
			max_delay = tx_clk_delay;
			break;
		}
	}

	sel_delay = (min_delay + max_delay) / 2;
	xil_printf ("Select Delay:  %02x \n\r", sel_delay<<4);
	ad9361_spi_write(phy->spi, REG_TX_CLOCK_DATA_DELAY, sel_delay<<4);
}
