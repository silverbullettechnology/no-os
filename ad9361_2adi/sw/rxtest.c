
#include "xil_cache.h"
#include "parameters.h"
#include "ad9361_api.h"
#include "adc_core.h"
#include "dac_core.h"
#include "console.h"
#include "command.h"
#include "rxtest.h"
#include "ad9361.h"

#define WordsToRx 16384

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

//	for (rx_clk_delay = 0xa; rx_clk_delay < 0x10; rx_clk_delay = rx_clk_delay + 1)
//	for (rx_clk_delay = 0x60; rx_clk_delay < 0xff; rx_clk_delay = rx_clk_delay + 0x10)
	{
		xil_printf("************ RXTEST START *********************\n\r");
		console_get_command(received_cmd);
		memset((void *)ADC_DDR_BASEADDR, 0, WordsToRx);
		Xil_DCacheFlush();
		//	CheckData();

		ad9361_spi_write(REG_BIST_CONFIG, 0X09);            // 0x09 for PRBS, 0x0B for tone
		//	ad9361_spi_write(REG_BIST_CONFIG, 0X0B);
		//ad9361_spi_write(REG_BIST_CONFIG, 0XFB);

		//ad9361_spi_write(REG_RX_CLOCK_DATA_DELAY, 0X0f);
		ad9361_spi_write(REG_RX_CLOCK_DATA_DELAY, rx_clk_delay);

		// If running software on SDRDC cut2 AD2 chip
		//ad9361_spi_write( REG_LVDS_INVERT_CTRL1, 0X10);
		xil_printf("%03x RX_CLOCK_DATA_DELAY  : %02x \n\r", REG_RX_CLOCK_DATA_DELAY, ad9361_spi_read (REG_RX_CLOCK_DATA_DELAY));

		adc_capture(WordsToRx, ADC_DDR_BASEADDR, adi_num);
		xil_printf("************ RXTEST DONE *********************\n\r");
		CheckRxData_PRBS();
//		ShowRxData();


		xil_printf("%03x RX_CLOCK_DATA_DELAY  : %02x \n\r", REG_RX_CLOCK_DATA_DELAY, ad9361_spi_read (REG_RX_CLOCK_DATA_DELAY));

		xil_printf("%03x PARALLEL_PORT_CONFIG1: %02x \n\r", REG_PARALLEL_PORT_CONF_1, ad9361_spi_read (REG_PARALLEL_PORT_CONF_1));
		xil_printf("%03x PARALLEL_PORT_CONFIG2: %02x \n\r", REG_PARALLEL_PORT_CONF_2, ad9361_spi_read (REG_PARALLEL_PORT_CONF_2));
		xil_printf("%03x PARALLEL_PORT_CONFIG3: %02x \n\r", REG_PARALLEL_PORT_CONF_3, ad9361_spi_read (REG_PARALLEL_PORT_CONF_3));

		xil_printf("%03x ENSM_MODE            : %02x \n\r", REG_ENSM_MODE, ad9361_spi_read (REG_ENSM_MODE));
		xil_printf("%03x ENSM_CONFIG_1        : %02x \n\r", REG_ENSM_CONFIG_1, ad9361_spi_read (REG_ENSM_CONFIG_1));
		xil_printf("%03x ENSM_CONFIG_2        : %02x \n\r", REG_ENSM_CONFIG_2, ad9361_spi_read (REG_ENSM_CONFIG_2));
		xil_printf("%03x CALIBRATION_CONTROL  : %02x \n\r", REG_CALIBRATION_CTRL, ad9361_spi_read (REG_CALIBRATION_CTRL));
		xil_printf("%03x STATE                : %02x \n\r", REG_STATE, ad9361_spi_read (REG_STATE));

		xil_printf("%03x DIGITAL_IO_CONTROL   : %02x \n\r", REG_DIGITAL_IO_CTRL, ad9361_spi_read (REG_DIGITAL_IO_CTRL));
		xil_printf("%03x LVDS_BIAS_CONTROL    : %02x \n\r", REG_LVDS_BIAS_CTRL, ad9361_spi_read (REG_LVDS_BIAS_CTRL));
		xil_printf("%03x LVDS_INVERT_CONTROL_1: %02x \n\r", REG_LVDS_INVERT_CTRL1, ad9361_spi_read (REG_LVDS_INVERT_CTRL1));
		xil_printf("%03x LVDS_INVERT_CONTROL_2: %02x \n\r", REG_LVDS_INVERT_CTRL2, ad9361_spi_read (REG_LVDS_INVERT_CTRL2));

		xil_printf("%03x REG_BIST_CONFIG                   : %02x \n\r", REG_BIST_CONFIG, ad9361_spi_read (REG_BIST_CONFIG));
		xil_printf("%03x REG_OBSERVE_CONFIG                : %02x \n\r", REG_OBSERVE_CONFIG, ad9361_spi_read (REG_OBSERVE_CONFIG));
		xil_printf("%03x REG_BIST_AND_DATA_PORT_TEST_CONFIG: %02x \n\r", REG_BIST_AND_DATA_PORT_TEST_CONFIG, ad9361_spi_read (REG_BIST_AND_DATA_PORT_TEST_CONFIG));
		ad9361_get_rx_sampling_freq (phy, &rx_data_clk);
		xil_printf("DATA CLK RATE: %d \n\r", rx_data_clk);
	}
}



void txrxtest_main(struct ad9361_rf_phy *phy)
{
	u8 			rx_clk_delay;
	u8 			tx_clk_delay;
	u8 			adi_num;

	uint32_t 	rx_data_clk;
	char		received_cmd[30] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
									0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
									0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	uint32_t status;


	rx_clk_delay = 0x0a;
	tx_clk_delay = 0x20;
	adi_num = phy->pcore_id;

//	ad9361_set_tx_sampling_freq (phy, 58000000);
	xil_printf("************ TXRXTEST START *********************\n\r");


//	for (tx_clk_delay = 0; tx_clk_delay < 0x10; tx_clk_delay = tx_clk_delay + 1)
//	for (tx_clk_delay = 0x10; tx_clk_delay < 0xFF; tx_clk_delay = tx_clk_delay + 0x10)
	while(1)
	{
		memset((void *)ADC_DDR_BASEADDR, 0xff, WordsToRx*2);
		Xil_DCacheFlush();
		//	CheckData();

		ad9361_spi_write(REG_BIST_CONFIG, 0X00);  // 0x09 for PRBS, 0x0B for tone, 0x00 no bist
		ad9361_spi_write(REG_RX_CLOCK_DATA_DELAY, rx_clk_delay);
		ad9361_spi_write(REG_TX_CLOCK_DATA_DELAY, tx_clk_delay);

		ad9361_spi_write(REG_OBSERVE_CONFIG, 0x01);  // 0x01 enable loopback of tx to rx

		// If running software on SDRDC cut2 AD2 chip
		//ad9361_spi_write( REG_LVDS_INVERT_CTRL1, 0X10);

		//while(1)
		dac_init(DATA_SEL_DMA, phy);
		//dac_init(DATA_SEL_DDS);
		adc_capture(WordsToRx*2, ADC_DDR_BASEADDR, adi_num);

//		adc_read(0x0400, &status, adi_num);  xil_printf("REG_CHAN_CNTRL        : %02x \n\r", status);
//		adc_read(0x0410, &status, adi_num);  xil_printf("REG_CHAN_CNTRL_1      : %02x \n\r", status);
//		adc_read(0x0414, &status, adi_num);  xil_printf("REG_CHAN_CNTRL_2      : %02x \n\r", status);
//		adc_read(0x0420, &status, adi_num);  xil_printf("REG_CHAN_USR_CNTRL_1  : %02x \n\r", status);
//		adc_read(0x0424, &status, adi_num);  xil_printf("REG_CHAN_USR_CNTRL_2  : %02x \n\r", status);
//
//		ShowTxData();

//		xil_printf("************ TXRXTEST DONE *********************\n\r");
		//ShowRxData();
		CheckRxData_DMA();

		main_xadcps();
		xil_printf ("ADI temperature: Raw: %d Corrected: %d Centigrade \n\r",ad9361_spi_read (REG_TEMPERATURE), ad9361_get_temp(phy));
		xil_printf("%03x REG_TEMP_OFFSET           : %02x \n\r", REG_TEMP_OFFSET, ad9361_spi_read (REG_TEMP_OFFSET));
		xil_printf("%03x REG_START_TEMP_READING    : %02x \n\r", REG_START_TEMP_READING, ad9361_spi_read (REG_START_TEMP_READING));
		xil_printf("%03x REG_TEMP_SENSE2           : %02x \n\r", REG_TEMP_SENSE2, ad9361_spi_read (REG_TEMP_SENSE2));
		xil_printf("%03x REG_TEMPERATURE           : %02x \n\r", REG_TEMPERATURE, ad9361_spi_read (REG_TEMPERATURE));
		xil_printf("%03x REG_TEMP_SENSOR_CONFIG    : %02x \n\r", REG_TEMP_SENSOR_CONFIG, ad9361_spi_read (REG_TEMP_SENSOR_CONFIG));
		xil_printf("%03x REG_AUXADC_CONFIG         : %02x \n\r", REG_AUXADC_CONFIG, ad9361_spi_read (REG_AUXADC_CONFIG));
		xil_printf ("\n\r");

		xil_printf("%03x TX_CLOCK_DATA_DELAY  : %02x \n\r", REG_TX_CLOCK_DATA_DELAY, ad9361_spi_read (REG_TX_CLOCK_DATA_DELAY));
		xil_printf("%03x RX_CLOCK_DATA_DELAY  : %02x \n\r", REG_RX_CLOCK_DATA_DELAY, ad9361_spi_read (REG_RX_CLOCK_DATA_DELAY));

		xil_printf("%03x PARALLEL_PORT_CONFIG1: %02x \n\r", REG_PARALLEL_PORT_CONF_1, ad9361_spi_read (REG_PARALLEL_PORT_CONF_1));
		xil_printf("%03x PARALLEL_PORT_CONFIG2: %02x \n\r", REG_PARALLEL_PORT_CONF_2, ad9361_spi_read (REG_PARALLEL_PORT_CONF_2));
		xil_printf("%03x PARALLEL_PORT_CONFIG3: %02x \n\r", REG_PARALLEL_PORT_CONF_3, ad9361_spi_read (REG_PARALLEL_PORT_CONF_3));

		xil_printf("%03x ENSM_MODE            : %02x \n\r", REG_ENSM_MODE, ad9361_spi_read (REG_ENSM_MODE));
		xil_printf("%03x ENSM_CONFIG_1        : %02x \n\r", REG_ENSM_CONFIG_1, ad9361_spi_read (REG_ENSM_CONFIG_1));
		xil_printf("%03x ENSM_CONFIG_2        : %02x \n\r", REG_ENSM_CONFIG_2, ad9361_spi_read (REG_ENSM_CONFIG_2));
		xil_printf("%03x CALIBRATION_CONTROL  : %02x \n\r", REG_CALIBRATION_CTRL, ad9361_spi_read (REG_CALIBRATION_CTRL));
		xil_printf("%03x STATE                : %02x \n\r", REG_STATE, ad9361_spi_read (REG_STATE));

		xil_printf("%03x DIGITAL_IO_CONTROL   : %02x \n\r", REG_DIGITAL_IO_CTRL, ad9361_spi_read (REG_DIGITAL_IO_CTRL));
		xil_printf("%03x LVDS_BIAS_CONTROL    : %02x \n\r", REG_LVDS_BIAS_CTRL, ad9361_spi_read (REG_LVDS_BIAS_CTRL));
		xil_printf("%03x LVDS_INVERT_CONTROL_1: %02x \n\r", REG_LVDS_INVERT_CTRL1, ad9361_spi_read (REG_LVDS_INVERT_CTRL1));
		xil_printf("%03x LVDS_INVERT_CONTROL_2: %02x \n\r", REG_LVDS_INVERT_CTRL2, ad9361_spi_read (REG_LVDS_INVERT_CTRL2));

		xil_printf("%03x REG_BIST_CONFIG                   : %02x \n\r", REG_BIST_CONFIG, ad9361_spi_read (REG_BIST_CONFIG));
		xil_printf("%03x REG_OBSERVE_CONFIG                : %02x \n\r", REG_OBSERVE_CONFIG, ad9361_spi_read (REG_OBSERVE_CONFIG));
		xil_printf("%03x REG_BIST_AND_DATA_PORT_TEST_CONFIG: %02x \n\r", REG_BIST_AND_DATA_PORT_TEST_CONFIG, ad9361_spi_read (REG_BIST_AND_DATA_PORT_TEST_CONFIG));

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

	int BYTES_TO_RX = 0x4000;//0x4000;//16384;

	RxPacket = (u8 *) ADC_DDR_BASEADDR;
	for(Index = 0; Index <16; Index++) {
		err_cnt[Index] = 0;
	}

	// Invalidate the DestBuffer before receiving the data, in case the
	// Data Cache is enabled
	//
	Xil_DCacheInvalidateRange((u32)RxPacket, BYTES_TO_RX);

	print("* print DDR rx buffer");
	for(Index = 0; Index < BYTES_TO_RX; Index++) {
	  if (Index%8==0)
	  {
  	    xil_printf("\r\nValue[%4d]: %02x ", Index, (unsigned int)RxPacket[Index]);
  	    ilsb = RxPacket[Index];
	  }
	  else
	  	xil_printf("%02x ",  (unsigned int)RxPacket[Index]);

	  if (Index%8 == 0)
		  ri_lsb = reverse( RxPacket[Index]);
	  if (Index%8 == 1)
		  ri_msb = reverse( RxPacket[Index]);
	  if (Index%8 == 2){
		  if ((ri_lsb ^ RxPacket[Index]) & 0x01) {err_cnt[0] = err_cnt[0] + 1; xil_printf("*");}
		  if ((ri_lsb ^ RxPacket[Index]) & 0x02) {err_cnt[1] = err_cnt[1] + 1; xil_printf("*");}
		  if ((ri_lsb ^ RxPacket[Index]) & 0x04) {err_cnt[2] = err_cnt[2] + 1; xil_printf("");}
		  if ((ri_lsb ^ RxPacket[Index]) & 0x08) {err_cnt[3] = err_cnt[3] + 1; xil_printf("*");}
		  if ((ri_lsb ^ RxPacket[Index]) & 0x10) {err_cnt[4] = err_cnt[4] + 1; xil_printf("*");}
		  if ((ri_lsb ^ RxPacket[Index]) & 0x20) {err_cnt[5] = err_cnt[5] + 1; xil_printf("");}
		  if ((ri_lsb ^ RxPacket[Index]) & 0x40) {err_cnt[6] = err_cnt[6] + 1; xil_printf("*");}
		  if ((ri_lsb ^ RxPacket[Index]) & 0x80) {err_cnt[7] = err_cnt[7] + 1; xil_printf("*");}
		  if ((ri_lsb ^ RxPacket[Index])       ) wrd_err_cnt = wrd_err_cnt + 1;
	  }
	}
	xil_printf("\r\n ");
	for(Index = 0; Index <8; Index++) {
		xil_printf("ERRORS[%d]: %d\r\n ", Index, err_cnt[Index]);
	}
	xil_printf("WRD ERRORS: %d\r\n ", wrd_err_cnt);

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


int CheckRxData_DMA(void)
{
	u32 rx_temp, tx_temp;
	int Index = 0;
    int index_start;
    int error = 0;

	int BYTES_TO_RX = 0x400;//0x4000;//16384;

	rx_temp = Xil_In32(ADC_DDR_BASEADDR);
	// find
	index_start = -1;
	for(Index = 0; Index < 128; Index = Index+2) {
		tx_temp = Xil_In32(ADC_DDR_BASEADDR + Index);
		if (tx_temp == rx_temp) index_start = Index;
	}
    if (index_start == -1) {
    	xil_printf("Error: No match");
    	return (-1);
    }

	for (Index =0; Index < BYTES_TO_RX; Index++) {
		rx_temp = Xil_In32(ADC_DDR_BASEADDR + Index);
		tx_temp = Xil_In32(ADC_DDR_BASEADDR + index_start + Index);
//  	    xil_printf("\r\nValue[%4d]: %08x . %08x", Index, rx_temp, tx_temp);

		if (rx_temp != tx_temp){
//			xil_printf("*");
			error = error + 1;
		}
	}

	xil_printf("\r\n ");
	xil_printf("ERRORS: %d / %d\r\n ", error, BYTES_TO_RX);

	return 0;
}
