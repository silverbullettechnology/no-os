/***************************************************************************//**
 *   @file   main.c
 *   @brief  Implementation of Main Function.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2013(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "xil_cache.h"
#include "parameters.h"
#include "ad9361_api.h"
#include "adc_core.h"
#include "dac_core.h"
#include "console.h"
#include "command.h"
#include "rxtest.h"
#include "xspips.h"
#include "ad9361.h"


/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/
extern command	  	cmd_list[];
extern char			cmd_no;
extern cmd_function	cmd_functions[11];
unsigned char		cmd				 =  0;
double				param[5]		 = {0, 0, 0, 0, 0};
char				param_no		 =  0;
int					cmd_type		 = -1;
char				invalid_cmd		 =  0;
char				received_cmd[30] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
										0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
										0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

AD9361_InitParam default_init_param = {
	/* Reference Clock */
	38400000UL,	//reference_clk_rate
	/* Base Configuration */
	1,		//two_rx_two_tx_mode_enable *** adi,2rx-2tx-mode-enable
	1,		//frequency_division_duplex_mode_enable *** adi,frequency-division-duplex-mode-enable
	0,		//tdd_use_dual_synth_mode_enable *** adi,tdd-use-dual-synth-mode-enable
	0,		//tdd_skip_vco_cal_enable *** adi,tdd-skip-vco-cal-enable
	0,		//external_rx_lo_enable *** adi,external-rx-lo-enable
	0,		//external_tx_lo_enable *** adi,external-tx-lo-enable
	5,		//dc_offset_tracking_update_event_mask *** adi,dc-offset-tracking-update-event-mask
	6,		//dc_offset_attenuation_high_range *** adi,dc-offset-tracking-update-event-mask
	5,		//dc_offset_attenuation_low_range *** adi,dc-offset-tracking-update-event-mask
	0x28,	//dc_offset_count_high_range *** adi,dc-offset-tracking-update-event-mask
	0x32,	//dc_offset_count_low_range *** adi,dc-offset-tracking-update-event-mask
	0,		//tdd_use_fdd_vco_tables_enable *** adi,tdd-use-fdd-vco-tables-enable
	0,		//split_gain_table_mode_enable *** adi,split-gain-table-mode-enable
	/* ENSM Control */
	0,		//ensm_enable_pin_pulse_mode_enable *** adi,ensm-enable-pin-pulse-mode-enable
	0,		//ensm_enable_txnrx_control_enable *** adi,ensm-enable-txnrx-control-enable
	/* LO Control */
	2400000000UL,	//rx_synthesizer_frequency_hz *** adi,rx-synthesizer-frequency-hz
	2400000000UL,	//tx_synthesizer_frequency_hz *** adi,tx-synthesizer-frequency-hz
	/* Rate & BW Control */
	{983040000, 245760000, 122880000, 61440000, 30720000, 30720000},//uint32_t	rx_path_clock_frequencies[6] *** adi,rx-path-clock-frequencies
	{983040000, 122880000, 122880000, 61440000, 30720000, 30720000},//uint32_t	tx_path_clock_frequencies[6] *** adi,tx-path-clock-frequencies
	18000000,//rf_rx_bandwidth_hz *** adi,rf-rx-bandwidth-hz
	18000000,//rf_tx_bandwidth_hz *** adi,rf-tx-bandwidth-hz
	/* RF Port Control */
	0,		//rx_rf_port_input_select *** adi,rx-rf-port-input-select
	0,		//tx_rf_port_input_select *** adi,tx-rf-port-input-select
	/* TX Attenuation Control */
	10000,	//tx_attenuation_mdB *** adi,tx-attenuation-mdB
	0,		//update_tx_gain_in_alert_enable *** adi,update-tx-gain-in-alert-enable
	/* Reference Clock Control */
	0,		//xo_disable_use_ext_refclk_enable *** adi,xo-disable-use-ext-refclk-enable
	{8, 5920},	//dcxo_coarse_and_fine_tune[2] *** adi,dcxo-coarse-and-fine-tune
	0,		//clk_output_mode_select *** adi,clk-output-mode-select
	/* Gain Control */
	2,		//gc_rx1_mode *** adi,gc-rx1-mode
	2,		//gc_rx2_mode *** adi,gc-rx2-mode
	58,		//gc_adc_large_overload_thresh *** adi,gc-adc-large-overload-thresh
	4,		//gc_adc_ovr_sample_size *** adi,gc-adc-ovr-sample-size
	47,		//gc_adc_small_overload_thresh *** adi,gc-adc-small-overload-thresh
	8192,	//gc_dec_pow_measurement_duration *** adi,gc-dec-pow-measurement-duration
	0,		//gc_dig_gain_enable *** adi,gc-dig-gain-enable
	800,	//gc_lmt_overload_high_thresh *** adi,gc-lmt-overload-high-thresh
	704,	//gc_lmt_overload_low_thresh *** adi,gc-lmt-overload-low-thresh
	24,		//gc_low_power_thresh *** adi,gc-low-power-thresh
	15,		//gc_max_dig_gain *** adi,gc-max-dig-gain
	/* Gain MGC Control */
	2,		//mgc_dec_gain_step *** adi,mgc-dec-gain-step
	2,		//mgc_inc_gain_step *** adi,mgc-inc-gain-step
	0,		//mgc_rx1_ctrl_inp_enable *** adi,mgc-rx1-ctrl-inp-enable
	0,		//mgc_rx2_ctrl_inp_enable *** adi,mgc-rx2-ctrl-inp-enable
	0,		//mgc_split_table_ctrl_inp_gain_mode *** adi,mgc-split-table-ctrl-inp-gain-mode
	/* Gain AGC Control */
	10,		//agc_adc_large_overload_exceed_counter *** adi,agc-adc-large-overload-exceed-counter
	2,		//agc_adc_large_overload_inc_steps *** adi,agc-adc-large-overload-inc-steps
	0,		//agc_adc_lmt_small_overload_prevent_gain_inc_enable *** adi,agc-adc-lmt-small-overload-prevent-gain-inc-enable
	10,		//agc_adc_small_overload_exceed_counter *** adi,agc-adc-small-overload-exceed-counter
	4,		//agc_dig_gain_step_size *** adi,agc-dig-gain-step-size
	3,		//agc_dig_saturation_exceed_counter *** adi,agc-dig-saturation-exceed-counter
	1000,	// agc_gain_update_interval_us *** adi,agc-gain-update-interval-us
	0,		//agc_immed_gain_change_if_large_adc_overload_enable *** adi,agc-immed-gain-change-if-large-adc-overload-enable
	0,		//agc_immed_gain_change_if_large_lmt_overload_enable *** adi,agc-immed-gain-change-if-large-lmt-overload-enable
	10,		//agc_inner_thresh_high *** adi,agc-inner-thresh-high
	1,		//agc_inner_thresh_high_dec_steps *** adi,agc-inner-thresh-high-dec-steps
	12,		//agc_inner_thresh_low *** adi,agc-inner-thresh-low
	1,		//agc_inner_thresh_low_inc_steps *** adi,agc-inner-thresh-low-inc-steps
	10,		//agc_lmt_overload_large_exceed_counter *** adi,agc-lmt-overload-large-exceed-counter
	2,		//agc_lmt_overload_large_inc_steps *** adi,agc-lmt-overload-large-inc-steps
	10,		//agc_lmt_overload_small_exceed_counter *** adi,agc-lmt-overload-small-exceed-counter
	5,		//agc_outer_thresh_high *** adi,agc-outer-thresh-high
	2,		//agc_outer_thresh_high_dec_steps *** adi,agc-outer-thresh-high-dec-steps
	18,		//agc_outer_thresh_low *** adi,agc-outer-thresh-low
	2,		//agc_outer_thresh_low_inc_steps *** adi,agc-outer-thresh-low-inc-steps
	1,		//agc_attack_delay_extra_margin_us; *** adi,agc-attack-delay-extra-margin-us
	0,		//agc_sync_for_gain_counter_enable *** adi,agc-sync-for-gain-counter-enable
	/* RSSI Control */
	1,		//rssi_delay *** adi,rssi-delay
	1000,	//rssi_duration *** adi,rssi-duration
	3,		//rssi_restart_mode *** adi,rssi-restart-mode
	0,		//rssi_unit_is_rx_samples_enable *** adi,rssi-unit-is-rx-samples-enable
	1,		//rssi_wait *** adi,rssi-wait
	/* Aux ADC Control */
	256,	//aux_adc_decimation *** adi,aux-adc-decimation
	40000000UL,	//aux_adc_rate *** adi,aux-adc-rate
	/* Temperature Sensor Control */
	256,	//temp_sense_decimation *** adi,temp-sense-decimation
	1000,	//temp_sense_measurement_interval_ms *** adi,temp-sense-measurement-interval-ms
	0xBF,     //0xCE	//temp_sense_offset_signed *** adi,temp-sense-offset-signed
	1,		//temp_sense_periodic_measurement_enable *** adi,temp-sense-periodic-measurement-enable
	/* Control Out Setup */
	0xFF,	//ctrl_outs_enable_mask *** adi,ctrl-outs-enable-mask
	0,		//ctrl_outs_index *** adi,ctrl-outs-index
	/* External LNA Control */
	0,		//elna_settling_delay_ns *** adi,elna-settling-delay-ns
	0,		//elna_gain_mdB *** adi,elna-gain-mdB
	0,		//elna_bypass_loss_mdB *** adi,elna-bypass-loss-mdB
	0,		//elna_rx1_gpo0_control_enable *** adi,elna-rx1-gpo0-control-enable
	0,		//elna_rx2_gpo1_control_enable *** adi,elna-rx2-gpo1-control-enable
	/* Digital Interface Control */
	1,		//pp_tx_swap_enable *** adi,pp-tx-swap-enable
	1,		//pp_rx_swap_enable *** adi,pp-rx-swap-enable
	0,		//tx_channel_swap_enable *** adi,tx-channel-swap-enable
	0,		//rx_channel_swap_enable *** adi,rx-channel-swap-enable
	1,		//rx_frame_pulse_mode_enable *** adi,rx-frame-pulse-mode-enable
	1,		//two_t_two_r_timing_enable *** adi,2t2r-timing-enable
	0,		//invert_data_bus_enable *** adi,invert-data-bus-enable
	0,		//invert_data_clk_enable *** adi,invert-data-clk-enable
	0,		//fdd_alt_word_order_enable *** adi,fdd-alt-word-order-enable
	0,		//invert_rx_frame_enable *** adi,invert-rx-frame-enable
	0,		//fdd_rx_rate_2tx_enable *** adi,fdd-rx-rate-2tx-enable
	0,		//swap_ports_enable *** adi,swap-ports-enable
	0,		//single_data_rate_enable *** adi,single-data-rate-enable
	1,		//lvds_mode_enable *** adi,lvds-mode-enable
	0,		//half_duplex_mode_enable *** adi,half-duplex-mode-enable
	0,		//single_port_mode_enable *** adi,single-port-mode-enable
	0,		//full_port_enable *** adi,full-port-enable
	0,		//full_duplex_swap_bits_enable *** adi,full-duplex-swap-bits-enable
	0,		//delay_rx_data *** adi,delay-rx-data
	0,		//rx_data_clock_delay *** adi,rx-data-clock-delay
	4,		//rx_data_delay *** adi,rx-data-delay
	7,		//tx_fb_clock_delay *** adi,tx-fb-clock-delay
	0,		//tx_data_delay *** adi,tx-data-delay
	150,	//lvds_bias_mV *** adi,lvds-bias-mV
	1,		//lvds_rx_onchip_termination_enable *** adi,lvds-rx-onchip-termination-enable
};

AD9361_RXFIRConfig rx_fir_config = {
	3, // rx;
	0, // rx_gain;
	1, // rx_dec;
	{-4, -6, -37, 35, 186, 86, -284, 315,
	 107, 219, -4, 271, 558, -307, -1182, -356,
	 658, 157, 207, 1648, 790, -2525, -2553, 748,
	 865, -476, 3737, 6560, -3583, -14731, -5278, 14819,
	 14819, -5278, -14731, -3583, 6560, 3737, -476, 865,
	 748, -2553, -2525, 790, 1648, 207, 157, 658,
	 -356, -1182, -307, 558, 271, -4, 219, 107,
	 -315, -284, 86, 186, 35, -37, -6, -4} // rx_coef[64];
};

AD9361_TXFIRConfig tx_fir_config = {
	3, // tx;
	-6, // tx_gain;
	1, // tx_inc;
	{-4, -6, -37, 35, 186, 86, -284, 315,
	 107, 219, -4, 271, 558, -307, -1182, -356,
	 658, 157, 207, 1648, 790, -2525, -2553, 748,
	 865, -476, 3737, 6560, -3583, -14731, -5278, 14819,
	 14819, -5278, -14731, -3583, 6560, 3737, -476, 865,
	 748, -2553, -2525, 790, 1648, 207, 157, 658,
	 -356, -1182, -307, 558, 271, -4, 219, 107,
	 -315, -284, 86, 186, 35, -37, -6, -4} // tx_coef[64];
};
struct ad9361_rf_phy *ad9361_phy_0;
struct ad9361_rf_phy *ad9361_phy_1;

extern XSpiPs			spi_instance;


/***************************************************************************//**
 * @brief main
*******************************************************************************/
int main(void)
{
	int temp;
	u8  delay_vec[16];

	xil_printf("************ MAIN START *********************\n\r");
	Xil_ICacheEnable();
	Xil_DCacheEnable();


	// make sure that PL clocks are set correctly
//	xil_printf ("SLCR_LOCKSTA (%08x): %08x\r\n", 0xf800000C, Xil_In32(0xf800000C));
	Xil_Out32(0xf8000008, 0xdf0d); // unlock System Level Control Registers
//	xil_printf ("SLCR_LOCKSTA (%08x): %08x\r\n", 0xf800000C, Xil_In32(0xf800000C));
	Xil_Out32(0xf8000170, 0x00100A00);
	Xil_Out32(0xf8000180, 0x00100400);
	Xil_Out32(0xf8000190, 0x00100400);
	Xil_Out32(0xf8000004, 0x767B); // lock System Level Control Registers
//	xil_printf ("SLCR_LOCKSTA (%08x): %08x\r\n", 0xf800000C, Xil_In32(0xf800000C));

	xil_printf (" ***** Checking PL Clocks\r\n");
	xil_printf ("IO_PLL_CTRL (%08x): %08x\r\n", 0xf8000108, Xil_In32(0xf8000108));
	xil_printf ("PLL_STATUS (%08x):  %08x\r\n",  0xf800010c, Xil_In32(0xf800010c));
	xil_printf ("IO_PLL_CFG (%08x):  %08x\r\n",  0xf8000118, Xil_In32(0xf8000118));
	xil_printf ("FPGA0_CLK_CTRL (%08x): %08x\r\n", 0xf8000170, Xil_In32(0xf8000170));
	xil_printf ("FPGA1_CLK_CTRL (%08x): %08x\r\n", 0xf8000180, Xil_In32(0xf8000180));
	xil_printf ("FPGA2_CLK_CTRL (%08x): %08x\r\n", 0xf8000190, Xil_In32(0xf8000190));

	xil_printf ("Enter to start test:");
	temp = console_get_num(received_cmd);

	gpio_init(GPIO_DEVICE_ID);
//	gpio_direction(54 + 46, 1);

	xil_printf("************ TOGGLE LED *********************\n\r");

	// SDRDC pins
	gpio_direction(LED0_pin, 1);   //
	gpio_direction(LED1_pin, 1);   //
	gpio_direction(LED2_pin, 1);   //
	gpio_direction(LED3_pin, 1);   //
	gpio_direction(AD0_reset_pin, 1);   //
	gpio_direction(AD1_reset_pin, 1);   //

	gpio_direction(AD0_enable_pin, 1);   //
	gpio_direction(AD0_txnrx_pin, 1);   //
	gpio_direction(AD1_enable_pin, 1);   //
	gpio_direction(AD1_txnrx_pin, 1);   //

	gpio_data(AD0_enable_pin, 1);   //
	gpio_data(AD0_txnrx_pin, 1);   //
	gpio_data(AD1_enable_pin, 1);   //
	gpio_data(AD1_txnrx_pin, 1);   //

	gpio_data(LED0_pin, 1);
	gpio_data(AD0_reset_pin, 1);
	gpio_data(AD1_reset_pin, 1);

	sleep(1);

	gpio_data(LED0_pin, 0);
	gpio_data(AD0_reset_pin, 0);
	gpio_data(AD1_reset_pin, 0);

	sleep(1);

	gpio_data(LED0_pin, 1);
	gpio_data(AD0_reset_pin, 1);
	gpio_data(AD1_reset_pin, 1);

	xil_printf("************ SPI INIT *********************\n\r");
	spi_init(SPI_DEVICE_ID, 1, 0);


	xil_printf("************ SPI SS TEST *********************\n\r");
	set_spi_ss(0); xil_printf ("9361_chip_0 initial  %x \n\r", ad9361_spi_read(REG_CTRL_OUTPUT_ENABLE));
	set_spi_ss(1); xil_printf ("9361_chip_1 initial  %x \n\r", ad9361_spi_read(REG_CTRL_OUTPUT_ENABLE));
	set_spi_ss(0);	ad9361_spi_write(REG_CTRL_OUTPUT_ENABLE, 0xaa);
	set_spi_ss(1);	ad9361_spi_write(REG_CTRL_OUTPUT_ENABLE, 0x55);
	set_spi_ss(0);	xil_printf ("9361_chip_0 after  %x \n\r", ad9361_spi_read(REG_CTRL_OUTPUT_ENABLE));
	set_spi_ss(1);	xil_printf ("9361_chip_1 after  %x \n\r", ad9361_spi_read(REG_CTRL_OUTPUT_ENABLE));


	xil_printf("************ ADC INIT *********************\n\r");
	adc_init(0);
	adc_init(1);

//	xil_printf("************ DAC INIT *********************\n\r");
//	dac_init(DATA_SEL_DDS);

	xil_printf("************ 9361 INIT_0 *********************\n\r");
	set_spi_ss(0);
	ad9361_phy_0 = ad9361_init(&default_init_param, AD0_reset_pin, 0);
	ad9361_set_tx_fir_config(ad9361_phy_0, tx_fir_config);
	ad9361_set_rx_fir_config(ad9361_phy_0, rx_fir_config);
	ad9361_set_en_state_machine_mode(ad9361_phy_0,ENSM_STATE_ALERT);
	ad9361_set_en_state_machine_mode(ad9361_phy_0,ENSM_STATE_FDD);

	xil_printf("************ 9361 INIT_1 *********************\n\r");
	set_spi_ss(1);
	ad9361_phy_1 = ad9361_init(&default_init_param, AD1_reset_pin, 1);

	ad9361_phy_1->pdata->port_ctrl.lvds_invert[0] = 0xEF;
	ad9361_phy_1->pdata->port_ctrl.lvds_invert[1] = 0x0F;
	ad9361_pp_port_setup(ad9361_phy_1, false);

	ad9361_set_tx_fir_config(ad9361_phy_1, tx_fir_config);
	ad9361_set_rx_fir_config(ad9361_phy_1, rx_fir_config);
	ad9361_set_en_state_machine_mode(ad9361_phy_1,ENSM_STATE_ALERT);
	ad9361_set_en_state_machine_mode(ad9361_phy_1,ENSM_STATE_FDD);

	// Read version and id registers from both adi controllers
	xil_printf ("9361_ctrl_0  %x:%x \n\r", axiadc_read(ADI_REG_PCORE_VER, 0), axiadc_read(ADI_REG_PCORE_ID, 0));
	xil_printf ("9361_ctrl_1  %x:%x \n\r", axiadc_read(ADI_REG_PCORE_VER, 1), axiadc_read(ADI_REG_PCORE_ID, 1));
//	main_xadcps();




#ifdef DAC_DMA
	dac_init(DATA_SEL_DMA, ad9361_phy_0);
	dac_init(DATA_SEL_DMA, ad9361_phy_1);
#else
//	dac_init(DATA_SEL_DDS, ad9361_phy_0);
//	dac_init(DATA_SEL_DDS, ad9361_phy_1);
#endif

#ifdef CAPTURE_SCRIPT
	adc_capture(16384, ADC_DDR_BASEADDR, 0);
	adc_capture(16384, ADC_DDR_BASEADDR, 1);
	while(1);
#endif

	xil_printf ("Select ADI device to use (0 or 1):\r\n");
	temp = console_get_num(received_cmd);

	if (temp==0) {
		set_spi_ss(0);

		xil_printf("************ 9361_0_CTRL *********************\n\r");
		xil_printf ("Enter Data Clock Frequency (in Hz):");
		temp = console_get_num(received_cmd);
		xil_printf ("%d\n\r", temp);
		ad9361_set_tx_sampling_freq (ad9361_phy_0, temp);

		xil_printf ("\n\r");
		xil_printf (" ***** Calculate 9361_0 RX eye\r\n");
		ad9361_get_rx_sampling_freq (ad9361_phy_0, (uint32_t*)&temp);
		xil_printf("9361_0 SAMP CLK RATE: %d \n\r", temp);
		while (get_eye_rx (ad9361_phy_0, delay_vec) == 0);
		set_eye_rx (ad9361_phy_0, delay_vec);

		xil_printf ("\n\r");
		xil_printf (" ***** Calculate 9361_0 TX eye\r\n");
		if (get_eye_tx (ad9361_phy_0, delay_vec))
			set_eye_tx (ad9361_phy_0, delay_vec);
		else
			ad9361_spi_write(REG_TX_CLOCK_DATA_DELAY, 0x40);

		xil_printf ("Enter to start test:");
		temp = console_get_num(received_cmd);
		//rxtest_main(ad9361_phy_0);
		txrxtest_main(ad9361_phy_0);
	}
	else {
		set_spi_ss(1);

		xil_printf("************ 9361_1_CTRL *********************\n\r");
		xil_printf ("Enter Data Clock Frequency (in Hz):");
		temp = console_get_num(received_cmd);
		xil_printf ("%d\n\r", temp);
		ad9361_set_tx_sampling_freq (ad9361_phy_1, temp);

		xil_printf ("\n\r");
		xil_printf (" ***** Calculate 9361_1 RX eye\r\n");
		ad9361_get_rx_sampling_freq (ad9361_phy_0, (uint32_t*)&temp);
		xil_printf("9361_1 SAMP CLK RATE: %d \n\r", temp);
		while (get_eye_rx (ad9361_phy_1, delay_vec) == 0);
		set_eye_rx (ad9361_phy_1, delay_vec);

		xil_printf ("\n\r");
		xil_printf (" ***** Calculate 9361_1 TX eye\r\n");
		if (get_eye_tx (ad9361_phy_1, delay_vec))
			set_eye_tx (ad9361_phy_1, delay_vec);
		else
			ad9361_spi_write(REG_TX_CLOCK_DATA_DELAY, 0x40);

		xil_printf ("Enter to start test:");
		temp = console_get_num(received_cmd);
		//rxtest_main(ad9361_phy_1);
		txrxtest_main(ad9361_phy_1);
	}
while(1);



	get_help(NULL, 0);

	while(1)
	{
		console_get_command(received_cmd);
		invalid_cmd = 0;
		for(cmd = 0; cmd < cmd_no; cmd++)
		{
			param_no = 0;
			cmd_type = console_check_commands(received_cmd, cmd_list[cmd].name,
											  param, &param_no);
			if(cmd_type == UNKNOWN_CMD)
			{
				invalid_cmd++;
			}
			else
			{
				cmd_list[cmd].function(param, param_no);
			}
		}
		if(invalid_cmd == cmd_no)
		{
			console_print("Invalid command!\n");
		}
	}

	Xil_DCacheDisable();
	Xil_ICacheDisable();

	return 0;
}
