Chipscope signals 
  Debug units are connected to the second ADI path (AD1)
  There are are three debug units
  
UNIT:0 (INTERFACE DEBUG)
  The transmit chain (tx DMA -> data latch -> data mux -> DDR output)
  The following signals are found between tx DMA and data latch;  when dac_valid then dac_data* gets latched onto tx_data_*_d
      dac_valid;
      dac_data_i1;
      dac_data_q1;
      dac_data_i2;
      dac_data_q2;
  The following signals are found between data latch and data mux; multiplexes the tx data and generates frame data to interface with DDR
      tx_data_i1_d;
      tx_data_q1_d;
      tx_data_i2_d;
      tx_data_q2_d;
      tx_data_sel_s;
      tx_data_cnt;
      dac_r1_mode;
  The following signals are found between data mux and DDR output
      tx_frame;   (trigger)
      tx_data_n;
      tx_data_p;
  
  The receive chain (DDR input -> generate control signals  -> data demux -> ... )
                               -> generate data assuming R1
                               -> generate data assuming R2
  The following signals are at the DDR input
      rx_frame_p_s;
      rx_frame_n_s;
      rx_data_p_s;
      rx_data_n_s;
  The following signals are derived from these recieved signals
      rx_frame_n;
      rx_frame;   (trigger)
      rx_frame_d;
      rx_frame_s;
      rx_data_n;
      rx_data;
      rx_data_d;
  From the derived signals and assuming R1 mode, the following signals are generated
      rx_error_r1;
      rx_valid_r1;
      rx_data_i_r1;
      rx_data_q_r1;
  From the derived signals and assuming R2 mode, the following signals are generated
      rx_error_r2;
      rx_valid_r2;
      rx_data_i1_r2;
      rx_data_q1_r2;
      rx_data_i2_r2;
      rx_data_q2_r2;
  The final stage selects the final output signals based on r1_mode set by user
      adc_r1_mode;
      adc_status;
      adc_valid;    (trigger)
      adc_data_i1;
      adc_data_q1;
      adc_data_i2;
      adc_data_q2;


UNIT:1 (ADC MONITOR)
  Connected to receive chain of ADI controller (DDR input-> Interface Logic -> IQ correction -> rx DMA)
  Interface Logic demuxes the DDR input depending on number of receivers and rx protocol
  The following signals are found between Interface Logic and IQ correction
      adc_data_i1;
      adc_data_q1;
      adc_data_i2;
      adc_data_q2;
      adc_valid;    (trigger)
  The following signals are found between IQ correction and rx DMA
      adc_iqcor_data_0_s;
      adc_iqcor_data_1_s;
      adc_iqcor_data_2_s;
      adc_iqcor_data_3_s;
      adc_iqcor_valid_0_s;
      adc_iqcor_valid_1_s;
      adc_iqcor_valid_2_s;
      adc_iqcor_valid_3_s;
      adc_iqcore_valid_s; (trigger)

UNIT:2 (DMA MONITOR)
  Connected to the AXI bus between the transmit DMA module and PS
  (tx DMA <-> ADI controller)