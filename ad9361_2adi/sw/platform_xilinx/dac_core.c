/***************************************************************************//**
 *   @file   dac_core.c
 *   @brief  Implementation of DAC Core Driver.
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
#include <stdint.h>
#include <xil_cache.h>
#include <xil_io.h>
#include "dac_core.h"
#include "parameters.h"
#include "util.h"
#include "ad9361_api.h"

/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/
//struct dds_state dds_st;
extern struct ad9361_rf_phy *ad9361_phy;

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
const uint16_t sine_lut[32] = {
		0x000, 0x031, 0x061, 0x08D, 0x0B4, 0x0D4, 0x0EC, 0x0FA,
		0x0FF, 0x0FB, 0x0ED, 0x0D5, 0x0B5, 0x08E, 0x062, 0x032,
		0x000, 0xFCF, 0xF9F, 0xF73, 0xF4C, 0xF2C, 0xF14, 0xF06,
		0xF00, 0xF05, 0xF13, 0xF2B, 0xF4B, 0xF72, 0xF9E, 0xFCE
};

const uint16_t tri_lut[32] = {
		0x000, 0x020, 0x040, 0x060, 0x080, 0x0A0, 0x0C0, 0x0E0,
		0x100, 0x0E1, 0x0C1, 0x0A1, 0x081, 0x061, 0x041, 0x021,
		0x001, 0xFE1, 0xFC1, 0xFA1, 0xF81, 0xF61, 0xF41, 0xF21,
		0xF00, 0xF20, 0xF40, 0xF60, 0xF80, 0xFA0, 0xFC0, 0xFE0
};

const uint16_t rmp_lut[32] = {
		0xF12, 0xF22, 0xF32, 0xF42, 0xF52, 0xF62, 0xF72, 0xF82,
		0xF92, 0xFA2, 0xFB2, 0xFC2, 0xFD2, 0xFE2, 0xFF2, 0x002,
		0x012, 0x022, 0x032, 0x042, 0x052, 0x062, 0x072, 0x082,
		0x092, 0x0A2, 0x0B2, 0x0C2, 0x0D2, 0x0E2, 0x0F2, 0x102
};


const uint16_t sqr_lut[32] = {
		0x101, 0x0FF, 0x0FD, 0x0FB, 0x0F9, 0x0F7, 0x0F5, 0x0F3,
		0xF01, 0xF03, 0xF05, 0xF07, 0xF09, 0xF0B, 0xF0D, 0xF0F,
		0x0FE, 0x0FC, 0x0FA, 0x0F8, 0x0F6, 0x0F4, 0x0F2, 0x0F0,
		0xF00, 0xF02, 0xF04, 0xF06, 0xF08, 0xF0A, 0xF0C, 0xF0D
};

/***************************************************************************//**
 * @brief dac_read
*******************************************************************************/
void dac_read(uint32_t regAddr, uint32_t *data, uint32_t adi_num)
{
	switch (adi_num)
	{
		case 0:
			*data = Xil_In32(CF_AD9361_0_TX_BASEADDR + regAddr);
			break;
		case 1:
			*data = Xil_In32(CF_AD9361_1_TX_BASEADDR + regAddr);
			break;
		default:
			*data = Xil_In32(CF_AD9361_TX_BASEADDR + regAddr);
	}
}

/***************************************************************************//**
 * @brief dac_write
*******************************************************************************/
void dac_write(uint32_t regAddr, uint32_t data, uint32_t adi_num)
{
	switch (adi_num)
	{
		case 0:
			Xil_Out32(CF_AD9361_0_TX_BASEADDR + regAddr, data);
			break;
		case 1:
			Xil_Out32(CF_AD9361_1_TX_BASEADDR + regAddr, data);
			break;
		default:
			Xil_Out32(CF_AD9361_TX_BASEADDR + regAddr, data);
	}
}

/***************************************************************************//**
 * @brief dac_dma_read
*******************************************************************************/
void dac_dma_read(uint32_t regAddr, uint32_t *data, uint32_t adi_num)
{
	switch (adi_num)
	{
		case 0:
			*data = Xil_In32(CF_AD9361_0_TX_DMA_BASEADDR + regAddr);
			break;
		case 1:
			*data = Xil_In32(CF_AD9361_1_TX_DMA_BASEADDR + regAddr);
			break;
		default:
			*data = Xil_In32(CF_AD9361_TX_DMA_BASEADDR + regAddr);
	}
}

/***************************************************************************//**
 * @brief dac_dma_write
*******************************************************************************/
void dac_dma_write(uint32_t regAddr, uint32_t data, uint32_t adi_num)
{
	switch (adi_num)
	{
		case 0:
			Xil_Out32(CF_AD9361_0_TX_DMA_BASEADDR + regAddr, data);
			break;
		case 1:
			Xil_Out32(CF_AD9361_1_TX_DMA_BASEADDR + regAddr, data);
			break;
		default:
			Xil_Out32(CF_AD9361_TX_DMA_BASEADDR + regAddr, data);
	}
}

/***************************************************************************//**
 * @brief dds_default_setup
*******************************************************************************/
static int dds_default_setup(uint32_t chan, uint32_t phase,
							 uint32_t freq, double scale, struct ad9361_rf_phy *phy)
{
	dds_set_phase(chan, phase, phy);
	dds_set_frequency(chan, freq, phy);
	dds_set_scale(chan, scale, phy);
	phy->dds_st.cached_freq[chan] = freq;
	phy->dds_st.cached_phase[chan] = phase;
	phy->dds_st.cached_scale[chan] = scale;

	return 0;
}

/***************************************************************************//**
 * @brief dac_stop
*******************************************************************************/
void dac_stop(struct ad9361_rf_phy *phy)
{
	u8 adi_num;

	adi_num = phy->pcore_id;
	if (PCORE_VERSION_MAJOR(phy->dds_st.pcore_version) < 8)
	{
		dac_write(ADI_REG_CNTRL_1, 0, adi_num);
	}
}

/***************************************************************************//**
 * @brief dac_start_sync
*******************************************************************************/
void dac_start_sync(bool force_on, struct ad9361_rf_phy *phy)
{
	u8 adi_num;

	adi_num = phy->pcore_id;
	if (PCORE_VERSION_MAJOR(phy->dds_st.pcore_version) < 8)
	{
		dac_write(ADI_REG_CNTRL_1, (phy->dds_st.enable || force_on) ? ADI_ENABLE : 0, adi_num);
	}
	else
	{
		dac_write(ADI_REG_CNTRL_1, ADI_SYNC, adi_num);
	}
}

/***************************************************************************//**
 * @brief dac_init
*******************************************************************************/
void dac_init (struct ad9361_rf_phy *phy, uint8_t data_sel)
{
	const int num_packets = 1024;
	int packets;
	uint32_t tx_count;
	uint32_t index;
	uint32_t index_i1;
	uint32_t index_q1;
	uint32_t index_i2;
	uint32_t index_q2;
	uint32_t data_i1;
	uint32_t data_q1;
	uint32_t data_i2;
	uint32_t data_q2;
	u8 adi_num;

	adi_num = phy->pcore_id;

	dac_write(ADI_REG_RSTN, 0x0, adi_num);
	dac_write(ADI_REG_RSTN, ADI_RSTN | ADI_MMCM_RSTN, adi_num);

	dac_write(ADI_REG_RATECNTRL, ADI_RATE(3), adi_num);

	phy->dds_st.dac_clk = &phy->clks[TX_SAMPL_CLK]->rate;
	phy->dds_st.num_dds_channels = 8;	// FIXME

	dac_read(ADI_REG_VERSION, &phy->dds_st.pcore_version, adi_num);

	dac_stop(phy);
	switch (data_sel) {
	case DATA_SEL_DDS:
		dds_default_setup(DDS_CHAN_TX1_I_F1, 90000, 1000000, 0.25, phy);
		dds_default_setup(DDS_CHAN_TX1_I_F2, 90000, 1000000, 0.25, phy);
		dds_default_setup(DDS_CHAN_TX1_Q_F1, 0, 1000000, 0.25, phy);
		dds_default_setup(DDS_CHAN_TX1_Q_F2, 0, 1000000, 0.25, phy);
		dds_default_setup(DDS_CHAN_TX2_I_F1, 90000, 1000000, 0.25, phy);
		dds_default_setup(DDS_CHAN_TX2_I_F2, 90000, 1000000, 0.25, phy);
		dds_default_setup(DDS_CHAN_TX2_Q_F1, 0, 1000000, 0.25, phy);
		dds_default_setup(DDS_CHAN_TX2_Q_F2, 0, 1000000, 0.25, phy);
		dac_write(ADI_REG_CNTRL_2, 0, adi_num);
		dac_datasel(-1, DATA_SEL_DDS, phy);
		break;
	case DATA_SEL_DMA:
		tx_count = sizeof(sine_lut) / sizeof(uint16_t);
 // 	    xil_printf("dac_init: tx_count: %d \r\n", tx_count);

		for (packets=0; packets<num_packets; packets++)
		{
			for(index = 0; index < (tx_count * 2); index+=2)
			{
				index_i1 = index;
				index_q1 = index + (tx_count / 4);
				if(index_q1 >= (tx_count * 2))
					index_q1 -= (tx_count * 2);
				data_i1 = (sine_lut[index_i1 / 2] << 20);
				data_q1 = (sine_lut[index_q1 / 2] << 4);
				Xil_Out32(DAC_DDR_BASEADDR + packets*(tx_count * 8) + index * 4, data_i1 | data_q1);

				index_i2 = index_i1 + (tx_count / 2);
				index_q2 = index_q1 + (tx_count / 2);
				if(index_i2 >= (tx_count * 2))
					index_i2 -= (tx_count * 2);
				if(index_q2 >= (tx_count * 2))
					index_q2 -= (tx_count * 2);
				data_i2 = (sine_lut[index_i2 / 2] << 20);
				data_q2 = (sine_lut[index_q2 / 2] << 4);
				Xil_Out32(DAC_DDR_BASEADDR + packets*(tx_count * 8) + (index + 1) * 4, data_i2 | data_q2);
			}
		}

		Xil_DCacheFlush();
		dac_dma_write(AXI_DMAC_REG_CTRL, 0, adi_num);
		dac_dma_write(AXI_DMAC_REG_CTRL, AXI_DMAC_CTRL_ENABLE, adi_num);
		dac_dma_write(AXI_DMAC_REG_SRC_ADDRESS, DAC_DDR_BASEADDR, adi_num);
		dac_dma_write(AXI_DMAC_REG_SRC_STRIDE, 0x0, adi_num);

//		dac_dma_write(AXI_DMAC_REG_X_LENGTH, (tx_count * 8 * num_packets) - 1, adi_num);
		dac_dma_write(AXI_DMAC_REG_X_LENGTH, 255, adi_num);

		dac_dma_write(AXI_DMAC_REG_Y_LENGTH, 0x0, adi_num);
		dac_dma_write(AXI_DMAC_REG_START_TRANSFER, 0x1, adi_num);
		dac_write(ADI_REG_CNTRL_2, 0, adi_num);
		dac_datasel(-1, DATA_SEL_DMA, phy);
		break;
	default:
		break;
	}
	phy->dds_st.enable = true;
	dac_start_sync(0, phy);
}

/***************************************************************************//**
 * @brief dds_set_frequency
*******************************************************************************/
void dds_set_frequency(uint32_t chan, uint32_t freq, struct ad9361_rf_phy *phy)
{
	uint64_t val64;
	uint32_t reg;
	u8 adi_num;

	adi_num = phy->pcore_id;

	phy->dds_st.cached_freq[chan] = freq;
	dac_stop(phy);
	dac_read(ADI_REG_CHAN_CNTRL_2_IIOCHAN(chan), &reg, adi_num);
	reg &= ~ADI_DDS_INCR(~0);
	val64 = (uint64_t) freq * 0xFFFFULL;
	do_div(&val64, *phy->dds_st.dac_clk);
	reg |= ADI_DDS_INCR(val64) | 1;
	dac_write(ADI_REG_CHAN_CNTRL_2_IIOCHAN(chan), reg, adi_num);
	dac_start_sync(0, phy);
}

/***************************************************************************//**
 * @brief dds_set_phase
*******************************************************************************/
void dds_set_phase(uint32_t chan, uint32_t phase, struct ad9361_rf_phy *phy)
{
	uint64_t val64;
	uint32_t reg;
	u8 adi_num;

	adi_num = phy->pcore_id;

	phy->dds_st.cached_phase[chan] = phase;
	dac_stop(phy);
	dac_read(ADI_REG_CHAN_CNTRL_2_IIOCHAN(chan), &reg, adi_num);
	reg &= ~ADI_DDS_INIT(~0);
	val64 = (uint64_t) phase * 0x10000ULL + (360000 / 2);
	do_div(&val64, 360000);
	reg |= ADI_DDS_INIT(val64);
	dac_write(ADI_REG_CHAN_CNTRL_2_IIOCHAN(chan), reg, adi_num);
	dac_start_sync(0, phy);
}

/***************************************************************************//**
 * @brief dds_set_phase
*******************************************************************************/
void dds_set_scale(uint32_t chan, double scale, struct ad9361_rf_phy *phy)
{
	uint32_t scale_reg;
	uint32_t sign_part;
	uint32_t int_part;
	uint32_t fract_part;
	u8 adi_num;

	adi_num = phy->pcore_id;

	if (PCORE_VERSION_MAJOR(phy->dds_st.pcore_version) > 6)
	{
		if(scale >= 1.0)
		{
			sign_part = 0;
			int_part = 1;
			fract_part = 0;
			phy->dds_st.cached_scale[chan] = 1.0;
			goto set_scale_reg;
		}
		if(scale <= -1.0)
		{
			sign_part = 1;
			int_part = 1;
			fract_part = 0;
			phy->dds_st.cached_scale[chan] = -1.0;
			goto set_scale_reg;
		}
		if(scale < 0)
		{
			sign_part = 1;
			int_part = 0;
			phy->dds_st.cached_scale[chan] = scale;
			scale *= -1;
			goto set_scale_reg;
		}
		sign_part = 0;
		int_part = 0;
		phy->dds_st.cached_scale[chan] = scale;
		fract_part = (uint32_t)(scale * 0x4000);
	set_scale_reg:
		scale_reg = (sign_part << 15) | (int_part << 14) | fract_part;
	}
	else
	{
		if(scale >= 1.0)
		{
			scale_reg = 0;
			scale = 1.0;
		}
		if(scale <= 0.0)
		{
			scale_reg = 0;
			scale = 0.0;
		}
		phy->dds_st.cached_scale[chan] = scale;
		fract_part = (uint32_t)(scale * 1000000);
		scale_reg = 500000 / fract_part;
	}
	dac_stop(phy);
	dac_write(ADI_REG_CHAN_CNTRL_1_IIOCHAN(chan), ADI_DDS_SCALE(scale_reg), adi_num);
	dac_start_sync(0, phy);
}

/***************************************************************************//**
 * @brief dds_update
*******************************************************************************/
void dds_update(struct ad9361_rf_phy *phy)
{
	uint32_t chan;

	for(chan = DDS_CHAN_TX1_I_F1; chan <= DDS_CHAN_TX2_Q_F2; chan++)
	{
		dds_set_frequency(chan, phy->dds_st.cached_freq[chan], phy);
		dds_set_phase(chan, phy->dds_st.cached_phase[chan], phy);
		dds_set_scale(chan, phy->dds_st.cached_scale[chan], phy);
	}
}

/***************************************************************************//**
 * @brief dac_datasel
*******************************************************************************/
int dac_datasel(int32_t chan, enum dds_data_select sel, struct ad9361_rf_phy *phy)
{
	u8 adi_num;

	adi_num = phy->pcore_id;


//	xil_printf (" PCORE VERSION: %d \r\n", PCORE_VERSION_MAJOR(phy->dds_st.pcore_version));

	if (PCORE_VERSION_MAJOR(phy->dds_st.pcore_version) > 7) {
		if (chan < 0) { /* ALL */
			int i;
			for (i = 0; i < phy->dds_st.num_dds_channels; i++) {
				dac_write(ADI_REG_CHAN_CNTRL_7(i), sel, adi_num);
			}
		} else {
			dac_write(ADI_REG_CHAN_CNTRL_7(chan), sel, adi_num);
		}
	} else {
		uint32_t reg;

		switch(sel) {
		case DATA_SEL_DDS:
		case DATA_SEL_SED:
		case DATA_SEL_DMA:
			dac_read(ADI_REG_CNTRL_2, &reg, adi_num);
			reg &= ~ADI_DATA_SEL(~0);
			reg |= ADI_DATA_SEL(sel);
			dac_write(ADI_REG_CNTRL_2, reg, adi_num);
			break;
		default:
			return -EINVAL;
		}
	}

	return 0;
}
