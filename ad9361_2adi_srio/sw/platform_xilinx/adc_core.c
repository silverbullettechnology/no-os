/***************************************************************************//**
 *   @file   adc_core.c
 *   @brief  Implementation of ADC Core Driver.
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
#include <stdlib.h>
#include <xil_cache.h>
#include <xil_io.h>
#include "adc_core.h"
#include "parameters.h"
#include "xtime_l.h"
#include "util.h"


#ifdef XPAR_AXI_DMA_0_BASEADDR
	#include "xaxidma_hw.h"
#endif

/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/
struct adc_state adc_st;

/***************************************************************************//**
 * @brief adc_read
*******************************************************************************/
//void adc_read(uint32_t regAddr, uint32_t *data)
//{
//	*data = Xil_In32(CF_AD9361_RX_BASEADDR + regAddr);
//}

//void adc_read(uint32_t regAddr, uint32_t *data, uint32_t adi_num)
void adc_read(struct ad9361_rf_phy *phy, uint32_t regAddr, uint32_t *data)
{
	switch (phy->id_no)
	{
		case 0:
			*data = Xil_In32(CF_AD9361_0_RX_BASEADDR + regAddr);
			break;
		case 1:
			*data = Xil_In32(CF_AD9361_1_RX_BASEADDR + regAddr);
			break;
		default:
			*data = Xil_In32(CF_AD9361_RX_BASEADDR + regAddr);
	}
}

/***************************************************************************//**
 * @brief adc_write
*******************************************************************************/
//void adc_write(uint32_t regAddr, uint32_t data)
//{
//	Xil_Out32(CF_AD9361_RX_BASEADDR + regAddr, data);
//}

//void adc_write(uint32_t regAddr, uint32_t data, uint32_t adi_num)
void adc_write(struct ad9361_rf_phy *phy, uint32_t regAddr, uint32_t data)
{
	switch (phy->id_no)
	{
		case 0:
			Xil_Out32(CF_AD9361_0_RX_BASEADDR + regAddr, data);
			break;
		case 1:
			Xil_Out32(CF_AD9361_1_RX_BASEADDR + regAddr, data);
			break;
		default:
			Xil_Out32(CF_AD9361_RX_BASEADDR + regAddr, data);
	}
}

/***************************************************************************//**
 * @brief adc_init
*******************************************************************************/
void adc_init(struct ad9361_rf_phy *phy)
{
	adc_write(phy, ADC_REG_RSTN, 0);
	adc_write(phy, ADC_REG_RSTN, ADC_RSTN);

	adc_write(phy, ADC_REG_CHAN_CNTRL(0),
		ADC_IQCOR_ENB | ADC_FORMAT_SIGNEXT | ADC_FORMAT_ENABLE | ADC_ENABLE);
	adc_write(phy, ADC_REG_CHAN_CNTRL(1),
		ADC_IQCOR_ENB | ADC_FORMAT_SIGNEXT | ADC_FORMAT_ENABLE | ADC_ENABLE);

	adc_st.rx2tx2 = phy->pdata->rx2tx2;
	if(adc_st.rx2tx2)
	{
		adc_write(phy, ADC_REG_CHAN_CNTRL(2),
			ADC_IQCOR_ENB | ADC_FORMAT_SIGNEXT | ADC_FORMAT_ENABLE | ADC_ENABLE);
		adc_write(phy, ADC_REG_CHAN_CNTRL(3),
			ADC_IQCOR_ENB | ADC_FORMAT_SIGNEXT | ADC_FORMAT_ENABLE | ADC_ENABLE);
	}
	else
	{
		adc_write(phy, ADC_REG_CHAN_CNTRL(2), 0);
		adc_write(phy, ADC_REG_CHAN_CNTRL(3), 0);
	}
}


#ifdef XPAR_AXI_DMAC_0_BASEADDR

/***************************************************************************//**
 * @brief adc_dma_read
*******************************************************************************/
//void adc_dma_read(uint32_t regAddr, uint32_t *data)
//{
//	*data = Xil_In32(CF_AD9361_RX_DMA_BASEADDR + regAddr);
//}
void adc_dma_read(uint32_t regAddr, uint32_t *data, uint32_t adi_num)
{
	switch (adi_num)
	{
		case 0:
			*data = Xil_In32(CF_AD9361_0_RX_DMA_BASEADDR + regAddr);
			break;
		case 1:
			*data = Xil_In32(CF_AD9361_1_RX_DMA_BASEADDR + regAddr);
			break;
		default:
			*data = Xil_In32(CF_AD9361_RX_DMA_BASEADDR + regAddr);
	}
}

/***************************************************************************//**
 * @brief adc_dma_write
*******************************************************************************/
//void adc_dma_write(uint32_t regAddr, uint32_t data)
//{
//	Xil_Out32(CF_AD9361_RX_DMA_BASEADDR + regAddr, data);
//}
void adc_dma_write(uint32_t regAddr, uint32_t data, uint32_t adi_num)
{
	switch (adi_num)
	{
		case 0:
			Xil_Out32(CF_AD9361_0_RX_DMA_BASEADDR + regAddr, data);
			break;
		case 1:
			Xil_Out32(CF_AD9361_1_RX_DMA_BASEADDR + regAddr, data);
			break;
		default:
			Xil_Out32(CF_AD9361_RX_DMA_BASEADDR + regAddr, data);
	}
}




/***************************************************************************//**
 * @brief adc_capture
*******************************************************************************/
int32_t adc_capture_dmac(struct ad9361_rf_phy *phy, uint32_t size, uint32_t start_address, int timeout)
{
	uint32_t reg_val;
	uint32_t transfer_id;

	XTime tEnd, tCur;
	uint32_t length;

	if(adc_st.rx2tx2)
	{
		length = (size * 8);
	}
	else
	{
		length = (size * 4);
	}

	int timeout_sec = timeout;

	XTime_GetTime(&tCur);
	tEnd  = tCur + ((XTime) timeout_sec) * COUNTS_PER_SECOND;

	adc_dma_write(phy, AXI_DMAC_REG_CTRL, 0x0);
	adc_dma_write(phy, AXI_DMAC_REG_CTRL, AXI_DMAC_CTRL_ENABLE);

	adc_dma_write(phy, AXI_DMAC_REG_IRQ_MASK, 0x0);

	adc_dma_read(phy, AXI_DMAC_REG_TRANSFER_ID, &transfer_id);
	adc_dma_read(phy, AXI_DMAC_REG_IRQ_PENDING, &reg_val);
	adc_dma_write(phy, AXI_DMAC_REG_IRQ_PENDING, reg_val);

	adc_dma_write(phy, AXI_DMAC_REG_DEST_ADDRESS, start_address);
	adc_dma_write(phy, AXI_DMAC_REG_DEST_STRIDE, 0x0);
	adc_dma_write(phy, AXI_DMAC_REG_X_LENGTH, length - 1);
	adc_dma_write(phy, AXI_DMAC_REG_Y_LENGTH, 0x0);

	//dac_write(phy, ADI_REG_CNTRL_1, ADI_ENABLE);   /// herwin
	adc_dma_write(phy, AXI_DMAC_REG_START_TRANSFER, 0x1);

//	xil_printf("************ transfer QUEUE'ing *********************\n\r");
	/* Wait until the new transfer is queued. */
	do {
		adc_dma_read(phy, AXI_DMAC_REG_START_TRANSFER, &reg_val);
	    XTime_GetTime(&tCur);
	    if (tCur > tEnd) return (-1);
	}
	while(reg_val == 1);
//	xil_printf("************ transfer QUEUE'd *********************\n\r");

	/* Wait until the current transfer is completed. */
	do {
		adc_dma_read(phy, AXI_DMAC_REG_IRQ_PENDING, &reg_val);
	    XTime_GetTime(&tCur);
	    if (tCur > tEnd) return (-1);
	}
	while(reg_val != (AXI_DMAC_IRQ_SOT | AXI_DMAC_IRQ_EOT));

//	xil_printf("************ transfer COMPLETE'd *********************\n\r");
	adc_dma_write(phy, AXI_DMAC_REG_IRQ_PENDING, reg_val);

	/* Wait until the transfer with the ID transfer_id is completed. */
	do {
		adc_dma_read(phy, AXI_DMAC_REG_TRANSFER_DONE, &reg_val);
	    XTime_GetTime(&tCur);
	    if (tCur > tEnd) return (-1);
	}
	while((reg_val & (1 << transfer_id)) != (1 << transfer_id));

	return 0;
}

#endif

int32_t adc_capture (struct ad9361_rf_phy *phy, uint32_t size, uint32_t start_address, int timeout, int adi2axis_mode)
{
#ifdef XPAR_AXI_DMAC_0_BASEADDR
	return adc_capture_dmac(phy, size, start_address, timeout);
#endif

#ifdef XPAR_AXI_DMA_0_BASEADDR
	return adc_capture_axidma(phy, size, start_address, timeout, adi2axis_mode);
#endif
}



#ifdef XPAR_AXI_DMA_0_BASEADDR

/***************************************************************************//**
 * @brief dma_write
*******************************************************************************/
void adc_axidma_write(struct ad9361_rf_phy *phy, uint32_t regAddr, uint32_t data)
{
	switch (phy->id_no)
	{
	case 0:
		Xil_Out32(XPAR_AXI_DMA_0_BASEADDR + XAXIDMA_RX_OFFSET + regAddr, data);
		break;
	case 1:
		Xil_Out32(XPAR_AXI_DMA_1_BASEADDR + XAXIDMA_RX_OFFSET + regAddr, data);
		break;
	default:
		Xil_Out32(XPAR_AXI_DMA_0_BASEADDR + XAXIDMA_RX_OFFSET + regAddr, data);
	}
}

/***************************************************************************/

void adi2axis_write(struct ad9361_rf_phy *phy, uint32_t regAddr, uint32_t data)
{
	switch (phy->id_no)
	{
		case 0:
			Xil_Out32(XPAR_ADI2AXIS_0_BASEADDR + regAddr, data);
			break;
		case 1:
			Xil_Out32(XPAR_ADI2AXIS_1_BASEADDR + regAddr, data);
			break;
		default:
			Xil_Out32(XPAR_ADI2AXIS_0_BASEADDR + regAddr, data);
	}
}



/***************************************************************************/

void adi2axis_read(struct ad9361_rf_phy *phy, uint32_t regAddr, uint32_t *data)
{
	switch (phy->id_no)
	{
		case 0:
			*data = Xil_In32(XPAR_ADI2AXIS_0_BASEADDR + regAddr);
			break;
		case 1:
			*data = Xil_In32(XPAR_ADI2AXIS_1_BASEADDR + regAddr);
			break;
		default:
			*data = Xil_In32(XPAR_ADI2AXIS_0_BASEADDR + regAddr);
	}
}


void reset_dmarx(struct ad9361_rf_phy *phy)
{
	adc_axidma_write(phy, XAXIDMA_CR_OFFSET, XAXIDMA_CR_RESET_MASK); // Reset DMA engine
	adc_axidma_write(phy, XAXIDMA_CR_OFFSET, 0);
}

/***************************************************************************//**
 * @brief adc_capture
*******************************************************************************/

// adi2axis_mode : 0 legacy - adi2axis block controls number of words to pass
//                 1 trig   - vita49_trig block controls when to start and stop data transfer (done bit not used)
int32_t adc_capture_axidma(struct ad9361_rf_phy *phy, uint32_t size, uint32_t start_address, int timeout, int adi2axis_mode)
{
	uint32_t status;
	uint32_t ba;

	XTime tEnd, tCur;
	uint32_t length;
	if(adc_st.rx2tx2)
	{
		length = (size * 8);
	}
	else
	{
		length = (size * 4);
	}

	int timeout_sec = timeout;

	XTime_GetTime(&tCur);
	tEnd  = tCur + ((XTime) timeout_sec) * COUNTS_PER_SECOND;


	ba = start_address + (size*16);
	Xil_Out32((ba + 0x000), (ba + 0x40)); // next descriptor
	Xil_Out32((ba + 0x004), 0x00); // reserved
	Xil_Out32((ba + 0x008), start_address); // start address
	Xil_Out32((ba + 0x00c), 0x00); // reserved
	Xil_Out32((ba + 0x010), 0x00); // reserved
	Xil_Out32((ba + 0x014), 0x00); // reserved
	Xil_Out32((ba + 0x018), (size*16)); // no. of bytes
	Xil_Out32((ba + 0x01c), 0x00); // status
	Xil_Out32((ba + 0x040), (ba + 0x00)); // next descriptor
	Xil_Out32((ba + 0x044), 0x00); // reserved
	Xil_Out32((ba + 0x048), start_address); // start address
	Xil_Out32((ba + 0x04c), 0x00); // reserved
	Xil_Out32((ba + 0x050), 0x00); // reserved
	Xil_Out32((ba + 0x054), 0x00); // reserved
	Xil_Out32((ba + 0x058), (size*16)); // no. of bytes
	Xil_Out32((ba + 0x05c), 0x00); // status
	Xil_DCacheFlush();

//	adc_axidma_write(phy, XAXIDMA_CR_OFFSET, XAXIDMA_CR_RESET_MASK); // Reset DMA engine
//	adc_axidma_write(phy, XAXIDMA_CR_OFFSET, 0);
	adc_axidma_write(phy, XAXIDMA_CDESC_OFFSET, ba); // Current descriptor pointer
	adc_axidma_write(phy, XAXIDMA_CR_OFFSET, XAXIDMA_CR_RUNSTOP_MASK); // Start DMA channel
	adc_axidma_write(phy, XAXIDMA_TDESC_OFFSET, (ba+0x40)); // Tail descriptor pointer


	adi2axis_write(phy, ADI2AXIS_CTRL_REG, 0);
	// legacy adi2axis mode
	if (adi2axis_mode == 0x0){
		adi2axis_write(phy, ADI2AXIS_COUNT_REG, length);
		adi2axis_write(phy, ADI2AXIS_CTRL_REG, 0x1);   // start adi2axis block in legacy mode

		do
		{
			adi2axis_read(phy, ADI2AXIS_STAT_REG, &status);
	//		xil_printf("adc_capture: stat %x\n\r", status);
			usleep(1000);
			XTime_GetTime(&tCur);
			if (tCur > tEnd) return (-1);
		}
		while((status & 0x02) == 0);  // poll done bit in status register
	}

	if (adi2axis_mode == 0x1){
		adi2axis_write(phy, ADI2AXIS_COUNT_REG, length);
		adi2axis_write(phy, ADI2AXIS_CTRL_REG, 0x2);     // start axi2axis block in trigger mode
	}


	adc_read(phy, ADI2AXIS_STAT_REG, &status);
	if((status & ADC_DMA_OVF) == ADC_DMA_OVF)
	{
		xil_printf("adc_capture: overflow occurred\n\r");
	}
	Xil_DCacheFlush();

	return 0;

}

#endif