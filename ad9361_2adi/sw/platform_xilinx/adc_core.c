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

void adc_read(uint32_t regAddr, uint32_t *data, uint32_t adi_num)
{
	switch (adi_num)
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

void adc_write(uint32_t regAddr, uint32_t data, uint32_t adi_num)
{
	switch (adi_num)
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
 * @brief axiadc_read
*******************************************************************************/
//uint32_t axiadc_read(uint32_t reg)
//{
//	uint32_t val;
//
//	adc_read(reg, &val);
//
//	return val;
//}
//uint32_t axiadc_read(uint32_t reg, uint32_t adi_num)
//{
//	uint32_t val;
//
//	adc_read(reg, &val, adi_num);
//
//	return val;
//}

/***************************************************************************//**
 * @brief axiadc_write
*******************************************************************************/
//void axiadc_write(uint32_t reg, uint32_t val)
//{
//	adc_write(reg, val);
//}
//void axiadc_write(uint32_t reg, uint32_t val, uint32_t adi_num)
//{
//	adc_write(reg, val, adi_num);
//}

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
 * @brief adc_init
*******************************************************************************/
void adc_init(struct ad9361_rf_phy *phy)
{
	u8 adi_num;
	adi_num = phy->pcore_id;

	xil_printf("************ adc_init (%x) *********************\n\r", adi_num);

	xil_printf("************ adc_write(ADI_REG_RSTN, 0) *********************\n\r");
	adc_write(ADI_REG_RSTN, 0, adi_num);
	xil_printf("************ ADI_REG_RSTN, ADI_RSTN *********************\n\r");
	adc_write(ADI_REG_RSTN, ADI_RSTN, adi_num);

	xil_printf("************ adc_write(ADI_REG_CHAN_CNTRL(0) *********************\n\r");
	adc_write(ADI_REG_CHAN_CNTRL(0),
		ADI_IQCOR_ENB | ADI_FORMAT_SIGNEXT | ADI_FORMAT_ENABLE | ADI_ENABLE, adi_num);
	adc_write(ADI_REG_CHAN_CNTRL(1),
		ADI_IQCOR_ENB | ADI_FORMAT_SIGNEXT | ADI_FORMAT_ENABLE | ADI_ENABLE, adi_num);

	adc_st.rx2tx2 = phy->pdata->rx2tx2;
	if(adc_st.rx2tx2)
	{
		adc_write(ADI_REG_CHAN_CNTRL(2),
			ADI_IQCOR_ENB | ADI_FORMAT_SIGNEXT | ADI_FORMAT_ENABLE | ADI_ENABLE, adi_num);
		adc_write(ADI_REG_CHAN_CNTRL(3),
			ADI_IQCOR_ENB | ADI_FORMAT_SIGNEXT | ADI_FORMAT_ENABLE | ADI_ENABLE, adi_num);
	}
	xil_printf("************ EXIT: adc_init(void) *********************\n\r");

}

/***************************************************************************//**
 * @brief adc_capture
*******************************************************************************/
int32_t adc_capture(uint32_t size, uint32_t start_address, uint32_t adi_num)
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

	int timeout_sec = 2;

	XTime_GetTime(&tCur);
	tEnd  = tCur + ((XTime) timeout_sec) * COUNTS_PER_SECOND;

	adc_dma_write(AXI_DMAC_REG_CTRL, 0x0, adi_num);
	adc_dma_write(AXI_DMAC_REG_CTRL, AXI_DMAC_CTRL_ENABLE, adi_num);

	adc_dma_write(AXI_DMAC_REG_IRQ_MASK, 0x0, adi_num);

	adc_dma_read(AXI_DMAC_REG_TRANSFER_ID, &transfer_id, adi_num);
	adc_dma_read(AXI_DMAC_REG_IRQ_PENDING, &reg_val, adi_num);
	adc_dma_write(AXI_DMAC_REG_IRQ_PENDING, reg_val, adi_num);

	adc_dma_write(AXI_DMAC_REG_DEST_ADDRESS, start_address, adi_num);
	adc_dma_write(AXI_DMAC_REG_DEST_STRIDE, 0x0, adi_num);
	adc_dma_write(AXI_DMAC_REG_X_LENGTH, length - 1, adi_num);
	adc_dma_write(AXI_DMAC_REG_Y_LENGTH, 0x0, adi_num);

	//dac_write(ADI_REG_CNTRL_1, ADI_ENABLE);   /// herwin
	adc_dma_write(AXI_DMAC_REG_START_TRANSFER, 0x1, adi_num);

//	xil_printf("************ transfer QUEUE'ing *********************\n\r");
	/* Wait until the new transfer is queued. */
	do {
		adc_dma_read(AXI_DMAC_REG_START_TRANSFER, &reg_val, adi_num);
	    XTime_GetTime(&tCur);
	    if (tCur > tEnd) return (-1);
	}
	while(reg_val == 1);
//	xil_printf("************ transfer QUEUE'd *********************\n\r");

	/* Wait until the current transfer is completed. */
	do {
		adc_dma_read(AXI_DMAC_REG_IRQ_PENDING, &reg_val, adi_num);
	    XTime_GetTime(&tCur);
	    if (tCur > tEnd) return (-1);
	}
	while(reg_val != (AXI_DMAC_IRQ_SOT | AXI_DMAC_IRQ_EOT));

//	xil_printf("************ transfer COMPLETE'd *********************\n\r");
	adc_dma_write(AXI_DMAC_REG_IRQ_PENDING, reg_val, adi_num);

	/* Wait until the transfer with the ID transfer_id is completed. */
	do {
		adc_dma_read(AXI_DMAC_REG_TRANSFER_DONE, &reg_val, adi_num);
	    XTime_GetTime(&tCur);
	    if (tCur > tEnd) return (-1);
	}
	while((reg_val & (1 << transfer_id)) != (1 << transfer_id));

	return 0;
}
