
#include <stdio.h>

#include "xparameters.h"
#include "xil_exception.h"
#include "xil_cache.h"
#include "xstatus.h"
#include "platform.h"
#include "axilite_test.h"
#include "parameters.h"

#include "console.h"
#include "xllfifo.h"

#include "sbt_mod.h"
#include "srio_mod.h"
#include "srio_data_pkts.h"


#ifdef XPAR_SRIO_DMA_BASEADDR
	#include "xaxidma_hw.h"
#endif




int init_srio_fifo()
{
	int Status;

	Status = XLlFifoInit(&Fifo_Initiator, XPAR_AXI_SRIO_INITIATOR_FIFO_DEVICE_ID);
	if (Status != XST_SUCCESS) {
		xil_printf("XLlFifoInit: Fifo_Initiator Failed\n\r");
		xil_printf("--- Exiting main() ---\n\r");
		return XST_FAILURE;
	}

	Status = XLlFifoInit(&Fifo_Target, XPAR_AXI_SRIO_TARGET_FIFO_DEVICE_ID);
	if (Status != XST_SUCCESS) {
		xil_printf("XLlFifoInit: Fifo_Target Failed\n\r");
		xil_printf("--- Exiting main() ---\n\r");
		return XST_FAILURE;
	}

//	Status = XLlFifoInit(&Fifo_UserDef, XPAR_AXI_SRIO_USERDEF_FIFO_DEVICE_ID);
//	if (Status != XST_SUCCESS) {
//		xil_printf("XLlFifoInit: Fifo_UserDef Failed\n\r");
//		xil_printf("--- Exiting main() ---\n\r");
//		return XST_FAILURE;
//	}
}

static int CheckData(u32 *src, u32 *dest, u32 size)
{
	int Index = 0;

	for(Index = 0; Index < size; Index++) {
  	  xil_printf(" %08x  %08x", src[Index], dest[Index]);
	  if (src[Index] != dest[Index])
	  	xil_printf("  * error \r\n");
	  else
	  	xil_printf("    \r\n");
	}

	xil_printf("\r\n ");
	return XST_SUCCESS;
}



void reset_swrite_mod()
{
#ifdef	XPAR_SRIO_SWRITE_PACK_0_BASEADDR
    AXILITE_TEST_mWriteSlaveReg0 (XPAR_SRIO_SWRITE_PACK_0_BASEADDR, 0, 0x02);
    AXILITE_TEST_mWriteSlaveReg0 (XPAR_SRIO_SWRITE_PACK_0_BASEADDR, 0, 0x00);
#endif

#ifdef    XPAR_SRIO_SWRITE_PACK_1_BASEADDR
    AXILITE_TEST_mWriteSlaveReg0 (XPAR_SRIO_SWRITE_PACK_1_BASEADDR, 0, 0x02);
    AXILITE_TEST_mWriteSlaveReg0 (XPAR_SRIO_SWRITE_PACK_1_BASEADDR, 0, 0x00);
#endif

#ifdef    XPAR_SRIO_SWRITE_UNPACK_0_BASEADDR
    AXILITE_TEST_mWriteSlaveReg0 (XPAR_SRIO_SWRITE_UNPACK_0_BASEADDR, 0, 0x02);
    AXILITE_TEST_mWriteSlaveReg0 (XPAR_SRIO_SWRITE_UNPACK_0_BASEADDR, 0, 0x00);
#endif
}


void set_swrite_srcdest (int adi_path, int srcdest)
{
	if (adi_path==0) AXILITE_TEST_mWriteSlaveReg2 (XPAR_SRIO_SWRITE_PACK_0_BASEADDR, 0, srcdest);
	if (adi_path==1) AXILITE_TEST_mWriteSlaveReg2 (XPAR_SRIO_SWRITE_PACK_1_BASEADDR, 0, srcdest);
}



void set_swrite_addr (int adi_path, int txrx, int addr)
// adi_path: ad1(0), ad2(1)
// txrx:     tx.swrite_pack(0), rx.swrite_unpack(1)
// addr:     srio address for routing swrite packets
{

#ifdef    XPAR_SRIO_SWRITE_UNPACK_0_BASEADDR
	if (txrx == 1) {
		if (adi_path==0) AXILITE_TEST_mWriteSlaveReg1 (XPAR_SRIO_SWRITE_UNPACK_0_BASEADDR, 0, addr);
		if (adi_path==1) AXILITE_TEST_mWriteSlaveReg2 (XPAR_SRIO_SWRITE_UNPACK_0_BASEADDR, 0, addr);
	}
#endif

#ifdef XPAR_SRIO_SWRITE_PACK_1_BASEADDR
	if (txrx == 0) {
		if (adi_path==0) AXILITE_TEST_mWriteSlaveReg1 (XPAR_SRIO_SWRITE_PACK_0_BASEADDR, 0, addr);
		if (adi_path==1) AXILITE_TEST_mWriteSlaveReg1 (XPAR_SRIO_SWRITE_PACK_1_BASEADDR, 0, addr);
	}
#endif
}

void enable_swrite_unpack ()
{
#ifdef    XPAR_SRIO_SWRITE_UNPACK_0_BASEADDR
    AXILITE_TEST_mWriteSlaveReg0 (XPAR_SRIO_SWRITE_UNPACK_0_BASEADDR, 0, 0x1);
#endif
}

void enable_swrite_pack ()
{
#ifdef    XPAR_SRIO_SWRITE_PACK_0_BASEADDR
    AXILITE_TEST_mWriteSlaveReg0 (XPAR_SRIO_SWRITE_PACK_0_BASEADDR, 0, 0x1);
#endif

#ifdef    XPAR_SRIO_SWRITE_PACK_1_BASEADDR
    AXILITE_TEST_mWriteSlaveReg0 (XPAR_SRIO_SWRITE_PACK_1_BASEADDR, 0, 0x1);
#endif
}

// sends swrite packets from initiator fifo to test routing in feedback mode
int swrite_tozynq_test()
{
	int Status;

	set_swrite_bypass(1);   // send swrite packets to target fifo (zynq)

    xil_printf("\n\r--- SWRITE TO ZYNQ TEST ---\n\r");

    xil_printf("\n\r--- swrite_pkt1 ---\n\r");
	xil_printf("TxSend\n\r");
	/* Transmit the Data Stream */
	Status = TxSend(&Fifo_Initiator, swrite_pkt1, 11);
	if (Status != XST_SUCCESS){
		xil_printf("Transmisson of Data failed\n\r");
		return XST_FAILURE;
	}
	xil_printf("TxSend: Success\r\n");

	Status = RxReceive(&Fifo_Target, TReqBuffer);
	CheckData( swrite_pkt1, TReqBuffer, 11);

    xil_printf("\n\r--- swrite_pkt2 ---\n\r");
	xil_printf("TxSend\n\r");
	/* Transmit the Data Stream */
	Status = TxSend(&Fifo_Initiator, swrite_pkt2, 10);
	if (Status != XST_SUCCESS){
		xil_printf("Transmisson of Data failed\n\r");
		return XST_FAILURE;
	}
	xil_printf("TxSend: Success\r\n");
	Status = RxReceive(&Fifo_Target, TReqBuffer);
	CheckData( swrite_pkt2, TReqBuffer, 10);


    xil_printf("\n\r--- swrite_pkt3a ---\n\r");
	xil_printf("TxSend\n\r");
	/* Transmit the Data Stream */
	Status = TxSend(&Fifo_Initiator, swrite_pkt3a, 9);
	if (Status != XST_SUCCESS){
		xil_printf("Transmisson of Data failed\n\r");
		return XST_FAILURE;
	}
	xil_printf("TxSend: Success\r\n");
	Status = RxReceive(&Fifo_Target, TReqBuffer);
	CheckData( swrite_pkt3a, TReqBuffer, 9);

    xil_printf("\n\r--- swrite_pkt3b ---\n\r");
	xil_printf("TxSend\n\r");
	/* Transmit the Data Stream */
	Status = TxSend(&Fifo_Initiator, swrite_pkt3b, 5);
	if (Status != XST_SUCCESS){
		xil_printf("Transmisson of Data failed\n\r");
		return XST_FAILURE;
	}
	xil_printf("TxSend: Success\r\n");
	Status = RxReceive(&Fifo_Target, TReqBuffer);
	CheckData( swrite_pkt3b, TReqBuffer, 5);


    xil_printf("\n\r--- swrite_pkt4 ---\n\r");
	xil_printf("TxSend\n\r");
	/* Transmit the Data Stream */
	Status = TxSend(&Fifo_Initiator, swrite_pkt4, 7);
	if (Status != XST_SUCCESS){
		xil_printf("Transmisson of Data failed\n\r");
		return XST_FAILURE;
	}
	xil_printf("TxSend: Success\r\n");
	Status = RxReceive(&Fifo_Target, TReqBuffer);
	CheckData( swrite_pkt4, TReqBuffer, 7);

	getchar();

	return 1;
}

// sends swrite packets from initiator fifo to test routing in feedback mode
// packet should not show up on target fifo
int swrite_toadi_test()
{
	int Status;

    xil_printf("\n\r--- SWRITE TO ADI TEST ---\n\r");

	set_swrite_bypass(0);    //send swrite packets to adi
	reset_swrite_mod();
	enable_swrite_unpack();

	set_swrite_addr (0, 1, 0x0000dea8);
	//set_swrite_addr (1, 1, 0x00000000);
	set_swrite_addr (1, 1, 0xbeef0120);

	enable_vita_assem (XPAR_VITA49_ASSEM_0_BASEADDR);

    xil_printf("\n\r--- swrite_pkt1 ---\n\r");
	getchar();
	xil_printf("TxSend\n\r");
	/* Transmit the Data Stream */
	Status = TxSend(&Fifo_Initiator, swrite_pkt1, 11);
	if (Status != XST_SUCCESS){
		xil_printf("Transmisson of Data failed\n\r");
		return XST_FAILURE;
	}
	xil_printf ("vita_assem_0 err: %x\n\r", get_vita_assem_err(0));
	xil_printf("TxSend: Success\r\n");


    xil_printf("\n\r--- swrite_pkt5 ---\n\r");
	getchar();
	xil_printf("TxSend\n\r");
	/* Transmit the Data Stream */
	Status = TxSend(&Fifo_Initiator, swrite_pkt5, 67);
	if (Status != XST_SUCCESS){
		xil_printf("Transmisson of Data failed\n\r");
		return XST_FAILURE;
	}
	getchar();
	xil_printf ("vita_assem_0 err: %x\n\r", get_vita_assem_err(0));
	xil_printf("TxSend: Success\r\n");


    xil_printf("\n\r--- swrite_pkt3a ---\n\r");
	getchar();
	xil_printf("TxSend\n\r");
	/* Transmit the Data Stream */
	Status = TxSend(&Fifo_Initiator, swrite_pkt3a, 9);
	if (Status != XST_SUCCESS){
		xil_printf("Transmisson of Data failed\n\r");
		return XST_FAILURE;
	}
	xil_printf ("vita_assem_0 err: %x\n\r", get_vita_assem_err(0));
	xil_printf("TxSend: Success\r\n");

    xil_printf("\n\r--- swrite_pkt3b ---\n\r");
	getchar();
	xil_printf("TxSend\n\r");
	/* Transmit the Data Stream */
	Status = TxSend(&Fifo_Initiator, swrite_pkt3b, 5);
	if (Status != XST_SUCCESS){
		xil_printf("Transmisson of Data failed\n\r");
		return XST_FAILURE;
	}
	xil_printf ("vita_assem_0 err: %x\n\r", get_vita_assem_err(0));
	xil_printf("TxSend: Success\r\n");


    xil_printf("\n\r--- swrite_pkt4 ---\n\r");
	getchar();
	xil_printf("TxSend\n\r");
	/* Transmit the Data Stream */
	Status = TxSend(&Fifo_Initiator, swrite_pkt4, 7);
	if (Status != XST_SUCCESS){
		xil_printf("Transmisson of Data failed\n\r");
		return XST_FAILURE;
	}
	xil_printf ("vita_assem_0 err: %x\n\r", get_vita_assem_err(0));
	xil_printf("TxSend: Success\r\n");


	  xil_printf("\n\r--- swrite_pkt2 ---\n\r");
		getchar();
		xil_printf("TxSend\n\r");
		/* Transmit the Data Stream */
		Status = TxSend(&Fifo_Initiator, swrite_pkt2, 10);
		if (Status != XST_SUCCESS){
			xil_printf("Transmisson of Data failed\n\r");
			return XST_FAILURE;
		}
		xil_printf("TxSend: Success\r\n");



	return 1;
}



int msg_test()
{
	int Status;
    int temp;

//	AXILITE_TEST_mWriteSlaveReg0 (XPAR_SYS_REG_0_BASEADDR, SRIO_IREQ_SRCDEST_REG, 0x98765432);

    xil_printf("\n\r--- msg_pkt1 ---\n\r");
	xil_printf("TxSend\n\r");
	/* Transmit the Data Stream */
	Status = TxSend(&Fifo_Initiator, msg_pkt1, 67);
	if (Status != XST_SUCCESS){
		xil_printf("Transmisson of Data failed\n\r");
		return XST_FAILURE;
	}
	xil_printf("TxSend: Success\r\n");
	sleep(1);

	/* Receive the Data Stream */
	Status = RxReceive(&Fifo_Target, TReqBuffer);
//	if (Status != XST_SUCCESS){
//		xil_printf("Receiving data failed");
//		return XST_FAILURE;
//	}
	CheckData( msg_pkt1, TReqBuffer, 67);


    xil_printf("\n\r--- msg_pkt2 ---\n\r");
	xil_printf("TxSend\n\r");
	/* Transmit the Data Stream */
	Status = TxSend(&Fifo_Initiator, msg_pkt2, 5);
	if (Status != XST_SUCCESS){
		xil_printf("Transmisson of Data failed\n\r");
		return XST_FAILURE;
	}
	xil_printf("TxSend: Success\r\n");

	/* Receive the Data Stream */
	Status = RxReceive(&Fifo_Target, TReqBuffer);
	if (Status != XST_SUCCESS){
		xil_printf("Receiving data failed");
		return XST_FAILURE;
	}
	CheckData( msg_pkt2, TReqBuffer, 5);


    xil_printf("\n\r--- msg_pkt3 ---\n\r");
	xil_printf("TxSend\n\r");
	/* Transmit the Data Stream */
	Status = TxSend(&Fifo_Initiator, msg_pkt3, 7);
	if (Status != XST_SUCCESS){
		xil_printf("Transmisson of Data failed\n\r");
		return XST_FAILURE;
	}
	xil_printf("TxSend: Success\r\n");

	/* Receive the Data Stream */
	Status = RxReceive(&Fifo_Target, TReqBuffer);
	if (Status != XST_SUCCESS){
		xil_printf("Receiving data failed");
		return XST_FAILURE;
	}
	CheckData( msg_pkt3, TReqBuffer, 7);

//	temp    = AXILITE_TEST_mReadSlaveReg0 (XPAR_SYS_REG_0_BASEADDR, SRIO_TREQ_SRCDEST_REG);
//  xil_printf("\n RECEIVED TUSER: %x\n\r", temp);

	getchar();
}


int resp_test()
{
	int Status;

    xil_printf("\n\r--- resp_pkt1 ---\n\r");
	xil_printf("TxSend\n\r");
	/* Transmit the Data Stream */
	Status = TxSend(&Fifo_Target, resp_pkt1, 67);
	if (Status != XST_SUCCESS){
		xil_printf("Transmisson of Data failed\n\r");
		return XST_FAILURE;
	}
	xil_printf("TxSend: Success\r\n");
	sleep(6);

	/* Receive the Data Stream */
	Status = RxReceive(&Fifo_Initiator, TReqBuffer);
//	if (Status != XST_SUCCESS){
//		xil_printf("Receiving data failed");
//		return XST_FAILURE;
//	}
	CheckData(resp_pkt1, TReqBuffer, 67);


    xil_printf("\n\r--- resp_pkt2 ---\n\r");
	xil_printf("TxSend\n\r");
	/* Transmit the Data Stream */
	Status = TxSend(&Fifo_Target, resp_pkt2, 3);
	if (Status != XST_SUCCESS){
		xil_printf("Transmisson of Data failed\n\r");
		return XST_FAILURE;
	}
	xil_printf("TxSend: Success\r\n");

	/* Receive the Data Stream */
	Status = RxReceive(&Fifo_Initiator, TReqBuffer);
	if (Status != XST_SUCCESS){
		xil_printf("Receiving data failed");
		return XST_FAILURE;
	}
	CheckData( resp_pkt2, TReqBuffer, 3);


    xil_printf("\n\r--- resp_pkt3 ---\n\r");
	xil_printf("TxSend\n\r");
	/* Transmit the Data Stream */
	Status = TxSend(&Fifo_Target, resp_pkt3, 3);
	if (Status != XST_SUCCESS){
		xil_printf("Transmisson of Data failed\n\r");
		return XST_FAILURE;
	}
	xil_printf("TxSend: Success\r\n");

	/* Receive the Data Stream */
	Status = RxReceive(&Fifo_Initiator, TReqBuffer);
	if (Status != XST_SUCCESS){
		xil_printf("Receiving data failed");
		return XST_FAILURE;
	}
	CheckData(resp_pkt3, TReqBuffer, 3);

	getchar();
}


int XLlFifoInit(XLlFifo *InstancePtr, u16 DeviceId)
{
	XLlFifo_Config *Config;
	int Status;
	int i;
	int Error;
	Status = XST_SUCCESS;

	xil_printf("XLlFfio_LookupConfig\n\r");
	/* Initialize the Device Configuration Interface driver */
	Config = XLlFfio_LookupConfig(DeviceId);
	if (!Config) {
		xil_printf("No config found for %d\r\n", DeviceId);
		return XST_FAILURE;
	}

	xil_printf("XLlFifo_CfgInitialize\n\r");
	/*
	 * This is where the virtual address would be used, this example
	 * uses physical address.
	 */
	Status = XLlFifo_CfgInitialize(InstancePtr, Config, Config->BaseAddress);
	if (Status != XST_SUCCESS) {
		xil_printf("Initialization failed\n\r");
		return Status;
	}

	xil_printf("XLlFifo_Status\n\r");
	/* Check for the Reset value */
	Status = XLlFifo_Status(InstancePtr);
	XLlFifo_IntClear(InstancePtr,0xffffffff);
	Status = XLlFifo_Status(InstancePtr);
	if(Status != 0x0) {
		xil_printf("\n ERROR : Reset value of ISR0 : 0x%x\t"
			    "Expected : 0x0\n\r",
			    XLlFifo_Status(InstancePtr));
		return XST_FAILURE;
	}

	return Status;
}



/*****************************************************************************/
/*
*
* TxSend routine, It will send the requested amount of data at the
* specified addr.
*
* @param	InstancePtr is a pointer to the instance of the
*		XLlFifo component.
*
* @param	SourceAddr is the address where the FIFO stars writing
*
* @return	-XST_SUCCESS to indicate success
*		-XST_FAILURE to indicate failure
*
* @note		None
*
******************************************************************************/
int TxSend(XLlFifo *InstancePtr, u32 *pkt, u32 size)
{
	int Status;
	int Index;
	int i;
	u8  TxBuffer[8192];
	u32 *TxBufferPtr;
	u8  Value;

	xil_printf(" Transmitting Data ... \r\n");

    TxBufferPtr = pkt;
	Xil_DCacheFlushRange((u32)&TxBuffer, size);

	for(i=0 ; i < size ; i++){
      if( XLlFifo_iTxVacancy(InstancePtr) ){
        XLlFifo_TxPutWord(InstancePtr, TxBufferPtr[i]);
      }
    }

	XLlFifo_iTxSetLen(InstancePtr, size*4);

	/* Check for Transmission completion */
	while( !(XLlFifo_IsTxDone(InstancePtr)) ){

	}

	/* Transmission Complete */
	return XST_SUCCESS;
}

/*****************************************************************************/
/*
*
* RxReceive routine.It will receive the data from the FIFO.
*
* @param	InstancePtr is a pointer to the instance of the
*		XLlFifo instance.
*
* @param	DestinationAddr is the address where to copy the received data.
*
* @return	-XST_SUCCESS to indicate success
*		-XST_FAILURE to indicate failure
*
* @note		None
*
******************************************************************************/
int RxReceive (XLlFifo *InstancePtr, u32* DestinationAddr)
{

	int i;
	int Status;
	u32 RxWord;
    u32 Occupancy;

	u32 ReceiveLength = MAX_BUFF_SIZE * BEAT_SIZE;

	xil_printf(" Receiving data ....\n\r");
	/* Read Receive Length */
	Occupancy     = XLlFifo_iRxOccupancy(InstancePtr);

	if (Occupancy == 0){
		xil_printf("  No data in receive buffer \n\r");
		return XST_FAILURE;
	}

	ReceiveLength = (XLlFifo_iRxGetLen(InstancePtr))/WORD_SIZE;
//	xil_printf(" Occupancy: %d  ReceiveLength: %d ....\n\r", Occupancy, ReceiveLength);

	/* Start Receiving */
	for ( i=0; i < ReceiveLength; i++){
		RxWord = 0;
		RxWord = XLlFifo_RxGetWord(InstancePtr);
//		xil_printf("  RxWord %x \n\r", RxWord);

		if(XLlFifo_iRxOccupancy(InstancePtr)){
			RxWord = XLlFifo_RxGetWord(InstancePtr);
//			xil_printf("  RxWord* %x \n\r", RxWord);
		}
		*(DestinationAddr+i) = RxWord;
//		xil_printf("  DestinationAddr[%d] RxWord %x \n\r",i,  *(DestinationAddr+i) );

	}

	Status = XLlFifo_IsRxDone(InstancePtr);
	if(Status != TRUE){
		xil_printf("Failing in receive complete ... \r\n");
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}


void reset_srio()
{
	u32 temp;

#ifdef XPAR_SYS_REG_0_BASEADDR
	temp = AXILITE_TEST_mReadSlaveReg0(XPAR_SYS_REG_0_BASEADDR, SRIO_CTRL_REG);
	AXILITE_TEST_mWriteSlaveReg0 (XPAR_SYS_REG_0_BASEADDR, SRIO_CTRL_REG, temp|0x01);
	sleep(1);
	AXILITE_TEST_mWriteSlaveReg0 (XPAR_SYS_REG_0_BASEADDR, SRIO_CTRL_REG, temp);
	sleep(1);
#endif
}

void set_adi_adc_snk (int snk)
{
	u32 temp;
	snk = snk & 0x03;

#ifdef XPAR_SYS_REG_0_BASEADDR
	AXILITE_TEST_mWriteSlaveReg0 (XPAR_SYS_REG_0_BASEADDR, ADI_ADC_SNK_REG, snk);
#endif

#ifdef XPAR_ROUTING_REG_0_BASEADDR
	temp = AXILITE_TEST_mReadSlaveReg0(XPAR_ROUTING_REG_0_BASEADDR, 0x0);
	temp = (temp & (~0x03)) | snk;
	AXILITE_TEST_mWriteSlaveReg0 (XPAR_ROUTING_REG_0_BASEADDR, 0x0, temp);
#endif
}


void set_dma_loopback (int val)
{
#ifdef XPAR_ROUTING_REG_0_BASEADDR
	u32 temp;
	val = val & 0x03;
	temp = AXILITE_TEST_mReadSlaveReg0(XPAR_ROUTING_REG_0_BASEADDR, 0x0);
	temp = temp & (~0x0C);
	AXILITE_TEST_mWriteSlaveReg0 (XPAR_ROUTING_REG_0_BASEADDR, 0x0, temp | (val << 2) );

	temp = AXILITE_TEST_mReadSlaveReg0(XPAR_ROUTING_REG_0_BASEADDR, 0x0);

	xil_printf("routing register: %x\n\r", temp);

#endif
}


void set_swrite_bypass(int val)
{
	u32 temp;
	val = val & 0x03;

#ifdef XPAR_ROUTING_REG_0_BASEADDR
	temp = AXILITE_TEST_mReadSlaveReg0(XPAR_ROUTING_REG_0_BASEADDR, 0x000);
	temp = temp & (~0x30);
	AXILITE_TEST_mWriteSlaveReg0 (XPAR_ROUTING_REG_0_BASEADDR, 0x000, temp | (val << 4) );
#endif

	temp = AXILITE_TEST_mReadSlaveReg0(XPAR_ROUTING_REG_0_BASEADDR, 0x0);

	xil_printf("routing register: %x\n\r", temp);
}

void set_srio_rxlpmen (int en)
{
	u32 temp;
#ifdef XPAR_SYS_REG_0_BASEADDR
	temp = AXILITE_TEST_mReadSlaveReg0(XPAR_SYS_REG_0_BASEADDR, SRIO_CTRL_REG);
	if (en)
		AXILITE_TEST_mWriteSlaveReg0 (XPAR_SYS_REG_0_BASEADDR, SRIO_CTRL_REG, RXLPMEN | temp);
	else
		AXILITE_TEST_mWriteSlaveReg0 (XPAR_SYS_REG_0_BASEADDR, SRIO_CTRL_REG, ~RXLPMEN & temp);
#endif
}


void set_srio_loopback (int val)
{
#ifdef XPAR_SYS_REG_0_BASEADDR
	u32 temp;
	val = val & 0x07;
	temp = AXILITE_TEST_mReadSlaveReg0(XPAR_SYS_REG_0_BASEADDR, SRIO_CTRL_REG);
	temp = temp & ~LOOPBACK_MASK;
	AXILITE_TEST_mWriteSlaveReg0 (XPAR_SYS_REG_0_BASEADDR, SRIO_CTRL_REG, temp | (val << LOOPBACK_SHIFT) );
#endif
}

void set_srio_diffctl (int val)
{
#ifdef XPAR_SYS_REG_0_BASEADDR
	u32 temp;
	val = val & 0x0f;
	temp = AXILITE_TEST_mReadSlaveReg0(XPAR_SYS_REG_0_BASEADDR, SRIO_CTRL_REG);
	temp = temp & ~DIFFCTRL_MASK;
	AXILITE_TEST_mWriteSlaveReg0 (XPAR_SYS_REG_0_BASEADDR, SRIO_CTRL_REG, temp | (val << DIFFCTRL_SHIFT) );
#endif
}

void set_srio_txprecursor (int val)
{
#ifdef XPAR_SYS_REG_0_BASEADDR
	u32 temp;
	val = val & 0x1F;
	temp = AXILITE_TEST_mReadSlaveReg0(XPAR_SYS_REG_0_BASEADDR, SRIO_CTRL_REG);
	temp = temp & ~TXPRECURSOR_MASK;
	AXILITE_TEST_mWriteSlaveReg0 (XPAR_SYS_REG_0_BASEADDR, SRIO_CTRL_REG, temp | (val << TXPRECURSOR_SHIFT) );
#endif
}

void set_srio_txpostcursor (int val)
{
#ifdef XPAR_SYS_REG_0_BASEADDR
	u32 temp;
	val = val & 0x1F;
	temp = AXILITE_TEST_mReadSlaveReg0(XPAR_SYS_REG_0_BASEADDR, SRIO_CTRL_REG);
	temp = temp & ~TXPOSTCURSOR_MASK;
	AXILITE_TEST_mWriteSlaveReg0 (XPAR_SYS_REG_0_BASEADDR, SRIO_CTRL_REG, temp | (val << TXPOSTCURSOR_SHIFT) );
#endif
}



void print_srio_stat()
{
	u32 base_addr = XPAR_SYS_REG_0_BASEADDR;
	u32 temp;
	int mode_1x, clk_lock_out, port_initialized, link_initialized;
	int gtrx_disperr, gtrx_notintable, port_err;
	int device_id;

	temp    = AXILITE_TEST_mReadSlaveReg0 (base_addr, SRIO_CTRL_REG);
	xil_printf ("SRIO RESET: (%x)\n\r", temp);

	temp    = AXILITE_TEST_mReadSlaveReg0 (base_addr, SRIO_STAT_REG);

	mode_1x          = (temp & 0x08)? 1 : 0;
	clk_lock_out     = (temp & 0x04)? 1 : 0;
	port_initialized = (temp & 0x02)? 1 : 0;
	link_initialized = (temp & 0x01)? 1 : 0;

	gtrx_disperr     = (temp & 0x40)? 1 : 0;
	gtrx_notintable  = (temp & 0x20)? 1 : 0;
	port_err         = (temp & 0x10)? 1 : 0;

	device_id        = (temp & 0xFFFF0000) >> 16;

	xil_printf ("PRINT SRIO: (%x)\n\r", temp);
	xil_printf ("  DEVICE ID:        %x\n\r", device_id);
	xil_printf ("  mode_1x:          %x\n\r", mode_1x);
	xil_printf ("  clk_lock_out:     %x\n\r", clk_lock_out);
	xil_printf ("  port_initialized: %x\n\r", port_initialized);
	xil_printf ("  link_initialized: %x\n\r", link_initialized);
	xil_printf ("  gtrx_disperr:     %x\n\r", gtrx_disperr);
	xil_printf ("  gtrx_notintable:  %x\n\r", gtrx_notintable);
	xil_printf ("  port_err:         %x\n\r", port_err);
}



void print_srio_maint()
{
	u32 base_addr = XPAR_SRIO_GEN2_0_BASEADDR;
	u32 temp;

	temp    = AXILITE_TEST_mReadSlaveReg0 (base_addr, 0x00);
	xil_printf ("SRIO Device ID CAR:   (%08x)\n\r", temp);
	temp    = AXILITE_TEST_mReadSlaveReg0 (base_addr, 0x04);
	xil_printf ("SRIO Device INFO CAR: (%08x)\n\r", temp);
	temp    = AXILITE_TEST_mReadSlaveReg0 (base_addr, 0x08);
	xil_printf ("SRIO Assembly ID CAR: (%08x)\n\r", temp);

	temp    = AXILITE_TEST_mReadSlaveReg0 (base_addr, 0x60);
	xil_printf ("Base Device ID CSR:   (%08x)\n\r", temp);

}

void print_drp_rxafe_attrib()
{
	u32 base_addr = XPAR_DRP_BRIDGE_0_BASEADDR;
    u32 drp0_base = base_addr;
    u32 drp1_base = base_addr + 0x0800;
    u32 drp2_base = base_addr + 0x1000;
    u32 drp3_base = base_addr + 0x1800;

    u16 d0, d1, d2, d3;

    int rx_cm_sel     = 0x011;
    int pma_rsv2      = 0x082;
    int term_rcal_cfg = 0x069;

    d0 = AXILITE_TEST_mReadSlaveReg0 (drp0_base, rx_cm_sel * 0x04);
    d1 = AXILITE_TEST_mReadSlaveReg0 (drp1_base, rx_cm_sel * 0x04);
    d2 = AXILITE_TEST_mReadSlaveReg0 (drp2_base, rx_cm_sel * 0x04);
    d3 = AXILITE_TEST_mReadSlaveReg0 (drp3_base, rx_cm_sel * 0x04);
	xil_printf ("RX_CM_SEL / RX_CM_TRIM: %04x.%04x.%04x.%04x\n\r", d0, d1, d2, d3);

    d0 = AXILITE_TEST_mReadSlaveReg0 (drp0_base, pma_rsv2 * 0x04);
    d1 = AXILITE_TEST_mReadSlaveReg0 (drp1_base, pma_rsv2 * 0x04);
    d2 = AXILITE_TEST_mReadSlaveReg0 (drp2_base, pma_rsv2 * 0x04);
    d3 = AXILITE_TEST_mReadSlaveReg0 (drp3_base, pma_rsv2 * 0x04);
	xil_printf ("PMA_RSV2:               %04x.%04x.%04x.%04x\n\r", d0, d1, d2, d3);

    d0 = AXILITE_TEST_mReadSlaveReg0 (drp0_base, term_rcal_cfg * 0x04);
    d1 = AXILITE_TEST_mReadSlaveReg0 (drp1_base, term_rcal_cfg * 0x04);
    d2 = AXILITE_TEST_mReadSlaveReg0 (drp2_base, term_rcal_cfg * 0x04);
    d3 = AXILITE_TEST_mReadSlaveReg0 (drp3_base, term_rcal_cfg * 0x04);
	xil_printf ("TERM_RCAL_CFG:          %04x.%04x.%04x.%04x\n\r", d0, d1, d2, d3);

}

void srio_tx_axidma_write(uint32_t regAddr, uint32_t data)
{
	Xil_Out32(XPAR_SRIO_DMA_BASEADDR + regAddr, data);
}

void reset_sriodmatx()
{
	srio_tx_axidma_write(XAXIDMA_CR_OFFSET, XAXIDMA_CR_RESET_MASK); // Reset DMA engine
	srio_tx_axidma_write(XAXIDMA_CR_OFFSET, 0);
}

void srio_dma_setup ()
{

	Xil_Out32(XPAR_SRIO_DMA_COMB_0_BASEADDR + 0x0000, 0x02);  // reset
	Xil_Out32(XPAR_SRIO_DMA_COMB_0_BASEADDR + 0x0000, 0x00);
	Xil_Out32(XPAR_SRIO_DMA_COMB_0_BASEADDR + 0x0008, 0x08); // number of packets
	Xil_Out32(XPAR_SRIO_DMA_COMB_0_BASEADDR + 0x0000, 0x01); // enable



	Xil_Out32(XPAR_SRIO_DMA_SPLIT_0_BASEADDR + 0x0000, 0x02);  // reset
	Xil_Out32(XPAR_SRIO_DMA_SPLIT_0_BASEADDR + 0x0000, 0x00);
	Xil_Out32(XPAR_SRIO_DMA_SPLIT_0_BASEADDR + 0x0008, 0x08); // number of packets
	Xil_Out32(XPAR_SRIO_DMA_SPLIT_0_BASEADDR + 0x0000, 0x01); // enable

}

void srio_tx_axidma ()
{
	uint32_t length;
	uint32_t ba;
	uint32_t temp;

	Xil_Out32(DAC_DDR_BASEADDR + 0x00, 0x00000005);
	Xil_Out32(DAC_DDR_BASEADDR + 0x04, 0xffffffff);
	Xil_Out32(DAC_DDR_BASEADDR + 0x08, 0x0000dea8);
	Xil_Out32(DAC_DDR_BASEADDR + 0x0c, 0x00600000);
	Xil_Out32(DAC_DDR_BASEADDR + 0x10, 0x08000010);
	Xil_Out32(DAC_DDR_BASEADDR + 0x14, 0x0000efbe);
	Xil_Out32(DAC_DDR_BASEADDR + 0x18, 0x01dadada);
	Xil_Out32(DAC_DDR_BASEADDR + 0x1c, 0x02efefef);
	Xil_Out32(DAC_DDR_BASEADDR + 0x20, 0x03dadada);
	Xil_Out32(DAC_DDR_BASEADDR + 0x24, 0x04efefef);
	Xil_Out32(DAC_DDR_BASEADDR + 0x28, 0x05dadada);
	Xil_Out32(DAC_DDR_BASEADDR + 0x2c, 0x06efefef);

//	u32 swrite_dma_pkt1 [] = {
//		0x00000000, 0x00000005,
//		0x0000dea8, 0x00600000,    // SRIO address: 0xdea8
//		0x08000010, 0x0000efbe,//	0x10000008, 0xbeef0000,    // VITA IF Packet, size:8, streamID:0xbeef000
//		0x01dadada, 0x02efefef,//	0xdadada01, 0xefefef02,    // payload: 6 words
//		0x03dadada, 0x04efefef,//	0xdadada03, 0xefefef04,
//		0x05dadada, 0x06efefef //	0xdadada05, 0xefefef06
//	};
	length = 8*32 + 8;   //swrite = 256 bytes; tuser = 8 bytes
	length = 320;   // 16 word aligned address


		ba = DAC_DDR_BASEADDR + (length*2);
		Xil_Out32((ba + 0x000), (ba + 0x40)); // next descriptor
		Xil_Out32((ba + 0x004), 0x00); // reserved
		Xil_Out32((ba + 0x008), DAC_DDR_BASEADDR); // start address
		Xil_Out32((ba + 0x00c), 0x00); // reserved
		Xil_Out32((ba + 0x010), 0x00); // reserved
		Xil_Out32((ba + 0x014), 0x00); // reserved
		Xil_Out32((ba + 0x018), (length) | XAXIDMA_BD_CTRL_TXSOF_MASK ); // no. of bytes + TXSOF
		Xil_Out32((ba + 0x01c), 0x00); // status

		Xil_Out32((ba + 0x040), (ba + 0x00)); // next descriptor
		Xil_Out32((ba + 0x044), 0x00); // reserved
		Xil_Out32((ba + 0x048), DAC_DDR_BASEADDR); // start address
		Xil_Out32((ba + 0x04c), 0x00); // reserved
		Xil_Out32((ba + 0x050), 0x00); // reserved
		Xil_Out32((ba + 0x054), 0x00); // reserved
		Xil_Out32((ba + 0x058), (length)| XAXIDMA_BD_CTRL_TXEOF_MASK); // no. of bytes + TXEOF
		Xil_Out32((ba + 0x05c), 0x00); // status
		Xil_DCacheFlush();

		srio_tx_axidma_write(XAXIDMA_CDESC_OFFSET, ba); // Current descriptor pointer
		srio_tx_axidma_write(XAXIDMA_CR_OFFSET, XAXIDMA_CR_RUNSTOP_MASK | XAXIDMA_CR_CYCLIC_MASK); // Start DMA channel + cyclic bit
//		srio_tx_axidma_write(XAXIDMA_TDESC_OFFSET, (ba+0x40)); // Tail descriptor pointer
		srio_tx_axidma_write(XAXIDMA_TDESC_OFFSET, 0x50); // Tail descriptor pointer (dummy address if in cyclic mode)

		sleep(1);
}









void srio_rx_axidma_write(uint32_t regAddr, uint32_t data)
{
	Xil_Out32(XPAR_SRIO_DMA_BASEADDR + XAXIDMA_RX_OFFSET + regAddr, data);
}

void reset_sriodmarx()
{
	srio_rx_axidma_write(XAXIDMA_CR_OFFSET, XAXIDMA_CR_RESET_MASK); // Reset DMA engine
	srio_rx_axidma_write(XAXIDMA_CR_OFFSET, 0);
}


/***************************************************************************//**
 * @brief adc_capture
*******************************************************************************/

void srio_rx_axidma(uint32_t start_address, int timeout)
{
	uint32_t status;
	uint32_t ba;
	uint32_t length;

	length = 0x840;


	ba = start_address + (length*2);
	Xil_Out32((ba + 0x000), (ba + 0x40)); // next descriptor
	Xil_Out32((ba + 0x004), 0x00); // reserved
	Xil_Out32((ba + 0x008), start_address); // start address
	Xil_Out32((ba + 0x00c), 0x00); // reserved
	Xil_Out32((ba + 0x010), 0x00); // reserved
	Xil_Out32((ba + 0x014), 0x00); // reserved
	Xil_Out32((ba + 0x018), (length)); // no. of bytes
	Xil_Out32((ba + 0x01c), 0x00); // status
	Xil_Out32((ba + 0x040), (ba + 0x00)); // next descriptor
	Xil_Out32((ba + 0x044), 0x00); // reserved
	Xil_Out32((ba + 0x048), start_address); // start address
	Xil_Out32((ba + 0x04c), 0x00); // reserved
	Xil_Out32((ba + 0x050), 0x00); // reserved
	Xil_Out32((ba + 0x054), 0x00); // reserved
	Xil_Out32((ba + 0x058), length); // no. of bytes
	Xil_Out32((ba + 0x05c), 0x00); // status
	Xil_DCacheFlush();

//	srio_rx_axidma_write(XAXIDMA_CR_OFFSET, XAXIDMA_CR_RESET_MASK); // Reset DMA engine
//	srio_rx_axidma_write(XAXIDMA_CR_OFFSET, 0);
	srio_rx_axidma_write(XAXIDMA_CDESC_OFFSET, ba); // Current descriptor pointer
	srio_rx_axidma_write(XAXIDMA_CR_OFFSET, XAXIDMA_CR_RUNSTOP_MASK); // Start DMA channel
	srio_rx_axidma_write(XAXIDMA_TDESC_OFFSET, (ba+0x40)); // Tail descriptor pointer

	sleep(timeout);
}

