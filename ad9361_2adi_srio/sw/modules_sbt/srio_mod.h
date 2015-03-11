
#ifndef SRIO_MOD_H
#define SRIO_MOD_H

/***************************** Include Files *******************************/

#include "xbasic_types.h"
#include "xstatus.h"
#include "xil_io.h"
#include "xllfifo.h"
#include "xiicps.h"

#define WORD_SIZE 4			    /* Size of words in bytes */
#define BEAT_SIZE 2				/* Size of beat in words */
#define MAX_BUFF_SIZE 100		/* Size of buffer in beats */



// SYS_REG register offsets
#define SRIO_CTRL_REG			0x00
	#define SRIO_RESET    0x01
	#define SWRITE_BYPASS 0x02

#define SRIO_STAT_REG			0x04
#define SRIO_TRESP_SRCDEST_REG	0x08
#define SRIO_IREQ_SRCDEST_REG	0x0C
#define SRIO_TREQ_SRCDEST_REG	0x10
#define SRIO_IRESP_SRCDEST_REG	0x14
#define ADI_ADC_SNK_REG			0x18



#define AD0_TO_DDR  0x0
#define AD1_TO_DDR  0x0
#define AD0_TO_SRIO 0x1
#define AD1_TO_SRIO 0x2
#define ADI_TO_DDR  (AD0_TO_DDR  | AD1_TO_DDR)
#define ADI_TO_SRIO (AD0_TO_SRIO | AD1_TO_SRIO)

XLlFifo Fifo_Initiator;
XLlFifo Fifo_Target;
XLlFifo Fifo_UserDef;

XIicPs IIC_Instance;

u32 IRespBuffer[MAX_BUFF_SIZE * BEAT_SIZE];
u32 TReqBuffer [MAX_BUFF_SIZE * BEAT_SIZE];
u32 TUserBuffer [MAX_BUFF_SIZE * BEAT_SIZE];

u8 IIC_Buffer[MAX_BUFF_SIZE * BEAT_SIZE];
u8 IIC_REG[MAX_BUFF_SIZE * BEAT_SIZE];


int XLlFifoInit(XLlFifo *InstancePtr, u16 DeviceId);
int TxSend(XLlFifo *InstancePtr, u32 *pkt, u32 size);
int RxReceive(XLlFifo *InstancePtr, u32 *DestinationAddr);
static int CheckData(u32 *src, u32 *dest, u32 size);


void config_srio_clk();
void reset_srio();
void print_srio_stat();
int init_srio_fifo();

void reset_swrite_mod();
void set_swrite_addr (int adi_path, int txrx, int addr);
void enable_swrite_unpack ();
void enable_swrite_pack ();

void set_swrite_bypass(int en);
void set_adi_adc_snk (int snk);

int msg_test();
int resp_test();
int swrite_tozynq_test();
int swrite_toadi_test();

#endif /** SRIO_MOD_H */
