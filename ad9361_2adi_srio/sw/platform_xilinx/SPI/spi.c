
//#include "cortexM0.h"
#include "spi.h"
//#include "uart.h"

#include "xbasic_types.h"

#include "platform.h"

#include "xparameters.h"	/* SDK generated parameters */
//#include "xspips.h"		/* SPI device driver */
//#include "xgpiops.h"	/*GPIO device driver */
#include "xil_printf.h"
#include "xgpiops.h"

/*
 * The following constants map to the XPAR parameters created in the
 * xparameters.h file. They are defined here such that a user can easily
 * change all the needed parameters in one place.
 */
//#define SPI_DEVICE_ID		XPAR_XSPIPS_1_DEVICE_ID
#define GPIO_DEVICE_ID  	XPAR_XGPIOPS_0_DEVICE_ID

//#define SPI1_GPIO_BANK		XGPIOPS_BANK3

//#define SPI0_GPIO_BANK		XGPIOPS_BANK3

//#define SPI_EMIO_BANK		XGPIOPS_BANK2

#define SPI0_SS0_PIN		(54+39) //TRSS
#define SPI0_SS1_PIN		(54+38) //AFSE
#define SPI0_MOSI_PIN		(54+36)
#define SPI0_MISO_PIN		(54+35)
#define SPI0_CLK_PIN		(54+37)

//#define SPI0_PORT_MASK		0xF0
//#define SPI0_PORT_MASK		0x3F0000  //loopback mask
//#define SPI0_MISO_MASK		0x8
//#define SPI0_MISO_SHIFT		3

#define SPI1_SS0_PIN		(54+23) //ADI1
#define SPI1_SS1_PIN		(54+24) //ADI2
#define SPI1_MOSI_PIN		(54+18) //11 swapped
#define SPI1_MISO_PIN		(54+17) //10 swapped
#define SPI1_CLK_PIN		(54+19)
//#define SPI1_PORT_MASK		0x1E00
//#define SPI1_MISO_MASK		0x100
//#define SPI1_MISO_SHIFT		8
//
//#define EMIO_SS0_PIN		54
//#define EMIO_SS1_PIN		54
//#define EMIO_MOSI_PIN		55
//#define EMIO_MISO_PIN		56
//#define EMIO_CLK_PIN		57
//#define EMIO_PORT_MASK		0xB
//#define EMIO_MISO_MASK		0x4


/*
 * The following constant defines the slave select signal that is used to
 * to select the flash device on the SPI bus, this signal is typically
 * connected to the chip select of the device
 */
//#define ADI_SPI_SELECT	0x01
u8 ADI_Slave_Select = 3;
#define SS_DELAY		1000000
#define LED_DELAY		100

/*
 * The instances to support the device drivers are global such that they
 * are initialized to zero each time the program runs. They could be local
 * but should at least be static so they are zeroed.
 */

//XGpioPs Gpio; //gpio instance

//u8 SSelect_Var = 3;
//u8 Device_Id_Var = 2;

u8 gpio_bank;
u8 ss_pin;
u8 mosi_pin;
u8 miso_pin;
u8 clk_pin;
//u32 port_mask;
//u32 miso_pin_mask;
//u8 miso_shift;

void HAL_soft_spi_transfer(u8 *SendBufPtr,u8 *RecvBufPtr, unsigned ByteCount)
{
	volatile int Delay;
	int i;
	int j;

	u8 current_byte;
	u32 current_recv;

	int active_tx_bit;
	u32 active_rx_bit;


	ss_pin = (spi_ss)? SPI1_SS1_PIN : SPI1_SS0_PIN;

	// select adi
	gpio_data (ss_pin, 0);

	//XGpioPs_WritePin(&Gpio, mosi_pin, 1);
	gpio_data(mosi_pin, 1);   //

//	xil_printf("SS: %d ",ss_pin);
//	xil_printf("MOSI: %d ",mosi_pin);
//	xil_printf("MISO: %d ",miso_pin);
//	xil_printf("CLK: %d ",clk_pin);

	j = 0;
	while(j<ByteCount){
		current_byte = *(SendBufPtr+j);
		//xil_printf("\r\nCurrent Byte: %x\r\n", current_byte);
		current_recv = 0;

		for(i=7;i>=0;i--){

			active_tx_bit = current_byte>>i&0x1;

			//xil_printf("set the bit(%x) on the MOSI line\r\n", active_tx_bit);
			//XGpioPs_WritePin(&Gpio, mosi_pin, active_tx_bit);
			gpio_data(mosi_pin, active_tx_bit);   //

			//xil_printf("set the clock high\r\n");
			//XGpioPs_WritePin(&Gpio, clk_pin, 0x1);
			gpio_data(clk_pin, 1);   //

			//xil_printf("wait clock period\r\n");
			for (Delay = 0; Delay < LED_DELAY; Delay++);

			//xil_printf("read MISO line\r\n");
			//active_rx_bit = XGpioPs_Read(&Gpio, gpio_bank)&miso_pin_mask;
			active_rx_bit = gpio_get_value(miso_pin);
			//xil_printf("\r\nrx_bit:%x\r\n", active_rx_bit);

				//then have to apply some sort of mask or shift on this

			current_recv = current_recv|active_rx_bit<<i;

			//xil_printf("\r\nrx_byte:%x\r\n", current_recv);

			//xil_printf("set clock low\r\n");
//			XGpioPs_WritePin(&Gpio, clk_pin, 0x0);
			gpio_data(clk_pin, 0);   //

			//xil_printf("wait clock period\r\n");
			for (Delay = 0; Delay < LED_DELAY; Delay++);
		}

		//current_recv = current_recv>>miso_shift;
//		xil_printf("\r\nrx_byte:%x\r\n", current_recv);
		*(RecvBufPtr+j) = current_recv;

		j++;
	}


	gpio_data (ss_pin, 1);

}


void HAL_SPIInit()
{
	gpio_direction(SPI1_SS0_PIN, 1);
	gpio_direction(SPI1_SS1_PIN, 1);
	gpio_direction(SPI1_MOSI_PIN, 1);
	gpio_direction(SPI1_MISO_PIN, 0);
	gpio_direction(SPI1_CLK_PIN, 1);

	mosi_pin = SPI1_MOSI_PIN;
	miso_pin = SPI1_MISO_PIN;
	clk_pin = SPI1_CLK_PIN;
//	port_mask = SPI1_PORT_MASK;
//	miso_pin_mask = SPI1_MISO_MASK;
//	miso_shift = SPI1_MISO_SHIFT;

	gpio_data(SPI1_SS0_PIN, 1);   //
	gpio_data(SPI1_SS1_PIN, 1);   //

	xil_printf("HAL_SPIInit\r\n");

}


