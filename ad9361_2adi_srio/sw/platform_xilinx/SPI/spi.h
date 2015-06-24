
#ifndef _SPI_H_
#define _SPI_H_

#include "xbasic_types.h"
#include <stdint.h>

//void HAL_SPIMenuTitle (void);
//void HAL_SPIMenuOptions (void);
//void HAL_SPIChoose (void);
//void HAL_SPIWriteByte (u16 addr, u16 data);
//void HAL_SPIReadByte (u16 addr, u16 *readdata);
//void HAL_SPIExchangeByte(u8 *data);


extern int32_t spi_ss;

void HAL_SPIInit();
void HAL_soft_spi_transfer(u8 *SendBufPtr,u8 *RecvBufPtr, unsigned ByteCount);


#endif
