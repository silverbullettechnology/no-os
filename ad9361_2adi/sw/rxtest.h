
#ifndef RXTEST_H_
#define RXTEST_H_

#include "xil_cache.h"
#include "parameters.h"

void rxtest_main  (struct ad9361_rf_phy *phy);
void txrxtest_main(struct ad9361_rf_phy *phy);

u8 reverse (u8 in);
int CheckRxData_PRBS(void);
int CheckRxData_DMA(void);


int ShowTxData(void);
int ShowRxData(void);

#endif
