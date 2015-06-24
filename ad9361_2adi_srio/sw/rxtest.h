
#ifndef RXTEST_H_
#define RXTEST_H_

#include "xil_cache.h"
#include "parameters.h"

void rxtest_main  (struct ad9361_rf_phy *phy);
void txrxtest_main(struct ad9361_rf_phy *phy, int no_shift);

int  get_eye_rx(struct ad9361_rf_phy *phy, u8 *delay_vec);
void set_eye_rx(struct ad9361_rf_phy *phy, u8 *delay_vec);

int  get_eye_tx(struct ad9361_rf_phy *phy, u8 *delay_vec);
void set_eye_tx(struct ad9361_rf_phy *phy, u8 *delay_vec);

u8 reverse (u8 in);
int CheckRxData_PRBS(void);
int CheckRxData_DMA(int p, int no_shift);


int ShowTxData(void);
int ShowRxData(void);

#endif
