
#ifndef SBT_MOD_H
#define SBT_MOD_H

/***************************** Include Files *******************************/

#include "xbasic_types.h"
#include "xstatus.h"
#include "xil_io.h"

/************************** Constant Definitions ***************************/

void reset_dsnk(u32 base_addr);
void reset_dsrc(u32 base_addr);

void enable_adi2axis(u32 base_addr, u32 cnt);

#endif /** SBT_MOD_H */
