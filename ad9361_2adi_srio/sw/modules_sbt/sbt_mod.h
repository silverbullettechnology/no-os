
#ifndef SBT_MOD_H
#define SBT_MOD_H

/***************************** Include Files *******************************/

#include "xbasic_types.h"
#include "xstatus.h"
#include "xil_io.h"
#include "ad9361.h"

/************************** Constant Definitions ***************************/

void reset_dsnk(u32 base_addr);
void reset_dsrc(u32 base_addr);

void enable_adi2axis(u32 base_addr, u32 cnt);

void reset_vita_modules();
void pass_vita_modules();

void reset_vita_trig(u32 base_addr);
void reset_vita_pack(u32 base_addr);
void reset_vita_unpack(u32 base_addr);
void reset_vita_clk(u32 base_addr);

void enable_vita_trig(u32 base_addr);
void enable_vita_pack(u32 base_addr);
void enable_vita_unpack(u32 base_addr);
void enable_vita_clk();

void pass_vita_trig(u32 base_addr);
void pass_vita_pack(u32 base_addr);
void pass_vita_unpack(u32 base_addr);

void reset_vita_assem (u32 base_addr);
void enable_vita_assem (u32 base_addr);
void pass_vita_assem (u32 base_addr);
u32 get_vita_assem_err (int adinum);

void set_vita_trig_on (u32 base_addr, u32 tsi, u32 tsf_hi, u32 tsf_lo);
void set_vita_trig_off (u32 base_addr, u32 tsi, u32 tsf_hi, u32 tsf_lo);
void set_vita_clk (u32 tsi);
u32 get_vita_clk (int clk_num);


void vita_unpack_test (u32 adi_num, u32 stream_id, int length, struct ad9361_rf_phy *phy);
void vita_pack_test_legacy   (struct ad9361_rf_phy *phy, u32 stream_id, u32 pkt_size, u32 words_to_pack);
void vita_pack_test_trig   (struct ad9361_rf_phy *phy, u32 stream_id, u32 pkt_size, u32 words_to_pack);
void vita_endtoend_test (struct ad9361_rf_phy *phy, u32 srio_addr, u32 stream_id, u32 pkt_size, u32 words_to_pack);

#endif /** SBT_MOD_H */
