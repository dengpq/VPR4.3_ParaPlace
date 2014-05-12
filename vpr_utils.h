#ifndef  VPR_UTILS_H
#define  VPR_UTILS_H

#include "vpr_types.h"

boolean is_opin(int ipin);

/* For a clb in netlist, calculate the pins(contained output, input, clock) in *
 * all its owned subblocks.*/
void load_one_clb_fanout_count(int subblock_lut_size, subblock_t *subblock_inf,
                               int num_subblocks, int* num_uses_of_clb_ipin,
                               int* num_uses_of_sblk_opin, int iblk);
#endif

