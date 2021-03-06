#ifndef TIMING_PLACE_LOOKUP_H
#define TIMING_PLACE_LOOKUP_H

#include "vpr_types.h"

/*indicator of an array location that should never be accessed*/
#define IMPOSSIBLE -1 

void alloc_and_compute_delay_lookup_matrixes(router_opts_t router_opts,
                                 detail_routing_arch_t det_routing_arch,
                                 segment_info_t* segment_inf,
                                 timing_info_t timing_inf,
                                 chan_width_distr_t chan_width_dist,
                                 subblock_data_t subblock_data);

void free_place_lookup_structs(void);

extern double** delta_inpad_to_clb;
extern double** delta_clb_to_clb;
extern double** delta_clb_to_outpad;
extern double** delta_inpad_to_outpad;

#endif

