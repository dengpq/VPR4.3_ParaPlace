#ifndef TIMING_PLACE_LOOKUP_H
#define TIMING_PLACE_LOOKUP_H

#include "vpr_types.h"

/*indicator of an array location that should never be accessed*/
#define IMPOSSIBLE -1 

void compute_delay_lookup_tables(struct s_router_opts router_opts,
                                 struct s_det_routing_arch det_routing_arch,
                                 t_segment_inf* segment_inf,
                                 t_timing_inf timing_inf,
                                 t_chan_width_dist chan_width_dist,
                                 t_subblock_data subblock_data);

void free_place_lookup_structs(void);

extern double** delta_inpad_to_clb;
extern double** delta_clb_to_clb;
extern double** delta_clb_to_outpad;
extern double** delta_inpad_to_outpad;

#endif

