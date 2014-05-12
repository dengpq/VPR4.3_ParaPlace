#ifndef PLACE_AND_ROUTE_H
#define PLACE_AND_ROUTE_H

#include "vpr_types.h"

void place_and_route(operation_types_t operation,
                     placer_opts_t placer_opts,
                     char* place_file, char* netlist_file,
                     char* arch_file, char* route_file,
                     boolean full_stats, boolean verify_binary_search,
                     annealing_sched_t annealing_sched,
                     router_opts_t router_opts,
                     detail_routing_arch_t det_routing_arch,
                     segment_info_t* segment_inf, timing_info_t timing_inf,
                     subblock_data_t* subblock_data_ptr,
                     chan_width_distr_t chan_width_dist);

void init_channel_t(int cfactor,
                    chan_width_distr_t chan_width_dist);

#endif

