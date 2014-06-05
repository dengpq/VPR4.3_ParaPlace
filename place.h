#ifndef PLACE_H
#define PLACE_H

#include "vpr_types.h"

/* For comp_cost.  NORMAL means use the method that generates updateable  *
 * bounding boxes for speed.  CHECK means compute all bounding boxes from *
 * scratch using a very simple routine to allow checks of the other       *
 * costs.                                                                 */
enum cost_methods {
    NORMAL,
    CHECK
};

struct s_pos {
    int x;
    int y;
    int z;
}* pos;

/* This functions was used for placement using single-thread */
void try_place(const char*  netlist_file,
               const placer_opts_t*    placer_opts_ptr,
               const annealing_sched_t annealing_sched,
               chan_width_distr_t chan_width_dist,
               router_opts_t  router_opts,
               detail_routing_arch_t det_routing_arch,
               segment_info_t* segment_inf,
               timing_info_t   timing_inf,
               subblock_data_t* subblock_data_ptr);

void read_place(char* place_file,
                char* net_file,
                char* arch_file,
                placer_opts_t placer_opts_ptr,
                router_opts_t router_opts,
                chan_width_distr_t chan_width_dist,
                detail_routing_arch_t det_routing_arch,
                segment_info_t* segment_inf,
                timing_info_t   timing_inf,
                subblock_data_t* subblock_data_ptr);

#endif

