#ifndef READ_ARCH_H
#define READ_ARCH_H

#include "vpr_types.h"

void read_arch(char*   arch_file,
               router_types_t   route_type,
               detail_routing_arch_t*  det_routing_arch,
               segment_info_t**  segment_inf_ptr,
               timing_info_t*    timing_inf_ptr,
               subblock_data_t*  subblock_data_ptr,
               chan_width_distr_t* chan_width_dist_ptr);

void init_arch(double aspect_ratio,
               boolean user_sized);

void print_arch(char* arch_file,
                router_types_t route_type,
                detail_routing_arch_t  det_routing_arch,
                segment_info_t*  segment_inf,
                timing_info_t    timing_inf,
                subblock_data_t  subblock_data,
                chan_width_distr_t chan_width_dist);

#endif

