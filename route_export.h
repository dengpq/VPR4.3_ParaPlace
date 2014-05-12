#ifndef ROUTE_EXPORT_H
#define ROUTE_EXPORT_H

#include "globals.h"
#include "vpr_types.h"

/******** Function prototypes for functions in route_common.c that ***********
 ******** are used outside the router modules.                     ***********/

/* FIXME: main function about PathFinder-based Router */
boolean try_route(int width_fac, router_opts_t router_opts,
                  detail_routing_arch_t det_routing_arch,
                  segment_info_t* segment_inf, timing_info_t timing_inf,
                  double** net_slack, double** net_delay,
                  chan_width_distr_t  chan_width_dist,
                  vector_t**  clb_opins_used_locally);

boolean feasible_routing(void);

vector_t** alloc_route_structs(subblock_data_t subblock_data);

void free_route_structs(vector_t** clb_opins_used_locally);

struct s_trace** alloc_saved_routing(vector_t** clb_opins_used_locally,
                                     vector_t** *saved_clb_opins_used_locally_ptr);

void free_saved_routing(struct s_trace** best_routing,
                        vector_t** saved_clb_opins_used_locally);

void save_routing(struct s_trace** best_routing,
                  vector_t** clb_opins_used_locally,
                  vector_t** saved_clb_opins_used_locally);

void restore_routing(struct s_trace** best_routing,
                     vector_t** clb_opins_used_locally,
                     vector_t** saved_clb_opins_used_locally);

void get_serial_num(void);

void print_route(char* name);

#endif

