#ifndef ROUTE_EXPORT_H
#define ROUTE_EXPORT_H

#include "globals.h"
#include "vpr_types.h"

/******** Function prototypes for functions in route_common.c that ***********
 ******** are used outside the router modules.                     ***********/

/* FIXME: main function about PathFinder-based Router */
boolean try_route(int width_fac, struct s_router_opts router_opts,
                  struct s_det_routing_arch det_routing_arch,
                  t_segment_inf* segment_inf, t_timing_inf timing_inf,
                  double** net_slack, double** net_delay,
                  t_chan_width_dist  chan_width_dist,
                  t_ivec**  clb_opins_used_locally);

boolean feasible_routing(void);

t_ivec** alloc_route_structs(t_subblock_data subblock_data);

void free_route_structs(t_ivec** clb_opins_used_locally);

struct s_trace** alloc_saved_routing(t_ivec** clb_opins_used_locally,
                                     t_ivec** *saved_clb_opins_used_locally_ptr);

void free_saved_routing(struct s_trace** best_routing,
                        t_ivec** saved_clb_opins_used_locally);

void save_routing(struct s_trace** best_routing,
                  t_ivec** clb_opins_used_locally,
                  t_ivec** saved_clb_opins_used_locally);

void restore_routing(struct s_trace** best_routing,
                     t_ivec** clb_opins_used_locally,
                     t_ivec** saved_clb_opins_used_locally);

void get_serial_num(void);

void print_route(char* name);

#endif

