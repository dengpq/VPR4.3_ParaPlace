#ifndef RR_GRAPH_H
#define RR_GRAPH_H

#include "vpr_types.h"

void build_rr_graph(router_types_t route_type,
                    detail_routing_arch_t det_routing_arch,
                    segment_info_t* segment_inf,
                    timing_info_t   timing_inf,
                    router_base_cost_t router_base_cost_type);

void free_rr_graph_internals(router_types_t route_type,
                             detail_routing_arch_t det_routing_arch,
                             segment_info_t* segment_inf,
                             timing_info_t timing_inf,
                             router_base_cost_t router_base_cost_type);

void free_rr_graph(void);

void dump_rr_graph(char* file_name);                /* For debugging only */

void prinrr_node_t(FILE* fp, int ivex);            /* For debugging only */

void prinrr_indexed_data_t(FILE* fp, int index);    /* For debugging only */

void load_net_rr_terminals(int** rr_node_indices,
                           int nodes_per_chan);

int** gerr_node_t_indices(void);

int get_nodes_per_chan(void);

#endif

