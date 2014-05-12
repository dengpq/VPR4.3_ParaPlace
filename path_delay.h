#ifndef PATH_DELAY_H
#define PATH_DELAY_H

#include "vpr_types.h"

/* Array for mapping from a PIN(on a blocks) to a TNODE index. For pads, only *
 * the first two pin locations are used(INPUT of pad is first, output of pad *
 * pad is second). For CLBs, all OPEN pins on the CLB have their mapping set *
 * to OPEN so I won't use it by mistake.                                     */
int** block_pin_to_tnode; /* [0..num_blocks-1][0..pins_per_clb-1] */

/* Array for mapping from a pin(on a subblock) to a TNODE index. Unused or *
 * non-existent subblock pins have their mapping set to OPEN.             *
 * [0..num_blocks-1][0..num_subblocks_per_block-1][0..subblock_lut_size+1]*/
int*** sblk_pin_to_tnode;


/* This function create the Timing_Analyze_Graph(It's a DAG) for TDP. But VPR  *
 * didn't define any data structure about DAG. It only defined vertex_t* vertexes; *
 * [0...num_of_vertexs-1] and  edge_t*                                         */
void alloc_and_load_timing_graph(placer_opts_t placer_opts,
                                 timing_info_t timing_inf,
                                 subblock_data_t subblock_data);

t_linked_int* allocate_and_load_critical_path(void);

void load_timing_graph_net_delays(double** net_delay);

/* FIXME: calc_all_vertexs_arr_req_time() calcualte all pins of blocks arr_time, *
 * req_time, and all subnets connect to pins. The slack value stored in           *
 * double** net_slack           */
double** alloc_net_slack(void);

double calc_all_vertexs_arr_req_time(double target_cycle_time);

void compute_net_slacks(double** net_slack);

void free_timing_graph(double** net_slack);

void free_subblock_data(subblock_data_t* subblock_data_ptr);

void print_timing_graph(char* fname);

void print_net_slack(char* fname, double** net_slack);

void print_critical_path(char* fname);

void get_tnode_block_and_output_net(int ivex,
                                    int* iblk_ptr,
                                    int* inet_ptr);

void do_constant_net_delay_timing_analysis(placer_opts_t   placer_opts,
                                           timing_info_t   timing_inf,
                                           subblock_data_t subblock_data,
                                           double constant_net_delay_value);

/* [0...num_nets-1][1...num_pins-1] */
extern double** subnet_local_crit_weight;

extern double* front_crit_path_through_pin;  /* [0...num_pins-1] */
extern double* behind_crit_path_through_pin;

/**********************************************************
 *  double** subnet_local_crit_weight;    [ipin][inet]    *
 *********************************************************/
void compute_all_nets_local_crit_weight(double** net_slack,
                                        int**  block_pin_to_tnode,
                                        const double crit_delay);

/* calc_all_nets_local_crit_weight() will call following 2 functions */
int locate_source_vertex_index_by_net_index(const int inet);

int find_sink_vertex_index_by_net_and_pin_index(const int inet,
                                              const int ipin,
                                              int** block_pin_to_tnode);

void check_block_pin_to_tnode(int** block_pin_to_tnode);

void check_vertexs_levels(void);

#endif

