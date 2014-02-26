#ifndef PATH_DELAY_H
#define PATH_DELAY_H

#include "vpr_types.h"

double** alloc_and_load_timing_graph(t_timing_inf timing_inf,
                                     t_subblock_data subblock_data);

t_linked_int* allocate_and_load_critical_path(void);

void load_timing_graph_net_delays(double** net_delay);

/* FIXME: load_net_slack() calcualte all pins of blocks arr_time, req_time, and all *
 * subnets connect to pins. The slack value stored in double** net_slack           */
double load_net_slack(double** net_slack,
                      double target_cycle_time);

void free_timing_graph(double** net_slack);

void free_subblock_data(t_subblock_data* subblock_data_ptr);

void print_timing_graph(char* fname);

void print_net_slack(char* fname, double** net_slack);

void print_critical_path(char* fname);

void get_tnode_block_and_output_net(int inode, int* iblk_ptr, int* inet_ptr);

void do_constant_net_delay_timing_analysis(t_timing_inf timing_inf,
                                           t_subblock_data subblock_data,
                                           double constant_net_delay_value);

#endif

