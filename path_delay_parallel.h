#ifndef PATH_DELAY_PARALLEL_H
#define PATH_DELAY_PARALLEL_H

#include "vpr_types.h"

unsigned long load_timing_graph_net_delays_parallel(double** net_delay,
                                                    int start,
                                                    int finish);

double calc_tnodes_arr_time_parallel(int start_node,
                                    int finish_node,
                                    int ilevel);

void  calc_tnodes_req_time_parallel(double T_cycle,
                                    int start_node,
                                    int finish_node,
                                    int ilevel);

unsigned long compute_net_slacks_parallel(double** net_slack,
                                          int start_net,
                                          int finish_net);

#endif

