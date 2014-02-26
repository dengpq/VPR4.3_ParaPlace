#ifndef STATS_H
#define STATS_H

#include "vpr_types.h"

void routing_stats(boolean full_stats, enum e_route_type route_type,
                   int num_switch, t_segment_inf* segment_inf, int num_segment,
                   double R_minW_nmos, double R_minW_pmos, boolean timing_analysis_enabled,
                   double** net_slack, double** net_delay);

void print_wirelen_prob_dist(void);

void print_lambda(void);

#endif

