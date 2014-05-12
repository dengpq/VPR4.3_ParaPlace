#ifndef TIMING_PLACE_H
#define TIMING_PLACE_H

#include "vpr_types.h"

/* alloccate 4 delay_lookup_matrixes: delta_clb_to_clb, delta_inpad_to_clb,  *
 * delta_clb_to_outpad, delta_inpad_to_outpad. They are needed for calculate *
 * delay. And This function also allocate timing_place_crit, which store the *
 * weight of all subnets(or connections).                                    */
void alloc_delay_lookup_matrixes_and_criticalities(placer_opts_t   placer_opts,
                                                   subblock_data_t subblock_data,
                                                   chan_width_distr_t chan_width_dist,
                                                   timing_info_t   timing_inf,
                                                   router_opts_t   router_opts,
                                                   detail_routing_arch_t det_routing_arch,
                                                   segment_info_t* segment_inf,
                                                   double***  net_delay);

/* calculate all tedges' timing_criticality depend on its slack, critical_delay *
 * and crit_exponent                                                            */
void load_criticalities(double** net_slack,
                        double   crit_delay,
                        double   crit_exponent);

void free_lookups_and_criticalities(const placer_opts_t* placer_opts_ptr,
                                    double*** net_delay,
                                    double*** net_slack);

void print_sink_delays(char* fname);

extern double** timing_place_crit;

#endif

