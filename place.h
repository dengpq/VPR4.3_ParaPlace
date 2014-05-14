#ifndef PLACE_H
#define PLACE_H

#include "vpr_types.h"


/* This functions was used for placement using single-thread */
void try_place(const char* netlist_file,
               placer_opts_t     placer_opts,
               annealing_sched_t annealing_sched,
               chan_width_distr_t chan_width_dist,
               router_opts_t  router_opts,
               detail_routing_arch_t det_routing_arch,
               segment_info_t* segment_inf,
               timing_info_t   timing_inf,
               subblock_data_t* subblock_data_ptr);

void perform_timing_analyze(const placer_opts_t* placer_opts_ptr,
                            const int  num_connections,
                            const double*  bbox_cost,
                            int*   outer_crit_iter_count,
                            double**  net_delay,
                            double**  net_slack,
                            double*  place_delay_value,
                            double*  critical_delay,
                            double*  crit_exponent,
                            double*  timing_cost,
                            double*  delay_cost,
                            double*  inverse_prev_bb_cost,
                            double*  inverse_prev_timing_cost);

void  update_timing_cost_after_swap(const placer_opts_t*  placer_opts_ptr,
                                    const int inner_recompute_limit,
                                    const int inner_iter,
                                    const int move_limit,
                                    const int num_connections,
                                    int*  inner_crit_iter_count,
                                    double**  net_delay,
                                    double**  net_slack,
                                    double*  place_delay_value,
                                    double*  critical_delay,
                                    double*  crit_exponent,
                                    double*  timing_cost_ptr,
                                    double*  delay_cost_ptr);

void compute_timing_driven_cost(const placer_opts_t* placer_opts_ptr,
                                double** net_slack,
                                int**    block_pin_to_tnode,
                                const double critical_delay,
                                const double crit_exponent,
                                double* timing_cost,
                                double* delay_cost);

void  update_place_cost_after_max_move_times(const placer_opts_t* placer_opts_ptr,
                                             double* bbox_cost,
                                             double* new_bb_cost,
                                             double* timing_cost,
                                             double* new_timing_cost,
                                             double* delay_cost,
                                             double* new_delay_cost);

void compute_timing_driven_cost_in_outer_loop(const placer_opts_t* placer_opts_ptr,
                                              const double timing_cost,
                                              const double delay_cost,
                                              double*  new_timing_cost,
                                              double*  new_delay_cost);

void update_place_costs_by_success_sum(const int success_sum,
                                       const double bb_cost,
                                       const double timing_cost,
                                       const double delay_cost,
                                       const double total_cost,
                                       double* av_bb_cost_ptr,
                                       double* av_timing_cost_ptr,
                                       double* av_delay_cost_ptr,
                                       double* av_cost_ptr);

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

placer_paras_t* init_placer_paras(placer_costs_t* placer_costs_ptr);

void  free_placer_paras(placer_paras_t* placer_paras);

#endif

