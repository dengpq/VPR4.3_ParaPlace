#ifndef PLACE_H
#define PLACE_H

#include "vpr_types.h"

/* the purpose of start_finish_nets was to evenly divide up the work allocated *
 * to each thread for:
 * compute_nets_slacks_parallel();
 * compute_timing_driven_cost_parallel();
 * compute_bb_cost_parallel();
 *
 * allocateing an equal number of nets per partition(or region) doesn't work since
 * the inner loop of each of the functions depend on other parameters. num_of_edges
 * connect to each net or num_of_sinks . *
 *
 * edges_in_this_partition and sinks_in_this_partition will eventually balance out *
 * for each thread as program progress to provide an equal distribution of work . */
struct  start_finish_nets {
    int  m_start_edge;
    int  m_finish_edge;
    int  m_start_sinks;
    int  m_finish_sinks;

    int  m_edge_partition_size;
    int  m_sink_partition_size;
    int  m_counter_edge;
    int  m_counter_sink;

    unsigned long m_edges_in_this_partition;
    unsigned long m_sinks_in_this_partition;
} start_finish_nets[NUM_OF_THREADS]  __attribute__ ((aligned(64)));


typedef struct s_local_block {
    short m_x; /* grid_column */
    short m_y; /* grid_row */
    short m_z; /* grid_capacity */
} local_block_t;

/* designed for 64-bit cachelines */
typedef struct aligned_bar {
    int volatile m_arrived;
    int m_entry;
    int volatile m_proceed;
} __attribute__((aligned(64)))  aligned_bar_t;


typedef struct aligned_neighbor_bar {
    int volatile m_arrived[4];
} __attribute__((aligned(64))) aligned_neighbor_bar_t;

/* x_[start|end] was the [starting|ending] grid column of this region, y_[start|end] *
 * was the [starting|ending] row of this thread_region. */
typedef struct s_region_boundary {
    int  m_x_start;
    int  m_x_end;
    int  m_y_start;
    int  m_y_end;
} region_boudary_t;

typedef struct pthread_data {
    int      m_thread_id;
    region_boudary_t  m_boundary;

    boolean  m_fixed_pins;
    double*  m_temper; /* temperature */

    placer_opts_t      m_placer_opts;
    annealing_sched_t  m_annealing_sched;
    int*     m_move_limit;
    int*     m_success_sum;
    int*     m_total_iter;
    double*  m_success_ratio; /* ratio = success_sum / move_limit */
    int*     m_inner_iter_num;

    double** m_net_slack;
    double** m_net_delay;

    double*  m_av_cost;
    double*  m_av_bb_cost;
    double*  m_av_timing_cost;
    double*  m_av_delay_cost;

    double*  m_total_cost;
    double*  m_bb_cost;
    double*  m_timing_cost;
    double*  m_inverse_prev_bb_cost;
    double*  m_inverse_prev_timing_cost;

    double*  m_delay_cost;
    int*     m_num_connections;
    /* place_delay_value = delay_cost / num_connections */
    double*  m_place_delay_value;
    double*  m_max_delay;

    double*  m_sum_of_squares; /* sum_of_squares = total_cost * total_cost */
    double*  m_std_dev;

    int*     m_exit;
    double*  m_crit_exponent;
    double*  m_range_limit;

    int*     m_current_row;
    int*     m_current_row2;
} __attribute__((aligned(64))) pthread_data_t;


void try_place(const char* netlist_file,
               placer_opts_t placer_opts_ptr,
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
                timing_info_t timing_inf,
                subblock_data_t* subblock_data_ptr);


#endif

