#ifndef  PLACE_PARALLEL_H
#define  PLACE_PARALLEL_H

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
} __attribute__((aligned(64))) start_finish_nets[NUM_OF_THREADS];


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

    int      m_x_start;
    int      m_x_end;
    int      m_y_start;
    int      m_y_end;

    placer_opts_t      m_placer_opts;
    annealing_sched_t  m_annealing_sched;

    double**  m_net_slack;
    double**  m_net_delay;
    /* boolean  m_fixed_pins;
    double*  m_temper;
    int*     m_move_limit;
    int*     m_total_iter;
    int*     m_inner_iter_num;

    int*     m_success_sum;
    double*  m_success_ratio;

    int*     m_num_connections;

    double*  m_place_delay_value;

    double*  m_max_delay;

    double*  m_sum_of_squares;
    double*  m_std_dev;

    int*     m_exit;
    double*  m_crit_exponent;
    double*  m_range_limit; total 15 items */
    placer_paras_t*  m_local_place_paras_ptr;

    placer_costs_t*  m_local_place_cost_ptr;

    int*     m_current_row_ptr;
    int*     m_current_row2_ptr;
} __attribute__((aligned(64))) pthread_data_t;

typedef  struct  s_swap {
    int    m_x_to;      /* grid_column */
    int    m_y_to;      /* grid_row */
    int    m_z_to;      /* location */
    int    m_to_block;  /* to_block index number */

    int    m_i;        /*  */
    int    m_k;        /*  */
    int    m_inet;     /*  */
    int    m_keep_switch;
    int    m_num_of_pins;  /* */
    int    m_num_nets_affected;
    int    m_bb_index;

    double m_delta_cost;
    double m_delta_bb_cost;
    double m_delta_timing_cost;
    double m_delta_delay_cost;

    int    m_neighbor_index;
} __attribute__((aligned(64))) swap_t;

typedef struct aligned_mutex {
    pthread_mutex_t  m_mutex;
} __attribute__((aligned(64))) aligned_mutex_t;

aligned_neighbor_bar_t  g_neighbor_bar[NUM_OF_THREADS];
aligned_bar_t           g_barrier1[NUM_OF_THREADS];
aligned_mutex_t         global_data_access;

/* In VPR_ParaPlace, the partial_result was used for timing_cost and bb_cost,
 * partial_results2 was used for delay_cost */
double  g_partial_results[NUM_OF_THREADS];
double  g_partial_results2[NUM_OF_THREADS];
/*     Adding  Parallel Placement Data Structures Ending    */

/*  FIXME, the following was used for Parallel Placement  */
void try_place_by_multi_threads(const char*     netlist_file,
                                placer_opts_t   placer_opts,
                                annealing_sched_t  annealing_sched,
                                chan_width_distr_t chan_width_dist,
                                router_opts_t      router_opts,
                                detail_routing_arch_t  det_routing_arch,
                                segment_info_t*  segment_inf_ptr,
                                timing_info_t    timing_inf,
                                subblock_data_t* subblock_data_ptr);

/* Attention, the followint 2 functions were very important!, they were used for *
 * synchronous for threads.                                                      */
void   barrier_polling(const int thread_id);

void   barrier_polling_reset();

void  balance_two_consecutive_threads_edge(const int thread_id);

void  balance_two_consecutive_threads_sinks(const int thread_id);

void*  try_place_parallel(pthread_data_t*  input_args);

void   paraplace_init_localvert_grid(void);

void   paraplace_init_local(bbox_t* local_bb_edge,
                            bbox_t* local_bb_coord,
                            double* local_temp_net_cost,
                            double* local_net_cost,
                            int*    region_x_boundary,
                            grid_tile_t**  local_grid_ptr,
                            local_block_t* local_block_ptr);

void  paraplace_timing_update_full(const int thread_id,
                                   pthread_data_t* input_args,
                                   double**  net_delay_ptr,
                                   double*   max_delay_ptr);

void  paraplace_compute_net_slack(const int       thread_id,
                                  pthread_data_t* input_args,
                                  double***       net_slack_ptr,
                                  double*         timing_cost_ptr,
                                  double*         delay_cost_ptr,
                                  const double    crit_exponent,
                                  const double    max_delay);

void  paraplace_timing_calculate(const int       thread_id,
                                 pthread_data_t* input_args,
                                 double***       net_slack_ptr,
                                 double*         timing_cost_ptr,
                                 double*         delay_cost_ptr,
                                 const double    crit_exponent,
                                 const double    max_delay);

void paraplace_iter_data_update(const int        thread_id,
                                const int        iter,
                                const placer_opts_t placer_opts,
                                pthread_data_t*  input_args,
                                placer_costs_t*  placer_cost_ptr,
                                int*  region_x_boundary_ptr,
                                int*  region_y_boundary_ptr,
                                grid_tile_t**   local_grid_ptr,
                                local_block_t*  local_block_ptr,
                                bbox_t*         local_bb_edge,
                                bbox_t*         local_bb_coord,
                                double*         local_temp_net_cost,
                                double*         local_net_cost);

void  paraplace_local_data_update(const int row,
                                  const int col,
                                  int*  region_x_boundary_ptr,
                                  int*  region_y_boundary_ptr,
                                  grid_tile_t**  local_grid_ptr,
                                  local_block_t* local_block_ptr);

void  update_from_global_to_local_grid_only(grid_tile_t**  local_grid_ptr,
                                            const int x_start,
                                            const int x_end,
                                            const int y_start,
                                            const int y_end);

void  paraplace_initialize(pthread_data_t*  input_args,
                           int*      thread_id_ptr,
                           int*      y_start_ptr,
                           int*      y_end_ptr,
                           int*      x_start_ptr,
                           int*      x_end_ptr,
                           int*      region_y_boundary_ptr,
                           int*      region_x_boundary_ptr,
                           placer_paras_t*   place_paras,
                           double*** net_slack_ptr,
                           double*** net_delay_ptr,
                           int*      max_pins_per_fb_ptr);

void  paraplace_alloc_memory(const int  max_pins_per_fb,
                             placer_opts_t  placer_opts,
                             bbox_t**  bb_coord_new_ptr,
                             bbox_t**  bb_edge_new_ptr,
                             bbox_t**  local_bb_coord_ptr,
                             bbox_t**  local_bb_edge_ptr,
                             int**     nets_to_update_ptr,
                             int**     net_block_moved_ptr,
                             grid_tile_t***  local_grid_ptr,
                             local_block_t** local_block_ptr,
                             double**  local_temp_net_cost_ptr,
                             double**  local_net_cost_ptr,
                             double*** local_temp_point_to_point_delay_cost_ptr,
                             double*** local_temp_point_to_point_timing_cost_ptr);

void  get_bb_from_scratch_parallel(const int inet,
                                   bbox_t*  coords_ptr,
                                   bbox_t*  num_on_edges_ptr,
                                   local_block_t*  local_block_ptr);

void get_non_updateable_bb_parallel(const int inet,
                                    bbox_t*  bb_coord_new_ptr,
                                    local_block_t*  local_block_ptr);

void  update_bb_parallel(const int inet,
                         const int  x_old,
                         const int  y_old,
                         const int  x_new,
                         const int  y_new,
                         bbox_t*   local_bb_coord_ptr,
                         bbox_t*   local_bb_edge_ptr,
                         bbox_t*   bb_coord_new_ptr,
                         bbox_t*   bb_edge_new_ptr,
                         local_block_t*  local_block_ptr);

boolean find_to_block_parallel(const int thread_id,
                               const int x_from,
                               const int y_from,
                               block_type_ptr block_type,
                               int*  x_to_ptr,
                               int*  y_to_ptr,
                               const int  x_min,
                               const int  x_max,
                               const int  y_min,
                               const int  y_max);

int  find_affected_nets_parallel(const int from_block,
                                 const int to_block,
                                 const int num_of_pins,
                                 int*  nets_to_update_ptr,
                                 int*  net_block_moved_ptr,
                                 double* local_temp_net_cost_ptr);

int  assess_swap_parallel(const double delta_cost,
                          const double temper,
                          const int  local_seed);

void  update_temperature_parallel(placer_opts_t*  placer_opts_ptr,
                                  placer_paras_t* place_paras_ptr);

int   try_swap_parallel(const int thread_id,
                        const int from_block,
                        const int x_from,
                        const int y_from,
                        const int z_from,
                        placer_opts_t*   placer_opts_ptr,
                        placer_paras_t*  place_paras_ptr,
                        grid_tile_t**  local_grid_ptr,
                        local_block_t* local_block_ptr,
                        double*  local_temp_net_cost_ptr,
                        double*  local_net_cost_ptr,
                        bbox_t*  local_bb_coord_ptr,
                        bbox_t*  local_bb_edge_ptr,
                        int*     nets_to_update_ptr,
                        int*     net_block_moved_ptr,
                        double** local_temp_point_to_point_timing_cost_ptr,
                        double** local_temp_point_to_point_delay_cost_ptr,
                        bbox_t*  bb_coord_new_ptr,
                        bbox_t*  bb_edge_new_ptr,
                        int  x_min,
                        int  x_max,
                        int  y_min,
                        int  y_max);

double  compute_bb_cost_parallel(const int start_net,
                                 const int end_net);

double comp_td_point_to_point_delay_parallel(int inet,
                                            int ipin,
                                            local_block_t* local_block);
/**********************  End Of Parallel Placement   **************************/
#endif

