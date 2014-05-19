#ifndef PLACE_H
#define PLACE_H

#include "vpr_types.h"
#include "mst.h"
#include "const.h"

/*The propose of start_finish_nets is to evenly divide up the work allocated
 * to each processor for:
 *  compute_net_slacks_parallel();
 *  comp_td_costs_parallel();
 *  comp_bb_cost_parallel();
 *
 * allocating an equal # of nets per partition does not work since the inner loop
 * of each of the functions depend on other parameters , ie - number of edges
 * connected to each net or number of sinks.
 *
 * edges_in_this_partition and sinks_in_this_partition will eveutally balance
 * out for each thread as program progresses to provide an equal distribution
 * of work.  */

struct start_finish_nets {  /* FIXME, important data structs for timing_driven placement */
    int start_edge, finish_edge, start_sinks, finish_sinks;
    int edge_partition_size, sink_partition_size, counter_edge, counter_sink;
    unsigned long edges_in_this_partition;
    unsigned long sinks_in_this_partition;
} start_finish_nets[NUM_OF_THREADS] __attribute__ ((aligned(64)));

/*local block structure.
 * only x,y,z coordinates are needed since the other attributes do not change
 * ie - the nets connected to this block, or the name */
typedef struct s_local_block {
    /* (x,y) was the coordinate of the grid tile */
    short x;
    short y;
    /* z was the capacity of the grid tile */
    short z;
} local_block_t;

/*designed for 64b cachelines*/
typedef struct aligned_bar {
    int volatile arrived;
    int entry;
    int volatile proceed;
} __attribute__((aligned(64))) aligned_bar_t;

typedef struct aligned_neighbor_bar {
    int volatile arrived[4];
} __attribute__((aligned(64))) aligned_neighbor_bar_t;

/* structure to store local data to each thread */
typedef struct pthread_data {
    int thread_id;  /* Most important! which thread */
    /* x_{start|end} is the {starting|ending} column of this thread_region, *
     * y_{start|end} is the {starting|ending} row of this thread_region.    */
    int y_start, y_end, x_start, x_end;
    boolean fixed_pins;
    double* t;
    placer_opts_t placer_opts;
    struct s_annealing_sched annealing_sched;
    int* success_sum, *move_lim, *tot_iter;
    int* inner_iter_num;
    double* av_cost, *av_bb_cost, *av_timing_cost, *av_delay_cost, *sum_of_squares;
    double* timing_cost, *delay_cost;
    double* inverse_prev_bb_cost;
    double* inverse_prev_timing_cost;
    double* cost, *bb_cost, *success_rat, *std_dev;
    double* max_delay;
    int*   num_connections;
    double* place_delay_value;
    double** net_slack;
    double** net_delay;
    double* crit_exponent;
    int* exit;
    double* range_limit;
} __attribute__((aligned(64))) pthread_data_t;

/* I'd like to use the following struct to replace      *
 * int* region_x_boundary[3]; int* region_y_boundary[3] *
 * , which used to label the 4 boundaries usd for sub   *
 * regions in each region.                              */
typedef struct s_region_x_boundary {
    int x_left;
    int x_middle;
    int x_right;
} region_x_boundary_t;

typedef struct s_region_y_boundary {
    int y_bottom;
    int y_middle;
    int y_top;
} region_y_boundary_t;

/* local data for inside the try_swap loop */
struct swap {
    int x_to, y_to, z_to, to_block;
    int i, k, inet, keep_switch, num_of_pins;
    int num_nets_affected, bb_index;
    double delta_c, bb_delta_c, timing_delta_c, delay_delta_c;
    int iNeighbor;
} __attribute__ ((aligned(64)));

/* specialized mutex to prevent false sharing*/
typedef struct aligned_mutex {
    pthread_mutex_t mutex;
} __attribute__((aligned(64))) aligned_mutex_t;

/* global data declarations */
aligned_neighbor_bar_t  neigh_bar[NUM_OF_THREADS];
aligned_bar_t  barrier1[NUM_OF_THREADS];
aligned_mutex_t  global_data_access;
/* In VPR_ParaPlace, partial_result was used for bb_cost and timing_cost,
 * partial_results was used for delay_cost. */
double partial_results[NUM_OF_THREADS];
double partial_results2[NUM_OF_THREADS];


/*calculate wall-time difference*/
typedef struct {
    int     secs;
    int     usecs;
} TIME_DIFF;


/* this following 4 funtions were used for synchronous for threads */
void barrier_polling(int thread_id);
void barrier_polling_reset();

void try_place(placer_opts_t placer_opts,
               struct s_annealing_sched annealing_sched,
               t_chan_width_dist chan_width_dist,
               struct s_router_opts router_opts,
               struct s_det_routing_arch det_routing_arch,
               t_segment_inf* segment_inf,
               t_timing_inf timing_inf,
               t_subblock_data* subblock_data_ptr,
               t_mst_edge** * mst,
               enum e_operation operation);

void print_grid(void);

void balance_two_consecutive_threads_edge(int thread_id);

void balance_two_consecutive_threads_sinks(int thread_id);

void* try_place_parallel(void* args);

void comp_delta_td_cost_parallel(int from_block,
                                 int to_block,
                                 int num_of_pins,
                                 double* delta_timing,
                                 double* delta_delay,
                                 local_block_t* local_block,
                                 double** local_temp_point_to_point_timing_cost,
                                 double** local_temp_point_to_point_delay_cost);

void tp_init_localvert_grid();

void tp_init_local(bbox_t* local_bb_edge, bbox_t* local_bb_coord,
                   double* local_temp_net_cost, double* local_net_cost,
                   int* region_x_boundary, grid_tile_t** local_grid,
                   local_block_t* local_block);

void tp_timing_update_full(int thread_id,
                           double*** net_delay,
                           pthread_data_t* input,
                           double*  max_delay);

void tp_compute_net_slack(int thread_id,
                          double** *net_slack,
                          double* timing_cost,
                          double* delay_cost,
                          double crit_exponent,
                          double max_delay,
                          pthread_data_t* input);

void tp_timing_calc(int thread_id,
                    double** *net_slack,
                    double* timing_cost,
                    double* delay_cost,
                    double crit_exponent,
                    double max_delay,
                    pthread_data_t* input);

void tp_iter_data_update(int thread_id,
                         double* timing_cost,
                         double* delay_cost,
                         pthread_data_t* input,
                         placer_opts_t placer_opts,
                         double* cost,
                         int iter,
                         int* region_x_boundary, int* region_y_boundary,
                         grid_tile_t** local_grid, local_block_t* local_block,
                         double* inverse_prev_timing_cost,
                         double* bb_cost, double* inverse_prev_bb_cost,
                         bbox_t* local_bb_edge, bbox_t* local_bb_coord,
                         double* local_temp_net_cost, double* local_net_cost);

void tp_local_data_update(int* region_x_boundary, int* region_y_boundary,
                          grid_tile_t** local_grid, local_block_t* local_block,
                          int row, int col);

void update_from_global_to_local_grid_only(grid_tile_t** local_grid,
                                           const int x_start, const int x_end,
                                           const int y_start, const int y_end);

/* try_place_parallel functions */
void tp_initialize(pthread_data_t* input,
                   int* thread_id,
                   int* y_start,
                   int* y_end,
                   int* x_start,
                   int* x_end,
                   boolean* fixed_pins,
                   double* t,
                   placer_opts_t* placer_opts,
                   int* region_y_boundary,
                   int* region_x_boundary,
                   double* first_rlim,
                   double* range_limit,
                   double* final_rlim,
                   double* inverse_delta_rlim,
                   double* place_delay_value,
                   double* max_delay,
                   int* num_connections,
                   double** *net_slack,
                   double** *net_delay,
                   double* delay_cost,
                   int* max_pins_per_fb);

void tp_alloc_mem(int max_pins_per_fb, placer_opts_t placer_opts,
                  bbox_t** bb_coord_new, bbox_t** bb_edge_new,
                  bbox_t** local_bb_coord, bbox_t** local_bb_edge,
                  int** nets_to_update, int** net_block_moved,
                  grid_tile_t** *local_grid, local_block_t** local_block,
                  double** local_temp_net_cost, double** local_net_cost,
                  double** *local_temp_point_to_point_delay_cost,
                  double** *local_temp_point_to_point_timing_cost);

void tp_data_print_to_screen(int thread_id,
                             double* timing_cost,
                             double* delay_cost,
                             double* crit_exponent,
                             double max_delay,
                             pthread_data_t* input,
                             placer_opts_t placer_opts,
                             double* cost,
                             double* bb_cost,
                             double* inverse_prev_bb_cost,
                             bbox_t* local_bb_edge,
                             bbox_t* local_bb_coord,
                             double* local_temp_net_cost,
                             double* local_net_cost,
                             int* success_sum,
                             double* sum_of_squares,
                             int* move_counter,
                             int* tot_iter, double* success_rat,
                             double* av_cost, double* av_bb_cost,
                             double* av_timing_cost,
                             double* av_delay_cost, double* std_dev,
                             double* range_limit,
                             double* final_rlim,
                             double* inverse_delta_rlim,
                             double* t,
                             double* oldt, int* inner_iter_num,
                             double place_delay_value);

void get_bb_from_scratch_parallel(int inet,
                                  bbox_t* coords,
                                  bbox_t* num_on_edges,
                                  local_block_t* local_block);

void get_non_updateable_bb_parallel(int inet,
                                    bbox_t* bb_coord_new,
                                    local_block_t* local_block);

void update_bb_parallel(int inet,
                        bbox_t* local_bb_coord,
                        bbox_t* local_bb_edge,
                        bbox_t* bb_coord_new,
                        bbox_t* bb_edge_new,
                        int xold,
                        int yold,
                        int xnew,
                        int ynew,
                        local_block_t* local_block);

boolean find_to_block_parallel(int x_from,
                               int y_from,
                               t_type_ptr type,
                               int* x_to,
                               int* y_to,
                               int thread_id,
                               int xmin, int xmax,
                               int ymin, int ymax);

int find_affected_nets_parallel(int* nets_to_update,
                                int* net_block_moved,
                                int from_block,
                                int to_block,
                                int num_of_pins,
                                double* local_temp_net_cost);

int assess_swap_parallel(double delta_c,
                         double t,
                         int local_seed);

/* try swap a pair of blocks in each Extend-SubRegion by each thread parallely */
void try_swap_parallel(double t,
                       double* cost,
                       double* bb_cost,
                       double* timing_cost,
                       int place_cost_type,
                       enum e_place_algorithm place_algorithm,
                       double timing_tradeoff,
                       double inverse_prev_bb_cost,
                       double inverse_prev_timing_cost,
                       double* delay_cost,
                       int x_from,
                       int y_from,
                       int z_from,
                       int from_block,
                       grid_tile_t** local_grid,
                       local_block_t* local_block,
                       double* local_temp_net_cost,
                       double* local_net_cost,
                       bbox_t* local_bb_coord,
                       bbox_t* local_bb_edge,
                       int* nets_to_update,
                       int* net_block_moved,
                       int* retval,
                       double** local_temp_point_to_point_timing_cost,
                       double** local_temp_point_to_point_delay_cost,
                       bbox_t* bb_coord_new,
                       bbox_t* bb_edge_new,
                       int thread_id,
                       int xMin, int xMax,
                       int yMin, int yMax,
                       double range_limit);

#if 0
/* Moved to read_place file */
void read_place(char* place_file,
                char* net_file,
                char* arch_file,
                placer_opts_t placer_opts,
                struct s_router_opts router_opts,
                t_chan_width_dist chan_width_dist,
                struct s_det_routing_arch det_routing_arch,
                t_segment_inf* segment_inf,
                t_timing_inf timing_inf,
                t_subblock_data* subblock_data_ptr,
                t_mst_edge** * mst);
#endif

#endif

