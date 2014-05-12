#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <string.h>

#include "util.h"
#include "globals.h"
#include "place.h"
#include "read_place.h"
#include "draw.h"
#include "place_and_route.h"
#include "net_delay.h"
#include "path_delay.h"
#include "timing_place_lookup.h"
#include "timing_place.h"
#include "path_delay2.h"

/************** Types and defines local to place.c ***************************/
/* New added data structures for parallel placement */
/* the grid in architecture will be partition into x_partition(columns) X  *
 * y_partition(rows). For 8-threads, I'd like to set 4x2 regions. */ 
int x_partition[64] = {
    1, 1, 1, 2, 1,   /* 1-5   */
    3, 0, 4, 3, 5,   /* 6-10  */
    0, 4, 0, 7, 5,   /* 11-15 */
    4, 0, 6, 0, 5,   /* 16-20 */
    7, 11, 0, 6, 5,  /* 21-25 */
    13, 9, 7, 0, 6,  /* 29-30 */
    7, 6, 0, 19, 7,  /* 35-40 */
    0, 7, 0, 11, 9,  /* 41-45 */
    23, 0, 8, 7, 10, /* 46-50 */
    0, 0, 0, 9, 11,  /* 51-55 */
    8, 0, 0, 0, 10,  /* 56-60 */
    0, 0, 9, 8       /* 61-64 */
};

int y_partition[64] = {
    1, 1, 1, 2, 1,   /* 1-5   */
    2, 0, 2, 3, 2,   /* 6-10  */
    0, 3, 0, 2, 3,   /* 11-15 */
    4, 0, 3, 0, 4,   /* 19-20 */ 
    3, 2, 0, 4, 5,   /* 21-25 */
    2, 3, 4, 0, 5,   /* 29-30 */
    0, 4, 3, 0, 5,   /* 31-35 */
    6, 0, 0, 0, 5,   /* 39-40 */
    0, 6, 0, 4, 5,   /* 41-45 */
    2, 0, 6, 7, 5,   /* 49-50 */
    0, 0, 0, 6, 5,   /* 51-55 */
    7, 0, 0, 0, 6,   /* 59-60 */
    0, 0, 7, 8       /* 61-64 */
};

static void update_from_global_to_local_hori(local_block_t*  local_block_ptr,
                                             grid_tile_t**  local_grid_ptr,
                                             int x_start,
                                             int x_end,
                                             int y_start,
                                             int y_end);

static void update_from_global_to_local_vert(local_block_t* local_block_ptr,
                                             grid_tile_t**  local_grid_ptr,
                                             int x_start,
                                             int x_end,
                                             int y_start,
                                             int y_end);

static void update_from_local_to_global(local_block_t* local_block,
                                        grid_tile_t**  local_grid,
                                        int x_start,
                                        int x_end,
                                        int y_start,
                                        int y_end);
#define SMALL_NET 4    /* Cut off for incremental bounding box updates. */
/* 4 is fastest -- I checked. */


/* For comp_cost.  NORMAL means use the method that generates updateable  *
 * bounding boxes for speed.  CHECK means compute all bounding boxes from *
 * scratch using a very simple routine to allow checks of the other       *
 * costs.                                                                 */

enum cost_methods {
    NORMAL, CHECK
};

#define FROM        0  /* What blocks connected to a net has moved? */
#define TO          1
#define FROM_AND_TO 2

#define ERROR_TOL .001
#define MAX_MOVES_BEFORE_RECOMPUTE 1000000

#define EMPTY -1

/********************** Variables local to place.c ***************************/

/* [0..num_nets-1]  0 if net never connects to the same blocks more than  *
 *  once, otherwise it gives the number of duplicate connections.        */

static int* duplicate_pins;

/* [0..num_nets-1][0..num_unique_blocks-1]  Contains a list of blocks with *
 * no duplicated blocks for ONLY those nets that had duplicates.           */
static int** unique_pin_list;

/* Cost of a net, and a temporary cost of a net used during move assessment. */
static double* net_cost = NULL; /* double new_cost[0..num_nets-1] */
static double* temp_net_cost = NULL; /* double temp_new_cost[0..num_nets-1] */

/* [0..num_nets-1][1..num_pins-1]. What is the value of the timing   */
/* driven portion of the cost function. These arrays will be set to  */
/* (criticality * Tdel) for each point to point connection. */
static double** point_to_point_timing_cost = NULL;
static double** temp_point_to_point_timing_cost = NULL;



/* [0..num_nets-1][1..num_pins-1]. The value of the Tdel */
/* for each connection in the circuit */
static double**  point_to_point_delay_cost = NULL;
static double**  temp_point_to_point_delay_cost = NULL;


/* [0..num_blocks-1][0..pins_per_clb-1]. Indicates which pin on the net */
/* this blocks corresponds to, this is only required during timing-driven */
/* placement. It is used to allow us to update individual connections on */
/* each net. That is <pin_number, net_number>. */
static int** net_pin_index = NULL;


/* [0..num_nets-1], store the bounding_box coordinates of each net. */
static bbox_t* bb_coords = NULL;
/* [0..num_nets-1], store the number of blocks on each of a net's bounding_box *
 * (to allow efficient updates), respectively.                                 */
static bbox_t* bb_num_on_edges = NULL;

/* Stores the maximum and expected occupancies, plus the cost, of each   *
 * region in the placement.  Used only by the NONLINEAR_CONG cost        *
 * function.  [0..num_region-1][0..num_region-1].  Place_region_x and    *
 * y give the situation for the x and y directed channels, respectively. */
static place_region_t** place_region_x, ** place_region_y;

/* Used only with nonlinear congestion.  [0..num_regions]. */
static double* place_region_bounds_x, *place_region_bounds_y;

/* The arrays below are used to precompute the inverse of the average   *
 * number of tracks per channel between [subhigh] and [sublow].  Access *
 * them as chan?_place_cost_fac[subhigh][sublow].  They are used to     *
 * speed up the computation of the cost function that takes the length  *
 * of the net bounding box in each dimension, divided by the average    *
 * number of tracks in that direction; for other cost functions they    *
 * will never be used.                                                  */

static double** chanx_place_cost_fac = NULL;
static double** chany_place_cost_fac = NULL;


/* Expected crossing counts for nets with different #'s of pins.  From *
 * ICCAD 94 pp. 690 - 695 (with linear interpolation applied by me).   */

static const double cross_count[50] = {   /* [0..49] */
    1.0,    1.0,    1.0,    1.0828, 1.1536, 1.2206, 1.2823, 1.3385, 1.3991, 1.4493,
    1.4974, 1.5455, 1.5937, 1.6418, 1.6899, 1.7304, 1.7709, 1.8114, 1.8519, 1.8924,
    1.9288, 1.9652, 2.0015, 2.0379, 2.0743, 2.1061, 2.1379, 2.1698, 2.2016, 2.2334,
    2.2646, 2.2958, 2.3271, 2.3583, 2.3895, 2.4187, 2.4479, 2.4772, 2.5064, 2.5356,
    2.5610, 2.5864, 2.6117, 2.6371, 2.6625, 2.6887, 2.7148, 2.7410, 2.7671, 2.7933
};


/********************* Static subroutines local to place.c *******************/
static void alloc_and_load_unique_pin_list(void);

static void free_unique_pin_list(void);

static void alloc_place_regions(int num_regions);

static void load_place_regions(int num_regions);

static void free_place_regions(int num_regions);

static void alloc_and_load_placement_structs(int place_cost_type,
                                             int num_regions,
                                             double place_cost_exp,
                                             double** *old_region_occ_x,
                                             double** *old_region_occ_y,
                                             placer_opts_t placer_opts);

static void free_placement_structs(int place_cost_type,
                                   int num_regions,
                                   double** old_region_occ_x,
                                   double** old_region_occ_y,
                                   placer_opts_t placer_opts);

static void alloc_and_load_for_fast_cost_update(double place_cost_exp);

static void initial_placement(pad_loc_t pad_loc_type,
                              char* pad_loc_file);

static double comp_bb_cost(int method, int place_cost_type, int num_regions);

static int try_swap(const double t,
                    const double rlim,
                    const int* pins_on_block,
                    const int  place_cost_type,
                    double** old_region_occ_x,
                    double** old_region_occ_y,
                    const int  num_regions,
                    const boolean fixed_pins,
                    const place_algorithm_t place_algorithm,
                    const double timing_tradeoff,
                    const double inverse_prev_bb_cost,
                    const double inverse_prev_timing_cost,
                    double*  bb_cost_ptr,
                    double*  timing_cost_ptr,
                    double*  delay_cost_ptr,
                    double*  total_cost_ptr);

static void check_place(double bb_cost,
                        double timing_cost,
                        int place_cost_type,
                        int num_regions,
                        place_algorithm_t place_algorithm,
                        double delay_cost);

static double starting_temperature(const int* pins_on_block,
                                   const int  place_cost_type,
                                   double** old_region_occ_x,
                                   double** old_region_occ_y,
                                   const int  num_regions,
                                   const boolean fixed_pins,
                                   const annealing_sched_t annealing_sched,
                                   const int max_moves,
                                   const double rlim,
                                   const place_algorithm_t place_algorithm,
                                   const double timing_tradeoff,
                                   const double inverse_prev_bb_cost,
                                   const double inverse_prev_timing_cost,
                                   double*  bb_cost_ptr,
                                   double*  timing_cost_ptr,
                                   double*  delay_cost_ptr,
                                   double*  total_cost_ptr);

static void update_temperature(double* t,
                               double rlim,
                               double success_ratio,
                               annealing_sched_t annealing_sched);

static void update_range_limit(double* rlim,
                               double success_ratio);


static double update_crit_exponent(const placer_opts_t* placer_opts_ptr,
                                   const double rlim,
                                   const double final_rlim,
                                   const double inverse_delta_rlim);

static int exit_crit(double t,
                     double cost,
                     annealing_sched_t annealing_sched);


static int count_connections(void);

static void compute_net_pin_index_values(void);

static double get_std_dev(int n,
                          double sum_x_squared,
                          double av_x);

static void free_fast_cost_update_structs(void);

static double recompute_bb_cost(int place_cost_type, int num_regions);

/********************************************************************
 * The following functions were used for Timing-Driven-Placement  */
static double compute_point_to_point_delay(int inet,
                                           int ipin);

static void update_timing_driven_cost(int from_block,
                                      int to_block,
                                      int num_of_pins);

static void compute_delta_timing_driven_cost(place_algorithm_t place_algo,
                                             int  from_block,
                                             int  to_block,
                                             int  num_of_pins,
                                             double* delta_timing,
                                             double* delta_delay);

static void compute_timing_driven_cost_by_orig_algo(double* timing_cost,
                                                    double* connection_delay_sum);

/*======    New added by Pengqiu Deng for Timing-Driven Placement   ======*/
extern int num_of_vertexs;
extern vertex_t*  vertexes;
extern int* driver_node_index_of_net;

static void compute_timing_driven_costs_by_path_algo(double* total_timing_cost,
                                                     double* connection_delay_sum);
/********************     End!     ***********************************/

static int assess_swap(double delta_cost, double t);

static void find_to(int x_from, int y_from, int type, double rlim,
                    int* x_to, int* y_to);

static void get_non_updateable_bb(int inet, bbox_t* bb_coord_new);

static void update_bb(int inet, bbox_t* bb_coord_new, bbox_t
                      *bb_edge_new, int xold, int yold, int xnew, int ynew);

static int find_affected_nets(int* nets_to_update, int* net_block_moved,
                              int from_block, int to_block, int num_of_pins);

static double get_net_cost(int inet, bbox_t* bb_ptr);

static double nonlinear_cong_cost(int num_regions);

static void update_region_occ(int inet, bbox_t* coords,
                              int add_or_sub, int num_regions);

static void save_region_occ(double** old_region_occ_x,
                            double** old_region_occ_y, int num_regions);

static void restore_region_occ(double** old_region_occ_x,
                               double** old_region_occ_y, int num_regions);

static void get_bb_from_scratch(int inet, bbox_t* coords,
                                bbox_t* num_on_edges);


/* Very Important! */
/* Does almost all the work of placing a circuit. Width_fac gives the width of*
 * the widest channel. Place_cost_exp says what exponent the width should be  *
 * taken to when calculating costs. This allows a greater bias for anisotropic*
 * architectures. Place_cost_type determines which cost function is used.     *
 * num_regions is used only the place_cost_type is NONLINEAR_CONG.            */
void try_place(const char*         netlist_file,
               placer_opts_t       placer_opts,
               annealing_sched_t   annealing_sched,
               chan_width_distr_t  chan_width_dist,
               router_opts_t       router_opts,
               detail_routing_arch_t det_routing_arch,
               segment_info_t*   segment_inf,
               timing_info_t     timing_inf,
               subblock_data_t*  subblock_data_ptr)
{
    /* FIXME, according to "Timing-Driven Placement for FPGAs", before T-VPlace *
     * run, it allocate the Tdel lookup matrix for path-timing-driven placement */
    double** net_slack = NULL; /* FIXME */
    double** net_delay = NULL; /* FIXME */
    double** remember_net_delay_original_ptr = NULL; /*used to free net_delay if it is re-assigned*/
    double place_delay_value = 0.0;
    if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
          || placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE
          /* new added for support PATH Timing-Driven Placement */
          || placer_opts.place_algorithm == NEW_TIMING_DRIVEN_PLACE
          || placer_opts.enable_timing_computations) {
        /*do this before the initial placement to avoid messing up the initial placement */
        alloc_and_load_timing_graph(placer_opts,
                                    timing_inf,
                                    *subblock_data_ptr);
        net_slack = alloc_net_slack();
        assert(net_slack != NULL);

        alloc_delay_lookup_matrixes_and_criticalities(placer_opts,
                                                      *subblock_data_ptr,
                                                      chan_width_dist,
                                                      timing_inf,
                                                      router_opts,
                                                      det_routing_arch,
                                                      segment_inf,
                                                      &net_delay);
        remember_net_delay_original_ptr = net_delay;
    } /* end of create data structures(timing_graph) needed by TDP */

    int width_fac = placer_opts.place_chan_width;

    boolean fixed_pins = FALSE;
    if (placer_opts.pad_loc_type == FREE) { /* the io pad can place freely */
        fixed_pins = FALSE;
    } else {
        fixed_pins = TRUE;
    }

    init_channel_t(width_fac,
                   chan_width_dist); /* initial channel width */

    double**  old_region_occ_x = NULL;
    double**  old_region_occ_y = NULL;
    alloc_and_load_placement_structs(placer_opts.place_cost_type,
                                     placer_opts.num_regions,
                                     placer_opts.place_cost_exp,
                                     &old_region_occ_x,
                                     &old_region_occ_y,
                                     placer_opts);

    /* FIXME, run initial_placement, I understand it! */
    initial_placement(placer_opts.pad_loc_type,
                      placer_opts.pad_loc_file);

    init_draw_coords((double)width_fac);

    /* Storing the number of pins on each type of blocks makes the swap routine *
     * slightly more efficient.                                                */
    int pins_on_block[3];  /* 0: CLB, 1: OUTPAD, 2: INPAD */
    pins_on_block[CLB] = pins_per_clb;
    pins_on_block[OUTPAD] = 1;
    pins_on_block[INPAD] = 1;

    /* Gets initial cost and loads bounding boxes. */
    double total_cost = 0.0;
    double timing_cost = 0.0;
    double delay_cost = 0.0;
    double inverse_prev_bb_cost, inverse_prev_timing_cost;

    double crit_exponent = 0.0; /* timing_critical_exponent for TDP */
    double critical_delay = 0.0;
    double success_ratio = 0.0;
    int  num_connections = 0;
    int  inet = -1;
    int  ipin = -1;
    double bb_cost = comp_bb_cost(NORMAL,
                                  placer_opts.place_cost_type,
                                  placer_opts.num_regions);
    /*=========   Now compute initial_cost after initial placement    ========*/
    int outer_crit_iter_count = 0;
    if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
          || placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE
          || placer_opts.place_algorithm == NEW_TIMING_DRIVEN_PLACE) {
        crit_exponent = placer_opts.td_place_exp_first; /*this will be modified when rlim starts to change*/

        compute_net_pin_index_values();

        num_connections = count_connections();
        printf("\nThere are %d point to point connections in this circuit\n\n",
                num_connections);

        if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE) {
            for (inet = 0; inet < num_nets; ++inet) {
                for (ipin = 1; ipin < net[inet].num_pins; ++ipin) {
                    timing_place_crit[inet][ipin] = 0.0; /*dummy crit values*/
                } /* <source, sink> */
            }
            /* first pass gets delay_cost, which is used in criticality computations *
             * in the next call to compute_timing_driven_cost_by_orig_algo.                      */
            compute_timing_driven_cost_by_orig_algo(&timing_cost,
                                                    &delay_cost);

            /* Used for computing criticalities, but why did it subdivide *
             * num_connections? (FIXME)                                   */
            place_delay_value = delay_cost / num_connections;

            /* For NET_TIMING_DRIVEN_PLACE, all subnets of a net had same Tdel  *
             * value. so double net_delay[][] had same value for all connections. */
            load_constant_net_delay(net_delay,
                                    place_delay_value); /* same value */
        } else {
            place_delay_value = 0;
        }

        /* this keeps net_delay up to date with the same values that *
         * the placer is using point_to_point_delay_cost is computed *
         * each time that compute_timing_driven_cost_by_orig_algo is called, and *
         * is also updated after any swap is accepted.               *
         * point_to_point_delay was pin_to_pin_delay.                */
        if (placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE
              || placer_opts.place_algorithm == NEW_TIMING_DRIVEN_PLACE) {
            net_delay = point_to_point_delay_cost; 
        }

        /* Initialize all edges' Tdel(Tdel) in Timing_Analyze_Graph. */
        load_timing_graph_net_delays(net_delay);
        /* Compute each vertexes's req_time and arr_time time, and calculate the max_delay *
         * in critical path.                                                      */
        critical_delay = calc_all_vertexs_arr_req_time(place_delay_value);
        compute_net_slacks(net_slack);

        compute_timing_driven_cost(&placer_opts,
                                   net_slack,
                                   block_pin_to_tnode,
                                   critical_delay,
                                   crit_exponent,
                                   &timing_cost,
                                   &delay_cost);

        /* FIXME:the auto-normalized variables used for compute placement cost. */
        inverse_prev_timing_cost = 1 / timing_cost;
        inverse_prev_bb_cost = 1 / bb_cost;

        /* TODO: How about this variables? */
        outer_crit_iter_count = 1;

        /* our new cost function uses normalized values of bb_cost and timing_cost,*
         * the value of cost will be reset to 1 at each temperature when           *
         * TIMING_DRIVEN_PLACE is true.                                            */
        total_cost = 1.0;
    } else { /*BOUNDING_BOX_PLACE*/
        total_cost = bb_cost;
        timing_cost = 0;
        delay_cost = 0;
        place_delay_value = 0;
        outer_crit_iter_count = 0;
        num_connections = 0;
        critical_delay = 0;
        crit_exponent = 0;
        inverse_prev_timing_cost = 0; /*inverses not used */
        inverse_prev_bb_cost = 0;
    }
    /*-------------    Compute initial placemnt cost end    ---------------- */

    /* move_limit_timis = 10*(num_of_clb+num_of_io)^(4/3), it used to inner loop of VPR */
    /* Sometimes I want to run the router with a random placement. Avoid using *
     * 0 moves to stop division by 0 and 0 length vector problems, by setting  *
     * move_limit to 1 (which is still too small to do any significant optimization).  */
    int move_limit = (int)(annealing_sched.inner_num * pow(num_blocks,
                                                           1.3333));
    if (move_limit <= 0) {
        move_limit = 1;
    }

    int inner_recompute_limit = 0;
    if (placer_opts.inner_loop_recompute_divider != 0) { /* its default value was 0 */
        inner_recompute_limit = (int)(0.5 + (double)move_limit /
                                     (double)placer_opts.inner_loop_recompute_divider);
    } else { /*don't do an inner recompute */
        inner_recompute_limit = move_limit + 1;
    }


    /* rlim was used in timing-driven placement for exponent computation*/
    double rlim = (double)max(num_of_columns,
                              num_of_rows);
    const double first_rlim = rlim;
    const double final_rlim = 1.0;
    double inverse_delta_rlim = 1 / (first_rlim - final_rlim);

    /* FIXME: initial start_temperature */
    double t = starting_temperature(pins_on_block,
                                    placer_opts.place_cost_type,
                                    old_region_occ_x,
                                    old_region_occ_y,
                                    placer_opts.num_regions,
                                    fixed_pins,
                                    annealing_sched,
                                    move_limit,
                                    rlim,
                                    placer_opts.place_algorithm,
                                    placer_opts.timing_tradeoff,
                                    inverse_prev_bb_cost,
                                    inverse_prev_timing_cost,
                                    &bb_cost,
                                    &timing_cost,
                                    &delay_cost,
                                    &total_cost);

    int total_iter = 0;
    int moves_since_cost_recompute = 0;
    printf("Initial Placement Cost: %g bb_cost: %g td_cost: %g delay_cost: %g\n\n",
           total_cost,
           bb_cost,
           timing_cost,
           delay_cost);

#ifndef SPEC
    printf("%11s  %10s %11s  %11s  %11s %11s  %11s %9s %8s  %7s  %7s  %10s  %7s\n",
           "T", "Cost", "Av. BB Cost", "Av. TD Cost", "Av Tot Del", "P_to_P Del",
           "crit_delay", "Ac Rate", "Std Dev", "R limit", "Exp", "Tot. Moves",
           "Alpha");
    printf("%11s  %10s %11s  %11s  %11s %11s  %11s %9s %8s  %7s  %7s  %10s  %7s\n",
           "--------", "----------", "-----------", "-----------", "---------",
           "----------", "-----", "-------", "-------", "-------", "-------",
           "----------", "-----");
#endif
    char msg[BUFSIZE] = "";
    sprintf(msg, "Initial Placement.  Cost: %g  BB Cost: %g  TD Cost %g  Delay Cost: %g "
            "\t critical_delay %g Channel Factor: %d",
            total_cost,
            bb_cost,
            timing_cost,
            delay_cost,
            critical_delay,
            width_fac);
    update_screen(MAJOR, msg, PLACEMENT, FALSE);

    /* outer loop of SA-based Placement */
    double  new_bb_cost = 0.0;
    double  new_timing_cost = 0.0;
    double  new_delay_cost = 0.0;
    int     inner_iter = 0;
    double  av_cost, av_bb_cost, av_timing_cost, av_delay_cost;
    double  sum_of_squares, std_dev;
    double  old_temper = 0;
    int success_sum = 0;
    int inner_crit_iter_count = 0;
    while (exit_crit(t,
                     total_cost,
                     annealing_sched) == 0) { /* FIXME: outer loop of VPR */
        if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
              || placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE
              /* New added for support PATH Timing-Driven Placement */
              || placer_opts.place_algorithm == NEW_TIMING_DRIVEN_PLACE) {
            total_cost = 1.0;
        }

        av_cost = av_bb_cost = av_delay_cost = av_timing_cost = 0.0;
        sum_of_squares = 0.0;
        success_sum = 0;

        /*------   First running timing_analyze before try_swap() in outer loop ------*/
        if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
              || placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE
              /* new added for supporting PATH Timing-Driven Placement */
              || placer_opts.place_algorithm == NEW_TIMING_DRIVEN_PLACE) {
            perform_timing_analyze(&placer_opts,
                                   num_connections,
                                   &bb_cost,
                                   &outer_crit_iter_count,
                                   net_slack,
                                   net_delay,
                                   &place_delay_value,
                                   &critical_delay,
                                   &crit_exponent,
                                   &timing_cost,
                                   &delay_cost,
                                   &inverse_prev_bb_cost,
                                   &inverse_prev_timing_cost);
        } /* end of placement_algorithm == TIMING_DRIVEN_PLACE */
        /*----   Run Timing Analyze before try_swap() in Outer Loop OK! -----*/

        inner_crit_iter_count = 1;
        /* FIXME: move_limit was move_limit_times, it was used for VPR inner loop */
        for (inner_iter = 0; inner_iter < move_limit; ++inner_iter) {
            /* FIXME: try to swap a pair of clbs or io pads randomly */
            if (try_swap(t,
                         rlim,
                         pins_on_block,
                         placer_opts.place_cost_type,
                         old_region_occ_x,
                         old_region_occ_y,
                         placer_opts.num_regions,
                         fixed_pins,
                         placer_opts.place_algorithm,
                         placer_opts.timing_tradeoff,
                         inverse_prev_bb_cost,
                         inverse_prev_timing_cost,
                         &bb_cost,
                         &timing_cost,
                         &delay_cost,
                         &total_cost) == 1) {
                ++success_sum;
                av_cost += total_cost;  /* TODO Attention, this += bb_cost */
                av_bb_cost += bb_cost;  /* TODO */
                av_delay_cost += delay_cost;
                av_timing_cost += timing_cost;  /* TODO */
                sum_of_squares += total_cost * total_cost;
            } /* -------------     end of try_swap() success   -------------*/

            /*--------------   Update Timing-Driven Placement Cost  After swap ------------*/
            if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
                  || placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE
                  /* New added for supporting PATH Timing-Driven Placement */
                  || placer_opts.place_algorithm == NEW_TIMING_DRIVEN_PLACE) {
                update_timing_cost_after_swap(&placer_opts,
                                              inner_recompute_limit,
                                              inner_iter,
                                              move_limit,
                                              num_connections,
                                              &inner_crit_iter_count,
                                              net_delay,
                                              net_slack,
                                              &place_delay_value,
                                              &critical_delay,
                                              &crit_exponent,
                                              &timing_cost,
                                              &delay_cost);
            } /* end of update timing-cost after try_swap() */

#ifdef VERBOSE
            /* Attention, when I use the following codes, VPR will call  *
             * comp_bb_cost() as frequency as try_swap(), it will result *
             * in VPR costing 4 or 5 times long than origin algorithm. So*
             * Do not use the following codes!!!                         */
            if (fabs(bb_cost - comp_bb_cost(CHECK, placer_opts.place_cost_type,
                                            placer_opts.num_regions))
                    > bb_cost * ERROR_TOL) {
                exit(1);
            }
#endif
        } /* end of inner loop of VPR */

        /* Lines below prevent too much round-off error from accumulating *
         * in the cost over many iterations.  This round-off can lead to  *
         * error checks failing because the cost is different from what   *
         * you get when you recompute from scratch.                       */
        moves_since_cost_recompute += move_limit;
        if (moves_since_cost_recompute > MAX_MOVES_BEFORE_RECOMPUTE) { /* > 10^6 */
            update_place_cost_after_max_move_times(&placer_opts,
                                                   &bb_cost,
                                                   &new_bb_cost,
                                                   &timing_cost,
                                                   &new_timing_cost,
                                                   &delay_cost,
                                                   &new_delay_cost);
            moves_since_cost_recompute = 0;
        } /* update placement_cost after MAX_MOVES_BEFORE_RECOMPUTE in inner_loop */

        total_iter += move_limit;
        success_ratio = ((double)success_sum) / move_limit;
        update_place_costs_by_success_sum(success_sum,
                                          bb_cost,
                                          timing_cost,
                                          delay_cost,
                                          total_cost,
                                          &av_bb_cost,
                                          &av_timing_cost,
                                          &av_delay_cost,
                                          &av_cost);
        /* std_dev was standard deviation. */
        std_dev = get_std_dev(success_sum,
                              sum_of_squares,
                              av_cost);

#ifndef SPEC
        printf("%11.5g  %10.6g  %11.6g  %11.6g  %11.6g  %11.6g  %11.4g  %d  %9.4g  %8.3g  %7.4g %7.4g %10d\n",
               t, av_cost, av_bb_cost, av_timing_cost, av_delay_cost,
               place_delay_value,
               critical_delay, success_sum, success_ratio,
               std_dev,
               rlim,
               crit_exponent,
               total_iter);
#endif

        /* FIXME: update_temperature */
        old_temper = t;  /* for finding and printing alpha. */
        update_temperature(&t,
                           rlim,
                           success_ratio,
                           annealing_sched);

        sprintf(msg, "Cost: %g  BB Cost %g  TD Cost %g  Temperature: %g  critical_delay: %g",
                total_cost,
                bb_cost,
                timing_cost,
                t,
                critical_delay);
        update_screen(MINOR,
                      msg,
                      PLACEMENT,
                      FALSE);

        /* FIXME: update range_limit */
        update_range_limit(&rlim,
                           success_ratio);

        if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
              || placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE
              /* new added for supporting PATH timing-driven placement */
              || placer_opts.place_algorithm == NEW_TIMING_DRIVEN_PLACE) {
            /* FIXME: update timing_critical_exponent */
            crit_exponent = update_crit_exponent(&placer_opts,
                                                 rlim,
                                                 final_rlim,
                                                 inverse_delta_rlim);
        }

#ifdef VERBOSE
        dump_clbs();
#endif
    } /* FIXME:  end of VPR outer loop */

    /* Now run low-temperature Simulated-Annealing placement! */
    printf("Attention, then will run low-temperature SA-Based Placement.......\n");
    t = 0; /* freeze out */
    av_cost = av_bb_cost = av_timing_cost = av_delay_cost = 0.0;
    sum_of_squares = 0.0;
    success_sum = 0;

    /* before run low-temperature placement, it should do timing_analyze! */
    if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
          || placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE
          /* new added for supporting PATH timing-driven placement */
          || placer_opts.place_algorithm == NEW_TIMING_DRIVEN_PLACE) {
        /*at each temperature change we update these values to be used     */
        /*for normalizing the tradeoff between timing and wirelength (bb)  */
        perform_timing_analyze(&placer_opts,
                               num_connections,
                               &bb_cost,
                               &outer_crit_iter_count,
                               net_slack,
                               net_delay,
                               &place_delay_value,
                               &critical_delay,
                               &crit_exponent,
                               &timing_cost,
                               &delay_cost,
                               &inverse_prev_bb_cost,
                               &inverse_prev_timing_cost);
    } /* end of running timing-analyze before low-temperature SA-based placement */

    printf("Now low-temperature SA-Based Placement......\n");
    inner_crit_iter_count = 1;
    /* TODO Q: After SA-based loop finished, why did the loop occurred here? */
    for (inner_iter = 0; inner_iter < move_limit; ++inner_iter) {
        /* FIXME, after main loop in try_place(), current t set to 0. */
        if (try_swap(t,
                     rlim,
                     pins_on_block,
                     placer_opts.place_cost_type,
                     old_region_occ_x,
                     old_region_occ_y,
                     placer_opts.num_regions,
                     fixed_pins,
                     placer_opts.place_algorithm,
                     placer_opts.timing_tradeoff,
                     inverse_prev_bb_cost,
                     inverse_prev_timing_cost,
                     &bb_cost,
                     &timing_cost,
                     &delay_cost,
                     &total_cost) == 1) {
            ++success_sum;
            av_cost += total_cost;
            av_bb_cost += bb_cost;
            av_delay_cost += delay_cost;
            av_timing_cost += timing_cost;
            sum_of_squares += total_cost * total_cost;

            if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
                  || placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE
                  /* new added for supporting PATH timing-driven placement */
                  || placer_opts.place_algorithm == NEW_TIMING_DRIVEN_PLACE) {
                update_timing_cost_after_swap(&placer_opts,
                                              inner_recompute_limit,
                                              inner_iter,
                                              move_limit,
                                              num_connections,
                                              &inner_crit_iter_count,
                                              net_delay,
                                              net_slack,
                                              &place_delay_value,
                                              &critical_delay,
                                              &crit_exponent,
                                              &timing_cost,
                                              &delay_cost);
            } /* update timing-cost after try_swap() successful in low-temperature placement! */
        } /* end of try_swap() successful in low-temperature placement */
    } /* end of for(inner_iter = 0; inner_iter < move_limit; ++inner_iter) */
    /* Low temperature SA-based Placement end! */

    total_iter += move_limit;
    success_ratio = ((double)success_sum) / move_limit;

    update_place_costs_by_success_sum(success_sum,
                                      bb_cost,
                                      timing_cost,
                                      delay_cost,
                                      total_cost,
                                      &av_bb_cost,
                                      &av_timing_cost,
                                      &av_delay_cost,
                                      &av_cost);

    std_dev = get_std_dev(success_sum,
                          sum_of_squares,
                          av_cost);

#ifndef SPEC
    printf("After low-temperature SA-Based Placement, print the result......\n");
    printf("%11.5g  %10.6g %11.6g  %11.6g  %11.6g %11.6g %11.4g %9.4g %8.3g  %7.4g  %7.4g  %10d  \n\n",
           t,
           av_cost,
           av_bb_cost,
           av_timing_cost,
           av_delay_cost,
           place_delay_value,
           critical_delay,
           success_ratio,
           std_dev,
           rlim,
           crit_exponent,
           total_iter);

    printf("For low-temperature SA-Based Placment, success_sum = %d,\
            total_iter = %10d, success_ratio = %9.4g\n",
            success_sum,
            total_iter,
            success_ratio);
#endif

#ifdef VERBOSE
    dump_clbs();
#endif

    check_place(bb_cost,
                timing_cost,
                placer_opts.place_cost_type,
                placer_opts.num_regions,
                placer_opts.place_algorithm,
                delay_cost);

    if (placer_opts.enable_timing_computations &&
            placer_opts.place_algorithm == BOUNDING_BOX_PLACE) {
        /*need this done since the timing data has not been kept up to date*
         *in bounding_box mode */
        for (inet = 0; inet < num_nets; ++inet)
            for (ipin = 1; ipin < net[inet].num_pins; ++ipin) {
                timing_place_crit[inet][ipin] = 0; /*dummy crit values*/
            }
        /*computes point_to_point_delay_cost*/
        compute_timing_driven_cost_by_orig_algo(&timing_cost,
                                                &delay_cost);
    }

    double place_est_crit = 0.0;
    if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
          || placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE
          /* new added for supporting PATH algorithm */
          || placer_opts.place_algorithm == NEW_TIMING_DRIVEN_PLACE
          || placer_opts.enable_timing_computations) {
        /* this makes net_delay up to date with    *
         * the same values that the placer is using*/
        net_delay = point_to_point_delay_cost;
        load_timing_graph_net_delays(net_delay);

        place_est_crit = calc_all_vertexs_arr_req_time(0);

        compute_net_slacks(net_slack);

#ifdef PRINT_SINK_DELAYS
        print_sink_delays("Placement_Sink_Delays.echo");
#endif

#ifdef PRINT_NET_SLACKS
        print_net_slack("Placement_Net_Slacks.echo", net_slack);
#endif

/* #ifdef PRINT_PLACE_CRIT_PATH
        print_critical_path("Placement_Crit_Path.echo");
#endif */
        char crit_path_file_name[48] = "";
        strcpy(crit_path_file_name, netlist_file);
        strcat(crit_path_file_name, "-Place_Crit_Path.echo");
        print_critical_path(crit_path_file_name);
        printf("Placement Estimated Crit Path Delay: %g\n\n", place_est_crit);
    } /* end of  TDP */

    sprintf(msg, "Placement. Cost: %g, bb_cost: %g, td_cost: %g, Channel Factor: %d, critical_delay: %g",
            total_cost,
            bb_cost,
            timing_cost,
            width_fac,
            critical_delay);
    printf("Placement. Cost: %g, bb_cost: %g, timing_cost: %g, delay_cost: %g\n",
           total_cost,
           bb_cost,
           timing_cost,
           delay_cost);
    update_screen(MAJOR,
                  msg,
                  PLACEMENT,
                  FALSE);

    printf("Total moves attempted: %d.\n", total_iter);
    if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
          || placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE
          /* new added for supporting PATH timing-driven placement */
          || placer_opts.place_algorithm == NEW_TIMING_DRIVEN_PLACE
          || placer_opts.enable_timing_computations) {
        net_delay = remember_net_delay_original_ptr;
        free_placement_structs(placer_opts.place_cost_type,
                               placer_opts.num_regions,
                               old_region_occ_x,
                               old_region_occ_y,
                               placer_opts);
        free_lookups_and_criticalities(&placer_opts,
                                       &net_delay,
                                       &net_slack);
    }
} /* end of try_place() */

void perform_timing_analyze(const placer_opts_t* placer_opts_ptr,
                            const int num_connections,
                            const double* bbox_cost,
                            int*  outer_crit_iter_count,
                            double** net_slack,
                            double** net_delay,
                            double*  place_delay_value,
                            double*  critical_delay,
                            double*  crit_exponent,
                            double*  timing_cost,
                            double*  delay_cost,
                            double* inverse_prev_bb_cost,
                            double* inverse_prev_timing_cost)
{
     if (*outer_crit_iter_count >= placer_opts_ptr->recompute_crit_iter
           || placer_opts_ptr->inner_loop_recompute_divider != 0) {
#ifdef VERBOSE
        printf("Outer Loop Recompute Criticalities\n");
#endif
        *place_delay_value = *delay_cost / num_connections;
        /* For NET_TIMING_DRIVEN_PLACE, all subnets(or connections) will *
         * set the same and constant Tdel value.                        */
        if (placer_opts_ptr->place_algorithm == NET_TIMING_DRIVEN_PLACE) {
            /* delay_cost was total_delay_cost after Timing_Analyze_Graph *
             * analysis the circuit.                                      */
            load_constant_net_delay(net_delay,
                                    *place_delay_value);
        }

        /*note, for path_based, the net Tdel is not updated since it is current,
         *because it accesses point_to_point_delay array. */
        load_timing_graph_net_delays(net_delay);
        /* Compute each vertexes's req_time and arr_time time, and calculate *
         * the max_delay in critical path.                          */
        *critical_delay = calc_all_vertexs_arr_req_time(0);
        compute_net_slacks(net_slack);

        compute_timing_driven_cost(placer_opts_ptr,
                                   net_slack,
                                   block_pin_to_tnode,
                                   *critical_delay,
                                   *crit_exponent,
                                   timing_cost,
                                   delay_cost);
        *outer_crit_iter_count = 0;
    } /* end of update timing_cost in TIMING_DRIVEN_PLACE */

    ++(*outer_crit_iter_count);
    /*at each temperature change, we update these values to be used   */
    /*for normalizing the tradeoff between timing and wirelength(bb)  */
    *inverse_prev_bb_cost = 1 / *bbox_cost;
    *inverse_prev_timing_cost = 1 / *timing_cost;
}  /* end of void perform_timing_analyze() */

void  update_timing_cost_after_swap(const placer_opts_t* placer_opts_ptr,
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
                                    double*  delay_cost_ptr)
{
    if (*inner_crit_iter_count >= inner_recompute_limit
            && inner_iter != move_limit - 1) {
        *inner_crit_iter_count = 0;
#ifdef VERBOSE
        printf("Inner Loop Recompute Criticalities\n");
#endif
        if (placer_opts_ptr->place_algorithm == NET_TIMING_DRIVEN_PLACE) {
            *place_delay_value = *delay_cost_ptr / num_connections;
            load_constant_net_delay(net_delay,
                                    *place_delay_value);
        }

        load_timing_graph_net_delays(net_delay);
        *critical_delay = calc_all_vertexs_arr_req_time(0);
        compute_net_slacks(net_slack);

        compute_timing_driven_cost(placer_opts_ptr,
                                   net_slack,
                                   block_pin_to_tnode,
                                   *critical_delay,
                                   *crit_exponent,
                                   timing_cost_ptr,
                                   delay_cost_ptr);
    } /* update timing_cost_ptr in try_swap() at low-temperature placement */

    ++(*inner_crit_iter_count);
}  /* end of update_timing_cost_after_swap() */

void  compute_timing_driven_cost(const placer_opts_t* placer_opts_ptr,
                                 double**  net_slack,
                                 int** block_pin_to_tnode,
                                 const double critical_delay,
                                 const double crit_exponent,
                                 double*  timing_cost,
                                 double*  delay_cost)
{
    if (placer_opts_ptr->place_algorithm == NEW_TIMING_DRIVEN_PLACE) {
        compute_all_nets_local_crit_weight(net_slack,
                                           block_pin_to_tnode,
                                           critical_delay);
        compute_timing_driven_costs_by_path_algo(timing_cost,
                                                 delay_cost);
    } else { /* PATH_TIMING_DRIVEN_PLACE or NET_TIMING_DRIVEN_PLACE */
        load_criticalities(net_slack,
                           critical_delay,
                           crit_exponent);
        compute_timing_driven_cost_by_orig_algo(timing_cost,
                                                delay_cost);
    }
}  /* end of void compute_timing_driven_cost() */

void update_place_cost_after_max_move_times(const placer_opts_t* placer_opts_ptr,
                                            double*  bb_cost,
                                            double*  new_bb_cost,
                                            double*  timing_cost,
                                            double*  new_timing_cost,
                                            double*  delay_cost,
                                            double*  new_delay_cost)
{
    /* first recompute bbox_cost..... */
    *new_bb_cost = recompute_bb_cost(placer_opts_ptr->place_cost_type,
                                     placer_opts_ptr->num_regions);

    if (fabs(*new_bb_cost - *bb_cost) > *bb_cost * ERROR_TOL) {
        printf("Error in try_place:  new_bb_cost = %g, old bb_cost = %g.\n",
               *new_bb_cost, *bb_cost);
        exit(1);
    }
    *bb_cost = *new_bb_cost;

    /* then recompute timing-driven placement cost after innner-loop...*/
    if (placer_opts_ptr->place_algorithm == NET_TIMING_DRIVEN_PLACE
          || placer_opts_ptr->place_algorithm == PATH_TIMING_DRIVEN_PLACE
          /* new added for support PATH timing-driven placement */
          || placer_opts_ptr->place_algorithm == NEW_TIMING_DRIVEN_PLACE) {
        /* Q: current time, why it needn't loading Timing_Analyze_Graph, *
         * update edge slack value to compute timing-driven cost?        *
         * */
        compute_timing_driven_cost_in_outer_loop(placer_opts_ptr,
                                                 *timing_cost,
                                                 *delay_cost,
                                                 new_timing_cost,
                                                 new_delay_cost);

        *timing_cost = *new_timing_cost;  /* TODO */
        /* I new added */
        *delay_cost = *new_delay_cost;
    }
} /* end of void update_place_cost_after_max_move_times() */

void compute_timing_driven_cost_in_outer_loop(const placer_opts_t* placer_opts_ptr,
                                              const double timing_cost,
                                              const double delay_cost,
                                              double* new_timing_cost,
                                              double* new_delay_cost)
{
    if (placer_opts_ptr->place_algorithm == NET_TIMING_DRIVEN_PLACE
          || placer_opts_ptr->place_algorithm == PATH_TIMING_DRIVEN_PLACE) {
        compute_timing_driven_cost_by_orig_algo(new_timing_cost,
                                                new_delay_cost);
    } else { /* NEW_TIMING_DRIVEN_PLACE */
        compute_timing_driven_costs_by_path_algo(new_timing_cost,
                                                 new_delay_cost);
    }

    if (fabs(*new_timing_cost - timing_cost) > timing_cost * ERROR_TOL) {
        printf("Error in try_place:  new_timing_cost = %g, old timing_cost = %g.\n",
               *new_timing_cost,
               timing_cost);
        exit(1);
    }

    if (fabs(*new_delay_cost - delay_cost) > delay_cost * ERROR_TOL) {
        printf("Error in try_place:  new_delay_cost = %g, old delay_cost = %g.\n",
               *new_delay_cost,
               delay_cost);
        exit(1);
    }
}  /* end of void compute_timing_driven_cost_in_outer_loop() */

void update_place_costs_by_success_sum(const int success_sum,
                                       const double bb_cost,
                                       const double timing_cost,
                                       const double delay_cost,
                                       const double total_cost,
                                       double* av_bb_cost_ptr,
                                       double* av_timing_cost_ptr,
                                       double* av_delay_cost_ptr,
                                       double* av_cost_ptr)
{
    if (0 == success_sum) {
        *av_bb_cost_ptr = bb_cost;
        *av_timing_cost_ptr = timing_cost;
        *av_delay_cost_ptr = delay_cost;
        *av_cost_ptr = total_cost;
    } else {
        *av_bb_cost_ptr /= success_sum;
        *av_timing_cost_ptr /= success_sum;
        *av_delay_cost_ptr /= success_sum;
        *av_cost_ptr /= success_sum;
    }
}

/* It only count the inter-CLB net connections */
static int count_connections()
{
    /*only count non-global connections*/
    int count = 0;
    int inet = -1;
    for (inet = 0; inet < num_nets; ++inet) {
        if (is_global[inet]) {
            continue;
        }
        /* for a n-pin net, it had n-1 connections. */
        count += (net[inet].num_pins - 1);
    }

    return count;
}

/* Computes net_pin_index[0..num_blocks-1][0..num_pins-1] array, this array  *
 * allows us to quickly find which net connected on the pin(a blocks pin).    */
static void compute_net_pin_index_values()  /* FIXME */
{
    /*initialize values to OPEN */
    int iblk = -1;
    int ipin = -1;
    for (iblk = 0; iblk < num_blocks; ++iblk) {
        for (ipin = 0; ipin < pins_per_clb; ++ipin) {
            net_pin_index[iblk][ipin] = OPEN;
        }
    }

    int inet = -1;
    for (inet = 0; inet < num_nets; ++inet) {
        if (is_global[inet]) {
            continue;
        }

        int netpin = -1;
        for (netpin = 0; netpin < net[inet].num_pins; ++netpin) {
            int blk = net[inet].blocks[netpin]; /* blk_num */
            /* there is only one blocks pin, so it is 0, and it is driving the *
             * net since this is an INPAD.                                    */
            if (blocks[blk].type == INPAD) {
                net_pin_index[blk][0] = 0;
            } else if (blocks[blk].type == OUTPAD) {
              /*there is only one blocks pin, it is 0 */
                net_pin_index[blk][0] = netpin;
            } else {
                net_pin_index[blk][net[inet].blk_pin[netpin]] = netpin;
            }
        }
    }
} /* end of static void compute_net_pin_index_values(void) */

/* Returns the standard deviation of data set x.  There are n sample points, *
 * sum_x_squared is the summation over n of x^2 and av_x is the average x.   *
 * All operations are done in double precision, since round off error can be *
 * a problem in the initial temp. std_dev calculation for big circuits.      */
static double get_std_dev(int n, double sum_x_squared, double av_x)
{
    double std_dev = 0.0;
    if (n <= 1) {
        std_dev = 0.;
    } else {
        std_dev = (sum_x_squared - n * av_x * av_x) / (double)(n - 1);
    }

    if (std_dev > 0.) {      /* Very small variances sometimes round negative */
        std_dev = sqrt(std_dev);
    } else {
        std_dev = 0.;
    }

    return (std_dev);
}

static void update_range_limit(double* rlim,
                               double success_ratio)
{
    /* Update the range limited to keep acceptance prob. near 0.44.  Use *
     * a doubleing point rlim to allow gradual transitions at low temps.  */
    *rlim = (*rlim) * (1.0 - 0.44 + success_ratio);
    double upper_lim = max(num_of_columns,
                           num_of_rows);
    *rlim = min(*rlim, upper_lim);
    *rlim = max(*rlim, 1.0);
    /* *rlim = (double) num_of_columns; */
}

static double update_crit_exponent(const placer_opts_t* placer_opts_ptr,
                                   const double rlim,
                                   const double final_rlim,
                                   const double inverse_delta_rlim)
{
    const double td_place_exp_first = placer_opts_ptr->td_place_exp_first;
    const double td_place_exp_last = placer_opts_ptr->td_place_exp_last;
    double new_crit_exponent = (1 - (rlim - final_rlim) * inverse_delta_rlim)
                     * (td_place_exp_last - td_place_exp_first) + td_place_exp_first;
    
    return new_crit_exponent;
}

static int exit_crit(double t,
                     double cost,
                     annealing_sched_t annealing_sched)
{
    /* Return 1 when the exit criterion is meet. */
    if (annealing_sched.type == USER_SCHED) {
        if (t < annealing_sched.exit_t) {
            return(1);
        } else {
            return(0);
        }
    }

    /* Automatic annealing schedule */
    if (t < 0.005 * cost / num_nets) {
        return 1;
    } else {
        return 0;
    }
} /* end of static int exit_crit() */

static double starting_temperature(const int* pins_on_block,
                                   const int  place_cost_type,
                                   double** old_region_occ_x,
                                   double** old_region_occ_y,
                                   const int  num_regions,
                                   const boolean fixed_pins,
                                   const annealing_sched_t annealing_sched,
                                   const int max_moves,
                                   const double rlim,
                                   const place_algorithm_t place_algorithm,
                                   const double timing_tradeoff,
                                   const double inverse_prev_bb_cost,
                                   const double inverse_prev_timing_cost,
                                   double*  bb_cost_ptr,
                                   double*  timing_cost_ptr,
                                   double*  delay_cost_ptr,
                                   double*  total_cost_ptr)
{
    /* Finds the starting temperature (hot condition). */
    if (annealing_sched.type == USER_SCHED) {
        return (annealing_sched.init_t);
    }

    int move_limit = min(max_moves, num_blocks);
    int num_accepted = 0;
    double av_cost = 0.0;
    double sum_of_squares = 0.;
    /* Try one move per blocks.  Set t high so essentially all accepted. */
    int i = -1;
    for (i = 0; i < move_limit; ++i) {
        if (try_swap(1.e30,
                     rlim,
                     pins_on_block,
                     place_cost_type,
                     old_region_occ_x,
                     old_region_occ_y,
                     num_regions,
                     fixed_pins,
                     place_algorithm,
                     timing_tradeoff,
                     inverse_prev_bb_cost,
                     inverse_prev_timing_cost,
                     bb_cost_ptr,
                     timing_cost_ptr,
                     delay_cost_ptr,
                     total_cost_ptr) == 1) {
            ++num_accepted;
            av_cost += *total_cost_ptr;
            sum_of_squares += (*total_cost_ptr) * (*total_cost_ptr);
        }
    }

    /* Initial Temp = 20 * std_dev. */
    if (num_accepted != 0) {
        av_cost /= num_accepted;
    } else {
        av_cost = 0.;
    }

    /* Double important to avoid round off */
    double std_dev = get_std_dev(num_accepted, sum_of_squares, av_cost);
#ifdef DEBUG
    if (num_accepted != move_limit) {
        printf("Warning:  Starting t: %d of %d configurations accepted.\n",
               num_accepted, move_limit);
    }
#endif

    return (20. * std_dev);
    /* return (15.225523    ); */
} /* end of static double start_t() */

static void update_temperature(double* t,
                               double rlim,
                               double success_ratio,
                               annealing_sched_t annealing_sched)
{
    /* Update the temperature according to the annealing schedule selected. */

    /*  double fac; */
    if (annealing_sched.type == USER_SCHED) {
        *t = annealing_sched.alpha_t * (*t);
    } else {  /* AUTO_SCHED */
        if (success_ratio > 0.96) {
            *t = (*t) * 0.5;
        } else if (success_ratio > 0.8) {
            *t = (*t) * 0.9;
        } else if (success_ratio > 0.15 || rlim > 1.) {
            *t = (*t) * 0.95;
        } else {
            *t = (*t) * 0.8;
        }
    }
}

/* FIXME: Try to swap a pair of plbs or io_pads randomly*/
/* Picks some blocks and moves it to another spot. If this spot had occupied *
 * , switch the blocks. Assess the change in cost function, and accept or   *
 * reject the move. If rejected, return 0, else return 1. Pass back the new *
 * value of the cost function. rlim is the range_limit. pins_on_block gives *
 * the number of pins on each type of blocks(improves efficiency). Pins on   *
 * each type of blocks(improves efficiency).                                 */
static int try_swap(const double  t,
                    const double rlim,
                    const int* pins_on_block,
                    const int  place_cost_type,
                    double** old_region_occ_x,
                    double** old_region_occ_y,
                    const int num_regions,
                    const boolean fixed_pins,
                    const place_algorithm_t place_algorithm,
                    const double timing_tradeoff,
                    const double inverse_prev_bb_cost,
                    const double inverse_prev_timing_cost,
                    double* bb_cost_ptr,
                    double* timing_cost_ptr,
                    double* delay_cost_ptr,
                    double* total_cost_ptr)
{
    static bbox_t* bb_coord_new = NULL;
    static bbox_t* bb_edge_new = NULL;
    static int*  nets_to_update = NULL;
    static int*  net_block_moved = NULL;

    /* Allocate the local bb_coordinate storage, etc. only once. */
    /* Q: Why did the array allocate double memory space? TODO*/
    if (bb_coord_new == NULL) {
        bb_coord_new = (bbox_t*)my_malloc(2 * pins_per_clb *
                                                sizeof(bbox_t));
        bb_edge_new = (bbox_t*)my_malloc(2 * pins_per_clb *
                                               sizeof(bbox_t));
        nets_to_update = (int*)my_malloc(2 * pins_per_clb * sizeof(int));
        net_block_moved = (int*)my_malloc(2 * pins_per_clb * sizeof(int));
    }

    int from_block = my_irand(num_blocks - 1); /* choose from blocks randomly */
    int to_block = 0;
    /* If the pins are fixed, we never move them from their initial random *
     * locations. The code below could be made more efficient by using the *
     * fact that pins appear first in the blocks list, but this shouldn't   *
     * cause any significant slowdown and won't be broken if I ever change *
     * the parser so that the pins aren't necessarily at the start of the  *
     * blocks list.                                                         */
    if (fixed_pins == TRUE) {
        while (blocks[from_block].type != CLB) {
            from_block = my_irand(num_blocks - 1);
        }
    }

    int x_from = blocks[from_block].x;
    int y_from = blocks[from_block].y;
    int x_to = 0;
    int y_to = 0;
    /* find the plb or io_pad (x_to, y_to) randomly */
    find_to(x_from,
            y_from,
            blocks[from_block].type,
            rlim,
            &x_to,
            &y_to);

    /* Make the switch in order to let compute the new bounding-box simpler. If *
     * increase is too high, switch them back.(blocks data structures switched,  *
     * clb not switched until success of move is determinded.)                 */
    int io_num = 0;
    if (blocks[from_block].type == CLB) {
        if (clb[x_to][y_to].occ == 1) { /* target clb location had occupied -- do a switch */
            /* then swap (x_from, y_from) and (x_to, y_to) coordinate. */
            to_block = clb[x_to][y_to].u.blocks;
            blocks[from_block].x = x_to;
            blocks[from_block].y = y_to;
            blocks[to_block].x = x_from;
            blocks[to_block].y = y_from;
        } else { /* if clb[x_to][y_to] was empty, move blocks[from_block] to clb[x_to][y_to] location. */
            to_block = EMPTY;
            blocks[from_block].x = x_to;
            blocks[from_block].y = y_to;
        }
    } else { /* io pads was selected for moving */
        io_num = my_irand(io_rat - 1);
        if (io_num >= clb[x_to][y_to].occ) { /* Moving to an empty location, why? TODO:*/
            to_block = EMPTY;
            blocks[from_block].x = x_to;
            blocks[from_block].y = y_to;
        } else { /* Swapping two blocks */
            to_block = *(clb[x_to][y_to].u.io_blocks + io_num);
            blocks[to_block].x = x_from;
            blocks[to_block].y = y_from;
            blocks[from_block].x = x_to;
            blocks[from_block].y = y_to;
        }
    }

    /*===========  Now Compute the cost ==============*/
    /* Now update the cost function. May have to do major optimizations here    *
     * later. I'm using negative values of temp_net_cost as a flag, so DO NOT   *
     * use cost-functions that can go negative.                                 */
    const int num_of_pins = pins_on_block[blocks[from_block].type];
    /* When blocks[to_block] was a EMPTY, it only deal with blocks[from_block] */
    const int num_nets_affected = find_affected_nets(nets_to_update,
                                                     net_block_moved,
                                                     from_block,
                                                     to_block,
                                                     num_of_pins);

    if (place_cost_type == NONLINEAR_CONG) {
        save_region_occ(old_region_occ_x,
                        old_region_occ_y,
                        num_regions);
    }

    /* Then calculate the wirelength_cost and timing_cost_ptr */
    double bb_delta_c = 0.0;
    int bb_index = 0; /* Index of new bounding box. */
    int k = -1;
    int inet = -1;
    int off_from = -1;
    for (k = 0; k < num_nets_affected; ++k) {
        inet = nets_to_update[k];
        /* If we swapped two blocks connected to the same net, its bounding box *
         * doesn't change. Due to it only swap a pair of blocks, it didn't change*
         * the bounding-box coordinate.                                         */
        if (net_block_moved[k] == FROM_AND_TO) {
            continue;
        }
        /* Then update the net bounding-box. */
        if (net[inet].num_pins <= SMALL_NET) { /* 4 */
            get_non_updateable_bb(inet,
                                  &bb_coord_new[bb_index]);
        } else {
            if (net_block_moved[k] == FROM) {
                update_bb(inet,
                          &bb_coord_new[bb_index],
                          &bb_edge_new[bb_index],
                          x_from,
                          y_from,
                          x_to,
                          y_to);
            } else {
                update_bb(inet,
                          &bb_coord_new[bb_index],
                          &bb_edge_new[bb_index],
                          x_to,
                          y_to,
                          x_from,
                          y_from);
            }
        }

        /* then calculate bounding_box_cost */
        if (place_cost_type != NONLINEAR_CONG) {
            temp_net_cost[inet] = get_net_cost(inet,
                                               &bb_coord_new[bb_index]);
            bb_delta_c += temp_net_cost[inet] - net_cost[inet];
        } else {
            /* Rip up, then replace with new bb. */
            update_region_occ(inet,
                              &bb_coords[inet],
                              -1,
                              num_regions);
            update_region_occ(inet,
                              &bb_coord_new[bb_index],
                              1,
                              num_regions);
        }

        ++bb_index;
    } /* end of for (k = 0; k < num_nets_affected; k++)*/

    double  newcost = 0.0;
    if (place_cost_type == NONLINEAR_CONG) {
        newcost = nonlinear_cong_cost(num_regions);
        bb_delta_c = newcost - *bb_cost_ptr;
    }

    /* then calculate timing-total_cost_ptr */
    double delta_cost = 0.0; /* Change in total_cost_ptr due to this swap. */
    double timing_delta_c = 0.0;
    double delay_delta_c = 0.0;
    if (place_algorithm == NET_TIMING_DRIVEN_PLACE
          || place_algorithm == PATH_TIMING_DRIVEN_PLACE
    /* new added for supporting PATH timing-driven placement */
          || place_algorithm == NEW_TIMING_DRIVEN_PLACE) {
        /*in this case we redefine delta_cost as a combination of timing and bb.*
         *additionally, we normalize all values, therefore delta_cost is in     *
         *relation to 1 */
        compute_delta_timing_driven_cost(place_algorithm,
                                         from_block,
                                         to_block,
                                         num_of_pins,
                                         &timing_delta_c,
                                         &delay_delta_c);

        /* FIXME: delta_cost = lambda * (bbox_delta_cost / previous_bbox_cost) +  *
         *              (1-lambda) * (timing_delta_cost / previous_timing_cost)   *
         * inverse_prev_bb_cost and inverse_prev_timing_cost was auto-normalized  *
         * variables in VPR4.3 papers.                                            */
        const double normal_bb_cost = bb_delta_c * inverse_prev_bb_cost;
        const double normal_timing_cost = timing_delta_c * inverse_prev_timing_cost;
        delta_cost = (1 - timing_tradeoff) * normal_bb_cost
                       + timing_tradeoff * normal_timing_cost;
    } else {
        delta_cost = bb_delta_c;
    }

    /* 1: move accepted, 0: rejected. */
    int keep_switch = assess_swap(delta_cost,
                                  t);
    if (keep_switch) {
        *total_cost_ptr = *total_cost_ptr + delta_cost;
        *bb_cost_ptr = *bb_cost_ptr + bb_delta_c;

        if (place_algorithm == NET_TIMING_DRIVEN_PLACE
              || place_algorithm == PATH_TIMING_DRIVEN_PLACE
              || place_algorithm == NEW_TIMING_DRIVEN_PLACE) {
            *timing_cost_ptr = *timing_cost_ptr + timing_delta_c;
            *delay_cost_ptr = *delay_cost_ptr + delay_delta_c;
            /* update the point_to_point_timing_cost and point_to_point_delay_cost,*
             * when VPR had accept current move.                                   */
            update_timing_driven_cost(from_block,
                                      to_block,
                                      num_of_pins);
        }

        /* update net total_cost_ptr functions and reset flags. */
        bb_index = 0;
        for (k = 0; k < num_nets_affected; ++k) {
            inet = nets_to_update[k];

            /* If we swapped two blocks connected to the same net, its bounding *
             * box doesn't change. So ignore it                                 */
            if (net_block_moved[k] == FROM_AND_TO) {
                temp_net_cost[inet] = -1;
                continue;
            }

            bb_coords[inet] = bb_coord_new[bb_index];
            if (net[inet].num_pins > SMALL_NET) {
                bb_num_on_edges[inet] = bb_edge_new[bb_index];
            }

            ++bb_index;
            net_cost[inet] = temp_net_cost[inet];
            temp_net_cost[inet] = -1;
        }

        /* Update Clb data structures since we kept the move. */
        if (blocks[from_block].type == CLB) {
            if (to_block != EMPTY) {
                clb[x_from][y_from].u.blocks = to_block;
                clb[x_to][y_to].u.blocks = from_block;
            } else {
                clb[x_to][y_to].u.blocks = from_block;
                clb[x_to][y_to].occ = 1;
                clb[x_from][y_from].occ = 0;
            }
        } else {   /* io blocks was selected for moving */
            /* Get the "sub_block" number of the from_block blocks. */
            for (off_from = 0;; ++off_from) {
                if (clb[x_from][y_from].u.io_blocks[off_from] == from_block) {
                    break;
                }
            }

            if (to_block != EMPTY) { /* Swapped two blocks. */
                clb[x_to][y_to].u.io_blocks[io_num] = from_block;
                clb[x_from][y_from].u.io_blocks[off_from] = to_block;
            } else {             /* Moved to an empty location */
                clb[x_to][y_to].u.io_blocks[clb[x_to][y_to].occ] = from_block;
                ++(clb[x_to][y_to].occ);

                for (k = off_from; k < clb[x_from][y_from].occ-1; ++k) { /* prevent gap  */
                    clb[x_from][y_from].u.io_blocks[k] =   /* in io_blocks */
                        clb[x_from][y_from].u.io_blocks[k + 1];
                }

                --(clb[x_from][y_from].occ);
            }
        } /* end of selecting move io_block */
    } else {  /* Move was rejected.  */
        /* Reset the net total_cost_ptr function flags first. */
        for (k = 0; k < num_nets_affected; k++) {
            inet = nets_to_update[k];
            temp_net_cost[inet] = -1;
        }

        /* Restore the blocks data structures to their state before the move. */
        blocks[from_block].x = x_from;
        blocks[from_block].y = y_from;

        if (to_block != EMPTY) {
            blocks[to_block].x = x_to;
            blocks[to_block].y = y_to;
        }

        /* Restore the region occupancies to their state before the move. */
        if (place_cost_type == NONLINEAR_CONG) {
            restore_region_occ(old_region_occ_x, old_region_occ_y, num_regions);
        }
    }

    return keep_switch;
} /* end of static int try_swap() */


static void save_region_occ(double** old_region_occ_x,
                            double** old_region_occ_y,
                            int num_regions)
{
    /* Saves the old occupancies of the placement subregions in case the  *
     * current move is not accepted.  Used only for NONLINEAR_CONG.       */
    int i, j;

    for (i = 0; i < num_regions; i++) {
        for (j = 0; j < num_regions; j++) {
            old_region_occ_x[i][j] = place_region_x[i][j].occupancy;
            old_region_occ_y[i][j] = place_region_y[i][j].occupancy;
        }
    }
}


static void restore_region_occ(double** old_region_occ_x,
                               double** old_region_occ_y, int num_regions)
{
    /* Restores the old occupancies of the placement subregions when the  *
     * current move is not accepted.  Used only for NONLINEAR_CONG.       */
    int i, j;

    for (i = 0; i < num_regions; i++) {
        for (j = 0; j < num_regions; j++) {
            place_region_x[i][j].occupancy = old_region_occ_x[i][j];
            place_region_y[i][j].occupancy = old_region_occ_y[i][j];
        }
    }
}


/* When blocks[to_block] was a EMPTY, it only deal with blocks[from_block] */
static int find_affected_nets(int* nets_to_update, int* net_block_moved,
                              int from_block, int to_block, int num_of_pins)
{
    /* Puts a list of all the nets connected to from_block and to_block into nets_to_update. *
     * Returns the number of affected nets. Net_block_moved is either FROM, TO or    *
     * FROM_AND_TO -- the blocks connected to this net that has moved.                */
    int count = 0;
    int affected_index = 0;
    int inet = 0;
    int k = -1;
    for (k = 0; k < num_of_pins; ++k) {
        inet = blocks[from_block].nets[k];
        if (inet == OPEN || is_global[inet] == TRUE) {
            continue;
        }

        /* This is here in case the same blocks connects to a net twice. */
        if (temp_net_cost[inet] > 0.0) {
            continue;
        }

        nets_to_update[affected_index] = inet;
        net_block_moved[affected_index] = FROM;
        ++affected_index;
        temp_net_cost[inet] = 1.0; /* Flag to say we've marked this net. */
    }

    if (to_block != EMPTY) {
        for (k = 0; k < num_of_pins; ++k) {
            inet = blocks[to_block].nets[k];
            if (inet == OPEN || is_global[inet] == TRUE) {
                continue;
            }

            if (temp_net_cost[inet] > 0.0) { /* Net already marked. */
                for (count = 0; count < affected_index; ++count) {
                    /* this net connect both FROM and TO blocks. */
                    if (nets_to_update[count] == inet) {
                        if (net_block_moved[count] == FROM) {
                            net_block_moved[count] = FROM_AND_TO;
                        }

                        break;
                    }
                }

#ifdef DEBUG
                if (count > affected_index) {
                    printf("Error in find_affected_nets -- count = %d,"
                           " affected index = %d.\n", count, affected_index);
                    exit(1);
                }

#endif
            } else { /* Net not marked yet. */
                nets_to_update[affected_index] = inet;
                net_block_moved[affected_index] = TO;
                ++affected_index;
                temp_net_cost[inet] = 1.;    /* Flag means we've  marked net. */
            }
        } /* end of for(k = 0; k < num_of_pins; ++k) */
    } /* end of if(to_block != EMPTY) */

    return (affected_index);
} /* end of static int find_affected_nets() */


static void find_to(int x_from, int y_from, int type, double rlim,
                    int* x_to, int* y_to)
{
    /* Returns the point to which I want to swap, properly range limited.  *
     * rlim must always be between 1 and num_of_columns(inclusive) for this routine to *
     * work.                                                               */
    int rlx = min(num_of_columns, rlim);  /* x_range_limit, Only needed when num_of_columns < num_of_rows. */
    int rly = min(num_of_rows, rlim);  /* y_range_limit, Added rly for aspect_ratio != 1 case. */

#ifdef DEBUG
    if (rlx < 1 || rlx > num_of_columns) {
        printf("Error in find_to: rlx = %d\n", rlx);
        exit(1);
    }
#endif

    do {   /* Until (x_to, y_to) different from (x_from, y_from) */
        int x_rel = 0;
        int y_rel = 0;
        if (type == CLB) {
            x_rel = my_irand(2 * rlx); /* x_rel >= 0 && x_rel <= 2 * rlx */
            y_rel = my_irand(2 * rly);
            /* why? Now I see. */
            /* When x_rel >= 0 && x_rel < rlx, (*x_to) = x_from - (rlx - x_rel), *
             * when x_rel >= rlx && x_rel <= 2 * rlx, (*x_to) = x_from + (x_rel - rlx)*/
            *x_to = x_from + (x_rel - rlx);
            *y_to = y_from + (y_rel - rly);

            if (*x_to > num_of_columns) {
                *x_to = *x_to - num_of_columns;    /* better spectral props. */
            }
            if (*x_to < 1) {
                *x_to = *x_to + num_of_columns;    /* than simple min, max   */
            }

            if (*y_to > num_of_rows) {
                *y_to = *y_to - num_of_rows;    /* clipping. */
            }
            if (*y_to < 1) {
                *y_to = *y_to + num_of_rows;
            }
        } else { /* IO blocks will be moved. */
            int iside = 0;
            int iplace = 0;
            if (rlx >= num_of_columns) { /* x_range_limit >= num_columns */
                iside = my_irand(3); /* the random number was: 0,1,2,3 */
                /********************************
                 *       +-----1----+           *
                 *       |          |           *
                 *       |          |           *
                 *       0          2           *
                 *       |          |           *
                 *       |          |           *
                 *       +-----3----+           *
                 ********************************/
                switch (iside) {
                    case 0:
                        iplace = my_irand(num_of_rows - 1) + 1;
                        *x_to = 0;
                        *y_to = iplace;
                        break;

                    case 1:
                        iplace = my_irand(num_of_columns - 1) + 1;
                        *x_to = iplace;
                        *y_to = num_of_rows + 1;
                        break;

                    case 2:
                        iplace = my_irand(num_of_rows - 1) + 1;
                        *x_to = num_of_columns + 1;
                        *y_to = iplace;
                        break;

                    case 3:
                        iplace = my_irand(num_of_columns - 1) + 1;
                        *x_to = iplace;
                        *y_to = 0;
                        break;

                    default:
                        printf("Error in find_to.  Unexpected io swap location.\n");
                        exit(1);
                } /* end of switch(iside) */
            } else { /* rlx is less than whole chip */
                if (x_from == 0) {
                    iplace = my_irand(2 * rly);
                    *y_to = y_from + (iplace - rly);
                    *x_to = x_from;

                    if (*y_to > num_of_rows) {
                        *y_to = num_of_rows + 1; /* Attention, num_of_rows+1 was top iopads */
                        *x_to = my_irand(rlx - 1) + 1;
                    } else if (*y_to < 1) {
                        *y_to = 0;
                        *x_to = my_irand(rlx - 1) + 1;
                    }
                } else if (x_from == num_of_columns + 1) {
                    iplace = my_irand(2 * rly);
                    *y_to = y_from + (iplace - rly);
                    *x_to = x_from;

                    if (*y_to > num_of_rows) {
                        *y_to = num_of_rows + 1;
                        *x_to = num_of_columns - my_irand(rlx - 1);
                    } else if (*y_to < 1) {
                        *y_to = 0;
                        *x_to = num_of_columns - my_irand(rlx - 1);
                    }
                } else if (y_from == 0) {
                    iplace = my_irand(2 * rlx);
                    *x_to = x_from + (iplace - rlx);
                    *y_to = y_from;

                    if (*x_to > num_of_columns) {
                        *x_to = num_of_columns + 1;
                        *y_to = my_irand(rly - 1) + 1;
                    } else if (*x_to < 1) {
                        *x_to = 0;
                        *y_to = my_irand(rly - 1) + 1;
                    }
                } else { /* *y_from == num_of_rows + 1 */
                    iplace = my_irand(2 * rlx);
                    *x_to = x_from + (iplace - rlx);
                    *y_to = y_from;

                    if (*x_to > num_of_columns) {
                        *x_to = num_of_columns + 1;
                        *y_to = num_of_rows - my_irand(rly - 1);
                    } else if (*x_to < 1) {
                        *x_to = 0;
                        *y_to = num_of_rows - my_irand(rly - 1);
                    }
                }
            }  /* End of else(rlx is less than whole chip) */
        } /* end of else(io_block to be moved) */
    } while ((x_from == *x_to) && (y_from == *y_to));

#ifdef DEBUG
    if (*x_to < 0 || *x_to > num_of_columns + 1 || *y_to < 0 || *y_to > num_of_rows + 1) {
        printf("Error in routine find_to:  (x_to,y_to) = (%d,%d)\n",
               *x_to, *y_to);
        exit(1);
    }

    if (type == CLB) {
        if (clb[*x_to][*y_to].type != CLB) {
            printf("Error: Moving CLB to illegal type blocks at (%d,%d)\n",
                   *x_to, *y_to);
            exit(1);
        }
    } else {
        if (clb[*x_to][*y_to].type != IO) {
            printf("Error: Moving IO blocks to illegal type location at "
                   "(%d,%d)\n", *x_to, *y_to);
            exit(1);
        }
    }

#endif
    /* printf("(%d,%d) moved to (%d,%d)\n",x_from,y_from,*x_to,*y_to); */
} /* end of static void find_to() */


static int assess_swap(double delta_cost,
                       double t)
{
    /* Returns: 1 -> move accepted, 0 -> rejected. */
    int accept = -1;
    double fnum;

    if (delta_cost <= 0) {
#ifdef SPEC          /* Reduce variation in final solution due to round off */
        fnum = my_frand();
#endif
        accept = 1;
        return(accept);
    }

    if (t == 0.) {
        return(0);
    }

    fnum = my_frand();
    double prob_fac = exp(-delta_cost / t);
    if (prob_fac > fnum) {
        accept = 1;
    } else {
        accept = 0;
    }

    return accept;
}


static double recompute_bb_cost(int place_cost_type, int num_regions)
{
    /* Recomputes the cost to eliminate roundoff that may have accrued.  *
     * This routine does as little work as possible to compute this new  *
     * cost.                                                             */
    int i, j, inet;
    double cost;
    cost = 0;

    /* Initialize occupancies to zero if regions are being used. */

    if (place_cost_type == NONLINEAR_CONG) {
        for (i = 0; i < num_regions; i++) {
            for (j = 0; j < num_regions; j++) {
                place_region_x[i][j].occupancy = 0.;
                place_region_y[i][j].occupancy = 0.;
            }
        }
    }

    for (inet = 0; inet < num_nets; inet++) { /* for each net ... */
        if (is_global[inet] == FALSE) {    /* Do only if not global. */

            /* Bounding boxes don't have to be recomputed; they're correct. */
            if (place_cost_type != NONLINEAR_CONG) {
                cost += net_cost[inet];
            } else {    /* Must be nonlinear_cong case. */
                update_region_occ(inet, &bb_coords[inet], 1, num_regions);
            }
        }
    }

    if (place_cost_type == NONLINEAR_CONG) {
        cost = nonlinear_cong_cost(num_regions);
    }

    return (cost);
}

/* FIXME: compute a subnet Tdel from source pin to sink pin according to *
 * find (delta_x, delta_y) in Tdel lookup matrix                         *
 * TODO: In vpr4.3, due to FPGA arch was homogeneous arch, when the arch  *
 * change to heterogeneous one, How did I compute bb_cost and td_cost?    */
static double compute_point_to_point_delay(int inet,
                                           int sink_pin)
{
    int source_block_index = net[inet].blocks[0];
    block_types_t source_type = blocks[source_block_index].type; /* it maybe plb, or io pads*/

    int sink_block_index = net[inet].blocks[sink_pin];
    block_types_t sink_type = blocks[sink_block_index].type;

    int delta_x = abs(blocks[sink_block_index].x - blocks[source_block_index].x);
    int delta_y = abs(blocks[sink_block_index].y - blocks[source_block_index].y);

    /* from clb_outpin_to_clb_inpin, delta_clb_to_clb[delta_x][delta_y]. */
    double delay_source_to_sink = 0.0;
    if (source_type == CLB) {
        if (sink_type == CLB) { /* from clb_outpin to clb input_pin */
            delay_source_to_sink = delta_clb_to_clb[delta_x][delta_y];
        } else if (sink_type == OUTPAD) { /* from clb_outpin to output pad */
        /* from clb_outpin_to_outpad, delta_clb_to_outpad[delta_x][delta_y] */
            delay_source_to_sink = delta_clb_to_outpad[delta_x][delta_y];
        } else {  /* NO clb_to_inpad */
            printf("Error in compute_point_to_point_delay in place.c, \
                    cannot from clb output pin to input pad\n");
            exit(1);
        }
    } else if (source_type == INPAD) {
        if (sink_type == CLB) { /* from input pad to clb input pin */
            delay_source_to_sink = delta_inpad_to_clb[delta_x][delta_y];
        } else if (sink_type == OUTPAD) { /* from input pad to output pad */
            delay_source_to_sink = delta_inpad_to_outpad[delta_x][delta_y];
        } else { /* NO inpad_to_inpad */
            printf("Error in compute_point_to_point_delay in place.c, \
                    cannot from input pad to input pad\n");
            exit(1);
        }
    } else {  /* NO outpad to others */
        printf("Error in compute_point_to_point_delay in place.c, bad source_type\n");
        exit(1);
    }

    if (delay_source_to_sink < 0.0) {
        printf("Error in compute_point_to_point_delay in place.c, Tdel is less than 0\n");
        exit(1);
    }

    return delay_source_to_sink;
} /* end of static double compute_point_to_point_delay() */

/* Update point_to_point_delay_cost and point_to_point_timing_cost, *
 * when from_block and to_block had moved.                                  */
static void update_timing_driven_cost(int from_block,
                                      int to_block,
                                      int num_of_pins)
{
    /* Update the point_to_point_timing_cost values from the temporary *
     * values for all connections that have changed */
    int blkpin, net_pin, inet, ipin;
    for (blkpin = 0; blkpin < num_of_pins; ++blkpin) {
        inet = blocks[from_block].nets[blkpin];
        if (inet == OPEN || is_global[inet] == TRUE) {
            continue;
        }

        net_pin = net_pin_index[from_block][blkpin];
        if (net_pin != 0) {
            /*the following "if" prevents the value from being updated twice*/
            if (net[inet].blocks[0] != to_block
                  && net[inet].blocks[0] != from_block) {
                point_to_point_delay_cost[inet][net_pin] =
                        temp_point_to_point_delay_cost[inet][net_pin];
                temp_point_to_point_delay_cost[inet][net_pin] = -1;

                point_to_point_timing_cost[inet][net_pin] =
                        temp_point_to_point_timing_cost[inet][net_pin];
                temp_point_to_point_timing_cost[inet][net_pin] = -1;
            }
        } else { /*this net is being driven by a moved blocks, recompute */
            /*all point to point connections on this net.*/
            for (ipin = 1; ipin < net[inet].num_pins; ++ipin) {
                point_to_point_delay_cost[inet][ipin] =
                        temp_point_to_point_delay_cost[inet][ipin];
                temp_point_to_point_delay_cost[inet][ipin] = -1;

                point_to_point_timing_cost[inet][ipin] =
                        temp_point_to_point_timing_cost[inet][ipin];
                temp_point_to_point_timing_cost[inet][ipin] = -1;
            }
        }
    } /* end of for(blkpin = 0; blkpin < num_of_pins; blkpin++) */

    if (to_block != EMPTY) {
        for (blkpin = 0; blkpin < num_of_pins; ++blkpin) {
            inet = blocks[to_block].nets[blkpin];
            if (inet == OPEN || is_global[inet] == TRUE) {
                continue;
            }

            net_pin = net_pin_index[to_block][blkpin];
            if (net_pin != 0) {
                /*the following "if" prevents the value from being updated 2x*/
                if (net[inet].blocks[0] != to_block
                      && net[inet].blocks[0] != from_block) {
                    point_to_point_delay_cost[inet][net_pin] =
                        temp_point_to_point_delay_cost[inet][net_pin];
                    temp_point_to_point_delay_cost[inet][net_pin] = -1;

                    point_to_point_timing_cost[inet][net_pin] =
                        temp_point_to_point_timing_cost[inet][net_pin];
                    temp_point_to_point_timing_cost[inet][net_pin] = -1;
                }
            } else { /*this net is being driven by a moved blocks, recompute */
                /*all point to point connections on this net.*/
                for (ipin = 1; ipin < net[inet].num_pins; ++ipin) {
                    point_to_point_delay_cost[inet][ipin] =
                        temp_point_to_point_delay_cost[inet][ipin];
                    temp_point_to_point_delay_cost[inet][ipin] = -1;

                    point_to_point_timing_cost[inet][ipin] =
                        temp_point_to_point_timing_cost[inet][ipin];
                    temp_point_to_point_timing_cost[inet][ipin] = -1;
                }
            }
        }
    } /* end of if(to_block != EMPTY) */
}  /* end of static void update_timing_driven_cost() */

/* TODO: compute the timing-cost incrmentally, this function was very important! */
/* A net that is being driven by a moved blocks must have all of its sinks timing *
 * costs recomputed. A net that is driving a moved blocks must only have the      *
 * timing cost on the connection driving the input pin computed.                 */
static void compute_delta_timing_driven_cost(place_algorithm_t place_algorithm,
                                             int  from_block,
                                             int  to_block,
                                             int  num_of_pins,
                                             double* delta_timing,
                                             double* delta_delay)
{
    int inet, net_pin, ipin;
    double temp_delay = 0.0;
    double delta_timing_cost = 0.0;
    double delta_delay_cost = 0.0;
    int k = -1;
    for (k = 0; k < num_of_pins; ++k) { /* num_of_pins was pins of from blocks */
        inet = blocks[from_block].nets[k];
        if (inet == OPEN || is_global[inet]) {
            continue;
        }

        net_pin = net_pin_index[from_block][k]; /* it was pin index */
        if (net_pin != 0) { /*this net is driving a moved blocks */
        /*if this net is being driven by a blocks that has moved, we do not  */
        /*need to compute the change in the timing cost (here) since it will*/
        /*be computed in the fanout of the net on  the driving blocks, also  */
        /*computing it here would double count the change, and mess up the  */
        /*delta_timing_cost value */
            if (net[inet].blocks[0] != to_block
                  && net[inet].blocks[0] != from_block) {
                temp_delay = compute_point_to_point_delay(inet,
                                                          net_pin);
                temp_point_to_point_delay_cost[inet][net_pin] = temp_delay;

                if (place_algorithm == NEW_TIMING_DRIVEN_PLACE) {
                    temp_point_to_point_timing_cost[inet][net_pin] =
                      subnet_local_crit_weight[inet][net_pin] * temp_delay;
                } else {
                    temp_point_to_point_timing_cost[inet][net_pin] =
                      timing_place_crit[inet][net_pin] * temp_delay;
                }

                delta_delay_cost += temp_point_to_point_delay_cost[inet][net_pin]
                                      - point_to_point_delay_cost[inet][net_pin];
                delta_timing_cost += temp_point_to_point_timing_cost[inet][net_pin]
                                       - point_to_point_timing_cost[inet][net_pin];
            }
        } else { /*this net is being driven by a moved blocks, recompute */
            /*Why?  all point to point connections on this net.*/
            for (ipin = 1; ipin < net[inet].num_pins; ++ipin) {
                temp_delay = compute_point_to_point_delay(inet,
                                                          ipin);
                temp_point_to_point_delay_cost[inet][ipin] = temp_delay;

                if (place_algorithm == NEW_TIMING_DRIVEN_PLACE) {
                    temp_point_to_point_timing_cost[inet][ipin] =
                      subnet_local_crit_weight[inet][ipin] * temp_delay;
                } else {
                    temp_point_to_point_timing_cost[inet][ipin] =
                      timing_place_crit[inet][ipin] * temp_delay;
                }

                delta_delay_cost += temp_point_to_point_delay_cost[inet][ipin]
                                      - point_to_point_delay_cost[inet][ipin];
                delta_timing_cost += temp_point_to_point_timing_cost[inet][ipin]
                                       - point_to_point_timing_cost[inet][ipin];
            }
        }
    } /* end of for (k = 0; k < num_of_pins; k++) */

    if (to_block != EMPTY) {
        for (k = 0; k < num_of_pins; ++k) {
            inet = blocks[to_block].nets[k];
            if (inet == OPEN || is_global[inet]) {
                continue;
            }

            net_pin = net_pin_index[to_block][k];
            if (net_pin != 0) { /*this net is driving a moved blocks*/
            /*if this net is being driven by a blocks that has moved, we do not */
            /*need to compute the change in the timing cost (here) since it was*/
            /*computed in the fanout of the net on  the driving blocks, also    */
            /*computing it here would double count the change, and mess up the */
            /*delta_timing_cost value */
                if (net[inet].blocks[0] != to_block
                      && net[inet].blocks[0] != from_block) {
                    temp_delay = compute_point_to_point_delay(inet,
                                                              net_pin);
                    temp_point_to_point_delay_cost[inet][net_pin] = temp_delay;

                    if (place_algorithm == NEW_TIMING_DRIVEN_PLACE) {
                        temp_point_to_point_timing_cost[inet][net_pin] =
                          subnet_local_crit_weight[inet][net_pin] * temp_delay;
                    } else {
                        temp_point_to_point_timing_cost[inet][net_pin] =
                          timing_place_crit[inet][net_pin] * temp_delay;
                    }

                    delta_delay_cost += temp_point_to_point_delay_cost[inet][net_pin]
                                          - point_to_point_delay_cost[inet][net_pin];
                    delta_timing_cost += temp_point_to_point_timing_cost[inet][net_pin]
                                           - point_to_point_timing_cost[inet][net_pin];
                }
            } else { /*this net is being driven by a moved blocks, recompute */
                /*Why?  all point to point connections on this net.*/
                for (ipin = 1; ipin < net[inet].num_pins; ++ipin) {
                    temp_delay = compute_point_to_point_delay(inet,
                                                              ipin);
                    temp_point_to_point_delay_cost[inet][ipin] = temp_delay;

                    if (place_algorithm == NEW_TIMING_DRIVEN_PLACE) {
                        temp_point_to_point_timing_cost[inet][ipin] =
                          subnet_local_crit_weight[inet][ipin] * temp_delay;
                    } else {
                        temp_point_to_point_timing_cost[inet][ipin] =
                          timing_place_crit[inet][ipin] * temp_delay;
                    }

                    delta_delay_cost += temp_point_to_point_delay_cost[inet][ipin]
                                          - point_to_point_delay_cost[inet][ipin];

                    delta_timing_cost += temp_point_to_point_timing_cost[inet][ipin]
                                           - point_to_point_timing_cost[inet][ipin];
                }
            } /* end of else */
        }  /* end of for(k = 0; k < num_of_pins; k++) */
    } /* end of if (to_block != EMPTY) */

    *delta_timing = delta_timing_cost;
    *delta_delay = delta_delay_cost;
}  /* end of static void compute_delta_td_cost() */

/* FIXME: compute timing-driven costs of all signal nets(its all subnets)  *
 * Attention: I found this function was called by NET_TIMING_DRIVEN_PLACE. */
/* Computes the cost(from scratch) due to the Tdels and criticalities on all  *
 * point-to-point connections, we define the timing cost of each connection as *
 * criticality * Tdel.                                                        */
static void compute_timing_driven_cost_by_orig_algo(double* timing_cost,
                                                    double* connection_delay_sum)
{
    double local_timing_cost = 0.0;
    double local_connect_delay_sum = 0.0;

    int inet = -1;
    for (inet = 0; inet < num_nets; ++inet) { /* for each net ... */
        if (is_global[inet] == FALSE) { /* Do only for signal nets. */
            int ipin = 0;
            for (ipin = 1; ipin < net[inet].num_pins; ++ipin) {
                /* FIXME: subnet_delay_cost was a subnet(in a net) delay_cost */
                double subnet_delay_cost = compute_point_to_point_delay(inet,
                                                                        ipin);
                point_to_point_delay_cost[inet][ipin] = subnet_delay_cost;
                local_connect_delay_sum += subnet_delay_cost;

                temp_point_to_point_delay_cost[inet][ipin] = -1; /*undefined*/

                /* subnet_timing_cost was a subnet(in a net) timing_cost */
                double subnet_timing_cost =
                           subnet_delay_cost * timing_place_crit[inet][ipin];
                point_to_point_timing_cost[inet][ipin] = subnet_timing_cost;
                local_timing_cost += subnet_timing_cost;

                /* TODO: Why? */
                temp_point_to_point_timing_cost[inet][ipin] = -1; /*undefined*/
            }
        } /* end of if(is_global[inet] == FALSE) */
    } /* end of for (inet = 0; inet < num_nets; ++inet) */

    *timing_cost = local_timing_cost;
    *connection_delay_sum = local_connect_delay_sum;
} /* end of static void compute_timing_driven_cost_by_orig_algo() */

/* FIXME: This function was used for calculate Timing-Driven_Placement by *
 * PATH algorithm, which noted at "A Novel Net Weighting Algorithm for    *
 * Timing-Driven Placement", Tim Kong, 2002.                              */
static void compute_timing_driven_costs_by_path_algo(double* timing_cost,
                                                     double* connection_delay_sum)
{
    double local_timing_cost = 0.0;
    double local_connect_delay_sum = 0.0;

    int inet = -1;
    for (inet = 0; inet < num_nets; ++inet) {
        if (is_global[inet] == FALSE) {
            int ipin = 0;
            for (ipin = 1; ipin < net[inet].num_pins; ++ipin) {
                double subnet_delay_cost = compute_point_to_point_delay(inet,
                                                                        ipin);
                point_to_point_delay_cost[inet][ipin] = subnet_delay_cost;
                local_connect_delay_sum += subnet_delay_cost;

                temp_point_to_point_delay_cost[inet][ipin] = -1;

                double subnet_timing_cost =
                    subnet_delay_cost * subnet_local_crit_weight[inet][ipin];

                point_to_point_timing_cost[inet][ipin] = subnet_timing_cost;
                local_timing_cost += subnet_timing_cost;

                temp_point_to_point_timing_cost[inet][ipin] = -1;
            }  /* end of for(ipin = 1; ipin < net[inet].num_pins; ++ipin) */
        } /* end of if(is_global[inet] == FALSE) */
    } /* end of for(inet = 0; inet < num_nets; ++inet) */

    *timing_cost = local_timing_cost;
    *connection_delay_sum = local_connect_delay_sum;
}  /* end of static void compute_timing_driven_costs_by_path_algo() */

/* compute_net_boundingbox_cost, Wirelength cost, TODO */
static double comp_bb_cost(int method,
                           int place_cost_type,
                           int num_regions)
{
    /* Finds the cost from scratch. Done only when the placement has been     *
     * radically changed(i.e. after initial placement). Otherwise find the    *
     * cost change Incrementally. If method check is NORMAL, we find bounding *
     * -boxes that are updateable for the larger nets. If the method is CHECK,*
     * all bounding-boxes are found via the non_updateable_bb routine, to     *
     * provide a cost which can be used to chekc the correctness of the other *
     * routine.                                                               */

    /* Initialize occupancies to zero if regions are being used. */
    if (place_cost_type == NONLINEAR_CONG) {
        int i = -1;
        int j = -1;
        for (i = 0; i < num_regions; i++) {
            for (j = 0; j < num_regions; j++) {
                place_region_x[i][j].occupancy = 0.;
                place_region_y[i][j].occupancy = 0.;
            }
        }
    }

    int n = -1;
    double cost = 0.0;
    for (n = 0; n < num_nets; ++n) { /* for each net ... */
        if (is_global[n] == FALSE) { /* Do only if not global. */
            /* Small nets don't use incremental updating on their bounding boxes, *
             * so they can use a fast bounding box calculator.                    */
            if (net[n].num_pins > SMALL_NET && method == NORMAL) { /* 4 */
                get_bb_from_scratch(n, &bb_coords[n], &bb_num_on_edges[n]);
            } else {
                get_non_updateable_bb(n, &bb_coords[n]);
            }

            if (place_cost_type != NONLINEAR_CONG) {
                net_cost[n] = get_net_cost(n, &bb_coords[n]);
                cost += net_cost[n];
            } else {    /* Must be nonlinear_cong case. */
                update_region_occ(n, &bb_coords[n], 1, num_regions);
            }
        }
    }

    if (place_cost_type == NONLINEAR_CONG) {
        cost = nonlinear_cong_cost(num_regions);
    }

    return (cost);
}


static double nonlinear_cong_cost(int num_regions)
{
    /* This routine computes the cost of a placement when the NONLINEAR_CONG *
     * option is selected.  It assumes that the occupancies of all the       *
     * placement subregions have been properly updated, and simply           *
     * computes the cost due to these occupancies by summing over all        *
     * subregions.  This will be inefficient for moves that don't affect     *
     * many subregions (i.e. small moves late in placement), esp. when there *
     * are a lot of subregions.  May recode later to update only affected    *
     * subregions.                                                           */
    double cost, tmp;
    int i, j;
    cost = 0.;

    for (i = 0; i < num_regions; i++) {
        for (j = 0; j < num_regions; j++) {
            /* Many different cost metrics possible.  1st try:  */
            if (place_region_x[i][j].occupancy < place_region_x[i][j].capacity) {
                cost += place_region_x[i][j].occupancy *
                        place_region_x[i][j].inv_capacity;
            } else { /* Overused region -- penalize. */
                tmp = place_region_x[i][j].occupancy *
                      place_region_x[i][j].inv_capacity;
                cost += tmp * tmp;
            }

            if (place_region_y[i][j].occupancy < place_region_y[i][j].capacity) {
                cost += place_region_y[i][j].occupancy *
                        place_region_y[i][j].inv_capacity;
            } else { /* Overused region -- penalize. */
                tmp = place_region_y[i][j].occupancy *
                      place_region_y[i][j].inv_capacity;
                cost += tmp * tmp;
            }
        }
    }

    return (cost);
}


static void update_region_occ(int inet, bbox_t* coords,
                              int add_or_sub, int num_regions)
{
    /* Called only when the place_cost_type is NONLINEAR_CONG.  If add_or_sub *
     * is 1, this uses the new net bounding box to increase the occupancy     *
     * of some regions.  If add_or_sub = - 1, it decreases the occupancy      *
     * by that due to this bounding box.                                      */
    double net_xmin, net_xmax, net_ymin, net_ymax, crossing;
    double inv_region_len, inv_region_height;
    double inv_bb_len, inv_bb_height;
    double overlap_xlow, overlap_xhigh, overlap_ylow, overlap_yhigh;
    double y_overlap, x_overlap, x_occupancy, y_occupancy;
    int imin, imax, jmin, jmax, i, j;

    if (net[inet].num_pins > 50) {
        crossing = 2.7933 + 0.02616 * (net[inet].num_pins - 50);
    } else {
        crossing = cross_count[net[inet].num_pins - 1];
    }

    net_xmin = coords->xmin - 0.5;
    net_xmax = coords->xmax + 0.5;
    net_ymin = coords->ymin - 0.5;
    net_ymax = coords->ymax + 0.5;
    /* I could precompute the two values below.  Should consider this. */
    inv_region_len = (double) num_regions / (double) num_of_columns;
    inv_region_height = (double) num_regions / (double) num_of_rows;
    /* Get integer coordinates defining the rectangular area in which the *
     * subregions have to be updated.  Formula is as follows:  subtract   *
     * 0.5 from net_xmin, etc. to get numbers from 0 to num_of_columns or num_of_rows;         *
     * divide by num_of_columns or num_of_rows to scale between 0 and 1; multiply by           *
     * num_regions to scale between 0 and num_regions; and truncate to    *
     * get the final answer.                                              */
    imin = (int)(net_xmin - 0.5) * inv_region_len;
    imax = (int)(net_xmax - 0.5) * inv_region_len;
    imax = min(imax, num_regions - 1);        /* Watch for weird roundoff */
    jmin = (int)(net_ymin - 0.5) * inv_region_height;
    jmax = (int)(net_ymax - 0.5) * inv_region_height;
    jmax = min(jmax, num_regions - 1);        /* Watch for weird roundoff */
    inv_bb_len = 1. / (net_xmax - net_xmin);
    inv_bb_height = 1. / (net_ymax - net_ymin);

    /* See RISA paper (ICCAD '94, pp. 690 - 695) for a description of why *
     * I use exactly this cost function.                                  */

    for (i = imin; i <= imax; i++) {
        for (j = jmin; j <= jmax; j++) {
            overlap_xlow = max(place_region_bounds_x[i], net_xmin);
            overlap_xhigh = min(place_region_bounds_x[i + 1], net_xmax);
            overlap_ylow = max(place_region_bounds_y[j], net_ymin);
            overlap_yhigh = min(place_region_bounds_y[j + 1], net_ymax);
            x_overlap = overlap_xhigh - overlap_xlow;
            y_overlap = overlap_yhigh - overlap_ylow;
#ifdef DEBUG

            if (x_overlap < -0.001) {
                printf("Error in update_region_occ:  x_overlap < 0"
                       "\n inet = %d, overlap = %g\n", inet, x_overlap);
            }

            if (y_overlap < -0.001) {
                printf("Error in update_region_occ:  y_overlap < 0"
                       "\n inet = %d, overlap = %g\n", inet, y_overlap);
            }

#endif
            x_occupancy = crossing * y_overlap * x_overlap * inv_bb_height *
                          inv_region_len;
            y_occupancy = crossing * x_overlap * y_overlap * inv_bb_len *
                          inv_region_height;
            place_region_x[i][j].occupancy += add_or_sub * x_occupancy;
            place_region_y[i][j].occupancy += add_or_sub * y_occupancy;
        }
    }
}


static void free_place_regions(int num_regions)
{
    /* Frees the place_regions data structures needed by the NONLINEAR_CONG *
     * cost function.                                                       */
    free_matrix(place_region_x,
                0,
                num_regions - 1,
                0,
                sizeof(place_region_t));
    free_matrix(place_region_y,
                0,
                num_regions - 1,
                0,
                sizeof(place_region_t));
    free(place_region_bounds_x);
    free(place_region_bounds_y);
}


static void free_placement_structs(int place_cost_type,
                                   int num_regions,
                                   double** old_region_occ_x,
                                   double** old_region_occ_y,
                                   placer_opts_t placer_opts)
{
    /* Frees the major structures needed by the placer(and not needed *
     * elsewhere).   */

    if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
          || placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE
          || placer_opts.place_algorithm == NEW_TIMING_DRIVEN_PLACE
          || placer_opts.enable_timing_computations) {
        int inet = -1;
        for (inet = 0; inet < num_nets; ++inet) {
            /*add one to the address since it is indexed from 1 not 0 */
            ++point_to_point_delay_cost[inet];
            free(point_to_point_delay_cost[inet]);

            ++point_to_point_timing_cost[inet];
            free(point_to_point_timing_cost[inet]);

            ++temp_point_to_point_delay_cost[inet];
            free(temp_point_to_point_delay_cost[inet]);

            ++temp_point_to_point_timing_cost[inet];
            free(temp_point_to_point_timing_cost[inet]);
        }

        free(point_to_point_delay_cost);
        free(temp_point_to_point_delay_cost);
        free(point_to_point_timing_cost);
        free(temp_point_to_point_timing_cost);
        free_matrix(net_pin_index,
                    0,
                    num_blocks - 1,
                    0,
                    sizeof(int));
    } /* end of if () */

    free(net_cost);
    free(temp_net_cost);
    free(bb_num_on_edges);
    free(bb_coords);
    net_cost = NULL;  /* Defensive coding. */
    temp_net_cost = NULL;
    bb_num_on_edges = NULL;
    bb_coords = NULL;
    free_unique_pin_list();

    if (place_cost_type == NONLINEAR_CONG) {
        free_place_regions(num_regions);
        free_matrix(old_region_occ_x, 0, num_regions - 1, 0, sizeof(double));
        free_matrix(old_region_occ_y, 0, num_regions - 1, 0, sizeof(double));
    } else if (place_cost_type == LINEAR_CONG) {
        free_fast_cost_update_structs();
    }
} /* static void free_placement_structs() */

/* Allocates the major structures needed only by the placer, primarily for *
 * computing costs quickly and such. FIXME                                 */
static void alloc_and_load_placement_structs(int  place_cost_type,
                                             int  num_regions,
                                             double place_cost_exp,
                                             double*** old_region_occ_x,
                                             double*** old_region_occ_y,
                                             placer_opts_t placer_opts)
{
    int inet = -1;
    int ipin = -1;
    /* If I didn't add NEW_TIMING_DRIVEN_PLACE, point_to_point_delay_cost
     * point_to_point_timing_cost and net_pin_index must be NULL pointer.
     * It must result in segment fault error! */
    if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
          || placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE
          || placer_opts.place_algorithm == NEW_TIMING_DRIVEN_PLACE
          || placer_opts.enable_timing_computations) {
        point_to_point_delay_cost = (double**)my_malloc(num_nets
                                                     * sizeof(double*));
        temp_point_to_point_delay_cost = (double**)my_malloc(num_nets
                                                     * sizeof(double*));

        point_to_point_timing_cost = (double**)my_malloc(num_nets
                                                     * sizeof(double*));
        temp_point_to_point_timing_cost = (double**)my_malloc(num_nets
                                                     * sizeof(double*));

        net_pin_index = (int**)alloc_matrix(0,
                                            num_blocks - 1,
                                            0,
                                            pins_per_clb - 1,
                                            sizeof(int));

        /* TODO: When I comment the following 4 statements like:                *
         * --point_to_point_dealy_cost[inet], the placeent result keep the same *
         * like previous vpr4.3, but when free the placement structs, it generated
         * segment fault. Now I should discover the this phenomenon.            */
        for (inet = 0; inet < num_nets; ++inet) {
        /* in the following, subract one so index starts at 1 instead of 0 */
            point_to_point_delay_cost[inet] =
                (double*)my_malloc((net[inet].num_pins - 1) * sizeof(double));
            --(point_to_point_delay_cost[inet]);

            temp_point_to_point_delay_cost[inet] =
                (double*)my_malloc((net[inet].num_pins - 1) * sizeof(double));
            --(temp_point_to_point_delay_cost[inet]);

            point_to_point_timing_cost[inet] =
                (double*)my_malloc((net[inet].num_pins - 1) * sizeof(double));
            --(point_to_point_timing_cost[inet]);

            temp_point_to_point_timing_cost[inet] =
                (double*)my_malloc((net[inet].num_pins - 1) * sizeof(double));
            --(temp_point_to_point_timing_cost[inet]);
        } /* end of for() */

        for (inet = 0; inet < num_nets; ++inet) {
            for (ipin = 1; ipin < net[inet].num_pins; ++ipin) {
                point_to_point_delay_cost[inet][ipin] = 0;
                temp_point_to_point_delay_cost[inet][ipin] = 0;
            }
        }
    }
    /* double new_cost[0..num_nets-1] */
    net_cost = (double*)my_malloc(num_nets * sizeof(double));
    temp_net_cost = (double*)my_malloc(num_nets * sizeof(double));

    /* Used to store costs for moves not yet made and to indicate when a net's   *
     * cost has been recomputed. temp_net_cost[inet] < 0 means net's cost hasn't *
     * been recomputed.                                                          */
    for (inet = 0; inet < num_nets; ++inet) {
        temp_net_cost[inet] = -1.0;
    }

    bb_coords = (bbox_t*)my_malloc(num_nets * sizeof(bbox_t));
    bb_num_on_edges = (bbox_t*)my_malloc(num_nets * sizeof(bbox_t));

    /* Get a list of pins with no duplicates. */
    alloc_and_load_unique_pin_list();

    /* Allocate storage for subregion data, if needed. */
    if (place_cost_type == NONLINEAR_CONG) {
        alloc_place_regions(num_regions);
        load_place_regions(num_regions);
        *old_region_occ_x = (double**)alloc_matrix(0,
                                                   num_regions - 1,
                                                   0,
                                                   num_regions - 1,
                                                   sizeof(double));
        *old_region_occ_y = (double**)alloc_matrix(0,
                                                   num_regions - 1,
                                                   0,
                                                   num_regions - 1,
                                                   sizeof(double));
    } else { /* Shouldn't use them; crash hard if I do!   */
        *old_region_occ_x = NULL;
        *old_region_occ_y = NULL;

        alloc_and_load_for_fast_cost_update(place_cost_exp);
    }

    /* if (place_cost_type == LINEAR_CONG) {
        alloc_and_load_for_fast_cost_update(place_cost_exp);
    } */
} /* end of static void alloc_and_load_placement_structs() */


static void alloc_place_regions(int num_regions)
{
    /* Allocates memory for the regional occupancy, cost, etc. counts *
     * kept when we're using the NONLINEAR_CONG placement cost        *
     * function.                                                      */
    place_region_x = (place_region_t**)alloc_matrix(0,
                                                    num_regions - 1,
                                                    0,
                                                    num_regions - 1,
                                                    sizeof(place_region_t));
    place_region_y = (place_region_t**)alloc_matrix(0,
                                                    num_regions - 1,
                                                    0,
                                                    num_regions - 1,
                                                    sizeof(place_region_t));
    place_region_bounds_x = (double*)my_malloc((num_regions + 1) *
                                               sizeof(double));
    place_region_bounds_y = (double*)my_malloc((num_regions + 1) *
                                               sizeof(double));
}

static void load_place_regions(int num_regions)
{
    /* Loads the capacity values in each direction for each of the placement *
     * regions.  The chip is divided into a num_regions x num_regions array. */
    int i, j, low_block, high_block, rnum;
    double low_lim, high_lim, capacity, fac, block_capacity;
    double len_fac, height_fac;

    /* First load up horizontal channel capacities.  */
    for (j = 0; j < num_regions; j++) {
        capacity = 0.;
        low_lim = (double) j / (double) num_regions * num_of_rows + 1.;
        high_lim = (double)(j + 1) / (double) num_regions * num_of_rows;
        low_block = floor(low_lim);
        low_block = max(1, low_block); /* Watch for weird roundoff effects. */
        high_block = ceil(high_lim);
        high_block = min(high_block, num_of_rows);
        block_capacity = (chan_width_x[low_block - 1] +
                          chan_width_x[low_block]) / 2.;

        if (low_block == 1) {
            block_capacity += chan_width_x[0] / 2.;
        }

        fac = 1. - (low_lim - low_block);
        capacity += fac * block_capacity;

        for (rnum = low_block + 1; rnum < high_block; rnum++) {
            block_capacity = (chan_width_x[rnum - 1] + chan_width_x[rnum]) / 2.;
            capacity += block_capacity;
        }

        block_capacity = (chan_width_x[high_block - 1] +
                          chan_width_x[high_block]) / 2.;

        if (high_block == num_of_rows) {
            block_capacity += chan_width_x[num_of_rows] / 2.;
        }

        fac = 1. - (high_block - high_lim);
        capacity += fac * block_capacity;

        for (i = 0; i < num_regions; i++) {
            place_region_x[i][j].capacity = capacity;
            place_region_x[i][j].inv_capacity = 1. / capacity;
            place_region_x[i][j].occupancy = 0.;
            place_region_x[i][j].cost = 0.;
        }
    }

    /* Now load vertical channel capacities.  */
    for (i = 0; i < num_regions; i++) {
        capacity = 0.;
        low_lim = (double) i / (double) num_regions * num_of_columns + 1.;
        high_lim = (double)(i + 1) / (double) num_regions * num_of_columns;
        low_block = floor(low_lim);
        low_block = max(1, low_block); /* Watch for weird roundoff effects. */
        high_block = ceil(high_lim);
        high_block = min(high_block, num_of_columns);
        block_capacity = (chan_width_y[low_block - 1] +
                          chan_width_y[low_block]) / 2.;

        if (low_block == 1) {
            block_capacity += chan_width_y[0] / 2.;
        }

        fac = 1. - (low_lim - low_block);
        capacity += fac * block_capacity;

        for (rnum = low_block + 1; rnum < high_block; rnum++) {
            block_capacity = (chan_width_y[rnum - 1] + chan_width_y[rnum]) / 2.;
            capacity += block_capacity;
        }

        block_capacity = (chan_width_y[high_block - 1] +
                          chan_width_y[high_block]) / 2.;

        if (high_block == num_of_columns) {
            block_capacity += chan_width_y[num_of_columns] / 2.;
        }

        fac = 1. - (high_block - high_lim);
        capacity += fac * block_capacity;

        for (j = 0; j < num_regions; j++) {
            place_region_y[i][j].capacity = capacity;
            place_region_y[i][j].inv_capacity = 1. / capacity;
            place_region_y[i][j].occupancy = 0.;
            place_region_y[i][j].cost = 0.;
        }
    }

    /* Finally set up the arrays indicating the limits of each of the *
     * placement subregions.                                          */
    len_fac = (double) num_of_columns / (double) num_regions;
    height_fac = (double) num_of_rows / (double) num_regions;
    place_region_bounds_x[0] = 0.5;
    place_region_bounds_y[0] = 0.5;

    for (i = 1; i <= num_regions; i++) {
        place_region_bounds_x[i] = place_region_bounds_x[i - 1] + len_fac;
        place_region_bounds_y[i] = place_region_bounds_y[i - 1] + height_fac;
    }
}


static void free_unique_pin_list(void)
{
    /* Frees the unique pin list structures.                               */
    int any_dup, inet;
    any_dup = 0;

    for (inet = 0; inet < num_nets; inet++) {
        if (duplicate_pins[inet] != 0) {
            free(unique_pin_list[inet]);
            any_dup = 1;
        }
    }

    if (any_dup != 0) {
        free(unique_pin_list);
    }

    free(duplicate_pins);
}

/* This routine looks for multiple pins going to the same blocks in the *
 * pinlist of each net.  If it finds any, it marks that net as having  *
 * duplicate pins, and creates a new pinlist with no duplicates.  This *
 * is then used by the updatable bounding box calculation routine for  *
 * efficiency.                                                         */
static void alloc_and_load_unique_pin_list(void)
{
    duplicate_pins = (int*)my_calloc(num_nets, sizeof(int));

    /* [0..num_blocks-1]: number of times a blocks is   *
     * listed in the pinlist of a net.  Temp. storage. */
    int* times_listed = (int*)my_calloc(num_blocks, sizeof(int));
    int  any_dups = 0;
    int inet = -1;
    for (inet = 0; inet < num_nets; ++inet) {
        int num_dup = 0;
        int ipin = -1;
        int block_num = -1;
        for (ipin = 0; ipin < net[inet].num_pins; ++ipin) {
            block_num = net[inet].blocks[ipin];
            ++times_listed[block_num];
            if (times_listed[block_num] > 1) {
                ++num_dup;
            }
        }

        if (num_dup > 0) { /* Duplicates found. Make unique pin list. */
            duplicate_pins[inet] = num_dup;
            if (any_dups == 0) { /* This is the first duplicate found */
                /* int unique_pin_list[0..num_nets-1][0..num_unique_blocks-1] */
                unique_pin_list = (int**)my_calloc(num_nets, sizeof(int*));
                any_dups = 1;
            }

            /* nets[inet].num_pins - num_dup */
            unique_pin_list[inet] = (int*)my_malloc((net[inet].num_pins - num_dup) *
                                              sizeof(int));

            int offset = 0;
            for (ipin = 0; ipin < net[inet].num_pins; ++ipin) {
                block_num = net[inet].blocks[ipin];
                if (times_listed[block_num] != 0) {
                    times_listed[block_num] = 0;
                    unique_pin_list[inet][offset] = block_num;
                    ++offset;
                }
            }
        } else { /* No duplicates found. Reset times_listed. */
            for (ipin = 0; ipin < net[inet].num_pins; ++ipin) {
                block_num = net[inet].blocks[ipin];
                times_listed[block_num] = 0;
            }
        }
    }

    free(times_listed);
    times_listed = NULL;
}


/* This routine finds the bounding-box of each net from scratch(i.e. from   *
 * only the blocks location information). It updates both the coordinate and *
 * number of blocks on each edge information. It should only be called when *
 * the bounding-box information is not valid.                               */
static void get_bb_from_scratch(int inet, bbox_t* coords,
                                bbox_t* num_on_edges)
{
    /* I need a list of blocks to which this net connects, with no blocks listed *
     * more than once, in order to get a proper count of the number on the edge *
     * of the bounding box.                                                     */
    int  n_pins;
    int* plist;
    if (duplicate_pins[inet] == 0) {
        plist = net[inet].blocks;
        n_pins = net[inet].num_pins;
    } else {
        plist = unique_pin_list[inet];
        n_pins = net[inet].num_pins - duplicate_pins[inet];
    }

    int x = blocks[plist[0]].x; /* plist[0] was source pin */
    int y = blocks[plist[0]].y;
    x = max(min(x, num_of_columns), 1);
    y = max(min(y, num_of_rows), 1);

    int xmin = x;
    int ymin = y;
    int xmax = x;
    int ymax = y;

    int xmin_edge = 1;
    int ymin_edge = 1;
    int xmax_edge = 1;
    int ymax_edge = 1;

    int ipin = 0;
    for (ipin = 1; ipin < n_pins; ++ipin) {
        int bnum = plist[ipin];
        x = blocks[bnum].x;
        y = blocks[bnum].y;
        /* Code below counts IO blocks as being within the (1..num_of_columns, 1..num_of_rows clb) array. *
         * This is because channels do not go out of the 0..num_of_columns, 0..num_of_rows range, and I   *
         * always take all channels impinging on the bounding box to be within that  *
         * bounding box. Hence, this "movement" of IO blocks does not affect the     *
         * which channels are included within the bounding box, and it simplifies the*
         * the code a lot.                                                           */
        x = max(min(x, num_of_columns), 1);
        y = max(min(y, num_of_rows), 1);

        if (x == xmin) {
            ++xmin_edge;
        } else if (x == xmax) { /* Recall that xmin could equal xmax -- don't use else */
            ++xmax_edge;
        } else if (x < xmin) {
            xmin = x;
            xmin_edge = 1;
        } else if (x > xmax) {
            xmax = x;
            xmax_edge = 1;
        }

        if (y == ymin) {
            ++ymin_edge;
        } else if (y == ymax) {
            ++ymax_edge;
        } else if (y < ymin) {
            ymin = y;
            ymin_edge = 1;
        } else if (y > ymax) {
            ymax = y;
            ymax_edge = 1;
        }
    } /* end of for (ipin = 1; ipin < n_pins; ++ipin) */

    /* Copy the coordinates and number on edges information into the proper   *
     * structures.                                                            */
    coords->xmin = xmin;
    coords->xmax = xmax;
    coords->ymin = ymin;
    coords->ymax = ymax;
    num_on_edges->xmin = xmin_edge;
    num_on_edges->xmax = xmax_edge;
    num_on_edges->ymin = ymin_edge;
    num_on_edges->ymax = ymax_edge;
} /* end of static void get_bb_from_scratch() */


static double get_net_cost(int inet, bbox_t* bbptr)
{
    /* Finds the cost due to one net by looking at its coordinate bounding  *
     * box.                                                                 */
    double ncost, crossing;

    /* Get the expected "crossing count" of a net, based on its number *
     * of pins.  Extrapolate for very large nets.                      */
    if (net[inet].num_pins > 50) {
        crossing = 2.7933 + 0.02616 * (net[inet].num_pins - 50);
        /*    crossing = 3.0;    Old value  */
    } else {
        crossing = cross_count[net[inet].num_pins - 1];
    }

    /* Could insert a check for xmin == xmax.  In that case, assume  *
     * connection will be made with no bends and hence no x-cost.    *
     * Same thing for y-cost.                                        */
    /* Cost = wire length along channel * cross_count / average      *
     * channel capacity.   Do this for x, then y direction and add.  */
    ncost = (bbptr->xmax - bbptr->xmin + 1) * crossing *
            chanx_place_cost_fac[bbptr->ymax][bbptr->ymin - 1];
    ncost += (bbptr->ymax - bbptr->ymin + 1) * crossing *
             chany_place_cost_fac[bbptr->xmax][bbptr->xmin - 1];
    return(ncost);
}


static void get_non_updateable_bb(int inet, bbox_t* bb_coord_new)
{
    /* Finds the bounding box of a net and stores its coordinates in the  *
     * bb_coord_new data structure.  This routine should only be called   *
     * for small nets, since it does not determine enough information for *
     * the bounding box to be updated incrementally later.                *
     * Currently assumes channels on both sides of the CLBs forming the   *
     * edges of the bounding box can be used.  Essentially, I am assuming *
     * the pins always lie on the outside of the bounding box.            */
    int k, xmax, ymax, xmin, ymin, x, y;
    x = blocks[net[inet].blocks[0]].x;
    y = blocks[net[inet].blocks[0]].y;
    xmin = x;
    ymin = y;
    xmax = x;
    ymax = y;

    for (k = 1; k < net[inet].num_pins; k++) {
        x = blocks[net[inet].blocks[k]].x;
        y = blocks[net[inet].blocks[k]].y;

        if (x < xmin) {
            xmin = x;
        } else if (x > xmax) {
            xmax = x;
        }

        if (y < ymin) {
            ymin = y;
        } else if (y > ymax) {
            ymax = y;
        }
    }

    /* Now I've found the coordinates of the bounding box.  There are no *
     * channels beyond num_of_columns and num_of_rows, so I want to clip to that.  As well,   *
     * since I'll always include the channel immediately below and the   *
     * channel immediately to the left of the bounding box, I want to    *
     * clip to 1 in both directions as well (since minimum channel index *
     * is 0).  See route.c for a channel diagram.                        */
    bb_coord_new->xmin = max(min(xmin, num_of_columns), 1);
    bb_coord_new->ymin = max(min(ymin, num_of_rows), 1);
    bb_coord_new->xmax = max(min(xmax, num_of_columns), 1);
    bb_coord_new->ymax = max(min(ymax, num_of_rows), 1);
}


static void update_bb(int inet, bbox_t* bb_coord_new, bbox_t
                      *bb_edge_new, int xold, int yold, int xnew, int ynew)
{
    /* Updates the bounding box of a net by storing its coordinates in    *
     * the bb_coord_new data structure and the number of blocks on each   *
     * edge in the bb_edge_new data structure.  This routine should only  *
     * be called for large nets, since it has some overhead relative to   *
     * just doing a brute force bounding box calculation.  The bounding   *
     * box coordinate and edge information for inet must be valid before  *
     * this routine is called.                                            *
     * Currently assumes channels on both sides of the CLBs forming the   *
     * edges of the bounding box can be used.  Essentially, I am assuming *
     * the pins always lie on the outside of the bounding box.            */
    /* IO blocks are considered to be one cell in for simplicity. */
    xnew = max(min(xnew, num_of_columns), 1);
    ynew = max(min(ynew, num_of_rows), 1);
    xold = max(min(xold, num_of_columns), 1);
    yold = max(min(yold, num_of_rows), 1);

    /* Check if I can update the bounding box incrementally. */

    if (xnew < xold) {                          /* Move to left. */

        /* Update the xmax fields for coordinates and number of edges first. */
        if (xold == bb_coords[inet].xmax) {       /* Old position at xmax. */
            if (bb_num_on_edges[inet].xmax == 1) {
                get_bb_from_scratch(inet, bb_coord_new, bb_edge_new);
                return;
            } else {
                bb_edge_new->xmax = bb_num_on_edges[inet].xmax - 1;
                bb_coord_new->xmax = bb_coords[inet].xmax;
            }
        } else {            /* Move to left, old postion was not at xmax. */
            bb_coord_new->xmax = bb_coords[inet].xmax;
            bb_edge_new->xmax = bb_num_on_edges[inet].xmax;
        }

        /* Now do the xmin fields for coordinates and number of edges. */

        if (xnew < bb_coords[inet].xmin) {    /* Moved past xmin */
            bb_coord_new->xmin = xnew;
            bb_edge_new->xmin = 1;
        } else if (xnew == bb_coords[inet].xmin) { /* Moved to xmin */
            bb_coord_new->xmin = xnew;
            bb_edge_new->xmin = bb_num_on_edges[inet].xmin + 1;
        } else {                                /* Xmin unchanged. */
            bb_coord_new->xmin = bb_coords[inet].xmin;
            bb_edge_new->xmin = bb_num_on_edges[inet].xmin;
        }
    }    /* End of move to left case. */
    else if (xnew > xold) {             /* Move to right. */

        /* Update the xmin fields for coordinates and number of edges first. */
        if (xold == bb_coords[inet].xmin) {   /* Old position at xmin. */
            if (bb_num_on_edges[inet].xmin == 1) {
                get_bb_from_scratch(inet, bb_coord_new, bb_edge_new);
                return;
            } else {
                bb_edge_new->xmin = bb_num_on_edges[inet].xmin - 1;
                bb_coord_new->xmin = bb_coords[inet].xmin;
            }
        } else {              /* Move to right, old position was not at xmin. */
            bb_coord_new->xmin = bb_coords[inet].xmin;
            bb_edge_new->xmin = bb_num_on_edges[inet].xmin;
        }

        /* Now do the xmax fields for coordinates and number of edges. */

        if (xnew > bb_coords[inet].xmax) {    /* Moved past xmax. */
            bb_coord_new->xmax = xnew;
            bb_edge_new->xmax = 1;
        } else if (xnew == bb_coords[inet].xmax) { /* Moved to xmax */
            bb_coord_new->xmax = xnew;
            bb_edge_new->xmax = bb_num_on_edges[inet].xmax + 1;
        } else {                                /* Xmax unchanged. */
            bb_coord_new->xmax = bb_coords[inet].xmax;
            bb_edge_new->xmax = bb_num_on_edges[inet].xmax;
        }
    }    /* End of move to right case. */
    else {          /* xnew == xold -- no x motion. */
        bb_coord_new->xmin = bb_coords[inet].xmin;
        bb_coord_new->xmax = bb_coords[inet].xmax;
        bb_edge_new->xmin = bb_num_on_edges[inet].xmin;
        bb_edge_new->xmax = bb_num_on_edges[inet].xmax;
    }

    /* Now account for the y-direction motion. */

    if (ynew < yold) {                  /* Move down. */

        /* Update the ymax fields for coordinates and number of edges first. */
        if (yold == bb_coords[inet].ymax) {       /* Old position at ymax. */
            if (bb_num_on_edges[inet].ymax == 1) {
                get_bb_from_scratch(inet, bb_coord_new, bb_edge_new);
                return;
            } else {
                bb_edge_new->ymax = bb_num_on_edges[inet].ymax - 1;
                bb_coord_new->ymax = bb_coords[inet].ymax;
            }
        } else {            /* Move down, old postion was not at ymax. */
            bb_coord_new->ymax = bb_coords[inet].ymax;
            bb_edge_new->ymax = bb_num_on_edges[inet].ymax;
        }

        /* Now do the ymin fields for coordinates and number of edges. */

        if (ynew < bb_coords[inet].ymin) {    /* Moved past ymin */
            bb_coord_new->ymin = ynew;
            bb_edge_new->ymin = 1;
        } else if (ynew == bb_coords[inet].ymin) { /* Moved to ymin */
            bb_coord_new->ymin = ynew;
            bb_edge_new->ymin = bb_num_on_edges[inet].ymin + 1;
        } else {                                /* ymin unchanged. */
            bb_coord_new->ymin = bb_coords[inet].ymin;
            bb_edge_new->ymin = bb_num_on_edges[inet].ymin;
        }
    }    /* End of move down case. */
    else if (ynew > yold) {             /* Moved up. */

        /* Update the ymin fields for coordinates and number of edges first. */
        if (yold == bb_coords[inet].ymin) {   /* Old position at ymin. */
            if (bb_num_on_edges[inet].ymin == 1) {
                get_bb_from_scratch(inet, bb_coord_new, bb_edge_new);
                return;
            } else {
                bb_edge_new->ymin = bb_num_on_edges[inet].ymin - 1;
                bb_coord_new->ymin = bb_coords[inet].ymin;
            }
        } else {              /* Moved up, old position was not at ymin. */
            bb_coord_new->ymin = bb_coords[inet].ymin;
            bb_edge_new->ymin = bb_num_on_edges[inet].ymin;
        }

        /* Now do the ymax fields for coordinates and number of edges. */

        if (ynew > bb_coords[inet].ymax) {    /* Moved past ymax. */
            bb_coord_new->ymax = ynew;
            bb_edge_new->ymax = 1;
        } else if (ynew == bb_coords[inet].ymax) { /* Moved to ymax */
            bb_coord_new->ymax = ynew;
            bb_edge_new->ymax = bb_num_on_edges[inet].ymax + 1;
        } else {                                /* ymax unchanged. */
            bb_coord_new->ymax = bb_coords[inet].ymax;
            bb_edge_new->ymax = bb_num_on_edges[inet].ymax;
        }
    }    /* End of move up case. */
    else {          /* ynew == yold -- no y motion. */
        bb_coord_new->ymin = bb_coords[inet].ymin;
        bb_coord_new->ymax = bb_coords[inet].ymax;
        bb_edge_new->ymin = bb_num_on_edges[inet].ymin;
        bb_edge_new->ymax = bb_num_on_edges[inet].ymax;
    }
}


struct s_pos {
    int x;
    int y;
} *pos;

static void initial_placement(pad_loc_t pad_loc_type,
                              char* pad_loc_file) /* FIXME */
{
    /* Randomly places the blocks to create an initial placement. */
    int k;
    int tsize = max(num_of_columns * num_of_rows,
                    2 * (num_of_columns + num_of_rows));
    pos = (struct s_pos*)my_malloc(tsize * sizeof(struct s_pos));

    /* Initialize all occupancy to zero. */
    int i = -1;
    int j = -1;
    for (i = 0; i <= num_of_columns + 1; i++) {
        for (j = 0; j <= num_of_rows + 1; j++) {
            clb[i][j].occ = 0;
        } /* clb[rows][columns] described FPGA Architecture */
    }  /* clb[0~(1-num_of_columns)~num_of_columns+1][0~(1-num_of_rows)~num_of_rows+1] */

    /* initialize all clb location from left to right, from bottom to top */
    int count = 0;
    for (i = 1; i <= num_of_columns; i++) {  /* from left to right */
        for (j = 1; j <= num_of_rows; j++) { /* from bottom to top */
            pos[count].x = i;
            pos[count].y = j;
            ++count; /* total num_of_columns * num_of_rows clbs clb locations */
        }
    }

    int iblk = -1;
    int choice = -1;
    for (iblk = 0; iblk < num_blocks; ++iblk) { /* num_blocks = plbs + io_blocks */
        if (blocks[iblk].type == CLB) { /* only place CLBs in center */
            choice = my_irand(count - 1); /* choice >= 1 && choice <= count-1*/
            clb[pos[choice].x][pos[choice].y].u.blocks = iblk;
            clb[pos[choice].x][pos[choice].y].occ = 1;

            /* Ensure randomizer doesn't pick this blocks again */
            pos[choice] = pos[count - 1]; /* overwrite used blocks position */
            /* I think it should be pos[count-1] = pos[choice] */
            --count;
        }
    }

    /* Now do the io blocks around the periphery */
    if (pad_loc_type == USER) {
        read_user_pad_loc(pad_loc_file);
    } else {
        /*========   Now place the I/O pads randomly. ========*/
        count = 0;
        for (i = 1; i <= num_of_columns; ++i) {
            /* initial I/O pad locations at BOTTEOM side*/
            pos[count].x = i;
            pos[count].y = 0;
            /* initial I/O pad locations at TOP side */
            pos[count + 1].x = i;
            pos[count + 1].y = num_of_rows + 1;
            count += 2;
        }
        for (j = 1; j <= num_of_rows; j++) {
            /* initial I/O pad locations at LEFT side */
            pos[count].x = 0;
            pos[count].y = j;
            /* initial I/O pad locations at RIGHT side */
            pos[count + 1].x = num_of_columns + 1;
            pos[count + 1].y = j;
            count += 2;
        }
        /* current time, the count == num_of_io_pads */
        for (iblk = 0; iblk < num_blocks; ++iblk) {
            if (blocks[iblk].type == INPAD || blocks[iblk].type == OUTPAD) {
                choice = my_irand(count - 1);

                int isubblk = clb[pos[choice].x][pos[choice].y].occ;

                clb[pos[choice].x][pos[choice].y].u.io_blocks[isubblk] = iblk;
                ++(clb[pos[choice].x][pos[choice].y].occ);
                /* In an I/O pad location of FPGA chip, it may accommodate more
                 * than 1 I/O pad. */
                if (clb[pos[choice].x][pos[choice].y].occ == io_rat) {
                    /* Ensure randomizer doesn't pick this blocks again */
                    pos[choice] = pos[count - 1]; /* overwrite used blocks position */
                    /* the "choice" location had used, so don't choose it again */
                    --count;
                }
            }
        }
    }    /* End randomly place IO blocks branch of if */

    /* All the blocks are placed now. Make the blocks array agree with the *
     * clb array.                                                         */
    for (i = 0; i <= num_of_columns + 1; ++i) {
        for (j = 0; j <= num_of_rows + 1; ++j) {
            if (clb[i][j].type == CLB && clb[i][j].occ == 1) {
                blocks[clb[i][j].u.blocks].x = i;
                blocks[clb[i][j].u.blocks].y = j;
            } else if (clb[i][j].type == IO) {
                for (k = 0; k < clb[i][j].occ; ++k) {
                    blocks[clb[i][j].u.io_blocks[k]].x = i;
                    blocks[clb[i][j].u.io_blocks[k]].y = j;
                }
            }
        }
    }

#ifdef VERBOSE
    printf("At end of initial_placement.\n");
    dump_clbs();
#endif
    free(pos);
} /* end of void initial_placement() */


static void free_fast_cost_update_structs(void)
{
    /* Frees the structures used to speed up evaluation of the nonlinear   *
     * congestion cost function.                                           */
    int i;

    for (i = 0; i <= num_of_rows; i++) {
        free(chanx_place_cost_fac[i]);
    }

    free(chanx_place_cost_fac);

    for (i = 0; i <= num_of_columns; i++) {
        free(chany_place_cost_fac[i]);
    }

    free(chany_place_cost_fac);
}


static void alloc_and_load_for_fast_cost_update(double place_cost_exp)
{
    /* Allocates and loads the chanx_place_cost_fac and chany_place_cost_fac *
     * arrays with the inverse of the average_number_of_tracks_per_channel   *
     * between [subhigh] and [sublow].  This is only useful for the cost     *
     * function that takes the length of the net bounding box in each        *
     * dimension divided by the average number of tracks in that direction.  *
     * For other cost functions, you don't have to bother calling this       *
     * routine; when using the cost function described above, however, you   *
     * must always call this routine after you call init_channel_t and before     *
     * you do any placement cost determination.  The place_cost_exp factor   *
     * specifies to what power the width of the channel should be taken --   *
     * larger numbers make narrower channels more expensive.                 */
    int low, high, i;

    /* Access arrays below as chan?_place_cost_fac[subhigh][sublow].  Since   *
     * subhigh must be greater than or equal to sublow, we only need to       *
     * allocate storage for the lower half of a matrix.                       */
    chanx_place_cost_fac = (double**)my_malloc((num_of_rows + 1) * sizeof(double*));
    for (i = 0; i <= num_of_rows; ++i) { /* why did it set 0~num_of_rows for channel_x? */
        chanx_place_cost_fac[i] = (double*)my_malloc((i + 1) * sizeof(double));
    }

    /* Why did it set 0~num_of_columns in channel_y? */
    chany_place_cost_fac = (double**)my_malloc((num_of_columns + 1) * sizeof(double*));
    for (i = 0; i <= num_of_columns; ++i) {
        chany_place_cost_fac[i] = (double*)my_malloc((i + 1) * sizeof(double));
    }

    /* First compute the number of tracks between channel high and channel *
     * low, inclusive, in an efficient manner.                             */
    chanx_place_cost_fac[0][0] = chan_width_x[0];

    for (high = 1; high <= num_of_rows; ++high) {
        chanx_place_cost_fac[high][high] = chan_width_x[high]; /* ? */

        for (low = 0; low < high; ++low) {
            chanx_place_cost_fac[high][low] = chanx_place_cost_fac[high-1][low]
                                              + chan_width_x[high]; /* ? */
        }
    }

    /* Now compute the inverse of the average_number_of_tracks_per_channel *
     * between high and low.  The cost function divides by the average     *
     * number of tracks per channel, so by storing the inverse I convert   *
     * this to a faster multiplication.  Take this final number to the     *
     * place_cost_exp power -- numbers other than one mean this is no      *
     * longer a simple "average number of tracks"; it is some power of     *
     * that, allowing greater penalization of narrow channels.             */
    for (high = 0; high <= num_of_rows; ++high)
        for (low = 0; low <= high; ++low) {
            chanx_place_cost_fac[high][low] = (high - low + 1.0) /
                                              chanx_place_cost_fac[high][low];
            chanx_place_cost_fac[high][low] =
                pow((double)chanx_place_cost_fac[high][low],
                    (double)place_cost_exp);
        }

    /* Now do the same thing for the y-directed channels.  First get the  *
     * number of tracks between channel high and channel low, inclusive.  */
    chany_place_cost_fac[0][0] = chan_width_y[0];
    for (high = 1; high <= num_of_columns; ++high) {
        chany_place_cost_fac[high][high] = chan_width_y[high];
        for (low = 0; low < high; ++low) {
            chany_place_cost_fac[high][low] = chany_place_cost_fac[high - 1][low]
                                              + chan_width_y[high];
        }
    }

    /* Now compute the inverse of the average number of tracks per channel *
     * between high and low.  Take to specified power.                     */
    for (high = 0; high <= num_of_columns; high++)
        for (low = 0; low <= high; low++) {
            chany_place_cost_fac[high][low] = (high - low + 1.) /
                                              chany_place_cost_fac[high][low];
            chany_place_cost_fac[high][low] =
                pow((double) chany_place_cost_fac[high][low],
                    (double) place_cost_exp);
        }
}


static void check_place(double bb_cost,
                        double timing_cost,
                        int place_cost_type,
                        int num_regions,
                        place_algorithm_t place_algorithm,
                        double delay_cost)
{
    /* Checks that the placement has not confused our data structures. *
     * i.e. the clb and blocks structures agree about the locations of  *
     * every blocks, blocks are in legal spots, etc.  Also recomputes   *
     * the final placement cost from scratch and makes sure it is      *
     * within roundoff of what we think the cost is.                   */
    int i, j, k, bnum;
    double bb_cost_check = comp_bb_cost(CHECK,
                                        place_cost_type,
                                        num_regions);
    printf("bb_cost recomputed from scratch is %g.\n",
           bb_cost_check);

    int error = 0;
    if (fabs(bb_cost_check - bb_cost) > bb_cost * ERROR_TOL) {
        printf("Error:  bb_cost_check: %g and bb_cost: %g differ in check_place.\n",
               bb_cost_check, bb_cost);
        error++;
    }

    double timing_cost_check = 0.0;
    double delay_cost_check = 0.0;
    if (place_algorithm == NET_TIMING_DRIVEN_PLACE
          || place_algorithm == PATH_TIMING_DRIVEN_PLACE
          || place_algorithm == NEW_TIMING_DRIVEN_PLACE) {
        if (place_algorithm == NEW_TIMING_DRIVEN_PLACE) {
            compute_timing_driven_costs_by_path_algo(&timing_cost_check,
                                                     &delay_cost_check);
        } else {
            compute_timing_driven_cost_by_orig_algo(&timing_cost_check,
                                        &delay_cost_check);
        }
        printf("timing_cost recomputed from scratch is %g. \n",
               timing_cost_check);

        if (fabs(timing_cost_check - timing_cost) > timing_cost * ERROR_TOL) {
            printf("Error:  timing_cost_check: %g and timing_cost: "
                   "%g differ in check_place.\n",
                   timing_cost_check,
                   timing_cost);
            ++error;
        }

        printf("delay_cost recomputed from scratch is %g. \n", delay_cost_check);

        if (fabs(delay_cost_check - delay_cost) > delay_cost * ERROR_TOL) {
            printf("Error:  delay_cost_check: %g and delay_cost: "
                   "%g differ in check_place.\n",
                   delay_cost_check,
                   delay_cost);
            ++error;
        }
    } /* end of check timing_driven_placement cost */

    static int* bdone;
    bdone = (int*)my_malloc(num_blocks * sizeof(int));

    for (i = 0; i < num_blocks; i++) {
        bdone[i] = 0;
    }

    /* Step through clb array. Check it against blocks array. */
    for (i = 0; i <= num_of_columns + 1; i++)
        for (j = 0; j <= num_of_rows + 1; j++) {
            if (clb[i][j].occ == 0) {
                continue;
            }

            if (clb[i][j].type == CLB) {
                bnum = clb[i][j].u.blocks;

                if (blocks[bnum].type != CLB) {
                    printf("Error:  blocks %d type does not match clb(%d,%d) type.\n",
                           bnum, i, j);
                    error++;
                }

                if ((blocks[bnum].x != i) || (blocks[bnum].y != j)) {
                    printf("Error:  blocks %d location conflicts with clb(%d,%d)"
                           "data.\n", bnum, i, j);
                    error++;
                }

                if (clb[i][j].occ > 1) {
                    printf("Error: clb(%d,%d) has occupancy of %d\n",
                           i, j, clb[i][j].occ);
                    error++;
                }

                bdone[bnum]++;
            } else { /* IO blocks */
                if (clb[i][j].occ > io_rat) {
                    printf("Error:  clb(%d,%d) has occupancy of %d\n", i, j,
                           clb[i][j].occ);
                    error++;
                }

                for (k = 0; k < clb[i][j].occ; k++) {
                    bnum = clb[i][j].u.io_blocks[k];

                    if ((blocks[bnum].type != INPAD) && blocks[bnum].type != OUTPAD) {
                        printf("Error:  blocks %d type does not match clb(%d,%d) type.\n",
                               bnum, i, j);
                        error++;
                    }

                    if ((blocks[bnum].x != i) || (blocks[bnum].y != j)) {
                        printf("Error:  blocks %d location conflicts with clb(%d,%d)"
                               "data.\n", bnum, i, j);
                        error++;
                    }

                    bdone[bnum]++;
                }
            }
        }

    /* Check that every blocks exists in the clb and blocks arrays somewhere. */
    for (i = 0; i < num_blocks; i++)
        if (bdone[i] != 1) {
            printf("Error:  blocks %d listed %d times in data structures.\n",
                   i, bdone[i]);
            error++;
        }

    free(bdone);

    if (error == 0) {
        printf("\nCompleted placement consistency check successfully.\n\n");
    } else {
        printf("\nCompleted placement consistency check, %d Errors found.\n\n",
               error);
        printf("Aborting program.\n");
        exit(1);
    }
}


void read_place(char* place_file,
                char* net_file,
                char* arch_file,
                placer_opts_t placer_opts,
                router_opts_t router_opts,
                chan_width_distr_t chan_width_dist,
                detail_routing_arch_t det_routing_arch,
                segment_info_t* segment_inf,
                timing_info_t timing_inf,
                subblock_data_t* subblock_data_ptr)
{
    /* Reads in a previously computed placement of the circuit.  It      *
     * checks that the placement corresponds to the current architecture *
     * and netlist file.                                                 */
    char msg[BUFSIZE];
    int chan_width_factor, num_connections, inet, ipin;
    double bb_cost, delay_cost, timing_cost, est_crit;
    double** dummy_x, **dummy_y;
    double** net_slack = NULL;
    double** net_delay = NULL;
    double** remember_net_delay_original_ptr; /*used to free net_delay if it is re-assigned*/
    remember_net_delay_original_ptr = NULL; /*prevents compiler warning*/

    if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
          || placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE
          || placer_opts.place_algorithm == NEW_TIMING_DRIVEN_PLACE
          || placer_opts.enable_timing_computations) {
        /*this must be called before alloc_and_load_placement_structs *
         *and parse_placement_file since it modifies the structures*/
        alloc_and_load_timing_graph(placer_opts,
                                    timing_inf,
                                    *subblock_data_ptr);
        net_slack = alloc_net_slack();

        alloc_delay_lookup_matrixes_and_criticalities(placer_opts,
                                                      *subblock_data_ptr,
                                                      chan_width_dist,
                                                      timing_inf,
                                                      router_opts,
                                                      det_routing_arch,
                                                      segment_inf,
                                                      &net_delay);
        num_connections = count_connections();
        remember_net_delay_original_ptr = net_delay;
    } else {
        num_connections = 0;
    }

    /* First read in the placement.   */
    parse_placement_file(place_file,
                         net_file,
                         arch_file);
    /* Load the channel occupancies and cost factors so that:   *
     * (1) the cost check will be OK, and                       *
     * (2) the geometry will draw correctly.                    */
    chan_width_factor = placer_opts.place_chan_width;
    init_channel_t(chan_width_factor, chan_width_dist);
    /* NB:  dummy_x and dummy_y used because I'll never use the old_place_occ *
     * arrays in this routine.  I need the placement structures loaded for    *
     * comp_cost and check_place to work.                                     */
    alloc_and_load_placement_structs(placer_opts.place_cost_type,
                                     placer_opts.num_regions,
                                     placer_opts.place_cost_exp,
                                     &dummy_x,  &dummy_y, placer_opts);
    /* Need cost in order to call check_place. */
    bb_cost = comp_bb_cost(NORMAL, placer_opts.place_cost_type,
                           placer_opts.num_regions);

    if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
          || placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE
          || placer_opts.place_algorithm == NEW_TIMING_DRIVEN_PLACE
          || placer_opts.enable_timing_computations) {
        for (inet = 0; inet < num_nets; ++inet)
            for (ipin = 1; ipin < net[inet].num_pins; ++ipin) {
                timing_place_crit[inet][ipin] = 0;    /*dummy crit values*/
            }

        net_delay = point_to_point_delay_cost;  /*this keeps net_delay up to date with the *
                         *same values that the placer is using     */
        load_timing_graph_net_delays(net_delay);
        est_crit = calc_all_vertexs_arr_req_time(0);
        compute_net_slacks(net_slack);

        compute_timing_driven_cost_by_orig_algo(&timing_cost,
                                    &delay_cost);  /*set up point_to_point_delay_cost*/

        printf("Placement. bb_cost: %g  delay_cost: %g.\n\n",
               bb_cost, delay_cost);
#ifdef PRINT_SINK_DELAYS
        print_sink_delays("Placement_Sink_Delays.echo");
#endif
#ifdef PRINT_NET_SLACKS
        print_net_slack("Placement_Net_Slacks.echo", net_slack);
#endif
#ifdef PRINT_PLACE_CRIT_PATH
        print_critical_path("Placement_Crit_Path.echo");
#endif
        printf("Placement Estimated Crit Path Delay: %g\n\n", est_crit);
    } else {
        timing_cost = 0;
        delay_cost = 0;
        printf("Placement bb_cost is %g.\n", bb_cost);
    }

    check_place(bb_cost, timing_cost, placer_opts.place_cost_type, placer_opts.num_regions,
                placer_opts.place_algorithm, delay_cost);
    free_placement_structs(placer_opts.place_cost_type, placer_opts.num_regions,
                           dummy_x, dummy_y, placer_opts);

    if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE ||
            placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE ||
            placer_opts.enable_timing_computations) {
        net_delay = remember_net_delay_original_ptr;
        free_lookups_and_criticalities(&placer_opts,
                                       &net_delay,
                                       &net_slack);
    }

    init_draw_coords((double) chan_width_factor);
    sprintf(msg, "Placement from file %s.  bb_cost %g.", place_file,
            bb_cost);
    update_screen(MAJOR, msg, PLACEMENT, FALSE);
}

