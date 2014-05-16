#ifndef PLACE_H
#define PLACE_H

#include "vpr_types.h"


/* For comp_cost.  NORMAL means use the method that generates updateable  *
 * bounding boxes for speed.  CHECK means compute all bounding boxes from *
 * scratch using a very simple routine to allow checks of the other       *
 * costs.                                                                 */
enum cost_methods {
    NORMAL, CHECK
};


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

void run_low_temperature_place(const placer_opts_t   placer_opts,
                               const int*   pins_on_block,
                               placer_paras_t*  placer_paras_ptr,
                               double**  old_region_occ_x,
                               double**  old_region_occ_y,
                               double**  net_slack,
                               double**  net_delay,
                               placer_costs_t*  placer_cost_ptr);

void perform_timing_analyze(const placer_opts_t* placer_opts_ptr,
                            double**  net_delay,
                            double**  net_slack,
                            placer_paras_t*  placer_paras_ptr,
                            placer_costs_t*  placer_costs_ptr);

void  update_timing_cost_after_swap(const placer_opts_t*  placer_opts_ptr,
                                    const int inner_iter,
                                    double**  net_delay,
                                    double**  net_slack,
                                    placer_paras_t*  placer_paras_ptr,
                                    placer_costs_t*  placer_costs_ptr);

void compute_timing_driven_cost(const placer_opts_t*  placer_opts_ptr,
                                const placer_paras_t* placer_paras_ptr,
                                double** net_slack,
                                int**    block_pin_to_tnode,
                                placer_costs_t*  placer_costs_ptr);

void  update_place_cost_after_max_move_times(const placer_opts_t* placer_opts_ptr,
                                             placer_costs_t*  placer_costs_ptr);

void compute_timing_driven_cost_in_outer_loop(const placer_opts_t* placer_opts_ptr,
                                              placer_costs_t*  placer_costs_ptr);

void update_place_costs_by_success_sum(const placer_paras_t* placer_paras,
                                       placer_costs_t*  placer_opts_ptr);

/* New change the following functions to global functions due to place_parallel */
int count_connections(void);

void compute_net_pin_index_values(void);

double starting_temperature(const annealing_sched_t annealing_sched,
                            const int* pins_on_block,
                            const placer_opts_t   placer_opts,
                            placer_paras_t* placer_paras_ptr,
                            double**  old_region_occ_x,
                            double**  old_region_occ_y,
                            placer_costs_t* placer_costs_ptr);

/* FIXME: Try to swap a pair of plbs or io_pads randomly*/
/* Picks some blocks and moves it to another spot. If this spot had occupied *
 * , switch the blocks. Assess the change in cost function, and accept or   *
 * reject the move. If rejected, return 0, else return 1. Pass back the new *
 * value of the cost function. rlim is the range_limit. pins_on_block gives *
 * the number of pins on each type of blocks(improves efficiency). Pins on   *
 * each type of blocks(improves efficiency).                                 */
int try_swap(const placer_paras_t* placer_paras_ptr,
             const placer_opts_t   placer_opts,
             const int*  pins_on_block,
             double**  old_region_occ_x,
             double**  old_region_occ_y,
             placer_costs_t*  placer_costs_ptr);

/* Finds the cost from scratch. Done only when the placement has been     *
 * radically changed(i.e. after initial placement). Otherwise find the    *
 * cost change Incrementally. If method check is NORMAL, we find bounding *
 * -boxes that are updateable for the larger nets. If the method is CHECK,*
 * all bounding-boxes are found via the non_updateable_bb routine, to     *
 * provide a cost which can be used to chekc the correctness of the other *
 * routine.                                                               */
double compute_bb_cost(int method,
                       int place_cost_type,
                       int num_regions);

/* I change this function from static to global function. Due to some functions
 * was need by both single-thread and multi-threads */
void alloc_and_load_placement_structs(const placer_opts_t* placer_opts_ptr,
                                      double*** old_region_occ_x,
                                      double*** old_region_occ_y);

void initial_placement(pad_loc_t pad_loc_type,
                       char* pad_loc_file);

void check_place(placer_opts_t*  placer_opts_ptr,
                 placer_costs_t* placer_costs_ptr);

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
#endif

