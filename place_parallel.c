#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "globals.h"
#include "place_and_route.h"
#include "timing_place.h"
#include "place_parallel.h"
#include "timing_place_lookup.h"
#include "path_delay.h"
#include "path_delay2.h"
#include "net_delay.h"
#include "read_place.h"
#include "util.h"
#include "place.h"
#include "vpr_utils.h"
#include "draw.h"


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

void try_place_by_multi_threads(const char*     netlist_file,
                                placer_opts_t   placer_opts,
                                annealing_sched_t  annealing_sched,
                                chan_width_distr_t chan_width_dist,
                                router_opts_t      router_opts,
                                detail_routing_arch_t  det_routing_arch,
                                segment_info_t*  segment_inf_ptr,
                                timing_info_t    timing_inf,
                                subblock_data_t* subblock_data_ptr)
{
    /* ATTENTION, WHEN PLACEMENT USING MULTI-THREADS, THE INITIAL PLACEMENT WAS *
     * SAME WITH PLACEMENT USING SINGLE-THREAD.                                 */

    /* Allocated here because  it goes into timing-critical code where
     * each memory allocation is expensive. */
    int*  x_lookup = (int*)my_malloc(num_grid_columns * sizeof(int));

    /* used ot free net_delay if it is re-assigned */
    double**  remember_net_delay_original_ptr = NULL;
    double**  net_slack = NULL;
    double**  net_delay = NULL;
    if (NET_TIMING_DRIVEN_PLACE == placer_opts.place_algorithm
          || PATH_TIMING_DRIVEN_PLACE == placer_opts.place_algorithm
          || NEW_TIMING_DRIVEN_PLACE == placer_opts.place_algorithm
          || placer_opts.enable_timing_computations) {
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
                                                      segment_inf_ptr,
                                                      &net_delay);
        remember_net_delay_original_ptr = net_delay;
    }

    placer_paras_t*  placer_paras_ptr = init_placer_paras();
    placer_paras_ptr->m_width_factor = placer_opts.place_chan_width;

    if (placer_opts.pad_loc_type == FREE) {
        placer_paras_ptr->m_fixed_pins = FALSE;
    } else {
        placer_paras_ptr->m_fixed_pins = TRUE;
    }

    init_channel_t(placer_paras_ptr->m_width_factor,
                   chan_width_dist);

    double** old_region_occ_x = NULL;
    double** old_region_occ_y = NULL;
    alloc_and_load_placement_structs(&placer_opts,
                                     &old_region_occ_x,
                                     &old_region_occ_y);

    /* 2), run initial_placement  */
    initial_placement(placer_opts.pad_loc_type,
                      placer_opts.pad_loc_file);

    init_draw_coords((double)placer_paras_ptr->m_width_factor);

    int  pins_on_block[3];
    pins_on_block[CLB_TYPE] = pins_per_clb;
    pins_on_block[INPAD_TYPE] = 1;
    pins_on_block[OUTPAD_TYPE] = 1;

    /* 3) compute wirelength cost */
    placer_costs_t*  placer_costs_ptr = init_placer_costs();
    placer_costs_ptr->m_bb_cost = compute_bb_cost(NORMAL,
                                                  placer_opts.place_cost_type,
                                                  placer_opts.num_regions);

    placer_paras_ptr->m_outer_crit_iter_count = 0;
    if (placer_opts.place_algorithm == NET_TIMING_DRIVEN_PLACE
          || placer_opts.place_algorithm == PATH_TIMING_DRIVEN_PLACE
          || placer_opts.place_algorithm == NEW_TIMING_DRIVEN_PLACE) {
        placer_paras_ptr->m_crit_exponent = placer_opts.td_place_exp_first;

        compute_net_pin_index_values();

        placer_paras_ptr->m_num_connections = count_connections();
    }  /* end of TIMING_DRIVEN_PLACEMENT */

    /***************   Parallel Placement Starting   *******************/

    /***************   Parallel Placement Ending     *******************/
    run_low_temperature_place(placer_opts,
                              pins_on_block,
                              placer_paras_ptr,
                              old_region_occ_x,
                              old_region_occ_y,
                              net_slack,
                              net_delay,
                              placer_costs_ptr);

    /* After Placement, Don't free memory */
    free_placer_costs(placer_costs_ptr);
    free_placer_paras(placer_paras_ptr);
}

