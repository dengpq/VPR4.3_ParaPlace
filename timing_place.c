#include <stdio.h>
#include <math.h>
#include "util.h"
#include "globals.h"
#include "path_delay.h"
#include "path_delay2.h"
#include "net_delay.h"
#include "timing_place_lookup.h"
#include "timing_place.h"

/* double timing_place_crit[0..num_nets-1][1..num_pins-1].      *
 * it store the Timing_Criticality of all subnets in netlist.   *
 * For PATH_TIMING_DRIVEN_PLACE and NET_TIMING_DRIVEN_PLACE     */
double** timing_place_crit;

/* these following variable were new added by Deng Pengqiu for PATH algorithm */
/* NEW_TIMING_DRIVEN_PLACE  [0...num_nets-1][1...num_pins-1] */
double** subnet_local_crit_weight; /* [inet][ipin] */


static linked_vptr_t*  timing_place_crit_chunk_list_head;
static linked_vptr_t*  net_delay_chunk_list_head;

/******** prototypes ******************/
/* For NET_TIMING_DRIVEN_PLACE and PATH_TIMING_DRIVEN_PLACE */
static double** alloc_all_nets_crit(linked_vptr_t** chunk_list_head_ptr);

/*******************************************************
 *  double** subnet_local_crit_weight; [ipin][inet]    *
 *  For NEW_TIMING_DRIVEN_PLACE                        *
 ******************************************************/
static double** alloc_all_nets_local_crit_weight(linked_vptr_t** chunk_list_head_ptr);

static void free_crit(linked_vptr_t** chunk_list_head_ptr);

/******************************************************************************/
static double** alloc_all_nets_crit(linked_vptr_t** chunk_list_head_ptr)
{
    /* Allocates space for the timing_place_crit data structure *
     * [0..num_nets-1][1..num_pins-1].  I chunk the data to save space on large    *
     * problems.                                                                   */
    *chunk_list_head_ptr = NULL;

    int chunk_bytes_avail = 0;
    char* chunk_next_avail_mem = NULL;

    /* double load_crit[0..num_nets-1][1..num_pins-1] */
    double** local_crit = (double**)my_malloc(num_nets * sizeof(double*));

    int inet = -1;
    for (inet = 0; inet < num_nets; ++inet) {
        double* tmp_ptr = (double*)my_chunk_malloc((net[inet].num_pins-1)
                                                     * sizeof(double),
                                                   chunk_list_head_ptr,
                                                   &chunk_bytes_avail,
                                                   &chunk_next_avail_mem);
        local_crit[inet] = tmp_ptr - 1; /* only need [1..num_pins-1] */
    }

    return local_crit;
}  /* end of static double** alloc_all_nets_crit(chunk_list_head_ptr) */

/* Allocates space for the subnet_local_crit_weight data structure */
static double** alloc_all_nets_local_crit_weight(linked_vptr_t** chunk_list_head_ptr)
{
    /* [0..num_nets-1][1..num_pins-1]. I chunk the data to save space on large *
     * problems.                                                              */
    *chunk_list_head_ptr = NULL;

    int   chunk_bytes_avail = 0;
    char* chunk_next_avail_mem = NULL;

    /* double load_crit[0..num_nets-1][1..num_pins-1] */
    double** local_crit_weight = (double**)my_malloc(num_nets * sizeof(double*));

    int inet = -1;
    for (inet = 0; inet < num_nets; ++inet) {
        double* tmp_ptr = (double*)my_chunk_malloc((net[inet].num_pins-1)
                                                     * sizeof(double),
                                                   chunk_list_head_ptr,
                                                   &chunk_bytes_avail,
                                                   &chunk_next_avail_mem);
        local_crit_weight[inet] = tmp_ptr - 1; /* only need [1..num_pins-1] */
    }

    return local_crit_weight;
}  /* end of static double** alloc_all_nets_crit(chunk_list_head_ptr) */

static void free_crit(linked_vptr_t** chunk_list_head_ptr)
{
    free_chunk_memory(*chunk_list_head_ptr);
    *chunk_list_head_ptr = NULL;
}

/**************************************/
void print_sink_delays(char* fname)
{
    FILE* fp = my_fopen(fname, "w", 0);
    int ilevel = -1;
    for (ilevel = num_tnode_levels - 1; ilevel >= 0; --ilevel) {
        int num_at_level = tnodes_at_level[ilevel].nelem;

        int i = -1;
        for (i = 0; i < num_at_level; ++i) {
            int ivex = tnodes_at_level[ilevel].list[i];
            int num_edges = vertexes[ivex].num_edges;
            if (num_edges == 0) {   /* sink */
                fprintf(fp, "%g\n", vertexes[ivex].arr_time);
            }
        }
    }

    fclose(fp);
}

/************************************************************************
 * FIXME: Calculate all tedges's criticality in Timing_Analyze_Graph.   *
 *        Set criticality values of all nets. It assumes that net_slack *
 *        contains correct values, ie. assumes that                     *
 *        calc_all_vertexs_arr_req_time has been called.               *
 *  crit(source, ipin) = (1 - slack(source,ipin)/max_delay) ^ exponent  *
 ************************************************************************/
void load_criticalities(double** net_slack,
                        double crit_delay,
                        double crit_exponent)
{
    int inet = -1;
    for (inet = 0; inet < num_nets; ++inet) {
        if (inet == OPEN || is_global[inet]) {
            continue;
        }

        int ipin = 0;
        for (ipin = 1; ipin < net[inet].num_pins; ++ipin) {
        /* clip the criticality to never go negative(could happen for a  *
         * constant generator since it's slack is huge).                 *
         * criticality<SOURCE, ipin> = 1 - slack<SOURCE, ipn>/max_delay  */
            double pin_crit = max(1 - net_slack[inet][ipin] / crit_delay,
                                  0.0);
            timing_place_crit[inet][ipin] = pow(pin_crit,
                                                crit_exponent);
        }
    }
} /* end of void load_criticalities() */

/* FIXME: alloated all needed delay-lookup-matrix for timing-driven placement */
void alloc_delay_lookup_matrixes_and_criticalities(placer_opts_t      placer_opts,
                                                   subblock_data_t    subblock_data,
                                                   chan_width_distr_t chan_width_dist,
                                                   timing_info_t      timing_inf,
                                                   router_opts_t      router_opts,
                                                   detail_routing_arch_t det_routing_arch,
                                                   segment_info_t*    segment_inf,
                                                   double***          net_delay)
{
    /* allocate delay for TIMING_DRIVEN_PLACE, the net_slack was depend on *
     * the net_delay value.                                                */
    (*net_delay) = alloc_net_delay(&net_delay_chunk_list_head);

    /* compute 4 delay-lookup-matrixes for TIMING_DRIVEN_PLACEMENT *
     * delta_clb_to_clb, delta_inpad_to_clb, delta_clb_to_outpad,  *
     * delta_inpad_to_outpad.    */
    alloc_and_compute_delay_lookup_matrixes(router_opts,
                                            det_routing_arch,
                                            segment_inf,
                                            timing_inf,
                                            chan_width_dist,
                                            subblock_data);

    /* FIXME: allocate double** timing_crit[][] for */
    if (placer_opts.place_algorithm == NEW_TIMING_DRIVEN_PLACE) {
        subnet_local_crit_weight =
            alloc_all_nets_local_crit_weight(&timing_place_crit_chunk_list_head);
    } else { /* PATH_TIMING_DRIVEN_PLACE or NET_TIMING_DRIVEN_PLACE */
        timing_place_crit =
                      alloc_all_nets_crit(&timing_place_crit_chunk_list_head);
    }
} /* end of allocate 4 delay-lookup-matrix and timing_crit. */

/**************************************/
void free_lookups_and_criticalities(const placer_opts_t* placer_opts_ptr,
                                    double*** net_delay,
                                    double*** net_slack)
{
    if (NEW_TIMING_DRIVEN_PLACE == placer_opts_ptr->place_algorithm) {
        free(subnet_local_crit_weight);
    } else {
        free(timing_place_crit);
    }
    free_crit(&timing_place_crit_chunk_list_head);
    free_timing_graph(*net_slack);
    free_net_delay(*net_delay,
                   &net_delay_chunk_list_head);
    /* Now added free data structure used for NEW_TIMING_DRIVEN_PLACE */
    free(front_crit_path_through_pin);
    front_crit_path_through_pin = NULL;
    free(behind_crit_path_through_pin);
    behind_crit_path_through_pin = NULL;
    /* then free double** subnet_local_crit_weight[0..num_nets-1][0..num_pins-1] */
}
/**************************************/

