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
 * FIXME: Now it was very important for PATH algorithm, it will *
 * store all nets'(and its subnets) timing_crit value, which was*
 * calcuate by PATH algorithm.                                  */
double** timing_place_crit;

static struct s_linked_vptr* timing_place_crit_chunk_list_head;
static struct s_linked_vptr* net_delay_chunk_list_head;


/******** prototypes ******************/
static double** alloc_crit(struct s_linked_vptr** chunk_list_head_ptr);

static void free_crit(struct s_linked_vptr** chunk_list_head_ptr);
/**************************************/

static double** alloc_crit(struct s_linked_vptr** chunk_list_head_ptr)
{
    /* Allocates space for the timing_place_crit data structure *
     * [0..num_nets-1][1..num_pins-1].  I chunk the data to save space on large    *
     * problems.                                                                   */
    *chunk_list_head_ptr = NULL;

    double* tmp_ptr;
    int chunk_bytes_avail = 0;
    char* chunk_next_avail_mem = NULL;

    /* double load_crit[0..num_nets-1][1..num_pins-1] */
    double** local_crit = (double**)my_malloc(num_nets * sizeof(double*));

    int inet = -1;
    for (inet = 0; inet < num_nets; inet++) {
        tmp_ptr = (double*)my_chunk_malloc((net[inet].num_pins-1) * sizeof(double),
                                           chunk_list_head_ptr,
                                           &chunk_bytes_avail,
                                           &chunk_next_avail_mem);
        local_crit[inet] = tmp_ptr - 1; /* only need [1..num_pins-1] */
    }

    return local_crit;
}

/**************************************/
static void free_crit(struct s_linked_vptr** chunk_list_head_ptr)
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
            int inode = tnodes_at_level[ilevel].list[i];
            int num_edges = tnode[inode].num_edges;
            if (num_edges == 0) {   /* sink */
                fprintf(fp, "%g\n", tnode[inode].arr_time);
            }
        }
    }

    fclose(fp);
}

/************************************************************************
 * FIXME: Calculate all tedges's criticality in Timing_Analyze_Graph.   *
 *        Set criticality values of all nets. It assumes that net_slack *
 *        contains correct values, ie. assumes that load_net_slack has  *
 *        been called.                                                  *
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

/* FIXME: alloated all needed delay lookup matrix for timing-driven placement */
void alloc_lookup_and_criticalities(t_chan_width_dist chan_width_dist,
                                     struct s_router_opts  router_opts,
                                     struct s_det_routing_arch det_routing_arch,
                                     t_segment_inf* segment_inf,
                                     t_timing_inf  timing_inf,
                                     t_subblock_data subblock_data,
                                     double*** net_delay,
                                     double*** net_slack)
{
    /* initial Timing_Analyze_Graph for PATH_TIMING_DRIVEN_PLACE and calculate *
     * all nets' slack.                                                        */
    (*net_slack) = alloc_and_load_timing_graph(timing_inf,
                                               subblock_data);

    /* allocate delay for TIMING_DRIVEN_PLACE, the net_slack was depend on *
     * the net_delay value.                                                */
    (*net_delay) = alloc_net_delay(&net_delay_chunk_list_head);

    /* compute 4 delay lookup matrixes for TIMING_DRIVEN_PLACEMENT */
    compute_delay_lookup_tables(router_opts,
                                det_routing_arch,
                                segment_inf,
                                timing_inf,
                                chan_width_dist,
                                subblock_data);

    /* allocate double** timing_crit[][] for */
    timing_place_crit = alloc_crit(&timing_place_crit_chunk_list_head);
} /* end of allocate 4 delay-lookup-matrix and timing_crit. */

/**************************************/
void free_lookups_and_criticalities(double*** net_delay, double*** net_slack)
{
    free(timing_place_crit);
    free_crit(&timing_place_crit_chunk_list_head);
    free_timing_graph(*net_slack);
    free_net_delay(*net_delay,
                   &net_delay_chunk_list_head);
}
/**************************************/

