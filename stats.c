#include <stdio.h>
#include <math.h>
#include "util.h"
#include "vpr_types.h"
#include "globals.h"
#include "rr_graph_area.h"
#include "segment_stats.h"
#include "stats.h"
#include "net_delay.h"
#include "path_delay.h"


/********************** Subroutines local to this module *********************/

static void load_channel_occupancies(int** chanx_occ, int** chany_occ);

static void get_num_bends_and_length(int inet, int* bends, int* length, int
                                     *segments);

static void get_length_and_bends_stats(void);

static void gechannel_tnel_occupancy_stats(void);



/************************* Subroutine definitions ****************************/


void routing_stats(boolean full_stats, router_types_t route_type,
                   int num_switch, segment_info_t* segment_inf, int num_segment,
                   double R_minW_nmos, double R_minW_pmos, boolean timing_analysis_enabled,
                   double** net_slack, double** net_delay)
{
    /* Prints out various statistics about the current routing.  Both a routing *
     * and an rr_graph must exist when you call this routine.                   */
    double T_crit;
    get_length_and_bends_stats();
    gechannel_tnel_occupancy_stats();

    if (route_type == DETAILED) {
        count_routing_transistors(num_switch, R_minW_nmos, R_minW_pmos);
        get_segment_usage_stats(num_segment, segment_inf);

        if (timing_analysis_enabled) {
            load_net_delay_from_routing(net_delay);
#ifdef PRINT_NET_DELAYS
            print_net_delay(net_delay, "net_delay.echo");
#endif
            load_timing_graph_net_delays(net_delay);
            T_crit = calc_all_vertexs_arr_req_time(0);
            compute_net_slacks(net_slack);

#ifdef PRINT_TIMING_GRAPH
            print_timing_graph("timing_graph.echo");
#endif
#ifdef PRINT_NET_SLACKS
            print_net_slack("net_slack.echo", net_slack);
#endif
            printf("\n");
            print_critical_path("critical_path.echo");
            printf("Critical Path: %g (s)\n", T_crit);
        }
    }

    if (full_stats == TRUE) {
        print_wirelen_prob_dist();
    }
}


static void get_length_and_bends_stats(void)
{
    /* Figures out maximum, minimum and average number of bends and net length   *
     * in the routing.                                                           */
    int inet, bends, total_bends, max_bends;
    int length, total_length, max_length;
    int segments, total_segments, max_segments;
    double av_bends, av_length, av_segments;
    max_bends = 0;
    total_bends = 0;
    max_length = 0;
    total_length = 0;
    max_segments = 0;
    total_segments = 0;

    for (inet = 0; inet < num_nets; inet++) {
        if (is_global[inet] == FALSE) {       /* Globals don't count. */
            get_num_bends_and_length(inet, &bends, &length, &segments);
            total_bends += bends;
            max_bends = max(bends, max_bends);
            total_length += length;
            max_length = max(length, max_length);
            total_segments += segments;
            max_segments = max(segments, max_segments);
        }
    }

    av_bends = (double) total_bends / (double)(num_nets - num_globals);
    printf("\nAverage number of bends per net: %#g  Maximum # of bends: %d\n\n",
           av_bends, max_bends);
    av_length = (double) total_length / (double)(num_nets - num_globals);
    printf("Wirelength results (all in units of 1 clb segments):\n");
    printf("\tTotal wirelength: %d   Average net length: %#g\n", total_length,
           av_length);
    printf("\tMaximum net length: %d\n\n", max_length);
    av_segments = (double) total_segments / (double)(num_nets - num_globals);
    printf("Wirelength results in terms of physical segments:\n");
    printf("\tTotal wiring segments used: %d   Av. wire segments per net: "
           "%#g\n", total_segments, av_segments);
    printf("\tMaximum segments used by a net: %d\n\n", max_segments);
}


static void gechannel_tnel_occupancy_stats(void)
{
    /* Determines how many tracks are used in each channel.                    */
    int i, j, max_occ, total_x, total_y;
    double av_occ;
    int** chanx_occ;   /* [1..num_grid_columns][0..num_grid_rows] */
    int** chany_occ;   /* [0..num_grid_columns][1..num_grid_rows] */
    chanx_occ = (int**) alloc_matrix(1, num_grid_columns, 0, num_grid_rows, sizeof(int));
    chany_occ = (int**) alloc_matrix(0, num_grid_columns, 1, num_grid_rows, sizeof(int));
    load_channel_occupancies(chanx_occ, chany_occ);
    printf("\nX - Directed channels:\n\n");
    printf("j\tmax m_usage\tav_occ\t\tcapacity\n");
    total_x = 0;

    for (j = 0; j <= num_grid_rows; j++) {
        total_x += chan_width_x[j];
        av_occ = 0.;
        max_occ = -1;

        for (i = 1; i <= num_grid_columns; i++) {
            max_occ = max(chanx_occ[i][j], max_occ);
            av_occ += chanx_occ[i][j];
        }

        av_occ /= num_grid_columns;
        printf("%d\t%d\t%-#9g\t%d\n", j, max_occ, av_occ, chan_width_x[j]);
    }

    printf("\nY - Directed channels:\n\n");
    printf("i\tmax m_usage\tav_occ\t\tcapacity\n");
    total_y = 0;

    for (i = 0; i <= num_grid_columns; i++) {
        total_y += chan_width_y[i];
        av_occ = 0.;
        max_occ = -1;

        for (j = 1; j <= num_grid_rows; j++) {
            max_occ = max(chany_occ[i][j], max_occ);
            av_occ += chany_occ[i][j];
        }

        av_occ /= num_grid_rows;
        printf("%d\t%d\t%-#9g\t%d\n", i, max_occ, av_occ, chan_width_y[i]);
    }

    printf("\nTotal Tracks in X-direction: %d  in Y-direction: %d\n\n",
           total_x, total_y);
    free_matrix(chanx_occ, 1, num_grid_columns, 0, sizeof(int));
    free_matrix(chany_occ, 0, num_grid_columns, 1, sizeof(int));
}


static void load_channel_occupancies(int** chanx_occ, int** chany_occ)
{
    /* Loads the two arrays passed in with the total occupancy at each of the  *
     * channel segments in the FPGA.                                           */
    int i, j, ivex, inet;
    struct s_trace* tptr;
    rr_types_t rr_type;

    /* First set the occupancy of everything to zero. */

    for (i = 1; i <= num_grid_columns; i++)
        for (j = 0; j <= num_grid_rows; j++) {
            chanx_occ[i][j] = 0;
        }

    for (i = 0; i <= num_grid_columns; i++)
        for (j = 1; j <= num_grid_rows; j++) {
            chany_occ[i][j] = 0;
        }

    /* Now go through each net and count the tracks and pins used everywhere */

    for (inet = 0; inet < num_nets; inet++) {
        if (is_global[inet]) {          /* Skip global nets. */
            continue;
        }

        tptr = trace_head[inet];

        while (tptr != NULL) {
            ivex = tptr->index;
            rr_type = rr_node[ivex].type;

            if (rr_type == SINK) {
                tptr = tptr->next;                /* Skip next segment. */

                if (tptr == NULL) {
                    break;
                }
            } else if (rr_type == CHANX) {
                j = rr_node[ivex].ylow;

                for (i = rr_node[ivex].xlow; i <= rr_node[ivex].xhigh; i++) {
                    chanx_occ[i][j]++;
                }
            } else if (rr_type == CHANY) {
                i = rr_node[ivex].xlow;

                for (j = rr_node[ivex].ylow; j <= rr_node[ivex].yhigh; j++) {
                    chany_occ[i][j]++;
                }
            }

            tptr = tptr->next;
        }
    }
}


static void get_num_bends_and_length(int inet, int* bends_ptr, int* len_ptr,
                                     int* segments_ptr)
{
    /* Counts and returns the number of bends, wirelength, and number of routing *
     * resource segments in net inet's routing.                                  */
    struct s_trace* tptr, *prevptr;
    int ivex;
    rr_types_t curr_type, prev_type;
    int bends, length, segments;
    bends = 0;
    length = 0;
    segments = 0;
    prevptr = trace_head[inet];   /* Should always be SOURCE. */

    if (prevptr == NULL) {
        printf("Error in get_num_bends_and_length:  net #%d has no traceback.\n",
               inet);
        exit(1);
    }

    ivex = prevptr->index;
    prev_type = rr_node[ivex].type;
    tptr = prevptr->next;

    while (tptr != NULL) {
        ivex = tptr->index;
        curr_type = rr_node[ivex].type;

        if (curr_type == SINK) {  /* Starting a new segment */
            tptr = tptr->next;      /* Link to existing path - don't add to len. */

            if (tptr == NULL) {
                break;
            }

            curr_type = rr_node[tptr->index].type;
        } else if (curr_type == CHANX || curr_type == CHANY) {
            segments++;
            length += 1 + rr_node[ivex].xhigh - rr_node[ivex].xlow +
                      rr_node[ivex].yhigh - rr_node[ivex].ylow;

            if (curr_type != prev_type && (prev_type == CHANX || prev_type ==
                                           CHANY)) {
                bends++;
            }
        }

        prev_type = curr_type;
        tptr = tptr->next;
    }

    *bends_ptr = bends;
    *len_ptr = length;
    *segments_ptr = segments;
}


void print_wirelen_prob_dist(void)
{
    /* Prints out the probability distribution of the wirelength / number   *
     * input pins on a net -- i.e. simulates 2-point net length probability *
     * distribution.                                                        */
    double* prob_dist;
    double norm_fac, two_point_length;
    int inet, bends, length, segments, index;
    double av_length;
    prob_dist = (double*) my_calloc(num_grid_columns + num_grid_rows + 3, sizeof(double));
    norm_fac = 0.;

    for (inet = 0; inet < num_nets; ++inet) {
        if (is_global[inet] == FALSE) {
            get_num_bends_and_length(inet, &bends, &length, &segments);
            /*  Assign probability to two integer lengths proportionately -- i.e.  *
             *  if two_point_length = 1.9, add 0.9 of the pins to prob_dist[2] and *
             *  only 0.1 to prob_dist[1].                                          */
            const int knum_net_pins = net[inet].num_net_pins;
            two_point_length = (double) length / (double)(knum_net_pins - 1);
            index = (int) two_point_length;
            prob_dist[index] += (knum_net_pins - 1.0) * (1 - two_point_length
                                                             + index);
            index++;
            prob_dist[index] += (knum_net_pins - 1.0) * (1 - index +
                                                             two_point_length);
            norm_fac += knum_net_pins - 1.0;
        }
    }

    /* Normalize so total probability is 1 and print out. */
    printf("\nProbability distribution of 2-pin net lengths:\n\n");
    printf("Length    p(Lenth)\n");
    av_length = 0;

    for (index = 0; index < num_grid_columns + num_grid_rows + 3; index++) {
        prob_dist[index] /= norm_fac;
        printf("%6d  %10.6f\n", index, prob_dist[index]);
        av_length += prob_dist[index] * index;
    }

    printf("\nExpected value of 2-pin net length (R) is: %g\n",
           av_length);
    free((void*) prob_dist);
}


void print_lambda(void)
{
    /* Finds the average number of input pins used per clb.  Does not   *
     * count inputs which are hooked to global nets (i.e. the clock     *
     * when it is marked global).                                       */
    int block_num, ipin;
    int num_inputs_used = 0;
    int iclass, inet;
    double lambda;

    for (block_num = 0; block_num < num_blocks; block_num++) {
        if (blocks[block_num].type == CLB_TYPE) {
            for (ipin = 0; ipin < pins_per_clb; ipin++) {
                iclass = clb_pin_class[ipin];

                if (class_inf[iclass].type == RECEIVER) {
                    inet = blocks[block_num].nets[ipin];

                    if (inet != OPEN)               /* Pin is connected? */
                        if (is_global[inet] == FALSE) {  /* Not a global clock */
                            num_inputs_used++;
                        }
                }
            }
        }
    }

    lambda = (double) num_inputs_used / (double) num_clbs;
    printf("Average lambda (input pins used per clb) is: %g\n", lambda);
}
