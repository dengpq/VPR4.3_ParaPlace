#include <math.h>  /* Needed only for sqrt call (remove if sqrt removed) */
#include "util.h"
#include "vpr_types.h"
#include "globals.h"
#include "rr_graph2.h"
#include "rr_graph_indexed_data.h"


/******************* Subroutines local to this module ************************/

static void load_rr_indexed_data_base_costs(int nodes_per_chan, int**
                                            rr_node_indices, router_base_cost_t router_base_cost_type, int
                                            wire_to_ipin_switch);

static double get_delay_normalization_fac(int nodes_per_chan, int**
                                         rr_node_indices);

static double get_average_opin_delay(int** rr_node_indices, int
                                    nodes_per_chan);

static void load_rr_indexed_data_T_values(int index_start, int
                                          num_indices_to_load, rr_types_t rr_type, int nodes_per_chan,
                                          int** rr_node_indices, segment_info_t* segment_inf);



/******************** Subroutine definitions *********************************/


void alloc_and_load_rr_indexed_data(segment_info_t* segment_inf,
                                    int num_segment, int** rr_node_indices, int nodes_per_chan,
                                    int wire_to_ipin_switch, router_base_cost_t router_base_cost_type)
{
    /* Allocates the rr_indexed_data array and loads it with appropriate values. *
     * It currently stores the segment type (or OPEN if the index doesn't        *
     * correspond to an CHANX or CHANY type), the base cost of nodes of that     *
     * type, and some info to allow rapid estimates of time to get to a target   *
     * to be computed by the router.                                             */
    /* Right now all SOURCES have the same base cost; and similarly there's only *
     * one base cost for each of SINKs, OPINs, and IPINs (four total).  This can *
     * be changed just by allocating more space in the array below and changing  *
     * the cost_index values for these rr_nodes, if you want to make some pins   *
     * etc. more expensive than others.  I give each segment type in an          *
     * x-channel its own cost_index, and each segment type in a y-channel its    *
     * own cost_index.                                                           */
    int iseg, length, i, index;
    num_rr_indexed_data = CHANX_COST_INDEX_START + 2 * num_segment;
    rr_indexed_data = (rr_indexed_data_t*) my_malloc(num_rr_indexed_data *
                                                     sizeof(rr_indexed_data_t));

    /* For rr_types that aren't CHANX or CHANY, base_cost is valid, but most     *
     * other fields are invalid.  For IPINs, the T_linear field is also valid;   *
     * all other fields are invalid.  For SOURCES, SINKs and OPINs, all fields   *
     * other than base_cost are invalid. Mark invalid fields as OPEN for safety. */

    for (i = SOURCE_COST_INDEX; i <= IPIN_COST_INDEX; i++) {
        rr_indexed_data[i].ortho_cost_index = OPEN;
        rr_indexed_data[i].seg_index = OPEN;
        rr_indexed_data[i].inv_length = OPEN;
        rr_indexed_data[i].T_linear = OPEN;
        rr_indexed_data[i].T_quadratic = OPEN;
        rr_indexed_data[i].C_load = OPEN;
    }

    rr_indexed_data[IPIN_COST_INDEX].T_linear =
        switch_inf[wire_to_ipin_switch].Tdel;

    /* X-directed segments. */

    for (iseg = 0; iseg < num_segment; iseg++) {
        index = CHANX_COST_INDEX_START + iseg;
        rr_indexed_data[index].ortho_cost_index = index + num_segment;

        if (segment_inf[iseg].longline) {
            length = num_grid_columns;
        } else {
            length = min(segment_inf[iseg].length, num_grid_columns);
        }

        rr_indexed_data[index].inv_length = 1. / length;
        rr_indexed_data[index].seg_index = iseg;
    }

    load_rr_indexed_data_T_values(CHANX_COST_INDEX_START, num_segment, CHANX,
                                  nodes_per_chan, rr_node_indices, segment_inf);

    /* Y-directed segments. */

    for (iseg = 0; iseg < num_segment; iseg++) {
        index = CHANX_COST_INDEX_START + num_segment + iseg;
        rr_indexed_data[index].ortho_cost_index = index - num_segment;

        if (segment_inf[iseg].longline) {
            length = num_grid_rows;
        } else {
            length = min(segment_inf[iseg].length, num_grid_rows);
        }

        rr_indexed_data[index].inv_length = 1. / length;
        rr_indexed_data[index].seg_index = iseg;
    }

    load_rr_indexed_data_T_values(CHANX_COST_INDEX_START + num_segment,
                                  num_segment, CHANY, nodes_per_chan, rr_node_indices, segment_inf);
    load_rr_indexed_data_base_costs(nodes_per_chan, rr_node_indices,
                                    router_base_cost_type, wire_to_ipin_switch);
}


static void load_rr_indexed_data_base_costs(int nodes_per_chan, int**
                                            rr_node_indices, router_base_cost_t router_base_cost_type,
                                            int wire_to_ipin_switch)
{
    /* Loads the base_cost member of rr_indexed_data according to the specified *
     * router_base_cost_type.                                                          */
    double delay_normalization_fac;
    int index;

    if (router_base_cost_type == DELAY_NORMALIZED) {
        delay_normalization_fac = get_delay_normalization_fac(nodes_per_chan,
                                                              rr_node_indices);
    } else {
        delay_normalization_fac = 1.;
    }

    if (router_base_cost_type == DEMAND_ONLY || router_base_cost_type == DELAY_NORMALIZED) {
        rr_indexed_data[SOURCE_COST_INDEX].base_cost = delay_normalization_fac;
        rr_indexed_data[SINK_COST_INDEX].base_cost = 0.;
        rr_indexed_data[OPIN_COST_INDEX].base_cost = delay_normalization_fac;
#ifndef SPEC
        rr_indexed_data[IPIN_COST_INDEX].base_cost = 0.95 * delay_normalization_fac;
#else     /* Avoid roundoff for SPEC */
        rr_indexed_data[IPIN_COST_INDEX].base_cost = delay_normalization_fac;
#endif
    } else if (router_base_cost_type == INTRINSIC_DELAY) {
        rr_indexed_data[SOURCE_COST_INDEX].base_cost = 0.;
        rr_indexed_data[SINK_COST_INDEX].base_cost = 0.;
        rr_indexed_data[OPIN_COST_INDEX].base_cost = get_average_opin_delay(
                                                         rr_node_indices, nodes_per_chan);
        rr_indexed_data[IPIN_COST_INDEX].base_cost =
            switch_inf[wire_to_ipin_switch].Tdel;
    }

    /* Load base costs for CHANX and CHANY segments */

    for (index = CHANX_COST_INDEX_START; index < num_rr_indexed_data; index++) {
        if (router_base_cost_type == INTRINSIC_DELAY)
            rr_indexed_data[index].base_cost = rr_indexed_data[index].T_linear +
                                               rr_indexed_data[index].T_quadratic;
        else
            /*       rr_indexed_data[index].base_cost = delay_normalization_fac /
                                                rr_indexed_data[index].inv_length;  */
        {
            rr_indexed_data[index].base_cost = delay_normalization_fac;
        }

        /*       rr_indexed_data[index].base_cost = delay_normalization_fac *
                          sqrt (1. / rr_indexed_data[index].inv_length);  */
        /*       rr_indexed_data[index].base_cost = delay_normalization_fac *
                          (1. + 1. / rr_indexed_data[index].inv_length);  */
    }

    /* Save a copy of the base costs -- if dynamic costing is used by the     *
     * router, the base_cost values will get changed all the time and being   *
     * able to restore them from a saved version is useful.                   */

    for (index = 0; index < num_rr_indexed_data; index++) {
        rr_indexed_data[index].saved_base_cost = rr_indexed_data[index].base_cost;
    }
}


static double get_delay_normalization_fac(int nodes_per_chan, int**
                                         rr_node_indices)
{
    /* Returns the average Tdel to go 1 CLB_TYPE distance along a wire.  */
    const int clb_dist = 3;  /* Number of CLBs I think the average conn. goes. */
    int ivex, itrack, cost_index;
    double Tdel, delay_sum, frac_num_seg;
    delay_sum = 0.;

    for (itrack = 0; itrack < nodes_per_chan; itrack++) {
        ivex = gerr_node_t_index((num_grid_columns + 1) / 2, (num_grid_rows + 1) / 2, CHANX, itrack,
                                  nodes_per_chan, rr_node_indices);
        cost_index = rr_node[ivex].cost_index;
        frac_num_seg = clb_dist * rr_indexed_data[cost_index].inv_length;
        Tdel = frac_num_seg * rr_indexed_data[cost_index].T_linear +
               frac_num_seg * frac_num_seg * rr_indexed_data[cost_index].T_quadratic;
        delay_sum += Tdel / (double) clb_dist;
    }

    for (itrack = 0; itrack < nodes_per_chan; itrack++) {
        ivex = gerr_node_t_index((num_grid_columns + 1) / 2, (num_grid_rows + 1) / 2, CHANY, itrack,
                                  nodes_per_chan, rr_node_indices);
        cost_index = rr_node[ivex].cost_index;
        frac_num_seg = clb_dist * rr_indexed_data[cost_index].inv_length;
        Tdel = frac_num_seg * rr_indexed_data[cost_index].T_linear +
               frac_num_seg * frac_num_seg * rr_indexed_data[cost_index].T_quadratic;
        delay_sum += Tdel / (double) clb_dist;
    }

    return (delay_sum / (2. * nodes_per_chan));
}


static double get_average_opin_delay(int** rr_node_indices, int
                                    nodes_per_chan)
{
    /* Returns the average Tdel from an OPIN to a wire in an adjacent channel. */
    int ivex, ipin, iclass, iedge, num_edges, to_switch, to_node, num_conn;
    double Cload, Tdel;
    Tdel = 0.;
    num_conn = 0;

    for (ipin = 0; ipin < pins_per_clb; ipin++) {
        iclass = clb_pin_class[ipin];

        if (class_inf[iclass].type == DRIVER) {   /* OPIN */
            ivex = gerr_node_t_index((num_grid_columns + 1) / 2, (num_grid_rows + 1) / 2, OPIN, ipin,
                                      nodes_per_chan, rr_node_indices);
            num_edges = rr_node[ivex].num_edges;

            for (iedge = 0; iedge < num_edges; iedge++) {
                to_node = rr_node[ivex].edges[iedge];
                to_switch = rr_node[ivex].switches[iedge];
                Cload = rr_node[to_node].C;
                Tdel += Cload * switch_inf[to_switch].R + switch_inf[to_switch].Tdel;
                num_conn++;
            }
        }
    }

    Tdel /= (double) num_conn;
    return (Tdel);
}


static void load_rr_indexed_data_T_values(int index_start, int
                                          num_indices_to_load, rr_types_t rr_type, int nodes_per_chan,
                                          int** rr_node_indices, segment_info_t* segment_inf)
{
    /* Loads the average propagation times through segments of each index type  *
     * for either all CHANX segment types or all CHANY segment types.  It does  *
     * this by looking at all the segments in one channel in the middle of the  *
     * array and averaging the R and C values of all segments of the same type  *
     * and using them to compute average Tdel values for this type of segment. */
    int itrack, iseg, ivex, cost_index, iswitch;
    double* C_total, *R_total;    /* [0..num_rr_indexed_data - 1] */
    int* num_nodes_of_index;     /* [0..num_rr_indexed_data - 1] */
    double Rnode, Cnode, Rsw, Tsw;
    num_nodes_of_index = (int*) my_calloc(num_rr_indexed_data, sizeof(int));
    C_total = (double*) my_calloc(num_rr_indexed_data, sizeof(double));
    R_total = (double*) my_calloc(num_rr_indexed_data, sizeof(double));

    /* Get average C and R values for all the segments of this type in one      *
     * channel segment, near the middle of the array.                           */

    for (itrack = 0; itrack < nodes_per_chan; itrack++) {
        ivex = gerr_node_t_index((num_grid_columns + 1) / 2, (num_grid_rows + 1) / 2, rr_type, itrack,
                                  nodes_per_chan, rr_node_indices);
        cost_index = rr_node[ivex].cost_index;
        num_nodes_of_index[cost_index]++;
        C_total[cost_index] += rr_node[ivex].C;
        R_total[cost_index] += rr_node[ivex].R;
    }

    for (cost_index = index_start; cost_index < index_start + num_indices_to_load;
            cost_index++) {
        if (num_nodes_of_index[cost_index] == 0) {     /* Segments don't exist. */
            rr_indexed_data[cost_index].T_linear = OPEN;
            rr_indexed_data[cost_index].T_quadratic = OPEN;
            rr_indexed_data[cost_index].C_load = OPEN;
        } else {
            Rnode = R_total[cost_index] / num_nodes_of_index[cost_index];
            Cnode = C_total[cost_index] / num_nodes_of_index[cost_index];
            iseg = rr_indexed_data[cost_index].seg_index;
            iswitch = segment_inf[iseg].wire_switch;
            Rsw = switch_inf[iswitch].R;
            Tsw = switch_inf[iswitch].Tdel;

            if (switch_inf[iswitch].buffered) {
                rr_indexed_data[cost_index].T_linear = Tsw + Rsw * Cnode + 0.5 *
                                                       Rnode * Cnode;
                rr_indexed_data[cost_index].T_quadratic = 0.;
                rr_indexed_data[cost_index].C_load = 0.;
            } else {  /* Pass transistor */
                rr_indexed_data[cost_index].C_load = Cnode;
                /* See Dec. 23, 1997 notes for deriviation of formulae. */
                rr_indexed_data[cost_index].T_linear = Tsw + 0.5 * Rsw * Cnode;
                rr_indexed_data[cost_index].T_quadratic = (Rsw + Rnode) * 0.5 * Cnode;
            }
        }
    }

    free(num_nodes_of_index);
    free(C_total);
    free(R_total);
}
