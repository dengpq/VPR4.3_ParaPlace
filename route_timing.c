#include <stdio.h>
#include <math.h>
#include "util.h"
#include "vpr_types.h"
#include "globals.h"
#include "route_export.h"
#include "route_common.h"
#include "route_tree_timing.h"
#include "route_timing.h"
#include "heapsort.h"
#include "path_delay.h"
#include "net_delay.h"


/******************** Subroutines local to route_timing.c ********************/

static int get_max_pins_per_net(void);

static void add_route_tree_to_heap(t_rt_node* rt_node, int target_node,
                                   double target_criticality, double astar_fac);

static void timing_driven_expand_neighbours(struct s_heap* current, int inet,
                                            double bend_cost, double criticality_fac, int target_node, double
                                            astar_fac);

static double get_timing_driven_expected_cost(int ivex, int target_node,
                                             double criticality_fac, double R_upstream);

static int get_expected_segs_to_target(int ivex, int target_node, int *
                                       num_segs_ortho_dir_ptr);

static void update_rr_base_costs(int inet, double largest_criticality);

static void timing_driven_check_net_delays(double** net_delay);


/************************ Subroutine definitions *****************************/
/* FIXME: Timing-Driven Router */
boolean try_timing_driven_route(router_opts_t router_opts,
                                double** net_slack, double** net_delay,
                                vector_t** clb_opins_used_locally)
{
    /* Timing-driven routing algorithm.  The timing graph (includes net_slack)   *
     * must have already been allocated, and net_delay must have been allocated. *
     * Returns TRUE if the routing succeeds, FALSE otherwise.                    */

    double* pin_criticality;        /* [1..max_pins_per_net-1]. */
    int*   sink_order;             /* [1..max_pins_per_net-1]. */
    t_rt_node** rt_node_of_sink;   /* [1..max_pins_per_net-1]. */
    alloc_timing_driven_route_structs(&pin_criticality, &sink_order,
                                      &rt_node_of_sink);

    /* First do one routing iteration ignoring congestion and marking all sinks *
     * on each net as critical to get reasonable net Tdel estimates.           */
    int inet, ipin;
    for (inet = 0; inet < num_nets; ++inet) {
        if (is_global[inet] == FALSE) {
            for (ipin = 1; ipin < net[inet].num_pins; ++ipin) {
                net_slack[inet][ipin] = 0.0;
            }
        } else {  /* Set Tdel of global signals to zero. */
            for (ipin = 1; ipin < net[inet].num_pins; ++ipin) {
                net_delay[inet][ipin] = 0.0;
            }
        }
    }

    double T_crit = 1.0; /* timing_critical */
    double pres_fac = router_opts.first_iter_pres_fac; /* Typically 0 -> ignore cong. */
    int itry = 0;
    for (itry = 1; itry <= router_opts.max_router_iterations; ++itry) {
        for (inet = 0; inet < num_nets; ++inet) {
            if (is_global[inet] == FALSE) {    /* Skip global nets. */
                boolean is_routable = timing_driven_route_net(inet, pres_fac,
                                                      router_opts.max_criticality,
                                                      router_opts.criticality_exp,
                                                      router_opts.astar_fac,
                                                      router_opts.bend_cost,
                                                      net_slack[inet],
                                                      pin_criticality,
                                                      sink_order,
                                                      rt_node_of_sink,
                                                      T_crit, net_delay[inet]);

                /* Impossible to route? (disconnected rr_graph) */
                if (!is_routable) {
                    printf("Routing failed.\n");
                    free_timing_driven_route_structs(pin_criticality, sink_order,
                                                     rt_node_of_sink);
                    return (FALSE);
                }
            }
        }

        /* Make sure any CLB OPINs used up by subblocks being hooked directly     *
         * to them are reserved for that purpose.                                 */
        boolean rip_up_local_opins;
        if (itry == 1) {
            rip_up_local_opins = FALSE;
        } else {
            rip_up_local_opins = TRUE;
        }

        reserve_locally_used_opins(pres_fac, rip_up_local_opins,
                                   clb_opins_used_locally);
        /* Pathfinder guys quit after finding a feasible route. I may want to keep *
         * going longer, trying to improve timing.  Think about this some.         */
        boolean success = feasible_routing();
        if (success) {
            printf("Successfully routed after %d routing iterations.\n", itry);
            free_timing_driven_route_structs(pin_criticality, sink_order,
                                             rt_node_of_sink);
#ifdef DEBUG
            timing_driven_check_net_delays(net_delay);
#endif
            return (TRUE);
        }

        if (itry == 1) {
            pres_fac = router_opts.initial_pres_fac;
            pathfinder_update_cost(pres_fac, 0.);   /* Acc_fac=0 for first iter. */
        } else {
            pres_fac *= router_opts.pres_fac_mult;
            pathfinder_update_cost(pres_fac, router_opts.acc_fac);
        }

        /* Update slack values by doing another timing analysis.                 *
         * Timing_driven_route_net updated the net Tdel values.                 */
        load_timing_graph_net_delays(net_delay);
        T_crit = calc_all_vertexs_arr_req_time(0);
        compute_net_slacks(net_slack);

        printf("T_crit: %g.\n", T_crit);
    }  /* end of for () */

    printf("Routing failed.\n");
    free_timing_driven_route_structs(pin_criticality, sink_order,
                                     rt_node_of_sink);
    return (FALSE);
}


/* Allocates all the structures needed only by the timing-driven router. */
void alloc_timing_driven_route_structs(double** pin_criticality_ptr,
                                       int**   sink_order_ptr,
                                       t_rt_node***  rt_node_of_sink_ptr)
{
    int max_pins_per_net = get_max_pins_per_net();

    /* the following memeory was dynamic memory, it couldn't free as exit this function */
    double* pin_criticality = (double*)my_malloc((max_pins_per_net - 1) * sizeof(double));
    /* I see, the pin #0 was the SOURCE pin */
    *pin_criticality_ptr = pin_criticality - 1;  /* First sink is pin #1. */

    int* sink_order = (int*)my_malloc((max_pins_per_net - 1) * sizeof(int));
    *sink_order_ptr = sink_order - 1;

    t_rt_node** rt_node_of_sink = (t_rt_node**)my_malloc((max_pins_per_net - 1) *
                                              sizeof(t_rt_node*));
    *rt_node_of_sink_ptr = rt_node_of_sink - 1;

    alloc_route_tree_timing_structs();
}


void free_timing_driven_route_structs(double* pin_criticality, int
                                      *sink_order, t_rt_node** rt_node_of_sink)
{
    /* Frees all the stuctures needed only by the timing-driven router.        */
    free(pin_criticality + 1);    /* Starts at index 1. */
    free(sink_order + 1);
    free(rt_node_of_sink + 1);
    free_route_tree_timing_structs();
}


static int get_max_pins_per_net(void)
{
    /* Returns the largest number of pins on any non-global net.    */
    int max_pins_per_net = 0;
    int inet = 0;
    for (inet = 0; inet < num_nets; ++inet) {
        if (is_global[inet] == FALSE) {
            max_pins_per_net = max(max_pins_per_net, net[inet].num_pins);
        }
    }

    return (max_pins_per_net);
}

/* FIXME: timing-driven_routing_one_net */
boolean timing_driven_route_net(int inet, double pres_fac,
                                double max_criticality, double criticality_exp,
                                double astar_fac, double bend_cost,
                                double* net_slack, double* pin_criticality,
                                int* sink_order, t_rt_node** rt_node_of_sink,
                                double T_crit, double* net_delay)
{
    /* Returns TRUE as long is found some way to hook up this net, even if that *
     * way resulted in overuse of resources(congestion). If there is no way to  *
     * route this net, even ignoring congestion, it returns FALSE.  In this case *
     * the rr_graph is disconnected and you can give up.                        */

    /* Rip-up any old routing. */
    /* printf ("\nRouting inet: %d\n", inet);  */
    pathfinder_update_one_cost(trace_head[inet], -1, pres_fac);
    free_traceback(inet);

    /* TODO: What did the pin_criticality used for? */
    int ipin = 0;
    double pin_crit = 0.0;
    for (ipin = 1; ipin < net[inet].num_pins; ++ipin) { /* For all sinks */
        /* was this should be max((max_criticality - net_slack[ipin])/T_crit, 0.0) */
        pin_crit = max(max_criticality - net_slack[ipin] / T_crit, 0.0);
        pin_crit = pow(pin_crit, criticality_exp);
        pin_crit = min(pin_crit, max_criticality);
        pin_criticality[ipin] = pin_crit;
    }

    int num_sinks = net[inet].num_pins - 1; /* only one source, other was sinks */
    /* Sorting int sink_order[num_sinks], according to pin_criticality */
    heapsort(sink_order, pin_criticality, num_sinks);

    /* Update base costs according to fanout and criticality rules */
    double largest_criticality = pin_criticality[sink_order[1]];
    update_rr_base_costs(inet, largest_criticality);
    mark_ends(inet); /* Only needed to check for multiply-connected SINKs */
    t_rt_node* rt_root = init_route_tree_to_source(inet);

    int itarget = 0;
    for (itarget = 1; itarget <= num_sinks; ++itarget) {
        int target_pin = sink_order[itarget];
        int target_node = net_rr_terminals[inet][target_pin];
        /*    printf ("Target #%d, pin number %d, target_node: %d.\n",
                   itarget, target_pin, target_node);  */
        double target_criticality = pin_criticality[target_pin];
        add_route_tree_to_heap(rt_root, target_node, target_criticality,
                               astar_fac);

        struct s_heap* current = get_heap_head();
        if (current == NULL) { /* Infeasible routing.  No possible path for net. */
            reset_path_costs();
            free_route_tree(rt_root);
            return (FALSE);
        }

        int ivex = current->index;
        while (ivex != target_node) {
            double old_tcost = rr_node_route_inf[ivex].path_cost;
            double new_tcost = current->cost;

            double old_back_cost = 0.0;
            if (old_tcost > 0.99 * HUGE_FLOAT) { /* First time touched. */
                old_back_cost = HUGE_FLOAT;
            } else {
                old_back_cost = rr_node_route_inf[ivex].backward_path_cost;
            }

            double new_back_cost = current->backward_path_cost;

            /* I only re-expand a node if both the "known" backward cost is lower  *
             * in the new expansion (this is necessary to prevent loops from       *
             * forming in the routing and causing havoc) *and* the expected total  *
             * cost to the sink is lower than the old value.  Different R_upstream *
             * values could make a path with lower back_path_cost less desirable   *
             * than one with higher cost.  Test whether or not I should disallow   *
             * re-expansion based on a higher total cost.                          */

            if (old_tcost > new_tcost && old_back_cost > new_back_cost) {
                /*       if (old_tcost > new_tcost)   */
                rr_node_route_inf[ivex].prev_node = current->u.prev_node;
                rr_node_route_inf[ivex].prev_edge = current->prev_edge;
                rr_node_route_inf[ivex].path_cost = new_tcost;
                rr_node_route_inf[ivex].backward_path_cost = new_back_cost;

                if (old_tcost > 0.99 * HUGE_FLOAT) { /* First time touched. */
                    add_to_mod_list(&rr_node_route_inf[ivex].path_cost);
                }

                timing_driven_expand_neighbours(current, inet, bend_cost,
                                                target_criticality, target_node, astar_fac);
            }

            free_heap_data(current);
            current = get_heap_head();

            if (current == NULL) {  /* Impossible routing.  No path for net. */
                reset_path_costs();
                free_route_tree(rt_root);
                return (FALSE);
            }

            ivex = current->index;
        } /* end of while(ivex != target_node) */

        /* NB:  In the code below I keep two records of the partial routing:  the   *
         * traceback and the route_tree.  The route_tree enables fast recomputation *
         * of the Elmore Tdel to each node in the partial routing.  The traceback  *
         * lets me reuse all the routines written for breadth-first routing, which  *
         * all take a traceback structure as input.  Before this routine exits the  *
         * route_tree structure is destroyed; only the traceback is needed at that  *
         * point.                                                                   */
        rr_node_route_inf[ivex].target_flag--;    /* Connected to this SINK. */
        struct s_trace* new_route_start_tptr = update_traceback(current, inet);
        rt_node_of_sink[target_pin] = update_route_tree(current);
        free_heap_data(current);
        pathfinder_update_one_cost(new_route_start_tptr, 1, pres_fac);
        empty_heap();
        reset_path_costs();
    } /* end of for (itarget = 1; itarget <= num_sinks; ++itarget) */

    /* For later timing analysis. */
    update_net_delays_from_route_tree(net_delay, rt_node_of_sink, inet);
    free_route_tree(rt_root);
    return (TRUE);
} /* end of boolean timing_driven_route_net() */


static void add_route_tree_to_heap(t_rt_node* rt_node, int target_node,
                                   double target_criticality, double astar_fac)
{
    /* Puts the entire partial routing below and including rt_node onto the heap *
     * (except for those parts marked as not to be expanded) by calling itself   *
     * recursively.                                                              */
    int ivex;
    t_rt_node* child_node;
    t_linked_rt_edge* linked_rt_edge;
    double tot_cost, backward_path_cost, R_upstream;

    /* Pre-order depth-first traversal */

    if (rt_node->re_expand) {
        ivex = rt_node->ivex;
        backward_path_cost = target_criticality * rt_node->Tdel;
        R_upstream = rt_node->R_upstream;
        tot_cost = backward_path_cost + astar_fac * get_timing_driven_expected_cost
                   (ivex, target_node, target_criticality, R_upstream);
        node_to_heap(ivex, tot_cost, NO_PREVIOUS, NO_PREVIOUS,
                     backward_path_cost, R_upstream);
    }

    linked_rt_edge = rt_node->u.child_list;

    while (linked_rt_edge != NULL) {
        child_node = linked_rt_edge->child;
        add_route_tree_to_heap(child_node, target_node, target_criticality,
                               astar_fac);
        linked_rt_edge = linked_rt_edge->next;
    }
}


static void timing_driven_expand_neighbours(struct s_heap* current, int inet,
                                            double bend_cost, double criticality_fac, int target_node, double
                                            astar_fac)
{
    /* Puts all the rr_nodes adjacent to current on the heap.  rr_nodes outside *
     * the expanded bounding box specified in route_bb are not added to the     *
     * heap.                                                                    */
    int iconn, to_node, num_edges, ivex, iswitch, target_x, target_y;
    rr_types_t from_type, to_type;
    double new_tot_cost, old_back_pcost, new_back_pcost, R_upstream;
    double new_R_upstream, Tdel;
    ivex = current->index;
    old_back_pcost = current->backward_path_cost;
    R_upstream = current->R_upstream;
    num_edges = rr_node[ivex].num_edges;
    target_x = rr_node[target_node].xhigh;
    target_y = rr_node[target_node].yhigh;

    for (iconn = 0; iconn < num_edges; iconn++) {
        to_node = rr_node[ivex].edges[iconn];

        if (rr_node[to_node].xhigh < route_bb[inet].xmin ||
                rr_node[to_node].xlow > route_bb[inet].xmax  ||
                rr_node[to_node].yhigh < route_bb[inet].ymin ||
                rr_node[to_node].ylow > route_bb[inet].ymax) {
            continue;    /* Node is outside (expanded) bounding box. */
        }

        /* Prune away IPINs that lead to blocks other than the target one.  Avoids  *
         * the issue of how to cost them properly so they don't get expanded before *
         * more promising routes, but makes route-throughs (via CLBs) impossible.   *
         * Change this if you want to investigate route-throughs.                   */
        to_type = rr_node[to_node].type;

        if (to_type == IPIN && (rr_node[to_node].xhigh != target_x ||
                                rr_node[to_node].yhigh != target_y)) {
            continue;
        }

        /* new_back_pcost stores the "known" part of the cost to this node -- the   *
         * congestion cost of all the routing resources back to the existing route  *
         * plus the known Tdel of the total path back to the source.  new_tot_cost *
         * is this "known" backward cost + an expected cost to get to the target.   */
        new_back_pcost = old_back_pcost + (1. - criticality_fac) *
                         get_rr_cong_cost(to_node);
        iswitch = rr_node[ivex].switches[iconn];

        if (switch_inf[iswitch].buffered) {
            new_R_upstream = switch_inf[iswitch].R;
        } else {
            new_R_upstream = R_upstream + switch_inf[iswitch].R;
        }

        Tdel = rr_node[to_node].C * (new_R_upstream + 0.5 * rr_node[to_node].R);
        Tdel += switch_inf[iswitch].Tdel;
        new_R_upstream += rr_node[to_node].R;
        new_back_pcost += criticality_fac * Tdel;

        if (bend_cost != 0.) {
            from_type = rr_node[ivex].type;
            to_type = rr_node[to_node].type;

            if ((from_type == CHANX && to_type == CHANY) ||
                    (from_type == CHANY && to_type == CHANX)) {
                new_back_pcost += bend_cost;
            }
        }

        new_tot_cost = new_back_pcost + astar_fac *
                       get_timing_driven_expected_cost(to_node, target_node,
                                                       criticality_fac, new_R_upstream);
        node_to_heap(to_node, new_tot_cost, ivex, iconn, new_back_pcost,
                     new_R_upstream);
    }  /* End for all neighbours */
}


static double get_timing_driven_expected_cost(int ivex, int target_node,
                                             double criticality_fac, double R_upstream)
{
    /* Determines the expected cost (due to both Tdel and resouce cost) to reach *
     * the target node from ivex.  It doesn't include the cost of ivex --       *
     * that's already in the "known" path_cost.                                   */
    rr_types_t rr_type;
    int cost_index, ortho_cost_index, num_segs_same_dir, num_segs_ortho_dir;
    double expected_cost, cong_cost, Tdel;
    rr_type = rr_node[ivex].type;

    if (rr_type == CHANX || rr_type == CHANY) {
        num_segs_same_dir = get_expected_segs_to_target(ivex, target_node,
                                                        &num_segs_ortho_dir);
        cost_index = rr_node[ivex].cost_index;
        ortho_cost_index = rr_indexed_data[cost_index].ortho_cost_index;
        cong_cost = num_segs_same_dir * rr_indexed_data[cost_index].base_cost +
                    num_segs_ortho_dir * rr_indexed_data[ortho_cost_index].base_cost;
        cong_cost += rr_indexed_data[IPIN_COST_INDEX].base_cost +
                     rr_indexed_data[SINK_COST_INDEX].base_cost;
        Tdel = num_segs_same_dir * rr_indexed_data[cost_index].T_linear +
               num_segs_ortho_dir * rr_indexed_data[ortho_cost_index].T_linear +
               num_segs_same_dir * num_segs_same_dir *
               rr_indexed_data[cost_index].T_quadratic +
               num_segs_ortho_dir * num_segs_ortho_dir *
               rr_indexed_data[ortho_cost_index].T_quadratic +
               R_upstream * (num_segs_same_dir * rr_indexed_data[cost_index].C_load
                             + num_segs_ortho_dir * rr_indexed_data[ortho_cost_index].C_load);
        Tdel += rr_indexed_data[IPIN_COST_INDEX].T_linear;
        expected_cost = criticality_fac * Tdel + (1. - criticality_fac) * cong_cost;
        return (expected_cost);
    } else if (rr_type == IPIN) { /* Change if you're allowing route-throughs */
        return (rr_indexed_data[SINK_COST_INDEX].base_cost);
    } else {     /* Change this if you want to investigate route-throughs */
        return (0.);
    }
}


/* Macro used below to ensure that fractions are rounded up, but doubleing   *
 * point values very close to an integer are rounded to that integer.       */

#define ROUND_UP(x) (ceil (x - 0.001))


static int get_expected_segs_to_target(int ivex, int target_node, int *
                                       num_segs_ortho_dir_ptr)
{
    /* Returns the number of segments the same type as ivex that will be needed *
     * to reach target_node (not including ivex) in each direction (the same    *
     * direction (horizontal or vertical) as ivex and the orthogonal direction).*/
    rr_types_t rr_type;
    int target_x, target_y, num_segs_same_dir, cost_index, ortho_cost_index;
    int no_need_to_pass_by_clb;
    double inv_length, ortho_inv_length, ylow, yhigh, xlow, xhigh;
    target_x = rr_node[target_node].xlow;
    target_y = rr_node[target_node].ylow;
    cost_index = rr_node[ivex].cost_index;
    inv_length = rr_indexed_data[cost_index].inv_length;
    ortho_cost_index = rr_indexed_data[cost_index].ortho_cost_index;
    ortho_inv_length = rr_indexed_data[ortho_cost_index].inv_length;
    rr_type = rr_node[ivex].type;

    if (rr_type == CHANX) {
        ylow = rr_node[ivex].ylow;
        xhigh = rr_node[ivex].xhigh;
        xlow = rr_node[ivex].xlow;

        /* Count vertical (orthogonal to ivex) segs first. */

        if (ylow > target_y) {         /* Coming from a row above target? */
            *num_segs_ortho_dir_ptr = ROUND_UP((ylow - target_y + 1.) *
                                               ortho_inv_length);
            no_need_to_pass_by_clb = 1;
        } else if (ylow < target_y - 1) { /* Below the CLB bottom? */
            *num_segs_ortho_dir_ptr = ROUND_UP((target_y - ylow) *
                                               ortho_inv_length);
            no_need_to_pass_by_clb = 1;
        } else {            /* In a row that passes by target CLB */
            *num_segs_ortho_dir_ptr = 0;
            no_need_to_pass_by_clb = 0;
        }

        /* Now count horizontal (same dir. as ivex) segs. */

        if (xlow > target_x + no_need_to_pass_by_clb) {
            num_segs_same_dir = ROUND_UP((xlow - no_need_to_pass_by_clb -
                                          target_x) * inv_length);
        } else if (xhigh < target_x - no_need_to_pass_by_clb) {
            num_segs_same_dir = ROUND_UP((target_x - no_need_to_pass_by_clb -
                                          xhigh) * inv_length);
        } else {
            num_segs_same_dir = 0;
        }
    } else { /* ivex is a CHANY */
        ylow = rr_node[ivex].ylow;
        yhigh = rr_node[ivex].yhigh;
        xlow = rr_node[ivex].xlow;

        /* Count horizontal (orthogonal to ivex) segs first. */

        if (xlow > target_x) {         /* Coming from a column right of target? */
            *num_segs_ortho_dir_ptr = ROUND_UP((xlow - target_x + 1.) *
                                               ortho_inv_length);
            no_need_to_pass_by_clb = 1;
        } else if (xlow < target_x - 1) { /* Left of and not adjacent to the CLB? */
            *num_segs_ortho_dir_ptr = ROUND_UP((target_x - xlow) *
                                               ortho_inv_length);
            no_need_to_pass_by_clb = 1;
        } else {            /* In a column that passes by target CLB */
            *num_segs_ortho_dir_ptr = 0;
            no_need_to_pass_by_clb = 0;
        }

        /* Now count vertical (same dir. as ivex) segs. */

        if (ylow > target_y + no_need_to_pass_by_clb) {
            num_segs_same_dir = ROUND_UP((ylow - no_need_to_pass_by_clb -
                                          target_y) * inv_length);
        } else if (yhigh < target_y - no_need_to_pass_by_clb) {
            num_segs_same_dir = ROUND_UP((target_y - no_need_to_pass_by_clb -
                                          yhigh) * inv_length);
        } else {
            num_segs_same_dir = 0;
        }
    }

    return (num_segs_same_dir);
}


static void update_rr_base_costs(int inet, double largest_criticality)
{
    /* Changes the base costs of different types of rr_nodes according to the *
     * criticality, fanout, etc. of the current net being routed (inet).      */
    double fanout = net[inet].num_pins - 1.0;
    double factor = sqrt(fanout);

    int index = 0;
    for (index = CHANX_COST_INDEX_START; index < num_rr_indexed_data; ++index) {
        if (rr_indexed_data[index].T_quadratic > 0.0) { /* pass transistor */
            rr_indexed_data[index].base_cost =
                rr_indexed_data[index].saved_base_cost * factor;
        } else {
            rr_indexed_data[index].base_cost =
                rr_indexed_data[index].saved_base_cost;
        }
    }
}


#define ERROR_TOL 0.0001

static void timing_driven_check_net_delays(double** net_delay)
{
    /* Checks that the net Tdels computed incrementally during timing driven    *
     * routing match those computed from scratch by the net_delay.c module.      */
    int inet, ipin;
    double** net_delay_check;
    linked_vptr_t* ch_list_head_net_delay_check;
    net_delay_check = alloc_net_delay(&ch_list_head_net_delay_check);
    load_net_delay_from_routing(net_delay_check);

    for (inet = 0; inet < num_nets; inet++) {
        for (ipin = 1; ipin < net[inet].num_pins; ipin++) {
            if (net_delay_check[inet][ipin] == 0.) { /* Should be only GLOBAL nets */
                if (net_delay_check[inet][ipin] != 0.) {
                    printf("Error in timing_driven_check_net_delays: net %d pin %d."
                           "\tIncremental calc. net_delay is %g, but from scratch "
                           "net Tdel is %g.\n", inet, ipin, net_delay[inet][ipin],
                           net_delay_check[inet][ipin]);
                    exit(1);
                }
            } else {
                if (fabs(1. - net_delay[inet][ipin] / net_delay_check[inet][ipin])
                        > ERROR_TOL) {
                    printf("Error in timing_driven_check_net_delays: net %d pin %d."
                           "\tIncremental calc. net_delay is %g, but from scratch "
                           "net Tdel is %g.\n", inet, ipin, net_delay[inet][ipin],
                           net_delay_check[inet][ipin]);
                    exit(1);
                }
            }
        }
    }

    free_net_delay(net_delay_check, &ch_list_head_net_delay_check);
    printf("Completed net Tdel value cross check successfully.\n");
}
