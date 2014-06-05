#include <stdio.h>
#include "util.h"
#include "vpr_types.h"
#include "globals.h"
#include "route_export.h"
#include "check_route.h"
#include "check_rr_graph.h"


/******************** Subroutines local to this module **********************/

static void check_node_and_range(int ivex, router_types_t route_type);
static void check_source(int ivex, int inet);
static void check_sink(int ivex, int inet, boolean* pin_done);
static void check_switch(struct s_trace* tptr, int num_switch);
static boolean check_adjacent(int from_node, int to_node);
static int pin_and_chan_adjacent(int pin_node, int chan_node);
static int chanx_chany_adjacent(int chanx_node, int chany_node);
static void reset_flags(int inet, boolean* connected_to_route);
static void recompute_occupancy_from_scratch(vector_t** clb_opins_used_locally);
static void check_locally_used_clb_opins(vector_t** clb_opins_used_locally,
                                         router_types_t route_type);


/************************ Subroutine definitions ****************************/


void check_route(router_types_t route_type, int num_switch,
                 vector_t** clb_opins_used_locally)
{
    /* This routine checks that a routing:  (1) Describes a properly         *
     * connected path for each net, (2) this path connects all the           *
     * pins spanned by that net, and (3) that no routing resources are       *
     * oversubscribed (the occupancy of everything is recomputed from        *
     * scratch).                                                             */
    int inet, ipin, max_pins, ivex, prev_node;
    boolean valid, connects;
    boolean* connected_to_route;    /* [0 .. num_rr_nodes-1] */
    struct s_trace* tptr;
    boolean* pin_done;
    printf("\nChecking to ensure routing is legal ...\n");
    /* Recompute the occupancy from scratch and check for overuse of routing *
     * resources.  This was already checked in order to determine that this  *
     * is a successful routing, but I want to double check it here.          */
    recompute_occupancy_from_scratch(clb_opins_used_locally);
    valid = feasible_routing();

    if (valid == FALSE) {
        printf("Error in check_route -- routing resources are overused.\n");
        exit(1);
    }

    check_locally_used_clb_opins(clb_opins_used_locally, route_type);
    connected_to_route = (boolean*) my_calloc(num_rr_nodes, sizeof(boolean));
    max_pins = 0;

    for (inet = 0; inet < num_nets; ++inet) {
        max_pins = max(max_pins, net[inet].num_net_pins);
    }

    pin_done = (boolean*)my_malloc(max_pins * sizeof(boolean));

    /* Now check that all nets are indeed connected. */

    for (inet = 0; inet < num_nets; ++inet) {
        if (is_global[inet]) {             /* Skip global nets. */
            continue;
        }

        const int knum_net_pins = net[inet].num_net_pins;
        for (ipin = 0; ipin < knum_net_pins; ++ipin) {
            pin_done[ipin] = FALSE;
        }

        /* Check the SOURCE of the net. */
        tptr = trace_head[inet];

        if (tptr == NULL) {
            printf("Error in check_route:  net %d has no routing.\n", inet);
            exit(1);
        }

        ivex = tptr->index;
        check_node_and_range(ivex, route_type);
        check_switch(tptr, num_switch);
        connected_to_route[ivex] = TRUE;   /* Mark as in path. */
        check_source(ivex, inet);
        pin_done[0] = TRUE;
        prev_node = ivex;
        tptr = tptr->next;

        /* Check the rest of the net */
        while (tptr != NULL) {
            ivex = tptr->index;
            check_node_and_range(ivex, route_type);
            check_switch(tptr, num_switch);

            if (rr_node[prev_node].type == SINK) {
                if (connected_to_route[ivex] == FALSE) {
                    printf("Error in check_route.  Node %d does not link "
                           "into the existing routing for net %d.\n", ivex, inet);
                    exit(1);
                }
            } else {
                connects = check_adjacent(prev_node, ivex);
                if (!connects) {
                    printf("Error in check_route while checking net %d.\n",
                           inet);
                    printf("Non-adjacent segments in traceback.\n");
                    exit(1);
                }

                if (connected_to_route[ivex] && rr_node[ivex].type != SINK) {
                    /* Note:  Can get multiple connections to the same logically-equivalent     *
                     * SINK in some logic blocks.                                               */
                    printf("Error in check_route:  net %d routing is not a tree.\n",
                           inet);
                    exit(1);
                }

                connected_to_route[ivex] = TRUE;  /* Mark as in path. */

                if (rr_node[ivex].type == SINK) {
                    check_sink(ivex, inet, pin_done);
                }
            }    /* End of prev_node type != SINK */

            prev_node = ivex;
            tptr = tptr->next;
        }  /* End while */

        if (rr_node[prev_node].type != SINK) {
            printf("Error in check_route.  Net %d does not end\n", inet);
            printf("with a SINK.\n");
            exit(1);
        }

        for (ipin = 0; ipin < knum_net_pins; ++ipin) {
            if (pin_done[ipin] == FALSE) {
                printf("Error in check_route.  Net %d does not \n", inet);
                printf("connect to pin %d.\n", ipin);
                exit(1);
            }
        }

        reset_flags(inet, connected_to_route);
    }  /* End for each net */

    free(pin_done);
    free(connected_to_route);
    printf("Completed routing consistency check successfully.\n\n");
}


static void check_sink(int ivex, int inet, boolean* pin_done)
{
    /* Checks that this SINK node is one of the terminals of inet, and marks   *
     * the appropriate pin as being reached.                                   */
    int i = rr_node[ivex].xlow;
    int j = rr_node[ivex].ylow;
    int ptc_num = rr_node[ivex].ptc_num;
    int ifound = 0;

    int ipin, block_num, iclass, blk_pin;
    if (clb_grids[i][j].block_type == CLB_TYPE) {
        block_num = clb_grids[i][j].u.blocks;

        const int knum_net_pins = net[inet].num_net_pins;
        for (ipin = 1; ipin < knum_net_pins; ++ipin) { /* All net SINKs */
            if (net[inet].node_blocks[ipin] == block_num) {
                blk_pin = net[inet].node_block_pins[ipin];
                iclass = clb_pin_class[blk_pin];

                if (iclass == ptc_num) {
                    /* Could connect to same pin class on the same clb more than once.  Only   *
                     * update pin_done for a pin that hasn't been reached yet.                 */
                    if (pin_done[ipin] == FALSE) {
                        ifound++;
                        pin_done[ipin] = TRUE;
                        break;
                    }
                }
            }
        }
    } else { /* IO_TYPE pad */
        block_num = clb_grids[i][j].u.io_blocks[ptc_num];

        const int knum_net_pins = net[inet].num_net_pins;
        for (ipin = 0; ipin < knum_net_pins; ++ipin) {
            if (net[inet].node_blocks[ipin] == block_num) {   /* Pad:  no pin class */
                ifound++;
                pin_done[ipin] = TRUE;
            }
        }
    }

    if (ifound > 1) {
        printf("Error in check_sink:  found %d terminals of net %d of class/pad"
               "\n %d at location (%d, %d).\n", ifound, inet, ptc_num, i, j);
        exit(1);
    }

    if (ifound < 1) {
        printf("Error in check_sink:  node %d does not connect to any terminal "
               "\n of net %d.\n", ivex, inet);
        exit(1);
    }
}


static void check_source(int ivex, int inet)
{
    /* Checks that the node passed in is a valid source for this net.        */
    rr_types_t rr_type;
    int i, j, ptc_num, block_num, blk_pin, iclass;
    rr_type = rr_node[ivex].type;

    if (rr_type != SOURCE) {
        printf("Error in check_source:  net %d begins with a node of type %d.\n",
               inet, rr_type);
        exit(1);
    }

    i = rr_node[ivex].xlow;
    j = rr_node[ivex].ylow;
    ptc_num = rr_node[ivex].ptc_num;
    block_num = net[inet].node_blocks[0];

    if (blocks[block_num].x != i || blocks[block_num].y != j) {
        printf("Error in check_source:  net SOURCE is in wrong location (%d,%d)."
               "\n", i, j);
        exit(1);
    }

    if (blocks[block_num].block_type == CLB_TYPE) {
        blk_pin = net[inet].node_block_pins[0];
        iclass = clb_pin_class[blk_pin];

        if (ptc_num != iclass) {
            printf("Error in check_source:  net SOURCE is of wrong class (%d).\n",
                   ptc_num);
            exit(1);
        }
    } else {  /* IO_TYPE Pad.  NB:  check_node ensured ptc_num < m_usage of this pad.  */
        if (clb_grids[i][j].u.io_blocks[ptc_num] != block_num) {
            printf("Error in check_source:  net SOURCE is at wrong pad (pad #%d)."
                   "\n", ptc_num);
            exit(1);
        }
    }
}


static void check_switch(struct s_trace* tptr, int num_switch)
{
    /* Checks that the switch leading from this traceback element to the next *
     * one is a legal switch type.                                            */
    int ivex;
    short switch_type;
    ivex = tptr->index;
    switch_type = tptr->iswitch;

    if (rr_node[ivex].type != SINK) {
        if (switch_type < 0 || switch_type >= num_switch) {
            printf("Error in check_switch: rr_node %d left via switch type %d.\n",
                   ivex, switch_type);
            printf("Switch type is out of range.\n");
            exit(1);
        }
    } else { /* Is a SINK */

        /* Without feedthroughs, there should be no switch.  If feedthroughs are    *
         * allowed, change to treat a SINK like any other node (as above).          */
        if (switch_type != OPEN) {
            printf("Error in check_switch:  rr_node %d is a SINK, but attempts \n"
                   "to use a switch of type %d.\n", ivex, switch_type);
            exit(1);
        }
    }
}


static void reset_flags(int inet, boolean* connected_to_route)
{
    /* This routine resets the flags of all the channel segments contained *
     * in the traceback of net inet to 0.  This allows us to check the     *
     * next net for connectivity (and the default state of the flags       *
     * should always be zero after they have been used).                   */
    struct s_trace* tptr;
    int ivex;
    tptr = trace_head[inet];

    while (tptr != NULL) {
        ivex = tptr->index;
        connected_to_route[ivex] = FALSE;   /* Not in routed path now. */
        tptr = tptr->next;
    }
}


static boolean check_adjacent(int from_node, int to_node)
{
    /* This routine checks if the rr_node to_node is reachable from from_node.   *
     * It returns TRUE if is reachable and FALSE if it is not.  Check_node has   *
     * already been used to verify that both nodes are valid rr_nodes, so only   *
     * adjacency is checked here.                                                */
    boolean reached = FALSE;
    int iconn = -1;
    for (iconn = 0; iconn < rr_node[from_node].num_edges; ++iconn) {
        if (rr_node[from_node].edges[iconn] == to_node) {
            reached = TRUE;
            break;
        }
    }

    if (!reached) {
        return (FALSE);
    }

    /* Now we know the rr graph says these two nodes are adjacent.  Double  *
     * check that this makes sense, to verify the rr graph.                 */
    int num_adj = 0;
    rr_types_t from_type = rr_node[from_node].type;
    int from_xlow = rr_node[from_node].xlow;
    int from_ylow = rr_node[from_node].ylow;
    int from_ptc = rr_node[from_node].ptc_num;

    rr_types_t to_type = rr_node[to_node].type;
    int to_xlow = rr_node[to_node].xlow;
    int to_ylow = rr_node[to_node].ylow;
    int to_ptc = rr_node[to_node].ptc_num;

    int to_xhigh, to_yhigh, from_xhigh, from_yhigh;
    switch (from_type) {
        case SOURCE:
            if (to_type == OPIN) {
                if (from_xlow == to_xlow && from_ylow == to_ylow) {
                    if (clb_grids[to_xlow][to_ylow].block_type == CLB_TYPE) {
                        int iclass = clb_pin_class[to_ptc];
                        if (iclass == from_ptc) {
                            ++num_adj;
                        }
                    } else { /* IO_TYPE blocks */
                        if (to_ptc == from_ptc) {
                            ++num_adj;
                        }
                    }
                }
            }
            break;

        case SINK:
            if (to_type == SOURCE) {   /* Feedthrough.  Not in code as yet. */
                if (from_xlow == to_xlow && from_ylow == to_ylow &&
                        clb_grids[to_xlow][to_ylow].block_type == CLB_TYPE) {
                    ++num_adj;
                }
            }
            break;

        case OPIN:
            if (to_type == CHANX || to_type == CHANY) {
                num_adj += pin_and_chan_adjacent(from_node, to_node);
            }

            break;

        case IPIN:
            if (to_type == SINK && from_xlow == to_xlow && from_ylow == to_ylow) {
                if (clb_grids[from_xlow][from_ylow].block_type == CLB_TYPE) {

                    int iclass = clb_pin_class[from_ptc];
                    if (iclass == to_ptc) {
                        ++num_adj;
                    }
                } else {  /* OUTPAD_TYPE */
                    if (from_ptc == to_ptc) {
                        ++num_adj;
                    }
                }
            }
            break;

        case CHANX:
            if (to_type == IPIN) {
                num_adj += pin_and_chan_adjacent(to_node, from_node);
            } else if (to_type == CHANX) {
                from_xhigh = rr_node[from_node].xhigh;
                to_xhigh = rr_node[to_node].xhigh;

                if (from_ylow == to_ylow) {
                    if (to_xhigh == from_xlow - 1 || from_xhigh == to_xlow - 1) {
                        ++num_adj;
                    }
                }
            } else if (to_type == CHANY) {
                num_adj += chanx_chany_adjacent(from_node, to_node);
            }

            break;

        case CHANY:
            if (to_type == IPIN) {
                num_adj += pin_and_chan_adjacent(to_node, from_node);
            } else if (to_type == CHANY) {
                from_yhigh = rr_node[from_node].yhigh;
                to_yhigh = rr_node[to_node].yhigh;

                if (from_xlow == to_xlow) {
                    if (to_yhigh == from_ylow - 1 || from_yhigh == to_ylow - 1) {
                        ++num_adj;
                    }
                }
            } else if (to_type == CHANX) {
                num_adj += chanx_chany_adjacent(to_node, from_node);
            }

            break;

        default:
            break;
    }

    if (num_adj == 1) {
        return (TRUE);
    } else if (num_adj == 0) {
        return (FALSE);
    }

    printf("Error in check_adjacent: num_adj = %d. Expected 0 or 1.\n", num_adj);
    exit(1);
}  /* end of static boolean check_adjacent(int from_node, int to_node) */


static int chanx_chany_adjacent(int chanx_node, int chany_node)
{
    /* Returns 1 if the specified CHANX and CHANY nodes are adjacent, 0         *
     * otherwise.                                                               */
    int chanx_y = rr_node[chanx_node].ylow;
    int chanx_xlow = rr_node[chanx_node].xlow;
    int chanx_xhigh = rr_node[chanx_node].xhigh;

    int chany_x = rr_node[chany_node].xlow;
    int chany_ylow = rr_node[chany_node].ylow;
    int chany_yhigh = rr_node[chany_node].yhigh;

    if (chany_ylow > chanx_y + 1 || chany_yhigh < chanx_y
          || chanx_xlow > chany_x + 1 || chanx_xhigh < chany_x) {
        return 0;
    }

    return 1;
}


static int pin_and_chan_adjacent(int pin_node, int chan_node)
{
    /* Checks if pin_node is adjacent to chan_node.  It returns 1 if the two   *
     * nodes are adjacent and 0 if they are not (any other value means there's *
     * a bug in this routine).                                                 */
    int num_adj = 0;
    int pin_x = rr_node[pin_node].xlow;
    int pin_y = rr_node[pin_node].ylow;
    int pin_ptc = rr_node[pin_node].ptc_num;
    rr_types_t chan_type = rr_node[chan_node].type;
    int chan_xlow = rr_node[chan_node].xlow;
    int chan_ylow = rr_node[chan_node].ylow;
    int chan_xhigh = rr_node[chan_node].xhigh;
    int chan_yhigh = rr_node[chan_node].yhigh;

    if (clb_grids[pin_x][pin_y].block_type == CLB_TYPE) {
        if (chan_type == CHANX) {
            if (chan_ylow == pin_y) {   /* CHANX above CLB_TYPE */
                if (pinloc[TOP][pin_ptc] == 1 && pin_x <= chan_xhigh &&
                        pin_x >= chan_xlow) {
                    num_adj++;
                }
            } else if (chan_ylow == pin_y - 1) { /* CHANX below CLB_TYPE */
                if (pinloc[BOTTOM][pin_ptc] == 1 && pin_x <= chan_xhigh &&
                        pin_x >= chan_xlow) {
                    num_adj++;
                }
            }
        } else if (chan_type == CHANY) {
            if (chan_xlow == pin_x) {   /* CHANY to right of CLB_TYPE */
                if (pinloc[RIGHT][pin_ptc] == 1 && pin_y <= chan_yhigh &&
                        pin_y >= chan_ylow) {
                    num_adj++;
                }
            } else if (chan_xlow == pin_x - 1) { /* CHANY to left of CLB_TYPE */
                if (pinloc[LEFT][pin_ptc] == 1 && pin_y <= chan_yhigh &&
                        pin_y >= chan_ylow) {
                    num_adj++;
                }
            }
        }
    } else {          /* IO_TYPE pad */
        if (pin_y == 0) {    /* Bottom row of pads. */
            if (chan_type == CHANX && chan_ylow == 0 && pin_x <= chan_xhigh &&
                    pin_x >= chan_xlow) {
                num_adj++;
            }
        } else if (pin_y == num_grid_rows + 1) { /* Top row of pads. */
            if (chan_type == CHANX && chan_ylow == num_grid_rows && pin_x <= chan_xhigh &&
                    pin_x >= chan_xlow) {
                num_adj++;
            }
        } else if (pin_x == 0) {   /* Left column of pads */
            if (chan_type == CHANY && chan_xlow == 0 && pin_y <= chan_yhigh &&
                    pin_y >= chan_ylow) {
                num_adj++;
            }
        } else if (pin_x == num_grid_columns + 1) { /* Right row of pads */
            if (chan_type == CHANY && chan_xlow == num_grid_columns && pin_y <= chan_yhigh &&
                    pin_y >= chan_ylow) {
                num_adj++;
            }
        }
    }

    return (num_adj);
}


static void recompute_occupancy_from_scratch(vector_t** clb_opins_used_locally)
{
    /* This routine updates the m_usage field in the rr_node structure according to *
     * the resource usage of the current routing.  It does a brute force        *
     * recompute from scratch that is useful for sanity checking.               */
    int ivex, inet, iblk, iclass, ipin, num_local_opins;
    struct s_trace* tptr;

    /* First set the occupancy of everything to zero. */

    for (ivex = 0; ivex < num_rr_nodes; ivex++) {
        rr_node[ivex].m_usage = 0;
    }

    /* Now go through each net and count the tracks and pins used everywhere */

    for (inet = 0; inet < num_nets; inet++) {
        if (is_global[inet]) {          /* Skip global nets. */
            continue;
        }

        tptr = trace_head[inet];

        if (tptr == NULL) {
            continue;
        }

        while (1) {
            ivex = tptr->index;
            rr_node[ivex].m_usage++;

            if (rr_node[ivex].type == SINK) {
                tptr = tptr->next;                /* Skip next segment. */

                if (tptr == NULL) {
                    break;
                }
            }

            tptr = tptr->next;
        }
    }

    /* Now update the occupancy of each of the "locally used" OPINs on each CLB_TYPE *
     * (CLB_TYPE outputs used up by being directly wired to subblocks used only      *
     * locally).                                                                */

    for (iblk = 0; iblk < num_blocks; iblk++) {
        for (iclass = 0; iclass < num_pin_class; iclass++) {
            num_local_opins = clb_opins_used_locally[iblk][iclass].nelem;

            /* Will always be 0 for pads or SINK classes. */
            for (ipin = 0; ipin < num_local_opins; ipin++) {
                ivex = clb_opins_used_locally[iblk][iclass].list[ipin];
                rr_node[ivex].m_usage++;
            }
        }
    }
}


static void check_locally_used_clb_opins(vector_t** clb_opins_used_locally,
                                         router_types_t route_type)
{
    /* Checks that enough OPINs on CLBs have been set aside (used up) to make a *
     * legal routing if subblocks connect to OPINs directly.                    */
    int iclass, iblk, num_local_opins, ivex, ipin;
    rr_types_t rr_type;

    for (iblk = 0; iblk < num_blocks; iblk++) {
        for (iclass = 0; iclass < num_pin_class; iclass++) {
            num_local_opins = clb_opins_used_locally[iblk][iclass].nelem;
            /* Always 0 for pads and for SINK classes */

            for (ipin = 0; ipin < num_local_opins; ipin++) {
                ivex = clb_opins_used_locally[iblk][iclass].list[ipin];
                check_node_and_range(ivex, route_type);   /* Node makes sense? */
                /* Now check that node is an OPIN of the right type. */
                rr_type = rr_node[ivex].type;

                if (rr_type != OPIN) {
                    printf("Error in check_locally_used_opins:  Block #%d (%s)\n"
                           "\tclass %d locally used OPIN is of the wrong rr_type --\n"
                           "\tit is rr_node #%d of type %d.\n", iblk,
                           blocks[iblk].name, iclass, ivex, rr_type);
                    exit(1);
                }

                ipin = rr_node[ivex].ptc_num;

                if (clb_pin_class[ipin] != iclass) {
                    printf("Error in check_locally_used_opins:  Block #%d (%s):\n"
                           "\tExpected class %d locally used OPIN, got class %d."
                           "\trr_node #: %d.\n", iblk, blocks[iblk].name, iclass,
                           clb_pin_class[ipin], ivex);
                    exit(1);
                }
            }
        }
    }
}


static void check_node_and_range(int ivex, router_types_t route_type)
{
    /* Checks that ivex is within the legal range, then calls check_node to    *
     * check that everything else about the node is OK.                         */
    if (ivex < 0 || ivex >= num_rr_nodes) {
        printf("Error in check_node_and_range:  rr_node #%d is out of legal "
               "\trange (0 to %d).\n", ivex, num_rr_nodes - 1);
        exit(1);
    }

    check_node(ivex, route_type);
}
