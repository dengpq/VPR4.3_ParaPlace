#include <stdio.h>
#include <assert.h>

#include "util.h"
#include "globals.h"
#include "path_delay_parallel.h"
#include "path_delay2.h"
#include "net_delay.h"
#include "vpr_utils.h"

/* TODO: Add option for registered inputs and outputs once works, currently, outputs only */

/****************** Timing graph Structure ************************************
 *                                                                            *
 * In the timing graph I create, input pads and constant generators have no   *
 * inputs; everything else has inputs.  Every input pad and output pad is     *
 * represented by two tnodes -- an input pin and an output pin.  For an input *
 * pad the input pin comes from off chip and has no fanin, while the output   *
 * pin drives outpads and/or CLBs.  For output pads, the input node is driven *
 * by a FB or input pad, and the output node goes off chip and has no        *
 * fanout (out-edges).  I need two nodes to respresent things like pads       *
 * because I mark all Tdel on tedges, not on tnodes.                         *
 *                                                                            *
 * Every used (not OPEN) FB pin becomes a timing node.  As well, every used  *
 * subblock pin within a FB also becomes a timing node.  Unused (OPEN) pins  *
 * don't create any timing nodes. If a subblock is used in combinational mode *
 * (i.e. its clock pin is open), I just hook the subblock input tnodes to the *
 * subblock output vertexes.  If the subblock is used in sequential mode, I      *
 * create two extra tnodes.  One is just the subblock clock pin, which is     *
 * connected to the subblock output.  This means that FFs don't generate      *
 * their output until their clock arrives.  For global clocks coming from an  *
 * input pad, the Tdel of the clock is 0, so the FFs generate their outputs  *
 * at T = 0, as usual.  For locally-generated or gated clocks, however, the   *
 * clock will arrive later, and the FF output will be generated later.  This  *
 * lets me properly model things like ripple counters and gated clocks.  The  *
 * other extra node is the FF storage node (i.e. a sink), which connects to   *
 * the subblock inputs and has no fanout.                                     *
 *                                                                            *
 * One other subblock that needs special attention is a constant generator.   *
 * This has no used inputs, but its output is used.  I create an extra vertexes, *
 * a dummy input, in addition to the output pin vertexes.  The dummy vertexes has   *
 * no fanin.  Since constant generators really generate their outputs at T =  *
 * -infinity, I set the Tdel from the input vertexes to the output to a large-  *
 * magnitude negative number.  This guarantees every block that needs the     *
 * output of a constant generator sees it available very early.               *
 *                                                                            *
 * For this routine to work properly, subblocks whose outputs are unused must *
 * be completely empty -- all their input pins and their clock pin must be    *
 * OPEN.  Check_netlist checks the input netlist to guarantee this -- don't   *
 * disable that check.                                                        *
 *                                                                            *
 * NB:  The discussion below is only relevant for circuits with multiple      *
 * clocks.  For circuits with a single clock, everything I do is exactly      *
 * correct.                                                                   *
 *                                                                            *
 * A note about how I handle FFs:  By hooking the clock pin up to the FF      *
 * output, I properly model the time at which the FF generates its output.    *
 * I don't do a completely rigorous job of modelling required arrival time at *
 * the FF input, however.  I assume every FF and outpad needs its input at    *
 * T = 0, which is when the earliest clock arrives.  This can be conservative *
 * -- a fuller analysis would be to do a fast path analysis of the clock      *
 * feeding each FF and subtract its earliest arrival time from the Tdel of   *
 * the D signal to the FF input.  This is too much work, so I'm not doing it. *
 * Alternatively, when one has N clocks, it might be better to just do N      *
 * separate timing analyses, with only signals from FFs clocked on clock i    *
 * being propagated forward on analysis i, and only FFs clocked on i being    *
 * considered as sinks.  This gives all the critical paths within clock       *
 * domains, but ignores interactions.  Instead, I assume all the clocks are   *
 * more-or-less synchronized (they might be gated or locally-generated, but   *
 * they all have the same frequency) and explore all interactions.  Tough to  *
 * say what's the better way.  Since multiple clocks aren't important for my  *
 * work, it's not worth bothering about much.                                 *
 *                                                                            *
 ******************************************************************************/

#define T_CONSTANT_GENERATOR -1000  /* Essentially -ve infinity */

/***************** Types local to this module ***************************/
enum e_subblock_pin_type {
    SUB_INPUT = 0,
    SUB_OUTPUT,
    SUB_CLOCK,
    NUM_SUB_PIN_TYPES
};


/********************* Subroutine definitions ****************************/
/* Sets the delays of the inter-FB nets to the values specified by          *
* net_delay[0..num_nets-1][1..num_pins-1].  These net delays should have    *
* been allocated and loaded with the net_delay routines.  This routine      *
* marks the corresponding edges in the timing graph with the proper Tdel.  */
unsigned long load_timing_graph_net_delays_parallel(double** net_delay,
                                                    int start_net,
                                                    int finish_net)
{
    unsigned long sinks = 0;
    int inet = 0;
    for (inet = start_net; inet < finish_net; ++inet) {
        int inode = driver_node_index_of_net[inet];
        edge_t* tedge = vertexes[inode].out_edges;
        sinks += (long)net[inet].num_net_pins;

        /* Note that the edges of a vertexes corresponding to a FB or INPAD opin must*
         * be in the same order as the pins of the net driven by the vertexes.       */
        int ipin = 0;
        const int knum_net_pins = net[inet].num_net_pins;
        /* for (ipin = 1; ipin < knum_net_pins + 1; ++ipin) */
        for (ipin = 1; ipin < knum_net_pins; ++ipin) {
            tedge[ipin - 1].Tdel = net_delay[inet][ipin];
            int to_node = tedge[ipin - 1].to_node;

            int counter = -1;
            for (counter = 0; counter < vertexes[to_node].num_parents; ++counter) {
                if (vertexes[to_node].in_edges[counter].to_node != inode) {
                    continue;
                }

                vertexes[to_node].in_edges[counter].Tdel = net_delay[inet][ipin];
                break;
            }

            if (counter == vertexes[to_node].num_parents) {
                printf("disrepency between fanout and fanin nodes\n");
                exit(-1);
            }
        }
    }

    return sinks;
} /* end of unsigned long load_timing_graph_net_delays_parallel() */

/* Sets the delays of the inter-FB nets to the values specified by          *
 * net_delay[0..num_net  s-1][1..num_pins-1].  These net delays should have    *
 * been allocated and loaded with the net_delay routines.  This routine      *
 * marks the corresponding edges in the timing graph with the proper Tdel.  */
/* void load_timing_graph_net_delays(double** net_delay)
{
    int inet, ipin, inode;
    edge_t* tedge;

    for (inet = 0; inet < num_nets; ++inet) {
        inode = driver_node_index_of_net[inet];
        tedge = vertexes[inode].out_edges;

        const int knum_net_pins = net[inet].num_net_pins;
        for (ipin = 1; ipin < knum_net_pins + 1; ++ipin) {
            tedge[ipin - 1].Tdel = net_delay[inet][ipin];
        }
    }
} */


/* Determines the slack of every source-sink pair of block pins in the      *
 * circuit.  The timing graph must have already been built.  target_cycle_  *
 * time is the target Tdel for the circuit -- if 0, the target_cycle_time  *
 * is set to the critical path found in the timing graph.  This routine     *
 * loads net_slack, and returns the current critical path Tdel.            */
double calc_tnodes_arr_time_parallel(int start_node,
                                    int finish_node,
                                    int ilevel)
{
    /* Reset all arrival times to -ve infinity. Can't just set to zero or the *
     * constant propagation (constant generators work at -ve infinity) won't  *
     * work.                                                                  */
    double T_crit = 0.0;

    int i = 0;
    for (i = start_node; i < finish_node; ++i) {
        int inode = tnodes_at_level[ilevel].list[i];
        int num_parent_edges = vertexes[inode].num_parents;
        edge_t* tedge = vertexes[inode].in_edges;

        int iedge = -1;
        for (iedge = 0; iedge < num_parent_edges; ++iedge) {
            int   from_node = tedge[iedge].to_node;
            double arr_time = vertexes[from_node].arr_time;
            double Tdel = tedge[iedge].Tdel;
            vertexes[inode].arr_time = max(vertexes[inode].arr_time,
                                           arr_time + Tdel);
        }

        T_crit = max(T_crit, vertexes[inode].arr_time);
    }

    return T_crit;
}  /* end of double calc_tnodes_arr_time_parallel(int start_node,...) */

/* Determines the slack of every source-sink pair of block pins in the circuit.   *
 * The Timing_Graph must have already been built. target_cycle_time is the target *
 * Tdel for the circuit -- if 0, the target_cycle_time is set to the critical path*
 * found in the Timing-Graph. This routine loads net_slack, and returns the current*
 * critical path Tdel.            */
void calc_tnodes_req_time_parallel(double T_cycle,
                                   int start_node,
                                   int finish_node,
                                   int ilevel)
{
    /* Compute the required arrival times with a backward breadth-first analysis *
     * from sinks (output pads, etc.) to primary inputs.                         */
    int i = 0;
    for (i = start_node; i < finish_node; ++i) {
        int inode = tnodes_at_level[ilevel].list[i];

        int num_out_edges = vertexes[inode].num_out_edges;
        if (num_out_edges == 0) {
            /* this node was in last level. It's a sink node. */
            vertexes[inode].req_time = T_cycle;
        } else {
            edge_t* tedge = vertexes[inode].out_edges;
            int    to_node = tedge[0].to_node;
            double  Tdel = tedge[0].Tdel;
            double  req_time = vertexes[to_node].req_time - Tdel;

            int iedge = 0;
            for (iedge = 1; iedge < num_out_edges; ++iedge) {
                to_node = tedge[iedge].to_node;
                Tdel = tedge[iedge].Tdel;
                req_time = min(req_time, vertexes[to_node].req_time - Tdel);
            }
            vertexes[inode].req_time = req_time;
        }
    }  /* end of for (i = start_node; i < finish_node; ++i) */
}  /* end of void calc_tnodes_req_time_parallel(double** net_slack,..) */


/* Puts the slack of each source-sink pair of block pins in net_slack.      *
 * local storage may not be more efficient due to the malloc and copy back  *
 * overhead. False sharing in the original implementation since consecutive *
 * net_slack data are being modified.                                       */
unsigned long compute_net_slacks_parallel(double** net_slack,
                                          int start_net,
                                          int finish_net)
{
    unsigned long count_work = 0;
    int inet = 0;
    for (inet = start_net; inet < finish_net; ++inet) {
        const int driver_node = driver_node_index_of_net[inet];
        const double arr_time = vertexes[driver_node].arr_time;
        edge_t* tedge = vertexes[driver_node].out_edges;

        const int num_out_edges = vertexes[driver_node].num_out_edges;
        count_work += (unsigned long int)num_out_edges;

        int iedge = -1;
        for (iedge = 0; iedge < num_out_edges; ++iedge) {
            const int to_node = tedge[iedge].to_node;
            const double Tdel = tedge[iedge].Tdel;
            const double req_time = vertexes[to_node].req_time;
            net_slack[inet][iedge + 1] = req_time - arr_time - Tdel;
        }  /* compute a few nets' slack OK! */
    }

    return count_work;
} /* end of unsigned long compute_net_slacks_parallel(double** net_slack, ) */

