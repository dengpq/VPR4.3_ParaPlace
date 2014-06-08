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
 * subblock output tnode.  If the subblock is used in sequential mode, I      *
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
 * This has no used inputs, but its output is used.  I create an extra tnode, *
 * a dummy input, in addition to the output pin tnode.  The dummy tnode has   *
 * no fanin.  Since constant generators really generate their outputs at T =  *
 * -infinity, I set the Tdel from the input tnode to the output to a large-  *
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

/***************** Variables local to this module ***************************/
/* Variables for "chunking" the tedge memory.  If the head pointer is NULL, *
 * no timing graph exists now.                                              */
static linked_vptr_t* tedge_ch_list_head = NULL;
static int tedge_ch_bytes_avail = 0;
static char* tedge_ch_next_avail = NULL;


/***************** Subroutines local to this module *************************/
static int alloc_and_load_pin_mappings(int** *block_pin_to_tnode_ptr,
                                       int**** *snode_block_pin_to_tnode_ptr,
                                       subblock_data_t subblock_data,
                                       int** *num_uses_of_sblk_opin);

static void free_pin_mappings(int** block_pin_to_tnode,
                              int**** snode_block_pin_to_tnode,
                              int* num_subblocks_per_block);

static void alloc_and_load_fanout_counts(int** *num_uses_of_fb_ipin_ptr,
                                         int**** num_uses_of_sblk_opin_ptr,
                                         subblock_data_t subblock_data);

static void free_fanout_counts(int** num_uses_of_fb_ipin,
                               int** *num_uses_of_sblk_opin);

static double** alloc_net_slack(void);

static void compute_net_slacks(double** net_slack);

static void alloc_and_load_tnodes_and_net_mapping(int** num_uses_of_fb_ipin,
                                                  int
                                                  ** *num_uses_of_sblk_opin,
                                                  int** block_pin_to_tnode,
                                                  int
                                                  **** snode_block_pin_to_tnode,
                                                  subblock_data_t
                                                  subblock_data,
                                                  timing_info_t timing_inf);

static void build_fb_tnodes(int iblk,
                            int* n_uses_of_fb_ipin,
                            int** block_pin_to_tnode,
                            int** *sub_pin_to_tnode,
                            int num_subs,
                            subblock_t* sub_inf,
                            double T_fb_ipin_to_sblk_ipin);

static void build_subblock_tnodes(int** n_uses_of_sblk_opin,
                                  int* node_block_pin_to_tnode,
                                  int** *sub_pin_to_tnode,
                                  int* num_subblocks_per_block,
                                  subblock_t** subblock_inf,
                                  timing_info_t timing_inf,
                                  int iblk);


static boolean is_global_clock(int iblk,
                               int sub,
                               int subpin,
                               int* num_subblocks_per_block,
                               subblock_t** subblock_inf);

static void build_block_output_tnode(int inode,
                                     int iblk,
                                     int ipin,
                                     int** block_pin_to_tnode);


/********************* Subroutine definitions *******************************/
static void free_fanout_counts(int** num_uses_of_fb_ipin,
                               int** *num_uses_of_sblk_opin)
{

    /* Frees the fanout count arrays. */

    int iblk;
    block_types_t type;

    for (iblk = 0; iblk < num_blocks; iblk++) {
        type = blocks[iblk].block_type;
        free(num_uses_of_fb_ipin[iblk]);
        free_matrix(num_uses_of_sblk_opin[iblk], 0,
                    type->max_subblocks - 1, 0, sizeof(int));
    }

    free(num_uses_of_fb_ipin);
    free(num_uses_of_sblk_opin);
}


static void
alloc_and_load_tnodes_and_net_mapping(int** num_uses_of_fb_ipin,
                                      int** *num_uses_of_sblk_opin,
                                      int** block_pin_to_tnode,
                                      int**** snode_block_pin_to_tnode,
                                      subblock_data_t subblock_data,
                                      timing_info_t timing_inf)
{

    int iblk, i;
    int* num_subblocks_per_block;
    subblock_t** subblock_inf;


    tnode = (t_tnode*) my_malloc(num_of_vertexs * sizeof(t_tnode));
    tnode_descript = (t_tnode_descript*) my_malloc(num_of_vertexs *
                                                   sizeof(t_tnode_descript));

    net_to_driver_tnode = (int*)my_malloc(num_nets * sizeof(int));

    subblock_inf = subblock_data.subblock_inf;
    num_subblocks_per_block = subblock_data.num_subblocks_per_block;

    /* allocate from_edges matrix*/
    for (i = 0; i < num_of_vertexs; i++) {
        tnode[i].num_parents = 0;
        tnode[i].in_edges = (edge_t*) my_malloc (6 * sizeof(edge_t));
    }

    for (iblk = 0; iblk < num_blocks; iblk++) {
        build_fb_tnodes(iblk, num_uses_of_fb_ipin[iblk],
                        block_pin_to_tnode,
                        snode_block_pin_to_tnode[iblk],
                        num_subblocks_per_block[iblk], subblock_inf[iblk],
                        blocks[iblk].type->type_timing_inf.
                        T_fb_ipin_to_sblk_ipin);

        build_subblock_tnodes(num_uses_of_sblk_opin[iblk],
                              block_pin_to_tnode[iblk],
                              snode_block_pin_to_tnode[iblk],
                              num_subblocks_per_block, subblock_inf,
                              timing_inf, iblk);
    }
}


static void
build_fb_tnodes(int iblk,
                int* n_uses_of_fb_ipin,
                int** block_pin_to_tnode,
                int** *sub_pin_to_tnode,
                int num_subs,
                subblock_t* sub_inf,
                double T_fb_ipin_to_sblk_ipin)
{

    /* This routine builds the tnodes corresponding to the fb pins of this
     * block, and properly hooks them up to the rest of the graph. Note that
     * only the snode_block_pin_to_tnode, etc. element for this block is passed in.
     * Assumes that pins are ordered as [inputs][outputs][clk]
     */

    int isub, ipin, iedge, from_pin, opin;
    int inode, to_node, num_edges;
    edge_t* tedge;

    block_types_t type = blocks[iblk].type;
    int* next_ipin_edge = (int*)my_malloc(type->num_pins * sizeof(int));
    int clk_pin = 0;

    /* Start by allocating the tedge arrays, and for opins, loading them.    */

    for (ipin = 0; ipin < blocks[iblk].type->num_pins; ipin++) {
        inode = block_pin_to_tnode[iblk][ipin];

        if (inode != OPEN) {
            /* Pin is used->put in graph */
            if (is_opin(ipin, blocks[iblk].type)) {
                build_block_output_tnode(inode, iblk, ipin,
                                         block_pin_to_tnode);
                tnode_descript[inode].type = FB_OPIN;
            } else {
                /* FB ipin */
                next_ipin_edge[ipin] = 0;   /* Reset */
                num_edges = n_uses_of_fb_ipin[ipin];

                /* if clock pin, timing edges go to each subblock output used */
                for (isub = 0; isub < num_subs; isub++) {
                    if (sub_inf[isub].clock == ipin) {
                        for (opin = 0;
                                opin <
                                type->max_subblock_outputs;
                                opin++) {
                            if (sub_inf[isub].
                                    outputs[opin] != OPEN) {
                                num_edges++;
                            }
                        }

                        num_edges--;    /* Remove clock_pin count, replaced by outputs */
                    }
                }

                tnode[inode].num_edges = num_edges;

                tnode[inode].out_edges =
                    (edge_t*) my_chunk_malloc(num_edges *
                                               sizeof(edge_t),
                                               &tedge_ch_list_head,
                                               &tedge_ch_bytes_avail,
                                               &tedge_ch_next_avail);

                tnode_descript[inode].type = FB_IPIN;
            }

            tnode_descript[inode].ipin = ipin;
            tnode_descript[inode].isubblk = OPEN;
            tnode_descript[inode].iblk = iblk;
        }
    }

    /* Now load the tedge arrays for the FB input pins. Do this by looking at   *
     * where the subblock input and clock pins are driven from.                 */

    for (isub = 0; isub < num_subs; isub++) {
        for (ipin = 0; ipin < type->max_subblock_inputs; ipin++) {
            from_pin = sub_inf[isub].inputs[ipin];

            /* Not OPEN and comes from fb ipin? */

            if (from_pin != OPEN
                    && from_pin < blocks[iblk].type->num_pins) {
                inode = block_pin_to_tnode[iblk][from_pin];
                assert(inode != OPEN);
                to_node = sub_pin_to_tnode[isub][SUB_INPUT][ipin];
                tedge = tnode[inode].out_edges;
                iedge = next_ipin_edge[from_pin]++;
                tedge[iedge].to_node = to_node;
                tedge[iedge].Tdel = T_fb_ipin_to_sblk_ipin;
            }
        }

        from_pin = sub_inf[isub].clock;

        if (from_pin != OPEN && from_pin < blocks[iblk].type->num_pins) {
            inode = block_pin_to_tnode[iblk][from_pin];
            to_node = sub_pin_to_tnode[isub][SUB_CLOCK][clk_pin];   /* Feeds seq. output */

            /* connect to each output flip flop */
            for (opin = 0; opin < type->max_subblock_outputs; opin++) {
                if (sub_inf[isub].outputs[opin] != OPEN) {
                    tedge = tnode[inode].out_edges;
                    iedge = next_ipin_edge[from_pin]++;
                    tedge[iedge].to_node = to_node;

                    /* For the earliest possible clock I want this Tdel to be zero, so it       *
                     * * arrives at flip flops at T = 0.  For later clocks or locally generated    *
                     * * clocks that may accumulate Tdel (like the clocks in a ripple counter),   *
                     * * I might want to make this Tdel nonzero.  Not worth bothering about now.  */

                    tedge[iedge].Tdel = 0.;
                    to_node += 2;
                }
            }
        }
    }

    free(next_ipin_edge);
}


static void
build_block_output_tnode(int inode,
                         int iblk,
                         int ipin,
                         int** block_pin_to_tnode)
{

    /* Sets the number of edges and the tedge array for an output pin from a      *
     * block.  This pin must be hooked to something -- i.e. not OPEN.            */

    int iedge, to_blk, to_pin, to_node;

    int inet = blocks[iblk].nets[ipin];  /* Won't be OPEN, as inode exists */
    assert(inet != OPEN);   /* Sanity check. */

    net_to_driver_tnode[inet] = inode;

    int num_edges = (net[inet].num_net_pins + 1) - 1;
    tnode[inode].num_edges = num_edges;

    tnode[inode].out_edges = (edge_t*) my_chunk_malloc(num_edges *
                                                        sizeof(edge_t),
                                                        &tedge_ch_list_head,
                                                        &tedge_ch_bytes_avail,
                                                        &tedge_ch_next_avail);

    edge_t* tedge = tnode[inode].out_edges;
    const int knum_net_pins = net[inet].num_net_pins;
    for (iedge = 0; iedge < knum_net_pins; ++iedge) {
        to_blk = net[inet].node_blocks[iedge + 1];

        to_pin = net[inet].node_block_pins[iedge + 1];

        to_node = block_pin_to_tnode[to_blk][to_pin];
        tedge[iedge].to_node = to_node;
        /* Set Tdel from net delays with a later call */
    }
}


static void build_subblock_tnodes(int** n_uses_of_sblk_opin,
                      int* node_block_pin_to_tnode,
                      int** *sub_pin_to_tnode,
                      int* num_subblocks_per_block,
                      subblock_t** subblock_inf,
                      timing_info_t timing_inf,
                      int iblk)
{
    /* This routine builds the tnodes of the subblock pins within one FB. Note *
     * that only the block_pin_to_tnode, etc. data for *this* block are passed  *
     * in.                                                                      */
    int isub, ipin, inode, to_node, from_pin, to_pin, opin, from_opin,
        clk_pin, used_opin_count;
    int num_subs, from_sub;
    subblock_t* sub_inf;
    int iedge, num_edges;
    double sink_delay;
    edge_t* tedge;
    boolean has_inputs, has_outputs;
    int** next_sblk_opin_edge;  /* [0..max_subblocks-1][0..max_subblock_outputs-1] */
    int* num_opin_used_in_sblk; /* [0..max_subblocks-1] */
    block_types_t type = blocks[iblk].type;

    sub_inf = subblock_inf[iblk];
    num_subs = num_subblocks_per_block[iblk];

    next_sblk_opin_edge =
        (int**)alloc_matrix(0, type->max_subblocks - 1, 0,
                            type->max_subblock_outputs - 1, sizeof(int));
    num_opin_used_in_sblk =
        (int*)my_malloc(type->max_subblocks * sizeof(int));

    clk_pin = 0;

    /* Allocate memory for output pins first. */
    for (isub = 0; isub < num_subs; isub++) {
        num_opin_used_in_sblk[isub] = 0;

        for (opin = 0; opin < type->max_subblock_outputs; opin++) {
            inode = sub_pin_to_tnode[isub][SUB_OUTPUT][opin];

            if (inode != OPEN) {
                /* Output is used->timing node exists. */
                num_opin_used_in_sblk[isub]++;
                next_sblk_opin_edge[isub][opin] = 0;    /* Reset */
                num_edges = n_uses_of_sblk_opin[isub][opin];
                tnode[inode].num_edges = num_edges;

                tnode[inode].out_edges =
                    (edge_t*) my_chunk_malloc(num_edges *
                                               sizeof(edge_t),
                                               &tedge_ch_list_head,
                                               &tedge_ch_bytes_avail,
                                               &tedge_ch_next_avail);

                if (IO_TYPE == type) {
                    tnode_descript[inode].type = INPAD_OPIN;
                    tnode[inode + 1].num_edges = 1;
                    tnode[inode + 1].out_edges = (edge_t*)
                                                 my_chunk_malloc(sizeof(edge_t),
                                                                 &tedge_ch_list_head,
                                                                 &tedge_ch_bytes_avail,
                                                                 &tedge_ch_next_avail);
                    tedge = tnode[inode + 1].out_edges;
                    tedge[0].to_node = inode;

                    /* For input pads, use sequential output for source timing Tdel (treat as register) */
                    if (is_global_clock(iblk, isub, opin,
                             num_subblocks_per_block,
                             subblock_inf)) {
                        tedge[0].Tdel = 0.;
                    } else
                        tedge[0].Tdel =
                            type->type_timing_inf.
                            T_subblock[isub].T_seq_out[opin];

                    tnode_descript[inode + 1].type =
                        INPAD_SOURCE;
                    tnode_descript[inode + 1].ipin = OPEN;
                    tnode_descript[inode + 1].isubblk = isub;
                    tnode_descript[inode + 1].iblk = iblk;
                } else {
                    tnode_descript[inode].type = SUBBLK_OPIN;
                }

                tnode_descript[inode].ipin = opin;
                tnode_descript[inode].isubblk = isub;
                tnode_descript[inode].iblk = iblk;
            }
        }
    }

    /* First pass, load the subblock input pins to output pins without connecting edges to output pins.
     * If the subblock is used in sequential mode (i.e. is clocked), the two clock pin nodes.
     * Connect edges to output pins and take care of constant generators in second pass
     */
    for (isub = 0; isub < num_subs; isub++) {
        has_outputs = FALSE;

        for (opin = 0; opin < type->max_subblock_outputs; opin++) {
            if (sub_pin_to_tnode[isub][SUB_OUTPUT][opin] != OPEN) {
                has_outputs = TRUE;
            }
        }

        if (!has_outputs && type != IO_TYPE) {
            /* Empty, so skip */
            continue;
        }

        if (sub_inf[isub].clock != OPEN) {
            /* Sequential mode */
            inode = sub_pin_to_tnode[isub][SUB_CLOCK][clk_pin];

            /* Each output of a subblock has a flip-flop sink and source */
            for (opin = 0; opin < type->max_subblock_outputs; opin++) {
                if (sub_inf[isub].outputs[opin] != OPEN) {
                    /* First node is the clock input pin; it feeds the sequential output */
                    tnode[inode].num_edges = 1;
                    tnode[inode].out_edges = (edge_t*)
                                             my_chunk_malloc(sizeof(edge_t),
                                                             &tedge_ch_list_head,
                                                             &tedge_ch_bytes_avail,
                                                             &tedge_ch_next_avail);

                    tnode_descript[inode].type = FF_SOURCE;
                    tnode_descript[inode].ipin = OPEN;
                    tnode_descript[inode].isubblk = isub;
                    tnode_descript[inode].iblk = iblk;

                    /* Now create the "sequential sink" -- i.e. the FF input node. */

                    inode++;
                    tnode[inode].num_edges = 0;
                    tnode[inode].out_edges = NULL;

                    tnode_descript[inode].type = FF_SINK;
                    tnode_descript[inode].ipin = OPEN;
                    tnode_descript[inode].isubblk = isub;
                    tnode_descript[inode].iblk = iblk;
                    inode++;
                }
            }
        }

        /* Build and hook up subblock inputs. */

        for (ipin = 0; ipin < type->max_subblock_inputs; ipin++) {
            inode = sub_pin_to_tnode[isub][SUB_INPUT][ipin];

            if (inode != OPEN) {
                /* tnode exists->pin is used */
                if (type == IO_TYPE) {
                    /* Output pad */
                    tnode[inode].num_edges = 1;
                    opin = 0;
                    tnode[inode].out_edges = (edge_t*)
                                             my_chunk_malloc(sizeof(edge_t),
                                                             &tedge_ch_list_head,
                                                             &tedge_ch_bytes_avail,
                                                             &tedge_ch_next_avail);
                    tnode_descript[inode].type = OUTPAD_IPIN;
                    tnode[inode + 1].num_edges = 0;
                    tnode[inode + 1].out_edges = NULL;
                    tedge = tnode[inode].out_edges;
                    tedge[0].to_node = inode + 1;

                    /* For output pads, use subblock combinational time
                     * and sequential in for timing (treat as register) */
                    tedge[0].Tdel =
                        type->type_timing_inf.
                        T_subblock[isub].T_comb[ipin][opin] +
                        type->type_timing_inf.
                        T_subblock[isub].T_seq_in[opin];

                    tnode_descript[inode + 1].type =
                        OUTPAD_SINK;
                    tnode_descript[inode + 1].ipin = OPEN;
                    tnode_descript[inode + 1].isubblk = isub;
                    tnode_descript[inode + 1].iblk = iblk;
                } else {
                    tnode[inode].num_edges =
                        num_opin_used_in_sblk[isub];
                    tnode[inode].out_edges =
                        (edge_t*)
                        my_chunk_malloc(num_opin_used_in_sblk
                                        [isub] *
                                        sizeof(edge_t),
                                        &tedge_ch_list_head,
                                        &tedge_ch_bytes_avail,
                                        &tedge_ch_next_avail);
                    tnode[inode].num_edges =
                        num_opin_used_in_sblk[isub];
                    tnode_descript[inode].type = SUBBLK_IPIN;
                }

                tnode_descript[inode].ipin = ipin;
                tnode_descript[inode].isubblk = isub;
                tnode_descript[inode].iblk = iblk;
            }
        }
    }           /* End for each subblock */

    /* Second pass load the input pins to output pins array */
    /* Load the output pins tedge arrays. */
    for (isub = 0; isub < num_subs; isub++) {
        used_opin_count = 0;

        for (opin = 0; opin < type->max_subblock_outputs; opin++) {
            if (sub_pin_to_tnode[isub][SUB_OUTPUT][opin] == OPEN) {
                /* Empty, so skip */
                continue;
            }

            for (ipin = 0; ipin < type->max_subblock_inputs; ipin++) {
                /* sblk opin to sblk ipin */

                from_pin = sub_inf[isub].inputs[ipin];

                /* Not OPEN and comes from local subblock output? */

                if (from_pin >= type->num_pins) {
                    /* Convention for numbering netlist pins.
                     * Internal connections are numbered subblock_index + subblock_output_index + num_pins
                     */
                    from_sub =
                        (from_pin -
                         type->num_pins) /
                        type->max_subblock_outputs;
                    from_opin =
                        (from_pin -
                         type->num_pins) %
                        type->max_subblock_outputs;
                    inode =
                        sub_pin_to_tnode[from_sub][SUB_OUTPUT]
                        [from_opin];
                    to_node =
                        sub_pin_to_tnode[isub][SUB_INPUT]
                        [ipin];
                    tedge = tnode[inode].out_edges;
                    iedge = next_sblk_opin_edge[from_sub]
                            [from_opin]++;
                    tedge[iedge].to_node = to_node;
                    tedge[iedge].Tdel =
                        type->type_timing_inf.
                        T_sblk_opin_to_sblk_ipin;
                }
            }

            from_pin = sub_inf[isub].clock; /* sblk opin to sblk clock */

            /* Not OPEN and comes from local subblock output? */

            if (from_pin >= type->num_pins) {
                from_sub =
                    (from_pin -
                     type->num_pins) / type->max_subblock_outputs;
                from_opin =
                    (from_pin -
                     type->num_pins) % type->max_subblock_outputs;
                inode = sub_pin_to_tnode[from_sub][SUB_OUTPUT]
                        [from_opin];
                /* Feeds seq. output, one ff per output pin */
                to_node =
                    sub_pin_to_tnode[isub][SUB_CLOCK][clk_pin] +
                    2 * used_opin_count;
                tedge = tnode[inode].out_edges;
                iedge =
                    next_sblk_opin_edge[from_sub][from_opin]++;
                tedge[iedge].to_node = to_node;

                /* NB: Could make sblk opin to clk Tdel parameter; not worth it right now. */
                tedge[iedge].Tdel =
                    type->type_timing_inf.
                    T_sblk_opin_to_sblk_ipin;
            }

            to_pin = sub_inf[isub].outputs[opin];

            if (to_pin != OPEN) {
                /* sblk opin goes to fb opin? */

                /* Check that FB pin connects to something->    *
                 * not just a mandatory BLE to FB opin connection */

                if (blocks[iblk].nets[to_pin] != OPEN) {
                    to_node = node_block_pin_to_tnode[to_pin];
                    inode = sub_pin_to_tnode[isub][SUB_OUTPUT]
                            [opin];
                    tedge = tnode[inode].out_edges;
                    iedge = next_sblk_opin_edge[isub][opin]++;
                    tedge[iedge].to_node = to_node;
                    tedge[iedge].Tdel =
                        type->type_timing_inf.
                        T_sblk_opin_to_fb_opin;
                }
            }

            used_opin_count++;
        }
    }

    /* Now load the subblock input pins and, if the subblock is used in        *
     * sequential mode (i.e. is clocked), the two clock pin nodes for the output pin.              */
    for (isub = 0; isub < num_subs; isub++) {
        used_opin_count = 0;

        for (opin = 0; opin < type->max_subblock_outputs; opin++) {

            if (sub_pin_to_tnode[isub][SUB_OUTPUT][opin] == OPEN) { /* Empty, so skip */
                continue;
            }

            /* Begin loading */
            if (sub_inf[isub].clock == OPEN) {
                /* Combinational mode */
                to_node =
                    sub_pin_to_tnode[isub][SUB_OUTPUT][opin];
                sink_delay = 0;
            } else {
                /* Sequential mode.  Load two clock nodes. */
                inode =
                    sub_pin_to_tnode[isub][SUB_CLOCK][clk_pin] +
                    2 * used_opin_count;

                /* First node is the clock input pin; it feeds the sequential output */
                tedge = tnode[inode].out_edges;
                tedge[0].to_node =
                    sub_pin_to_tnode[isub][SUB_OUTPUT][opin];
                tedge[0].Tdel =
                    type->type_timing_inf.T_subblock[isub].
                    T_seq_out[opin];

                /* Now create the "sequential sink" -- i.e. the FF input node. */

                inode++;
                to_node = inode;
                sink_delay =
                    type->type_timing_inf.T_subblock[isub].
                    T_seq_in[opin];
            }

            /* Build and hook up subblock inputs. */

            has_inputs = FALSE;

            for (ipin = 0; ipin < type->max_subblock_inputs; ipin++) {
                inode = sub_pin_to_tnode[isub][SUB_INPUT][ipin];

                if (inode != OPEN) {
                    /* tnode exists->pin is used */
                    has_inputs = TRUE;
                    tedge = tnode[inode].out_edges;
                    tedge[used_opin_count].to_node = to_node;
                    tedge[used_opin_count].Tdel =
                        sink_delay +
                        type->type_timing_inf.
                        T_subblock[isub].T_comb[ipin][opin];
                }
            }

            if (!has_inputs && type != IO_TYPE) {
                /* Constant generator.  Give fake input. */

                inode =
                    sub_pin_to_tnode[isub][SUB_OUTPUT][opin] + 1;
                tnode[inode].num_edges = 1;
                tnode[inode].out_edges =
                    (edge_t*) my_chunk_malloc(sizeof(edge_t),
                                               &tedge_ch_list_head,
                                               &tedge_ch_bytes_avail,
                                               &tedge_ch_next_avail);
                tedge = tnode[inode].out_edges;
                tedge[used_opin_count].to_node = to_node;

                /* Want constants generated early so they never affect the critical path. */

                tedge[used_opin_count].Tdel =
                    T_CONSTANT_GENERATOR;

                tnode_descript[inode].type = CONSTANT_GEN_SOURCE;
                tnode_descript[inode].ipin = OPEN;
                tnode_descript[inode].isubblk = isub;
                tnode_descript[inode].iblk = iblk;
            }

            used_opin_count++;
        }
    }

    free_matrix(next_sblk_opin_edge, 0, type->max_subblocks - 1, 0,
                sizeof(int));
    free(num_opin_used_in_sblk);
}

static boolean is_global_clock(int iblk,
                               int sub,
                               int subpin,
                               int* num_subblocks_per_block,
                               subblock_t** subblock_inf)
{
    /* Returns TRUE if the net driven by this block (which must be an INPAD) is  *
       (1) a global signal, and (2) used as a clock input to at least one block.
       Assumes that there is only one subblock in an IO */
    block_types_t type = blocks[iblk].type;
    assert(type == IO_TYPE);

    int inet = blocks[iblk].nets[subblock_inf[iblk][sub].outputs[subpin]];
    assert(inet != OPEN);
    if (!net[inet].is_global) {
        return FALSE;
    }

    int  ipin = 0;
    const int knum_net_pins = net[inet].num_net_pins;
    for (ipin = 1; ipin < knum_net_pins + 1; ++ipin) {
        int to_blk = net[inet].node_blocks[ipin];
        int to_pin = net[inet].node_block_pins[ipin];
        int isub = -1;
        for (isub = 0; isub < num_subblocks_per_block[to_blk]; ++isub) {
            if (subblock_inf[to_blk][isub].clock == to_pin) {
                return TRUE;
            }
        }
    }

    return  FALSE;
}  /* end of static boolean is_global_clock(int iblk,) */

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
        int inode = net_to_driver_tnode[inet];
        edge_t* tedge = tnode[inode].out_edges;
        sinks += (long)net[inet].num_net_pins;

        /* Note that the edges of a tnode corresponding to a FB or INPAD opin must*
         * be in the same order as the pins of the net driven by the tnode.       */
        int ipin = 0;
        const int knum_net_pins = net[inet].num_net_pins;
        for (ipin = 1; ipin < knum_net_pins + 1; ++ipin) {
            tedge[ipin - 1].Tdel = net_delay[inet][ipin];
            int to_node = tedge[ipin - 1].to_node;

            int counter = -1;
            for (counter = 0; counter < tnode[to_node].num_parents; ++counter) {
                if (tnode[to_node].in_edges[counter].to_node != inode) {
                    continue;
                }

                tnode[to_node].in_edges[counter].Tdel = net_delay[inet][ipin];
                break;
            }

            if (counter == tnode[to_node].num_parents) {
                printf("disrepency between fanout and fanin nodes\n");
                exit(-1);
            }
        }
    }

    return sinks;
} /* end of unsigned long load_timing_graph_net_delays_parallel() */

void load_timing_graph_net_delays(double** net_delay)
{

    /* Sets the delays of the inter-FB nets to the values specified by          *
      * net_delay[0..num_net  s-1][1..num_pins-1].  These net delays should have    *
     * been allocated and loaded with the net_delay routines.  This routine      *
     * marks the corresponding edges in the timing graph with the proper Tdel.  */

    int inet, ipin, inode;
    edge_t* tedge;

    for (inet = 0; inet < num_nets; inet++) {
        inode = net_to_driver_tnode[inet];
        tedge = tnode[inode].out_edges;

        /* Note that the edges of a tnode corresponding to a FB or INPAD opin must  *
         * be in the same order as the pins of the net driven by the tnode.          */
        const int knum_net_pins = net[inet].num_net_pins;
        for (ipin = 1; ipin < knum_net_pins + 1; ++ipin) {
            tedge[ipin - 1].Tdel = net_delay[inet][ipin];
        }
    }
}


void free_timing_graph(double** net_slack)
{

    /* Frees the timing graph data. */

    if (tedge_ch_list_head == NULL) {
        printf("Error in free_timing_graph: No timing graph to free.\n");
        exit(1);
    }

    free_chunk_memory(tedge_ch_list_head);
    free(tnode);
    free(tnode_descript);
    free(net_to_driver_tnode);
    free_ivec_vector(tnodes_at_level, 0, num_tnode_levels - 1);
    free(net_slack);

    tedge_ch_list_head = NULL;
    tedge_ch_bytes_avail = 0;
    tedge_ch_next_avail = NULL;

    tnode = NULL;
    tnode_descript = NULL;
    num_of_vertexs = 0;
    net_to_driver_tnode = NULL;
    tnodes_at_level = NULL;
    num_tnode_levels = 0;
}


void print_net_slack(char* fname,
                     double** net_slack)
{

    /* Prints the net slacks into a file.                                     */
    FILE* fp = my_fopen(fname, "w");
    fprintf(fp, "Net #\tSlacks\n\n");

    int inet, ipin;
    for (inet = 0; inet < num_nets; ++inet) {
        fprintf(fp, "%5d", inet);
        const int knum_net_pins = net[inet].num_net_pins;
        for (ipin = 1; ipin < knum_net_pins + 1; ++ipin) {
            fprintf(fp, "\t%g", net_slack[inet][ipin]);
        }

        fprintf(fp, "\n");
    }
}


void print_timing_graph(char* fname)
{

    /* Prints the timing graph into a file.           */

    FILE* fp;
    int inode, iedge, ilevel, i;
    edge_t* tedge;
    t_tnode_type itype;
    char* tnode_type_names[] = { "INPAD_SOURCE", "INPAD_OPIN",
                                 "OUTPAD_IPIN", "OUTPAD_SINK", "FB_IPIN", "FB_OPIN",
                                 "SUBBLK_IPIN", "SUBBLK_OPIN", "FF_SINK", "FF_SOURCE",
                                 "CONSTANT_GEN_SOURCE"
                               };


    fp = my_fopen(fname, "w");

    fprintf(fp, "num_of_vertexs: %d\n", num_of_vertexs);
    fprintf(fp, "Node #\tType\t\tipin\tisubblk\tiblk\t# edges\t"
            "Edges (to_node, Tdel)\n\n");

    for (inode = 0; inode < num_of_vertexs; inode++) {
        fprintf(fp, "%d\t", inode);

        itype = tnode_descript[inode].type;
        fprintf(fp, "%-15.15s\t", tnode_type_names[itype]);

        fprintf(fp, "%d\t%d\t%d\t", tnode_descript[inode].ipin,
                tnode_descript[inode].isubblk,
                tnode_descript[inode].iblk);

        fprintf(fp, "%d\t", tnode[inode].num_edges);
        tedge = tnode[inode].out_edges;

        for (iedge = 0; iedge < tnode[inode].num_edges; iedge++) {
            fprintf(fp, "\t(%4d,%7.3g)", tedge[iedge].to_node,
                    tedge[iedge].Tdel);
        }

        fprintf(fp, "\n");
    }

    fprintf(fp, "\n\nnum_tnode_levels: %d\n", num_tnode_levels);

    for (ilevel = 0; ilevel < num_tnode_levels; ilevel++) {
        fprintf(fp, "\n\nLevel: %d  Num_nodes: %d\nNodes:", ilevel,
                tnodes_at_level[ilevel].nelem);

        for (i = 0; i < tnodes_at_level[ilevel].nelem; i++) {
            fprintf(fp, "\t%d", tnodes_at_level[ilevel].list[i]);
        }
    }

    fprintf(fp, "\n");
    fprintf(fp, "\n\nNet #\tNet_to_driver_tnode\n");

    for (i = 0; i < num_nets; i++) {
        fprintf(fp, "%4d\t%6d\n", i, net_to_driver_tnode[i]);
    }

    fprintf(fp, "\n\nNode #\t\tT_arr\t\tT_req\n\n");

    for (inode = 0; inode < num_of_vertexs; inode++)
        fprintf(fp, "%d\t%12g\t%12g\n", inode, tnode[inode].arr_time,
                tnode[inode].req_time);

    fclose(fp);
}


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
        int num_edges = tnode[inode].num_parents;
        edge_t* tedge = tnode[inode].in_edges;

        int iedge = -1;
        for (iedge = 0; iedge < num_edges; ++iedge) {
            int   from_node = tedge[iedge].to_node;
            double arr_time = tnode[from_node].arr_time;
            double Tdel = tedge[iedge].Tdel;
            tnode[inode].arr_time = max(tnode[inode].arr_time, arr_time + Tdel);
        }

        T_crit = max(T_crit, tnode[inode].arr_time);
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

        int num_edges = tnode[inode].num_edges;
        if (num_edges == 0) {
            /* this node was in last level. It's a sink node. */
            tnode[inode].req_time = T_cycle;
        } else {
            edge_t* tedge = tnode[inode].out_edges;
            int    to_node = tedge[0].to_node;
            double  Tdel = tedge[0].Tdel;
            double  req_time = tnode[to_node].req_time - Tdel;

            int iedge = 0;
            for (iedge = 1; iedge < num_edges; ++iedge) {
                to_node = tedge[iedge].to_node;
                Tdel = tedge[iedge].Tdel;
                req_time = min(req_time, tnode[to_node].req_time - Tdel);
            }
            tnode[inode].req_time = req_time;
        }
    }  /* end of for (i = start_node; i < finish_node; ++i) */
}  /* end of void calc_tnodes_req_time_parallel(double** net_slack,..) */

/* Determines the slack of every source-sink pair of block pins in the      *
 * circuit.  The timing graph must have already been built.  target_cycle_  *
 * time is the target Tdel for the circuit -- if 0, the target_cycle_time  *
 * is set to the critical path found in the timing graph.  This routine     *
 * loads net_slack, and returns the current critical path Tdel.            */
double load_net_slack(double** net_slack,
                     double target_cycle_time)
{
    double T_crit, arr_time, Tdel, T_cycle, req_time;
    int inode, ilevel, num_at_level, i, num_edges, iedge, to_node;
    edge_t* tedge;

    /* Reset all arrival times to -ve infinity.  Can't just set to zero or the   *
     * constant propagation (constant generators work at -ve infinity) won't     *
     * work.                                                                     */
    for (inode = 0; inode < num_of_vertexs; ++inode) {
        tnode[inode].arr_time = T_CONSTANT_GENERATOR;
    }

    /* Compute all arrival times with a breadth-first analysis from inputs to   *
     * outputs.  Also compute critical path (T_crit).                           */
    T_crit = 0.;

    /* Primary inputs arrive at T = 0. */
    num_at_level = tnodes_at_level[0].nelem;

    for (i = 0; i < num_at_level; i++) {
        inode = tnodes_at_level[0].list[i];
        tnode[inode].arr_time = 0.;
    }

    for (ilevel = 0; ilevel < num_tnode_levels; ilevel++) {
        num_at_level = tnodes_at_level[ilevel].nelem;

        for (i = 0; i < num_at_level; i++) {
            inode = tnodes_at_level[ilevel].list[i];
            arr_time = tnode[inode].arr_time;
            num_edges = tnode[inode].num_edges;
            tedge = tnode[inode].out_edges;
            T_crit = max(T_crit, arr_time);

            for (iedge = 0; iedge < num_edges; iedge++) {
                to_node = tedge[iedge].to_node;
                Tdel = tedge[iedge].Tdel;
                tnode[to_node].arr_time =
                    max(tnode[to_node].arr_time, arr_time + Tdel);
            }
        }

    }

    if (target_cycle_time > 0.) { /* User specified target cycle time */
        T_cycle = target_cycle_time;
    } else {        /* Otherwise, target = critical path */
        T_cycle = T_crit;
    }

    /* Compute the required arrival times with a backward breadth-first analysis *
     * from sinks (output pads, etc.) to primary inputs.                         */

    for (ilevel = num_tnode_levels - 1; ilevel >= 0; ilevel--) {
        num_at_level = tnodes_at_level[ilevel].nelem;

        for (i = 0; i < num_at_level; i++) {
            inode = tnodes_at_level[ilevel].list[i];
            num_edges = tnode[inode].num_edges;

            if (num_edges == 0) {
                /* sink */
                tnode[inode].req_time = T_cycle;
            } else {
                tedge = tnode[inode].out_edges;
                to_node = tedge[0].to_node;
                Tdel = tedge[0].Tdel;
                req_time = tnode[to_node].req_time - Tdel;

                for (iedge = 1; iedge < num_edges; iedge++) {
                    to_node = tedge[iedge].to_node;
                    Tdel = tedge[iedge].Tdel;
                    req_time =
                        min(req_time,
                            tnode[to_node].req_time - Tdel);
                }

                tnode[inode].req_time = req_time;
            }
        }
    }

    compute_net_slacks(net_slack);

    return (T_crit);
}  /* end of load_net_slacks() */

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
        const int driver_node = net_to_driver_tnode[inet];
        const double arr_time = tnode[driver_node].arr_time;
        edge_t* tedge = tnode[driver_node].out_edges;

        const int num_edges = tnode[driver_node].num_edges;
        count_work += (unsigned long int)num_edges;

        int iedge = -1;
        for (iedge = 0; iedge < num_edges; ++iedge) {
            const int to_node = tedge[iedge].to_node;
            const double Tdel = tedge[iedge].Tdel;
            const double req_time = tnode[to_node].req_time;
            net_slack[inet][iedge + 1] = req_time - arr_time - Tdel;
        }  /* compute a few nets' slack OK! */
    }

    return count_work;
} /* end of unsigned long compute_net_slacks_parallel(double** net_slack, int start_net, int finish_net) */

static void compute_net_slacks(double** net_slack)
{
    /* Puts the slack of each source-sink pair of block pins in net_slack. */
    int inet, iedge, inode, to_node, num_edges;
    edge_t* tedge;
    double arr_time, Tdel, req_time;

    for (inet = 0; inet < num_nets; inet++) {
        inode = net_to_driver_tnode[inet];
        arr_time = tnode[inode].arr_time;
        num_edges = tnode[inode].num_edges;
        tedge = tnode[inode].out_edges;

        for (iedge = 0; iedge < num_edges; iedge++) {
            to_node = tedge[iedge].to_node;
            Tdel = tedge[iedge].Tdel;
            req_time = tnode[to_node].req_time;
            net_slack[inet][iedge + 1] = req_time - arr_time - Tdel;
        }
    }
}

/* Finds the critical path and puts a list of the tnodes on the critical    *
 * path in a linked list, from the path SOURCE to the path SINK.            */
t_linked_int* allocate_and_load_critical_path(void)
{

    t_linked_int* critical_path_head, *curr_crit_node, *prev_crit_node;
    int inode, iedge, to_node, num_at_level, i, crit_node, num_edges;
    double min_slack, slack;
    edge_t* tedge;

    num_at_level = tnodes_at_level[0].nelem;
    min_slack = HUGE_FLOAT;
    crit_node = OPEN;       /* Stops compiler warnings. */

    for (i = 0; i < num_at_level; i++) {
        /* Path must start at SOURCE (no inputs) */
        inode = tnodes_at_level[0].list[i];
        slack = tnode[inode].req_time - tnode[inode].arr_time;

        if (slack < min_slack) {
            crit_node = inode;
            min_slack = slack;
        }
    }

    critical_path_head = (t_linked_int*) my_malloc(sizeof(t_linked_int));
    critical_path_head->data = crit_node;
    prev_crit_node = critical_path_head;
    num_edges = tnode[crit_node].num_edges;

    while (num_edges != 0) {
        /* Path will end at SINK (no fanout) */
        curr_crit_node = (t_linked_int*) my_malloc(sizeof(t_linked_int));
        prev_crit_node->next = curr_crit_node;
        tedge = tnode[crit_node].out_edges;
        min_slack = HUGE_FLOAT;

        for (iedge = 0; iedge < num_edges; iedge++) {
            to_node = tedge[iedge].to_node;
            slack = tnode[to_node].req_time - tnode[to_node].arr_time;

            if (slack < min_slack) {
                crit_node = to_node;
                min_slack = slack;
            }
        }

        curr_crit_node->data = crit_node;
        prev_crit_node = curr_crit_node;
        num_edges = tnode[crit_node].num_edges;
    }

    prev_crit_node->next = NULL;
    return (critical_path_head);
}


void
get_tnode_block_and_output_net(int inode,
                               int* iblk_ptr,
                               int* inet_ptr)
{

    /* Returns the index of the block that this tnode is part of.  If the tnode *
     * is a FB_OPIN or INPAD_OPIN (i.e. if it drives a net), the net index is  *
     * returned via inet_ptr.  Otherwise inet_ptr points at OPEN.               */

    int inet, ipin, iblk;
    t_tnode_type tnode_type;

    iblk = tnode_descript[inode].iblk;
    tnode_type = tnode_descript[inode].type;

    if (tnode_type == FB_OPIN || tnode_type == INPAD_OPIN) {
        ipin = tnode_descript[inode].ipin;
        inet = blocks[iblk].nets[ipin];
    } else {
        inet = OPEN;
    }

    *iblk_ptr = iblk;
    *inet_ptr = inet;
}


void
do_constant_net_delay_timing_analysis(timing_info_t timing_inf,
                                      subblock_data_t subblock_data,
                                      double constant_net_delay_value)
{

    /* Does a timing analysis (simple) where it assumes that each net has a      *
     * constant Tdel value.  Used only when operation == TIMING_ANALYSIS_ONLY.  */

    linked_vptr_t* net_delay_chunk_list_head;
    double** net_delay, **net_slack;

    double T_crit;

    net_slack = alloc_and_load_timing_graph(timing_inf, subblock_data);
    net_delay = alloc_net_delay(&net_delay_chunk_list_head);

    load_constant_net_delay(net_delay, constant_net_delay_value);
    load_timing_graph_net_delays(net_delay);
    T_crit = load_net_slack(net_slack, 0);

    printf("\n");
    printf("\nCritical Path: %g (s)\n", T_crit);

#ifdef CREATE_ECHO_FILES
    print_critical_path("critical_path.echo", subblock_data);
    print_timing_graph("timing_graph.echo");
    print_net_slack("net_slack.echo", net_slack);
    print_net_delay(net_delay, "net_delay.echo");
#endif /* CREATE_ECHO_FILES */

    free_timing_graph(net_slack);
    free_net_delay(net_delay, &net_delay_chunk_list_head);
}
