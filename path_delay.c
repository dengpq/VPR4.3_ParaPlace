#include <stdio.h>
#include <assert.h>

#include "util.h"
#include "vpr_types.h"
#include "vpr_utils.h"
#include "globals.h"
#include "path_delay.h"
#include "path_delay2.h"
#include "net_delay.h"
#include "place.h"
#include "string.h"

#define T_CONSTANT_GENERATOR -1000   /* Essentially -ve infinity */

/****************** Timing graph Structure ************************************
 * In the timing graph I create, input pads and constant generators have no   *
 * inputs; everything else has inputs.  Every input pad and output pad is     *
 * represented by two tnodes -- an input pin and an output pin.  For an input *
 * pad the input pin comes from off chip and has no fanin, while the output   *
 * pin drives outpads and/or CLBs.  For output pads, the input node is driven *
 * by a CLB_TYPE or input pad, and the output node goes off chip and has no        *
 * fanout (out-edges).  I need two nodes to respresent things like pads       *
 * because I mark all Tdel on tedges, not on tnodes.                         *
 *                                                                            *
 * Every used (not OPEN) CLB_TYPE pin becomes a timing node.  As well, every used  *
 * subblock pin within a CLB_TYPE also becomes a timing node.  Unused (OPEN) pins  *
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
 * magnitude negative number.  This guarantees every blocks that needs the     *
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
 ******************************************************************************/

/***************** Variables local to this module ***************************/
/* Variables for "chunking" the tedge memory.  If the head pointer is NULL, *
 * no timing graph exists now.                                              */
extern double ground_num; /* in main.c, for NEW_TIMING_DRIVEN_PLACE */
static linked_vptr_t* tedge_ch_list_head = NULL;
static int tedge_ch_bytes_avail = 0;
static char* tedge_ch_next_avail = NULL;

/***************** Subroutines local to this module *************************/
static int alloc_and_load_pin_mappings(int** *block_pin_to_tnode_ptr,
                                       int**** sblk_pin_to_tnode_ptr,
                                       subblock_data_t subblock_data,
                                       int** num_uses_of_sblk_opin);

static void free_pin_mappings(int**  block_pin_to_tnode,
                              int*** sblk_pin_to_tnode,
                              int* num_subblocks_per_block);

static void alloc_and_load_fanout_counts(int** *num_uses_of_clb_ipin_ptr,
                                         int** *num_uses_of_sblk_opin_ptr,
                                         subblock_data_t subblock_data);

static void free_fanout_counts(int** num_uses_of_clb_ipin,
                               int** num_uses_of_sblk_opin);
/* First, reset all vertexes front_crit_path_through_pin and *
 * behind_crit_path_through_pin 0.                          */
static void reset_all_tnodes_front_behind_crit_path(void);

/* Second, front_crit_path_through_pin[primary_input] = 1 */
static void assign_front_crit_path_through_pin_to_primary_inputs(void);

/* Third, calc all vertexes front_crit_path_through_pin */
static void calc_all_tnodes_front_crit_path_through_pin(const double crit_delay);

/* Forth, set behind_crit_path_through_pin[primary_outputs] = 1 */
static void assign_behind_crit_path_through_pin_to_primary_outputs(void);

/* Fifth, calc all vertexes behind_crit_path_through_pin */
static void calc_all_tnodes_behind_crit_path_through_pin(const double crit_delay);

/* Sixth,calc all point-to-point nets of no-global nets local_crit_weight[inet][ipin] */
static void calc_subnet_local_crit_weight(double** net_slack,
                                          const double crit_delay,
                                          int** block_pin_to_tnode);

static double compute_discount_value(const double ground_num,
                                     const double x,
                                     const double y);
/*******            New added end             **********/

static void alloc_and_load_tnodes_and_net_mapping(int**  num_uses_of_clb_ipin,
                                                  int**  num_uses_of_sblk_opin,
                                                  int**  block_pin_to_tnode,
                                                  int*** sblk_pin_to_tnode,
                                                  subblock_data_t  subblock_data,
                                                  timing_info_t  timing_inf);

static void build_clb_tnodes(int iblk,
                             int* n_uses_of_clb_ipin,
                             int** block_pin_to_tnode,
                             int** sub_pin_to_tnode,
                             int subblock_lut_size,
                             int subblks_in_block,
                             subblock_t* sub_inf,
                             double T_clb_ipin_to_sblk_ipin,
                             int* next_clb_ipin_edge);

static void build_subblock_tnodes(int* n_uses_of_sblk_opin,
                                  int* blk_pin_to_tnode,
                                  int** sub_pin_to_tnode,
                                  int subblock_lut_size,
                                  int subblks_in_block,
                                  subblock_t* sub_inf,
                                  double T_sblk_opin_to_sblk_ipin,
                                  double T_sblk_opin_to_clb_opin,
                                  T_subblock_t* T_subblock,
                                  int* next_sblk_opin_edge,
                                  int iblk);

static void build_ipad_tnodes(int   iblk,
                              int** block_pin_to_tnode,
                              double T_ipad,
                              int* num_subblocks_per_block,
                              subblock_t** subblock_inf);

static boolean is_global_clock(int iblk,
                               int* num_subblocks_per_block,
                               subblock_t** subblock_inf);

static void build_opad_tnodes(int* blk_pin_to_tnode,
                              double T_opad, int iblk);

static void build_block_output_tnode(int ivex,
                                     int iblk,
                                     int ipin,
                                     int** block_pin_to_tnode);


/********************* Subroutine definitions *******************************/
void free_subblock_data(subblock_data_t* subblock_data_ptr)
{
    /* Frees all the subblock data structures. */
    free_chunk_memory(subblock_data_ptr->chunk_head_ptr);
    free(subblock_data_ptr->num_subblocks_per_block);
    free(subblock_data_ptr->subblock_inf);
    /* Mark as freed. */
    subblock_data_ptr->num_subblocks_per_block = NULL;
    subblock_data_ptr->subblock_inf = NULL;
    subblock_data_ptr->chunk_head_ptr = NULL;
}


/* FIXME:This routine builds the graph used for Timing-Analysis. Every CLB_TYPE *
 *       or subblock pin is a timing node(vertexes). The connectivity between *
 *       pins is represented by timing edges(tedges). All Tdel is marked  *
 *       on edges, not on nodes. This routine returns an array that will   *
 *       store slack values.                                               */
void alloc_and_load_timing_graph(placer_opts_t   placer_opts,
                                 timing_info_t   timing_inf,
                                 subblock_data_t subblock_data)
{
    if (tedge_ch_list_head != NULL) {
        printf("Error in alloc_and_load_timing_graph:\n"
               "\tAn old timing graph still exists.\n");
        exit(1);
    }

    /* If either of the checks below ever fail, change the definition of *
     * tnode_descript to use ints instead of shorts for isubblk or ipin. */
    if (subblock_data.max_subblocks_per_block > MAX_SHORT) {
        printf("Error in alloc_and_load_timing_graph: max_subblocks_per_block"
               "\tis %d -- will cause short overflow in tnode_descript.\n",
               subblock_data.max_subblocks_per_block);
        exit(1);
    }
    if (pins_per_clb > MAX_SHORT) {
        printf("Error in alloc_and_load_timing_graph: pins_per_clb is %d."
               "\tWill cause short overflow in tnode_descript.\n", pins_per_clb);
        exit(1);
    }

    /* The two arrays below are valid only for CLBs, not pads. */
    /* [0..num_blocks-1][0..pins_per_clb-1] */
    int** num_uses_of_clb_ipin = NULL;
    /* [0..num_blocks-1][0..num_subs_per[iblk]-1] */
    int** num_uses_of_sblk_opin = NULL;
    /*FIXME: this function was most important in alloc_and_load_timing_graph().*
     *       It initial all the pin-net connections in circuit netlist blocks   *
     *       or subblocks.                                                     */
    alloc_and_load_fanout_counts(&num_uses_of_clb_ipin,
                                 &num_uses_of_sblk_opin,
                                 subblock_data);

    /*=== Then initial all the blocks and its subblocks <pin, pin_number> index ===*/
    /* block_pin_to_tnode[iblk][ipin] = curr_node++;                       *
     * sblk_pin_to_tnode[iblk][isub][ipin] = curr_node++;                  */
    num_of_vertexs = alloc_and_load_pin_mappings(&block_pin_to_tnode,
                                                 &sblk_pin_to_tnode,
                                                 subblock_data,
                                                 num_uses_of_sblk_opin);
    printf("Now check block_pin_to_tnode[%d][%d]...\n",
           num_blocks,
           pins_per_clb);
    check_block_pin_to_tnode(block_pin_to_tnode);

    /* then create the 2 arrays needed by PATH algo */
    if (placer_opts.place_algorithm == NEW_TIMING_DRIVEN_PLACE) {
        front_crit_path_through_pin = (double*)my_malloc(num_of_vertexs
                                                           * sizeof(double));
        behind_crit_path_through_pin = (double*)my_malloc(num_of_vertexs
                                                            * sizeof(double));
    }

    /* FIXME: Then initial all tnodes to create Timing_Graph */
    alloc_and_load_tnodes_and_net_mapping(num_uses_of_clb_ipin,
                                          num_uses_of_sblk_opin,
                                          block_pin_to_tnode,
                                          sblk_pin_to_tnode,
                                          subblock_data,
                                          timing_inf);

    /* Initial the all tedges and Timing_Graph levels */
    int num_sinks = alloc_and_load_timing_graph_edge_levels();

    check_timing_graph(subblock_data.num_const_gen,
                       subblock_data.num_ff,
                       num_sinks);

    check_vertexs_levels(); /* New added */

    free_fanout_counts(num_uses_of_clb_ipin,
                       num_uses_of_sblk_opin);
    /* free_pin_mappings(block_pin_to_tnode,
                         sblk_pin_to_tnode,
                         subblock_data.num_subblocks_per_block); */
} /* end of double** alloc_and_load_timing_graph() */

/* First, front_crit_path_through_pin[all_vertexs] = 0.0, *
 * behind_crit_path_through_pin[all_vertexs] = 0.0        */
static void reset_all_tnodes_front_behind_crit_path(void)
{
    int v = -1;
    for (v = 0; v < num_of_vertexs; ++v) {
        front_crit_path_through_pin[v] = 0.0;
        behind_crit_path_through_pin[v] = 0.0;
    }
}

/* Second, front_crit_path_through_pin[primary_inputs] = 1 */
static void assign_front_crit_path_through_pin_to_primary_inputs(void)
{
    const int num_at_level_0 = tnodes_at_level[0].nelem;
    int i = -1;
    for (i = 0; i < num_at_level_0; ++i) {
        const int ivex = tnodes_at_level[0].list[i];
        front_crit_path_through_pin[ivex] = 1.0;
    }
}

/* It required subnet_forward_local_slack[inet][ipin]; net_t net;
* double** net_delay and double crit_delay.          *
* Third, front_crit_path_through_pin[tail] +=
*    discount * front_crit_path_through_pin[source] */
static void calc_all_tnodes_front_crit_path_through_pin(const double crit_delay)
{
    int ilevel = -1;
    int v = -1;
    int iedge = -1;
    for (ilevel = 0; ilevel < num_tnode_levels; ++ilevel) {
        const int num_at_level = tnodes_at_level[ilevel].nelem;
        for (v = 0; v < num_at_level; ++v) {
            const int    from_node = tnodes_at_level[ilevel].list[v];
            const double source_arr_time = vertexes[from_node].arr_time;

            edge_t*   conn_edges = vertexes[from_node].out_edges;
            const int num_conn_edges = vertexes[from_node].num_edges;
            for (iedge = 0; iedge < num_conn_edges; ++iedge) {
                const int    to_node = conn_edges[iedge].to_node;
                const double sink_arr_time = vertexes[to_node].arr_time;
                const double delay = conn_edges[iedge].Tdel;

                const double forward_local_slack =
                                        sink_arr_time - source_arr_time - delay;

                const double discount_value =
                                     compute_discount_value(ground_num,
                                                            forward_local_slack,
                                                            crit_delay);
                front_crit_path_through_pin[to_node] +=
                        discount_value * front_crit_path_through_pin[from_node];
            } /* end of for(iedge = 0; iedge < num_conn_edge; ++iedge) */
        } /* end of for(v = 0; v < num_at_level; ++v) */
    } /* end of for(ilevel = 0; ilevel < num_tnode_levels; ++ilevle) */
}

/* Forth, behind_crit_path_through_pin[primary_outputs] = 1 */
static void assign_behind_crit_path_through_pin_to_primary_outputs(void)
{
    const int num_at_last_level = tnodes_at_level[num_tnode_levels - 1].nelem;
    int i = -1;
    for (i = 0; i < num_at_last_level; ++i) {
        const int ivex = tnodes_at_level[num_tnode_levels-1].list[i];
        behind_crit_path_through_pin[ivex] = 1.0;
    }
}

/* Fifth, behind_crit_path_through_pin[source] +=
 *              discount * behind_crit_path_through_pin[tail] */
static void calc_all_tnodes_behind_crit_path_through_pin(const double crit_delay)
{
    int ilevel = -1;
    int v = -1;
    int iedge = -1;
    for (ilevel = num_tnode_levels - 1; ilevel >= 0; --ilevel) {
        const int num_at_level = tnodes_at_level[ilevel].nelem;
        for (v = 0; v < num_at_level; ++v) {
            const int    from_node = tnodes_at_level[ilevel].list[v];
            const double source_req_time = vertexes[from_node].req_time;

            edge_t* conn_edges = vertexes[from_node].out_edges;
            const int  num_conn_edges = vertexes[from_node].num_edges;
            for (iedge = 0; iedge < num_conn_edges; ++iedge) {
                const double delay = conn_edges[iedge].Tdel;
                const int    to_node = conn_edges[iedge].to_node;
                const double sink_req_time = vertexes[to_node].req_time;

                const double backward_local_slack =
                                      sink_req_time - source_req_time - delay;

                const double discount_value =
                                    compute_discount_value(ground_num,
                                                           backward_local_slack,
                                                           crit_delay);
                behind_crit_path_through_pin[from_node] +=
                       discount_value * behind_crit_path_through_pin[to_node];
            } /* end of for(iedge = 0; iedge < num_conn_edges; ++iedge) */
        } /* end of for(v = 0; v < num_at_level; ++v) */
    } /* end of for(ilevel = num_tnode_levels-1; ilevel >= 0; --ilevel) */
}
/*******************************************************************************
 * Sixth, subnet_local_crit_weight[inet][ipin] = front_crit_path_pin[source]
 *         * behind_crit_path_pin[sink] * discount_value(net_slack[inet][ipin],T)
 ******************************************************************************/
static void calc_subnet_local_crit_weight(double** net_slack,
                                          const double crit_delay,
                                          int** block_pin_to_tnode)
{
    int inet = -1;
    int source_vertex_idx = -1;
    int sink_vertex_idx = -1;
    double discount_value = 0.0;
    for (inet = 0; inet < num_nets; ++inet) {
        if (is_global[inet] == FALSE) {
            source_vertex_idx = locate_source_vertex_index_by_net_index(inet);

            int ipin = 0;
            for (ipin = 1; ipin < net[inet].num_pins; ++ipin) {
                discount_value = compute_discount_value(ground_num,
                                                        net_slack[inet][ipin],
                                                        crit_delay);
                sink_vertex_idx =
                      find_sink_vertex_index_by_net_and_pin_index(inet,
                                                                  ipin,
                                                                  block_pin_to_tnode);
                subnet_local_crit_weight[inet][ipin] =
                                front_crit_path_through_pin[source_vertex_idx]
                                  * behind_crit_path_through_pin[sink_vertex_idx]
                                  * discount_value;
            }
        }
    }
} /* end of void calc_subnet_local_crit_weight() */

static double compute_discount_value(const double ground_num,
                                     const double x,
                                     const double y)
{
    assert(ground_num > 0.0 && y != 0);
    return pow(ground_num, -x / y);
}

/* FIXME: Main flow of NEW_TIMING_DRIVEN_PLACE */
void compute_all_nets_local_crit_weight(double** net_slack,
                                        int** block_pin_to_tnode,
                                        const double critical_delay)
{
    reset_all_tnodes_front_behind_crit_path();

    assign_front_crit_path_through_pin_to_primary_inputs();
    calc_all_tnodes_front_crit_path_through_pin(critical_delay);

    assign_behind_crit_path_through_pin_to_primary_outputs();
    calc_all_tnodes_behind_crit_path_through_pin(critical_delay);

    calc_subnet_local_crit_weight(net_slack,
                                  critical_delay,
                                  block_pin_to_tnode);
}  /* end of void compute_all_nets_local_crit_weight() */

int locate_source_vertex_index_by_net_index(const int net_index)
{
    assert(driver_node_index_of_net != NULL);

    return driver_node_index_of_net[net_index];
}

int find_sink_vertex_index_by_net_and_pin_index(const int net_index,
                                                const int sink_pin_index,
                                                int** block_pin_to_tnode)
{
    assert(net != NULL && block_pin_to_tnode != NULL && (net_index >= 0
            && net_index <= num_nets - 1));
    /* first, according to net_index, locate the connected CLB_TYPE */
    const int block_index = net[net_index].blocks[sink_pin_index];

    /* according to sink_pin_index, locate the accurate CLB_TYPE's input_pin */
    const int block_pin_index = net[net_index].blk_pin[sink_pin_index];

    /* Last, according to block_pin_to_tnode, get sink_vertex_idx. */
    int sink_vertex_idx = block_pin_to_tnode[block_index][block_pin_index];

    return sink_vertex_idx;
}

void check_block_pin_to_tnode(int** block_pin_to_tnode)
{
    FILE*  output_file = fopen("check_block_pin_to_tnode.txt",
                               "w");
    char  message[BUFSIZE] = "";
    int iblock = -1;
    int ipin = -1;
    for (iblock = 0; iblock < num_blocks; ++iblock) {
        for (ipin = 0; ipin < pins_per_clb; ++ipin) {
            sprintf(message, "block_pin_to_tnode[%d][%d] = %d\n",
                    iblock, ipin, block_pin_to_tnode[iblock][ipin]);
            fputs(message, output_file);
        }
    }
    fclose(output_file);
}  /* end of void check_block_pin_to_tnode(void) */

void  check_vertexs_levels()
{
    int vertexs_appear_count[num_of_vertexs];
    int v = -1;
    for (v = 0; v < num_of_vertexs; ++v) {
        vertexs_appear_count[v] = 0;
    }

    FILE* file = fopen("check_all_vertexs_levels.txt", "w");
    char message[BUFSIZE * 300];
    char tmp[8];

    sprintf(message, "It had %d levles", num_tnode_levels);
    fputs(message, file);
    int l = -1;
    for (l = 0; l < num_tnode_levels; ++l) {
        const int vertexs_at_level = tnodes_at_level[l].nelem;
        sprintf(message, "\nLevel %d, it had %d vertexes\n",
                l, vertexs_at_level);
        fputs(message, file);

        strcpy(message, "");
        for (v = 0; v < vertexs_at_level; ++v) {
            const int vertex_index = tnodes_at_level[l].list[v];
            ++vertexs_appear_count[vertex_index];

            strcat(message, "V");
            sprintf(tmp, "%d", vertex_index);
            strcat(message, tmp);
            if (v != vertexs_at_level - 1) {
                strcat(message, ", ");
            }
        }
        fputs(message, file);
    }

    fputs("\nall vertexes appeared count: ", file);
    for (v = 0; v < num_of_vertexs; ++v) {
        sprintf(message, "\nV%d appeared count: %d", v, vertexs_appear_count[v]);
        fputs(message, file);
    }
    fclose(file);
}  /* end of void check_vertexs_levels() */

/* Allocates and loads the blocks and subblk pin mapping to vertexes    *
 * structures, and computes num_of_vertexs.                         */
static int alloc_and_load_pin_mappings(int***  block_pin_to_tnode_ptr,
                                       int**** sblk_pin_to_tnode_ptr,
                                       subblock_data_t subblock_data,
                                       int** num_uses_of_sblk_opin)
{
    const int subblock_lut_size = subblock_data.subblock_lut_size; /* LUT-k */
    /* INPUT[0..subblock_lut_size-1], OUTPUT, CLK */
    const int out_pin = subblock_lut_size;
    const int clk_pin = subblock_lut_size + 1;
    /* int num_subblocks_per_block[0...num_blocks-1] */
    int* num_subblocks_per_block = subblock_data.num_subblocks_per_block;
    /* subblock_inf[0...num_blocks-1][0...num_subblks_per_block[iblk]-1] */
    subblock_t** subblock_inf = subblock_data.subblock_inf;

    /* Attention: The most important parameters in current function! */
    int** block_pin_to_tnode = (int**)alloc_matrix(0,
                                                   num_blocks-1,
                                                   0,
                                                   pins_per_clb-1,
                                                   sizeof(int));
    int*** sblk_pin_to_tnode = (int***)my_malloc(num_blocks * sizeof(int**));


    int iblk = -1;
    int ipin = -1;
    int curr_tnode = 0;
    for (iblk = 0; iblk < num_blocks; ++iblk) {
        if (blocks[iblk].type == CLB_TYPE) {
            /* First deal with CLB_TYPE's pin mapping */
            for (ipin = 0; ipin < pins_per_clb; ++ipin) {
                if (blocks[iblk].nets[ipin] == OPEN) {
                    block_pin_to_tnode[iblk][ipin] = OPEN;
                } else {
                    block_pin_to_tnode[iblk][ipin] = curr_tnode++;
                }
            }
            /* Now do all subblocks in current netlist's blocks mapping. */
            const int num_subblocks = num_subblocks_per_block[iblk];
            /* subblock_lut_sizt+1 contained INPUT, OUTPUT and CLOCK */
            sblk_pin_to_tnode[iblk] = (int**)alloc_matrix(0,
                                                          num_subblocks - 1,
                                                          0,
                                                          subblock_lut_size + 1,
                                                          sizeof(int));
            int isub = -1;
            for (isub = 0; isub < num_subblocks; ++isub) {
                boolean has_inputs = FALSE;
                /* Subblock INPUTs */
                for (ipin = 0; ipin < subblock_lut_size; ++ipin) {
                    if (subblock_inf[iblk][isub].inputs[ipin] != OPEN) {
                        has_inputs = TRUE;
                        sblk_pin_to_tnode[iblk][isub][ipin] = curr_tnode++;
                    } else { /* Don't forget the unused subblock's pins */
                        sblk_pin_to_tnode[iblk][isub][ipin] = OPEN;
                    }
                }
                /* Subblock OUTPUT  */
                /* If the subblock opin is unused,the subblock is empty and we *
                 * shoudn't count it.                                          */
                if (num_uses_of_sblk_opin[iblk][isub] != 0) {
                    sblk_pin_to_tnode[iblk][isub][out_pin] = curr_tnode;
                    if (has_inputs) { /* Regular sblk */
                        ++curr_tnode;
                    } else { /* Constant generator. Make room for dummy input */
                        curr_tnode += 2;
                    }
                } else { /* the subblock was unused! set it OPEN */
                    sblk_pin_to_tnode[iblk][isub][out_pin] = OPEN;
                }

                /* Subblock CLOCK */
                if (subblock_inf[iblk][isub].clock != OPEN) {
                   /* If this is a sequential blocks, we have two more pins: *
                    * #1: the clock input(connects to the subblock output   *
                    * node) and #2: the sequential sink(which the subblock  *
                    * LUT inputs will connect to).                          */
                    sblk_pin_to_tnode[iblk][isub][clk_pin] = curr_tnode;
                    curr_tnode += 2;
                } else {
                    sblk_pin_to_tnode[iblk][isub][clk_pin] = OPEN;
                }
            } /* end of for (isub = 0; isub < num_subblocks; ++isub) */
        } else { /* distinguish INPAD_TYPE and OUTPAD_TYPE */
            block_pin_to_tnode[iblk][0] = curr_tnode;     /* Pad input  */
            block_pin_to_tnode[iblk][1] = curr_tnode + 1; /* Pad output */
            curr_tnode += 2;

            for (ipin = 2; ipin < pins_per_clb; ++ipin) {
                block_pin_to_tnode[iblk][ipin] = OPEN;
            }
            /* There was no subblocks in IO_TYPE pads. */
            sblk_pin_to_tnode[iblk] = NULL;  /* No subblock pins */
        }
    }  /* End for all blocks */

    *sblk_pin_to_tnode_ptr = sblk_pin_to_tnode;
    *block_pin_to_tnode_ptr = block_pin_to_tnode;

    return curr_tnode;
} /* end of static int alloc_and_load_pin_mappings() */


static void free_pin_mappings(int** block_pin_to_tnode,
                              int*** sblk_pin_to_tnode,
                              int* num_subblocks_per_block)
{
    /* Frees the arrays that map from pins to vertexes coordinates. */
    free_matrix(block_pin_to_tnode,
                0,
                num_blocks - 1,
                0,
                sizeof(int));

    int iblk = -1;
    for (iblk = 0; iblk < num_blocks; ++iblk) {
        if (blocks[iblk].type == CLB_TYPE) {
            free_matrix(sblk_pin_to_tnode[iblk],
                        0,
                        num_subblocks_per_block[iblk] - 1,
                        0,
                        sizeof(int));
        }
    }

    free(sblk_pin_to_tnode);
}


/* Create and Initialize Timing-Analyze-Graph for circuit netlist, so using *
 * num_blocks, not num_clbs in arch file.                                   */
static void alloc_and_load_fanout_counts(int*** num_uses_of_clb_ipin_ptr,
                                         int*** num_uses_of_sblk_opin_ptr,
                                         subblock_data_t subblock_data)
{
    /* number of subblocks in each blocks. for io pads, it was 0. For a CLB_TYPE, it *
     * was from 1 to max_subblocks_per_block.                                  *
     * int num_subblocks_per_block[0...num_blocks-1].                         */
    int* num_subblocks_per_block = subblock_data.num_subblocks_per_block;
    /* subblock_t subblock_inf[0...num_blocks-1][0...num_subblocks[iblk]-1] */
    subblock_t** subblock_inf = subblock_data.subblock_inf;
    int subblock_lut_size = subblock_data.subblock_lut_size; /* lut-k */

    /* Allocates and loads two arrays that say how many points each clb input *
     * pin and each subblock output fan out to.                               */
    int** num_uses_of_clb_ipin = (int**)my_malloc(num_blocks * sizeof(int*));
    int** num_uses_of_sblk_opin = (int**)my_malloc(num_blocks * sizeof(int*));

    int iblk = -1;
    for (iblk = 0; iblk < num_blocks; ++iblk) {
        if (blocks[iblk].type != CLB_TYPE) {
            num_uses_of_clb_ipin[iblk] = NULL;
            num_uses_of_sblk_opin[iblk] = NULL;
        } else { /* CLB_TYPE */
            num_uses_of_clb_ipin[iblk] = (int*)my_calloc(pins_per_clb,
                                                         sizeof(int));
            num_uses_of_sblk_opin[iblk] = (int*)my_calloc(num_subblocks_per_block[iblk],
                                                          sizeof(int));
            /* FIXME: allocate the 2 arrarys for each plb in circuit netlist. */
            load_one_clb_fanout_count(subblock_lut_size,
                                      subblock_inf[iblk],
                                      num_subblocks_per_block[iblk], /* num_subblocks_in_block */
                                      num_uses_of_clb_ipin[iblk],
                                      num_uses_of_sblk_opin[iblk],
                                      iblk);
        }  /* End if CLB_TYPE */
    }  /* End for all blocks */

    *num_uses_of_clb_ipin_ptr = num_uses_of_clb_ipin;
    *num_uses_of_sblk_opin_ptr = num_uses_of_sblk_opin;
} /* end of static void alloc_and_load_fanout_counts() */

static void free_fanout_counts(int** num_uses_of_clb_ipin,
                               int** num_uses_of_sblk_opin)
{
    /* Frees the fanout count arrays. */
    int iblk = -1;
    for (iblk = 0; iblk < num_blocks; iblk++) {
        if (blocks[iblk].type == CLB_TYPE) {
            free(num_uses_of_clb_ipin[iblk]);
            free(num_uses_of_sblk_opin[iblk]);
        }
    }

    free(num_uses_of_clb_ipin);
    free(num_uses_of_sblk_opin);
}

/* FIXME: allocate and load timing_graph node(for clb, inpad, outpad)and edges for Timing*/
static void alloc_and_load_tnodes_and_net_mapping(int** num_uses_of_clb_ipin,
                                                  int** num_uses_of_sblk_opin,
                                                  int** block_pin_to_tnode,
                                                  int*** sblk_pin_to_tnode,
                                                  subblock_data_t subblock_data,
                                                  timing_info_t timing_inf)
{
    /* Does the actual allocation and building of the timing graph. */
    vertexes = (vertex_t*)my_malloc(num_of_vertexs * sizeof(vertex_t));
    tnode_descript = (vertex_descript*)my_malloc(num_of_vertexs *
                                                 sizeof(vertex_descript));
    /* Attention, tedge in Timing-Analyze-Graph was depend on point-to-point *
     * connection, not net.                                                  */
    driver_node_index_of_net = (int*)my_malloc(num_nets * sizeof(int)); /* <net, vertexes> */

    /* ? */
    int* next_clb_ipin_edge = (int*)my_malloc(pins_per_clb * sizeof(int));
    int* next_sblk_opin_edge = (int*)my_malloc(subblock_data.max_subblocks_per_block
                                           * sizeof(int));

    /* [0..num_of_blocks-1][0..num_subbloks_per_blk[iblk]-1] */
    subblock_t** subblock_inf = subblock_data.subblock_inf;
    const int subblock_lut_size = subblock_data.subblock_lut_size;
    int* num_subblocks_per_block = subblock_data.num_subblocks_per_block;

    int iblk = -1;
    for (iblk = 0; iblk < num_blocks; ++iblk) {
        switch (blocks[iblk].type) { /* netlist blocks type: CLB_TYPE, INPAD_TYPE, OUTPAD_TYPE */
            case CLB_TYPE:
                build_clb_tnodes(iblk,
                                 num_uses_of_clb_ipin[iblk],
                                 block_pin_to_tnode,
                                 sblk_pin_to_tnode[iblk],
                                 subblock_lut_size,
                                 num_subblocks_per_block[iblk],
                                 subblock_inf[iblk],
                                 timing_inf.T_clb_ipin_to_sblk_ipin,
                                 next_clb_ipin_edge);

                build_subblock_tnodes(num_uses_of_sblk_opin[iblk],
                                      block_pin_to_tnode[iblk],
                                      sblk_pin_to_tnode[iblk],
                                      subblock_lut_size,
                                      num_subblocks_per_block[iblk],
                                      subblock_inf[iblk],
                                      timing_inf.T_sblk_opin_to_sblk_ipin,
                                      timing_inf.T_sblk_opin_to_clb_opin,
                                      timing_inf.T_subblock,
                                      next_sblk_opin_edge,
                                      iblk);
                break;

            case INPAD_TYPE:
                build_ipad_tnodes(iblk,
                                  block_pin_to_tnode,
                                  timing_inf.T_ipad,
                                  num_subblocks_per_block,
                                  subblock_inf);
                break;

            case OUTPAD_TYPE:
                build_opad_tnodes(block_pin_to_tnode[iblk],
                                  timing_inf.T_opad,
                                  iblk);
                break;

            default:
                printf("Error in alloc_and_load_tnodes_and_net_mapping:\n"
                       "\tUnexpected blocks type (%d) for blocks %d (%s).\n",
                       blocks[iblk].type,
                       iblk,
                       blocks[iblk].name);
                exit(1);
        } /* end of switch (blocks[iblk].type) */
    } /* end of for (iblk = 0; iblk < num_blocks; ++iblk) */

    free(next_clb_ipin_edge);
    free(next_sblk_opin_edge);
} /* end of static void alloc_and_load_tnodes_and_net_mapping() */


/* This routine builds the tnodes corresponding to the CLB_TYPE pins and all  *
 * subblocks of this BLOCK, and properly hooks them up to the rest of the*
 * graph. Note that only the sblk_pin_to_tnode, etc. element for this    *
 * blocks is passed in.                                                   */
static void build_clb_tnodes(int   iblk,
                             int*  num_uses_of_clb_ipin,
                             int** block_pin_to_tnode,
                             int** sub_pin_to_tnode,
                             int   subblock_lut_size,
                             int   subblks_in_block,
                             subblock_t* sub_inf,
                             double T_clb_ipin_to_sblk_ipin,
                             int*   next_clb_ipin_edge)
{
    int num_edges = 0;
    const int clk_pin = subblock_lut_size + 1;
    /* Start by allocating the edge arrays, and for opins, loading them. */
    int ipin = -1;
    int from_node = -1;
    for (ipin = 0; ipin < pins_per_clb; ++ipin) {
        from_node = block_pin_to_tnode[iblk][ipin];
        if (from_node != OPEN) { /* Pin is used -> put in graph */
            if (is_opin(ipin)) { /* CLB_TYPE output pin */
                build_block_output_tnode(from_node,
                                         iblk,
                                         ipin,
                                         block_pin_to_tnode);
                tnode_descript[from_node].type = CLB_OPIN;
            } else { /* CLB_TYPE input_pin */
                next_clb_ipin_edge[ipin] = 0; /* Reset */
                /* When the number of this CLB_TYPE's input pin connect to, the edges *
                 * it should create and connect.   */
                num_edges = num_uses_of_clb_ipin[ipin];
                vertexes[from_node].num_edges = num_edges;
                vertexes[from_node].out_edges = (edge_t*)my_chunk_malloc(num_edges * sizeof(edge_t),
                                                                 &tedge_ch_list_head,
                                                                 &tedge_ch_bytes_avail,
                                                                 &tedge_ch_next_avail);
                tnode_descript[from_node].type = CLB_IPIN;
            } /* CLB_IPIN */

            tnode_descript[from_node].ipin = ipin;
            tnode_descript[from_node].isubblk = OPEN;
            tnode_descript[from_node].iblk = iblk;
        } /* end of if (from_node != OPEN) */
    } /* end of for(ipin = 0; ipin < pins_per_clb; ++ipin) */

    /* Then load the edge arrays for the CLB_TYPE input_pins to SUBBLOCK input_pins.
     Do this by looking at where the SUBBLOCK input and clock pins are driven
     from.               */
    int  from_pin = -1;
    int  to_node = -1;
    edge_t*  tedge = NULL;
    int  isub = -1;
    int  iedge = 0;
    for (isub = 0; isub < subblks_in_block; ++isub) {
        /* First deal with the input pins of subblock */
        for (ipin = 0; ipin < subblock_lut_size; ++ipin) {
            from_pin = sub_inf[isub].inputs[ipin];
            /* Not OPEN and comes from clb ipin? */
            if (from_pin != OPEN && from_pin < pins_per_clb) {
                from_node = block_pin_to_tnode[iblk][from_pin];
                to_node = sub_pin_to_tnode[isub][ipin];

                /* the out_edges were edges start from clb_input_pin *
                 * to sblk_input_pin.                                */
                tedge = vertexes[from_node].out_edges;
                iedge = next_clb_ipin_edge[from_pin]++; /* next_clb_ipin_out_edge_index */
                tedge[iedge].to_node = to_node;
                /* T_CLB_ipin_to_sblk_ipin */
                tedge[iedge].Tdel = T_clb_ipin_to_sblk_ipin;
            }
        }

        /* Attention: why did it not deal with subblock's output pins? */
        from_pin = sub_inf[isub].clock;
        if (from_pin != OPEN && from_pin < pins_per_clb) {
            from_node = block_pin_to_tnode[iblk][from_pin];
            to_node = sub_pin_to_tnode[isub][clk_pin]; /* Feeds seq. output */

            tedge = vertexes[from_node].out_edges;
            iedge = next_clb_ipin_edge[from_pin]++; /* ? */
            tedge[iedge].to_node = to_node;
            /* For the earliest possible clock I want this Tdel to be zero,     *
             * so it arrives at flip flops at T = 0. For later clocks or locally *
             * generated clocks that may accumulate Tdel(like the clocks in a   *
             * ripple counter), I might want to make this Tdel nonzero. Not worth*
             * bothering about now.  */
            tedge[iedge].Tdel = 0.0;
        }
    } /* end of for(isub = 0; isub < subblks_in_block; isub++) */
} /* end of static void build_clb_tnodes() */

static void build_block_output_tnode(int ivex,
                                     int iblk,
                                     int ipin,
                                     int** block_pin_to_tnode)
{
    /* Sets the number of edges and the edge array for an output pin from a *
     * blocks. This pin must be hooked to something -- i.e. not OPEN.       */
    int inet = blocks[iblk].nets[ipin]; /* Won't be OPEN, as ivex exists */
    assert(inet != OPEN);          /* Sanity check. */

    driver_node_index_of_net[inet] = ivex;
    int num_edges = net[inet].num_pins - 1;
    vertexes[ivex].num_edges = num_edges;
    vertexes[ivex].out_edges = (edge_t*)my_chunk_malloc(num_edges * sizeof(edge_t),
                                                       &tedge_ch_list_head,
                                                       &tedge_ch_bytes_avail,
                                                       &tedge_ch_next_avail);
    edge_t* tedge = vertexes[ivex].out_edges;
    int iedge = -1;
    for (iedge = 0; iedge < net[inet].num_pins-1; ++iedge) {
        int to_blk = net[inet].blocks[iedge + 1];

        int to_pin = -1;
        if (blocks[to_blk].type == CLB_TYPE) {
            to_pin = net[inet].blk_pin[iedge + 1];
        } else { /* OUTPAD_TYPE */
            to_pin = 0;
        }

        int to_node = block_pin_to_tnode[to_blk][to_pin];
        tedge[iedge].to_node = to_node;
        /* Set Tdel from net Tdels with a later call */
    }
} /* end of static void build_block_output_tnode() */

/* This routine builds the tnodes of the subblock pins within one CLB_TYPE. Note *
 * that only the block_pin_to_tnode, etc. data for *this* blocks are passed  *
 * in.                                                                      */
static void build_subblock_tnodes(int* num_uses_of_sblk_opin,
                                  int* blk_pin_to_tnode,
                                  int** sub_pin_to_tnode, /* subblock pin index to vertexes */
                                  int  subblock_lut_size,
                                  int  subblks_in_block,
                                  subblock_t* sub_inf,
                                  double T_sblk_opin_to_sblk_ipin,
                                  double T_sblk_opin_to_clb_opin,
                                  T_subblock_t* T_subblock,
                                  int* next_sblk_opin_edge,
                                  int iblk)
{
    int iedge, num_edges;
    const int out_pin = subblock_lut_size;
    const int clk_pin = subblock_lut_size + 1;

    /* Allocate memory for SUBBLOCK OUTPUT pins first. */
    int isub = -1;
    int from_node = -1;
    int ipin = -1;
    for (isub = 0; isub < subblks_in_block; ++isub) {
        from_node = sub_pin_to_tnode[isub][out_pin]; /* output node */
        if (from_node != OPEN) { /* Output is used -> timing node exists. */
            next_sblk_opin_edge[isub] = 0; /* Reset */
            num_edges = num_uses_of_sblk_opin[isub];
            vertexes[from_node].num_edges = num_edges;
            vertexes[from_node].out_edges = (edge_t*)my_chunk_malloc(num_edges * sizeof(edge_t),
                                                              &tedge_ch_list_head,
                                                              &tedge_ch_bytes_avail,
                                                              &tedge_ch_next_avail);
            tnode_descript[from_node].type = SUBBLK_OPIN;

            tnode_descript[from_node].ipin = out_pin;
            tnode_descript[from_node].isubblk = isub;
            tnode_descript[from_node].iblk = iblk;
        } /* end of if(from_node != OPEN) */
    } /* end of for(isub = 0; isub < subblks_in_block; ++isub) */

    int from_pin = -1;
    int to_pin = -1;
    int to_node = -1;
    edge_t* tedge = NULL;
    /* Load the SUBBLOCK INPUT pins edge arrays. */
    for (isub = 0; isub < subblks_in_block; ++isub) {
        for (ipin = 0; ipin < subblock_lut_size; ++ipin) { /* sblk opin to sblk ipin */
            from_pin = sub_inf[isub].inputs[ipin];

            /* Not OPEN and comes from local subblock output? */
            if (from_pin >= pins_per_clb) {
                /* from_pin - pins_per_clb = subblock_index */
                from_node = sub_pin_to_tnode[from_pin - pins_per_clb][out_pin];
                /* from_pin is a subblk output_pin which driver this input_pin */
                to_node = sub_pin_to_tnode[isub][ipin];

                tedge = vertexes[from_node].out_edges;
                iedge = next_sblk_opin_edge[from_pin - pins_per_clb]++;
                tedge[iedge].to_node = to_node;
                tedge[iedge].Tdel = T_sblk_opin_to_sblk_ipin;
            } /* end of if(from_pin >= pins_per_clb) */
        } /* end of for(ipin = 0; ipin < subblock_lut_size; ++ipin) */

        from_pin = sub_inf[isub].clock;
        if (from_pin >= pins_per_clb) {
            /* this clock_pin was drived by other subblock's output_pin *
             * Attention: why did it not driver by clock_pin?           */
            from_node = sub_pin_to_tnode[from_pin - pins_per_clb][out_pin];
            /* from_node = sub_pin_to_tnode[from_pin - pins_per_clb][clk_pin] */

            to_node = sub_pin_to_tnode[isub][clk_pin]; /* Feeds seq. output */

            tedge = vertexes[from_node].out_edges;
            iedge = next_sblk_opin_edge[from_pin - pins_per_clb]++;
            tedge[iedge].to_node = to_node;
            /* NB: Could make sblk opin to clk Tdel parameter; not worth it right now. */
            tedge[iedge].Tdel = T_sblk_opin_to_sblk_ipin;
        }

        to_pin = sub_inf[isub].output;
        if (to_pin != OPEN) {  /* sblk opin goes to clb opin? */
            /* Check that CLB_TYPE pin connects to something ->     *
             * not just a mandatory BLE to CLB_TYPE opin connection */
            if (blocks[iblk].nets[to_pin] != OPEN) {
                to_node = blk_pin_to_tnode[to_pin];
                from_node = sub_pin_to_tnode[isub][out_pin];
                tedge = vertexes[from_node].out_edges;

                iedge = next_sblk_opin_edge[isub]++;

                tedge[iedge].to_node = to_node;
                tedge[iedge].Tdel = T_sblk_opin_to_clb_opin;
            }
        }
    } /* end of traverse all subblocks in this CLB_TYPE */

    /*==============   then deal with tnode_descript  ===================*/
    /* Now build the subblock input pins and, if the subblock is used in  *
     * sequential mode(i.e. is clocked), the two clock pin nodes.        */
    double  ipin_to_sink_delay = 0.0;
    boolean has_inputs = FALSE;
    for (isub = 0; isub < subblks_in_block; ++isub) {
        if (sub_pin_to_tnode[isub][out_pin] == OPEN) {  /* Empty, so skip */
            continue;
        }

        if (sub_inf[isub].clock == OPEN) { /* Combinational mode */
            to_node = sub_pin_to_tnode[isub][out_pin];
            ipin_to_sink_delay = T_subblock[isub].T_comb;
        } else { /* Sequential mode. Build two clock nodes. */
            /* First node is the clock input pin; it feeds the sequential output */
            from_node = sub_pin_to_tnode[isub][clk_pin];
            vertexes[from_node].num_edges = 1;
            vertexes[from_node].out_edges = (edge_t*)my_chunk_malloc(sizeof(edge_t),
                                                              &tedge_ch_list_head,
                                                              &tedge_ch_bytes_avail,
                                                              &tedge_ch_next_avail);
            tedge = vertexes[from_node].out_edges;
            tedge[0].to_node = sub_pin_to_tnode[isub][out_pin]; /* why? */
            tedge[0].Tdel = T_subblock[isub].T_seq_out;

            tnode_descript[from_node].type = FF_SOURCE;
            tnode_descript[from_node].ipin = OPEN;
            tnode_descript[from_node].isubblk = isub;
            tnode_descript[from_node].iblk = iblk;

            /* Now create the "sequential sink" -- i.e. the FF input node. */
            from_node++;
            vertexes[from_node].num_edges = 0;
            vertexes[from_node].out_edges = NULL;
            tnode_descript[from_node].type = FF_SINK;
            tnode_descript[from_node].ipin = OPEN;
            tnode_descript[from_node].isubblk = isub;
            tnode_descript[from_node].iblk = iblk;

            /* Subblock inputs connect to this node. */
            to_node = from_node;
            ipin_to_sink_delay = T_subblock[isub].T_seq_in;
        } /* end of else (in Sequential mode) */

        /* Build and hook up subblock inputs. */
        has_inputs = FALSE;
        for (ipin = 0; ipin < subblock_lut_size; ++ipin) {
            from_node = sub_pin_to_tnode[isub][ipin];
            if (from_node != OPEN) { /* vertexes exists -> pin is used */
                has_inputs = TRUE;
                vertexes[from_node].num_edges = 1;
                vertexes[from_node].out_edges = (edge_t*)my_chunk_malloc(sizeof(edge_t),
                                                                  &tedge_ch_list_head,
                                                                  &tedge_ch_bytes_avail,
                                                                  &tedge_ch_next_avail);
                tedge = vertexes[from_node].out_edges;
                tedge[0].to_node = to_node;
                tedge[0].Tdel = ipin_to_sink_delay;
                tnode_descript[from_node].type = SUBBLK_IPIN;
                tnode_descript[from_node].ipin = ipin;
                tnode_descript[from_node].isubblk = isub;
                tnode_descript[from_node].iblk = iblk;
            }
        } /* end of for(ipin = 0; ipin < subblock_lut_size; ++ipin) */


        if (FALSE == has_inputs) { /* Constant generator. Give fake input. FIXME */
            from_node = sub_pin_to_tnode[isub][out_pin] + 1; /* ? */
            vertexes[from_node].num_edges = 1;
            vertexes[from_node].out_edges = (edge_t*)my_chunk_malloc(sizeof(edge_t),
                                                              &tedge_ch_list_head,
                                                              &tedge_ch_bytes_avail,
                                                              &tedge_ch_next_avail);
            tedge = vertexes[from_node].out_edges;
            tedge[0].to_node = to_node;
            /* Want constants generated early so they never affect the critical path. */
            tedge[0].Tdel = T_CONSTANT_GENERATOR;
            tnode_descript[from_node].type = CONSTANT_GEN_SOURCE;
            tnode_descript[from_node].ipin = OPEN;
            tnode_descript[from_node].isubblk = isub;
            tnode_descript[from_node].iblk = iblk;
        } /* end of if(has_inputs == FALSE) */
    }  /* End for each subblock */
} /* end of static void build_subblock_tnode() */

/* Builds the two tnodes corresponding to an input pad, and hooks them into *
 * the timing graph.                                                        */
static void build_ipad_tnodes(int iblk,
                              int** block_pin_to_tnode,
                              double T_ipad,
                              int* num_subblocks_per_block,
                              subblock_t** subblock_inf)
{
    /* First node: input node to the pad -> nothing comes into this. Second *
     * node is the output node of the pad, that has edges to CLB_TYPE ipins.     */
    int from_node = block_pin_to_tnode[iblk][0]; /* from_node */
    int to_node = block_pin_to_tnode[iblk][1];
    vertexes[from_node].num_edges = 1;
    vertexes[from_node].out_edges = (edge_t*)my_chunk_malloc(sizeof(edge_t),
                                                     &tedge_ch_list_head,
                                                     &tedge_ch_bytes_avail,
                                                     &tedge_ch_next_avail);
    edge_t* tedge = vertexes[from_node].out_edges;
    tedge[0].to_node = to_node;

    /* By definition, global clocks from pads arrive at T = 0. The earliest any *
     * clock can arrive at any flip flop is T = 0, and the fastest global clock *
     * from a pad should have zero Tdel on its edges so it does get to the     *
     * flip-flop clock pin at T = 0.                                            */
    if (is_global_clock(iblk,
                        num_subblocks_per_block,
                        subblock_inf)) {
        tedge[0].Tdel = 0.0;
    } else {
        tedge[0].Tdel = T_ipad;
    }

    tnode_descript[from_node].type = INPAD_SOURCE;
    tnode_descript[from_node].ipin = OPEN;
    tnode_descript[from_node].isubblk = OPEN;
    tnode_descript[from_node].iblk = iblk;

    /* Now do pad output. */
    build_block_output_tnode(to_node,
                             iblk,
                             0,
                             block_pin_to_tnode);
    tnode_descript[to_node].type = INPAD_OPIN;
    tnode_descript[to_node].ipin = 0;
    tnode_descript[to_node].isubblk = OPEN;
    tnode_descript[to_node].iblk = iblk;
} /* end of static void build_ipad_tnodes() */


static boolean is_global_clock(int iblk,
                               int* num_subblocks_per_block,
                               subblock_t** subblock_inf)
{
    /* Returns TRUE if the net driven by this blocks (which must be an INPAD_TYPE) is  *
     * (1) a global signal, and (2) used as a clock input to at least one blocks. */
    int inet = blocks[iblk].nets[0];
    if (!is_global[inet]) {
        return FALSE;
    }

    int ipin = -1;
    for (ipin = 1; ipin < net[inet].num_pins; ipin++) {
        int to_blk = net[inet].blocks[ipin];
        int to_pin = net[inet].blk_pin[ipin];

        int isub = -1;
        for (isub = 0; isub < num_subblocks_per_block[to_blk]; isub++) {
            if (subblock_inf[to_blk][isub].clock == to_pin) {
                return TRUE;
            }
        }
    }

    return FALSE;
}


/* Builds the two tnodes in an output pad and connects them up. */
/* First node: input node to the pad -> will be driven some net. Second *
 * node is the output node of the pad, which connects to nothing.*/
static void build_opad_tnodes(int* blk_pin_to_tnode,
                              double T_opad,
                              int iblk)
{
    int from_node = blk_pin_to_tnode[0];
    int to_node = blk_pin_to_tnode[1];
    vertexes[from_node].num_edges = 1;
    vertexes[from_node].out_edges = (edge_t*)my_chunk_malloc(sizeof(edge_t),
                                                          &tedge_ch_list_head,
                                                          &tedge_ch_bytes_avail,
                                                          &tedge_ch_next_avail);
    edge_t* tedge = vertexes[from_node].out_edges;
    tedge[0].to_node = to_node;
    tedge[0].Tdel = T_opad;

    tnode_descript[from_node].type = OUTPAD_IPIN;
    tnode_descript[from_node].ipin = 0;
    tnode_descript[from_node].isubblk = OPEN;
    tnode_descript[from_node].iblk = iblk;

    vertexes[to_node].num_edges = 0;
    vertexes[to_node].out_edges = NULL;
    tnode_descript[to_node].type = OUTPAD_SINK;
    tnode_descript[to_node].ipin = OPEN;
    tnode_descript[to_node].isubblk = OPEN;
    tnode_descript[to_node].iblk = iblk;
} /* end of static void build_opad_tnodes() */

/* Initialize all tedges's Tdel in Timing_Analyze_Graph using double** net_delay. */
/* Sets the Tdels of the inter-CLB_TYPE nets to the values specified by (double**)     *
 * net_delay[0..num_nets-1][1..num_pins-1]. These net Tdels should have been      *
 * allocated and loaded with the net_delay routines. This routine marks the corres-*
 * ponding edges in the timing graph with the proper Tdel.                        */
void load_timing_graph_net_delays(double** net_delay) /* FIXME */
{
    int inet = -1;
    for (inet = 0; inet < num_nets; ++inet) {
        /* this net's driver node(that is a driver pin) */
        const int from_node = driver_node_index_of_net[inet];
        edge_t* tedge = vertexes[from_node].out_edges;

        /* Note that the edges of a vertexes corresponding to a CLB_TYPE or INPAD_TYPE opin must*
         * be in the same order as the pins of the net driven by the vertexes.        */
        int ipin = 0;
        for (ipin = 1; ipin < net[inet].num_pins; ++ipin) {
            tedge[ipin-1].Tdel = net_delay[inet][ipin]; /* tedge's Tdel*/
        }
    }
} /* end of void load_timing_graph_net_delays(double** net_delay) */


/* Allocates the net_slack structure. Chunk allocated to save space. */
/* double net_slack[0..num_nets-1][1..num_pins-1], and its value     *
 * depended on double net_delay[0...num_nets-1][1...num_pins-1].     */
double** alloc_net_slack(void)
{
    double** net_slack = (double**)my_malloc(num_nets * sizeof(double*));

    int inet = -1;
    for (inet = 0; inet < num_nets; ++inet) {
        double* tmp_ptr = (double*)my_chunk_malloc((net[inet].num_pins-1) * sizeof(double),
                                                   &tedge_ch_list_head,
                                                   &tedge_ch_bytes_avail,
                                                   &tedge_ch_next_avail);
        net_slack[inet] = tmp_ptr - 1; /* [1..num_pins-1] */
    }

    return net_slack;
} /* end of static double** alloc_net_slack() */


void free_timing_graph(double** net_slack)
{
    /* Frees the timing graph data. */
    if (tedge_ch_list_head == NULL) {
        printf("Error in free_timing_graph: No timing graph to free.\n");
        exit(1);
    }

    free_chunk_memory(tedge_ch_list_head);
    free(vertexes);
    free(tnode_descript);
    free(driver_node_index_of_net);
    free_ivec_vector(tnodes_at_level, 0, num_tnode_levels - 1);
    free(net_slack);
    tedge_ch_list_head = NULL;
    tedge_ch_bytes_avail = 0;
    tedge_ch_next_avail = NULL;
    vertexes = NULL;
    tnode_descript = NULL;
    num_of_vertexs = 0;
    driver_node_index_of_net = NULL;
    tnodes_at_level = NULL;
    num_tnode_levels = 0;
}

void print_net_slack(char* fname, double** net_slack)
{
    /* Prints the net slacks into a file.                                     */
    int inet, ipin;
    FILE* fp;
    fp = my_fopen(fname, "w", 0);
    fprintf(fp, "Net #\tSlacks\n\n");

    for (inet = 0; inet < num_nets; inet++) {
        fprintf(fp, "%5d", inet);

        for (ipin = 1; ipin < net[inet].num_pins; ipin++) {
            fprintf(fp, "\t%g", net_slack[inet][ipin]);
        }

        fprintf(fp, "\n");
    }
}

void print_timing_graph(char* fname)
{
    /* Prints the timing graph into a file.           */
    edge_t* tedge;
    vertex_type_t itype;
    char* tnode_type_names[] = {"INPAD_SOURCE",
                                "INPAD_OPIN",
                                "OUTPAD_IPIN",
                                "OUTPAD_SINK",
                                "CLB_IPIN",
                                "CLB_OPIN",
                                "SUBBLK_IPIN",
                                "SUBBLK_OPIN",
                                "FF_SINK",
                                "FF_SOURCE",
                                "CONSTANT_GEN_SOURCE"};
    FILE* fp = my_fopen(fname, "w", 0);
    fprintf(fp, "num_of_vertexs: %d\n", num_of_vertexs);
    fprintf(fp, "Node #\tType\t\tipin\tisubblk\tiblk\t# edges\t"
            "Edges (to_node, Tdel)\n\n");

    int ivex, iedge;
    for (ivex = 0; ivex < num_of_vertexs; ivex++) {
        fprintf(fp, "%d\t", ivex);
        itype = tnode_descript[ivex].type;
        fprintf(fp, "%-15.15s\t", tnode_type_names[itype]);
        fprintf(fp, "%d\t%d\t%d\t", tnode_descript[ivex].ipin,
                tnode_descript[ivex].isubblk, tnode_descript[ivex].iblk);
        fprintf(fp, "%d\t", vertexes[ivex].num_edges);
        tedge = vertexes[ivex].out_edges;

        for (iedge = 0; iedge < vertexes[ivex].num_edges; iedge++) {
            fprintf(fp, "\t(%4d,%7.3g)", tedge[iedge].to_node, tedge[iedge].Tdel);
        }

        fprintf(fp, "\n");
    }
    fprintf(fp, "\n\nnum_tnode_levels: %d\n", num_tnode_levels);

    int ilevel, i;
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
        fprintf(fp, "%4d\t%6d\n", i, driver_node_index_of_net[i]);
    }

    fprintf(fp, "\n\nNode #\t\tT_arr\t\tT_req\n\n");

    for (ivex = 0; ivex < num_of_vertexs; ivex++)
        fprintf(fp, "%d\t%12g\t%12g\n", ivex, vertexes[ivex].arr_time,
                vertexes[ivex].req_time);

    fclose(fp);
}

/* FIXME: calculate arr_time and req_time for all tnodes in Timing_Analyze_Graph, *
 * and then calculate all edges slack and find all cirtical path in graph.        *
 * Determines the slack of every source-sink pair of blocks pins in the circuit.   *
 * The timing graph must have already been built. target_cycle_time is the target *
 * Tdel for the circuit -- if 0, the target_cycle_time is set to the crit. path  *
 * found in the timing graph. This routine loads net_slack, and returns the current *
 * crit. path Tdel. If a source-sink pair in timing-graph slack was 0, it was a  *
 * critical_path in timing-graph.                                                 */
double calc_all_vertexs_arr_req_time(double target_cycle_time)
{
    /* Reset all arrival times to minus infinity. Can't just set to zero or the   *
     * constant propagation (constant generators work at -ve infinity) won't work */
    int from_node = -1;
    for (from_node = 0; from_node < num_of_vertexs; ++from_node) {
        vertexes[from_node].arr_time = T_CONSTANT_GENERATOR; /* -1000 */
    }

    /* Attention: All primary-inputs arr_time set 0.0 */
    int i = -1;
    int num_at_level = tnodes_at_level[0].nelem;
    for (i = 0; i < num_at_level; ++i) {
        from_node = tnodes_at_level[0].list[i];
        vertexes[from_node].arr_time = 0.0;
    }

    /* arr_time: actual_arrival_time, Tdel: edge_delay_time, T_cycle: cycle_time,*
     * req_time: required_arrival_time                                          */
    double arr_time = 0.0;
    double req_time = 0.0;
    double Tdel = 0.0; /* slack(i,j) = req_time(j)-arr_time(i)-Tdel(i,j) */

    int ilevel = -1;
    int num_edges = -1;
    int iedge = -1;
    int to_node = -1;
    edge_t* conn_edges = NULL;
    /* Compute all arrival times with a breadth-first analysis from inputs to *
     * outputs. Also compute critical path (T_crit).                         */
    double T_crit = 0.0; /* max-Tdel in critical-path */
    /* use breath-first-search(from level 0 to last level)to calculate the arr_time, T_c*/
    for (ilevel = 0; ilevel < num_tnode_levels; ++ilevel) {
        num_at_level = tnodes_at_level[ilevel].nelem;
        for (i = 0; i < num_at_level; ++i) {
            from_node = tnodes_at_level[ilevel].list[i];
            arr_time = vertexes[from_node].arr_time;
            /* find the max_arrival_time(crit_time) at tnodes_at_level[ilevel]*/
            T_crit = max(T_crit, arr_time);

            conn_edges = vertexes[from_node].out_edges;
            num_edges = vertexes[from_node].num_edges;
            for (iedge = 0; iedge < num_edges; ++iedge) {
                to_node = conn_edges[iedge].to_node;
                Tdel = conn_edges[iedge].Tdel; /* edge_delay_time */
                /* calculate arr_time: actual arrived time */
                vertexes[to_node].arr_time = max(vertexes[to_node].arr_time,
                                                 arr_time + Tdel);
            }
        }
    } /* end of for(ilevel = 0; ilevel < num_tnode_levels; ++ilevel) */

    double T_cycle = 0.0; /* clock_time: T = 1 / fmax */
    if (target_cycle_time > 0.0) {
        /* User-specified target T_cycle, i.e. if user set fmax = 500MHz, *
         * the target_cycle_time = 2ns.                                   */
        T_cycle = target_cycle_time;
    } else {
        /* In this circumstance, the default fmax was the max-Tdel. */
        T_cycle = T_crit;
    }

    /* Compute the all tnodes'req_time(required_arrival_times) with a backward *
     * breadth-first analysis from sinks(output pads, etc.) to primary inputs  *
     * Req_time calculate start from the last level, that were primary outputs. */
    /* from last level to 0 level */
    for (ilevel = num_tnode_levels - 1; ilevel >= 0; --ilevel) {
        num_at_level = tnodes_at_level[ilevel].nelem;
        for (i = 0; i < num_at_level; ++i) {
            from_node = tnodes_at_level[ilevel].list[i];
            num_edges = vertexes[from_node].num_edges;

            if (num_edges == 0) { /* sink */
                vertexes[from_node].req_time = T_cycle;
            } else {
                conn_edges = vertexes[from_node].out_edges;
                Tdel = conn_edges[0].Tdel;

                to_node = conn_edges[0].to_node;
                req_time = vertexes[to_node].req_time - Tdel;
                for (iedge = 1; iedge < num_edges; ++iedge) {
                    to_node = conn_edges[iedge].to_node;
                    Tdel = conn_edges[iedge].Tdel;
                    /* calculate T-req: required arrived time */
                    req_time = min(req_time,
                                   vertexes[to_node].req_time - Tdel);
                }
                vertexes[from_node].req_time = req_time;
            }
        }
    } /* end of for (ilevel = num_tnode_levels - 1; ilevel >= 0; ilevel--) */

    return T_crit;
} /* end of double calc_all_vertexs_arr_req_time(double** net_slack, double target_cycle_time) */

/* Puts the slack of each source-sink pair of blocks pins in net_slack.  *
 * For a directed-edge in Timing_analyze_graph, <i,j>, its slack<i,j> = *
 * req_time(j) - arr_time(i) - Tdel(i,j).                               */
void compute_net_slacks(double** net_slack)
{
    int inet = -1;
    for (inet = 0; inet < num_nets; ++inet) {
        /* from_node is the driver_node of current net */
        int from_node = driver_node_index_of_net[inet];
        double arr_time = vertexes[from_node].arr_time;

        int num_edges = vertexes[from_node].num_edges;
        edge_t* tedge = vertexes[from_node].out_edges;
        int iedge = -1;
        for (iedge = 0; iedge < num_edges; ++iedge) {
            int to_node = tedge[iedge].to_node;
            double Tdel = tedge[iedge].Tdel;
            double req_time = vertexes[to_node].req_time;
            /* TODO: Why set net_slack[inet][iedge+1], not net_slack[inet][iedge]? */
            net_slack[inet][iedge + 1] = req_time - arr_time - Tdel;
        }
    }
} /* end of static void compute_net_slacks(double**  net_slack) */

void print_critical_path(char* fname)
{
    /* Prints out the critical path to a file.  */
    FILE* fp = my_fopen(fname, "w", 0);

    int iblk = -1;
    int inet = -1;
    /* Attention, allocate_and_load_critical_path() only display a critical path */
    t_linked_int* critical_path_head = allocate_and_load_critical_path();

    int tnodes_on_crit_path = 0;
    int non_global_nets_on_crit_path = 0;
    int global_nets_on_crit_path = 0;
    double total_net_delay = 0.0;
    double total_logic_delay = 0.0;
    t_linked_int* critical_path_node = critical_path_head;
    while (critical_path_node != NULL) {
        double delay = print_critical_path_node(fp, critical_path_node);
        int    ivex = critical_path_node->data;
        vertex_type_t type = tnode_descript[ivex].type;
        ++tnodes_on_crit_path;

        if (type == INPAD_OPIN || type == CLB_OPIN) {
            get_tnode_block_and_output_net(ivex, &iblk, &inet);

            if (!is_global[inet]) {
                ++non_global_nets_on_crit_path;
            } else {
                ++global_nets_on_crit_path;
            }

            total_net_delay += delay;
        } else {
            total_logic_delay += delay;
        }

        critical_path_node = critical_path_node->next;
    } /* end of while (critical_path_node != NULL) */

    fprintf(fp, "\nTnodes on crit. path: %d  Non-global nets on crit. path: %d.\n",
            tnodes_on_crit_path,
            non_global_nets_on_crit_path);

    fprintf(fp, "Global nets on crit. path: %d.\n",
            global_nets_on_crit_path);

    fprintf(fp, "Total logic delay: %g (s)  Total net delay: %g (s)\n",
            total_logic_delay,
            total_net_delay);

    printf("Nets on crit. path: %d normal, %d global.\n",
           non_global_nets_on_crit_path,
           global_nets_on_crit_path);

    printf("Total logic delay: %g (s)  Total net delay: %g (s)\n",
           total_logic_delay,
           total_net_delay);
    fclose(fp);

    free_int_list(&critical_path_head);
} /* end of void print_critical_path(char* fname) */

t_linked_int* allocate_and_load_critical_path(void)
{
    /* Finds the critical path and puts a list of the tnodes on the critical    *
     * path in a linked list, from the path SOURCE to the path SINK.            */
    int ivex, iedge, to_node;

    /* found out min_slack in level 0 */
    int i = -1;
    double slack = 0.0;
    int crit_node = OPEN;   /* Stops compiler warnings. */
    /* For VPR4.3, slack of all edges in critical path was 0. */
    double min_slack = HUGE_FLOAT;
    const int num_vertexs_at_level_0 = tnodes_at_level[0].nelem;
    for (i = 0; i < num_vertexs_at_level_0; ++i) { /* Path must start at SOURCE (no inputs) */
        ivex = tnodes_at_level[0].list[i];
        slack = vertexes[ivex].req_time - vertexes[ivex].arr_time;
        if (slack < min_slack) {
            crit_node = ivex;
            min_slack = slack;
        }
    }
    int num_crit_vertexs_in_level_0 = 0;
    for (i = 0; i < num_vertexs_at_level_0; ++i) {
        ivex = tnodes_at_level[0].list[i];
        slack = vertexes[ivex].req_time - vertexes[ivex].arr_time;
        if (slack == min_slack) {
            ++num_crit_vertexs_in_level_0;
        }
    }
    if (num_crit_vertexs_in_level_0 > 1) {
        printf("Attention! There are %d critical vertexs in Level 0.\n",
               num_crit_vertexs_in_level_0);
    }

    t_linked_int* critical_path_head =
                         (t_linked_int*)my_malloc(sizeof(t_linked_int));
    critical_path_head->data = crit_node;
    int num_edges = vertexes[crit_node].num_edges;

    t_linked_int* curr_crit_node = NULL;
    edge_t* tedge = NULL;
    t_linked_int* prev_crit_node = critical_path_head;
    while (num_edges != 0) {   /* Path will end at SINK (no fanout) */
        curr_crit_node = (t_linked_int*)my_malloc(sizeof(t_linked_int));
        prev_crit_node->next = curr_crit_node;
        tedge = vertexes[crit_node].out_edges;
        min_slack = HUGE_FLOAT;

        for (iedge = 0; iedge < num_edges; ++iedge) {
            to_node = tedge[iedge].to_node;
            slack = vertexes[to_node].req_time - vertexes[to_node].arr_time;

            if (slack < min_slack) {
                crit_node = to_node;
                min_slack = slack;
            }
        }

        curr_crit_node->data = crit_node;
        prev_crit_node = curr_crit_node;
        num_edges = vertexes[crit_node].num_edges;
    }

    prev_crit_node->next = NULL;
    return (critical_path_head);
}

void get_tnode_block_and_output_net(int ivex, int* iblk_ptr, int* inet_ptr)
{
    /* Returns the index of the blocks that this vertexes is part of.  If the vertexes *
     * is a CLB_OPIN or INPAD_OPIN (i.e. if it drives a net), the net index is  *
     * returned via inet_ptr.  Otherwise inet_ptr points at OPEN.               */
    int inet, ipin;
    int iblk = tnode_descript[ivex].iblk;

    vertex_type_t tnode_type = tnode_descript[ivex].type;
    if (tnode_type == CLB_OPIN || tnode_type == INPAD_OPIN) {
        ipin = tnode_descript[ivex].ipin;
        inet = blocks[iblk].nets[ipin];
    } else {
        inet = OPEN;
    }

    *iblk_ptr = iblk;
    *inet_ptr = inet;
}

/* Does a timing analysis (simple) where it assumes that each net has a      *
 * constant Tdel value.  Used only when operation == TIMING_ANALYSIS_ONLY.  */
void do_constant_net_delay_timing_analysis(placer_opts_t   placer_opts,
                                           timing_info_t   timing_inf,
                                           subblock_data_t subblock_data,
                                           double constant_net_delay_value)
{
    alloc_and_load_timing_graph(placer_opts,
                                timing_inf,
                                subblock_data);
    double** net_slack = alloc_net_slack();
    linked_vptr_t* net_delay_chunk_list_head = NULL;
    double** net_delay = alloc_net_delay(&net_delay_chunk_list_head);

    load_constant_net_delay(net_delay,
                            constant_net_delay_value);

    load_timing_graph_net_delays(net_delay);
    /* target_cycle_time was 0 */
    double T_crit = calc_all_vertexs_arr_req_time(0);
    compute_net_slacks(net_slack);

    printf("\n");
    print_critical_path("critical_path.echo");
    printf("\nCritical Path: %g (s)\n", T_crit);
    /* print_timing_graph ("timing_graph.echo");
     print_net_slack ("net_slack.echo", net_slack);
     print_net_delay (net_delay, "net_delay.echo"); */
    free_timing_graph(net_slack);
    free_net_delay(net_delay,
                   &net_delay_chunk_list_head);
} /* end of void do_constant_net_delay_timing_analysis() */

