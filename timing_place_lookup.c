#include <stdio.h>
#include <math.h>
#include <string.h>

#include "vpr_types.h"
#include "globals.h"
#include "route_common.h"
#include "place_and_route.h"
#include "route_tree_timing.h"
#include "route_timing.h"
#include "timing_place_lookup.h"
#include "rr_graph.h"
#include "route_export.h"


/*this file contains routines that generate the array containing*/
/*the Tdels between blocks, this is used in the timing driven  */
/*placement routines */

/*To compute Tdel between blocks we place temporary blocks at */
/*different locations in the FPGA and route nets  between      */
/*the blocks.  From this procedure we generate a lookup table  */
/*which tells us the Tdel between different locations in      */
/*the FPGA */

/*Note: if you do not have logically equivalent output and    */
/*inputs, some of this code must be re-written.               */

/*Note: these routines assume that there is a uniform and even */
/*distribution of the different wire segments. If this is not  */
/*the case, then this lookup table will be off */

#define NET_COUNT 1 /*we only use one net in these routines,   */
/*it is repeatedly routed and ripped up    */
/*to compute Tdels between different      */
/*locations, this value should not change  */
#define NET_USED 0 /*we use net at location zero of the net    */
/*structure                                 */
#define NET_USED_SOURCE_BLOCK 0  /*net.blocks[0] is source blocks*/
#define NET_USED_SINK_BLOCK 1    /*net.blocks[1] is sink blocks */
#define SOURCE_BLOCK 0 /*blocks[0] is source */
#define SINK_BLOCK 1 /*blocks[1] is sink*/

#define BLOCK_COUNT 2 /*use 2 blocks to compute Tdel between  */
/*the various FPGA locations             */
/*do not change this number unless you   */
/*really know what you are doing, it is  */
/*assumed that the net only connects to  */
/*two blocks*/

#define DEBUG_TIMING_PLACE_LOOKUP /*initialize arrays to known state*/

#define DUMPFILE "lookup_dump.echo"
/*#define PRINT_ARRAYS*/ /*only used during debugging, calls routine to  */
/*print out the various lookup arrays           */

/***variables that are exported to other modules***/

/*the delta arrays are used to contain the best case routing Tdel */
/*between different locations on the FPGA. */

double** delta_inpad_to_clb;
double** delta_clb_to_clb;
double** delta_clb_to_outpad;
double** delta_inpad_to_outpad;


/*** Other Global Arrays ******/
/* I could have allocated these as local variables, and passed them all */
/* around, but was too lazy, since this is a small file, it should not  */
/* be a big problem */

static double** net_delay;
static double** net_slack;
static double* pin_criticality;
static int* sink_order;
static t_rt_node** rt_node_of_sink;

vector_t** clb_opins_used_locally;

static FILE* lookup_dump; /*if debugging mode is on, print out to   */
/*the file defined in DUMPFILE */

/*** Function Prototypes *****/

static void alloc_net(void);

static void alloc_block(void);

static void alloc_and_assign_internal_structures(net_t** original_net,
                                                 block_t** original_block,
                                                 int* original_num_nets,
                                                 int* original_num_blocks);

static void free_and_reset_internal_structures(net_t* original_net,
                                               block_t* original_block,
                                               int original_num_nets,
                                               int original_num_blocks);

static void setup_chan_width(router_opts_t router_opts,
                             chan_width_distr_t chan_width_dist);

static void alloc_routing_structs(router_opts_t router_opts,
                                  detail_routing_arch_t det_routing_arch,
                                  segment_info_t* segment_inf,
                                  timing_info_t timing_inf, subblock_data_t subblock_data);

static void free_routing_structs(router_opts_t router_opts,
                                 detail_routing_arch_t det_routing_arch,
                                 segment_info_t* segment_inf,
                                 timing_info_t timing_inf
                                );

static void assign_locations(block_types_t source_type,
                             int source_x_loc, int source_y_loc,
                             block_types_t sink_type,
                             int sink_x_loc, int sink_y_loc);

static double assign_blocks_and_route_net(block_types_t source_type,
                                          int source_x_loc,
                                          int source_y_loc,
                                          block_types_t sink_type,
                                          int sink_x_loc,
                                          int sink_y_loc,
                                          router_opts_t router_opts);

static void alloc_delta_arrays(void);

static void free_delta_arrays(void);

static void generic_compute_matrix(double** *matrix_ptr,
                                   block_types_t source_type,
                                   block_types_t sink_type, int source_x,
                                   int source_y, int start_x,
                                   int end_x, int start_y, int end_y,
                                   router_opts_t router_opts,
                                   detail_routing_arch_t det_routing_arch,
                                   segment_info_t* segment_inf,
                                   timing_info_t timing_inf);

static void compute_delta_clb_to_clb(router_opts_t router_opts,
                                     int longest_length);

static void compute_delta_inpad_to_clb(router_opts_t router_opts,
                                       detail_routing_arch_t det_routing_arch,
                                       segment_info_t* segment_inf,
                                       timing_info_t timing_inf);

static void compute_delta_clb_to_outpad(router_opts_t router_opts);

static void compute_delta_inpad_to_outpad(router_opts_t router_opts);

static void compute_delta_arrays(router_opts_t router_opts,
                                 detail_routing_arch_t det_routing_arch,
                                 segment_info_t* segment_inf,
                                 timing_info_t timing_inf, int longest_length);

static int get_first_pin(pin_types_t pintype);

static int get_longest_segment_length(detail_routing_arch_t det_routing_arch,
                                      segment_info_t* segment_inf);
static void init_occ(void);

#ifdef PRINT_ARRAYS
static void  print_array(double** array_to_print, int x1, int x2, int y1, int y2);
#endif
/**************************************/
static int  get_first_pin(pin_types_t pintype)
{
    /*this code assumes logical equivilance between all driving pins*/
    /*global pins are not hooked up to the temporary net */
    int currpin = 0;
    int i = -1;
    for (i = 0; i < num_pin_class; ++i) {
        if (class_inf[i].type == pintype && !is_global_clb_pin[currpin]) {
            return(class_inf[i].pinlist[0]);
        } else {
            currpin += class_inf[i].num_pins;
        }
    }

    exit(0); /*should never hit this line*/
}

/**************************************/
static int get_longest_segment_length(detail_routing_arch_t det_routing_arch,
                                      segment_info_t* segment_inf)
{
    int length = 0;
    int i = -1;
    for (i = 0; i < det_routing_arch.num_segment; i++) {
        if (segment_inf[i].length > length) {
            length = segment_inf[i].length;
        }
    }

    return length;
}

/**************************************/
static void alloc_net(void)
{
    net = (net_t*)my_malloc(num_nets * sizeof(net_t));

    int i = -1;
    for (i = 0; i < NET_COUNT; ++i) {
        int len = strlen("TEMP_NET");
        net[i].name = (char*)my_malloc((len + 1) * sizeof(char));
        strcpy(net[NET_USED].name, "TEMP_NET");
        net[i].num_net_pins = BLOCK_COUNT;
        net[i].node_blocks = (int*)my_malloc(BLOCK_COUNT * sizeof(int));
        net[i].node_blocks[NET_USED_SOURCE_BLOCK] = NET_USED_SOURCE_BLOCK;/*driving blocks*/
        net[i].node_blocks[NET_USED_SINK_BLOCK] = NET_USED_SINK_BLOCK;  /*target blocks */
        net[i].node_block_pins = (int*)my_malloc(BLOCK_COUNT * sizeof(int));
        /*the values for this are allocated in assign_blocks_and_route_net*/
    }
}

/**************************************/
static void alloc_block(void)
{
    /*allocates blocks structure, and assigns values to known parameters*/
    /*type and x,y fields are left undefined at this stage since they  */
    /*are not known until we start moving blocks through the clb array */
    blocks = (block_t*)my_malloc(num_blocks * sizeof(block_t));

    int iblk = -1;
    for (iblk = 0; iblk < BLOCK_COUNT; ++iblk) {
        int len = strlen("TEMP_BLOCK");
        blocks[iblk].name = (char*)my_malloc((len + 1) * sizeof(char));
        strcpy(blocks[iblk].name, "TEMP_BLOCK");
        blocks[iblk].nets = (int*)my_malloc(pins_per_clb * sizeof(int));
        blocks[iblk].nets[0] = 0; /* 0 is DRIVER */

        int ipin = 0;
        for (ipin = 1; ipin < pins_per_clb; ++ipin) {
            blocks[iblk].nets[ipin] = OPEN;
        }
    }
}
/**************************************/
static void init_occ(void)
{
    int i, j;
    for (i = 0; i <= num_grid_columns + 1; i++) {
        for (j = 0; j <= num_grid_rows + 1; j++) {
            clb_grids[i][j].m_usage = 0;
        }
    }
}
/**************************************/
static void alloc_and_assign_internal_structures(net_t** original_net,
                                                 block_t** original_block,
                                                 int* original_num_nets,
                                                 int* original_num_blocks)
{
    /*allocate new data structures to hold net, and blocks info*/
    *original_net = net;
    *original_num_nets = num_nets;
    num_nets = NET_COUNT; /* why? */

    alloc_net();

    *original_block = blocks;
    *original_num_blocks = num_blocks;
    num_blocks = BLOCK_COUNT;

    alloc_block();
    /* [0..num_nets-1][1..num_pins-1] */
    net_delay = (double**)alloc_matrix(0,
                                       NET_COUNT - 1,
                                       1,
                                       BLOCK_COUNT - 1,
                                       sizeof(double));
    net_slack = (double**)alloc_matrix(0,
                                       NET_COUNT - 1,
                                       1,
                                       BLOCK_COUNT - 1,
                                       sizeof(double));
    init_occ();
}

/**************************************/
static void free_and_reset_internal_structures(net_t* original_net,
                                               block_t* original_block,
                                               int original_num_nets,
                                               int original_num_blocks)
{
    /*reset gloabal data structures to the state that they were in before these*/
    /*lookup computation routines were called */
    int i;

    /*there should be only one net to free, but this is safer*/
    for (i = 0; i < NET_COUNT; ++i) {
        free(net[i].name);
        free(net[i].node_blocks);
        free(net[i].node_block_pins);
    }

    free(net);
    net = original_net;

    for (i = 0; i < BLOCK_COUNT; ++i) {
        free(blocks[i].name);
        free(blocks[i].nets);
    }

    free(blocks);
    blocks = original_block;
    num_nets = original_num_nets;
    num_blocks = original_num_blocks;
    free_matrix(net_delay, 0, NET_COUNT - 1, 1, sizeof(double));
    free_matrix(net_slack, 0, NET_COUNT - 1, 1, sizeof(double));
}  /* end of static void free_and_reset_internal_structures(net_t* original_net,) */

/**************************************/
static void setup_chan_width(router_opts_t router_opts,
                             chan_width_distr_t chan_width_dist)
{
    /*we give plenty of tracks, this increases routability for the */
    /*lookup table generation */
    int width_fac = 0;

    if (router_opts.fixed_channel_width == NO_FIXED_CHANNEL_WIDTH) {
        width_fac = 4 * pins_per_clb; /*this is 2x the value that binary search starts*/
    }
    /*this should be enough to allow most pins to   */
    /*connect to tracks in the architecture */
    else {
        width_fac = router_opts.fixed_channel_width;
    }

    init_channel_t(width_fac, chan_width_dist);
}
/**************************************/
static void alloc_routing_structs(router_opts_t router_opts,
                                  detail_routing_arch_t det_routing_arch,
                                  segment_info_t* segment_inf,
                                  timing_info_t timing_inf,
                                  subblock_data_t subblock_data)
{
    /*calls routines that set up routing resource graph and associated structures*/
    /*must set up dummy blocks for the first pass through*/
    assign_locations(B_CLB_TYPE, 1, 1,
                     B_CLB_TYPE, num_grid_columns, num_grid_rows);

    clb_opins_used_locally = alloc_route_structs(subblock_data);
    free_rr_graph();
    build_rr_graph(router_opts.route_type,
                   det_routing_arch, segment_inf,
                   timing_inf,
                   router_opts.router_base_cost_type);

    alloc_and_load_rr_node_route_structs();
    alloc_timing_driven_route_structs(&pin_criticality,
                                      &sink_order,
                                      &rt_node_of_sink);
    int bb_factor = num_grid_columns + num_grid_rows; /*set it to a huge value*/
    init_route_structs(bb_factor);
}
/**************************************/
static void free_routing_structs(router_opts_t router_opts,
                                 detail_routing_arch_t det_routing_arch,
                                 segment_info_t* segment_inf,
                                 timing_info_t timing_inf
                                )
{
    free_rr_graph_internals(router_opts.route_type, det_routing_arch, segment_inf,
                            timing_inf, router_opts.router_base_cost_type);
    free_rr_graph();
    free_rr_node_route_structs();
    free_route_structs(clb_opins_used_locally);
    free_trace_structs();
    free_timing_driven_route_structs(pin_criticality, sink_order, rt_node_of_sink);
}
/**************************************/
static void assign_locations(block_types_t source_type,
                             int source_x_loc,
                             int source_y_loc,
                             block_types_t sink_type,
                             int sink_x_loc,
                             int sink_y_loc)
{
    /*all routing occurs between blocks 0 (source) and blocks 1 (sink)*/
    blocks[SOURCE_BLOCK].block_type = source_type;
    blocks[SINK_BLOCK].block_type = sink_type;
    blocks[SOURCE_BLOCK].x = source_x_loc;
    blocks[SOURCE_BLOCK].y = source_y_loc;
    blocks[SINK_BLOCK].x = sink_x_loc;
    blocks[SINK_BLOCK].y = sink_y_loc;

    int isubblk = 0;
    if (source_type == B_CLB_TYPE) {
        net[NET_USED].node_block_pins[NET_USED_SOURCE_BLOCK] = get_first_pin(DRIVER);
        clb_grids[source_x_loc][source_y_loc].u.blocks = SOURCE_BLOCK;
        clb_grids[source_x_loc][source_y_loc].m_usage += 1;
    } else {
        net[NET_USED].node_block_pins[NET_USED_SOURCE_BLOCK] = OPEN;
        isubblk = clb_grids[source_x_loc][source_y_loc].m_usage;
        clb_grids[source_x_loc][source_y_loc].u.io_blocks[isubblk] = SOURCE_BLOCK;
        clb_grids[source_x_loc][source_y_loc].m_usage += 1;
    }

    if (sink_type == B_CLB_TYPE) {
        net[NET_USED].node_block_pins[NET_USED_SINK_BLOCK] = get_first_pin(RECEIVER);
        clb_grids[sink_x_loc][sink_y_loc].u.blocks = SINK_BLOCK;
        clb_grids[sink_x_loc][sink_y_loc].m_usage += 1;
    } else {
        net[NET_USED].node_block_pins[NET_USED_SINK_BLOCK] = OPEN;
        isubblk = clb_grids[sink_x_loc][sink_y_loc].m_usage;
        clb_grids[sink_x_loc][sink_y_loc].u.io_blocks[isubblk] = SINK_BLOCK;
        clb_grids[sink_x_loc][sink_y_loc].m_usage += 1;
    }
}

/**************************************/
static double assign_blocks_and_route_net(block_types_t source_type,
                                         int source_x_loc, int source_y_loc,
                                         block_types_t sink_type,
                                         int sink_x_loc, int sink_y_loc,
                                         router_opts_t router_opts)
{
    /*places blocks at the specified locations, and routes a net between them*/
    /*returns the Tdel of this net */
    double net_delay_value = IMPOSSIBLE; /*set to known value for debug purposes*/
    assign_locations(source_type,
                     source_x_loc,
                     source_y_loc,
                     sink_type,
                     sink_x_loc,
                     sink_y_loc);
    int** rr_node_indices = gerr_node_t_indices();
    int nodes_per_chan = get_nodes_per_chan();
    load_net_rr_terminals(rr_node_indices,
                          nodes_per_chan);
    double T_crit = 1.0;
    double pres_fac = 0.0; /* ignore congestion */

    const int knum_net_pins = net[NET_USED].num_net_pins;
    int ipin = 0;
    for (ipin = 1; ipin < knum_net_pins; ++ipin) {
        net_slack[NET_USED][ipin] = 0;
    }

    boolean is_routeable = timing_driven_route_net(NET_USED,
                                                   pres_fac,
                                                   router_opts.max_criticality,
                                                   router_opts.criticality_exp,
                                                   router_opts.astar_fac,
                                                   router_opts.bend_cost,
                                                   net_slack[NET_USED],
                                                   pin_criticality,
                                                   sink_order,
                                                   rt_node_of_sink,
                                                   T_crit,
                                                   net_delay[NET_USED]);
    net_delay_value = net_delay[NET_USED][NET_USED_SINK_BLOCK];
    clb_grids[source_x_loc][source_y_loc].m_usage = 0;
    clb_grids[sink_x_loc][sink_y_loc].m_usage = 0;
    return net_delay_value;
}

/**************************************/
static void alloc_delta_arrays(void)  /* FIXME */
{
    delta_clb_to_clb = (double**)alloc_matrix(0,
                                              num_grid_columns - 1,
                                              0,
                                              num_grid_rows - 1,
                                              sizeof(double));
    delta_inpad_to_clb = (double**)alloc_matrix(0,
                                                num_grid_columns,
                                                0,
                                                num_grid_rows,
                                                sizeof(double));
    delta_clb_to_outpad = (double**)alloc_matrix(0,
                                                 num_grid_columns,
                                                 0,
                                                 num_grid_rows,
                                                 sizeof(double));
    delta_inpad_to_outpad = (double**)alloc_matrix(0,
                                                   num_grid_columns + 1,
                                                   0,
                                                   num_grid_rows + 1,
                                                   sizeof(double));

    /*initialize all of the array locations to -1*/
    int id_x = -1;
    int id_y = -1;
    for (id_x = 0; id_x <= num_grid_columns; ++id_x) {
        for (id_y = 0; id_y <= num_grid_rows; ++id_y) {
            delta_inpad_to_clb[id_x][id_y] = IMPOSSIBLE;
        }
    }

    for (id_x = 0; id_x <= num_grid_columns - 1; ++id_x) {
        for (id_y = 0; id_y <= num_grid_rows - 1; ++id_y) {
            delta_clb_to_clb[id_x][id_y] = IMPOSSIBLE;
        }
    }

    for (id_x = 0; id_x <= num_grid_columns; ++id_x) {
        for (id_y = 0; id_y <= num_grid_rows; ++id_y) {
            delta_clb_to_outpad[id_x][id_y] = IMPOSSIBLE;
        }
    }

    for (id_x = 0; id_x <= num_grid_columns + 1; ++id_x) {
        for (id_y = 0; id_y <= num_grid_rows + 1; ++id_y) {
            delta_inpad_to_outpad[id_x][id_y] = IMPOSSIBLE;
        }
    }
} /* end of static void alloc_delta_arrays() */

/**************************************/
static void free_delta_arrays(void)
{
    free_matrix(delta_inpad_to_clb,
                0,
                num_grid_columns,
                0,
                sizeof(double));
    free_matrix(delta_clb_to_clb,
                0,
                num_grid_columns - 1,
                0,
                sizeof(double));
    free_matrix(delta_clb_to_outpad,
                0,
                num_grid_columns,
                0,
                sizeof(double));
    free_matrix(delta_inpad_to_outpad,
                0,
                num_grid_columns + 1,
                0,
                sizeof(double));
}
/**************************************/
static void generic_compute_matrix(double*** matrix_ptr,
                                   block_types_t source_type,
                                   block_types_t sink_type,
                                   int source_x,
                                   int source_y,
                                   int start_x,
                                   int end_x,
                                   int start_y,
                                   int end_y,
                                   router_opts_t router_opts,
                                   detail_routing_arch_t det_routing_arch,
                                   segment_info_t* segment_inf,
                                   timing_info_t timing_inf)
{
    int delta_x = 0;
    int delta_y = 0;
    int sink_x = 0;
    int sink_y = 0;
    for (sink_x = start_x; sink_x <= end_x; ++sink_x) {
        for (sink_y = start_y; sink_y <= end_y; ++sink_y) {
            delta_x = abs(sink_x - source_x);
            delta_y = abs(sink_y - source_y);

            if (delta_x == 0 && delta_y == 0) {
                continue;    /*do not compute distance from a blocks to itself     */
            }

            /*if a value is desired, pre-assign it somewhere else*/
            (*matrix_ptr)[delta_x][delta_y] =
                assign_blocks_and_route_net(source_type,
                                            source_x,
                                            source_y,
                                            sink_type,
                                            sink_x,
                                            sink_y,
                                            router_opts);
        }
    }
}

/*this routine must compute Tdel values in a slightly different way than the*
 *other compute routines. We cannot use a location close to the edge as the  *
 *source location for the majority of the Tdel computations. Because this   *
 *would give gradually increasing Tdel values. To avoid this from happening *
 *a clb that is at least longest_length away from an edge should be chosen   *
 *as a source , if longest_length is more than 0.5 of the total size then    *
 *choose a CLB_TYPE at the center as the source CLB_TYPE */
static void compute_delta_clb_to_clb(router_opts_t router_opts,
                                     int longest_length)
{
    block_types_t source_type = B_CLB_TYPE;
    block_types_t sink_type = B_CLB_TYPE;

    int start_x = 0;
    if (longest_length < 0.5 * (num_grid_columns)) {
        start_x = longest_length;
    } else {
        start_x = (int)(0.5 * num_grid_columns);
    }
    const int end_x = num_grid_columns;

    int start_y = 0;
    if (longest_length < 0.5 * (num_grid_rows)) {
        start_y = longest_length;
    } else {
        start_y = (int)(0.5 * num_grid_rows);
    }
    const int end_y = num_grid_rows;

    /*don't put the sink all the way to the corner, until it is necessary*/
    int delta_x = 0;
    int delta_y = 0;
    int sink_x = 0;
    int sink_y = 0;
    int source_x = start_x;
    int source_y = start_y;
    for (sink_x = start_x; sink_x <= end_x - 1; ++sink_x) {
        for (sink_y = start_y; sink_y <= end_y - 1; ++sink_y) {
            delta_x = abs(sink_x - source_x);
            delta_y = abs(sink_y - source_y);
            if (delta_x == 0 && delta_y == 0) {
                delta_clb_to_clb[delta_x][delta_y] = 0.0;
                continue;
            }

            delta_clb_to_clb[delta_x][delta_y] =
                assign_blocks_and_route_net(source_type,
                                            source_x,
                                            source_y,
                                            sink_type,
                                            sink_x,
                                            sink_y,
                                            router_opts);
        }
    }

    sink_x = end_x - 1;
    sink_y = end_y - 1;
    for (source_x = start_x - 1; source_x >= 1; --source_x) {
        for (source_y = start_y; source_y <= end_y - 1; ++source_y) {
            delta_x = abs(sink_x - source_x);
            delta_y = abs(sink_y - source_y);
            delta_clb_to_clb[delta_x][delta_y] =
                assign_blocks_and_route_net(source_type,
                                            source_x,
                                            source_y,
                                            sink_type,
                                            sink_x,
                                            sink_y,
                                            router_opts);
        }
    }

    for (source_x = 1; source_x <= end_x - 1; ++source_x) {
        for (source_y = 1; source_y < start_y; ++source_y) {
            delta_x = abs(sink_x - source_x);
            delta_y = abs(sink_y - source_y);
            delta_clb_to_clb[delta_x][delta_y] =
                assign_blocks_and_route_net(source_type,
                                            source_x,
                                            source_y,
                                            sink_type,
                                            sink_x,
                                            sink_y,
                                            router_opts);
        }
    }

    /*now move sink into the top right corner*/
    sink_x = end_x;
    sink_y = end_y;
    source_x = 1;
    for (source_y = 1; source_y <= end_y; ++source_y) {
        delta_x = abs(sink_x - source_x);
        delta_y = abs(sink_y - source_y);
        delta_clb_to_clb[delta_x][delta_y] =
            assign_blocks_and_route_net(source_type,
                                        source_x,
                                        source_y,
                                        sink_type,
                                        sink_x,
                                        sink_y,
                                        router_opts);
    }

    sink_x = end_x;
    sink_y = end_y;
    source_y = 1;
    for (source_x = 1; source_x <= end_x; ++source_x) {
        delta_x = abs(sink_x - source_x);
        delta_y = abs(sink_y - source_y);
        delta_clb_to_clb[delta_x][delta_y] =
            assign_blocks_and_route_net(source_type,
                                        source_x,
                                        source_y,
                                        sink_type,
                                        sink_x,
                                        sink_y,
                                        router_opts);
    }
} /* end of static void compute_delta_clb_to_clb() */

/**************************************/
static void compute_delta_inpad_to_clb(router_opts_t router_opts,
                                       detail_routing_arch_t det_routing_arch,
                                       segment_info_t* segment_inf,
                                       timing_info_t timing_inf)
{
    block_types_t source_type = INPAD_TYPE;
    block_types_t sink_type = B_CLB_TYPE;
    delta_inpad_to_clb[0][0] = IMPOSSIBLE;
    delta_inpad_to_clb[num_grid_columns][num_grid_rows] = IMPOSSIBLE;
    int source_x = 0;
    int source_y = 1;
    int start_x = 1;
    int end_x = num_grid_columns;
    int start_y = 1;
    int end_y = num_grid_rows;
    generic_compute_matrix(&delta_inpad_to_clb,
                           source_type,
                           sink_type,
                           source_x,
                           source_y,
                           start_x,
                           end_x,
                           start_y,
                           end_y,
                           router_opts,
                           det_routing_arch,
                           segment_inf,
                           timing_inf);
    source_x = 1;
    source_y = 0;
    start_x = 1;
    end_x = 1;
    start_y = 1;
    end_y = num_grid_rows;
    generic_compute_matrix(&delta_inpad_to_clb,
                           source_type,
                           sink_type,
                           source_x,
                           source_y,
                           start_x,
                           end_x,
                           start_y,
                           end_y,
                           router_opts,
                           det_routing_arch,
                           segment_inf,
                           timing_inf);
    start_x = 1;
    end_x = num_grid_columns;
    start_y = num_grid_rows;
    end_y = num_grid_rows;
    generic_compute_matrix(&delta_inpad_to_clb,
                           source_type,
                           sink_type,
                           source_x,
                           source_y,
                           start_x,
                           end_x,
                           start_y,
                           end_y,
                           router_opts,
                           det_routing_arch,
                           segment_inf,
                           timing_inf);
} /* end of static void compute_delta_inpad_to_clb() */

/**************************************/
static void compute_delta_clb_to_outpad(router_opts_t router_opts)
{
    block_types_t source_type = B_CLB_TYPE;
    block_types_t sink_type = OUTPAD_TYPE;
    delta_clb_to_outpad[0][0] = IMPOSSIBLE;
    delta_clb_to_outpad[num_grid_columns][num_grid_rows] = IMPOSSIBLE;

    int sink_x = 0;
    int sink_y = 1;
    int source_x = 0;
    int source_y = 0;
    int delta_x = 0;
    int delta_y = 0;
    for (source_x = 1; source_x <= num_grid_columns; ++source_x) {
        for (source_y = 1; source_y <= num_grid_rows; ++source_y) {
            delta_x = abs(source_x - sink_x);
            delta_y = abs(source_y - sink_y);
            delta_clb_to_outpad[delta_x][delta_y] =
                            assign_blocks_and_route_net(source_type,
                                                        source_x,
                                                        source_y,
                                                        sink_type,
                                                        sink_x,
                                                        sink_y,
                                                        router_opts);
        }
    }

    sink_x = 1;
    sink_y = 0;
    source_x = 1;
    delta_x = abs(source_x - sink_x);
    for (source_y = 1; source_y <= num_grid_rows; ++source_y) {
        delta_y = abs(source_y - sink_y);
        delta_clb_to_outpad[delta_x][delta_y] =
            assign_blocks_and_route_net(source_type,
                                        source_x,
                                        source_y,
                                        sink_type,
                                        sink_x,
                                        sink_y,
                                        router_opts);
    }

    sink_x = 1;
    sink_y = 0;
    source_y = num_grid_rows;
    delta_y = abs(source_y - sink_y);
    for (source_x = 2; source_x <= num_grid_columns; ++source_x) {
        delta_x = abs(source_x - sink_x);
        delta_clb_to_outpad[delta_x][delta_y] =
            assign_blocks_and_route_net(source_type,
                                        source_x,
                                        source_y,
                                        sink_type,
                                        sink_x,
                                        sink_y,
                                        router_opts);
    }
} /* end of static void compute_delta_clb_to_outpad() */

/**************************************/
static void compute_delta_inpad_to_outpad(router_opts_t router_opts)
{
    block_types_t source_type = INPAD_TYPE;
    block_types_t sink_type = OUTPAD_TYPE;

    delta_inpad_to_outpad[0][0] = 0; /*Tdel to itself is 0 (this can happen)*/
    delta_inpad_to_outpad[num_grid_columns + 1][num_grid_rows + 1] = IMPOSSIBLE;
    delta_inpad_to_outpad[0][num_grid_rows] = IMPOSSIBLE;
    delta_inpad_to_outpad[num_grid_columns][0] = IMPOSSIBLE;
    delta_inpad_to_outpad[num_grid_columns][num_grid_rows + 1] = IMPOSSIBLE;
    delta_inpad_to_outpad[num_grid_columns + 1][num_grid_rows] = IMPOSSIBLE;

    int source_x = 0;
    int source_y = 1;
    int sink_x = 0;
    int sink_y = 0;
    int delta_x = abs(sink_x - source_x);
    int delta_y = 0;
    for (sink_y = 2; sink_y <= num_grid_rows; ++sink_y) {
        delta_y = abs(sink_y - source_y);
        delta_inpad_to_outpad[delta_x][delta_y] =
            assign_blocks_and_route_net(source_type,
                                        source_x,
                                        source_y,
                                        sink_type,
                                        sink_x,
                                        sink_y,
                                        router_opts);
    }

    source_x = 0;
    source_y = 1;
    sink_x = num_grid_columns + 1;
    delta_x = abs(sink_x - source_x);
    for (sink_y = 1; sink_y <= num_grid_rows; ++sink_y) {
        delta_y = abs(sink_y - source_y);
        delta_inpad_to_outpad[delta_x][delta_y] =
            assign_blocks_and_route_net(source_type,
                                        source_x,
                                        source_y,
                                        sink_type,
                                        sink_x,
                                        sink_y,
                                        router_opts);
    }

    source_x = 1;
    source_y = 0;
    sink_y = 0;
    delta_y = abs(sink_y - source_y);
    for (sink_x = 2; sink_x <= num_grid_columns; ++sink_x) {
        delta_x = abs(sink_x - source_x);
        delta_inpad_to_outpad[delta_x][delta_y] =
            assign_blocks_and_route_net(source_type,
                                        source_x,
                                        source_y,
                                        sink_type,
                                        sink_x,
                                        sink_y,
                                        router_opts);
    }

    source_x = 1;
    source_y = 0;
    sink_y = num_grid_rows + 1;
    delta_y = abs(sink_y - source_y);
    for (sink_x = 1; sink_x <= num_grid_columns; ++sink_x) {
        delta_x = abs(sink_x - source_x);
        delta_inpad_to_outpad[delta_x][delta_y] =
            assign_blocks_and_route_net(source_type,
                                        source_x,
                                        source_y,
                                        sink_type,
                                        sink_x,
                                        sink_y,
                                        router_opts);
    }

    source_x = 0;
    sink_y = num_grid_rows + 1;
    for (source_y = 1; source_y <= num_grid_rows; ++source_y) {
        for (sink_x = 1; sink_x <= num_grid_columns; ++sink_x) {
            delta_y = abs(source_y - sink_y);
            delta_x = abs(source_x - sink_x);
            delta_inpad_to_outpad[delta_x][delta_y] =
                assign_blocks_and_route_net(source_type,
                                            source_x,
                                            source_y,
                                            sink_type,
                                            sink_x,
                                            sink_y,
                                            router_opts);
        }
    }
} /* end of static void compute_delta_inpad_to_outpad() */
/**************************************/
#ifdef PRINT_ARRAYS
static void  print_array(double** array_to_print,
                         int x1,
                         int x2,
                         int y1,
                         int y2)
{
    int idx_x, idx_y;
    fprintf(lookup_dump, "\nPrinting Array \n\n");

    for (idx_y = y2; idx_y >= y1; idx_y --) {
        for (idx_x = x1; idx_x <= x2; idx_x ++) {
            fprintf(lookup_dump, " %9.2e", array_to_print[idx_x][idx_y]);
        }

        fprintf(lookup_dump, "\n");
    }

    fprintf(lookup_dump, "\n\n");
}
#endif

/**************************************/
static void compute_delta_arrays(router_opts_t router_opts,   /* FIXME */
                                 detail_routing_arch_t det_routing_arch,
                                 segment_info_t* segment_inf,
                                 timing_info_t timing_inf, int longest_length)
{
    compute_delta_clb_to_clb(router_opts,
                             longest_length);
    printf("Computing delta_clb_to_clb lookup matrix OK!\n");

    compute_delta_inpad_to_clb(router_opts,
                               det_routing_arch,
                               segment_inf,
                               timing_inf);
    printf("Computing delta_inpad_to_clb lookup matrix OK!\n");

    compute_delta_clb_to_outpad(router_opts);
    printf("Computing delta_clb_to_outpad lookup matrix OK!\n");

    compute_delta_inpad_to_outpad(router_opts);
    printf("Computing delta_inpad_to_outpad lookup matrix OK!\n");

#ifdef PRINT_ARRAYS
    lookup_dump = my_fopen(DUMPFILE, "w");

    fprintf(lookup_dump, "\n\nprinting delta_clb_to_clb\n");
    print_array(delta_clb_to_clb, 0 , num_grid_columns - 1, 0, num_grid_rows - 1);

    fprintf(lookup_dump, "\n\nprinting delta_inpad_to_clb\n");
    print_array(delta_inpad_to_clb, 0, num_grid_columns, 0, num_grid_rows);

    fprintf(lookup_dump, "\n\nprinting delta_clb_to_outpad\n");
    print_array(delta_clb_to_outpad, 0, num_grid_columns, 0, num_grid_rows);

    fprintf(lookup_dump, "\n\nprinting delta_inpad_to_outpad\n");
    print_array(delta_inpad_to_outpad, 0, num_grid_columns + 1, 0, num_grid_rows + 1);

    fclose(lookup_dump);
#endif
}

/******* Globally Accessable Functions **********/

/*************************************************************
 * FIXME Main function in timing_place_lookup.c                    *
 * Compute 4 delay_lookup_matrixes for TIMING_DRIVEN_PLACEMENT *
 ************************************************************/
void alloc_and_compute_delay_lookup_matrixes(router_opts_t      router_opts,
                                             detail_routing_arch_t det_routing_arch,
                                             segment_info_t*    segment_inf,
                                             timing_info_t      timing_inf,
                                             chan_width_distr_t chan_width_dist,
                                             subblock_data_t    subblock_data)
{
    /* original_net will be used as a pointer to remember what the  *
     * "real" nets in the circuit are. This is required because we  *
     * are using the net structure in these routines to find Tdels *
     * between blocks.                                              */
    static net_t* original_net = NULL;
    /*original_block: same def as original_nets, but for blocks  */
    static block_t* original_block = NULL;

    static int original_num_nets;
    static int original_num_blocks;
    alloc_and_assign_internal_structures(&original_net,
                                         &original_block,
                                         &original_num_nets,
                                         &original_num_blocks);

    setup_chan_width(router_opts,
                     chan_width_dist);

    alloc_routing_structs(router_opts,
                          det_routing_arch,
                          segment_inf,
                          timing_inf,
                          subblock_data);

    static int longest_length;
    longest_length = get_longest_segment_length(det_routing_arch,
                                                segment_inf);

    /*now setup and compute 4 actual arrays: delta_clb_to_clb,         *
     * delta_inpad_to_clb, delta_clb_to_outpad, delta_inpad_to_outpad. */
    alloc_delta_arrays();
    compute_delta_arrays(router_opts,
                         det_routing_arch,
                         segment_inf,
                         timing_inf,
                         longest_length);  /* FIXME */

    /*free all data structures that are no longer needed*/
    free_routing_structs(router_opts,
                         det_routing_arch,
                         segment_inf,
                         timing_inf);
    free_and_reset_internal_structures(original_net,
                                       original_block,
                                       original_num_nets,
                                       original_num_blocks);
} /* end of static void alloc_and_compute_delay_lookup_matrixes() */

/**************************************/
void free_place_lookup_structs(void)
{
    free_delta_arrays();
}
