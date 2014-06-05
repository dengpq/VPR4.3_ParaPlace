#include <math.h>
#include <stdio.h>
#include "util.h"
#include "vpr_types.h"
#include "globals.h"
#include "route_export.h"
#include "route_common.h"
#include "route_tree_timing.h"
#include "route_timing.h"
#include "route_breadth_first.h"
#include "place_and_route.h"
#include "rr_graph.h"

/***************** Variables shared only by route modules *******************/

rr_node_t_route_inf* rr_node_route_inf = NULL;       /* [0..num_rr_nodes-1] */

bbox_t* route_bb = NULL; /* [0..num_nets-1]. Limits area in which each  */
/* net must be routed.                         */


/**************** Static variables local to route_common.c ******************/
static struct s_heap** heap;  /* Indexed from [1..heap_size] */
static int heap_size;   /* Number of slots in the heap array */
static int heap_tail;   /* Index of first unused slot in the heap array */

/* For managing my own list of currently free heap data structures.     */
static struct s_heap* heap_free_head = NULL;

/* For managing my own list of currently free trace data structures.    */
static struct s_trace* trace_free_head = NULL;

#ifdef DEBUG
static int num_trace_allocated = 0;   /* To watch for memory leaks. */
static int num_heap_allocated = 0;
static int num_linked_f_pointer_allocated = 0;
#endif

static struct s_linked_f_pointer* rr_modified_head = NULL;
static struct s_linked_f_pointer* linked_f_pointer_free_head = NULL;


/*  The numbering relation between the channels and clbs is:               *
 *                                                                         *
 *  | IO_TYPE | chan_   |  CLB_TYPE  | chan_   |  CLB_TYPE |               *
 *  |clb[0][2]| y[0][2] | clb[1][2]  | y[1][2] |  clb[2][2]|               *
 *  +-------- +         +------------+         +-----------+               *
 *                                                           } capacity in *
 *   No channel          chan_x[1][1]          chan_x[2][1]  } chan_width  *
 *                                                           } _x[1]       *
 *  +---------+         +------------+         +-----------+               *
 *  |         | chan_   |            | chan_   |           |               *
 *  | IO_TYPE | y[0][1] |  CLB_TYPE  | y[1][1] | CLB_TYPE  |               *
 *  |clb[0][1]|         |  clb[1][1] |         | clb[2][1] |               *
 *  |         |         |            |         |           |               *
 *  +---------+         +------------+         +-----------+               *
 *                                                           } capacity in *
 *                      chan_x[1][0]           chan_x[2][0]  } chan_width  *
 *                                                           } _x[0]       *
 *                      +------------+         +-----------+               *
 *              No      |            | No      |           |               *
 *            Channel   | IO_TYPE    | Channel |  IO_TYPE  |               *
 *                      |  clb[1][0] |         | clb[2][0] |               *
 *                      |            |         |           |               *
 *                      +------------+         +-----------+               *
 *                                                                         *
 *             {=======}              {=======}                            *
 *            Capacity in            Capacity in                           *
 *          chan_width_y[0]        chan_width_y[1]                         *
 *                                                                         */
/******************** Subroutines local to route_common.c *******************/

static void free_trace_data(struct s_trace* tptr);
static void load_route_bb(int bb_factor);

static struct s_trace* alloc_trace_data(void);
static void add_to_heap(struct s_heap* hptr);
static struct s_heap* alloc_heap_data(void);
static struct s_linked_f_pointer* alloc_linked_f_pointer(void);

static vector_t** alloc_and_load_clb_opins_used_locally(subblock_data_t
                                                      subblock_data);
static void adjust_one_rr_occ_and_pcost(int ivex, int add_or_sub, double
                                        pres_fac);



/************************** Subroutine definitions ***************************/

void save_routing(struct s_trace** best_routing, vector_t
                  ** clb_opins_used_locally, vector_t** saved_clb_opins_used_locally)
{
    /* This routing frees any routing currently held in best routing,       *
     * then copies over the current routing (held in trace_head), and       *
     * finally sets trace_head and trace_tail to all NULLs so that the      *
     * connection to the saved routing is broken.  This is necessary so     *
     * that the next iteration of the router does not free the saved        *
     * routing elements.  Also saves any data about locally used clb_opins, *
     * since this is also part of the routing.                              */
    int inet, iblk, iclass, ipin, num_local_opins;
    struct s_trace* tptr, *tempptr;

    for (inet = 0; inet < num_nets; inet++) {
        /* Free any previously saved routing.  It is no longer best. */
        tptr = best_routing[inet];

        while (tptr != NULL) {
            tempptr = tptr->next;
            free_trace_data(tptr);
            tptr = tempptr;
        }

        /* Save a pointer to the current routing in best_routing. */
        best_routing[inet] = trace_head[inet];
        /* Set the current (working) routing to NULL so the current trace       *
         * elements won't be reused by the memory allocator.                    */
        trace_head[inet] = NULL;
        trace_tail[inet] = NULL;
    }

    /* Save which OPINs are locally used.                           */

    for (iblk = 0; iblk < num_blocks; iblk++) {
        for (iclass = 0; iclass < num_pin_class; iclass++) {
            num_local_opins = clb_opins_used_locally[iblk][iclass].nelem;

            for (ipin = 0; ipin < num_local_opins; ipin++) {
                saved_clb_opins_used_locally[iblk][iclass].list[ipin] =
                    clb_opins_used_locally[iblk][iclass].list[ipin];
            }
        }
    }
}


void restore_routing(struct s_trace** best_routing, vector_t
                     ** clb_opins_used_locally, vector_t** saved_clb_opins_used_locally)
{
    /* Deallocates any current routing in trace_head, and replaces it with    *
     * the routing in best_routing.  Best_routing is set to NULL to show that *
     * it no longer points to a valid routing.  NOTE:  trace_tail is not      *
     * restored -- it is set to all NULLs since it is only used in            *
     * update_traceback.  If you need trace_tail restored, modify this        *
     * routine.  Also restores the locally used opin data.                    */
    int inet, iblk, ipin, iclass, num_local_opins;

    for (inet = 0; inet < num_nets; inet++) {
        /* Free any current routing. */
        free_traceback(inet);
        /* Set the current routing to the saved one. */
        trace_head[inet] = best_routing[inet];
        best_routing[inet] = NULL;          /* No stored routing. */
    }

    /* Save which OPINs are locally used.                           */

    for (iblk = 0; iblk < num_blocks; iblk++) {
        for (iclass = 0; iclass < num_pin_class; iclass++) {
            num_local_opins = clb_opins_used_locally[iblk][iclass].nelem;

            for (ipin = 0; ipin < num_local_opins; ipin++) {
                clb_opins_used_locally[iblk][iclass].list[ipin] =
                    saved_clb_opins_used_locally[iblk][iclass].list[ipin];
            }
        }
    }
}


void get_serial_num(void)
{
    /* This routine finds a "magic cookie" for the routing and prints it.    *
     * Use this number as a routing serial number to ensure that programming *
     * changes do not break the router.                                      */
    int inet, serial_num, ivex;
    struct s_trace* tptr;
    serial_num = 0;

    for (inet = 0; inet < num_nets; inet++) {
        /* Global nets will have null trace_heads (never routed) so they *
         * are not included in the serial number calculation.            */
        tptr = trace_head[inet];

        while (tptr != NULL) {
            ivex = tptr->index;
            serial_num += (inet + 1) * (rr_node[ivex].xlow * (num_grid_columns + 1) -
                                        rr_node[ivex].yhigh);
            serial_num -= rr_node[ivex].ptc_num * (inet + 1) * 10;
            serial_num -= rr_node[ivex].type * (inet + 1) * 100;
            serial_num %= 2000000000;  /* Prevent overflow */
            tptr = tptr->next;
        }
    }

    printf("Serial number (magic cookie) for the routing is: %d\n",
           serial_num);
}


/* FIXME: main function about PathFinder-based Router */
boolean try_route(int width_fac, router_opts_t router_opts,
                  detail_routing_arch_t det_routing_arch,
                  segment_info_t* segment_inf, timing_info_t timing_inf,
                  double** net_slack, double** net_delay,
                  chan_width_distr_t chan_width_dist,
                  vector_t** clb_opins_used_locally)
{
    /* Attempts a routing via an iterated maze router algorithm.  Width_fac *
     * specifies the relative width of the channels, while the members of   *
     * router_opts determine the value of the costs assigned to routing     *
     * resource node, etc.  det_routing_arch describes the detailed routing *
     * architecture (connection and switch boxes) of the FPGA; it is used   *
     * only if a DETAILED routing has been selected.                        */
    printf("\nAttempting routing with a width factor (usually maximum channel ");
    printf("width) of %d.\n", width_fac);

    /* Set the channel widths */
    init_channel_t(width_fac, chan_width_dist);

    /* Free any old routing graph, if one exists. */
    free_rr_graph();

    /* Set up the routing resource graph defined by this FPGA architecture. */
    build_rr_graph(router_opts.route_type, det_routing_arch, segment_inf,
                   timing_inf, router_opts.router_base_cost_type);

    /* TODO: Q: Why did Router free routing-resource_graph internals? */
    free_rr_graph_internals(router_opts.route_type, det_routing_arch, segment_inf,
                            timing_inf, router_opts.router_base_cost_type);

    /* Allocate and load some additional rr_graph information needed only by *
     * the router.                                                           */
    alloc_and_load_rr_node_route_structs();
    init_route_structs(router_opts.bb_factor);

    boolean success;
    if (router_opts.router_algorithm == BREADTH_FIRST) { /* Wirelength-Driven Router */
        success = try_breadth_first_route(router_opts, clb_opins_used_locally);
    } else {  /* TIMING_DRIVEN route, Default Router */
        success = try_timing_driven_route(router_opts, net_slack, net_delay,
                                          clb_opins_used_locally);
    }

    free_rr_node_route_structs();
    return (success);
} /* end of boolean try_route() */


boolean feasible_routing(void)
{
    /* This routine checks to see if this is a resource-feasible routing.      *
     * That is, are all rr_node capacity limitations respected?  It assumes    *
     * that the occupancy arrays are up to date when it is called.             */
    int ivex;

    for (ivex = 0; ivex < num_rr_nodes; ivex++)
        if (rr_node[ivex].m_usage > rr_node[ivex].capacity) {
            return (FALSE);
        }

    return (TRUE);
}


void pathfinder_update_one_cost(struct s_trace* route_segment_start,
                                int add_or_sub, double pres_fac)
{
    /* This routine updates the occupancy and pres_cost of the rr_nodes that are *
     * affected by the portion of the routing of one net that starts at          *
     * route_segment_start.  If route_segment_start is trace_head[inet], the     *
     * cost of all the nodes in the routing of net inet are updated.  If         *
     * add_or_sub is -1 the net (or net portion) is ripped up, if it is 1 the    *
     * net is added to the routing.  The size of pres_fac determines how severly *
     * oversubscribed rr_nodes are penalized.                                    */
    struct s_trace* tptr;
    int ivex, m_usage, capacity;
    tptr = route_segment_start;

    if (tptr == NULL) {      /* No routing yet. */
        return;
    }

    while (1) {
        ivex = tptr->index;
        m_usage = rr_node[ivex].m_usage + add_or_sub;
        capacity = rr_node[ivex].capacity;
        rr_node[ivex].m_usage = m_usage;

        /* pres_cost is Pn in the Pathfinder paper. I set my pres_cost according to *
         * the overuse that would result from having ONE MORE net use this routing  *
         * node.                                                                    */

        if (m_usage < capacity) {
            rr_node_route_inf[ivex].pres_cost = 1.;
        } else {
            rr_node_route_inf[ivex].pres_cost = 1. + (m_usage + 1 - capacity) *
                                                 pres_fac;
        }

        if (rr_node[ivex].type == SINK) {
            tptr = tptr->next;             /* Skip next segment. */

            if (tptr == NULL) {
                break;
            }
        }

        tptr = tptr->next;
    }   /* End while loop -- did an entire traceback. */
}


void pathfinder_update_cost(double pres_fac, double acc_fac)
{
    /* This routine recomputes the pres_cost and acc_cost of each routing        *
     * resource for the pathfinder algorithm after all nets have been routed.    *
     * It updates the accumulated cost to by adding in the number of extra       *
     * signals sharing a resource right now (i.e. after each complete iteration) *
     * times acc_fac.  It also updates pres_cost, since pres_fac may have        *
     * changed.  THIS ROUTINE ASSUMES THE OCCUPANCY VALUES IN RR_NODE ARE UP TO  *
     * DATE.                                                                     */
    int ivex, m_usage, capacity;

    for (ivex = 0; ivex < num_rr_nodes; ivex++) {
        m_usage = rr_node[ivex].m_usage;
        capacity = rr_node[ivex].capacity;

        if (m_usage > capacity) {
            rr_node_route_inf[ivex].acc_cost += (m_usage - capacity) * acc_fac;
            rr_node_route_inf[ivex].pres_cost = 1. + (m_usage + 1 - capacity) *
                                                 pres_fac;
        }
        /* If m_usage == capacity, we don't need to increase acc_cost, but a change    *
         * in pres_fac could have made it necessary to recompute the cost anyway.  */
        else if (m_usage == capacity) {
            rr_node_route_inf[ivex].pres_cost = 1. + pres_fac;
        }
    }
}


void init_route_structs(int bb_factor)
{
    /* Call this before you route any nets.  It frees any old traceback and   *
     * sets the list of rr_nodes touched to empty.                            */
    int i;

    for (i = 0; i < num_nets; i++) {
        free_traceback(i);
    }

    load_route_bb(bb_factor);

    /* Check that things that should have been emptied after the last routing *
     * really were.                                                           */

    if (rr_modified_head != NULL) {
        printf("Error in init_route_structs.  List of modified rr nodes is \n"
               "not empty.\n");
        exit(1);
    }

    if (heap_tail != 1) {
        printf("Error in init_route_structs.  Heap is not empty.\n");
        exit(1);
    }
}


struct s_trace* update_traceback(struct s_heap* hptr, int inet) {

    /* This routine adds the most recently finished wire segment to the         *
     * traceback linked list.  The first connection starts with the net SOURCE  *
     * and begins at the structure pointed to by trace_head[inet]. Each         *
     * connection ends with a SINK.  After each SINK, the next connection       *
     * begins (if the net has more than 2 pins).  The first element after the   *
     * SINK gives the routing node on a previous piece of the routing, which is *
     * the link from the existing net to this new piece of the net.             *
     * In each traceback I start at the end of a path and trace back through    *
     * its predecessors to the beginning.  I have stored information on the     *
     * predecesser of each node to make traceback easy -- this sacrificies some *
     * memory for easier code maintenance.  This routine returns a pointer to   *
     * the first "new" node in the traceback (node not previously in trace).    */

    struct s_trace* tptr, *prevptr, *temptail, *ret_ptr;
    int ivex;
    short iedge;
#ifdef DEBUG
    rr_types_t rr_type;
#endif

    ivex = hptr->index;

#ifdef DEBUG
    rr_type = rr_node[ivex].type;

    if (rr_type != SINK) {
        printf("Error in update_traceback.  Expected type = SINK (%d).\n",
               SINK);
        printf("Got type = %d while tracing back net %d.\n", rr_type, inet);
        exit(1);
    }

#endif

    tptr = alloc_trace_data();    /* SINK on the end of the connection */
    tptr->index = ivex;
    tptr->iswitch = OPEN;
    tptr->next = NULL;
    temptail = tptr;              /* This will become the new tail at the end */
    /* of the routine.                          */

    /* Now do it's predecessor. */

    ivex = hptr->u.prev_node;
    iedge = hptr->prev_edge;

    while (ivex != NO_PREVIOUS) {
        prevptr = alloc_trace_data();
        prevptr->index = ivex;
        prevptr->iswitch = rr_node[ivex].switches[iedge];
        prevptr->next = tptr;
        tptr = prevptr;
        iedge = rr_node_route_inf[ivex].prev_edge;
        ivex = rr_node_route_inf[ivex].prev_node;
    }

    if (trace_tail[inet] != NULL) {
        trace_tail[inet]->next = tptr;     /* Traceback ends with tptr */
        ret_ptr = tptr->next;              /* First new segment.       */
    } else {              /* This was the first "chunk" of the net's routing */
        trace_head[inet] = tptr;
        ret_ptr = tptr;                    /* Whole traceback is new. */
    }

    trace_tail[inet] = temptail;
    return (ret_ptr);
}


void reset_path_costs(void)
{
    /* The routine sets the path_cost to HUGE_FLOAT for all channel segments   *
     * touched by previous routing phases.                                     */
    struct s_linked_f_pointer* mod_ptr;
#ifdef DEBUG
    int num_mod_ptrs;
#endif

    /* The traversal method below is slightly painful to make it faster. */
    if (rr_modified_head != NULL) {
        mod_ptr = rr_modified_head;
#ifdef DEBUG
        num_mod_ptrs = 1;
#endif
        while (mod_ptr->next != NULL) {
            *(mod_ptr->fptr) = HUGE_FLOAT;
            mod_ptr = mod_ptr->next;
#ifdef DEBUG
            num_mod_ptrs++;
#endif
        }

        *(mod_ptr->fptr) = HUGE_FLOAT;   /* Do last one. */
        /* Reset the modified list and put all the elements back in the free   *
         * list.                                                               */
        mod_ptr->next = linked_f_pointer_free_head;
        linked_f_pointer_free_head = rr_modified_head;
        rr_modified_head = NULL;
#ifdef DEBUG
        num_linked_f_pointer_allocated -= num_mod_ptrs;
#endif
    }
}


double get_rr_cong_cost(int ivex)
{
    /* Returns the *congestion* cost of using this rr_node. */
    short cost_index;
    double cost;
    cost_index = rr_node[ivex].cost_index;
    cost = rr_indexed_data[cost_index].base_cost *
           rr_node_route_inf[ivex].acc_cost *
           rr_node_route_inf[ivex].pres_cost;
    return (cost);
}


void mark_ends(int inet)
{
    /* Mark all the SINKs of this net as targets by setting their target flags  *
     * to the number of times the net must connect to each SINK.  Note that     *
     * this number can occassionally be greater than 1 -- think of connecting   *
     * the same net to two inputs of an and-gate (and-gate inputs are logically *
     * equivalent, so both will connect to the same SINK).                      */
    /* int ipin = 0; */
    const int knum_net_pins = net[inet].num_net_pins;
    int ipin = 0;
    for (ipin = 1; ipin < knum_net_pins; ++ipin) {
        int ivex = net_rr_terminals[inet][ipin];
        ++(rr_node_route_inf[ivex].target_flag);
    }
}


void node_to_heap(int ivex, double cost, int prev_node, int prev_edge,
                  double backward_path_cost, double R_upstream)
{
    /* Puts an rr_node on the heap, if the new cost given is lower than the     *
     * current path_cost to this channel segment.  The index of its predecessor *
     * is stored to make traceback easy.  The index of the edge used to get     *
     * from its predecessor to it is also stored to make timing analysis, etc.  *
     * easy.  The backward_path_cost and R_upstream values are used only by the *
     * timing-driven router -- the breadth-first router ignores them.           */
    struct s_heap* hptr;

    if (cost >= rr_node_route_inf[ivex].path_cost) {
        return;
    }

    hptr = alloc_heap_data();
    hptr->index = ivex;
    hptr->cost = cost;
    hptr->u.prev_node = prev_node;
    hptr->prev_edge = prev_edge;
    hptr->backward_path_cost = backward_path_cost;
    hptr->R_upstream = R_upstream;
    add_to_heap(hptr);
}


void free_traceback(int inet)
{
    /* Puts the entire traceback (old routing) for this net on the free list *
     * and sets the trace_head pointers etc. for the net to NULL.            */
    struct s_trace* trace_node = trace_head[inet];
    while (trace_node != NULL) {
        /* Attention: before free trace_node, I must find its next node */
        struct s_trace* next_trace_node = trace_node->next;
        free_trace_data(trace_node);
        trace_node = next_trace_node;
    }

    trace_head[inet] = NULL;
    trace_tail[inet] = NULL;
}


vector_t** alloc_route_structs(subblock_data_t subblock_data)
{
    /* Allocates the data structures needed for routing.    */
    vector_t** clb_opins_used_locally;
    trace_head = (struct s_trace**) my_calloc(num_nets,
                                              sizeof(struct s_trace*));
    trace_tail = (struct s_trace**) my_malloc(num_nets *
                                              sizeof(struct s_trace*));
    heap_size = num_grid_columns * num_grid_rows;
    heap = (struct s_heap**) my_malloc(heap_size *
                                       sizeof(struct s_heap*));
    heap--;   /* heap stores from [1..heap_size] */
    heap_tail = 1;
    route_bb = (bbox_t*) my_malloc(num_nets * sizeof(bbox_t));
    clb_opins_used_locally = alloc_and_load_clb_opins_used_locally(
                                 subblock_data);
    return (clb_opins_used_locally);
}


struct s_trace** alloc_saved_routing(vector_t** clb_opins_used_locally,
                                     vector_t** *saved_clb_opins_used_locally_ptr) {

    /* Allocates data structures into which the key routing data can be saved,   *
     * allowing the routing to be recovered later (e.g. after a another routing  *
     * is attempted).                                                            */

    struct s_trace** best_routing;
    vector_t** saved_clb_opins_used_locally;
    int iblk, iclass, num_local_opins;

    best_routing = (struct s_trace**) my_calloc(num_nets,
                                                sizeof(struct s_trace*));

    saved_clb_opins_used_locally = (vector_t**) alloc_matrix(0, num_blocks - 1, 0,
                                                           num_pin_class - 1, sizeof(vector_t));

    for (iblk = 0; iblk < num_blocks; iblk++) {
        for (iclass = 0; iclass < num_pin_class; iclass++) {
            num_local_opins = clb_opins_used_locally[iblk][iclass].nelem;
            saved_clb_opins_used_locally[iblk][iclass].nelem = num_local_opins;

            if (num_local_opins == 0) {
                saved_clb_opins_used_locally[iblk][iclass].list = NULL;
            } else {
                saved_clb_opins_used_locally[iblk][iclass].list = (int*)
                                                                  my_malloc(num_local_opins * sizeof(int));
            }
        }
    }

    *saved_clb_opins_used_locally_ptr = saved_clb_opins_used_locally;
    return (best_routing);
}


static vector_t** alloc_and_load_clb_opins_used_locally(subblock_data_t
                                                      subblock_data)
{
    /* Allocates and loads the data needed to make the router reserve some CLB_TYPE  *
     * output pins for connections made locally within a CLB_TYPE (if the netlist    *
     * specifies that this is necessary).                                       */
    vector_t** clb_opins_used_locally;
    int* num_subblocks_per_block;
    subblock_t** subblock_inf;
    int iblk, isub, clb_pin, iclass, num_local_opins;
    clb_opins_used_locally = (vector_t**) alloc_matrix(0, num_blocks - 1, 0,
                                                     num_pin_class - 1, sizeof(vector_t));
    num_subblocks_per_block = subblock_data.num_subblocks_per_block;
    subblock_inf = subblock_data.subblock_inf;

    for (iblk = 0; iblk < num_blocks; ++iblk) {
        if (blocks[iblk].block_type != CLB_TYPE) {
            for (iclass = 0; iclass < num_pin_class; ++iclass) {
                clb_opins_used_locally[iblk][iclass].nelem = 0;
                clb_opins_used_locally[iblk][iclass].list = NULL;
            }
        } else { /* CLB_TYPE */
            for (iclass = 0; iclass < num_pin_class; iclass++) {
                clb_opins_used_locally[iblk][iclass].nelem = 0;
            }

            for (isub = 0; isub < num_subblocks_per_block[iblk]; isub++) {
                clb_pin = subblock_inf[iblk][isub].output;

                /* Subblock output used only locally, but must connect to a CLB_TYPE OPIN?  */
                if (clb_pin != OPEN && blocks[iblk].nets[clb_pin] == OPEN) {
                    iclass = clb_pin_class[clb_pin];
                    clb_opins_used_locally[iblk][iclass].nelem++;
                }
            }

            for (iclass = 0; iclass < num_pin_class; iclass++) {
                num_local_opins =
                    clb_opins_used_locally[iblk][iclass].nelem;

                if (num_local_opins == 0) {
                    clb_opins_used_locally[iblk][iclass].list = NULL;
                } else
                    clb_opins_used_locally[iblk][iclass].list =
                        (int*) my_malloc(num_local_opins * sizeof(int));
            }
        }
    }

    return (clb_opins_used_locally);
}

void free_trace_structs(void)
{
    /*the trace lists are only freed after use by the timing-driven placer*/
    /*Do not  free them after use by the router, since stats, and draw  */
    /*routines use the trace values */
    int i;

    for (i = 0; i < num_nets; i++) {
        free_traceback(i);
    }

    free(trace_head);
    free(trace_tail);
    trace_head = NULL;
    trace_tail = NULL;
}

void free_route_structs(vector_t** clb_opins_used_locally)
{
    /* Frees the temporary storage needed only during the routing.  The  *
     * final routing result is not freed.                                */
    free(heap + 1);
    free(route_bb);
    heap = NULL;         /* Defensive coding:  crash hard if I use these. */
    route_bb = NULL;
    free_ivec_matrix(clb_opins_used_locally, 0, num_blocks - 1, 0, num_pin_class - 1);
    /* NB:  Should use my chunk_malloc for tptr, hptr, and mod_ptr structures. *
     * I could free everything except the tptrs at the end then.               */
}


void free_saved_routing(struct s_trace** best_routing, vector_t
                        ** saved_clb_opins_used_locally)
{
    /* Frees the data structures needed to save a routing.                     */
    free(best_routing);
    free_ivec_matrix(saved_clb_opins_used_locally, 0, num_blocks - 1, 0,
                     num_pin_class - 1);
}

/* allocate and load routing_resource_node_route_information  */
void alloc_and_load_rr_node_route_structs(void)
{
    /* Allocates some extra information about each rr_node that is used only   *
     * during routing.                                                         */
    int ivex;

    if (rr_node_route_inf != NULL) {
        printf("Error in alloc_and_load_rr_node_route_structs:  \n"
               "old rr_node_route_inf array exists.\n");
        exit(1);
    }

    rr_node_route_inf = my_malloc(num_rr_nodes * sizeof(rr_node_t_route_inf));

    for (ivex = 0; ivex < num_rr_nodes; ivex++) {
        rr_node_route_inf[ivex].prev_node = NO_PREVIOUS;
        rr_node_route_inf[ivex].prev_edge = NO_PREVIOUS;
        rr_node_route_inf[ivex].pres_cost = 1.;
        rr_node_route_inf[ivex].acc_cost = 1.;
        rr_node_route_inf[ivex].path_cost = HUGE_FLOAT;
        rr_node_route_inf[ivex].target_flag = 0;
    }
}


void free_rr_node_route_structs(void)
{
    /* Frees the extra information about each rr_node that is needed only      *
     * during routing.                                                         */
    free(rr_node_route_inf);
    rr_node_route_inf = NULL;   /* Mark as free */
}

/* This routine loads the bounding box arrays used to limit the space  *
 * searched by the maze router when routing each net.  The search is   *
 * limited to channels contained with the net bounding box expanded    *
 * by bb_factor channels on each side.  For example, if bb_factor is   *
 * 0, the maze router must route each net within its bounding box.     *
 * If bb_factor = num_grid_columns, the maze router will search every  *
 * channel in the FPGA if necessary. The bounding boxes returned by    *
 * this routine are different from the ones used by the placer in that *
 * they are clipped to lie within (0,0) and (num_grid_columns+1,       *
 * num_grid_rows+1) rather than (1,1) and (num_grid_columns,num_grid_rows).*/
static void load_route_bb(int bb_factor)
{
    int k, xmax, ymax, xmin, ymin, x, y, inet;
    for (inet = 0; inet < num_nets; ++inet) {
        x = blocks[net[inet].node_blocks[0]].x;
        y = blocks[net[inet].node_blocks[0]].y;
        xmin = x;
        ymin = y;
        xmax = x;
        ymax = y;

        const int knum_net_pins = net[inet].num_net_pins;
        for (k = 1; k < knum_net_pins; ++k) {
            x = blocks[net[inet].node_blocks[k]].x;
            y = blocks[net[inet].node_blocks[k]].y;

            if (x < xmin) {
                xmin = x;
            } else if (x > xmax) {
                xmax = x;
            }

            if (y < ymin) {
                ymin = y;
            } else if (y > ymax) {
                ymax = y;
            }
        }

        /* Want the channels on all 4 sides to be usuable, even if bb_factor = 0. */
        xmin -= 1;
        ymin -= 1;
        /* Expand the net bounding box by bb_factor, then clip to the physical *
         * chip area.                                                          */
        route_bb[inet].xmin = max(xmin - bb_factor, 0);
        route_bb[inet].xmax = min(xmax + bb_factor, num_grid_columns + 1);
        route_bb[inet].ymin = max(ymin - bb_factor, 0);
        route_bb[inet].ymax = min(ymax + bb_factor, num_grid_rows + 1);
    }
}


void add_to_mod_list(double* fptr)
{
    /* This routine adds the doubleing point pointer (fptr) into a  *
     * linked list that indicates all the pathcosts that have been *
     * modified thus far.                                          */
    struct s_linked_f_pointer* mod_ptr;
    mod_ptr = alloc_linked_f_pointer();
    /* Add this element to the start of the modified list. */
    mod_ptr->next = rr_modified_head;
    mod_ptr->fptr = fptr;
    rr_modified_head = mod_ptr;
}


static void add_to_heap(struct s_heap* hptr)
{
    /* Adds an item to the heap, expanding the heap if necessary.             */
    int ito, ifrom;
    struct s_heap* temp_ptr;

    if (heap_tail > heap_size) {           /* Heap is full */
        heap_size *= 2;
        heap = my_realloc((void*)(heap + 1), heap_size *
                          sizeof(struct s_heap*));
        heap--;     /* heap goes from [1..heap_size] */
    }

    heap[heap_tail] = hptr;
    ifrom = heap_tail;
    ito = ifrom / 2;
    heap_tail++;

    while ((ito >= 1) && (heap[ifrom]->cost < heap[ito]->cost)) {
        temp_ptr = heap[ito];
        heap[ito] = heap[ifrom];
        heap[ifrom] = temp_ptr;
        ifrom = ito;
        ito = ifrom / 2;
    }
}


struct s_heap* get_heap_head(void) {

    /* Returns a pointer to the smallest element on the heap, or NULL if the     *
     * heap is empty.  Invalid (index == OPEN) entries on the heap are never     *
     * returned -- they are just skipped over.                                   */

    int ito, ifrom;
    struct s_heap* heap_head, *temp_ptr;

    do {
        if (heap_tail == 1) {    /* Empty heap. */
            printf("Empty heap occurred in get_heap_head.\n");
            printf("Some blocks are impossible to connect in this architecture.\n");
            return (NULL);
        }

        heap_head = heap[1];                /* Smallest element. */
        /* Now fix up the heap */
        heap_tail--;
        heap[1] = heap[heap_tail];
        ifrom = 1;
        ito = 2 * ifrom;

        while (ito < heap_tail) {
            if (heap[ito + 1]->cost < heap[ito]->cost) {
                ito++;
            }

            if (heap[ito]->cost > heap[ifrom]->cost) {
                break;
            }

            temp_ptr = heap[ito];
            heap[ito] = heap[ifrom];
            heap[ifrom] = temp_ptr;
            ifrom = ito;
            ito = 2 * ifrom;
        }
    } while (heap_head->index == OPEN);  /* Get another one if invalid entry. */

    return(heap_head);
}


void empty_heap(void)
{
    int i;

    for (i = 1; i < heap_tail; i++) {
        free_heap_data(heap[i]);
    }

    heap_tail = 1;
}

#define NCHUNK 200  /* # of various structs malloced at a time. */


static struct s_heap* alloc_heap_data(void) {

    int i;
    struct s_heap* temp_ptr;

    if (heap_free_head == NULL) {   /* No elements on the free list */
        heap_free_head = (struct s_heap*) my_malloc(NCHUNK *
                                                    sizeof(struct s_heap));

        /* If I want to free this memory, I have to store the original pointer *
         * somewhere.  Not worthwhile right now -- if you need more memory     *
         * for post-routing stages, look into it.                              */

        for (i = 0; i < NCHUNK - 1; i++) {
            (heap_free_head + i)->u.next = heap_free_head + i + 1;
        }

        (heap_free_head + NCHUNK - 1)->u.next = NULL;
    }

    temp_ptr = heap_free_head;
    heap_free_head = heap_free_head->u.next;
#ifdef DEBUG
    num_heap_allocated++;
#endif
    return (temp_ptr);
}


void free_heap_data(struct s_heap* hptr)
{
    hptr->u.next = heap_free_head;
    heap_free_head = hptr;
#ifdef DEBUG
    num_heap_allocated--;
#endif
}


void invalidate_heap_entries(int sink_node, int ipin_node)
{
    /* Marks all the heap entries consisting of sink_node, where it was reached *
     * via ipin_node, as invalid (OPEN).  Used only by the breadth_first router *
     * and even then only in rare circumstances.                                */
    int i;

    for (i = 1; i < heap_tail; i++) {
        if (heap[i]->index == sink_node && heap[i]->u.prev_node == ipin_node) {
            heap[i]->index = OPEN;    /* Invalid. */
        }
    }
}


static struct s_trace* alloc_trace_data(void) {

    int i;
    struct s_trace* temp_ptr;

    if (trace_free_head == NULL) {   /* No elements on the free list */
        trace_free_head = (struct s_trace*) my_malloc(NCHUNK *
                                                      sizeof(struct s_trace));

        /* If I want to free this memory, I have to store the original pointer *
         * somewhere.  Not worthwhile right now -- if you need more memory     *
         * for post-routing stages, look into it.                              */

        for (i = 0; i < NCHUNK - 1; i++) {
            (trace_free_head + i)->next = trace_free_head + i + 1;
        }

        (trace_free_head + NCHUNK - 1)->next = NULL;
    }

    temp_ptr = trace_free_head;
    trace_free_head = trace_free_head->next;
#ifdef DEBUG
    num_trace_allocated++;
#endif
    return (temp_ptr);
}


static void free_trace_data(struct s_trace* tptr)
{
    /* Puts the traceback structure pointed to by tptr on the free list. */
    tptr->next = trace_free_head;
    trace_free_head = tptr;
#ifdef DEBUG
    num_trace_allocated--;
#endif
}


static struct s_linked_f_pointer* alloc_linked_f_pointer(void) {

    /* This routine returns a linked list element with a double pointer as *
     * the node data.                                                     */

    int i;
    struct s_linked_f_pointer* temp_ptr;

    if (linked_f_pointer_free_head == NULL) {
        /* No elements on the free list */
        linked_f_pointer_free_head = (struct s_linked_f_pointer*)
                                     my_malloc(NCHUNK * sizeof(struct s_linked_f_pointer));

        /* If I want to free this memory, I have to store the original pointer *
         * somewhere.  Not worthwhile right now -- if you need more memory     *
         * for post-routing stages, look into it.                              */

        for (i = 0; i < NCHUNK - 1; i++) {
            (linked_f_pointer_free_head + i)->next = linked_f_pointer_free_head
                                                     + i + 1;
        }

        (linked_f_pointer_free_head + NCHUNK - 1)->next = NULL;
    }

    temp_ptr = linked_f_pointer_free_head;
    linked_f_pointer_free_head = linked_f_pointer_free_head->next;

#ifdef DEBUG
    num_linked_f_pointer_allocated++;
#endif

    return (temp_ptr);
}


void print_route(char* route_file)
{
    /* Prints out the routing to file route_file.  */
    int inet, ivex, ipin, block_num, ilow, jlow, blk_pin, iclass;
    rr_types_t rr_type;
    struct s_trace* tptr;
    char* name_type[] = {"SOURCE", "SINK", "IPIN", "OPIN", "CHANX", "CHANY"};
    FILE* fp = my_fopen(route_file, "w");
    fprintf(fp, "Array size: %d x %d logic blocks.\n", num_grid_columns, num_grid_rows);
    fprintf(fp, "\nRouting:");

    for (inet = 0; inet < num_nets; inet++) {
        if (is_global[inet] == FALSE) {
            fprintf(fp, "\n\nNet %d (%s)\n\n", inet, net[inet].name);
            tptr = trace_head[inet];

            while (tptr != NULL) {
                ivex = tptr->index;
                rr_type = rr_node[ivex].type;
                ilow = rr_node[ivex].xlow;
                jlow = rr_node[ivex].ylow;
                fprintf(fp, "%6s (%d,%d) ", name_type[rr_type], ilow, jlow);

                if ((ilow != rr_node[ivex].xhigh) || (jlow !=
                                                       rr_node[ivex].yhigh))
                    fprintf(fp, "to (%d,%d) ", rr_node[ivex].xhigh,
                            rr_node[ivex].yhigh);

                switch (rr_type) {
                    case IPIN:
                    case OPIN:
                        if (clb_grids[ilow][jlow].grid_type == CLB_TYPE) {
                            fprintf(fp, " Pin: ");
                        } else { /* IO_TYPE Pad. */
                            fprintf(fp, " Pad: ");
                        }

                        break;

                    case CHANX:
                    case CHANY:
                        fprintf(fp, " Track: ");
                        break;

                    case SOURCE:
                    case SINK:
                        if (clb_grids[ilow][jlow].grid_type == CLB_TYPE) {
                            fprintf(fp, " Class: ");
                        } else { /* IO_TYPE Pad. */
                            fprintf(fp, " Pad: ");
                        }

                        break;

                    default:
                        printf("Error in print_route:  Unexpected traceback element "
                               "type: %d (%s).\n", rr_type, name_type[rr_type]);
                        exit(1);
                        break;
                }

                fprintf(fp, "%d  ", rr_node[ivex].ptc_num);
                /* Uncomment line below if you're debugging and want to see the switch types *
                 * used in the routing.                                                      */
                /*          fprintf (fp, "Switch: %d", tptr->iswitch);    */
                fprintf(fp, "\n");
                tptr = tptr->next;
            }
        } else {  /* Global net.  Never routed. */
            fprintf(fp, "\n\nNet %d (%s): global net connecting:\n\n", inet,
                    net[inet].name);

            const int knum_net_pins = net[inet].num_net_pins;
            for (ipin = 0; ipin < knum_net_pins; ++ipin) {
                block_num = net[inet].node_blocks[ipin];

                if (blocks[block_num].block_type == CLB_TYPE) {
                    blk_pin = net[inet].node_block_pins[ipin];
                    iclass = clb_pin_class[blk_pin];
                } else {            /* IO_TYPE pad */
                    iclass = OPEN;   /* Class not relevant */
                }

                fprintf(fp, "Block %s (#%d) at (%d, %d), Pin class %d.\n",
                        blocks[block_num].name, block_num, blocks[block_num].x, blocks[block_num].y,
                        iclass);
            }
        }
    }

    fclose(fp);
#ifdef DEBUG
    fp = my_fopen("mem.echo", "w");
    fprintf(fp, "\nNum_heap_allocated: %d   Num_trace_allocated: %d\n",
            num_heap_allocated, num_trace_allocated);
    fprintf(fp, "Num_linked_f_pointer_allocated: %d\n",
            num_linked_f_pointer_allocated);
    fclose(fp);
#endif
}


void reserve_locally_used_opins(double pres_fac, boolean rip_up_local_opins,
                                vector_t** clb_opins_used_locally)
{
    /* If some subblock outputs are hooked directly to CLB_TYPE outputs, then      *
     * some CLB_TYPE outputs are occupied if their associated subblock is used     *
     * locally, even though the inter-CLB_TYPE netlist does not say those outputs  *
     * have to connect to anything.  This is important when you have          *
     * logically equivalent outputs.  Code below makes sure any CLB_TYPE outputs   *
     * that are used by being directly hooked to subblocks get properly       *
     * reserved.                                                              */
    int iblk, num_local_opin, ivex, from_node, iconn, num_edges, to_node;
    int iclass, ipin;
    double cost;
    struct s_heap* heap_head_ptr;

    if (rip_up_local_opins) {
        for (iblk = 0; iblk < num_blocks; iblk++) {
            for (iclass = 0; iclass < num_pin_class; iclass++) {
                num_local_opin = clb_opins_used_locally[iblk][iclass].nelem;

                /* Always 0 for pads and for RECEIVER (IPIN) classes */
                for (ipin = 0; ipin < num_local_opin; ipin++) {
                    ivex = clb_opins_used_locally[iblk][iclass].list[ipin];
                    adjust_one_rr_occ_and_pcost(ivex, -1, pres_fac);
                }
            }
        }
    }

    for (iblk = 0; iblk < num_blocks; iblk++) {
        for (iclass = 0; iclass < num_pin_class; iclass++) {
            num_local_opin = clb_opins_used_locally[iblk][iclass].nelem;
            /* Always 0 for pads and for RECEIVER (IPIN) classes */

            if (num_local_opin != 0) {  /* Have to reserve (use) some OPINs */
                from_node = rr_clb_source[iblk][iclass];
                num_edges = rr_node[from_node].num_edges;

                for (iconn = 0; iconn < num_edges; iconn++) {
                    to_node = rr_node[from_node].edges[iconn];
                    cost = get_rr_cong_cost(to_node);
                    node_to_heap(to_node, cost, OPEN, OPEN, 0., 0.);
                }

                for (ipin = 0; ipin < num_local_opin; ipin++) {
                    heap_head_ptr = get_heap_head();
                    ivex = heap_head_ptr->index;
                    adjust_one_rr_occ_and_pcost(ivex, 1, pres_fac);
                    clb_opins_used_locally[iblk][iclass].list[ipin] = ivex;
                    free_heap_data(heap_head_ptr);
                }

                empty_heap();
            }
        }
    }
}


static void adjust_one_rr_occ_and_pcost(int ivex, int add_or_sub, double
                                        pres_fac)
{
    /* Increments or decrements (depending on add_or_sub) the occupancy of    *
     * one rr_node, and adjusts the present cost of that node appropriately.  */
    int m_usage, capacity;
    m_usage = rr_node[ivex].m_usage + add_or_sub;
    capacity = rr_node[ivex].capacity;
    rr_node[ivex].m_usage = m_usage;

    if (m_usage < capacity) {
        rr_node_route_inf[ivex].pres_cost = 1.;
    } else {
        rr_node_route_inf[ivex].pres_cost = 1. + (m_usage + 1 - capacity) *
                                             pres_fac;
    }
}
