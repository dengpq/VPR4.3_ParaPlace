#include <stdio.h>
#include "util.h"
#include "vpr_types.h"
#include "globals.h"
#include "net_delay.h"


/***************** Types and defines local to this module ********************/

struct s_linked_rc_edge {
    struct s_rc_node* child;
    short iswitch;
    struct
            s_linked_rc_edge* next;
};

typedef struct s_linked_rc_edge t_linked_rc_edge;

/* Linked list listing the children of an rc_node.                           *
 * child:  Pointer to an rc_node (child of the current node).                *
 * iswitch:  Index of the switch type used to connect to the child node.     *
 * next:   Pointer to the next linked_rc_edge in the linked list (allows     *
 *         you to get the next child of the current rc_node).                */



struct s_rc_node {
    union {
        t_linked_rc_edge* child_list;
        struct s_rc_node
                *next;
    } u;
    int ivex;
    double C_downstream;
    double Tdel;
};

typedef struct s_rc_node t_rc_node;

/* Structure describing one node in an RC tree (used to get net Tdels).     *
 * u.child_list:  Pointer to a linked list of linked_rc_edge.  Each one of   *
 *                the linked list entries gives a child of this node.        *
 * u.next:  Used only when this node is on the free list.  Gives the next    *
 *          node on the free list.                                           *
 * ivex:  index (ID) of the rr_node that corresponds to this rc_node.       *
 * C_downstream:  Total downstream capacitance from this rc_node.  That is,  *
 *                the total C of the subtree rooted at the current node,     *
 *                including the C of the current node.                       *
 * Tdel:  Time Tdel for the signal to get from the net source to this node. *
 *        Includes the time to go through this node.                         */


struct s_linked_rc_ptr {
    struct s_rc_node* rc_node;
    struct s_linked_rc_ptr
            *next;
};

typedef struct s_linked_rc_ptr t_linked_rc_ptr;

/* Linked list of pointers to rc_nodes.                                      *
 * rc_node:  Pointer to an rc_node.                                          *
 * next:  Next list element.                                                 */



/*********************** Subroutines local to this module ********************/

static t_rc_node* alloc_and_load_rc_tree(int inet,
                                         t_rc_node** rc_node_free_list_ptr,
                                         t_linked_rc_edge** rc_edge_free_list_ptr,
                                         t_linked_rc_ptr* rr_node_to_rc_node);

static void add_to_rc_tree(t_rc_node* parent_rc,
                           t_rc_node* child_rc,
                           short iswitch,
                           int ivex,
                           t_linked_rc_edge** rc_edge_free_list_ptr);

static t_rc_node* alloc_rc_node(t_rc_node** rc_node_free_list_ptr);

static void free_rc_node(t_rc_node* rc_node,
                         t_rc_node** rc_node_free_list_ptr);

static t_linked_rc_edge* alloc_linked_rc_edge(t_linked_rc_edge** rc_edge_free_list_ptr);

static void free_linked_rc_edge(t_linked_rc_edge* rc_edge,
                                t_linked_rc_edge** rc_edge_free_list_ptr);

static double load_rc_tree_C(t_rc_node* rc_node);

static void load_rc_tree_T(t_rc_node* rc_node,
                           double T_arrival);

static void load_one_net_delay(double** net_delay,
                               int inet,
                               t_linked_rc_ptr* rr_node_to_rc_node);

static void load_one_constant_net_delay(double** net_delay,
                                        int inet,
                                        double delay_value);

static void free_rc_tree(t_rc_node* rc_root,
                         t_rc_node** rc_node_free_list_ptr,
                         t_linked_rc_edge** rc_edge_free_list_ptr);

static void reserr_node_t_to_rc_node(t_linked_rc_ptr* rr_node_to_rc_node,
                                     int inet);

static void free_rc_node_free_list(t_rc_node* rc_node_free_list);

static void free_rc_edge_free_list(t_linked_rc_edge* rc_edge_free_list);

/********************* Subroutine definitions ********************
 * double net_delay[0...num_nets-1][1...num_pins-1] was used for *
 * TIMING_DRIVEN_PLACE *
 ****************************************************************/
double** alloc_net_delay(linked_vptr_t** chunk_list_head_ptr)
{
    /* Allocates space for the net_delay data structure                          *
     * [0..num_nets-1][1..num_pins-1].  I chunk the data to save space on large  *
     * problems.                                                                 */
    int chunk_bytes_avail = 0;
    char* chunk_next_avail_mem = NULL;
    *chunk_list_head_ptr = NULL;
    /* double** net_delay[0..num_nets-1][1..num_pins-1] */
    double** net_delay = (double**)my_malloc(num_nets * sizeof(double*));

    int inet = -1;
    for (inet = 0; inet < num_nets; ++inet) {
        double* tmp_ptr = (double*)my_chunk_malloc((net[inet].num_pins-1) * sizeof(double),
                                                   chunk_list_head_ptr,
                                                   &chunk_bytes_avail,
                                                   &chunk_next_avail_mem);
        net_delay[inet] = tmp_ptr - 1; /* it only need [1..num_pins-1] */
    }

    return net_delay;
}

void free_net_delay(double** net_delay,
                    linked_vptr_t** chunk_list_head_ptr)
{
    /* Frees the net_delay structure.  Assumes it was chunk allocated.*/
    free(net_delay);
    free_chunk_memory(*chunk_list_head_ptr);
    *chunk_list_head_ptr = NULL;
}

void load_net_delay_from_routing(double** net_delay)
{
    /* This routine loads net_delay[0..num_nets-1][1..num_pins-1].  Each entry   *
     * is the Elmore Tdel from the net source to the appropriate sink.  Both    *
     * the rr_graph and the routing traceback must be completely constructed     *
     * before this routine is called, and the net_delay array must have been     *
     * allocated.                                                                */
    t_rc_node* rc_node_free_list, *rc_root;
    t_linked_rc_edge* rc_edge_free_list;
    int inet;
    t_linked_rc_ptr* rr_node_to_rc_node;          /* [0..num_rr_nodes-1]  */
    rr_node_to_rc_node = (t_linked_rc_ptr*) my_calloc(num_rr_nodes,
                                                      sizeof(t_linked_rc_ptr));
    rc_node_free_list = NULL;
    rc_edge_free_list = NULL;

    for (inet = 0; inet < num_nets; inet++) {
        if (is_global[inet]) {
            load_one_constant_net_delay(net_delay, inet, 0.);
        } else {
            rc_root = alloc_and_load_rc_tree(inet, &rc_node_free_list,
                                             &rc_edge_free_list, rr_node_to_rc_node);
            load_rc_tree_C(rc_root);
            load_rc_tree_T(rc_root, 0.);
            load_one_net_delay(net_delay, inet, rr_node_to_rc_node);
            free_rc_tree(rc_root, &rc_node_free_list, &rc_edge_free_list);
            reserr_node_t_to_rc_node(rr_node_to_rc_node, inet);
        }
    }

    free_rc_node_free_list(rc_node_free_list);
    free_rc_edge_free_list(rc_edge_free_list);
    free(rr_node_to_rc_node);
}


/* Loads the net_delay array with delay_value for every <source, sink> connection *
 * that is not on a global net, and with 0 for every <source, sink> connection on *
 * a global net.(This can be used to allow timing analysis before routing is done *
 * with a constant net Tdel model).                                              */
void load_constant_net_delay(double** net_delay,
                             double delay_value)
{
    /* TODO: Why did VPR set all subnets(or connections) had same net Tdel value *
     *       in NET_TIMING_DRIVEN_PLACE?                                          */
    int inet;
    for (inet = 0; inet < num_nets; ++inet) {
        if (is_global[inet]) {
            load_one_constant_net_delay(net_delay,
                                        inet,
                                        0.0);
        } else {
            load_one_constant_net_delay(net_delay,
                                        inet,
                                        delay_value);
        }
    }
}


static t_rc_node* alloc_and_load_rc_tree(int inet, t_rc_node** rc_node_free_list_ptr,
                                         t_linked_rc_edge** rc_edge_free_list_ptr,
                                         t_linked_rc_ptr* rr_node_to_rc_node)
{
    /* Builds a tree describing the routing of net inet.  Allocates all the data *
     * and inserts all the connections in the tree.                              */
    t_rc_node* curr_rc, *prev_rc;
    t_linked_rc_ptr* linked_rc_ptr;
    t_rc_node* root_rc = alloc_rc_node(rc_node_free_list_ptr);

    struct s_trace* tptr = trace_head[inet];
    if (tptr == NULL) {
        printf("Error in alloc_and_load_rc_tree:  Traceback for net %d doesn't "
               "exist.\n", inet);
        exit(1);
    }

    int ivex = tptr->index;
    short iswitch = tptr->iswitch;
    root_rc->ivex = ivex;
    root_rc->u.child_list = NULL;
    rr_node_to_rc_node[ivex].rc_node = root_rc;
    prev_rc = root_rc;
    tptr = tptr->next;

    while (tptr != NULL) {
        ivex = tptr->index;

        /* Is this node a "stitch-in" point to part of the existing routing or a   *
         * new piece of routing along the current routing "arm?"                   */

        if (rr_node_to_rc_node[ivex].rc_node == NULL) { /* Part of current "arm" */
            curr_rc = alloc_rc_node(rc_node_free_list_ptr);
            add_to_rc_tree(prev_rc, curr_rc, iswitch, ivex,
                           rc_edge_free_list_ptr);
            rr_node_to_rc_node[ivex].rc_node = curr_rc;
            prev_rc = curr_rc;
        } else if (rr_node[ivex].type != SINK) { /* Connection to old stuff. */
#ifdef DEBUG
            int prev_node = prev_rc->ivex;
            if (rr_node[prev_node].type != SINK) {
                printf("Error in alloc_and_load_rc_tree:  Routing of net %d is "
                       "not a tree.\n", inet);
                exit(1);
            }

#endif
            prev_rc = rr_node_to_rc_node[ivex].rc_node;
        } else {        /* SINK that this net has connected to more than once. */
            /* I can connect to a SINK node more than once in some weird architectures. *
             * That means the routing isn't really a tree -- there is reconvergent      *
             * fanout from two or more IPINs into one SINK.  I convert this structure   *
             * into a true RC tree on the fly by creating a new rc_node each time I hit *
             * the same sink.  This means I need to keep a linked list of the rc_nodes  *
             * associated with the rr_node (ivex) associated with that SINK.           */
            curr_rc = alloc_rc_node(rc_node_free_list_ptr);
            add_to_rc_tree(prev_rc, curr_rc, iswitch, ivex,
                           rc_edge_free_list_ptr);
            linked_rc_ptr = (t_linked_rc_ptr*) my_malloc(sizeof(t_linked_rc_ptr));
            linked_rc_ptr->next = rr_node_to_rc_node[ivex].next;
            rr_node_to_rc_node[ivex].next = linked_rc_ptr;
            linked_rc_ptr->rc_node = curr_rc;
            prev_rc = curr_rc;
        }

        iswitch = tptr->iswitch;
        tptr = tptr->next;
    }

    return (root_rc);
}


static void add_to_rc_tree(t_rc_node* parent_rc, t_rc_node* child_rc, short
                           iswitch, int ivex, t_linked_rc_edge** rc_edge_free_list_ptr)
{
    /* Adds child_rc to the child list of parent_rc, and sets the switch between *
     * them to iswitch.  This routine also intitializes the child_rc properly    *
     * and sets its node value to ivex.                                         */
    t_linked_rc_edge* linked_rc_edge;
    linked_rc_edge = alloc_linked_rc_edge(rc_edge_free_list_ptr);
    linked_rc_edge->next = parent_rc->u.child_list;
    parent_rc->u.child_list = linked_rc_edge;
    linked_rc_edge->child = child_rc;
    linked_rc_edge->iswitch = iswitch;
    child_rc->u.child_list = NULL;
    child_rc->ivex = ivex;
}


static t_rc_node* alloc_rc_node(t_rc_node** rc_node_free_list_ptr)
{
    /* Allocates a new rc_node, from the free list if possible, from the free   *
     * store otherwise.                                                         */
    t_rc_node* rc_node;
    rc_node = *rc_node_free_list_ptr;

    if (rc_node != NULL) {
        *rc_node_free_list_ptr = rc_node->u.next;
    } else {
        rc_node = (t_rc_node*) my_malloc(sizeof(t_rc_node));
    }

    return (rc_node);
}


static void free_rc_node(t_rc_node* rc_node, t_rc_node
                         ** rc_node_free_list_ptr)
{
    /* Adds rc_node to the proper free list.          */
    rc_node->u.next = *rc_node_free_list_ptr;
    *rc_node_free_list_ptr = rc_node;
}


static t_linked_rc_edge* alloc_linked_rc_edge(t_linked_rc_edge
                                              ** rc_edge_free_list_ptr)
{
    /* Allocates a new linked_rc_edge, from the free list if possible, from the  *
     * free store otherwise.                                                     */
    t_linked_rc_edge* linked_rc_edge;
    linked_rc_edge = *rc_edge_free_list_ptr;

    if (linked_rc_edge != NULL) {
        *rc_edge_free_list_ptr = linked_rc_edge->next;
    } else {
        linked_rc_edge = (t_linked_rc_edge*) my_malloc(sizeof
                                                       (t_linked_rc_edge));
    }

    return (linked_rc_edge);
}


static void free_linked_rc_edge(t_linked_rc_edge* rc_edge, t_linked_rc_edge
                                ** rc_edge_free_list_ptr)
{
    /* Adds the rc_edge to the rc_edge free list.                       */
    rc_edge->next = *rc_edge_free_list_ptr;
    *rc_edge_free_list_ptr = rc_edge;
}


static double load_rc_tree_C(t_rc_node* rc_node)
{
    /* Does a post-order traversal of the rc tree to load each node's           *
     * C_downstream with the proper sum of all the downstream capacitances.     *
     * This routine calls itself recursively to perform the traversal.          */
    t_linked_rc_edge* linked_rc_edge;
    t_rc_node* child_node;
    int ivex;
    short iswitch;
    double C, C_downstream;
    linked_rc_edge = rc_node->u.child_list;
    ivex = rc_node->ivex;
    C = rr_node[ivex].C;

    while (linked_rc_edge != NULL) {            /* For all children */
        iswitch = linked_rc_edge->iswitch;
        child_node = linked_rc_edge->child;
        C_downstream = load_rc_tree_C(child_node);

        if (switch_inf[iswitch].buffered == FALSE) {
            C += C_downstream;
        }

        linked_rc_edge = linked_rc_edge->next;
    }

    rc_node->C_downstream = C;
    return (C);
}


static void load_rc_tree_T(t_rc_node* rc_node, double T_arrival)
{
    /* This routine does a pre-order depth-first traversal of the rc tree to    *
     * compute the Tdel to each node in the rc tree.  The T_arrival is the time *
     * at which the signal hits the input to this node.  This routine calls     *
     * itself recursively to perform the traversal.                             */
    double Tdel, Rmetal, Tchild;
    t_linked_rc_edge* linked_rc_edge;
    t_rc_node* child_node;
    short iswitch;
    int ivex;
    Tdel = T_arrival;
    ivex = rc_node->ivex;
    Rmetal = rr_node[ivex].R;
    /* NB:  rr_node[ivex].C gives the capacitance of this node, while          *
     * rc_node->C_downstream gives the unbuffered downstream capacitance rooted *
     * at this node, including the C of the node itself.  I want to multiply    *
     * the C of this node by 0.5 Rmetal, since it's a distributed RC line.      *
     * Hence 0.5 Rmetal * Cnode is a pessimistic estimate of Tdel (i.e. end to *
     * end).  For the downstream capacitance rooted at this node (not including *
     * the capacitance of the node itself), I assume it is, on average,         *
     * connected halfway along the line, so I also multiply by 0.5 Rmetal.  To  *
     * be totally pessimistic I would multiply the downstream part of the       *
     * capacitance by Rmetal.  Play with this equation if you like.             */
    /* Rmetal is distributed so x0.5 */
    Tdel += 0.5 * rc_node->C_downstream * Rmetal;
    rc_node->Tdel = Tdel;
    /* Now expand the children of this node to load their Tdel values.       */
    linked_rc_edge = rc_node->u.child_list;

    while (linked_rc_edge != NULL) {          /* For all children */
        iswitch = linked_rc_edge->iswitch;
        child_node = linked_rc_edge->child;
        Tchild = Tdel + switch_inf[iswitch].R * child_node->C_downstream;
        Tchild += switch_inf[iswitch].Tdel;   /* Intrinsic switch Tdel. */
        load_rc_tree_T(child_node, Tchild);
        linked_rc_edge = linked_rc_edge->next;
    }
}


static void load_one_net_delay(double** net_delay, int inet, t_linked_rc_ptr
                               *rr_node_to_rc_node)
{
    /* Loads the net Tdel array for net inet.  The rc tree for that net must  *
     * have already been completely built and loaded.                          */
    int ipin, ivex;
    double Tmax;
    t_rc_node* rc_node;
    t_linked_rc_ptr* linked_rc_ptr, *next_ptr;

    for (ipin = 1; ipin < net[inet].num_pins; ipin++) {
        ivex = net_rr_terminals[inet][ipin];
        linked_rc_ptr = rr_node_to_rc_node[ivex].next;
        rc_node = rr_node_to_rc_node[ivex].rc_node;
        Tmax = rc_node->Tdel;

        /* If below only executes when one net connects several times to the      *
         * same SINK.  In this case, I can't tell which net pin each connection   *
         * to this SINK corresponds to (I can just choose arbitrarily).  To make  *
         * sure the timing behaviour converges, I pessimistically set the Tdel   *
         * for all of the connections to this SINK by this net to be the max. of  *
         * the Tdels from this net to this SINK.  NB:  This code only occurs     *
         * when a net connect more than once to the same pin class on the same    *
         * logic blocks.  Only a weird architecture would allow this.              */

        if (linked_rc_ptr != NULL) {
            /* The first time I hit a multiply-used SINK, I choose the largest Tdel  *
             * from this net to this SINK and use it for every connection to this     *
             * SINK by this net.                                                      */
            do {
                rc_node = linked_rc_ptr->rc_node;

                if (rc_node->Tdel > Tmax) {
                    Tmax = rc_node->Tdel;
                    rr_node_to_rc_node[ivex].rc_node = rc_node;
                }

                next_ptr = linked_rc_ptr->next;
                free(linked_rc_ptr);
                linked_rc_ptr = next_ptr;
            } while (linked_rc_ptr != NULL);   /* End do while */

            rr_node_to_rc_node[ivex].next = NULL;
        }   /* End of if multiply-used SINK */

        net_delay[inet][ipin] = Tmax;
    }
}


static void load_one_constant_net_delay(double** net_delay, int inet, double
                                        delay_value)
{
    /* Sets each entry of the net_delay array for net inet to delay_value. */
    int ipin;
    /* TODO: why did all subnets(or connections) set same dealy_value? */
    /* This is used to NET_TIMING_DRIVEN_PLACE. */
    for (ipin = 1; ipin < net[inet].num_pins; ++ipin) {
        net_delay[inet][ipin] = delay_value;
    }
}


static void free_rc_tree(t_rc_node* rc_root, t_rc_node** rc_node_free_list_ptr,
                         t_linked_rc_edge** rc_edge_free_list_ptr)
{
    /* Puts the rc tree pointed to by rc_root back on the free list.  Depth-     *
     * first post-order traversal via recursion.                                 */
    t_rc_node* rc_node, *child_node;
    t_linked_rc_edge* rc_edge, *next_edge;
    rc_node = rc_root;
    rc_edge = rc_node->u.child_list;

    while (rc_edge != NULL) {     /* For all children */
        child_node = rc_edge->child;
        free_rc_tree(child_node, rc_node_free_list_ptr, rc_edge_free_list_ptr);
        next_edge = rc_edge->next;
        free_linked_rc_edge(rc_edge, rc_edge_free_list_ptr);
        rc_edge = next_edge;
    }

    free_rc_node(rc_node, rc_node_free_list_ptr);
}


static void reserr_node_t_to_rc_node(t_linked_rc_ptr* rr_node_to_rc_node, int
                                     inet)
{
    /* Resets the rr_node_to_rc_node mapping entries that were set during       *
     * construction of the RC tree for net inet.  Any extra linked list entries *
     * added to deal with a SINK being connected to multiple times have already *
     * been freed by load_one_net_delay.                                        */
    struct s_trace* tptr;
    int ivex;
    tptr = trace_head[inet];

    while (tptr != NULL) {
        ivex = tptr->index;
        rr_node_to_rc_node[ivex].rc_node = NULL;
        tptr = tptr->next;
    }
}


static void free_rc_node_free_list(t_rc_node* rc_node_free_list)
{
    /* Really frees (i.e. calls free()) all the rc_nodes on the free list.   */
    t_rc_node* rc_node, *next_node;
    rc_node = rc_node_free_list;

    while (rc_node != NULL) {
        next_node = rc_node->u.next;
        free(rc_node);
        rc_node = next_node;
    }
}

static void free_rc_edge_free_list(t_linked_rc_edge* rc_edge_free_list)
{
    /* Really frees (i.e. calls free()) all the rc_edges on the free list.   */
    t_linked_rc_edge* rc_edge, *next_edge;
    rc_edge = rc_edge_free_list;

    while (rc_edge != NULL) {
        next_edge = rc_edge->next;
        free(rc_edge);
        rc_edge = next_edge;
    }
}


void print_net_delay(double** net_delay, char* fname)
{
    /* Dumps the net Tdels into file fname.   */
    FILE* fp;
    int inet, ipin;
    fp = my_fopen(fname, "w", 0);

    for (inet = 0; inet < num_nets; inet++) {
        fprintf(fp, "Net: %d.\n", inet);
        fprintf(fp, "Delays:");

        for (ipin = 1; ipin < net[inet].num_pins; ipin++) {
            fprintf(fp, " %g", net_delay[inet][ipin]);
        }

        fprintf(fp, "\n\n");
    }

    fclose(fp);
}
