#ifndef PATH_DELAY2_H
#define PATH_DELAY2_H

#include <stdio.h>
#include "util.h"

/*********** Types and defines used by all path_delay modules ***************/
/* to_node: index of node at the sink end of this edge.                     *
 * Tdel: Tdel to go to to_node along this edge.                             */
typedef struct s_edge_t {
    /* int from_node */
    int    to_node;
    double Tdel; /* delay value in this edge(subnet) */
} edge_t; /* FIXME */

/* out_edges: [0..num_edges - 1].  Array of the edges leaving this vertexes.    *
 * num_edges: Number of edges leaving this node.                             *
 * arr_time:  Arrival time of the last input signal to this node.            *
 * req_time:  Required arrival time of the last input signal to this node if *
 *         the critical path is not to be lengthened.                        */
typedef struct s_vertex_t {
    edge_t* out_edges;
    int     num_edges; /* num_of_out_edges */

    /* int  global_netlist_pin_index; */
    /* which net this pin belonged to *
     * int  global_netlist_net_index; */

    double  arr_time; /* arrival_time */
    double  req_time; /* required_time */
} vertex_t;

/* Info. below is only used to print out and display the critical path.  It  *
 * gives a mapping from each t_node to what circuit element it represents.   *
 * I put this info in a separate structure to maximize cache effectiveness,  *
 * since it's not used much.                                                 */
typedef enum s_vertex_type_t {
    INPAD_SOURCE,
    INPAD_OPIN,

    OUTPAD_IPIN,
    OUTPAD_SINK,

    CLB_IPIN,
    CLB_OPIN,

    SUBBLK_IPIN,
    SUBBLK_OPIN,

    FF_SINK,
    FF_SOURCE,

    CONSTANT_GEN_SOURCE
} vertex_type_t;

/* type:  What is this vertexes? (Pad pin, clb pin, subblock pin, etc.)         *
 * ipin:  Number of the CLB_TYPE or subblock pin this vertexes represents, if        *
 *        applicable.                                                        *
 * isubblk: Number of the subblock this vertexes is part of, if applicable.     *
 * iblk:  Number of the blocks (CLB_TYPE or PAD) this vertexes is part of.            */
typedef struct {
    vertex_type_t type;
    short  ipin;
    short  isubblk;
    int    iblk;
} vertex_descript;

/*************** Variables shared only amongst path_delay modules ************/
extern vertex_t* vertexes;                  /* [0..num_of_vertexs - 1] */
extern vertex_descript* tnode_descript;  /* [0..num_of_vertexs - 1] */

extern int num_of_vertexs; /* Number of nodes in the timing graph */

/* [0..num_nets-1]. Gives the index of the vertexes that drives each net. */
extern int* driver_node_index_of_net;

/* [0..num_tnode_levels - 1].  Count and list of tnodes at each level of    *
 * the timing graph, to make breadth-first searches easier.                  */
extern vector_t* tnodes_at_level;
extern int num_tnode_levels; /* Number of levels in the timing graph. */

/* the following 2 arraries was used for calc_timing_driven_costs_by_path_algo() *
 * weight(i,j) = front_crit_path_pin(i) * behind_crit_path_through_pin(j) *      *
 * discount_value(net_slack[inet][ipin], T_crit).                                *
 * ipin was sink pin index, but the where the source pin?                        *
 * when I assign front_crit_path_through_pin = NULL, gcc reported that redefine  *
 * front_crit_path_through_pin and behind_crit_path_through_pin. Why?            */
double*  front_crit_path_through_pin;  /* [0...num_of_vertexs-1] */
double*  behind_crit_path_through_pin; /* [ivex] */

/***************** Subroutines exported by this module ***********************/
int alloc_and_load_timing_graph_edge_levels(void);

void check_timing_graph(int num_const_gen,
                        int num_ff,
                        int num_sinks);

double print_critical_path_node(FILE* fp,
                                t_linked_int* critical_path_node);

#endif

