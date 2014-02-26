#ifndef PATH_DELAY2_H
#define PATH_DELAY2_H

#include <stdio.h>
#include "util.h"

/*********** Types and defines used by all path_delay modules ****************/
/* to_node: index of node at the sink end of this edge.                      *
 * Tdel: delay to go to to_node along this edge.                             */
typedef struct s_edge_t {
    int to_node;
    double Tdel;
} edge_t;

/* out_edges: [0..num_edges - 1].  Array of the edges leaving this tnode.    *
 * num_edges: Number of edges leaving this node.                             *
 * arr_time:  Arrival time of the last input signal to this node.               *
 * req_time:  Required arrival time of the last input signal to this node if    *
 *         the critical path is not to be lengthened.                        */
typedef struct {
    edge_t* out_edges;
    int     num_edges;
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

/* type:  What is this tnode? (Pad pin, clb pin, subblock pin, etc.)         *
 * ipin:  Number of the CLB or subblock pin this tnode represents, if        *
 *        applicable.                                                        *
 * isubblk: Number of the subblock this tnode is part of, if applicable.     *
 * iblk:  Number of the block (CLB or PAD) this tnode is part of.            */
typedef struct {
    vertex_type_t type;
    short ipin;
    short isubblk;
    int iblk;
} vertex_descript;

/*************** Variables shared only amongst path_delay modules ************/
extern vertex_t* tnode;                    /* [0..num_tnodes - 1] */
extern vertex_descript* tnode_descript;  /* [0..num_tnodes - 1] */

extern int num_tnodes; /* Number of nodes in the timing graph */

/* [0..num_nets-1]. Gives the index of the tnode that drives each net. */
extern int* net_to_driver_tnode;

/* [0..num__tnode_levels - 1].  Count and list of tnodes at each level of    *
 * the timing graph, to make breadth-first searches easier.                  */
extern struct s_ivec* tnodes_at_level;
extern int num_tnode_levels; /* Number of levels in the timing graph. */


/***************** Subroutines exported by this module ***********************/
int alloc_and_load_timing_graph_levels(void);

void check_timing_graph(int num_const_gen, int num_ff, int num_sinks);

double print_critical_path_node(FILE* fp, t_linked_int* critical_path_node);

#endif

