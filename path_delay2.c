#include "util.h"
#include "vpr_types.h"
#include "globals.h"
#include "path_delay2.h"


/************* Variables (globals) shared by all path_delay modules **********/
vertex_t* vertexes;  /* all vertexes in Timing-Analyze-Graph, [0...num_of_vertexs-1] */
int num_of_vertexs; /* Number of nodes (pins) in the timing graph */

vertex_descript* tnode_descript; /* [0...num_of_vertexs-1] */


/* [0..num_nets-1].  Gives the index of the vertex that driver pin of each net. */
int* driver_node_index_of_net;


/* [0..num_tnode_levels-1]. Count and list of tnodes at each level of the *
 * Timing_Analyze_Graph, to make breadth-first searches easier.           */
vector_t* tnodes_at_level;
int       num_tnode_levels; /* Number of levels in the timing graph. */


/******************* Subroutines local to this module ************************/
static int* alloc_and_load_tnode_fanin_and_check_edges(int* num_sinks_ptr);


/************************** Subroutine definitions ***************************/
static int* alloc_and_load_tnode_fanin_and_check_edges(int* num_sinks_ptr)
{
    /* Allocates an array and fills it with the number of in-edges (inputs) to   *
     * each vertexes.  While doing this it also checks that each edge in the timing *
     * graph points to a valid vertexes. Also counts the number of sinks.           */
    int* all_vertexs_fanin_value = (int*)my_calloc(num_of_vertexs,
                                                   sizeof(int));
    int error = 0;
    int num_sinks = 0;

    int ivex = -1;
    for (ivex = 0; ivex < num_of_vertexs; ++ivex) {
        int num_out_edges = vertexes[ivex].num_out_edges;
        if (num_out_edges > 0) {
            edge_t* tedge = vertexes[ivex].out_edges;

            int iedge = 0;
            for (iedge = 0; iedge < num_out_edges; ++iedge) {
                int to_node = tedge[iedge].to_node;
                if (to_node < 0 || to_node >= num_of_vertexs) {
                    printf("Error in alloc_and_load_tnode_fanin_and_check_edges:\n"
                           "vertexes #%d edge #%d goes to illegal node #%d.\n", ivex,
                           iedge, to_node);
                    ++error;
                }

                ++all_vertexs_fanin_value[to_node];
            }
        } else if (num_out_edges == 0) {
            ++num_sinks;
        } else {
            printf("Error in alloc_and_load_tnode_fanin_and_check_edges: \n"
                   "vertexes #%d has %d edges.\n", ivex, num_out_edges);
            ++error;
        }
    }

    if (error != 0) {
        printf("Found %d Errors in the timing graph.  Aborting.\n", error);
        exit(1);
    }
    *num_sinks_ptr = num_sinks;

    return all_vertexs_fanin_value;
} /* end of static int* alloc_and_load_tnode_fanin_and_check_edges() */


/* FIXME: allocate and load timing_graph edges's level.                    *
 * Do a breadth-first search through the timing graph in order to levelize *
 * it. This allows subsequent breadth-first traversals to be faster. Also  *
 * returns the number of sinks in the graph(nodes with no fanout).         */
int alloc_and_load_timing_graph_edge_levels(void)
{
    int num_sinks = -1;

    /* [0..num_of_vertexs-1]. This function calc all vertexes' fanin value and *
     * return the number of sink nodes(fanout = 0).                           */
    int* tnode_fanin_left = alloc_and_load_tnode_fanin_and_check_edges(&num_sinks);
    /* Very conservative -> max number of levels = num_of_vertexs.  Realloc later.*
     * Temporarily need one extra level on the end because I look at the first  *
     * empty level.                                                             */
    tnodes_at_level = (vector_t*)my_malloc((num_of_vertexs + 1) *
                                                 sizeof(vector_t));
    /* Scan through the timing graph, putting all the primary input nodes (no *
     * fanin) into level 0 of the level structure.                           */
    t_linked_int* free_list_head = NULL;
    t_linked_int* nodes_at_level_head = NULL;
    int zero_fanin_at_level = 0;
    int from_node = 0;
    /* fanin = 0, they were Priamry_Inputs */
    for (from_node = 0; from_node < num_of_vertexs; ++from_node) {
        if (tnode_fanin_left[from_node] == 0) {
            ++zero_fanin_at_level;
            nodes_at_level_head = insert_in_int_list(nodes_at_level_head,
                                                     from_node,
                                                     &free_list_head);
        }
    }
    /* copy all primary_input nodes from nodes_at_level_head to *
     * tnodes_at_level[0].                                      */
    alloc_ivector_and_copy_int_list(&nodes_at_level_head,
                                    zero_fanin_at_level,
                                    &tnodes_at_level[0],
                                    &free_list_head);
    int num_levels = 0;
    while (zero_fanin_at_level != 0) { /* Until there's nothing in the queue. */
        ++num_levels;
        zero_fanin_at_level = 0;

        int i = 0;
        for (i = 0; i < tnodes_at_level[num_levels - 1].nelem; ++i) {
            from_node = tnodes_at_level[num_levels - 1].list[i];
            edge_t* tedge = vertexes[from_node].out_edges;
            int num_out_edges = vertexes[from_node].num_out_edges;

            int iedge = 0;
            for (iedge = 0; iedge < num_out_edges; ++iedge) {
                int to_node = tedge[iedge].to_node;
                --(tnode_fanin_left[to_node]);

                if (tnode_fanin_left[to_node] == 0) {
                    ++zero_fanin_at_level;  /* FIXME */
                    nodes_at_level_head = insert_in_int_list(nodes_at_level_head,
                                                             to_node,
                                                             &free_list_head);
                }
            } /* end of traverse all edges */
        } /* end of for(i = 0; i < tnodes_at_level[num_levels-1].nelem; ++i) */

        alloc_ivector_and_copy_int_list(&nodes_at_level_head,
                                        zero_fanin_at_level,
                                        &tnodes_at_level[num_levels],
                                        &free_list_head);
    } /* end of while (zero_fanin_at_level != 0) */

    tnodes_at_level = (vector_t*)my_realloc(tnodes_at_level,
                                             num_levels * sizeof(vector_t));
    num_tnode_levels = num_levels;

    free(tnode_fanin_left);
    free_int_list(&free_list_head);

    return num_sinks;
} /* end of int alloc_and_load_timing_graph_edge_levels() */

/* Checks the timing graph to see that: (1) all the tnodes have been put    *
 * into some level of the timing graph; (2) the number of primary inputs    *
 * to the timing graph is equal to the number of input pads + the number of *
 * constant generators; and (3) the number of sinks (nodes with no fanout)  *
 * equals the number of output pads + the number of flip-flops.             */
void check_timing_graph(int num_const_gen,
                        int num_ff,
                        int num_sinks)
{
    int error = 0;
    int num_tnodes_check = 0;
    printf("In check_timing_graph: total_vertexs = %d, num_const_gen: %d, num_ff: %d, num_sinks: %d.\n",
           num_of_vertexs,
           num_const_gen,
           num_ff,
           num_sinks);

    int ilevel = -1;
    for (ilevel = 0; ilevel < num_tnode_levels; ++ilevel) {
        num_tnodes_check += tnodes_at_level[ilevel].nelem;
    }

    if (num_tnodes_check != num_of_vertexs) {
        printf("Error in check_timing_graph: %d tnodes appear in the vertexes level "
               "structure.  Expected %d.\n", num_tnodes_check, num_of_vertexs);
        printf("Check the netlist for combinational cycles.\n");
        ++error;
    }
    /* What's CONST_GEN? */
    if (num_const_gen + num_primary_inputs != tnodes_at_level[0].nelem) {
        printf("Error in check_timing_graph: %d tnodes are sources (have no "
               "inputs -- expected %d.\n",
               tnodes_at_level[0].nelem,
               num_const_gen + num_primary_inputs);
        ++error;
    }

    if (num_sinks != num_primary_outputs + num_ff) {
        printf("Error in check_timing_graph: %d tnodes are sinks (have no "
               "outputs -- expected %d.\n",
               num_sinks,
               num_ff + num_primary_outputs);
        ++error;
    }

    int inet = -1;
    for (inet = 0; inet < num_nets; ++inet) {
        int ivex = driver_node_index_of_net[inet];
        if (ivex < 0 || ivex >= num_of_vertexs) {
            printf("Error in check_timing_graph:\n"
                   "\tdriver of net %d has a vertexes mapping of %d (out of range).\n",
                   inet,
                   ivex);
            ++error;
        }
    }

    if (error != 0) {
        printf("Found %d Errors in the timing graph.  Aborting.\n", error);
        exit(1);
    }
}

double print_critical_path_node(FILE* fp,
                                t_linked_int* critical_path_node)
{
    /* Prints one vertexes on the critical path out to fp. Returns the Tdel to    *
     * the next node.                                                           */
    static char* tnode_type_names[] = {"INPAD_SOURCE", "INPAD_OPIN",
                                       "OUTPAD_IPIN", "OUTPAD_SINK",
                                       "CLB_IPIN", "CLB_OPIN",
                                       "SUBBLK_IPIN", "SUBBLK_OPIN",
                                       "FF_SINK", "FF_SOURCE",
                                       "CONSTANT_GEN_SOURCE"
                                      };  /* NB: static for speed */
    int ivex = critical_path_node->data;
    int iblk = tnode_descript[ivex].iblk;
    int ipin = tnode_descript[ivex].ipin;
    vertex_type_t type = tnode_descript[ivex].type;
    fprintf(fp, "Node: %d  %s Block #%d (%s)\n", ivex, tnode_type_names[type],
            iblk, blocks[iblk].name);

    if (type != INPAD_SOURCE && type != OUTPAD_SINK && type != FF_SINK &&
            type != FF_SOURCE && type != CONSTANT_GEN_SOURCE) {
        fprintf(fp, "Pin: %d ", ipin);
    }

    if (type == SUBBLK_IPIN || type == SUBBLK_OPIN || type == FF_SINK
          || type == FF_SOURCE || type == CONSTANT_GEN_SOURCE) {
        fprintf(fp, "Subblock #%d ", tnode_descript[ivex].isubblk);
    }

    if (type != INPAD_SOURCE && type != OUTPAD_SINK) {
        fprintf(fp, "\n");
    }

    fprintf(fp, "arr_time: %g  req_time: %g  ",
            vertexes[ivex].arr_time,
            vertexes[ivex].req_time);
    t_linked_int* next_crit_node = critical_path_node->next;

    double Tdel = 0.0;
    int downstream_node = 0;
    if (next_crit_node != NULL) {
        downstream_node = next_crit_node->data;
        Tdel = vertexes[downstream_node].arr_time - vertexes[ivex].arr_time;
        fprintf(fp, "Tdel: %g\n", Tdel);
    } else { /* last node, no Tdel. */
        Tdel = 0.0;
        fprintf(fp, "\n");
    }

    int inet = 0;
    if (type == INPAD_OPIN || type == CLB_OPIN) {
        inet = blocks[iblk].nets[ipin];
        fprintf(fp, "Net to next node: #%d (%s).  Pins on net: %d.\n", inet,
                net[inet].name, net[inet].num_net_pins);
    }

    fprintf(fp, "\n");
    return (Tdel);
}

