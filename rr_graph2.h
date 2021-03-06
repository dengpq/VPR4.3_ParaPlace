#ifndef RR_GRAPH2_H
#define RR_GRAPH2_H

#include "vpr_types.h"
#include "rr_graph_util.h"


/************** Global variables shared only by the rr_* modules. ************/
extern boolean* rr_edge_done; /* [0..num_rr_nodes-1].  Used to keep track  *
                               * of whether or not a node has been put in  *
                               * an edge list yet.                         */


extern t_linked_edge* free_edge_list_head; /* Start of linked list of free   *
                                            * edge_list elements (for speed) */


/******************* Subroutines exported by rr_graph2.c *********************/
int** alloc_and_load_rr_node_indices(int nodes_per_clb,
                                     int nodes_per_pad, int nodes_per_chan,
                                     segment_details_t* seg_details_x,
                                     segment_details_t* seg_details_y);

void free_rr_node_indices(int** rr_node_indices);

int gerr_node_t_index(int i_in, int j_in, rr_types_t rr_type, int ioff,
                      int nodes_per_chan, int** rr_node_indices);

void free_seg_details(segment_details_t* seg_details, int nodes_per_chan);

segment_details_t* alloc_and_load_seg_details(int nodes_per_chan,
                                          segment_info_t*  segment_inf,
                                          int num_seg_types, int max_len);

void dump_seg_details(segment_details_t* seg_details, int nodes_per_chan, char* fname);

int get_closest_seg_start(segment_details_t* seg_details, int itrack, int seg_num,
                          int chan_num);

int get_seg_end(segment_details_t* seg_details, int itrack, int seg_start, int
                chan_num, int max_end);

boolean is_cbox(int seg_num, int chan_num, int itrack, segment_details_t
                *seg_details);

boolean is_sbox(int seg_num, int chan_num, int itrack, segment_details_t
                *seg_details, boolean above_or_right);


int get_clb_opin_connections(int***  clb_opin_to_tracks, int ipin,
                             int i, int j, int Fc_output,
                             segment_details_t* seg_details_x, segment_details_t* seg_details_y,
                             t_linked_edge** edge_list_ptr, int nodes_per_chan,
                             int** rr_node_indices);

int get_pad_opin_connections(int** pads_to_tracks, int ipad, int i, int j,
                             int Fc_pad, segment_details_t* seg_details_x, segment_details_t* seg_details_y,
                             t_linked_edge** edge_list_ptr, int nodes_per_chan, int
                             ** rr_node_indices);

int get_xtrack_to_clb_ipin_edges(int tr_istart, int tr_iend, int tr_j,
                                 int itrack, int iside, t_linked_edge** edge_list_ptr, vector_t
                                 ** tracks_to_clb_ipin, int nodes_per_chan, int** rr_node_indices,
                                 segment_details_t* seg_details_x, int wire_to_ipin_switch);

int get_ytrack_to_clb_ipin_edges(int tr_jstart, int tr_jend, int tr_i,
                                 int itrack, int iside, t_linked_edge** edge_list_ptr, vector_t
                                 ** tracks_to_clb_ipin, int nodes_per_chan, int** rr_node_indices,
                                 segment_details_t* seg_details_y, int wire_to_ipin_switch);

int get_xtrack_to_pad_edges(int tr_istart, int tr_iend, int tr_j, int pad_j,
                            int itrack, t_linked_edge** edge_list_ptr, vector_t
                            *tracks_to_pads, int nodes_per_chan, int** rr_node_indices,
                            segment_details_t* seg_details_x, int wire_to_ipin_switch);

int get_ytrack_to_pad_edges(int tr_jstart, int tr_jend, int tr_i, int pad_i,
                            int itrack, t_linked_edge** edge_list_ptr, vector_t
                            *tracks_to_pads, int nodes_per_chan, int** rr_node_indices,
                            segment_details_t* seg_details_y, int wire_to_ipin_switch);

int get_xtrack_to_ytracks(int from_istart, int from_iend, int from_j, int
                          from_track, int to_j, t_linked_edge** edge_list_ptr, int nodes_per_chan,
                          int** rr_node_indices, segment_details_t* seg_details_x, segment_details_t
                          *seg_details_y, switch_block_t switch_block_type);

int get_ytrack_to_xtracks(int from_jstart, int from_jend, int from_i, int
                          from_track, int to_i, t_linked_edge** edge_list_ptr, int nodes_per_chan,
                          int** rr_node_indices, segment_details_t* seg_details_x, segment_details_t
                          *seg_details_y, switch_block_t switch_block_type);

int get_xtrack_to_xtrack(int from_i, int j, int from_track, int to_i,
                         t_linked_edge** edge_list_ptr, int nodes_per_chan, int** rr_node_indices,
                         segment_details_t* seg_details_x, switch_block_t
                         switch_block_type);

int get_ytrack_to_ytrack(int i, int from_j, int from_track, int to_j,
                         t_linked_edge** edge_list_ptr, int nodes_per_chan, int** rr_node_indices,
                         segment_details_t* seg_details_y, switch_block_t
                         switch_block_type);

#endif

