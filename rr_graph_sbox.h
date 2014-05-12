#ifndef RR_GRAPH_SBOX_H
#define RR_GRAPH_SBOX_H

#include "vpr_types.h"

vector_t get_switch_box_tracks(int from_i, int from_j, int from_track,
                                    rr_types_t from_type,
                                    int to_i, int to_j, rr_types_t to_type,
                                    switch_block_t switch_block_type,
                                    int nodes_per_chan);

void free_switch_block_conn(int nodes_per_chan);

void alloc_and_load_switch_block_conn(int nodes_per_chan,
                                      switch_block_t switch_block_type);

#endif

