#ifndef GLOBALS_H
#define GLOBALS_H

#include "vpr_types.h"

/* Netlist to be placed stuff. */
/* default was 4 types: CLB_TYPE, OUTPAD_TYPE, INPAD_TYPE, IO_TYPE and EMPTY_TYPE */
extern int  g_num_grid_types;
/* int g_grid_capacity[g_num_grid_types]; CLB_TYPE default was 1, IO_TYPE was *
 * io_ratio, EMPTY_TYPE was 0. */
extern int* g_grid_capacity;

extern net_t*   net;
extern int num_nets;   /* total nets in current netlist */
extern boolean* is_global;
extern int num_globals;


extern block_t* blocks;
extern int num_blocks; /* total blocks in current netlist */
extern int num_clbs;
extern int num_primary_inputs; /* maybe num_of_primary_inputs */
extern int num_primary_outputs; /* maybe num_of_primary_outputs */


/* Physical FPGA architecture stuff */
extern int  num_grid_columns, num_grid_rows, io_ratio, max_pins_per_clb;
extern int** pinloc;
extern int* clb_pin_class;
extern boolean* is_global_clb_pin;
extern pin_class_t* class_inf;
extern int num_pin_class;

/* chan_width_x is the x-directed channel; i.e. between rows */
extern int* chan_width_x;
extern int* chan_width_y; /* numerical form */


extern grid_tile_t** bin_grids;

/* [0..num_nets-1] of linked list start pointers.  Defines the routing.  */
extern struct s_trace**  trace_head;
extern struct s_trace**  trace_tail;

/* Structures to define the routing architecture of the FPGA.           */
extern int num_rr_nodes;
extern rr_node_t* rr_node; /* [0..num_rr_nodes-1] */
extern int num_rr_indexed_data;
extern rr_indexed_data_t* rr_indexed_data;   /* [0 .. num_rr_indexed_data-1] */
extern int** net_rr_terminals;          /* [0..num_nets-1][0..num_pins-1] */
extern switch_info_t* switch_inf; /* [0..det_routing_arch.num_switch-1] */
extern int** rr_clb_source;             /* [0..num_blocks-1][0..num_pin_class-1] */

extern int* g_pins_on_block;

#endif

