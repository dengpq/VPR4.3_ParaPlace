/* Do not forget to add head file protector */
#ifndef VPR_TYPES_H
#define VPR_TYPES_H

#include "util.h"
/* #define DEBUG 1 */ /* Echoes input & checks error conditions */
/* Only causes about a 1% speed degradation in V 3.10 */
#ifdef SPEC
#define NO_GRAPHICS    /* Rips out graphics (for non-X11 systems)      */
#define NDEBUG         /* Turns off assertion checking for extra speed */
#endif

#define TOKENS " \t\n"   /* Input file parsing. */
/*#define VERBOSE 1*/      /* Prints all sorts of intermediate data */

/* I will use 4, 6, 8 to test the speed of VPR4.3_parallel */
#define NUM_OF_THREADS  8

/* Block Types in input netlist file */
typedef enum e_block_types {
    CLB_TYPE,
    OUTPAD_TYPE,
    INPAD_TYPE,
    IO_TYPE,
    ILLEGAL_TYPE
} block_types_t;

/* Gives the Tdels through a subblock.                                    *
 * T_comb: The Tdel from input to output when the subblock is used in     *
 *         combinational mode (clock input is open).                       *
 * T_seq_in: The Tdel from subblock input to storage element when the     *
 *           subblock is in sequential mode (clock input not open).  The   *
 *           Tdel includes any combinational logic Tdel (e.g. a LUT)     *
 *           done before the signal is latched, plus the setup time.       *
 * T_seq_out: The Tdel from storage element to subblock output when the   *
 *            subblock is in sequential mode.  Includes clock_to_Q plus    *
 *            any combinational path (muxes, etc.) on the output.          */
typedef struct {
    double T_comb;
    double T_seq_in;
    double T_seq_out;
} T_subblock_t;


/* C_ipin_cblock: Capacitance added to a routing track by the isolation     *
 *                buffer between a track and the Cblocks at an (i,j) loc.   *
 * T_ipin_cblock: Delay through an input pin connection box (from a         *
 *                   routing track to a logic blocks input pin).             *
 * T_sblk_opin_to_sblk_ipin: Delay through the local interconnect(mux,      *
 *       wires or whatever) in a clb containing multiple subblocks. That is,*
 *       the Tdel from a subblock output to the input of another subblock  *
 *       in the same clb.                                                   *
 * T_clb_ipin_to_sblk_ipin: Delay from a clb input pin to any subblock      *
 *                   input pin (e.g. the mux Tdel in an Altera 8K clb).    *
 * T_sblk_opin_to_clb_opin: Delay from a subblock output to a clb output.   *
 *                   Will be 0 in many architectures.                       *
 * T_ipad:  Delay through an input pad.                                     *
 * T_opad:  Delay through an output pad.                                    *
 * *T_subblock: Array giving the Tdel through each subblock.               *
 *              [0..max_subblocks_per_block - 1]                            */
typedef struct s_timing_inf {
    boolean  timing_analysis_enabled;
    double   C_ipin_cblock;
    double   T_ipin_cblock;

    double   T_sblk_opin_to_sblk_ipin;
    double   T_clb_ipin_to_sblk_ipin;
    double   T_sblk_opin_to_clb_opin;

    double   T_ipad;
    double   T_opad;
    T_subblock_t* T_subblock;
} timing_info_t;


/* Pin is unconnected, driving a net or in the fanout, respectively. */
typedef enum e_pin_type {
    OPEN = -1,
    DRIVER = 0,
    RECEIVER = 1
} pin_types_t;

typedef enum e_side {
    TOP = 0,
    BOTTOM = 1,
    LEFT = 2,
    RIGHT = 3
} side_types_t;

#define MINOR 0  /* For update_screen.  Denotes importance of update. */
#define MAJOR 1

#define HUGE_FLOAT 1.e30

/* Want to avoid overflows of shorts.  OPINs can have edges to 4 * width if  *
 * they are on all 4 sides, so set MAX_CHANNEL_WIDTH to 8000.                */
#define MAX_CHANNEL_WIDTH 8000
#define MAX_SHORT 32767

/* Annealing schedule type, default value was AUTO_SCHED */
typedef enum sched_type {
    AUTO_SCHED,
    USER_SCHED
} sched_types_t;

/* What's on screen? */
enum pic_type {
    NO_PICTURE,
    PLACEMENT,
    ROUTING
};

/* For the placer. Different types of cost functions that can be used. */
/* FIXME: both LINEAR_CONG and NONLINEAR_CONG were used to Wirelength-Driven  *
 *        placement, not used to TIMING_DRIVEN_PLACE. The default setting was *
 *        LINEAR_CONG.                                                        */
typedef enum place_c_types { /* congestion_type */
    LINEAR_CONG,
    NONLINEAR_CONG
} place_cong_types_t;

typedef enum e_operation {
    PLACE_AND_ROUTE,
    PLACE_ONLY,
    ROUTE_ONLY,
    TIMING_ANALYSIS_ONLY
} operation_types_t;

typedef enum pfreq {
    PLACE_NEVER,
    PLACE_ONCE,
    PLACE_ALWAYS
} place_freq_t;

/* Are the pads free to be moved, locked in a random configuration, or *
 * locked in user-specified positions?                                 */
typedef enum e_pad_loc_type {
    FREE,
    RANDOM,
    USER
} pad_loc_t;

/* FIXME: Data Structure about Net in circuit netlist.                       */
/* name:  ASCII net name for informative annotations in the output.          *
 * num_pins:  Number of pins on this net.                                    *
 * blocks: [0..num_pins-1]. Contains the block_index to which the pins of    *
 *         this net connect. Output in pins[0], inputs in other entries.     *
 * blk_pin: [0..num_pins-1]. Contains the pin_index number(on a blocks) to wh *
 *          -ich each net terminal connects. Since I/O pads have only one pin*
 *          , I set blk_pin to OPEN for them (it should only be used for clb *
 *          pins). For clbs, it is the blocks pin number, as expected.        */
typedef struct s_net {
    char* name;
    int   num_pins;
    int*  blocks;
    int*  blk_pin;
} net_t;

/* FIXME: Data Structure about clbs or io pads in circuit netlist.   */
/* name:  Taken from the net which it drives.                        *
 * type:  CLB_TYPE, INPAD_TYPE or OUTPAD_TYPE                                       *
 * nets[]:  List of nets connected to this blocks.  If nets[i] = OPEN *
            no net is connected to pin i.                            *
 * x,y:  physical location of the placed blocks.                      */
typedef struct s_block {
    char* name;
    enum  e_block_types type; /* CLB_TYPE,INPAD_TYPE,OUTPAD_TYPE,IO_TYPE,ILLEGAL */
    int*  nets; /* [0..pins_per_clb-1] */
    int   x;
    int   y;
} block_t;

/* FIXME: Data Structure about clb in FPGA chip architecture.        */
/* type: CLB_TYPE, IO_TYPE or ILLEGAL.                                         *
 * occ:  number of logical blocks in this physical group.            *
 * u.blocks: number of the blocks occupying this group if it is a CLB_TYPE. *
 * u.io_blocks[]: numbers of other blocks occupying groups (for      *
 *                IO_TYPE's), up to u.io_blocks[occ-1]                   */
typedef struct s_clb {
    block_types_t type;
    int  occ;
    union {
        int   blocks;
        int*  io_blocks;
    } u;
} clb_t;

typedef enum e_grid_loc_type {
    BOUNDARY = 0,
    FILL,
    COL_REPEAT,
    COL_REL
} grid_loc_type_t;

/* Define how to place type in the grid *
 * grid_loc_type - where the type gones and which numbers are valid;
 * start_col - the absolute value of the starting column from the left to fill,
 *             used with COL_REPEAT;
 * repeat - the number of columns to skip before placing the same type, used with
 *          COL_REPEAT. 0 means do not repeat;
 * rel_col - the fraction column to place type;
 * priority - in the event of confict, which type gets picked.     */
typedef struct s_grid_loc_def {
    grid_loc_type_t grid_loc_type;
    int    start_col;
    int    repeat;
    double  col_rel;
    int    priority;
} grid_loc_def_t;

typedef  struct s_type_descriptor {
    const char*   m_name;
    int     m_num_pins;
    int     m_capacity;
    int     m_height;
    int***  m_pinloc; /* [0..height-1][0..3][0..num_pins-1] */

    int     m_num_class;
    struct s_pin_class*  m_class_info; /* [0..num_class-1] */
    int*    m_pin_class;               /* [0..num_pins-1] */

    boolean*  is_global_pin;  /* [0..num_pins-1] */

    boolean   is_Fc_factor;
    boolean   is_Fc_out_full_flex;
    double    m_Fc_in;
    double    m_Fc_out;

    /* subblock info */
    int  m_max_subblocks;
    int  m_max_subblocks_inputs;
    int  m_max_subblocks_outputs;

    /* grid_location info */
    struct s_grid_loc_def* m_grid_loc_def;
    int    m_num_grid_loc_def;

    /* timing info */
    timing_info_t  m_timing_inf;
    
    /* This can be determinded from class_info and pin_class_t but store for
     * fast access */
    int  m_num_drivers;
    int  m_num_receivers;

    /* index of type_descriptor in array(allow for index reference) */
    int  m_index;
} type_descriptor_t;
typedef const type_descriptor_t*  block_type_ptr;

/* the grid_tile_t was similar with clb_t */
typedef struct s_grid_tile {
    block_type_ptr  m_type;
    int   m_offset;
    int   m_usage;
    int*  m_blocks;
} grid_tile_t; /* clb_t */

/* Stores the bounding box of a net in terms of the minimum and  *
 * maximum coordinates of the blocks forming the net, clipped to *
 * the region (1..num_grid_columns, 1..num_grid_rows).                                    */
typedef struct s_bb {
    int xmin;
    int xmax;
    int ymin;
    int ymax;
} bbox_t;

enum e_stat {
    UNIFORM,
    GAUSSIAN,
    PULSE,
    DELTA
};

/* Width is standard deviation for Gaussian. xpeak is where peak *
 * occurs. dc is the dc offset for Gaussian and pulse waveforms. */
typedef struct t_chan {
    enum e_stat type;
    double      peak;
    double      width;
    double      xpeak;
    double      dc;
} channel_t;

/* FIXME: Data Structure about Channel Width Distribution.                */
/* chan_width_io: The relative width of the I/O channel between the pads  *
 *                and logic array.                                        *
 * chan_x_dist: Describes the x-directed channel width distribution.      *
 * chan_y_dist: Describes the y-directed channel width distribution.      */
typedef struct s_chan_width_distr {
    double chan_width_io;
    channel_t chan_x_dist;
    channel_t chan_y_dist;
} chan_width_distr_t;

/* type: OPEN, DRIVER or RECEIVER (What is this pinclass?)               *
 * num_pins: The number of logically equivalent pins forming this class *
 * pinlist[]: List of clb pin numbers which belong to this class.       */
typedef struct s_pin_class {
    enum  e_pin_type type;
    int   num_pins;
    int*  pinlist;
} pin_class_t;

/* capacity:   Capacity of this region, in tracks.               *
 * occupancy:  Expected number of tracks that will be occupied.  *
 * cost:       Current cost of this usage.                       */
typedef struct s_place_region_t {
    double capacity;
    double inv_capacity;
    double occupancy;
    double cost;
} place_region_t;

/* This structure stores the contents of each logic blocks, in terms    *
 * of the basic LUTs that make up the cluster. This information is     *
 * used only for Timing-Analysis.  Note that it is possible to         *
 * describe essentially arbitrary timing patterns inside a logic       *
 * blocks via the correct pattern of LUTs.                              *
 * name:    Name of this subblock.                                     *
 * output:  Number of the clb pin which the LUT output drives, or OPEN *
 * clock:   Number of clb pin that drives the clock (or OPEN)          *
 * inputs:  [0..sub_block_lut_size-1]. Number of clb_pin that drives  *
 *          this input, or (num_of_subblock_output + pins_per_clb) if *
 *          this pin is driven by a subblock output, or OPEN if unused.*/
typedef struct s_subblock_t {
    char* name;
    int   output; /* driver pin index */
    int   clock;  /* driver pin index */
    int*  inputs; /* [0..sub_block_lut_size-1] */
} subblock_t;

/* This structure contains All the Information Relevant to Subblocks(what's *
 * in each logic blocks). This makes it easy to pass around the subblock     *
 * data all at once. This stuff is used only for Timing-Analysis.           *
 * subblock_inf: [0..num_blocks-1][0..num_subblock_per_block[iblk]-1].      *
 *               Contents of each logic blocks. Not valid for IO_TYPE blocks.     *
 * num_subblocks_per_block: [0..num_blocks-1]. Number of subblocks in each  *
 *                  blocks. 0 for IOs, between 1 and max_subblocks_per_block *
 *                  for CLBs.                                               *
 * max_subblocks_per_block: Maximum number of subblocks any logic blocks can *
 *                          have.                                           *
 * subblock_lut_size:  Number of inputs to each subblock LUT.               *
 * num_ff:  Number of flip flops in the input netlist (i.e. clocked sblks). *
 * num_const_gen:  Number of subblock constant generators in the netlist.   *
 * chunk_head_ptr: Points to the start of the list of memory chunks used for *
 *                 subblock storage.  Needed to free the subblock data.     */
typedef struct {
    subblock_t** subblock_inf; /* [num_blocks][num_subblock_per_block[iblk]] */
    int  max_subblocks_per_block;
    int* num_subblocks_per_block; /* int num_subblocks_per_block[0...num_blocks-1] */
    int  subblock_lut_size; /* input pins of subblock, that is, LUT-K. */
    int  num_ff; /* number of flip-flops in the input circuit netlist */
    int  num_const_gen;
    linked_vptr_t* chunk_head_ptr;
} subblock_data_t;

/* Annealing Schedule Information for the placer. The schedule type  *
 * is either USER_SCHED or AUTO_SCHED. Inner_num is multiplied by    *
 * num_blocks^4/3 to find the number of moves per temperature.  The  *
 * remaining information is used only for USER_SCHED, and have the   *
 * obvious meanings.                                                 */
typedef struct s_annealing_sched {
    sched_types_t type;
    double inner_num; /* used for controlling the move times in SA inner loop*/
    double init_t;  /* initial_temperature */
    double alpha_t; /* factor about descrease the temperature. */
    double exit_t;  /* exit_temperature */
} annealing_sched_t;

/*****************************************************************************
 * New added PATH_ALGO in vpr4.3 at 2014-03-18 by Pengqiu Deng, Tuesday.     *
 * This algorithm according to "A Novel Net Weighting Algorithm for Timing-  *
 * Driven Placement" by Tim Kong, 2000.                                      *
 ****************************************************************************/
typedef enum e_place_algorithm {
    BOUNDING_BOX_PLACE = 0, /* HPWL-driven */
    NET_TIMING_DRIVEN_PLACE,
    PATH_TIMING_DRIVEN_PLACE,
    /* NEW_TIMING_DRIVEN_PLACE was according to PATH_ALGO */
    NEW_TIMING_DRIVEN_PLACE
} place_algorithm_t;

/* Various options for VPR4.3 SA-based Placer(T-VPlace).                     *
 * place_algorithm:  BOUNDING_BOX_PLACE or NET_TIMING_DRIVEN_PLACE, or       *
 *                   PATH_TIMING_DRIVEN_PLACE                                *
 * timing_tradeoff:  When TIMING_DRIVEN_PLACE mode, what is the tradeoff *
 *                   timing driven and BOUNDING_BOX_PLACE.                   *
 * block_dist:  Initial guess of how far apart blocks on the critical path   *
 *              This is used to compute the initial slacks and criticalities *
 * place_cost_type:  LINEAR_CONG or NONLINEAR_CONG.                          *
 * place_cost_exp:  Power to which denominator is raised for linear_cong.    *
 * place_chan_width:  The channel width assumed if only one placement is     *
 *                    performed.                                             *
 * pad_loc_type:  Are pins FREE, fixed randomly, or fixed from a file.       *
 * pad_loc_file:  File to read pin locations form if pad_loc_type is USER.   *
 * place_freq:  Should the placement be skipped, done once, or done for each *
 *              channel width in the binary search.                          *
 * num_regions:  Used only with NONLINEAR_CONG; in that case, congestion is  *
 *               computed on an array of num_regions x num_regions basis.    *
 * recompute_crit_iter: how many temperature stages pass before we recompute *
 *               criticalities based on average point to point Tdel         *
 * enable_timing_computations: in bounding_box mode, normally, timing        *
 *               information is not produced, this causes the information    *
 *               to be computed. in *_TIMING_DRIVEN modes, this has no effect*
 * inner_loop_crit_divider: (move_lim/inner_loop_crit_divider) determines how*
 *               many inner_loop iterations pass before a recompute of       *
 *               criticalities is done.                                      *
 * td_place_exp_first: exponent that is used on the timing_driven criticlity *
 *               it is the value that the exponent starts at.                *
 * td_place_exp_last: value that the criticality exponent will be at the end *
 * parallel_place: whether use placement parallel, default value was FALSE   */
typedef struct s_placer_opts {
    place_algorithm_t  place_algorithm;
    double             timing_tradeoff;
    int                block_dist;
    enum place_c_types place_cost_type;
    double             place_cost_exp;
    int                place_chan_width;
    enum e_pad_loc_type pad_loc_type;
    char*              pad_loc_file;
    enum pfreq         place_freq;
    int                num_regions;
    int                recompute_crit_iter;
    boolean            enable_timing_computations;
    int                inner_loop_recompute_divider;
    double             td_place_exp_first; /* 1.0 */
    double             td_place_exp_last;  /* 8.0 */
    /* New added for support para_place */
    boolean            place_parallel;
} placer_opts_t;

typedef  struct s_placer_costs {
    double  m_bb_cost;
    double  m_timing_cost;
    double  m_delay_cost;
    double  m_total_cost;

    double  m_inverse_prev_bb_cost;
    double  m_inverse_prev_timing_cost;

    double  m_av_cost;
    double  m_av_bb_cost;
    double  m_av_timing_cost;
    double  m_av_delay_cost;

    double  m_new_bb_cost;
    double  m_new_timing_cost;
    double  m_new_delay_cost;
} placer_costs_t;

typedef  struct s_placer_paras {
    boolean   m_fixed_pins;
    int       m_width_factor;

    /* sum_of_squares = total_cost * total_cost; */
    double    m_sum_of_squares;
    int       m_num_connections;
    double    m_place_delay_value;
    /* crit_exponent was depend on range_limit */
    double    m_max_delay;
    double    m_crit_exponent;

    int       m_outer_crit_iter_count;
    int       m_move_limit;
    int       m_inner_crit_iter_count;
    int       m_inner_recompute_limit;
    /* update temperature range_limit was depend on sucess_ratio*/
    double    m_range_limit;
    double    m_final_rlim;
    double    m_inverse_delta_rlim;
    /* temperature and old_temperature */
    double    m_temper; /* temperature update was depend on sucess_ratio */

    int       m_total_iter;    /* total_iter += move_limit */
    int       m_success_sum;   /* ++success_sum */
    double    m_success_ratio; /* ratio = success_sum / total_iter */
    double    m_std_dev;
} placer_paras_t;  /* totally 19 items */
/******************************************************************************
 * Now declearing the data structures used for VPR4.3 PathFinder-based Router *
 *****************************************************************************/
typedef enum e_route_type {
    GLOBAL,
    DETAILED
} router_types_t;

typedef enum e_router_algorithm {
    BREADTH_FIRST,
    TIMING_DRIVEN
} router_algorithm_t;

typedef enum e_router_base_cost_type {
    INTRINSIC_DELAY,
    DELAY_NORMALIZED,
    DEMAND_ONLY
} router_base_cost_t;

#define NO_FIXED_CHANNEL_WIDTH  -1
/* All the parameters controlling the router's operation are in this structure.     *
 * first_iter_pres_fac: Present sharing penalty factor used for the very first      *
 *                      (congestion mapping) Pathfinder iteration. (Pn)             *
 * initial_pres_fac: Initial present sharing penalty factor for PathFinder; it      *
 *                   used to set pres_fac on 2nd iteration.                         *
 * pres_fac_mult: Amount by which pres_fac is multiplied each routing iteration.    *
 * acc_fac: Historical congestion cost multiplier.  Used unchanged for all          *
 *          iterations. (hn multiplier)                                             *
 * bend_cost: Cost of a bend (usually non-zero only for global routing).            *
 * max_router_iterations: Maximum number of iterations before giving up.            *
 * bb_factor: Linear distance a route can go outside the net bounding box.          *
 * route_type: GLOBAL or DETAILED.                                                  *
 * fixed_channel_width: Only attempt to route the design once, with the channel     *
 *                      width given. If this variable is == NO_FIXED_CHANNEL_WIDTH, *
 *                      do a binary search on channel width.                        *
 * router_algorithm: BREADTH_FIRST or TIMING_DRIVEN. Selects the desired routing    *
 *                   algorithm.                                                     *
 * router_base_cost_type: Specifies how to compute the base cost of each type of rr_node.  *
 *                 INTRINSIC_DELAY->base_cost = intrinsic Tdel of each node.       *
 *                 DELAY_NORMALIZED->base_cost = "demand" x average Tdel to route  *
 *                 past 1 CLB_TYPE. DEMAND_ONLY->expected demand of this node(old BFS cost)*
 *                 (bn for each rr_node)                                             *
 * The following parameters are used only by the Timing-Driven Router.      *
 * astar_fac:  Factor (alpha) used to weight expected future costs to       *
 *             target in the timing_driven router.  astar_fac = 0 leads to  *
 *             an essentially breadth-first search, astar_fac = 1 is near   *
 *             the usual astar algorithm and astar_fac > 1 are more         *
 *             aggressive.                                                  *
 * max_criticality: The maximum criticality factor (from 0 to 1) any sink   *
 *                  will ever have (i.e. clip criticality to this number).  *
 * criticality_exp: Set criticality to (path_length(sink) / longest_path) ^ *
 *                  criticality_exp (then clip to max_criticality).         */
typedef struct s_router_opts {
    double first_iter_pres_fac;
    double initial_pres_fac;
    double pres_fac_mult;
    double acc_fac;
    double bend_cost;
    int max_router_iterations;
    int bb_factor;
    router_types_t route_type;
    int fixed_channel_width;
    router_algorithm_t router_algorithm;
    router_base_cost_t router_base_cost_type;
    double astar_fac;
    double max_criticality;
    double criticality_exp;
} router_opts_t;

typedef enum e_switch_block_type {
    SUBSET,
    WILTON,
    UNIVERSAL
} switch_block_t;

enum e_Fc_type {
    ABSOLUTE,
    FRACTIONAL
};
typedef enum e_Fc_type Fc_type_t;

/* Defines the Detailed Routing Architecture of the FPGA. Only important if *
 * the route_type is DETAILED.                                              *
 * Fc_type:   Are the Fc values below absolute numbers, or fractions of W?  *
 * Fc_output:  Number of tracks to which each clb output pin connect in     *
 *             each channel to which it is adjacent.                        *
 * Fc_input:  Number of tracks to which each clb input pin connects.        *
 * Fc_pad:    Number of tracks to which each I/O pad connects.              *
 * switch_block_type:  Pattern of switches at each switch blocks. I assume Fs*
 *           is always 3. If the type is SUBSET, I use a Xilinx-liked switch*
 *           blocks where track i in one channel always connects to track i  *
 *           in other channels. If type is WILTON, I use a switch blocks where*
 *           track i does not always connect to track i in other channels.  *
 *           See Steve Wilton, Phd Thesis, University of Toronto, 1996. "The*
 *           UNIVERSAL Switch Block is from Y. W. Chang et al, TODAES, Jan. *
 *           pp. 80-101.                                                    *
 * num_segment:  Number of distinct segment types in the FPGA.              *
 * num_switch:  Number of distinct switch types (pass transistors or        *
 *              buffers) in the FPGA.                                       *
 * Tdelless_switch:  Index of a zero Tdel switch(used to connect things   *
 *                    that should have no Tdel).                           *
 * wire_to_ipin_switch:  Index of a switch used to connect wire segments to *
 *                       clb or pad input pins (IPINs).                     *
 * R_minW_nmos:  Resistance (in Ohms) of a minimum width nmos transistor.   *
 *               Used only in the FPGA area model.                          *
 * R_minW_pmos:  Resistance (in Ohms) of a minimum width pmos transistor.   */
typedef struct s_det_routing_arch {
    enum  e_Fc_type Fc_type;
    double Fc_output;
    double Fc_input;
    double Fc_pad;
    switch_block_t switch_block_type;
    int num_segment;
    short num_switch;
    short Tdelless_switch;
    short wire_to_ipin_switch;
    double R_minW_nmos;
    double R_minW_pmos;
} detail_routing_arch_t;

/* Lists all the important information about a certain Segment type.  Only   *
 * used if the route_type is DETAILED.  [0...det_routing_arch.num_segment]   *
 * frequency:  Fraction of tracks which are of this segment type.            *
 * length:     Length (in clbs) of the segment.                              *
 * wire_switch:  Index of the switch type that connects other wires *to*     *
 *               this segment.                                               *
 * opin_switch:  Index of the switch type that connects output pins (OPINs)  *
 *               *to* this segment.                                          *
 * frac_cb:  The fraction of logic blocks along its length to which this     *
 *           segment can connect. (i.e. internal population).                *
 * frac_sb:  The fraction of the length+1 switch blocks along the segment to *
 *           which the segment can connect. Segments that aren't long lines  *
 *           must connect to at least two switch boxes.                      *
 * Cmetal: Capacitance of a routing track, per unit logic blocks length.      *
 * Rmetal: Resistance of a routing track, per unit logic blocks length.       */
typedef struct s_segment_info_t {
    double frequency;
    int    length;
    short  wire_switch;
    short  opin_switch;
    double frac_cb;
    double frac_sb;
    boolean longline;
    double  Rmetal;
    double  Cmetal;
} segment_info_t;

/* Lists all the important information about a Switch Type.                        *
 * [0 .. det_routing_arch.num_switch]                                              *
 * buffered: Does this switch include a buffer?                                    *
 * R: Equivalent resistance of the buffer/switch.                                  *
 * Cin: Input capacitance.                                                         *
 * Cout: Output capacitance.                                                       *
 * Tdel: Intrinsic Tdel. The Tdel through an unloaded switch is (Tdel + R*Cout)  */
typedef struct s_switch_inf {
    boolean buffered; /* buffer or tri-state transistor */
    double R;
    double Cin;
    double Cout;
    double Tdel;
} switch_info_t;

/* Lists detailed information about Segmentation.  [0 .. W-1].              *
 * length:  length of segment.                                              *
 * start:  index at which a segment starts in channel 0.                    *
 * longline:  TRUE if this segment spans the entire channel.                *
 * sb:  [0..length]:  TRUE for every channel intersection, relative to the  *
 *      segment start, at which there is a switch box.                      *
 * cb:  [0..length-1]:  TRUE for every logic blocks along the segment at     *
 *      which there is a connection box.                                    *
 * wire_switch:  Index of the switch type that connects other wires *to*    *
 *               this segment.                                              *
 * opin_switch:  Index of the switch type that connects output pins (OPINs) *
 *               *to* this segment.                                         *
 * Cmetal: Capacitance of a routing track, per unit logic blocks length.     *
 * Rmetal: Resistance of a routing track, per unit logic blocks length.      *
 * index: index of the segment type used for this track.                    */
typedef struct t_seg_details {
    int      length;
    int      start;
    boolean  longline;
    boolean* sb;
    boolean* cb;
    short    wire_switch;
    short    opin_switch;
    double   Rmetal;
    double   Cmetal;
    int      index;
} segment_details_t;

/* A linked list of double pointers.  Used for keeping track of   *
 * which pathcosts in the router have been changed.              */
struct s_linked_f_pointer {
    struct s_linked_f_pointer* next;
    double* fptr;
};

/* Uncomment lines below to save some memory, at the cost of debugging ease. */
/*enum e_rr_type {SOURCE, SINK, IPIN, OPIN, CHANX, CHANY}; */
/* typedef short rr_types_t */


/* Type of a routing resource node. X-directed channel segment,  *
 * Y-directed channel segment, input pin to a clb to pad, output *
 * from a clb or pad (i.e. output pin of a net) and:             *
 * SOURCE:  A dummy node that is a logical output within a blocks *
 *          -- i.e., the gate that generates a signal.           *
 * SINK:    A dummy node that is a logical input within a blocks  *
 *          -- i.e. the gate that needs a signal.                */
typedef enum t_rr_type {
    SOURCE,
    SINK,
    IPIN,
    OPIN,
    CHANX,
    CHANY
} rr_types_t;

/* Basic element used to store the traceback(routing) of each net.       *
 * index:   Array index (ID) of this routing resource node.              *
 * iswitch:  Index of the switch type used to go from this rr_node to    *
 *           the next one in the routing. OPEN if there is no next node  *
 *           (i.e. this node is the last one (a SINK) in a branch of the *
 *           net's routing).                                             *
 * next:    pointer to the next traceback element in this route.         */
struct s_trace {
    int index;
    short iswitch;
    struct s_trace* next;
};


#define NO_PREVIOUS -1

/* Main structure describing one Routing-Resource-Node.  Everything in       *
 * this structure should describe the RR Graph -- information needed only    *
 * to store algorithm-specific data should be stored in one of the           *
 * parallel rr_node_?? structures.                                           *
 *                                                                           *
 * xlow, xhigh, ylow, yhigh:  Integer coordinates (see route.c for           *
 *       coordinate system) of the ends of this routing resource.            *
 *       xlow = xhigh and ylow = yhigh for pins or for segments of           *
 *       length 1.  These values are used to decide whether or not this      *
 *       node should be added to the expansion heap, based on things         *
 *       like whether it's outside the net bounding box or is moving         *
 *       further away from the target, etc.                                  *
 * type:  What is this routing resource?                                     *
 * ptc_num:  Pin, track or class number, depending on rr_node type.          *
 *           Needed to properly draw.                                        *
 * cost_index: An integer index into the table of routing resource indexed   *
 *             data (this indirection allows quick dynamic changes of rr     *
 *             base costs, and some memory storage savings for fields that   *
 *             have only a few distinct values).                             *
 * occ:        Current occupancy (usage) of this node.                       *
 * capacity:   Capacity of this node (number of routes that can use it).     *
 * num_edges:  Number of edges exiting this node.  That is, the number       *
 *             of nodes to which it connects.                                *
 * edges[0..num_edges-1]:  Array of indices of the neighbours of this        *
 *                         node.                                             *
 * switches[0..num_edges-1]:  Array of switch indexes for each of the        *
 *                            edges leaving this node.                       *
 *                                                                           *
 * The following parameters are only needed for timing analysis.             *
 * R:  Resistance to go through this node.  This is only metal               *
 *     resistance (end to end, so conservative) -- it doesn't include the    *
 *     switch that leads to another rr_node.                                 *
 * C:  Total capacitance of this node.  Includes metal capacitance, the      *
 *     input capacitance of all switches hanging off the node, the           *
 *     output capacitance of all switches to the node, and the connection    *
 *     box buffer capacitances hanging off it.                               */
typedef struct t_rr_node {
    short xlow;
    short xhigh;
    short ylow;
    short yhigh;
    short ptc_num;
    short cost_index;
    short occ;
    short capacity;
    short num_edges;
    rr_types_t type;
    int* edges;
    short* switches;
    double R;
    double C;
} rr_node_t;

/* Data that is pointed to by the .cost_index member of rr_node_t.  It's     *
 * purpose is to store the base_cost so that it can be quickly changed       *
 * and to store fields that have only a few different values (like           *
 * seg_index) or whose values should be an average over all rr_nodes of a    *
 * certain type (like T_linear etc., which are used to predict remaining     *
 * Tdel in the timing_driven router).                                       *
 *                                                                           *
 * base_cost:  The basic cost of using an rr_node.                           *
 * ortho_cost_index:  The index of the type of rr_node that generally        *
 *                    connects to this type of rr_node, but runs in the      *
 *                    orthogonal direction (e.g. vertical if the direction   *
 *                    of this member is horizontal).                         *
 * seg_index:  Index into segment_inf of this segment type if this type of   *
 *             rr_node is an CHANX or CHANY; OPEN (-1) otherwise.            *
 * inv_length:  1/length of this type of segment.                            *
 * T_linear:  Delay through N segments of this type is N * T_linear + N^2 *  *
 *            T_quadratic.  For buffered segments all Tdel is T_linear.     *
 * T_quadratic:  Dominant Tdel for unbuffered segments, 0 for buffered      *
 *               segments.                                                   *
 * C_load:  Load capacitance seen by the driver for each segment added to    *
 *          the chain driven by the driver.  0 for buffered segments.        */
typedef struct t_rr_indexed_data {
    double base_cost;
    double saved_base_cost;
    int ortho_cost_index;
    int seg_index;
    double inv_length;
    double T_linear;
    double T_quadratic;
    double C_load;
} rr_indexed_data_t;

/* Gives the index of the SOURCE, SINK, OPIN, IPIN, etc. member of rr_indexed_data */
enum e_cost_indices {
    SOURCE_COST_INDEX = 0,
    SINK_COST_INDEX,
    OPIN_COST_INDEX,
    IPIN_COST_INDEX,
    CHANX_COST_INDEX_START
};

#endif

