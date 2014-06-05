#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>

#include "util.h"
#include "vpr_types.h"
#include "globals.h"
#include "read_arch.h"

#define  NUM_GRID_TYPES  3

#define EMPTY  -1

/* This source file reads in the architectural description of an FPGA.     *
 * A # symbol anywhere in the input file denotes a comment to the end      *
 * of the line.  Put a \ at the end of a line if you want to continue      *
 * a command across multiple lines. Non-comment lines are in the           *
 * format keyword value(s).  The entire file should be lower case.         *
 * The keywords and their arguments are:                                   *
 *                                                                         *
 *   io_ratio integer(sets the number of io pads which fit into the        *
 *                    space one CLB_TYPE would use).                       *
 *   chan_width_io double(Width of the channels between the pads and       *
 *                        core relative to the widest core channel.)       *
 *   chan_width_x [gaussian|uniform|pulse] peak <width> <xpeak> <dc>.      *
 *       (<> bracketed quantities needed only for pulse and gaussian.      *
 *       Width and xpeak values from 0 to 1.  Sets the distribution of     *
 *       tracks for the x-directed channels.)                              *
 *       Other possibility:  delta peak xpeak dc.                          *
 *   chan_width_y [gaussian|uniform|pulse] peak <width> <xpeak> <dc>.      *
 *       (Sets the distribution of tracks for the y-directed channels.)    *
 *   outpin class: integer [top|bottom|left|right] [top|bottom|left|       *
 *       right] ...                                                          *
 *       (Sets the class to which each pin belongs and the side(s) of        *
 *       CLBs on which the physical output pin connection(s) is (are).       *
 *       All pins with the same class number are logically equivalent        *
 *       -- such as all the inputs of a LUT.  Class numbers must start       *
 *       at zero and be consecutive.)                                        *
 *   inpin class: integer <global> [top|bottom|left|right] [top|bottom|left| *
 *       right] ...                                                          *
 *       (All parameters have the same meanings as their counterparts        *
 *       in the outpin statement, except the optional global keyword.  If    *
 *       global is specified, this inpin can connect only to global nets.    *
 *       Global nets are not routed by the router, and are normally only     *
 *       things like clock nets.  A global pin has no switches connecting it *
 *       to the regular routing, so it takes up no area in the area model.)  *
 *                                                                           *
 *   NOTE:  The order in which your inpin and outpin statements appear       *
 *      must be the same as the order in which your netlist (.net)           *
 *      file lists the connections to the clbs.  For example, if the         *
 *      first pin on each clb in the netlist file is the clock pin,          *
 *      your first pin statement in the architecture file must be            *
 *      an inpin statement defining the clock pin.                           *
 *                                                                           *
 *   subblocks_per_clb <int>  (Number of LUT + ff logic blocks in each clb   *
 *      at most).                                                            *
 *   subblock_lut_size <int>  (Number of inputs to each LUT in the           *
 *      clbs.  Each subblock has subblock_lut_size inputs, one output        *
 *      and a clock input.)                                                  *
 *                                                                           *
 *  The following parameters only need to be in the architecture             *
 *  file if detailed routing is going to be performed (i.e. route_type       *
 *  == DETAILED).                                                            *
 *                                                                           *
 *   Fc_type [absolute|fractional]  (Are the 3 Fc values absolute            *
 *      numbers of tracks to connect to, or the fraction of the W            *
 *      tracks to which each pin can connect?)                               *
 *   Fc_output double (Sets the value of Fc -- the number of tracks           *
 *      each pin can connect to in each channel bordering the pin --         *
 *      for output pins.  The Fc_output value used is always                 *
 *      min(W,Fc_selected), so set Fc to be huge if you want Fc = W.)        *
 *   Fc_input double (Sets the value of Fc for input pins.)                   *
 *   Fc_pad double (Sets the value of Fc for pads.)                           *
 *   switch_block_type [subset|wilton|universal] (Chooses the type of        *
 *      switch blocks used.  See vpr_types.h for details.)                    *
 *   segment frequency: <double> length: <int | longline> wire_switch: <int>  *
 *      opin_switch: <int> Frac_cb: <double> Frac_sb: <double> Rmetal: <double> *
 *      Cmetal: <double>                                                      *
 *      Describes one type of segment.  wire_switch is the type of           *
 *      switch used when going *to* a segment of this type from any CHANX or *
 *      CHANY routing segment.  opin_switch is the type of switch used       *
 *      by a clb or pad output driver (OPIN) to connect to segments of this  *
 *      type.  Cmetal is the capacitance per logic blocks spanned (i.e. per   *
 *      channel segment) of a routing track of this segment type.            *
 *      Similarly, Rmetal is the resistance per logic blocks spanned.         *
 *   switch  <int> buffered: {yes|no} R: <double> Cin: <double>                *
 *      Cout: <double> Tdel: <double>.  Describes a type of switch.            *
 *   R_minW_nmos <double>   Resistance, in Ohms, of a minimum width nmos      *
 *      transistor.  Used only in the transistor-level area model.           *
 *   R_minW_pmos <double>   Resistance, in Ohms, of a minimum width pmos      *
 *      transistor.  Used only in the transistor-level area model.           *
 *                                                                           *
 * The following parameters allow timing analysis.                           *
 *   C_ipin_cblock <double>: Input capacitance of the buffer isolating a      *
 *      routing track from the input pin Cboxes connected to it at each      *
 *      (i,j) location.                                                      *
 *   T_ipin_cblock <double>: Delay to go through a connection blocks to a      *
 *      logic blocks (clb) input pin.                                         *
 *   T_sblk_opin_to_sblk_ipin <double>: Delay through the local interconnect  *
 *      (muxes, wires or whatever) in a clb containing multiple subblocks.   *
 *      That is, the Tdel from a subblock output to the input of another    *
 *      subblock in the same clb.                                            *
 *   T_clb_ipin_to_sblk_ipin <double>: Delay from a clb input pin to any      *
 *      subblock input pin (e.g. the mux Tdel in an Altera 8K clb).         *
 *   T_sblk_opin_to_clb_opin <double>: Delay from a subblock output to a clb  *
 *      output.  Will be 0 in many architectures.                            *
 *   T_ipad:  Delay through an input pad.                                    *
 *   T_opad:  Delay through an output pad (setup time if you assume the      *
 *               outputs are registered before being sent out).              *
 *   T_subblock T_comb: <double>  T_seq_in: <double> T_seq_out: <double>        *
 *      The combinational Tdel through a subblock combinational mode), the  *
 *      Tdel from subblock input to data being latched (including setup     *
 *      time), and the Tdel from clock to data coming out of the subblock   *
 *      output (i.e. clk_to_Q plus any Tdels due to muxes, etc.).  If a     *
 *      subblock is being used in combinational mode (clock input is open)   *
 *      then Tcomb is used to find the Tdel through it.  If the subblock    *
 *      is being used in sequential mode (clock input not open) then         *
 *      T_seq_in gives the Tdel to the storage element and T_seq_out gives  *
 *      the Tdel from storage element to subblock output.  You need one     *
 *      of these T_subblock lines for each of the subblocks_per_clb          *
 *      subblocks in your architecture.  The first line gives the Tdels for *
 *      subblock 0 (first one listed for each logic blocks in the netlist     *
 *      file) and so on.                                                     */
/******************* Defines and types local to this module *****************/
#define NUM_REQUIRED 8  /* Number of parameters that are always required. */
#define NUM_DETAILED 9  /* Number needed only if detailed routing used. */
#define NUM_TIMING 8   /* Number needed only if timing analysis used.  */

#define DETAILED_START NUM_REQUIRED
#define TIMING_START (NUM_REQUIRED + NUM_DETAILED)

/* Total number of different parameters in arch file. */
#define NUMINP (NUM_REQUIRED + NUM_DETAILED + NUM_TIMING)


/******************** Variables local to this module. **********************/
static int isread[NUMINP];
static const char* names[NUMINP] = {"io_ratio",
                                    "chan_width_x",
                                    "chan_width_y",
                                    "chan_width_io",
                                    "outpin",
                                    "inpin",
                                    "subblocks_per_clb",
                                    "subblock_lut_size",
                                    "Fc_output", "Fc_input", "Fc_pad", "Fc_type",
                                    "switch_block_type", "segment", "switch",
                                    "R_minW_nmos", "R_minW_pmos", "C_ipin_cblock",
                                    "T_ipin_cblock", "T_sblk_opin_to_sblk_ipin",
                                    "T_clb_ipin_to_sblk_ipin",
                                    "T_sblk_opin_to_clb_opin",
                                    "T_ipad", "T_opad", "T_subblock"};


/********************** Subroutines local to this module. ******************/
static double get_double(char* ptr, int inp_num, double llim, double ulim,
                         FILE* fp_arch, char* buf);

static double get_one_double(char* ptr, int inp_num, double low_lim,
                             double upp_lim, FILE* fp_arch, char* buf);

static int get_int(char* ptr, int inp_num, FILE* fp_arch, char* buf,
                   int min_val);

static char* get_middle_token(FILE* fp, char* buf);

static char* get_last_token(FILE* fp, char* buf);

static void check_keyword(FILE* fp, char* buf, const char* keyword);

static void check_arch(char* arch_file, router_types_t route_type,
                       detail_routing_arch_t det_routing_arch,
                       segment_info_t*  segment_inf,
                       timing_info_t timing_inf,
                       int max_subblocks_per_block,
                       chan_width_distr_t chan_width_dist);

static void fill_arch(void);

static void gechannel_t(char* ptr,
                        channel_t* chan,
                        int inp_num,
                        FILE* fp_arch,
                        char* buf);

static void get_pin(char* ptr, int pinnum, pin_types_t type,
                    FILE* fp_arch, char* buf);

static enum e_Fc_type get_Fc_type(char* ptr, FILE* fp_arch, char* buf);

static switch_block_t get_switch_block_type(FILE* fp_arch,
                                            char* buf);

static void gesegment_info_t(FILE* fp_arch, char* buf,
                            segment_info_t* seg_ptr, int num_switch, router_types_t
                            route_type);

static void get_switch_inf(FILE* fp_arch, char* buf, int num_switch,
                           router_types_t route_type);

static void get_T_subblock(FILE* fp_arch, char* buf, T_subblock_t
                           *T_subblock);

static int get_class(FILE* fp_arch, char* buf);

static void load_global_segment_and_switch(detail_routing_arch_t *
                                           det_routing_arch, segment_info_t* segment_inf, timing_info_t
                                           *timing_inf);

static void load_extra_switch_types(detail_routing_arch_t *
                                    det_routing_arch, timing_info_t* timing_inf);

static void countpass(FILE* fp_arch, router_types_t route_type,
                      segment_info_t** segment_inf_ptr,
                      detail_routing_arch_t* det_routing_arch_ptr,
                      timing_info_t*  timing_inf);

static void init_grid_capacity(void);

/****************** Subroutine definitions **********************************/
/* Reads in the architecture description file for the FPGA. */
void read_arch(char* arch_file, router_types_t route_type,
               detail_routing_arch_t* det_routing_arch, segment_info_t
               ** segment_inf_ptr, timing_info_t* timing_inf_ptr, subblock_data_t
               *subblock_data_ptr, chan_width_distr_t* chan_width_dist_ptr)
{
    FILE* fp_arch = my_fopen(arch_file, "r");
    countpass(fp_arch,
              route_type,
              segment_inf_ptr,
              det_routing_arch,
              timing_inf_ptr);
    rewind(fp_arch);

    linenum = 0;
    int pinnum = 0;
    int next_segment = 0;

    int i, j;
    for (i = 0; i < NUMINP; ++i) {
        isread[i] = 0;
    }

    pinloc = (int**)alloc_matrix(0, 3,
                                 0, pins_per_clb - 1,
                                 sizeof(int));
    for (i = 0; i <= 3; ++i)
        for (j = 0; j < pins_per_clb; ++j) {
            pinloc[i][j] = 0;
        }

    /* Initialize these two things to zero, since they're used in graph building *
     * and they needn't be set by the user if timing_analysis isn't enabled.     */
    timing_inf_ptr->C_ipin_cblock = 0.;
    timing_inf_ptr->T_ipin_cblock = 0.;

    /* Start the main pass (pass 2).   */
    T_subblock_t* next_T_subblock_ptr;
    char* ptr, buf[BUFSIZE];
    while ((ptr = my_fgets(buf, BUFSIZE, fp_arch)) != NULL) {

        ptr = my_strtok(ptr, TOKENS, fp_arch, buf);
        if (ptr == NULL) {
            continue;    /* Empty or comment line */
        }

        /* This linear compare is getting pretty long.  Could speed up with a hash *
         * table search -- do that if this starts getting slow.                    */
        if (strcmp(ptr, names[0]) == 0) { /* io_ratio */
            io_ratio = get_int(ptr, 0, fp_arch, buf, 1);
            continue;
        }

        if (strcmp(ptr, names[1]) == 0) { /*chan_width_x */
            gechannel_t(ptr, &chan_width_dist_ptr->chan_x_dist, 1, fp_arch, buf);
            continue;
        }

        if (strcmp(ptr, names[2]) == 0) { /* chan_width_y */
            gechannel_t(ptr, &chan_width_dist_ptr->chan_y_dist, 2, fp_arch, buf);
            continue;
        }

        if (strcmp(ptr, names[3]) == 0) { /* chan_width_io */
            chan_width_dist_ptr->chan_width_io = get_one_double(ptr, 3, 0. , 5000.,
                                                               fp_arch, buf);
            continue;
        }

        if (strcmp(ptr, names[4]) == 0) { /* outpin */
            get_pin(ptr, pinnum, DRIVER, fp_arch, buf);
            pinnum++;
            isread[4]++;
            continue;
        }

        if (strcmp(ptr, names[5]) == 0) { /* inpin */
            get_pin(ptr, pinnum, RECEIVER, fp_arch, buf);
            pinnum++;
            isread[5]++;
            continue;
        }

        if (strcmp(ptr, names[6]) == 0) { /* subblocks_per_clb */
            subblock_data_ptr->max_subblocks_per_block = get_int(ptr, 6, fp_arch,
                                                                 buf, 1);
            continue;
        }

        if (strcmp(ptr, names[7]) == 0) { /* subblock_lut_size */
            subblock_data_ptr->subblock_lut_size = get_int(ptr, 7, fp_arch, buf,
                                                           1);
            continue;
        }

        if (strcmp(ptr, names[DETAILED_START]) == 0) { /* Fc_output */
            det_routing_arch->Fc_output = get_one_double(ptr, DETAILED_START, 0.,
                                                        1.e20, fp_arch, buf);
            continue;
        }

        if (strcmp(ptr, names[DETAILED_START + 1]) == 0) { /* Fc_input */
            det_routing_arch->Fc_input = get_one_double(ptr, DETAILED_START + 1, 0.,
                                                       1.e20, fp_arch, buf);
            continue;
        }

        if (strcmp(ptr, names[DETAILED_START + 2]) == 0) { /* Fc_pad */
            det_routing_arch->Fc_pad = get_one_double(ptr, DETAILED_START + 2, 0.,
                                                     1.e20, fp_arch, buf);
            continue;
        }

        if (strcmp(ptr, names[DETAILED_START + 3]) == 0) {  /* Fc_type */
            det_routing_arch->Fc_type = get_Fc_type(ptr, fp_arch, buf);
            isread[DETAILED_START + 3]++;
            continue;
        }

        if (strcmp(ptr, names[DETAILED_START + 4]) == 0) { /* switch_block_type */
            det_routing_arch->switch_block_type =  get_switch_block_type(fp_arch,
                                                                         buf);
            isread[DETAILED_START + 4]++;
            continue;
        }

        if (strcmp(ptr, names[DETAILED_START + 5]) == 0) { /* segment */
            gesegment_info_t(fp_arch, buf, *segment_inf_ptr + next_segment,
                            det_routing_arch->num_switch, route_type);
            next_segment++;
            isread[DETAILED_START + 5]++;
            continue;
        }

        if (strcmp(ptr, names[DETAILED_START + 6]) == 0) { /* switch */
            get_switch_inf(fp_arch, buf, det_routing_arch->num_switch, route_type);
            isread[DETAILED_START + 6]++;
            continue;
        }

        if (strcmp(ptr, names[DETAILED_START + 7]) == 0) { /* R_minW_nmos */
            det_routing_arch->R_minW_nmos = get_one_double(ptr, DETAILED_START + 7,
                                                          0., 1.e20, fp_arch, buf);
            continue;
        }

        if (strcmp(ptr, names[DETAILED_START + 8]) == 0) { /* R_minW_pmos */
            det_routing_arch->R_minW_pmos = get_one_double(ptr, DETAILED_START + 8,
                                                          0., 1.e20, fp_arch, buf);
            continue;
        }

        if (strcmp(ptr, names[TIMING_START]) == 0) { /* C_ipin_cblock */
            timing_inf_ptr->C_ipin_cblock = get_one_double(ptr, TIMING_START,
                                                          -1e-30, 1e20, fp_arch, buf);
            continue;
        }

        if (strcmp(ptr, names[TIMING_START + 1]) == 0) { /* T_ipin_cblock */
            timing_inf_ptr->T_ipin_cblock = get_one_double(ptr, TIMING_START + 1,
                                                          -1e-30, 1e20, fp_arch, buf);
            continue;
        }

        if (strcmp(ptr, names[TIMING_START + 2]) == 0) {
            /* T_sblk_opin_to_sblk_ipin */
            timing_inf_ptr->T_sblk_opin_to_sblk_ipin = get_one_double(ptr,
                                                                     TIMING_START + 2, -1e-30, 1e20, fp_arch, buf);
            continue;
        }

        if (strcmp(ptr, names[TIMING_START + 3]) == 0) {
            /* T_clb_ipin_to_sblk_ipin */
            timing_inf_ptr->T_clb_ipin_to_sblk_ipin = get_one_double(ptr,
                                                                    TIMING_START + 3,
                                                                    -1e-30, 1e20,
                                                                    fp_arch,
                                                                    buf);
            continue;
        }

        if (strcmp(ptr, names[TIMING_START + 4]) == 0) {
            /* T_sblk_opin_to_clb_opin */
            timing_inf_ptr->T_sblk_opin_to_clb_opin = get_one_double(ptr,
                                                                     TIMING_START + 4,
                                                                     -1e-30,
                                                                     1e20,
                                                                     fp_arch,
                                                                     buf);
            continue;
        }

        if (strcmp(ptr, names[TIMING_START + 5]) == 0) { /* T_ipad */
            timing_inf_ptr->T_ipad = get_one_double(ptr, TIMING_START + 5,
                                                   -1e-30, 1e20, fp_arch, buf);
            continue;
        }

        if (strcmp(ptr, names[TIMING_START + 6]) == 0) { /* T_opad */
            timing_inf_ptr->T_opad = get_one_double(ptr, TIMING_START + 6,
                                                   -1e-30, 1e20, fp_arch, buf);
            continue;
        }

        if (strcmp(ptr, names[TIMING_START + 7]) == 0) { /* T_subblock */
            next_T_subblock_ptr = timing_inf_ptr->T_subblock +
                                  isread[TIMING_START + 7];
            get_T_subblock(fp_arch, buf, next_T_subblock_ptr);
            isread[TIMING_START + 7]++;
            continue;
        }

        printf("Error:  unrecognized keyword (%s) on line %d.\n", ptr, linenum);
        exit(1);
    }

    if (route_type == GLOBAL) {
        load_global_segment_and_switch(det_routing_arch,
                                       *segment_inf_ptr,
                                       timing_inf_ptr);
    } else {
        load_extra_switch_types(det_routing_arch, timing_inf_ptr);
    }

    check_arch(arch_file,
               route_type,
               *det_routing_arch,
               *segment_inf_ptr,
               *timing_inf_ptr,
               subblock_data_ptr->max_subblocks_per_block,
               *chan_width_dist_ptr);
    fclose(fp_arch);
}


/* This routine parses the input architecture file in order to count the    *
 * number of pinclasses, pins, segments, etc. so storage can be allocated   *
 * for them before the second(loading) pass begins.                        */
static void countpass(FILE* fp_arch,
                      router_types_t route_type,
                      segment_info_t**  segment_inf_ptr,
                      detail_routing_arch_t* det_routing_arch_ptr,
                      timing_info_t* timing_inf_ptr)
{
    linenum = 0;
    num_pin_class = 1;   /* Must be at least 1 class  */
    int num_segment = 0;
    int num_switch = 0;
    int num_T_subblock = 0;
    int* pins_per_class = (int*)my_calloc(num_pin_class, sizeof(int));

    int iclass, i;
    char buf[BUFSIZE], *ptr;
    while ((ptr = my_fgets(buf, BUFSIZE, fp_arch)) != NULL) {
        ptr = my_strtok(ptr, TOKENS, fp_arch, buf);
        if (ptr == NULL) { /* Empty or comment line. */
            continue;
        }

        if (strcmp(ptr, "inpin") == 0 || strcmp(ptr, "outpin") == 0
                || strcmp(ptr, "global_inpin") == 0) {

            iclass = get_class(fp_arch, buf);
            if (iclass >= num_pin_class) {
                pins_per_class = (int*)my_realloc(pins_per_class,
                                                  (iclass + 1) * sizeof(int));

                for (i = num_pin_class; i <= iclass; i++) {
                    pins_per_class[i] = 0;
                }

                num_pin_class = iclass + 1;
            }

            pins_per_class[iclass]++;
        } else if (strcmp(ptr, "segment") == 0) {
            ++num_segment;
        } else if (strcmp(ptr, "switch") == 0) {
            ++num_switch;
        } else if (strcmp(ptr, "T_subblock") == 0) {
            ++num_T_subblock;
        }

        /* Go to end of line (possibly continued) */

        while (ptr != NULL) {
            ptr = my_strtok(NULL, TOKENS, fp_arch, buf);
        }
    }  /* end of while() */

    /* Check for missing classes. */
    for (i = 0; i < num_pin_class; i++) {
        if (pins_per_class[i] == 0) {
            printf("\nError:  class index %d not used in architecture "
                   "file.\n", i);
            printf("          Specified class indices are not consecutive.\n");
            exit(1);
        }
    }

    /* I've now got a count of how many classes there are and how many    *
     * pins belong to each class.  Allocate the proper memory.            */
    class_inf = (pin_class_t*)my_malloc(num_pin_class * sizeof(pin_class_t));
    pins_per_clb = 0;
    for (i = 0; i < num_pin_class; ++i) {
        class_inf[i].type = OPEN;   /* Flag for not set yet. */
        class_inf[i].num_pins = 0;
        class_inf[i].pinlist = (int*)my_malloc(pins_per_class[i] *
                                                sizeof(int));
        pins_per_clb += pins_per_class[i];
    }

    free(pins_per_class);
    clb_pin_class = (int*)my_malloc(pins_per_clb * sizeof(int));
    is_global_clb_pin = (boolean*)my_malloc(pins_per_clb * sizeof(int));

    /* Now allocate space for segment and switch information if the route_type   *
     * is DETAILED.  Otherwise ignore the segment and switch information, and    *
     * create only one segment and one switch.                                   */
    if (route_type == GLOBAL) {
        num_segment = 1;
        num_switch = 1;
    } else { /* route_type == DETAILED */
        num_switch += 2;   /* Two extra switch types:  1 for zero Tdel, */
        /* 1 for connections from wires to IPINs.     */
    }

    if (num_segment < 1) {
        printf("Error:  No segment information specified in architecture.\n");
        exit(1);
    }

    *segment_inf_ptr = (segment_info_t*)my_malloc(num_segment *
                                                  sizeof(segment_info_t));
    det_routing_arch_ptr->num_segment = num_segment;

    if (num_switch < 1) {
        printf("Error:  No switch information specified in architecture.\n");
        exit(1);
    }

    switch_inf = (switch_info_t*)my_malloc(num_switch *
                                           sizeof(switch_info_t));
    for (i = 0; i < num_switch; i++) {
        switch_inf[i].R = -1.;    /* Flag to show it's not set yet. */
    }

    det_routing_arch_ptr->num_switch = num_switch;

    if (num_T_subblock != 0)
        timing_inf_ptr->T_subblock = (T_subblock_t*) my_malloc(num_T_subblock *
                                                               sizeof(T_subblock_t));
    else {
        timing_inf_ptr->T_subblock = NULL;
    }
}  /* end of static void countpass(FILE* fp_arch, ) */


static int get_class(FILE* fp_arch, char* buf)
{
    /* This routine is called when strtok has moved the pointer to just before *
     * the class: keyword.  It advances the pointer to after the class         *
     * descriptor and returns the class number.                                */
    char* ptr = my_strtok(NULL, TOKENS, fp_arch, buf);
    if (ptr == NULL) {
        printf("Error in get_class on line %d of architecture file.\n", linenum);
        printf("Expected class: keyword.\n");
        exit(1);
    }

    if (strcmp(ptr, "class:") != 0) {
        printf("Error in get_class on line %d of architecture file.\n", linenum);
        printf("Expected class: keyword.\n");
        exit(1);
    }

    /* Now get class number. */
    ptr = my_strtok(NULL, TOKENS, fp_arch, buf);
    if (ptr == NULL) {
        printf("Error in get_class on line %d of architecture file.\n", linenum);
        printf("Expected class number.\n");
        exit(1);
    }

    int iclass = my_atoi(ptr);
    if (iclass < 0) {
        printf("Error in get_class on line %d of architecture file.\n", linenum);
        printf("Expected class number >= 0, got %d.\n", iclass);
        exit(1);
    }

    return (iclass);
}


/* This routine parses an ipin or outpin line.  It should be called right   *
 * after the inpin, global_inpin, or outpin keyword has been parsed.        */
static void get_pin(char* ptr, int pinnum, pin_types_t type,
                    FILE* fp_arch, char* buf)
{
    char* position[4] = {"top",
                         "bottom",
                         "left",
                         "right"};

    int i, valid;
    int iclass = get_class(fp_arch, buf);
    if (class_inf[iclass].type == OPEN) {  /* First time through this class. */
        class_inf[iclass].type = type;
    } else {
        if (class_inf[iclass].type != type) {
            printf("Error in get_pin: architecture file, line %d.\n",
                   linenum);
            printf("Class %d contains both input and output pins.\n", iclass);
            exit(1);
        }
    }

    int ipin = class_inf[iclass].num_pins;
    class_inf[iclass].pinlist[ipin] = pinnum;
    class_inf[iclass].num_pins++;
    clb_pin_class[pinnum] = iclass;
    ptr = my_strtok(NULL, TOKENS, fp_arch, buf);

    if (ptr == NULL) {
        printf("Error:  pin statement specifies no locations, line %d.\n",
               linenum);
        exit(1);
    }

    if (type == RECEIVER && strcmp(ptr, "global") == 0) {
        is_global_clb_pin[pinnum] = TRUE;
        ptr = my_strtok(NULL, TOKENS, fp_arch, buf);

        if (ptr == NULL) {
            printf("Error:  pin statement specifies no locations, line %d.\n",
                   linenum);
            exit(1);
        }
    } else {
        is_global_clb_pin[pinnum] = FALSE;
    }

    do {
        valid = 0;
        for (i = 0; i <= 3; i++) {
            if (strcmp(ptr, position[i]) == 0) {
                pinloc[i][pinnum] = 1;
                valid = 1;
                break;
            }
        }

        if (valid != 1) {
            printf("Error:  bad pin location on line %d.\n", linenum);
            exit(1);
        }
    } while ((ptr = my_strtok(NULL, TOKENS, fp_arch, buf)) != NULL);
}


static enum e_Fc_type get_Fc_type(char* ptr, FILE* fp_arch, char* buf)
{
    /* Sets the Fc_type to either ABSOLUTE or FRACTIONAL.                    */
    enum e_Fc_type Fc_type;
    ptr = my_strtok(NULL, TOKENS, fp_arch, buf);

    if (ptr == NULL) {
        printf("Error:  missing Fc_type value on line %d of "
               "architecture file.\n", linenum);
        exit(1);
    }

    if (strcmp(ptr, "absolute") == 0) {
        Fc_type = ABSOLUTE;
    } else if (strcmp(ptr, "fractional") == 0) {
        Fc_type = FRACTIONAL;
    } else {
        printf("Error:  Bad Fc_type value (%s) on line %d of "
               "architecture file.\n", ptr, linenum);
        exit(1);
    }

    ptr = my_strtok(NULL, TOKENS, fp_arch, buf);

    if (ptr != NULL) {
        printf("Error:  extra characters at end of line %d.\n", linenum);
        exit(1);
    }

    return (Fc_type);
}


static switch_block_t get_switch_block_type(FILE* fp_arch,
                                            char* buf)
{
    /* Returns the proper value for the switch_block_type member of        *
     *  det_routing_arch.                                                  */
    char* ptr = my_strtok(NULL, TOKENS, fp_arch, buf);
    if (ptr == NULL) {
        printf("Error:  missing switch_block_type value on line %d of "
               "architecture file.\n", linenum);
        exit(1);
    }

    switch_block_t sblock_type;
    if (strcmp(ptr, "subset") == 0) {
        sblock_type = SUBSET;
    } else if (strcmp(ptr, "wilton") == 0) {
        sblock_type = WILTON;
    } else if (strcmp(ptr, "universal") == 0) {
        sblock_type = UNIVERSAL;
    } else {
        printf("Error:  Bad switch_block_type value (%s) on line %d of "
               "architecture file.\n", ptr, linenum);
        exit(1);
    }

    ptr = my_strtok(NULL, TOKENS, fp_arch, buf);
    if (ptr != NULL) {
        printf("Error:  extra characters at end of line %d.\n", linenum);
        exit(1);
    }

    return (sblock_type);
}


/* Parses one T_subblock line, and loads it into the T_subblock_t structure *
 * pointed to by T_subblock.                                                */
static void get_T_subblock(FILE* fp_arch, char* buf, T_subblock_t
                           *T_subblock)
{
    check_keyword(fp_arch, buf, "T_comb:");
    char* ptr = get_middle_token(fp_arch, buf);
    T_subblock->T_comb = atof(ptr);

    if (T_subblock->T_comb < 0.) {
        printf("Error on line %d:  T_comb value (%g) is negative.\n",
               linenum, T_subblock->T_comb);
        exit(1);
    }

    check_keyword(fp_arch, buf, "T_seq_in:");
    ptr = get_middle_token(fp_arch, buf);
    T_subblock->T_seq_in = atof(ptr);

    if (T_subblock->T_seq_in < 0.) {
        printf("Error on line %d:  T_seq_in value (%g) is negative.\n",
               linenum, T_subblock->T_seq_in);
        exit(1);
    }

    check_keyword(fp_arch, buf, "T_seq_out:");
    ptr = get_last_token(fp_arch, buf);
    T_subblock->T_seq_out = atof(ptr);

    if (T_subblock->T_seq_out < 0.) {
        printf("Error on line %d:  T_seq_out value (%g) is negative.\n",
               linenum, T_subblock->T_seq_out);
        exit(1);
    }
}


static void gesegment_info_t(FILE* fp_arch, char* buf, segment_info_t* seg_ptr,
                            int num_switch, router_types_t route_type)
{
    /* Loads the segment data structure pointed to by seg_ptr with the proper   *
     * values from fp_arch, if route_type == DETAILED.                          */
    char* ptr;
    int num_sb;

    if (route_type != DETAILED) {
        ptr = my_strtok(NULL, TOKENS, fp_arch, buf);

        while (ptr != NULL) {     /* Skip to the end of the line. */
            ptr = my_strtok(NULL, TOKENS, fp_arch, buf);
        }

        return;
    }

    check_keyword(fp_arch, buf, "frequency:");
    ptr = get_middle_token(fp_arch, buf);
    seg_ptr->frequency = atof(ptr);

    if (seg_ptr->frequency <= 0. || seg_ptr->frequency > 1.) {
        printf("Error on line %d:  Frequency value (%g) is out of range.\n",
               linenum, seg_ptr->frequency);
        exit(1);
    }

    check_keyword(fp_arch, buf, "length:");
    ptr = get_middle_token(fp_arch, buf);

    if (strcmp(ptr, "longline") == 0) {
        seg_ptr->length = -1;
        seg_ptr->longline = TRUE;
    } else {
        seg_ptr->length = my_atoi(ptr);
        seg_ptr->longline = FALSE;

        if (seg_ptr->length < 1) {
            printf("Error on line %d:  Length value (%d) is less than 1.\n",
                   linenum, seg_ptr->length);
            exit(1);
        }
    }

    check_keyword(fp_arch, buf, "wire_switch:");
    ptr = get_middle_token(fp_arch, buf);
    seg_ptr->wire_switch = my_atoi(ptr);

    /* Note:  last two switch types are generated automatically.  Shouldn't be *
     * used in the architecture file.                                          */
    if (seg_ptr->wire_switch < 0 || seg_ptr->wire_switch >= num_switch - 2) {
        printf("Error on line %d:  wire_switch value (%d) is out of range.\n",
               linenum, seg_ptr->wire_switch);
        exit(1);
    }

    check_keyword(fp_arch, buf, "opin_switch:");
    ptr = get_middle_token(fp_arch, buf);
    seg_ptr->opin_switch = my_atoi(ptr);

    /* Note:  last two switch types are generated automatically.  Shouldn't be *
     * used in the architecture file.                                          */
    if (seg_ptr->opin_switch < 0 || seg_ptr->opin_switch >= num_switch - 2) {
        printf("Error on line %d:  opin_switch value (%d) is out of range.\n",
               linenum, seg_ptr->opin_switch);
        exit(1);
    }

    check_keyword(fp_arch, buf, "Frac_cb:");
    ptr = get_middle_token(fp_arch, buf);
    seg_ptr->frac_cb = atof(ptr);

    if (seg_ptr->frac_cb < 0. || seg_ptr->frac_cb > 1.) {
        printf("Error on line %d:  Frac_cb value (%g) is out of range.\n",
               linenum, seg_ptr->frac_cb);
        exit(1);
    }

    check_keyword(fp_arch, buf, "Frac_sb:");
    ptr = get_middle_token(fp_arch, buf);
    seg_ptr->frac_sb = atof(ptr);

    if (seg_ptr->frac_sb < 0. || seg_ptr->frac_sb > 1.) {
        printf("Error on line %d:  Frac_sb value (%g) is out of range.\n",
               linenum, seg_ptr->frac_sb);
        exit(1);
    }

    if (seg_ptr->longline == FALSE) {
        /* Need at least two switch boxes along the length of non-longline segments *
         * to ensure all switches line up (at least for planar sboxes).             */
        num_sb = nint((seg_ptr->length + 1) * seg_ptr->frac_sb);

        if (num_sb < 2) {
            printf("Error on line %d:  Frac_sb value results in only %d switch "
                   "boxes.\n"
                   "Minimum 2 switch boxes on non-longline segments.\n", linenum,
                   num_sb);
            exit(1);
        }
    }

    check_keyword(fp_arch, buf, "Rmetal:");
    ptr = get_middle_token(fp_arch, buf);
    seg_ptr->Rmetal = atof(ptr);

    if (seg_ptr->Rmetal < 0.) {
        printf("Error on line %d:  Rmetal value (%g) is out of range.\n",
               linenum, seg_ptr->Rmetal);
        exit(1);
    }

    check_keyword(fp_arch, buf, "Cmetal:");
    ptr = get_last_token(fp_arch, buf);
    seg_ptr->Cmetal = atof(ptr);

    if (seg_ptr->Cmetal < 0.) {
        printf("Error on line %d:  Cmetal value (%g) is out of range.\n",
               linenum, seg_ptr->Cmetal);
        exit(1);
    }
}


static void get_switch_inf(FILE* fp_arch, char* buf, int num_switch,
                           router_types_t route_type)
{
    /* Loads up all the switch information.                                     */
    char* ptr;
    if (route_type != DETAILED) {
        ptr = my_strtok(NULL, TOKENS, fp_arch, buf);

        while (ptr != NULL) {     /* Skip to the end of the line. */
            ptr = my_strtok(NULL, TOKENS, fp_arch, buf);
        }

        return;
    }

    ptr = get_middle_token(fp_arch, buf);
    int index = my_atoi(ptr);
    if (index < 0) {
        printf("Error on line %d:  switch number (%d) is out of range.\n",
               linenum, index);
        exit(1);
    }

    if (index >= num_switch) {
        printf("Error on line %d:  switch numbers are not consecutive or do \n"
               "\tnot start at 0.\n", linenum);
        exit(1);
    }

    if (switch_inf[index].R >= 0.) {
        printf("Error on line %d:  switch %d properties already set.\n", linenum,
               index);
        exit(1);
    }

    check_keyword(fp_arch, buf, "buffered:");
    ptr = get_middle_token(fp_arch, buf);

    if (strcmp(ptr, "yes") == 0) {
        switch_inf[index].buffered = TRUE;
    } else if (strcmp(ptr, "no") == 0) {
        switch_inf[index].buffered = FALSE;
    } else {
        printf("Error on line %d:  invalid buffered value: %s.\n", linenum, ptr);
        exit(1);
    }

    check_keyword(fp_arch, buf, "R:");
    ptr = get_middle_token(fp_arch, buf);
    switch_inf[index].R = atof(ptr);

    if (switch_inf[index].R < 0) {
        printf("Error on line %d:  resistance value (%g) is negative.\n",
               linenum, switch_inf[index].R);
        exit(1);
    }

    check_keyword(fp_arch, buf, "Cin:");
    ptr = get_middle_token(fp_arch, buf);
    switch_inf[index].Cin = atof(ptr);

    if (switch_inf[index].Cin < 0) {
        printf("Error on line %d:  capacitance value (%g) is negative.\n",
               linenum, switch_inf[index].Cin);
        exit(1);
    }

    check_keyword(fp_arch, buf, "Cout:");
    ptr = get_middle_token(fp_arch, buf);
    switch_inf[index].Cout = atof(ptr);

    if (switch_inf[index].Cout < 0) {
        printf("Error on line %d:  capacitance value (%g) is negative.\n",
               linenum, switch_inf[index].Cout);
        exit(1);
    }

    if (switch_inf[index].buffered == FALSE && switch_inf[index].Cout
            != switch_inf[index].Cin) {
        printf("Error on line %d:  Cin (%g) and Cout (%g) differ for a "
               "pass transitor (switch #%d).\n", linenum, switch_inf[index].Cin,
               switch_inf[index].Cout, index);
        exit(1);
    }

    check_keyword(fp_arch, buf, "Tdel:");
    ptr = get_last_token(fp_arch, buf);
    switch_inf[index].Tdel = atof(ptr);

    if (switch_inf[index].Tdel < 0) {
        printf("Error on line %d:  Tdel value (%g) is negative.\n",
               linenum, switch_inf[index].Tdel);
        exit(1);
    }
}


/* Loads up the one segment type (unit length segment) and one switch type   *
 * (pass transistor) needed to make the graph builder allow global routing.  *
 * Also sets the switch blocks type to SUBSET (works for global routing).     */
static void load_global_segment_and_switch(detail_routing_arch_t* det_routing_arch,
                                           segment_info_t* segment_inf,
                                           timing_info_t* timing_inf)
{
    det_routing_arch->switch_block_type = SUBSET;
    det_routing_arch->Fc_output = 1.;
    det_routing_arch->Fc_input = 1.;
    det_routing_arch->Fc_pad = 1.;
    det_routing_arch->Fc_type = ABSOLUTE;
    det_routing_arch->Tdelless_switch = 0;
    det_routing_arch->wire_to_ipin_switch = 0;
    segment_inf[0].frequency = 1.;
    segment_inf[0].length = 1;
    segment_inf[0].wire_switch = 0;
    segment_inf[0].opin_switch = 0;
    segment_inf[0].frac_cb = 1.;
    segment_inf[0].frac_sb = 1.;
    segment_inf[0].longline = FALSE;
    segment_inf[0].Rmetal = 0.;
    segment_inf[0].Cmetal = 0.;
    switch_inf[0].buffered = TRUE;
    switch_inf[0].R = 0.;
    switch_inf[0].Cin = 0.;
    switch_inf[0].Cout = 0.;
    switch_inf[0].Tdel = 1.;  /* Need increasing Tdel with distance from      */
    /* source or the timing-driven router won't work for global routing. */
}


static void load_extra_switch_types(detail_routing_arch_t *
                                    det_routing_arch, timing_info_t* timing_inf)
{
    /* Adds two extra switches to the routing architecture. One is a zero Tdel, *
     * zero C switch that I use to make connections that should have no Tdel.   *
     * The other is a switch that models the buffer + connection blocks Tdel     *
     * that occurs when one goes from wire segments to IPINs.                    */
    int Tdelless_switch, wire_to_ipin_switch;
    Tdelless_switch = det_routing_arch->num_switch - 1;
    det_routing_arch->Tdelless_switch = Tdelless_switch;
    wire_to_ipin_switch = det_routing_arch->num_switch - 2;
    det_routing_arch->wire_to_ipin_switch = wire_to_ipin_switch;

    if (switch_inf[Tdelless_switch].R >= 0. ||
            switch_inf[wire_to_ipin_switch].R >= 0.) {
        printf("Error:  switch indices are not consecutive or do not start at 0."
               "\n");
        exit(1);
    }

    switch_inf[Tdelless_switch].buffered = TRUE;
    switch_inf[Tdelless_switch].R = 0.;
    switch_inf[Tdelless_switch].Cin = 0.;
    switch_inf[Tdelless_switch].Cout = 0.;
    switch_inf[Tdelless_switch].Tdel = 0.;
    switch_inf[wire_to_ipin_switch].buffered = TRUE;
    switch_inf[wire_to_ipin_switch].R = 0.;
    switch_inf[wire_to_ipin_switch].Cin = timing_inf->C_ipin_cblock;
    switch_inf[wire_to_ipin_switch].Cout = 0.;
    switch_inf[wire_to_ipin_switch].Tdel = timing_inf->T_ipin_cblock;
}

static void check_keyword(FILE* fp,
                          char* buf,
                          const char* keyword)
{
    /* Checks that the next token (which must be a middle token) is the proper *
     * keyword.                                                                */
    char* ptr = get_middle_token(fp, buf);
    if (strcmp(ptr, keyword) != 0) {
        printf("Error on line %d:  Expected keyword %s, got %s.\n", linenum,
               keyword, ptr);
        exit(1);
    }
}


static char* get_middle_token(FILE* fp, char* buf)
{
    /* Gets the next token and prints an error message if it is the last one.   *
     * This token also can't be the first token.                                */
    char* ptr = my_strtok(NULL, TOKENS, fp, buf);
    if (ptr == NULL) {
        printf("Error:  line %d is incomplete.\n", linenum);
        exit(1);
    }

    return (ptr);
}


static char* get_last_token(FILE* fp, char* buf)
{
    /* Gets the next token and prints an error message if it isn't the last one. */
    char* ptr = get_middle_token(fp, buf);

    char* ptr2 = my_strtok(NULL, TOKENS, fp, buf);
    if (ptr2 != NULL) {
        printf("Error:  Extra characters at end of line %d.\n", linenum);
        exit(1);
    }

    return (ptr);
}


static int get_int(char* ptr, int inp_num, FILE* fp_arch, char* buf,
                   int min_val)
{
    /* This routine gets the next integer on the line.  It must be greater *
     * than or equal to min_val or an error message is printed.            */
    ptr = my_strtok(NULL, TOKENS, fp_arch, buf);
    if (ptr == NULL) {
        printf("Error:  missing %s value on line %d.\n",
               names[inp_num], linenum);
        exit(1);
    }

    int val = my_atoi(ptr);
    if (val < min_val) {
        printf("Error:  Bad value.  %s = %d on line %d.\n",
               names[inp_num], val, linenum);
        exit(1);
    }

    ptr = my_strtok(NULL, TOKENS, fp_arch, buf);

    if (ptr != NULL) {
        printf("Error:  extra characters at end of line %d.\n", linenum);
        exit(1);
    }

    isread[inp_num]++;
    return(val);
}


static double get_one_double(char* ptr, int inp_num, double low_lim,
                             double upp_lim, FILE* fp_arch, char* buf)
{
    /* This routine gets one doubleing point number from ptr.  It checks that     *
     * there are no extra characters at the end of the line, and updates isread. *
     * Use this routine when you need to only read "var = double" type lines.     */
    double val;
    val = get_double(ptr, inp_num, low_lim, upp_lim, fp_arch, buf);
    ptr = my_strtok(NULL, TOKENS, fp_arch, buf);

    if (ptr != NULL) {
        printf("Error:  extra characters at end of line %d.\n", linenum);
        exit(1);
    }

    isread[inp_num]++;
    return (val);
}


static double get_double(char* ptr, int inp_num, double low_lim,
                       double upp_lim, FILE* fp_arch, char* buf)
{
    /* This routine gets the doubleing point number that is next on the line. *
     * low_lim and upp_lim specify the allowable range of numbers, while     *
     * inp_num gives the type of input line being parsed.                    */
    double val;
    ptr = my_strtok(NULL, TOKENS, fp_arch, buf);

    if (ptr == NULL) {
        printf("Error:  missing %s value on line %d.\n",
               names[inp_num], linenum);
        exit(1);
    }

    val = atof(ptr);

    if (val <= low_lim || val > upp_lim) {
        printf("Error:  Bad value parsing %s. %g on line %d.\n",
               names[inp_num], val, linenum);
        exit(1);
    }

    return(val);
}


/* Order:  chan_width_x [gaussian|uniform|pulse] peak <width>  *
 * <xpeak> <dc>.  (Bracketed quantities needed only for pulse  *
 * and gaussian).  All values from 0 to 1, except peak and dc, *
 * which can be anything.                                      *
 * Other possibility:  chan_width_x delta peak xpeak dc        */
static void gechannel_t(char* ptr, channel_t* chan, int inp_num, FILE* fp_arch,
                        char* buf)
{
    /* This routine parses a channel functional description line.  chan  *
     * is the channel data structure to be loaded, while inp_num is the  *
     * type of input line being parsed.                                  */
    ptr = my_strtok(NULL, TOKENS, fp_arch, buf);

    if (ptr == NULL) {
        printf("Error:  missing %s value on line %d.\n",
               names[inp_num], linenum);
        exit(1);
    }

    if (strcmp(ptr, "uniform") == 0) {
        isread[inp_num]++;
        chan->type = UNIFORM;
        chan->peak = get_double(ptr, inp_num, 0., 1., fp_arch, buf);
        chan->dc = OPEN;
        chan->width = OPEN;
        chan->xpeak = OPEN;
    } else if (strcmp(ptr, "delta") == 0) {
        isread[inp_num]++;
        chan->type = DELTA;
        chan->peak = get_double(ptr, inp_num, -1.e5, 1.e5, fp_arch, buf);
        chan->xpeak = get_double(ptr, inp_num, -1e-30, 1., fp_arch, buf);
        chan->dc = get_double(ptr, inp_num, -1e-30, 1., fp_arch, buf);
        chan->width = OPEN;
    } else {
        if (strcmp(ptr, "gaussian") == 0) {
            chan->type = GAUSSIAN;
        }

        if (strcmp(ptr, "pulse") == 0) {
            chan->type = PULSE;
        }

        if (chan->type == GAUSSIAN || chan->type == PULSE) {
            isread[inp_num]++;
            chan->peak = get_double(ptr, inp_num, -1., 1., fp_arch, buf);
            chan->width = get_double(ptr, inp_num, 0., 1.e10, fp_arch, buf);
            chan->xpeak = get_double(ptr, inp_num, -1e-30, 1., fp_arch, buf);
            chan->dc = get_double(ptr, inp_num, -1e-30, 1., fp_arch, buf);
        }
    }

    if (isread[inp_num] == 0) {
        printf("Error:  %s distribution keyword: %s unknown.\n",
               names[inp_num], ptr);
        exit(1);
    }

    if (my_strtok(NULL, TOKENS, fp_arch, buf) != NULL) {
        printf("Error:  extra value for %s at end of line %d.\n",
               names[inp_num], linenum);
        exit(1);
    }
}


/* This routine checks that the input architecture file makes sense and *
 * specifies all the needed parameters.  The parameters must also be    *
 * self-consistent and make sense.                                      */
static void check_arch(char* arch_file,
                       router_types_t route_type,
                       detail_routing_arch_t det_routing_arch,
                       segment_info_t* segment_inf,
                       timing_info_t timing_inf,
                       int max_subblocks_per_block,
                       chan_width_distr_t chan_width_dist)
{
    /* NUMINP parameters can be set in the architecture file.  The first      *
     * NUM_REQUIRED are always mandatory.  The next NUM_DETAILED ones are     *
     * needed only if detailed routing is going to be performed.  The last    *
     * NUM_TIMING ones are needed only if timing analysis is going to be      *
     * performed.  Expect the corresponding isread for each parameter to be   *
     * 1, except isread[4] (outpin), isread[5] (inpin), isread[13] (segment)  *
     * and isread[14] (switch)  which should all be greater than 0.           */
    boolean must_be_set[NUMINP];
    int i = -1;
    for (i = 0; i < NUM_REQUIRED; i++) {
        must_be_set[i] = TRUE;
    }

    for (i = NUM_REQUIRED; i < NUMINP; i++) {
        must_be_set[i] = FALSE;
    }

    if (route_type == DETAILED) {
        for (i = NUM_REQUIRED; i < NUM_REQUIRED + NUM_DETAILED; i++) {
            must_be_set[i] = TRUE;
        }
    }

    if (timing_inf.timing_analysis_enabled) {
        for (i = NUM_REQUIRED + NUM_DETAILED; i < NUMINP; i++) {
            must_be_set[i] = TRUE;
        }
    }

    int fatal = 0;
    for (i = 0; i < NUMINP; i++) {
        if (!must_be_set[i]) {
            continue;
        }

        if (i == TIMING_START + 7) {   /* T_subblock lines */
            if (isread[i] < 1 || isread[i] != max_subblocks_per_block) {
                printf("Error: Got %d T_subblock lines -- expected %d.\n",
                       isread[i], max_subblocks_per_block);
                fatal = 1;
            }
        } else if (i != 4 && i != 5 && i != DETAILED_START + 5 && i !=
                   DETAILED_START + 6) {
            if (isread[i] == 0) {
                printf("Error:  %s not set in file %s.\n", names[i],
                       arch_file);
                fatal = 1;
            }

            if (isread[i] > 1) {
                printf("Error:  %s set %d times in file %s.\n", names[i],
                       isread[i], arch_file);
                fatal = 1;
            }
        } else {  /* outpin, inpin, segment, or switch lines */
            if (isread[i] < 1) {
                printf("Error:  in file %s.  Clb has %d %s(s).\n", arch_file,
                       isread[i], names[i]);
                fatal = 1;
            }
        }
    }

    /* Segment info is used for both GLOBAL and DETAILED routing. */
    double total_segment_freq = 0.0;
    int  opin_switch = 0;
    for (i = 0; i < det_routing_arch.num_segment; i++) {
        total_segment_freq += segment_inf[i].frequency;
        opin_switch = segment_inf[i].opin_switch;

        if (switch_inf[opin_switch].buffered == FALSE) {
            printf("Error in check_arch:  opin_switch (#%d) of segment type #%d "
                   "is not buffered.\n", opin_switch, i);
            exit(1);
        }
    }

    if (fabs(total_segment_freq - 1.) > 0.001) {
        printf("Error in check_arch:  Segment frequencies must sum to 1.\n"
               "\tSum is %g.\n", total_segment_freq);
        fatal = 1;
    }

    /* Detailed routing is only supported on architectures with all channel  *
     * widths the same for now.  The router could handle non-uniform widths, *
     * but the routing resource graph generator doesn't build the rr_graph   *
     * for the nonuniform case as yet.                                       */
    channel_t chan_x_dist, chan_y_dist;
    double chan_width_io = 0.0;
    if (route_type == DETAILED) {
        chan_width_io = chan_width_dist.chan_width_io;
        chan_x_dist = chan_width_dist.chan_x_dist;
        chan_y_dist = chan_width_dist.chan_y_dist;

        if (chan_x_dist.type != UNIFORM || chan_y_dist.type != UNIFORM ||
                chan_x_dist.peak != chan_y_dist.peak || chan_x_dist.peak !=
                chan_width_io) {
            printf("Error in check_arch:  detailed routing currently only\n"
                   "supported on FPGAs with all channels of equal width.\n");
            fatal = 1;
        }

        if (det_routing_arch.Fc_type == ABSOLUTE) {
            if (det_routing_arch.Fc_output < 1 || det_routing_arch.Fc_input < 1
                    || det_routing_arch.Fc_pad < 1) {
                printf("Error in check_arch:  Fc values must be >= 1 in absolute "
                       "mode.\n");
                fatal = 1;
            }
        } else { /* FRACTIONAL mode */
            if (det_routing_arch.Fc_output > 1. || det_routing_arch.Fc_input > 1.
                    || det_routing_arch.Fc_pad > 1.) {
                printf("Error in check_arch:  Fc values must be <= 1. in "
                       "fractional mode.\n");
                fatal = 1;
            }
        }

        for (i = 0; i < det_routing_arch.num_switch; i++) {
            if (switch_inf[i].buffered) {
                /* Largest resistance tri-state buffer would have a minimum width     *
                 * transistor in the buffer pull-down and a min-width pass transistor *
                 * on the output.  Hence largest R = 2 * largest_transistor_R.        */
                if (switch_inf[i].R > 2 * det_routing_arch.R_minW_nmos) {
                    printf("Error in check_arch:  Switch %d R value (%g) is greater"
                           " than 2 * R_minW_nmos (%g).\n", i, switch_inf[i].R,
                           2 * det_routing_arch.R_minW_nmos);
                    exit(1);
                }
            } else {  /* Pass transistor switch */
                if (switch_inf[i].R > det_routing_arch.R_minW_nmos) {
                    printf("Error in check_arch:  Switch %d R value (%g) is greater "
                           "than R_minW_nmos (%g).\n", i, switch_inf[i].R,
                           det_routing_arch.R_minW_nmos);
                    exit(1);
                }
            }
        }     /* End for all switches. */
    }     /* End if route_type == DETAILED */

    if (fatal) {
        exit(1);
    }
}


void print_arch(char* arch_file,
                router_types_t route_type,
                detail_routing_arch_t det_routing_arch,
                segment_info_t* segment_inf,
                timing_info_t timing_inf,
                subblock_data_t subblock_data,
                chan_width_distr_t chan_width_dist)
{
    /* Prints out the architectural parameters for verification in the  *
     * file "arch.echo."  The name of the architecture file is passed   *
     * in and is printed out as well.                                   */
    int i, j;
    FILE* fp = my_fopen("arch.echo", "w");
    double chan_width_io = chan_width_dist.chan_width_io;
    channel_t chan_x_dist = chan_width_dist.chan_x_dist;
    channel_t chan_y_dist = chan_width_dist.chan_y_dist;
    fprintf(fp, "Input netlist file: %s\n\n", arch_file);
    fprintf(fp, "io_ratio: %d.\n", io_ratio);
    fprintf(fp, "chan_width_io: %g  pins_per_clb(pins per clb): %d\n",
            chan_width_io, pins_per_clb);
    fprintf(fp, "\n\nChannel Types:  UNIFORM = %d; GAUSSIAN = %d; PULSE = %d;"
            " DELTA = %d\n\n", UNIFORM, GAUSSIAN, PULSE, DELTA);
    fprintf(fp, "\nchan_width_x:\n");
    fprintf(fp, "type: %d  peak: %g  width: %g  xpeak: %g  dc: %g\n",
            chan_x_dist.type, chan_x_dist.peak, chan_x_dist.width,
            chan_x_dist.xpeak, chan_x_dist.dc);
    fprintf(fp, "\nchan_width_y:\n");
    fprintf(fp, "type: %d  peak: %g  width: %g  xpeak: %g  dc: %g\n\n",
            chan_y_dist.type, chan_y_dist.peak, chan_y_dist.width,
            chan_y_dist.xpeak, chan_y_dist.dc);
    fprintf(fp, "Pin #\tclass\ttop\tbottom\tleft\tright\tglobal");

    for (i = 0; i < pins_per_clb; i++) {
        fprintf(fp, "\n%d\t%d\t", i, clb_pin_class[i]);

        for (j = 0; j <= 3; j++) {
            fprintf(fp, "%d\t", pinloc[j][i]);
        }

        fprintf(fp, "%d", is_global_clb_pin[i]);
    }

    fprintf(fp, "\n\nClass types:  DRIVER = %d; RECEIVER = %d\n\n", DRIVER,
            RECEIVER);
    fprintf(fp, "Class\tType\tNumpins\tPins");

    for (i = 0; i < num_pin_class; i++) {
        fprintf(fp, "\n%d\t%d\t%d\t", i, class_inf[i].type, class_inf[i].num_pins);

        for (j = 0; j < class_inf[i].num_pins; j++) {
            fprintf(fp, "%d\t", class_inf[i].pinlist[j]);
        }
    }

    fprintf(fp, "\n\n");
    fprintf(fp, "subblocks_per_clb (maximum): %d\n",
            subblock_data.max_subblocks_per_block);
    fprintf(fp, "subblock_lut_size: %d\n", subblock_data.subblock_lut_size);

    if (route_type == DETAILED) {
        fprintf(fp, "\n");

        if (det_routing_arch.Fc_type == ABSOLUTE) {
            fprintf(fp, "Fc value is absolute number of tracks.\n");
        } else {
            fprintf(fp, "Fc value is fraction of tracks in a channel.\n");
        }

        fprintf(fp, "Fc_output: %g.  Fc_input: %g.  Fc_pad: %g.\n",
                det_routing_arch.Fc_output, det_routing_arch.Fc_input,
                det_routing_arch.Fc_pad);

        if (det_routing_arch.switch_block_type == SUBSET) {
            fprintf(fp, "switch_block_type: SUBSET.\n");
        } else if (det_routing_arch.switch_block_type == WILTON) {
            fprintf(fp, "switch_block_type: WILTON.\n");
        } else {
            fprintf(fp, "switch_block_type: UNIVERSAL.\n");
        }
    }

    /* Segmentation info. useful even if route_type == GLOBAL */
    fprintf(fp, "\nnum_segment: %d,  num_switch: %d.\n",
            det_routing_arch.num_segment, det_routing_arch.num_switch);

    if (route_type == DETAILED) {
        fprintf(fp, "(Two switch types were generated automatically.)\n");
    }

    fprintf(fp, "#%d:  Tdelless switch.  #%d:  wire_to_ipin_switch.\n",
            det_routing_arch.Tdelless_switch,
            det_routing_arch.wire_to_ipin_switch);
    fprintf(fp, "\nSeg. #\tfreq.\tlength\tlongln\topin_sw\twire_sw\tFrac_cb\t"
            "Frac_sb\tCmetal\tRmetal\n");

    for (i = 0; i < det_routing_arch.num_segment; i++)
        fprintf(fp, "%d\t%g\t%d\t%d\t%d\t%d\t%g\t%g\t%g\t%g\n", i,
                segment_inf[i].frequency, segment_inf[i].length,
                segment_inf[i].longline, segment_inf[i].opin_switch,
                segment_inf[i].wire_switch, segment_inf[i].frac_cb,
                segment_inf[i].frac_sb, segment_inf[i].Cmetal, segment_inf[i].Rmetal);

    fprintf(fp, "\nSwitch#\tbuff?\tR\tCin\tCout\tTdel\n");

    for (i = 0; i < det_routing_arch.num_switch; i++)
        fprintf(fp, "%d\t%d\t%g\t%g\t%g\t%g\n", i, switch_inf[i].buffered,
                switch_inf[i].R, switch_inf[i].Cin, switch_inf[i].Cout,
                switch_inf[i].Tdel);

    fprintf(fp, "\n\nR_minW_nmos: %g  R_minW_pmos: %g\n",
            det_routing_arch.R_minW_nmos, det_routing_arch.R_minW_pmos);

    T_subblock_t T_subblock;
    if (timing_inf.timing_analysis_enabled) {
        fprintf(fp, "\n\nTiming information:\n");
        fprintf(fp, "---------------------------------------------------------\n");
        fprintf(fp, "C_ipin_cblock: %g (F) \nT_ipin_cblock: %g (s)\n\n",
                timing_inf.C_ipin_cblock, timing_inf.T_ipin_cblock);
        fprintf(fp, "T_sblk_opin_to_sblk_ipin: %g (s)\n",
                timing_inf.T_sblk_opin_to_sblk_ipin);
        fprintf(fp, "T_clb_ipin_to_sblk_ipin: %g (s)\n",
                timing_inf.T_clb_ipin_to_sblk_ipin);
        fprintf(fp, "T_sblk_opin_to_clb_opin: %g (s)\n",
                timing_inf.T_sblk_opin_to_clb_opin);
        fprintf(fp, "T_ipad: %g (s)  \nT_opad: %g (s)\n",
                timing_inf.T_ipad, timing_inf.T_opad);
        fprintf(fp, "\nSubblock #\tT_comb (s)\tT_seq_in (s)\tT_seq_out (s)\n");

        for (i = 0; i < subblock_data.max_subblocks_per_block; i++) {
            T_subblock = timing_inf.T_subblock[i];
            fprintf(fp, "%10d\t%10g\t%10g\t%10g (s)\n", i, T_subblock.T_comb,
                    T_subblock.T_seq_in, T_subblock.T_seq_out);
        }
    }

    fclose(fp);
}


/* Allocates various data structures that depend on the FPGA         *
 * architecture.  Aspect_ratio specifies how many columns there are  *
 * relative to the number of rows -- i.e. width/height.  Used-sized  *
 * is TRUE if the user specified num_grid_columns and num_grid_rows already;
 * in that case use the user's values and don't recompute them.      */
void init_arch(double aspect_ratio, boolean user_sized)
{
    /* User specified the dimensions on the command line.  Check if they *
     * will fit the circuit.                                             */
    if (user_sized == TRUE) {
        /* clb didn't include the pheriperial io pads */
        if (num_clbs > num_grid_columns * num_grid_rows
              ||num_primary_inputs + num_primary_outputs >
                  io_ratio * (2 * (num_grid_columns + num_grid_rows))) {
            printf("Error:  User-specified size is too small for circuit.\n");
            exit(1);
        }
    } else {
        /* Size the FPGA automatically to be smallest that will fit circuit */
        /* Area = num_grid_columns * num_grid_rows = num_grid_rows * num_grid_rows * aspect_ratio                  *
         * Perimeter = 2 * (num_grid_columns + num_grid_rows) = 2 * num_grid_rows * (1. + aspect_ratio)  */
        num_grid_rows = (int)ceil(sqrt((double)(num_clbs / aspect_ratio)));
        int io_lim = (int)ceil((num_primary_inputs + num_primary_outputs) /
                                       (2 * io_ratio * (1.0 + aspect_ratio)));
        num_grid_rows = max(num_grid_rows, io_lim);
        num_grid_columns = (int)ceil(num_grid_rows * aspect_ratio);
    }

    /* If both num_grid_columns and num_grid_rows are 1, we only have one valid *
     * location for a clb. That's a major problem, as won't be able to move *
     * the clb and the find_to routine that tries moves in the placer will  *
     * go into an infinite loop trying to move it.  Exit with an error message*
     * instead.                                                            */
    if (num_grid_columns == 1  && num_grid_rows == 1 && num_clbs != 0) {
        printf("Error:\n");
        printf("Sorry, can't place a circuit with only one valid location\n");
        printf("for a logic blocks (clb).\n");
        printf("Try me with a more realistic circuit!\n");
        exit(1);
    }

    /* To remove this limitation, change ylow etc. in rr_node_t to        *
     * be ints instead.  Used shorts to save memory.                      */
    if (num_grid_columns > 32766 || num_grid_rows > 32766) {
        printf("Error:  num_grid_columns and num_grid_rows must be less than 32767, since the \n");
        printf("router uses shorts (16-bit) to store coordinates.\n");
        printf("num_grid_columns: %d.  num_grid_rows: %d.\n", num_grid_columns, num_grid_rows);
        exit(1);
    }

    clb_grids = (grid_tile_t**)alloc_matrix(0, num_grid_columns + 1,
                                            0, num_grid_rows + 1,
                                            sizeof(grid_tile_t));
    chan_width_x = (int*)my_malloc((num_grid_rows + 1) * sizeof(int));
    chan_width_y = (int*)my_malloc((num_grid_columns + 1) * sizeof(int));

    init_grid_capacity();

    fill_arch();
}  /* end of void init_arch(double aspect_ratio, boolean user_sized) */

/* Fill some of the FPGA architecture data structures.         */
static void fill_arch(void)
{
    /* allocate io_blocks arrays. Done this way to save storage */
    /* int  total_io_locations = 2 * io_ratio * (num_grid_columns + num_grid_rows);
    int* index = (int*)my_malloc(total_io_locations * sizeof(int)); */
    const int kio_bin_capacity = g_grid_capacity[IO_TYPE];
    int i = 0;
    int k = -1;
    for (i = 1; i <= num_grid_columns; ++i) {
        clb_grids[i][0].in_blocks = (int*)my_malloc(kio_bin_capacity * sizeof(int));
        clb_grids[i][num_grid_rows + 1].in_blocks =
            (int*)my_malloc(kio_bin_capacity * sizeof(int));
        for (k = 0; k < kio_bin_capacity; ++k) {
            clb_grids[i][0].in_blocks[k] =
              clb_grids[i][num_grid_rows + 1].in_blocks[k] = EMPTY;
        }
    }

    for (i = 1; i <= num_grid_rows; ++i) {
        clb_grids[0][i].in_blocks = (int*)my_malloc(kio_bin_capacity * sizeof(int));
        clb_grids[num_grid_columns + 1][i].in_blocks =
            (int*)my_malloc(kio_bin_capacity * sizeof(int));
        for (k = 0; k < kio_bin_capacity; ++k) {
            clb_grids[0][i].in_blocks[k] =
              clb_grids[num_grid_columns + 1][i].in_blocks[k] = EMPTY;
        }
    }

    /* Initialize type, and occupancy. */
    for (i = 1; i <= num_grid_columns; ++i) {
        clb_grids[i][0].grid_type = clb_grids[i][num_grid_rows + 1].grid_type
          = IO_TYPE;
        clb_grids[i][0].m_offset = clb_grids[i][num_grid_rows + 1].m_offset = 0;
        clb_grids[i][0].m_usage = clb_grids[i][num_grid_rows + 1].m_usage = 0;
        clb_grids[i][0].m_capacity = clb_grids[i][num_grid_rows + 1].m_capacity
          = kio_bin_capacity;
    }

    for (i = 1; i <= num_grid_rows; ++i) {
        clb_grids[0][i].grid_type = clb_grids[num_grid_columns + 1][i].grid_type
          = IO_TYPE;
        clb_grids[0][i].m_offset = clb_grids[num_grid_columns + 1][i].m_offset = 0;
        clb_grids[0][i].m_usage = clb_grids[num_grid_columns + 1][i].m_usage = 0;
        clb_grids[0][i].m_capacity = clb_grids[num_grid_columns + 1][i].m_capacity
          = kio_bin_capacity;
    }

    const int kclb_bin_capacity = g_grid_capacity[B_CLB_TYPE];
    for (i = 1; i <= num_grid_columns; ++i) { /* interior (LUT) cells */
        int j = 0;
        for (j = 1; j <= num_grid_rows; ++j) {
            clb_grids[i][j].in_blocks =
              (int*)my_malloc(kclb_bin_capacity * sizeof(int));
            clb_grids[i][j].grid_type = B_CLB_TYPE;
            clb_grids[i][j].m_capacity = kclb_bin_capacity;
            clb_grids[i][j].m_usage = 0;
            clb_grids[i][j].m_offset = 0;
            for (k = 0; k < kclb_bin_capacity; ++k) {
                clb_grids[i][j].in_blocks[k] = EMPTY;
            }
        }
    }

    /* Nothing goes in the corners. */
    clb_grids[0][0].grid_type = clb_grids[num_grid_columns + 1][0].grid_type =
        EMPTY_TYPE;
    clb_grids[0][0].m_capacity = clb_grids[num_grid_columns + 1][0].m_capacity
      = 0;
    clb_grids[0][0].m_offset = clb_grids[num_grid_columns + 1][0].m_offset
      = 0;

    clb_grids[0][num_grid_rows + 1].grid_type =
        clb_grids[num_grid_columns + 1][num_grid_rows + 1].grid_type = EMPTY_TYPE;
    clb_grids[0][num_grid_rows + 1].m_capacity =
      clb_grids[num_grid_columns + 1][num_grid_rows + 1].m_capacity = 0;
    clb_grids[0][num_grid_rows + 1].m_offset =
      clb_grids[num_grid_columns + 1][num_grid_rows + 1].m_offset = 0;
} /* end of static void fill_arch(void) */

static void init_grid_capacity(void)
{
    g_num_grid_types = NUM_GRID_TYPES;
    g_grid_capacity = (int*)my_malloc(g_num_grid_types * sizeof(int));
    g_grid_capacity[B_CLB_TYPE] = 1;

    assert(io_ratio > 0);
    g_grid_capacity[IO_TYPE] = io_ratio;

    g_grid_capacity[EMPTY_TYPE] = 0;
} /* attention, Do not forget free memory after placement! */

