#include <string.h>
#include <stdio.h>
#include "util.h"
#include "vpr_types.h"
#include "globals.h"
#include "read_netlist.h"
#include "hash.h"
#include "check_netlist.h"
#include <assert.h>


/* This source file reads in a .net file.  A .net file is a netlist   *
 * format defined by me to allow arbitary logic blocks (clbs) to      *
 * make up a netlist.  There are three legal blocks types: .input,     *
 * .output, and .clb.  To define a blocks, you start a line with one   *
 * of these keywords; the function of each type of blocks is obvious.  *
 * After the blocks type keyword, you specify the name of this blocks.  *
 * The line below must start with the keyword pinlist:, and gives a   *
 * list of the nets connected to the pins of this blocks.  .input and  *
 * .output blocks are pads and have only one net connected to them.   *
 * The number of pins on a .clb blocks is specified in the arch-       *
 * itecture file, as are the equivalences between pins.  Each         *
 * clb must have the same number of pins (heterogeneous FPGAs are     *
 * currently not supported) so any pins which are not used on a clb   *
 * must be identified with the reserved word "open".  All keywords    *
 * must be lower case.                                                *
 *                                                                    *
 * The lines immediately below the pinlist line must specify the      *
 * contents of the clb.  Each .subblock line lists the name of the    *
 * subblock, followed by the clb pin number or subblock output to     *
 * which each subblock pin should connect.  Each subblock is assummed *
 * to consist of a LUT with subblock_lut_size inputs, a FF, and an    *
 * output.  The pin order is input1, input2, ... output, clock.  The  *
 * architecture file sets the number of subblocks per clb and the LUT *
 * size used.  Subblocks are used only for timing analysis.  An       *
 * example clb declaration is:                                        *
 *                                                                    *
 * .clb name_of_clb  # comment                                        *
 *  pinlist:  net_1 net_2 my_net net_of_mine open D open              *
 *  subblock: sub_1 0 1 2 3 open open open                            *
 *  subblock: sub_2 1 3 ble_0 open open 5 open                        *
 *                                                                    *
 * Notice that the output of the first subblock (ble_0) is used only  *
 * by the second subblock.  This is fine.                             *
 *                                                                    *
 * Ending a line with a backslash (\) means it is continued on the    *
 * line below.  A sharp sign (#) indicates the rest of a line is      *
 * a comment.                                                         *
 * The vpack program can be used to convert a flat blif netlist       *
 * into .net format.                                                  *
 *                                                                    *
 * V. Betz, Jan. 29, 1997.                                            */

/* A note about the way the character buffer, buf, is passed around. *
 * strtok does not make a local copy of the character string         *
 * initially passed to it, so you must make sure it always exists.   *
 * Hence, I just use one buffer declared in read_netlist and pass it     *
 * downstream.  Starting a new line with an automatic variable       *
 * buffer in a downstream subroutine and then returning and trying   *
 * to keep tokenizing it would cause problems since the buffer now   *
 * lies on a stale part of the stack and can be overwritten.         */


/*************************** Variables local to this module *****************/

/* Temporary storage used during parsing. */
static int* num_driver, *temp_num_pins;
static hash_t** hash_table;
static int temp_block_storage;

/* Used for memory chunking of everything except subblock data. */

static int chunk_bytes_avail = 0;
static char* chunk_next_avail_mem = NULL;


/* Subblock data can be accessed anywhere within this module.  Pointers to *
 * the main subblock data structures are passed back to the rest of the    *
 * program through the subblock_data_ptr structure passed to read_netlist.     */

static int max_subblocks_per_block;
static int subblock_lut_size;
static subblock_t** subblock_inf;
static int* num_subblocks_per_block;

/* The subblock data is put in its own "chunk" so it can be freed without  *
 * hosing the other netlist data.                                          */

static int ch_subblock_bytes_avail;
static char* ch_subblock_next_avail_mem;
static linked_vptr_t* ch_subblock_head_ptr;



/************************ Subroutines local to this module ******************/

static int add_net(char* ptr,
                   pin_types_t type,
                   int block_num,
                   int blk_pnum,
                   int doall);

static char* get_token(char* buf, int doall, FILE* fp_net);

static void add_io(int doall,
                   const block_types_t kblock_type,
                   FILE* fp_net,
                   char* buf);

static char* add_clb(int doall,
                     FILE* fp_net,
                     char* buf);

static void add_global(int doall, FILE* fp_net, char* buf);
static void init_parse(int doall);
static void free_parse(void);
static void parse_name_and_pinlist(int doall, FILE* fp_net, char* buf);
static int get_pin_number(char* ptr);

static void load_subblock_array(int doall, FILE* fp_net, char* temp_buf,
                                int num_subblocks, int block_num);

static void set_subblock_count(int block_num, int num_subblocks);
static char* parse_subblocks(int doall, FILE* fp_net, char* buf, int block_num);



/********************** Subroutine definitions *******************************
 * Main routine that parses a netlist file in my(.net) format. */
void read_netlist(char* net_file,
                  subblock_data_t* subblock_data_ptr)
{
    char buf[BUFSIZE] = "";
    /* Make two variables below accessible anywhere in the module because I'm *
     * too lazy to pass them all over the place.                              */
    max_subblocks_per_block = subblock_data_ptr->max_subblocks_per_block; /* static int */
    subblock_lut_size = subblock_data_ptr->subblock_lut_size; /* static int */
    FILE* fp_net = my_fopen(net_file, "r");

    /* First pass builds the symbol table and counts the number of pins  *
     * on each net.  Then I allocate exactly the right amount of storage *
     * for each net.  Finally, the second pass loads the blocks and net   *
     * arrays.                                                           */
    int doall = -1;
    for (doall = 0; doall <= 1; ++doall) { /* Pass number. */
        init_parse(doall);

        linenum = 0;   /* Reset line number. */
        char* ptr = my_fgets(buf, BUFSIZE, fp_net);
        while (ptr != NULL) {
            ptr = get_token(buf,
                          doall,
                          fp_net);
        }

        rewind(fp_net);   /* Start at beginning of file again */
    }

    fclose(fp_net);

    /* Return the three data structures below through subblock_data_ptr.        */
    subblock_data_ptr->subblock_inf = subblock_inf;
    subblock_data_ptr->num_subblocks_per_block = num_subblocks_per_block;
    subblock_data_ptr->chunk_head_ptr = ch_subblock_head_ptr;

    check_netlist(subblock_data_ptr,
                  num_driver);

    free_parse();
} /* end of void read_netlist() */

static void init_parse(int doall)
{
    /* Allocates and initializes the data structures needed for the parse. */
    int i, j, len, nindex, pin_count;
    int* tmp_ptr;
    struct s_hash_iterator hash_iterator;
    hash_t* h_ptr;

    if (!doall) {  /* Initialization before first (counting) pass */
        num_nets = 0;
        hash_table = alloc_hash_table();

#define INITIAL_BLOCK_STORAGE 2000
        temp_block_storage = INITIAL_BLOCK_STORAGE;
        num_subblocks_per_block = (int*)my_malloc(INITIAL_BLOCK_STORAGE *
                                            sizeof(int));

        ch_subblock_bytes_avail = 0;
        ch_subblock_next_avail_mem = NULL;
        ch_subblock_head_ptr = NULL;
    } else {
    /* Allocate memory for second (load) pass */
        net = (net_t*)my_malloc(num_nets * sizeof(net_t));
        blocks = (block_t*)my_malloc(num_blocks * sizeof(block_t));
        is_global = (boolean*)my_calloc(num_nets, sizeof(boolean));
        num_driver = (int*)my_malloc(num_nets * sizeof(int));
        temp_num_pins = (int*)my_malloc(num_nets * sizeof(int));

        for (i = 0; i < num_nets; ++i) {
            num_driver[i] = 0;
            net[i].num_net_pins = 0;
        }

        /* Allocate blocks pin connection storage.  Some is wasted for io blocks. *
         * Method used below "chunks" the malloc of a bunch of small things to   *
         * reduce the memory housekeeping overhead of malloc.                    */
        tmp_ptr = (int*)my_malloc(max_pins_per_clb * num_blocks * sizeof(int));

        for (i = 0; i < num_blocks; ++i) {
            blocks[i].nets = tmp_ptr + i * max_pins_per_clb;
        }

        /* I use my_chunk_malloc for some storage locations below.  my_chunk_malloc  *
         * avoids the 12 byte or so overhead incurred by malloc, but since I call it *
         * with a NULL head_ptr, it will not keep around enough information to ever  *
         * free these data arrays.  If you ever have compatibility problems on a     *
         * non-SPARC architecture, just change all the my_chunk_malloc calls to      *
         * my_malloc calls.                                                          */
        hash_iterator = start_hash_table_iterator();
        h_ptr = get_next_hash(hash_table, &hash_iterator);

        while (h_ptr != NULL) {
            nindex = h_ptr->index;
            pin_count = h_ptr->count;
            net[nindex].node_blocks = (int*)my_chunk_malloc(pin_count * sizeof(int),
                                                            NULL,
                                                            &chunk_bytes_avail,
                                                            &chunk_next_avail_mem);
            net[nindex].node_block_pins = (int*)my_chunk_malloc(pin_count * sizeof(int),
                                                                NULL,
                                                                &chunk_bytes_avail,
                                                                &chunk_next_avail_mem);
            /* For avoiding assigning values beyond end of pins array. */
            temp_num_pins[nindex] = pin_count;
            len = strlen(h_ptr->name);
            net[nindex].name = (char*)my_chunk_malloc((len + 1) * sizeof(char),
                                                       NULL,
                                                       &chunk_bytes_avail,
                                                       &chunk_next_avail_mem);
            strcpy(net[nindex].name, h_ptr->name);
            h_ptr = get_next_hash(hash_table, &hash_iterator);
        }

        /* Allocate storage for subblock info. (what's in each logic blocks) */
        num_subblocks_per_block = (int*)my_realloc(num_subblocks_per_block,
                                                   num_blocks * sizeof(int));
        subblock_inf = (subblock_t**)my_malloc(num_blocks *
                                               sizeof(subblock_t*));

        for (i = 0; i < num_blocks; ++i) {
            if (num_subblocks_per_block[i] == 0) {
                subblock_inf[i] = NULL;
            } else {
                subblock_inf[i] = (subblock_t*) my_chunk_malloc(
                                      num_subblocks_per_block[i] * sizeof(subblock_t),
                                      &ch_subblock_head_ptr, &ch_subblock_bytes_avail,
                                      &ch_subblock_next_avail_mem);

                for (j = 0; j < num_subblocks_per_block[i]; j++)
                    subblock_inf[i][j].inputs = (int*) my_chunk_malloc
                                                (subblock_lut_size * sizeof(int), &ch_subblock_head_ptr,
                                                 &ch_subblock_bytes_avail, &ch_subblock_next_avail_mem);
            }
        }
    } /* end of else */

    /* Initializations for both passes. */
    linenum = 0;
    num_primary_inputs = 0;
    num_primary_outputs = 0;
    num_clbs = 0;
    num_blocks = 0;
    num_globals = 0;
}

/* Figures out which, if any token is at the start of this line and *
 * takes the appropriate action.  It always returns a pointer to    *
 * the next line (I need to do this so I can do some lookahead).    */
static char* get_token(char* buf,
                       int doall,
                       FILE* fp_net)
{
    char* ptr = my_strtok(buf,
                          TOKENS,
                          fp_net,
                          buf);
    if (ptr == NULL) {  /* Empty line, skip it */
        ptr = my_fgets(buf,
                       BUFSIZE,
                       fp_net);
        return ptr;
    }
    if (strcmp(ptr, ".clb") == 0) {
        ptr = add_clb(doall, fp_net, buf);
        return ptr;
    }
    if (strcmp(ptr, ".input") == 0) {
        add_io(doall, INPAD_TYPE, fp_net, buf);
        ptr = my_fgets(buf, BUFSIZE, fp_net);
        return ptr;
    }
    if (strcmp(ptr, ".output") == 0) {
        add_io(doall, OUTPAD_TYPE, fp_net, buf);
        ptr = my_fgets(buf, BUFSIZE, fp_net);
        return ptr;
    }
    if (strcmp(ptr, ".global") == 0) {
        add_global(doall, fp_net, buf);
        ptr = my_fgets(buf, BUFSIZE, fp_net);
        return ptr;
    }

    printf("Error in get_token while parsing netlist file.\n");
    printf("Line %d starts with an invalid token (%s).\n", linenum, ptr);
    exit(1);
}

static int get_pin_number(char* ptr)
{
    /* Returns the pin number to which a subblock pin connects.  This pin number *
     * can be OPEN, one of the clb pins (from 0 to clb_size-1) or a "hidden"     *
     * pin that refers to the output of one of the subblocks within the clb      *
     * (the output of subblock 0 is pin clb_size, the output of subblock         *
     * max_subblocks_per_block is clb_size + max_subblocks_per_block-1).         */
    int val;

    if (strcmp("open", ptr) == 0) {
        return (OPEN);
    }

    if (strncmp("ble_", ptr, 4) == 0) {    /* "Hidden" pin (Subblock output) */
        val = my_atoi(ptr + 4);

        if (val < 0 || val >= max_subblocks_per_block) {
            printf("Error in get_pin_number on line %d of netlist file.\n",
                   linenum);
            printf("Pin ble_%d is out of legal range (ble_%d to ble_%d).\n"
                   "Aborting.\n\n", val, 0, max_subblocks_per_block - 1);
            exit(1);
        }

        val += max_pins_per_clb;  /* max_pins_per_clb .. max_pins_per_clb + max_subblocks-1 */
        return (val);
    }

    /* Clb input pin. */
    val = my_atoi(ptr);

    if (val < 0 || val >= max_pins_per_clb) {
        printf("Error in get_pin_number on line %d of netlist file.\n",
               linenum);
        printf("Pin %d is out of legal range (%d to %d).\nAborting.\n\n",
               val, 0, max_pins_per_clb - 1);
        exit(1);
    }

    return (val);
}


static void load_subblock_array(int doall, FILE* fp_net,
                                char* temp_buf, int num_subblocks, int block_num)
{
    /* Parses one subblock line and, if doall is 1, loads the proper   *
     * arrays.  Each subblock line is of the format:                   *
     * subblock: <name> <ipin0> <ipin1> .. <ipin[subblock_lut_size-1]> *
     *          <opin> <clockpin>                                      */
    int ipin, len, connect_to;
    char* ptr;
    ipin = 0;
    ptr = my_strtok(NULL, TOKENS, fp_net, temp_buf);

    if (ptr == NULL) {
        printf("Error in load_subblock_array on line %d of netlist file.\n",
               linenum);
        printf("Subblock name is missing.\nAborting.\n\n");
        exit(1);
    }

    /* Load subblock name if this is the load pass. */
    if (doall == 1) {
        len = strlen(ptr);
        subblock_inf[block_num][num_subblocks - 1].name = my_chunk_malloc((len + 1) *
                                                                     sizeof(char), &ch_subblock_head_ptr, &ch_subblock_bytes_avail,
                                                                     &ch_subblock_next_avail_mem);
        strcpy(subblock_inf[block_num][num_subblocks - 1].name, ptr);
    }

    ptr = my_strtok(NULL, TOKENS, fp_net, temp_buf);

    while (ptr != NULL) {    /* For each subblock pin. */
        if (doall == 1) {
            connect_to = get_pin_number(ptr);

            if (ipin < subblock_lut_size) {      /* LUT input. */
                subblock_inf[block_num][num_subblocks - 1].inputs[ipin] = connect_to;
            } else if (ipin == subblock_lut_size) { /* LUT output. */
                subblock_inf[block_num][num_subblocks - 1].output = connect_to;
            } else if (ipin == subblock_lut_size + 1) { /* Clock input. */
                subblock_inf[block_num][num_subblocks - 1].clock = connect_to;
            }
        }

        ipin++;
        ptr = my_strtok(NULL, TOKENS, fp_net, temp_buf);
    }

    if (ipin != subblock_lut_size + 2) {
        printf("Error in load_subblock_array at line %d of netlist file.\n",
               linenum);
        printf("Subblock had %d pins, expected %d.\n", ipin,
               subblock_lut_size + 2);
        printf("Aborting.\n\n");
        exit(1);
    }
}


static void set_subblock_count(int block_num, int num_subblocks)
{
    /* Sets the temporary subblock count for blocks block_num to num_subblocks. *
     * Properly allocates whatever temporary storage is needed.           */
    if (block_num >= temp_block_storage) {
        temp_block_storage *= 2;
        num_subblocks_per_block = (int*) my_realloc
                                  (num_subblocks_per_block, temp_block_storage * sizeof(int));
    }

    num_subblocks_per_block[block_num] = num_subblocks;
}


static char* parse_subblocks(int doall, FILE* fp_net, char* buf,
                             int block_num)
{
    /* Loads the subblock arrays with the proper values. */
    char temp_buf[BUFSIZE], *ptr;
    int num_subblocks;
    num_subblocks = 0;

    while (1) {
        ptr = my_fgets(temp_buf, BUFSIZE, fp_net);

        if (ptr == NULL) {
            break;    /* EOF */
        }

        /* Save line in case it's not a sublock */
        strcpy(buf, temp_buf);
        ptr = my_strtok(temp_buf, TOKENS, fp_net, temp_buf);

        if (ptr == NULL) {
            continue;    /* Blank or comment line.  Skip. */
        }

        if (strcmp("subblock:", ptr) == 0) {
            num_subblocks++;
            load_subblock_array(doall, fp_net, temp_buf, num_subblocks,
                                block_num);
        } else {
            break;  /* Subblock list has ended.  Buf contains next line. */
        }
    }   /* End infinite while */

    if (num_subblocks < 1 || num_subblocks > max_subblocks_per_block) {
        printf("Error in parse_subblocks on line %d of netlist file.\n",
               linenum);
        printf("Block #%d has %d subblocks.  Out of range.\n",
               block_num, num_subblocks);
        printf("Aborting.\n\n");
        exit(1);
    }

    if (doall == 0) {
        set_subblock_count(block_num, num_subblocks);
    } else {
        assert(num_subblocks == num_subblocks_per_block[block_num]);
    }

    return (ptr);
}


static char* add_clb(int doall, FILE* fp_net, char* buf)
{
    /* Adds the clb (.clb) currently being parsed to the blocks array. Adds *
     * its pins to the nets data structure by calling add_net. If doall is *
     * zero this is a counting pass; if it is 1 this is the final(loading) *
     * pass.                                                               */
    ++num_blocks;
    parse_name_and_pinlist(doall,
                           fp_net,
                           buf);
    ++num_clbs;
    if (doall) {
        blocks[num_blocks - 1].block_type = B_CLB_TYPE;
    }

    /* then resolve the pinlist */
    int inet = 0;
    int pin_index = -1;
    char* ptr = my_strtok(NULL,
                          TOKENS,
                          fp_net,
                          buf);
    while (ptr != NULL) {
        ++pin_index;
        if (pin_index >= max_pins_per_clb) {
            printf("Error in add_clb on line %d of netlist file.\n", linenum);
            printf("Too many pins on this clb. Expected %d.\n", max_pins_per_clb);
            exit(1);
        }

        int iclass = clb_pin_class[pin_index];
        pin_types_t type = class_inf[iclass].type; /* DRIVER or RECEIVER */

        if (strcmp(ptr, "open") != 0) { /* Pin is connected. */
            inet = add_net(ptr,
                           type,
                           num_blocks - 1,
                           pin_index,
                           doall);
            if (doall) {  /* Loading pass only */
                blocks[num_blocks - 1].nets[pin_index] = inet;
            }
        } else {  /* Pin is unconnected (open) */
            if (doall) {
                blocks[num_blocks - 1].nets[pin_index] = OPEN;
            }
        }

        ptr = my_strtok(NULL, TOKENS, fp_net, buf);
    }

    if (pin_index != max_pins_per_clb - 1) {
        printf("Error in add_clb on line %d of netlist file.\n", linenum);
        printf("Expected %d pins on clb, got %d.\n", max_pins_per_clb, pin_index + 1);
        exit(1);
    }

    ptr = parse_subblocks(doall, fp_net, buf, num_blocks - 1);
    return (ptr);
}

/* Adds the INPAD_TYPE or OUTPAD_TYPE (specified by block_type)  currently being  *
 * parsed to the blocks array.  Adds its pin to the nets data structure  *
 * by calling add_net.  If doall is zero this is a counting pass; if it *
 * is 1 this is the final (loading) pass.                               */
static void add_io(int doall,
                   const block_types_t kblock_type,
                   FILE* fp_net,
                   char* buf)
{
    ++num_blocks;
    int inet, i;
    pin_types_t pin_type;

    if (doall == 0) {
        set_subblock_count(num_blocks - 1, 0); /* No subblocks for IO_TYPE */
    }

    parse_name_and_pinlist(doall, fp_net, buf);

    if (kblock_type == INPAD_TYPE) {
        ++num_primary_inputs;
        pin_type = DRIVER;
    } else {
        ++num_primary_outputs;
        pin_type = RECEIVER;
    }

    if (doall) { /* INPAD_TYPE or OUTPAD_TYPE */
        blocks[num_blocks - 1].block_type = kblock_type;
    }

    int   pin_index = -1;
    char* ptr = my_strtok(NULL, TOKENS, fp_net, buf);
    while (ptr != NULL) {
        ++pin_index;
        if (pin_index >= 1) {
            printf("Error in add_io on line %d of netlist file.\n", linenum);
            printf("Too many pins on this io.  Expected 1.\n");
            exit(1);
        }

        if (strcmp(ptr, "open") == 0) {    /* Pin unconnected. */
            printf("Error in add_io, line %d of netlist file.\n", linenum);
            printf("Inputs and Outputs cannot have open pins.\n");
            exit(1);
        }

        /* Note the dummy pin number for IO_TYPE pins.  Change this if necessary. I set *
         * them to OPEN because I want the code to crash if I try to look up the   *
         * class of an I/O pin (since I/O pins don't have classes).                */
        inet = add_net(ptr,
                       pin_type,
                       num_blocks - 1,
                       OPEN,
                       doall);
        if (doall) { /* Loading pass only */
            blocks[num_blocks - 1].nets[pin_index] = inet;
        }

        ptr = my_strtok(NULL, TOKENS, fp_net, buf);
    }

    if (pin_index != 0) {
        printf("Error in add_io on line %d of netlist file.\n", linenum);
        printf("Expected 1 pin on pad, got %d.\n", pin_index + 1);
        exit(1);
    }

    if (doall) {
        for (i = 1; i < max_pins_per_clb; i++) {
            blocks[num_blocks - 1].nets[i] = OPEN;
        }
    }
}


/* This routine does the first part of the parsing of a blocks.  It is *
 * called whenever any type of blocks (.clb, .input or .output) is to  *
 * be parsed.  It increments the blocks count (num_blocks), and checks *
 * that the blocks has a name.  If doall is 1, this is the loading     *
 * pass and it copies the name to the blocks data structure.  Finally  *
 * it checks that the pinlist: keyword exists.  On return, my_strtok  *
 * is set so that the next call will get the first net connected to   *
 * this blocks.                                                        */
static void parse_name_and_pinlist(int doall, FILE* fp_net, char* buf)
{
    /* Get blocks name. */
    char* ptr = my_strtok(NULL, TOKENS, fp_net, buf);
    if (ptr == NULL) {
        printf("Error in parse_name_and_pinlist on line %d of netlist file.\n",
               linenum);
        printf(".clb, .input or .output line has no associated name.\n");
        exit(1);
    }

    /* first get clb name */
    int len = 0;
    if (doall == 1) {    /* Second (loading) pass, store blocks name */
        len = strlen(ptr);
        blocks[num_blocks - 1].name = (char*)my_chunk_malloc((len + 1) * sizeof(char),
                                                            NULL,
                                                            &chunk_bytes_avail,
                                                            &chunk_next_avail_mem);
        strcpy(blocks[num_blocks - 1].name, ptr);
    }

    ptr = my_strtok(NULL, TOKENS, fp_net, buf);
    if (ptr != NULL) {
        printf("Error in parse_name_and_pinlist on line %d of netlist file.\n",
               linenum);
        printf("Extra characters at end of line.\n");
        exit(1);
    }

    /* Now get pinlist from the next line.  Note that a NULL return value *
     * from my_gets means EOF, while a NULL return from my_strtok just    *
     * means we had a blank or comment line.                              */
    do {
        ptr = my_fgets(buf, BUFSIZE, fp_net);
        if (ptr == NULL) {
            printf("Error in parse_name_and_pinlist on line %d of netlist file.\n",
                   linenum);
            printf("Missing pinlist: keyword.\n");
            exit(1);
        }

        ptr = my_strtok(buf, TOKENS, fp_net, buf);
    } while (ptr == NULL);

    if (strcmp(ptr, "pinlist:") != 0) {
        printf("Error in parse_name_and_pinlist on line %d of netlist file.\n",
               linenum);
        printf("Expected pinlist: keyword, got %s.\n", ptr);
        exit(1);
    }
}


static void add_global(int doall, FILE* fp_net, char* buf)
{
    /* Doall is 0 for the first (counting) pass and 1 for the second           *
     * (loading) pass.  fp_net is a pointer to the netlist file.  This         *
     * routine sets the proper entry(ies) in is_global to TRUE during the      *
     * loading pass.  The routine does nothing during the counting pass. If    *
     * is_global = TRUE for a net, it will not be considered in the placement  *
     * cost function, nor will it be routed.  This is useful for global        *
     * signals like clocks that generally have dedicated routing in FPGAs.     */
    hash_t* h_ptr;

    /* Do nothing if this is the counting pass. */
    if (doall == 0) {
        return;
    }

    char* ptr = my_strtok(NULL, TOKENS, fp_net, buf);
    int nindex;
    while (ptr != NULL) {     /* For each .global signal */
        num_globals++;
        h_ptr = get_hash_entry(hash_table, ptr);

        if (h_ptr == NULL) {        /* Net was not found in list! */
            printf("Error in add_global on netlist file line %d.\n", linenum);
            printf("Global signal %s does not exist.\n", ptr);
            exit(1);
        }

        nindex = h_ptr->index;
        is_global[nindex] = TRUE;    /* Flagged as global net */
        ptr = my_strtok(NULL, TOKENS, fp_net, buf);
    }
}

/* block_num is blk_num, blk_pnum is blk_pin_num */
static int add_net(char* ptr,
                   pin_types_t type,
                   int block_num,
                   int block_pin_num,
                   int doall)
{
    /* This routine is given a net name in *ptr, either DRIVER or RECEIVER *
     * specifying whether the blocks number given by block_num is driving this   *
     * net or in the fan-out and doall, which is 0 for the counting pass   *
     * and 1 for the loading pass.  It updates the net data structure and  *
     * returns the net number so the calling routine can update the blocks  *
     * data structure.                                                     */
    hash_t* h_ptr;
    int j, net_index;

    if (doall == 0) { /* Counting pass only */
        h_ptr = insert_in_hash_table(hash_table,
                                     ptr,
                                     num_nets);
        net_index = h_ptr->index;
        if (net_index == num_nets) {  /* Net was not in the hash table */
            ++num_nets;
        }
    } else { /* Load pass, this net had existed!! */
        h_ptr = get_hash_entry(hash_table,
                               ptr);
        if (h_ptr == NULL) {
            printf("Error in add_net:  the second (load) pass found could not\n");
            printf("find net %s in the symbol table.\n", ptr);
            exit(1);
        }

        net_index = h_ptr->index;
        ++(net[net_index].num_net_pins);

        if (type == DRIVER) {
            ++num_driver[net_index];
            j = 0; /* Driver always in position 0 of pinlist */
        } else {
            j = net[net_index].num_net_pins - num_driver[net_index];
            /* num_driver is the number of signal drivers of this net. *
             * It should always be zero or 1 unless the netlist is bad.*/
            if (j >= temp_num_pins[net_index]) {
                printf("Error:  Net #%d (%s) has no driver and will cause\n",
                       net_index, ptr);
                printf("memory corruption.\n");
                exit(1);
            }
        }

        net[net_index].node_blocks[j] = block_num;
        net[net_index].node_block_pins[j] = block_pin_num;
    }  /* end of else (doall != 0) */
    return net_index;
}  /* end of static int add_net() */


static void free_parse(void)
{
    /* Release memory needed only during circuit netlist parsing. */
    free(num_driver);
    free_hash_table(hash_table);
    free(temp_num_pins);
}
