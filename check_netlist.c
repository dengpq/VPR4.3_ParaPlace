#include <stdio.h>
#include "util.h"
#include "vpr_types.h"
#include "globals.h"
#include "hash.h"
#include "vpr_utils.h"
#include "check_netlist.h"


/**************** Subroutines local to this module **************************/

static int check_connections_to_global_clb_pins(int inet);

static int check_clb_conn(int iblk, int num_connections);

static void check_for_multiple_sink_connections(void);

static int check_for_duplicate_block_names(void);

static int get_num_conn(int block_num);

static int check_subblocks(int iblk, subblock_data_t* subblock_data_ptr,
                           int* num_uses_of_clb_pin, int* num_uses_of_sblk_opin);

static int check_subblock_pin(int clb_pin, int min_val, int max_val,
                              pin_types_t pin_type, int iblk, int isubblk, subblock_t
                              *subblock_inf);

static int check_internal_subblock_connections(subblock_data_t
                                               *subblock_data_ptr, int iblk, int* num_uses_of_sblk_opin);

static int check_clb_to_subblock_connections(int iblk, subblock_t
                                             *subblock_inf, int num_subblocks, int* num_uses_of_clb_pin);



/*********************** Subroutine definitions *****************************/
void check_netlist(subblock_data_t* subblock_data_ptr, int* num_driver)
{
    /* This routine checks that the netlist makes sense, and sets the num_ff    *
     * and num_const_gen members of subblock_data.                              */
    int* num_subblocks_per_block = subblock_data_ptr->num_subblocks_per_block;
    subblock_data_ptr->num_ff = 0;
    subblock_data_ptr->num_const_gen = 0;
    int  error = 0;
    int* num_uses_of_clb_pin = (int*)my_malloc(pins_per_clb * sizeof(int));
    int* num_uses_of_sblk_opin = (int*)my_malloc(
                    subblock_data_ptr->max_subblocks_per_block * sizeof(int));

    /* Check that nets fanout and have a driver. */
    int net_index = -1;
    for (net_index = 0; net_index < num_nets; ++net_index) {
        if (num_driver[net_index] != 1) {
            printf("Error:  net %s has %d signals driving it.\n",
                   net[net_index].name,
                   num_driver[net_index]);
            ++error;
        }

        if ((net[net_index].num_pins - num_driver[net_index]) < 1) {
            printf("Error:  net %s has no fanout.\n",
                   net[net_index].name);
            ++error;
        }
        error += check_connections_to_global_clb_pins(net_index);
    }

    /* Check that each blocks makes sense. */
    int num_connections = 0;
    int block_index = -1;
    for (block_index = 0; block_index < num_blocks; ++block_index) {
        num_connections = get_num_conn(block_index);

        if (blocks[block_index].type == CLB_TYPE) {
            error += check_clb_conn(block_index,
                                    num_connections);
            error += check_subblocks(block_index,
                                     subblock_data_ptr,
                                     num_uses_of_clb_pin,
                                     num_uses_of_sblk_opin);
        } else {  /* IO_TYPE blocks */
        /* This error check is a redundant double check.*/
            if (num_connections != 1) {
                printf("Error:  io blocks #%d (%s) of type %d"
                       "has %d pins.\n", block_index, blocks[block_index].name, blocks[block_index].type,
                       num_connections);
                error++;
            }

            /* IO_TYPE blocks must have no subblock information. */
            if (num_subblocks_per_block[block_index] != 0) {
                printf("Error:  IO_TYPE blocks #%d (%s) contains %d subblocks.\n"
                       "Expected 0.\n", block_index, blocks[block_index].name, num_subblocks_per_block[block_index]);
                error++;
            }
        }
    }

    check_for_multiple_sink_connections();
    error += check_for_duplicate_block_names();
    free(num_uses_of_clb_pin);
    free(num_uses_of_sblk_opin);

    if (error != 0) {
        printf("Found %d fatal Errors in the input netlist.\n", error);
        exit(1);
    }
} /* end of check_netlist() */

/* Checks that a global net(inet) connects only to global CLB_TYPE input pins  *
 * and that non-global nets never connects to a global CLB_TYPE pin.  Either    *
 * global or non-global nets are allowed to connect to pads.               */
static int check_connections_to_global_clb_pins(int inet)
{
    int error = 0;
    /* For now global signals can be driven by an I/O pad or any CLB_TYPE output  *
     * although a CLB_TYPE output generates a warning. I could make a global CLB_TYPE  *
     * output pin type to allow people to make architectures that didn't have *
     * this warning.                                                          */
    int ipin = -1;
    int num_pins = net[inet].num_pins;
    for (ipin = 0; ipin < num_pins; ++ipin) {
        int iblk = net[inet].blocks[ipin];

        if (blocks[iblk].type == CLB_TYPE) {  /* I/O pads are exempt. */
            int blk_pin = net[inet].blk_pin[ipin];

            if (is_global_clb_pin[blk_pin] != is_global[inet]) {
                /* Allow a CLB_TYPE output pin to drive a global net (warning only). */
                if (ipin == 0 && is_global[inet]) {
                    printf("Warning in check_connections_to_global_clb_pins:\n"
                           "\tnet #%d (%s) is driven by CLB_TYPE output pin (#%d)\n"
                           "\ton blocks #%d (%s).\n", inet, net[inet].name,
                           blk_pin, iblk, blocks[iblk].name);
                } else {    /* Otherwise -> Error */
                    printf("Error in check_connections_to_global_clb_pins:\n"
                           "\tpin %d on net #%d (%s) connects to CLB_TYPE input pin (#%d)\n"
                           "\ton blocks #%d (%s).\n",
                           ipin,
                           inet,
                           net[inet].name,
                           blk_pin,
                           iblk,
                           blocks[iblk].name);
                    ++error;
                }

                if (is_global[inet]) {
                    printf("\tNet is global, but CLB_TYPE pin is not.\n\n");
                } else {
                    printf("\tCLB pin is global, but net is not.\n\n");
                }
            }
        } /* end of if(blocks[iblk].type == CLB_TYPE) */
    }   /* End for all pins */

    return error;
}


static int check_clb_conn(int iblk,
                          int num_connections)
{
    /* Checks that the connections into and out of the clb make sense.  */
    int iclass, ipin;
    int error = 0;

    if (num_connections < 2) {
        printf("Warning:  logic blocks #%d (%s) has only %d pin.\n",
               iblk, blocks[iblk].name, num_connections);

        /* Allow the case where we have only one OUTPUT pin connected to continue. *
         * This is used sometimes as a constant generator for a primary output,    *
         * but I will still warn the user. If the only pin connected is an input, *
         * abort.                                                                 */
        if (num_connections == 1) {
            for (ipin = 0; ipin < pins_per_clb; ipin++) {
                if (blocks[iblk].nets[ipin] != OPEN) {
                    iclass = clb_pin_class[ipin];

                    if (class_inf[iclass].type != DRIVER) {
                        ++error;
                    } else {
                        printf("\tPin is an output -- may be a constant generator.\n"
                               "\tNon-fatal, but check this.\n");
                    }
                    break;
                }
            }
        } else { /* num_connections == 0 */
            ++error;
        }
    }

    /* This case should already have been flagged as an error -- this is *
     * just a redundant double check.                                    */
    if (num_connections > pins_per_clb) {
        printf("Error:  logic blocks #%d with output %s has %d pins.\n",
               iblk, blocks[iblk].name, num_connections);
        ++error;
    }

    return error;
}


static int check_for_duplicate_block_names(void)
{
    /* Checks that all blocks have duplicate names.  Returns the number of     *
     * duplicate names.                                                        */
    int error, iblk;
    hash_t** block_hash_table, *h_ptr;
    struct s_hash_iterator hash_iterator;
    error = 0;
    block_hash_table = alloc_hash_table();

    for (iblk = 0; iblk < num_blocks; iblk++) {
        h_ptr = insert_in_hash_table(block_hash_table, blocks[iblk].name, iblk);
    }

    hash_iterator = start_hash_table_iterator();
    h_ptr = get_next_hash(block_hash_table, &hash_iterator);

    while (h_ptr != NULL) {
        if (h_ptr->count != 1) {
            printf("Error:  %d blocks are named %s.  Block names must be unique."
                   "\n", h_ptr->count, h_ptr->name);
            error++;
        }

        h_ptr = get_next_hash(block_hash_table, &hash_iterator);
    }

    free_hash_table(block_hash_table);
    return (error);
}


/* This routine checks the subblocks of iblk (which must be a CLB_TYPE). *
 * It returns the number of errors found.                           */
static int check_subblocks(int iblk,
                           subblock_data_t* subblock_data_ptr,
                           int*  num_uses_of_clb_pin,
                           int*  num_uses_of_sblk_opin)
{
    int error = 0;
    subblock_t* subblock_inf = subblock_data_ptr->subblock_inf[iblk];
    int num_subblocks = subblock_data_ptr->num_subblocks_per_block[iblk];
    int max_subblocks_per_block = subblock_data_ptr->max_subblocks_per_block;
    int subblock_lut_size = subblock_data_ptr->subblock_lut_size;

    if (num_subblocks < 1 || num_subblocks > max_subblocks_per_block) {
        printf("Error:  blocks #%d (%s) contains %d subblocks.\n",
               iblk, blocks[iblk].name, num_subblocks);
        ++error;
    }

    /* Check that all pins connect to the proper type of CLB_TYPE pin and are in the *
     * correct range.                                                           */
    int isub, ipin, i;
    for (isub = 0; isub < num_subblocks; ++isub) {
        for (ipin = 0; ipin < subblock_lut_size; ++ipin) { /* Input pins */
            int clb_input_pin = subblock_inf[isub].inputs[ipin];
            error += check_subblock_pin(clb_input_pin,
                                        0,
                                        pins_per_clb + num_subblocks - 1,
                                        RECEIVER,
                                        iblk,
                                        isub,
                                        subblock_inf);
        }

        /* Subblock output pin. */
        int clb_output_pin = subblock_inf[isub].output;
        error += check_subblock_pin(clb_output_pin,
                                    0,
                                    pins_per_clb - 1,
                                    DRIVER,
                                    iblk,
                                    isub,
                                    subblock_inf);
        /* Subblock clock pin. */
        int clb_clock_pin = subblock_inf[isub].clock;
        error += check_subblock_pin(clb_clock_pin,
                                    0,
                                    pins_per_clb + num_subblocks - 1,
                                    RECEIVER,
                                    iblk,
                                    isub,
                                    subblock_inf);
    }  /* End subblock for loop. */

    /* If pins out of range, return.  Could get seg faults otherwise. */
    if (error != 0) {
        return error;
    }

    /* Reset fanout counts. */
    for (i = 0; i < pins_per_clb; i++) {
        num_uses_of_clb_pin[i] = 0;
    }
    for (i = 0; i < num_subblocks; i++) {
        num_uses_of_sblk_opin[i] = 0;
    }

    load_one_clb_fanout_count(subblock_lut_size,
                              subblock_inf,
                              num_subblocks,
                              num_uses_of_clb_pin,
                              num_uses_of_sblk_opin,
                              iblk);
    error += check_clb_to_subblock_connections(iblk,
                                               subblock_inf,
                                               num_subblocks,
                                               num_uses_of_clb_pin);
    error += check_internal_subblock_connections(subblock_data_ptr,
                                                 iblk,
                                                 num_uses_of_sblk_opin);

    return error;
}

static int check_subblock_pin(int clb_pin, int min_val, int max_val,
                              pin_types_t pin_type, int iblk, int isubblk, subblock_t
                              *subblock_inf)
{
    /* Checks that this subblock pin connects to a valid clb pin or BLE output *
     * within the clb.  Returns the number of errors found.                    */
    int iclass;

    if (clb_pin != OPEN) {
        if (clb_pin < min_val || clb_pin > max_val) {
            printf("Error:  Block #%d (%s) subblock #%d (%s)"
                   "connects to nonexistent clb pin #%d.\n", iblk, blocks[iblk].name,
                   isubblk, subblock_inf[isubblk].name, clb_pin);
            return (1);
        }

        if (clb_pin < pins_per_clb) {  /* clb pin */
            iclass = clb_pin_class[clb_pin];

            if (class_inf[iclass].type != pin_type) {
                printf("Error:  Block #%d (%s) subblock #%d (%s) pin connects\n"
                       "\tto clb pin (#%d) of wrong input/output type.\n", iblk,
                       blocks[iblk].name, isubblk, subblock_inf[isubblk].name,
                       clb_pin);
                return (1);
            }
        }
    }

    return (0);
}


static void check_for_multiple_sink_connections(void)
{
    /* The check is for nets that connect more than once to the same class of  *
     * pins on the same blocks.  For LUTs and cluster-based logic blocks that   *
     * doesn't make sense, although for some logic blocks it does.  The router *
     * can now handle this case, so maybe I should get rid of this check.      */
    int iblk, ipin, inet, iclass, class_pin;
    int* num_pins_connected;
    num_pins_connected = my_calloc(num_nets, sizeof(int));

    /* Have to do the check blocks by blocks, rather than net by net, for speed. *
     * This makes the code a bit messy.                                        */

    for (iblk = 0; iblk < num_blocks; iblk++) {
        for (iclass = 0; iclass < num_pin_class; iclass++) {
            /* Two DRIVER pins can never connect to the same net (already checked by    *
             * the multiple driver check) so skip that check.                           */
            if (class_inf[iclass].type == DRIVER) {
                continue;
            }

            for (class_pin = 0; class_pin < class_inf[iclass].num_pins; class_pin++) {
                ipin = class_inf[iclass].pinlist[class_pin];
                inet = blocks[iblk].nets[ipin];

                if (inet != OPEN) {
                    num_pins_connected[inet]++;
                }
            }

            for (class_pin = 0; class_pin < class_inf[iclass].num_pins; class_pin++) {
                ipin = class_inf[iclass].pinlist[class_pin];
                inet = blocks[iblk].nets[ipin];

                if (inet != OPEN) {
                    if (num_pins_connected[inet] > 1) {
                        printf("Warning:  blocks %d (%s) connects %d pins of class "
                               "%d to net %d (%s).\n", iblk, blocks[iblk].name,
                               num_pins_connected[inet], iclass, inet, net[inet].name);
                        printf("\tThis does not make sense for many logic blocks "
                               "(e.g. LUTs).\n"
                               "\tBe sure you really want this.\n");
                    }

                    num_pins_connected[inet] = 0;
                }
            }
        }
    }

    free(num_pins_connected);
}


static int get_num_conn(int block_num)
{
    /* This routine returns the number of connections to a blocks. */
    int i = -1;
    int num_connections = 0;
    for (i = 0; i < pins_per_clb; i++) {
        if (blocks[block_num].nets[i] != OPEN) {
            ++num_connections;
        }
    }

    return num_connections;
}


static int check_internal_subblock_connections(subblock_data_t* subblock_data_ptr,
                                               int iblk,
                                               int* num_uses_of_sblk_opin)
{
    /* This routine checks that all subblocks in this blocks are either           *
     * completely empty (no pins hooked to anything) or have their output used   *
     * somewhere.  It also counts the number of constant generators (no input    *
     * sblks) and the number of FFs used in the circuit.                         */
    int num_const_gen, num_ff, isub, ipin, error;
    boolean has_inputs;
    int subblock_lut_size;
    int num_subblocks;
    subblock_t* subblock_inf;
    subblock_lut_size = subblock_data_ptr->subblock_lut_size;
    num_subblocks = subblock_data_ptr->num_subblocks_per_block[iblk];
    subblock_inf = subblock_data_ptr->subblock_inf[iblk];
    num_const_gen = 0;
    num_ff = 0;
    error = 0;

    for (isub = 0; isub < num_subblocks; isub++) {
        has_inputs = FALSE;

        for (ipin = 0; ipin < subblock_lut_size; ipin++) {
            if (subblock_inf[isub].inputs[ipin] != OPEN) {
                has_inputs = TRUE;
                break;
            }
        }

        if (num_uses_of_sblk_opin[isub] == 0) {   /* Output unused */
            if (has_inputs || subblock_inf[isub].clock != OPEN) {
                printf("Error:  output of subblock #%d (%s) of blocks #%d (%s) is "
                       "never used.\n", isub, subblock_inf[isub].name, iblk,
                       blocks[iblk].name);
                error++;
            }
        }     /* End if output unused */
        /* Check that subblocks whose output is used have inputs. */
        else {   /* Subblock output is used. */
            if (!has_inputs) {   /* No inputs are used */
                if (subblock_inf[isub].clock == OPEN) {
                    printf("Warning:  blocks #%d (%s), subblock #%d (%s) is a "
                           "constant generator.\n\t(Has no inputs.)\n", iblk,
                           blocks[iblk].name, isub, subblock_inf[isub].name);
                    num_const_gen++;
                } else {
                    printf("Error:  blocks #%d (%s), subblock #%d (%s) is a CLOCKED "
                           "\n\tconstant generator.\n\t(Has no inputs but is clocked.)\n",
                           iblk, blocks[iblk].name, isub, subblock_inf[isub].name);
                    num_const_gen++;
                    error++;
                }
            } else { /* Both input and output are used */
                if (subblock_inf[isub].clock != OPEN) {
                    num_ff++;
                }
            }
        }
    }   /* End for all subblocks */

    subblock_data_ptr->num_const_gen += num_const_gen;
    subblock_data_ptr->num_ff += num_ff;
    return (error);
}


static int check_clb_to_subblock_connections(int iblk, subblock_t
                                             *subblock_inf, int num_subblocks, int* num_uses_of_clb_pin)
{
    /* This routine checks that each non-OPEN clb input pin connects to some    *
     * subblock inputs, and that each non-OPEN clb output pin is driven by      *
     * exactly one subblock output. It returns the number of errors found.      *
     * Note that num_uses_of_clb_pin is used to store the number of out-edges   *
     * (fanout) for a CLB_TYPE ipin, and the number of in-edges (fanin) for a CLB_TYPE    *
     * opin.                                                                    */
    int ipin, isub, clb_pin, error;
    error = 0;

    /* Count how many things connect to each clb output pin. */

    for (isub = 0; isub < num_subblocks; isub++) {
        clb_pin = subblock_inf[isub].output;

        if (clb_pin != OPEN) { /* Guaranteed to connect to DRIVER pin only */
            num_uses_of_clb_pin[clb_pin]++;
        }
    }

    for (ipin = 0; ipin < pins_per_clb; ipin++) {
        if (blocks[iblk].nets[ipin] != OPEN) {
            if (is_opin(ipin)) {   /* CLB_TYPE output */
                if (num_uses_of_clb_pin[ipin] == 0) {  /* No driver? */
                    printf("Error:  output pin %d on blocks #%d (%s) is not driven "
                           "by any subblock.\n", ipin, iblk, blocks[iblk].name);
                    error++;
                } else if (num_uses_of_clb_pin[ipin] > 1) { /* Multiple drivers? */
                    printf("Error:  output pin %d on blocks #%d (%s) is driven "
                           "by %d subblocks.\n", ipin, iblk, blocks[iblk].name,
                           num_uses_of_clb_pin[ipin]);
                    error++;
                }
            } else { /* CLB_TYPE ipin */
                if (num_uses_of_clb_pin[ipin] <= 0) {    /* Fans out? */
                    printf("Error:  pin %d on blocks #%d (%s) does not fanout to any "
                           "subblocks.\n", ipin, iblk, blocks[iblk].name);
                    error++;
                }
            }
        }   /* End if not OPEN */
        else if (is_opin(ipin)) {   /* OPEN CLB_TYPE output pin */
            if (num_uses_of_clb_pin[ipin] > 1) {
                printf("Error:  pin %d on blocks #%d (%s) is driven by %d "
                       "subblocks.\n", ipin, iblk, blocks[iblk].name,
                       num_uses_of_clb_pin[ipin]);
                error++;
            }
        }
    }

    return (error);
}
