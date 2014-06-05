#include <stdio.h>
#include <string.h>
#include "util.h"
#include "vpr_types.h"
#include "globals.h"
#include "print_netlist.h"


/******************** Subroutines local to this module ***********************/
static void print_pinnum(FILE* fp, int pinnum);


/********************* Subroutine definitions ********************************/
void print_netlist(char* foutput, char* net_file, subblock_data_t subblock_data)
{
    /* Prints out the netlist related data structures into the file    *
     * fname.                                                          */
    int i, j, ipin, max_pin;
    FILE* fp = my_fopen(foutput, "w");
    fprintf(fp, "Input netlist file: %s\n", net_file);
    fprintf(fp, "num_primary_inputs: %d, num_primary_outputs: %d, num_clbs: %d\n",
            num_primary_inputs, num_primary_outputs, num_clbs);
    fprintf(fp, "num_blocks: %d, num_nets: %d, num_globals: %d\n",
            num_blocks, num_nets, num_globals);
    fprintf(fp, "\nNet\tName\t\t#Pins\tDriver\t\tRecvs. (blocks, pin)\n");

    for (i = 0; i < num_nets; ++i) {
        fprintf(fp, "\n%d\t%s\t", i, net[i].name);

        if (strlen(net[i].name) < 8) {
            fprintf(fp, "\t");    /* Name field is 16 chars wide */
        }

        fprintf(fp, "%d", net[i].num_net_pins);

        const int knum_net_pins = net[i].num_net_pins;
        for (j = 0; j < knum_net_pins; ++j) {
            fprintf(fp, "\t(%4d,%4d)", net[i].node_blocks[j],
                    net[i].node_block_pins[j]);
        }
    }

    fprintf(fp, "\n\n\nBlock List:\t\tBlock Type Legend:\n");
    fprintf(fp, "\t\t\tINPAD = %d\tOUTPAD = %d\n", INPAD_TYPE, OUTPAD_TYPE);
    fprintf(fp, "\t\t\tCLB = %d\n\n", B_CLB_TYPE);
    fprintf(fp, "\nBlock\tName\t\tType\tPin Connections\n\n");

    for (i = 0; i < num_blocks; i++) {
        fprintf(fp, "\n%d\t%s\t", i, blocks[i].name);

        if (strlen(blocks[i].name) < 8) {
            fprintf(fp, "\t");    /* Name field is 16 chars wide */
        }

        fprintf(fp, "%d", blocks[i].block_type);

        if (blocks[i].block_type == INPAD_TYPE || blocks[i].block_type == OUTPAD_TYPE) {
            max_pin = 1;
        } else {
            max_pin = pins_per_clb;
        }

        for (j = 0; j < max_pin; j++) {
            print_pinnum(fp, blocks[i].nets[j]);
        }
    }

    fprintf(fp, "\n");
    /* Now print out subblock info. */
    int subblock_lut_size = subblock_data.subblock_lut_size;
    subblock_t** subblock_inf = subblock_data.subblock_inf;
    int*  num_subblocks_per_block = subblock_data.num_subblocks_per_block;
    fprintf(fp, "\n\nSubblock List:\n\n");

    for (i = 0; i < num_blocks; i++) {
        if (blocks[i].block_type != B_CLB_TYPE) {
            continue;
        }

        fprintf(fp, "\nBlock: %d (%s)\tNum_subblocks: %d\n", i,
                blocks[i].name, num_subblocks_per_block[i]);
        /* Print header. */
        fprintf(fp, "Index\tName\t\tInputs");

        for (j = 0; j < subblock_lut_size; j++) {
            fprintf(fp, "\t");
        }

        fprintf(fp, "Output\tClock\n");

        /* Print subblock info for blocks i. */

        for (j = 0; j < num_subblocks_per_block[i]; j++) {
            fprintf(fp, "%d\t%s", j, subblock_inf[i][j].name);

            if (strlen(subblock_inf[i][j].name) < 8) {
                fprintf(fp, "\t");    /* Name field is 16 characters */
            }

            for (ipin = 0; ipin < subblock_lut_size; ipin++) {
                print_pinnum(fp, subblock_inf[i][j].inputs[ipin]);
            }

            print_pinnum(fp, subblock_inf[i][j].output);
            print_pinnum(fp, subblock_inf[i][j].clock);
            fprintf(fp, "\n");
        }
    }

    fclose(fp);
}


static void print_pinnum(FILE* fp, int pinnum)
{
    /* This routine prints out either OPEN or the pin number, to file fp. */
    if (pinnum == OPEN) {
        fprintf(fp, "\tOPEN");
    } else {
        fprintf(fp, "\t%d", pinnum);
    }
}
